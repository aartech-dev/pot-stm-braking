#ifndef BRAKE_MODULE_H
#define BRAKE_MODULE_H

/* ============================================================
 *  AART Slot Car Braking Module  — Rev 3
 *  Target  : STM32G041K6U6 (QFN-32)
 *  Library : libopencm3
 *
 *  Pin assignments:
 *    PA0  ADC_IN0    Rail sense      (12.4V ÷ divider → ≤3.3V)
 *    PA1  GPIO IN    Wiper strip     (active LOW, pull-up)
 *    PA2  GPIO IN    Toggle bit 0    (active LOW, pull-up)
 *    PA3  ADC_IN3    Pot wiper       (10k, 0–3.3V)
 *    PA4  GPIO IN    Toggle bit 1    (active LOW, pull-up)
 *    PA5  ADC_IN5    Capture button  (10k pull-up; button → GND via 1k)
 *    PA6  TIM3_CH1   Brake PWM       → GD1 → Q1 N-ch low-side (INVERTED)
 *    PA7  TIM3_CH2   Keep-alive PWM  → GD2 → Q2 P-ch high-side (normal)
 *    PB6  GPIO OUT   Status LED      (blinks 3× on flash save)
 *
 *  Toggle decoding (PA2=bit0, PA4=bit1 — LOW = asserted):
 *    PA2=H PA4=H → MODE_SEL_A  brushed + antibrake
 *    PA2=L PA4=H → MODE_SEL_B  brushed, brake only
 *    PA2=H PA4=L → MODE_SEL_C  brushless + eCom keep-alive
 *    PA2=L PA4=L → invalid → treated as MODE_SEL_B (safe)
 *
 *  ── Pot mapping ──────────────────────────────────────────
 *
 *  Mode A (brushed + antibrake):
 *    CCW (0)        → ~8Ω brake       (BRAKE_CCR_SOFT)
 *    approaching ctr→ dead short       (BRAKE_CCR_HARD)
 *    centre ±dead   → dead short       (max braking, no freewheel)
 *    past centre CW → antibrake 0→3V  (Q2 injects, Q1 off)
 *    full CW        → ~3V antibrake
 *
 *  Mode B (brushed, no antibrake):
 *    CCW (0)        → ~8Ω brake
 *    full CW        → dead short
 *    No injection ever.
 *
 *  Mode C (brushless keep-alive):
 *    Q2 keep-alive ALWAYS on track at saved CCR value.
 *    CCW (0)        → ~8Ω brake  (Q1 PWM, Q2 keep-alive in parallel)
 *    full CW        → dead short (Q1 on,   Q2 keep-alive in parallel)
 *    Wiper open     → Q1 off,    Q2 keep-alive only
 *    Capture button → Q1 off,    Q2 pot-dialled; release saves to flash
 *
 *  HARDWARE NOTE: In mode C a Schottky diode in series with Q2's
 *  output is strongly recommended. When Q1 is on (dead short) and
 *  Q2 is simultaneously injecting keep-alive, the low-impedance
 *  shorted track pulls Q2's output low. The diode blocks reverse
 *  current through Q2 during the brake pulse.
 *
 *  ── Keep-alive capture ───────────────────────────────────
 *    Hold button → pot dials Q2 output 0→3V onto track
 *    Release     → current CCR written to flash page 31
 *    LED blinks 3× to confirm save
 * ============================================================ */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <stdint.h>
#include <stdbool.h>

/* ── ADC ─────────────────────────────────────────────────── */
#define ADC_MAX                4095U
#define ADC_CENTRE             2048U

/* Mode A: deadband around centre stays at dead short
 * (no freewheel — centre = maximum braking)               */
#define ADC_DEADBAND           150U

/* Rail sense: 4.7k/(15k+4.7k) → V_PA0 = 2.96V at 12.4V  */
#define ADC_RAIL_DIVIDER_NUM   47U
#define ADC_RAIL_DIVIDER_DEN   197U
#define RAIL_UNDERVOLTAGE_MV   5000U

/* Capture button thresholds on PA5
 * Unpressed: ~4095  Pressed: ~372  Threshold midpoint      */
#define BTN_PRESSED_THRESHOLD  2200U

/* ── PWM ─────────────────────────────────────────────────── */
/* TIM3 @ 20kHz: 64MHz / 1 / 3200 = 20kHz                  */
#define PWM_ARR                3199U

/* ── Q1 Brake FET (N-ch, low-side, TIM3_CH1 INVERTED) ─────
 *
 * INVERTED polarity means:
 *   CCR = 0        → pin permanently HIGH → FET fully ON  → dead short
 *   CCR = PWM_ARR  → pin permanently LOW  → FET fully OFF → no brake
 *   CCR = mid      → PWM → effective resistance between short and open
 *
 * Pot maps:
 *   pot CCW (0)    → BRAKE_CCR_SOFT  (~8Ω,  FET mostly off)
 *   pot CW  (4095) → BRAKE_CCR_HARD  (0,    FET fully on = dead short)
 *
 * At power-off/reset: 10k pull-up on GD1 input holds gate HIGH
 * → FET ON → dead short = safe default (motor braked).       */
#define BRAKE_CCR_HARD         0U        /* dead short */
#define BRAKE_CCR_SOFT         2720U     /* ~8Ω effective */
#define BRAKE_CCR_OFF          PWM_ARR   /* FET off */

/* ── Q2 Keep-alive / Antibrake FET (P-ch, high-side, TIM3_CH2 normal) ─
 *
 * Normal polarity:
 *   CCR = 0         → pin LOW → P-ch FET OFF → no injection
 *   CCR = KA_MAX    → PWM at ~3V effective duty
 *
 * ~3V: 3/12.4 × 3200 ≈ 774 counts                          */
#define KA_CCR_MAX             774U
#define KA_CCR_OFF             0U

/* ── Flash NV storage ────────────────────────────────────── */
/* STM32G041K6: 32 pages × 1K. Page 31 = NV keep-alive CCR.
 * Linker script uses LENGTH=31K — compiler cannot touch page 31. */
#define FLASH_SAVE_PAGE        31U
#define FLASH_SAVE_ADDR        (0x08000000U + (FLASH_SAVE_PAGE * 1024U))
#define FLASH_MAGIC            0xAA270001U   /* AA=0xAA, 27=AART initials, 0001=rev */

/* ── Timing ──────────────────────────────────────────────── */
#define WIPER_DEBOUNCE_MS      10U
#define BTN_DEBOUNCE_MS        20U
#define LED_BLINK_MS           80U
#define LED_BLINK_COUNT        3U

/* ── Pin aliases ─────────────────────────────────────────── */
#define WIPER_PORT             GPIOA
#define WIPER_PIN              GPIO1
#define TOGGLE0_PORT           GPIOA
#define TOGGLE0_PIN            GPIO2
#define TOGGLE1_PORT           GPIOA
#define TOGGLE1_PIN            GPIO4
#define PWM_PORT               GPIOA
#define PWM_PIN_BRAKE          GPIO6
#define PWM_PIN_KA             GPIO7
#define LED_PORT               GPIOB
#define LED_PIN                GPIO6

/* ── DMA ─────────────────────────────────────────────────── */
#define ADC_DMA_CHANNEL        DMA_CHANNEL1
#define ADC_DMAMUX_REQ         5U
#define ADC_NUM_CHANNELS       3U   /* CH0=rail, CH3=pot, CH5=button */

/* ── Types ───────────────────────────────────────────────── */
typedef enum {
    MODE_SEL_A = 0,   /* brushed + antibrake    */
    MODE_SEL_B,       /* brushed, brake only    */
    MODE_SEL_C,       /* brushless keep-alive   */
} ModeSelect_t;

typedef enum {
    STATE_PASSTHROUGH = 0,
    STATE_BRAKING,
    STATE_ANTIBRAKE,          /* mode A, CW of centre        */
    STATE_KEEPALIVE_ONLY,     /* mode C, wiper open          */
    STATE_CAPTURE,            /* mode C, button held         */
    STATE_SAFE_BRAKE,
} OpState_t;

typedef struct {
    ModeSelect_t  mode_sel;
    OpState_t     state;
    uint16_t      pot_raw;
    uint16_t      rail_mv;
    uint16_t      btn_raw;
    uint16_t      brake_ccr;
    uint16_t      ka_ccr;
    uint16_t      saved_ka_ccr;
    bool          wiper_active;
    bool          btn_pressed;
    bool          capture_active;
} BrakeCtx_t;

/* ── Public API ──────────────────────────────────────────── */
void         brake_init(void);
void         brake_tick(void);
OpState_t    brake_get_state(void);
ModeSelect_t brake_get_mode_sel(void);
void         brake_force_safe(void);
void         brake_systick_isr(void);

#endif /* BRAKE_MODULE_H */
