#ifndef BRAKE_MODULE_H
#define BRAKE_MODULE_H

/* ============================================================
 *  AART Slot Car Braking Module  — Rev 5
 *  Target  : STM32G041K6U6 (QFN-32)
 *  Library : libopencm3
 *
 *  Physical connections (6× 4mm banana sockets):
 *    WHITE  +12–16V always-live rail from track PSU (40–90A source)
 *    BLACK  Track positive (+ve to car via right braid)
 *    RED    Common negative (left braid)
 *    Three mirrored sockets on output side for hand controller
 *
 *  Pin assignments:
 *    PA0  ADC_IN0   WHITE rail sense    (÷ R8/R9  18k/4k7 → 3.31V at 16V)
 *    PA1  ADC_IN1   BLACK track sense   (÷ R10/R11 22k/4k7 → ≤3.3V at 16V)
 *    PA2  GPIO IN   Toggle bit 0        (active LOW, pull-up)
 *    PA3  ADC_IN3   Pot wiper           (10k centre-detent, 0–3.3V)
 *    PA4  GPIO IN   Toggle bit 1        (active LOW, pull-up)
 *    PA5  ADC_IN5   Capture button      (10k pull-up; button→GND via 1k)
 *    PA6  TIM3_CH1  Brake PWM           → GD1 → Q1 N-ch low-side (INVERTED)
 *    PA7  TIM3_CH2  Keep-alive PWM      → GD2 → Q2 P-ch high-side (normal)
 *    PB6  GPIO OUT  Status LED          (blinks on flash save)
 *
 *  Toggle decoding (PA2=bit0, PA4=bit1 — LOW=asserted):
 *    PA2=H PA4=H → MODE_A  Brushed motor, positive anti-brake
 *    PA2=L PA4=H → MODE_B  Brushed motor, reduced braking only
 *    PA2=H PA4=L → MODE_C  Brushless motor, eCom keep-alive
 *    PA2=L PA4=L → invalid → defaults to MODE_B (safe)
 *
 *  ── Brake detection ──────────────────────────────────────
 *  Voltage sense on BLACK terminal with hysteresis:
 *    V_BLACK < BRAKE_ENTER_MV (500mV)  → enter brake state
 *    V_BLACK > BRAKE_EXIT_MV  (1500mV) → exit brake state
 *    Between thresholds → hold current state (no chatter)
 *
 *  ── Pot mapping ──────────────────────────────────────────
 *  Mode A (brushed + positive anti-brake):
 *    CCW → centre:    Reduced braking 8Ω → dead short
 *    centre ±dead:    Dead short (maximum braking)
 *    centre → CW:     Positive anti-brake 0V → 2V
 *
 *  Mode B (brushed, reduced braking only):
 *    CCW → CW:        8Ω → dead short (full range)
 *
 *  Mode C (brushless, eCom keep-alive):
 *    CCW → CW:        8Ω → dead short
 *    Q2 always on at saved keep-alive CCR when V_BLACK ≤ KA setpoint
 *
 *  ── Brake ramp-in (modes A and B only) ──────────────────
 *  On brake entry, CCR ramps exponentially from BRAKE_CCR_OFF
 *  to target over a saved ramp time (0–200ms).
 *  Shape: CCR += (target - CCR) * alpha / 256  each 1ms tick
 *  alpha is derived from ramp_ms: shorter time = higher alpha.
 *  At ramp_ms=0 braking is immediate (alpha=256, one step).
 *
 *  ── Capture button (dual function) ──────────────────────
 *  Mode C: hold → pot dials keep-alive voltage; release → save
 *  Mode A/B: hold → pot dials ramp time (CCW=0ms, CW=200ms);
 *            release → save
 *  Both save to flash page 31. LED blinks 3× on save.
 *
 *  ── Keep-alive suppression (mode C) ─────────────────────
 *  Q2 suppressed when V_BLACK > KA setpoint voltage.
 *  Prevents Q2 fighting controller output during throttle.
 *
 *  ── Component selection notes ───────────────────────────
 *  D1  P6KE18CA (bidirectional TVS) — not P6KE18A. Both
 *      polarity transients must be clamped.
 *  D2/D3  SS310 (3A, 100V) — not SS34 (40V). Inductive
 *      turn-off spikes require 100V reverse rating.
 *  D4  MBRS360 or SS54 — not SS34. Sees repetitive 46A
 *      pulses at 20kHz in mode C. Most stressed diode.
 *  F1  500mA-1A polyfuse in WHITE rail only. Never in
 *      BLACK path — motor startup is 40A.
 *  R8  18kΩ (not 15kΩ) — 15k gives 3.82V at 16V rail,
 *      exceeding the 3.3V ADC reference.
 *
 *  ── BSCRA/ISRA legality note ─────────────────────────────
 *  Anti-brake voltage is sourced directly from the WHITE rail
 *  (track PSU). No stored energy (capacitors, batteries) is
 *  used. This satisfies the direct track power requirement
 *  in current BSCRA and ISRA controller regulations.
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

/* ── Voltage dividers ────────────────────────────────────── */
/* WHITE sense (PA0): R8=18k, R9=4k7 → ratio 4.7/22.7=0.2070
 * V_PA0 = V_WHITE × 0.2070 → 3.31V at 16V rail              */
#define WHITE_DIV_NUM          47U
#define WHITE_DIV_DEN          227U

/* BLACK sense (PA1): R10=22k, R11=4k7 → ratio 4.7/26.7=0.1760
 * V_PA1 = V_BLACK × 0.1760 → 2.82V at 16V rail              */
#define BLACK_DIV_NUM          47U
#define BLACK_DIV_DEN          267U

/* ── Brake hysteresis thresholds (mV on BLACK) ───────────── */
#define BRAKE_ENTER_MV         500U
#define BRAKE_EXIT_MV          1500U
#define RAIL_UNDERVOLTAGE_MV   5000U

/* ── ADC ─────────────────────────────────────────────────── */
#define ADC_MAX                4095U
#define ADC_CENTRE             2048U
#define ADC_DEADBAND           150U
#define BTN_PRESSED_THRESHOLD  2200U

/* ── PWM ─────────────────────────────────────────────────── */
/* TIM3 @ 20kHz: 64MHz PSC=0 ARR=3199                        */
#define PWM_ARR                3199U

/* Q1 brake (N-ch low-side, TIM3_CH1 INVERTED):
 *   CCR=0       → pin HIGH → FET ON  → dead short
 *   CCR=PWM_ARR → pin LOW  → FET OFF
 * Safe default: 10k pull-up on GD1 input → brake on at power-off */
#define BRAKE_CCR_HARD         0U
#define BRAKE_CCR_SOFT         2720U    /* ~8Ω effective */
#define BRAKE_CCR_OFF          PWM_ARR

/* Q2 keep-alive / anti-brake (P-ch high-side, TIM3_CH2 normal):
 *   CCR=0       → FET off
 *   CCR=KA_MAX  → ~2V effective (2/16 × 3200 = 400 at 16V)  */
#define KA_CCR_MAX             400U
#define KA_CCR_OFF             0U

/* ── Brake ramp-in ───────────────────────────────────────── */
/* Exponential ramp: each 1ms tick applies
 *   ccr += (target - ccr) * alpha / 256
 * alpha=256 → immediate (one step); alpha=3 → ~200ms to 95%
 * Pot maps CCW=0ms(alpha=256) to CW=200ms(alpha=3)
 * RAMP_MS_MAX: maximum ramp time settable via pot             */
#define RAMP_MS_MAX            200U
#define RAMP_ALPHA_INSTANT     256U     /* 0ms — immediate     */
#define RAMP_ALPHA_MIN         3U       /* 200ms               */

/* Maps ramp_ms (0–200) to alpha (256–3) linearly             */
#define RAMP_MS_TO_ALPHA(ms)   ((ms) == 0U ? RAMP_ALPHA_INSTANT \
    : (uint16_t)(RAMP_ALPHA_INSTANT - \
      ((uint32_t)(RAMP_ALPHA_INSTANT - RAMP_ALPHA_MIN) * (ms)) \
      / RAMP_MS_MAX))

/* ── Flash NV storage ────────────────────────────────────── */
/* Page 31 (0x08007C00), 1K — reserved by linker script.
 * 64-bit record layout:
 *   [63:48] ramp_ms   (uint16, 0–200)
 *   [47:32] ka_ccr    (uint16, 0–KA_CCR_MAX)
 *   [31:0]  magic     (0xAA270001)                           */
#define FLASH_SAVE_PAGE        31U
#define FLASH_SAVE_ADDR        (0x08000000U + (FLASH_SAVE_PAGE * 1024U))
#define FLASH_MAGIC            0xAA270001U

/* ── Timing ──────────────────────────────────────────────── */
#define BTN_DEBOUNCE_MS        20U
#define LED_BLINK_MS           80U
#define LED_BLINK_COUNT        3U

/* ── Pin aliases ─────────────────────────────────────────── */
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
/* ADC scan: CH0(WHITE) → CH1(BLACK) → CH3(pot) → CH5(btn)  */
#define ADC_DMA_CHANNEL        DMA_CHANNEL1
#define ADC_DMAMUX_REQ         5U
#define ADC_NUM_CHANNELS       4U

/* ── Types ───────────────────────────────────────────────── */
typedef enum {
    MODE_A = 0,   /* brushed motor, positive anti-brake       */
    MODE_B,       /* brushed motor, reduced braking only      */
    MODE_C,       /* brushless motor, eCom keep-alive         */
} ModeSelect_t;

typedef enum {
    STATE_PASSTHROUGH = 0,
    STATE_RAMP_IN,          /* brake ramp-in in progress       */
    STATE_BRAKING,          /* full brake applied              */
    STATE_ANTI_BRAKE,       /* positive anti-brake active      */
    STATE_KEEPALIVE_ONLY,   /* mode C, throttle active         */
    STATE_CAPTURE,          /* capture button held             */
    STATE_SAFE_BRAKE,
} OpState_t;

typedef struct {
    ModeSelect_t  mode_sel;
    OpState_t     state;
    uint16_t      pot_raw;
    uint16_t      white_mv;
    uint16_t      black_mv;
    uint16_t      btn_raw;
    uint16_t      brake_ccr;        /* current TIM3_CH1 CCR   */
    uint16_t      brake_ccr_target; /* ramp destination       */
    uint16_t      ka_ccr;           /* current TIM3_CH2 CCR   */
    uint16_t      saved_ka_ccr;     /* from flash             */
    uint16_t      saved_ramp_ms;    /* from flash, 0–200      */
    bool          brake_active;     /* hysteresis state       */
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
