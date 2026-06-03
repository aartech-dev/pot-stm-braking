#ifndef BRAKE_MODULE_H
#define BRAKE_MODULE_H

/* ============================================================
 *  AART Slot Car Braking Module  — Rev 6
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
 *    PA9  USART1_TX AF1  → J11 pin 1 (UART/DBG TX, 115200 8N1)
 *    PA10 USART1_RX AF1  → J11 pin 2 (UART/DBG RX)
 *    PA13 SWDIO      AF0  → J10 pin 1 (SWD debug header)
 *    PA14 SWDCLK     AF0  → J10 pin 2 (SWD debug header)
 *    PB6  GPIO OUT  Status LED          (blinks on flash save)
 *    PB8  BOOT0          → J12 (BOOT0 button for ROM bootloader)
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
 *  ── Latvian brake (track call / emergency stop) ──────────
 *  Monitors WHITE rail dV/dt each 1ms tick.
 *  If WHITE drops faster than latvian_dvdt_mv_per_ms, fires
 *  an immediate dead short regardless of mode, pot, or ramp.
 *  Q2 is also cut — keep-alive cannot run without WHITE rail.
 *
 *  Use case: marshal hits track kill switch during a race.
 *  WHITE collapses instantly; all cars must stop as fast as
 *  physically possible. The software fires before the MCU
 *  browns out; the hardware pull-up on GD1 holds the brake
 *  on after MCU power is gone.
 *
 *  Release: WHITE rises above BRAKE_EXIT_MV.
 *  Enable/disable: SET LATVIAN_DVDT <mV_per_ms> via UART.
 *    0 = disabled (default). Recommended starting value: 1000.
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
 *  Latvian brake ALWAYS bypasses ramp-in — immediate dead short.
 *
 *  ── Capture button (dual function) ──────────────────────
 *  Mode C: hold → pot dials keep-alive voltage; release → save
 *  Mode A/B: hold → pot dials ramp time (CCW=0ms, CW=200ms)
 *  Both save to flash page 31. LED blinks 3× on save.
 *
 *  ── Component selection notes ───────────────────────────
 *  D1  P6KE18CA (bidirectional TVS) — CA suffix essential.
 *  D2/D3  SS310 (3A, 100V) — 100V reverse rating required.
 *  D4  MBRS360 or SS54 — repetitive 46A peak in mode C.
 *  F1  500mA-1A polyfuse in WHITE rail only. Not in track path.
 *  R8  18kΩ — 15k gives 3.82V at 16V, exceeds ADC reference.
 *
 *  ── BSCRA/ISRA legality note ─────────────────────────────
 *  All injection voltages sourced directly from WHITE rail.
 *  No stored energy. Satisfies direct track power requirement.
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
#define WHITE_DIV_NUM          47U
#define WHITE_DIV_DEN          227U
#define BLACK_DIV_NUM          47U
#define BLACK_DIV_DEN          267U

/* ── Brake hysteresis (mV on BLACK) ──────────────────────── */
#define BRAKE_ENTER_MV         500U
#define BRAKE_EXIT_MV          1500U
#define RAIL_UNDERVOLTAGE_MV   5000U

/* ── Latvian brake defaults ──────────────────────────────── */
/* dV/dt threshold on WHITE rail (mV per 1ms tick).
 * 0 = disabled. Recommended: 1000 (1V/ms = 1000V/s).
 * At 12V nominal, a 1V/ms drop means the rail hits zero in
 * 12ms — well within one PWM period at 20kHz.               */
#define LATVIAN_DVDT_DEFAULT   0U        /* disabled until SET via UART */
#define LATVIAN_DVDT_MIN       100U      /* minimum meaningful threshold */
#define LATVIAN_DVDT_MAX       5000U     /* 5V/ms — catches only instant cuts */

/* ── ADC ─────────────────────────────────────────────────── */
#define ADC_MAX                4095U
#define ADC_CENTRE             2048U
#define ADC_DEADBAND           150U
#define BTN_PRESSED_THRESHOLD  2200U

/* ── PWM ─────────────────────────────────────────────────── */
#define PWM_ARR                3199U
#define BRAKE_CCR_HARD         0U
#define BRAKE_CCR_SOFT         2720U
#define BRAKE_CCR_OFF          PWM_ARR
#define KA_CCR_MAX             400U
#define KA_CCR_OFF             0U

/* ── Brake ramp-in ───────────────────────────────────────── */
#define RAMP_MS_MAX            200U
#define RAMP_ALPHA_INSTANT     256U
#define RAMP_ALPHA_MIN         3U
#define RAMP_MS_TO_ALPHA(ms)   ((ms) == 0U ? RAMP_ALPHA_INSTANT \
    : (uint16_t)(RAMP_ALPHA_INSTANT - \
      ((uint32_t)(RAMP_ALPHA_INSTANT - RAMP_ALPHA_MIN) * (ms)) \
      / RAMP_MS_MAX))

/* ── Flash NV storage ────────────────────────────────────── */
/* Page 31 (0x08007C00), 1K — reserved by linker (LENGTH=31K).
 * Three 64-bit double-word writes (24 bytes total):
 *
 *   Offset 0x00:  [63:32] = {ramp_ms:16, ka_ccr:16}
 *                 [31:0]  = magic (0xAA270002)
 *   Offset 0x08:  [63:32] = {brake_exit_mv:16, brake_enter_mv:16}
 *                 [31:0]  = 0xFFFFFFFF (padding)
 *   Offset 0x10:  [63:32] = {latvian_dvdt:16, brake_ccr_soft:16}
 *                 [31:0]  = 0xFFFFFFFF (padding)
 *
 * Magic bumped to 0xAA270002 — old single-word records are
 * automatically invalidated and defaults used until SAVE.    */
#define FLASH_SAVE_PAGE        31U
#define FLASH_SAVE_ADDR        (0x08000000U + (FLASH_SAVE_PAGE * 1024U))
#define FLASH_MAGIC            0xAA270002U

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
#define ADC_DMA_CHANNEL        DMA_CHANNEL1
#define ADC_DMAMUX_REQ         5U
#define ADC_NUM_CHANNELS       4U

/* ── Types ───────────────────────────────────────────────── */
typedef enum {
    MODE_A = 0,
    MODE_B,
    MODE_C,
} ModeSelect_t;

typedef enum {
    STATE_PASSTHROUGH = 0,
    STATE_RAMP_IN,
    STATE_BRAKING,
    STATE_ANTI_BRAKE,
    STATE_KEEPALIVE_ONLY,
    STATE_CAPTURE,
    STATE_LATVIAN_BRAKE,    /* emergency dead short — track call  */
    STATE_SAFE_BRAKE,
} OpState_t;

typedef struct {
    ModeSelect_t  mode_sel;
    OpState_t     state;
    uint16_t      pot_raw;
    uint16_t      white_mv;         /* current WHITE voltage mV   */
    uint16_t      white_prev_mv;    /* previous tick WHITE mV     */
    uint16_t      black_mv;
    uint16_t      btn_raw;
    uint16_t      brake_ccr;
    uint16_t      brake_ccr_target;
    uint16_t      ka_ccr;
    uint16_t      saved_ka_ccr;
    uint16_t      saved_ramp_ms;
    bool          brake_active;     /* normal brake hysteresis    */
    bool          latvian_active;   /* Latvian brake latched       */
    bool          btn_pressed;
    bool          capture_active;
} BrakeCtx_t;

/* ── Public API ──────────────────────────────────────────── */
void                brake_init(void);
void                brake_tick(void);
OpState_t           brake_get_state(void);
ModeSelect_t        brake_get_mode_sel(void);
const BrakeCtx_t   *brake_get_ctx(void);
void                brake_force_safe(void);
void                brake_systick_isr(void);
void                flash_save_all(uint16_t ka_ccr,   uint16_t ramp_ms,
                                   uint16_t b_enter,  uint16_t b_exit,
                                   uint16_t b_soft,   uint16_t latvian);
/* Trigger the status LED blink sequence (e.g. after UART SAVE) */
void                brake_led_blink(void);

#endif /* BRAKE_MODULE_H */
