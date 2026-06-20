#ifndef BRAKE_MODULE_H
#define BRAKE_MODULE_H

/* ============================================================
 *  AART Slot Car Braking Module  — Revision 11
 *  Target  : STM32G051K6U6 (QFN-32)
 *  Library : libopencm3
 *
 *  Mode A  — Brake only (brushed and brushless)
 *    Pot: linear DAC_BRAKE_SOFT(CCW) → DAC_BRAKE_HARD(CW)
 *    Q1: applies saved_ka_dac as a floor whenever
 *        V_BLACK would drop below it. 0 = no floor.
 *    Toggle to Mode C to set the floor, then return to A.
 *
 *  Mode B  — Anti-brake (brushed only)
 *    Pot: indexes brake_dac_table[] and ka_dac_table[]
 *         simultaneously. Both outputs live at all times.
 *    CCW = min brake + max anti-brake (~2V)
 *    CW  = max brake + no anti-brake
 *    No KA floor applied.
 *
 *  Mode C  — Keep-alive tuning (brushless)
 *    Pot: dials in KA floor live via ka_dac_table[idx]
 *    Voltmeter shows current KA voltage (not track voltage)
 *    On leaving Mode C (500ms after toggle change):
 *      saved_ka_dac = ka_dac_table[pot_idx]
 *      LED blinks 3x, value used in Mode A thereafter
 *    To zero the floor: pot full CCW in Mode C, toggle to A
 *
 *  Toggle decoding (PB0=bit0, PB1=bit1, LOW=asserted):
 *    PB0=H PB1=H → MODE_A
 *    PB0=L PB1=H → MODE_B
 *    PB0=H PB1=L → MODE_C
 *    PB0=L PB1=L → MODE_A (invalid, safe fallback)
 *
 *  Pin assignments:
 *    PA0  ADC_IN0   WHITE rail sense (R8=18k/R9=4k7)
 *    PA1  ADC_IN1   BLACK_to_track   (R10=22k/R11=4k7)
 *    PA3  ADC_IN3   Pot wiper (10k linear, no centre detent)
 *    PA4  DAC_CH1   Q2 brake gate → GD2 op-amp → Q2 N-ch
 *    PA5  DAC_CH2   Q1 KA gate   → GD1 op-amp → Q1 P-ch
 *    PA9  USART1_TX  UART 115200 8N1
 *    PA10 USART1_RX
 *    PA13 SWDIO  PA14 SWDCLK
 *    PB0  Toggle bit 0 (active LOW, pull-up)
 *    PB1  Toggle bit 1 (active LOW, pull-up)
 *    PB4  TIM3_CH1 AF1  Voltmeter PWM (1kHz → RC → J14)
 *    PB6  Status LED (blinks 3x on KA save)
 *    PB8  BOOT0 → J12
 *
 *  Flash record (page 31, 0x08007C00), magic 0xAA270008:
 *    DW0: {ramp_ms:16, ka_dac:16, magic:32}
 *    DW1: {brake_exit:16, brake_enter:16, 0xFFFF:32}
 *    DW2: {latvian:16, brake_ohms_idx:16, 0xFFFF:32}
 *    DW3: {rel_ms_c:16, rel_ms_b:16, rel_ms_a:32}
 * ============================================================ */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <stdint.h>
#include <stdbool.h>

/* ── Voltage dividers ────────────────────────────────────── */
#define WHITE_DIV_NUM          47U
#define WHITE_DIV_DEN          227U
#define BLACK_DIV_NUM          47U
#define BLACK_DIV_DEN          267U

/* ── Brake hysteresis ────────────────────────────────────── */
#define BRAKE_ENTER_MV         500U
#define BRAKE_EXIT_MV          1500U
#define RAIL_UNDERVOLTAGE_MV   5000U

/* ── ADC ─────────────────────────────────────────────────── */
#define ADC_MAX                4095U
#define ADC_NUM_CHANNELS       3U

/* ── DAC (12-bit, 0–4095 = 0–3.3V) ──────────────────────── */
/* Q2 N-ch brake — linear region ~1.5V to ~2.5V gate         */
#define DAC_BRAKE_OFF          0U
#define DAC_BRAKE_SOFT         1860U   /* legacy ref (~8Ω est.)*/
#define DAC_BRAKE_HARD         3095U   /* ~2.5V → dead short   */

/* Minimum braking resistance (pot CCW, gentlest brake) is
 * selectable via SET BRAKE_OHMS <1-5>, choosing one of five
 * precomputed brake tables. Higher DAC = higher Q2 gate voltage
 * = lower Rds(on) = LOWER resistance. The 1..5Ω labels are
 * nominal estimates; calibrate per motor and regenerate tables
 * with tools/gen_brake_tables.py. No runtime table math.        */
#define BRAKE_OHMS_COUNT       7U      /* 25,15,5,4,3,2,1Ω      */
#define BRAKE_OHMS_DEFAULT     4U      /* index 4 = ~3Ω default  */
#define BRAKE_OHMS_ARG_MIN     1U      /* SET BRAKE_OHMS Ω clamp  */
#define BRAKE_OHMS_ARG_MAX     30U     /* (snapped to nearest)   */

/* Q1 anti-brake CURRENT SOURCE — PWM duty sets demand        */
/* 50kHz PWM -> level-shift + RC filter -> current-source ref. */
/* 0 = off, KA_PWM_MAX = ~3A. Higher duty = more current.      */
#define KA_PWM_OFF             0U      /* 0% duty = 0A (off)    */
#define KA_PWM_MAX             2480U   /* full duty -> ~3A      */
#define DAC_KA_OFF             KA_PWM_OFF   /* compat alias      */
#define DAC_KA_MAX             KA_PWM_MAX   /* compat alias      */
#define DEFAULT_KA_DAC         KA_PWM_OFF
#define KA_PWM_ARR             2499U   /* TIM ARR for 50kHz @ 125MHz tim clk; confirm on bench */

/* ── KA save debounce (ms after leaving Mode C) ─────────── */
#define KA_SAVE_DEBOUNCE_MS    500U

/* ── Latvian brake ───────────────────────────────────────── */
#define LATVIAN_DVDT_DEFAULT   0U
#define LATVIAN_DVDT_MIN       100U
#define LATVIAN_DVDT_MAX       5000U

/* ── Ramp-in ─────────────────────────────────────────────── */
#define RAMP_MS_MAX            200U
#define RAMP_ALPHA_INSTANT     256U
#define RAMP_ALPHA_MIN         3U
#define RAMP_MS_TO_ALPHA(ms)   ((ms) == 0U ? RAMP_ALPHA_INSTANT \
    : (uint16_t)(RAMP_ALPHA_INSTANT - \
      ((uint32_t)(RAMP_ALPHA_INSTANT - RAMP_ALPHA_MIN) * (ms)) \
      / RAMP_MS_MAX))

/* ── Trail release ───────────────────────────────────────── */
#define RELEASE_MS_MAX         500U
#define RELEASE_SNAP_MV        500U

/* ── Flash ───────────────────────────────────────────────── */
#define FLASH_SAVE_PAGE        31U
#define FLASH_SAVE_ADDR        (0x08000000U + (FLASH_SAVE_PAGE * 1024U))
#define FLASH_MAGIC            0xAA270008U

/* ── Timing ──────────────────────────────────────────────── */
#define BTN_DEBOUNCE_MS        20U
#define LED_BLINK_MS           80U
#define LED_BLINK_COUNT        3U

/* ── Voltmeter PWM (TIM3_CH1, PB4, AF1) ─────────────────── */
#define VOLTMETER_PWM_PORT     GPIOB
#define VOLTMETER_PWM_PIN      GPIO4
#define VOLTMETER_TIM          TIM3
#define VOLTMETER_TIM_ARR      3199U
#define VOLTMETER_TIM_PSC      19U
#define VOLTMETER_SCALE_MV     16000U

/* ── Pins ────────────────────────────────────────────────── */
#define TOGGLE0_PORT           GPIOB
#define TOGGLE0_PIN            GPIO0
#define TOGGLE1_PORT           GPIOB
#define TOGGLE1_PIN            GPIO1
#define LED_PORT               GPIOB
#define LED_PIN                GPIO6
#define DAC_BRAKE_CHAN         DAC_CHANNEL1   /* PA4 → Q2 */
/* Anti-brake is PWM now: PA5 = TIM2_CH1 (AF2). DAC_CH2 freed. */
#define KA_PWM_TIMER           TIM2
#define KA_PWM_OC              TIM_OC1        /* PA5 TIM2_CH1 */
#define ADC_DMA_CHANNEL        DMA_CHANNEL1
#define ADC_DMAMUX_REQ         5U

/* ── Identity ────────────────────────────────────────────── */
#define FW_ID_STRING           "AART-BRAKE-v11-STM32G051"

/* ── Mode select ─────────────────────────────────────────── */
typedef enum {
    MODE_A = 0,   /* Brake only — linear pot, all motors       */
    MODE_B,       /* Anti-brake — table-coupled, brushed only  */
    MODE_C,       /* KA tuning — pot dials floor, save on exit */
} ModeSelect_t;

/* ── States ──────────────────────────────────────────────── */
typedef enum {
    STATE_PASSTHROUGH = 0,
    STATE_RAMP_IN,
    STATE_BRAKING,
    STATE_RELEASE,
    STATE_ANTI_BRAKE,
    STATE_KEEPALIVE_ONLY,
    STATE_KA_TUNING,        /* in Mode C, pot sets KA floor     */
    STATE_LATVIAN_BRAKE,
    STATE_SAFE_BRAKE,
} OpState_t;

/* ── Context ─────────────────────────────────────────────── */
typedef struct {
    ModeSelect_t  mode_sel;
    ModeSelect_t  mode_prev;
    OpState_t     state;

    uint16_t      pot_raw;      /* 0–4095 ADC reading            */
    uint8_t       pot_idx;      /* 0–255 table index             */
    uint16_t      white_mv;
    uint16_t      white_prev_mv;
    uint16_t      black_mv;

    /* Q1 (KA/anti-brake) output — alphabetically before Q2    */
    uint16_t      ka_dac;
    /* Q2 (brake) output                                        */
    uint16_t      brake_dac;
    uint16_t      brake_dac_target;

    /* Saved KA floor (set on leaving Mode C, used in Mode A)  */
    uint16_t      saved_ka_dac;
    uint16_t      saved_ramp_ms;
    uint16_t      saved_release_ms_a;
    uint16_t      saved_release_ms_b;
    uint16_t      saved_release_ms_c;

    /* Mode C save debounce                                     */
    bool          ka_pending_save;
    uint32_t      ka_save_tick;   /* system tick at toggle exit  */

    bool          brake_active;
    bool          latvian_active;
} BrakeCtx_t;

/* ── Public API ──────────────────────────────────────────── */
void                brake_init(void);
void                brake_tick(void);
OpState_t           brake_get_state(void);
ModeSelect_t        brake_get_mode_sel(void);
const BrakeCtx_t   *brake_get_ctx(void);
void                brake_force_safe(void);
void                brake_systick_isr(void);
void                flash_save_all(uint16_t ka_dac,
                                   uint16_t ramp_ms,
                                   uint16_t b_enter,
                                   uint16_t b_exit,
                                   uint16_t b_ohms_idx,
                                   uint16_t latvian,
                                   uint16_t rel_ms_a,
                                   uint16_t rel_ms_b,
                                   uint16_t rel_ms_c);
void                brake_led_blink(void);
uint16_t            brake_ohms_label_at(uint8_t idx);
uint8_t             brake_ohms_nearest(uint16_t ohms);

#endif /* BRAKE_MODULE_H */
