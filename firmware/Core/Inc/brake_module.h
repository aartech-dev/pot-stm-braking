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
 *    Display shows current KA floor voltage (not track voltage)
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
 *    PA5  DAC_OUT2  Q1 KA base   → GD1 op-amp → Q1 TIP147 PNP
 *    PA9  USART1_TX  UART 115200 8N1
 *    PA10 USART1_RX
 *    PA13 SWDIO  PA14 SWDCLK / BOOT0   (G0: BOOT0 is PA14-BOOT0)
 *    PB0  Toggle bit 0 (active LOW, pull-up)
 *    PB1  Toggle bit 1 (active LOW, pull-up)
 *    PB4  TM1637 CLK    (display clock → J14)
 *    PB5  Status LED 2 (green: drive/keep-alive)
 *    PB6  Status LED (red: brake/fault; blinks on save)
 *    PB8  TM1637 DIO    (display data → J14; was mislabelled BOOT0)
 *
 *  Flash record (page 31, 0x08007C00), magic 0xAA270008:
 *    +0x00 DW0 : {ramp_ms:16, ka_dac:16, magic:32}
 *    +0x08 DW1 : {ramp_curve:16, pot_min:16, b_exit:16, b_enter:16}
 *    +0x10 DW2 : {0xFFFFFFFF:32, latvian:16, brake_ohms_idx:16}
 *    +0x18 DW3 : {rel_c:16, rel_b:16, 0xFFFF:16, rel_a:16}
 *
 *  BOOT0 (G0): BOOT0 is the PA14-BOOT0 pin, NOT PB8. For the J12 button
 *  to enter the ROM bootloader, wire J12 to PA14 (series R, shares SWCLK)
 *  and program option bit nBOOT_SEL=0 so the pin level is used. PB8 now
 *  drives the TM1637 display data line. (Schematic still shows the old
 *  PB8 routing — KiCad TODO.)
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
#define IDIR_HOLD_TICKS        50U     /* hold brake across brief reverse-detect dropouts; tune vs tick rate */
#define RAIL_UNDERVOLTAGE_MV   5000U

/* ── ADC ─────────────────────────────────────────────────── */
#define ADC_MAX                4095U
#define POT_MIN_MAX            2048U   /* feature J: max accepted pot CCW zero offset */
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

/* Q1 anti-brake CURRENT SOURCE — DAC2 code sets demand       */
/* DAC2 (DAC_OUT2/PA5) DC -> R_KA1/R_KA2 divider -> GD3 -> GD1. */
/* 0 = off, KA_DAC_MAX = ~3A. Higher code = more current.      */
#define KA_DAC_OFF             0U      /* DAC code 0 = 0A (off) */
#define KA_DAC_MAX             2480U   /* max KA on the 0-4095 DAC scale (~3A)   */
#define DAC_KA_OFF             KA_DAC_OFF   /* alias             */
#define DAC_KA_MAX             KA_DAC_MAX   /* alias             */
#define DEFAULT_KA_DAC         KA_DAC_OFF

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
/* Ramp shape (feature D): SET RAMP_CURVE 0|1|2. Reshapes brake-in. */
#define RAMP_CURVE_LINEAR      0U      /* constant step                */
#define RAMP_CURVE_EXP         1U      /* fast initial bite (default)  */
#define RAMP_CURVE_LOG         2U      /* slow start, late bite        */
#define RAMP_CURVE_DEFAULT     RAMP_CURVE_EXP
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

/* ── TM1637 4-digit display (PB4=CLK, PB8=DIO, bit-banged) ─ */
#define TM1637_PORT            GPIOB
#define TM1637_CLK_PIN         GPIO4
#define TM1637_DIO_PIN         GPIO8
#define TM1637_BRIGHTNESS      3U       /* 0..7 (display-on cmd adds 0x88) */
#define DISP_RAIL_FS_MV        16000U   /* Mode A/B: rail display full-scale */
#define DISP_FLOOR_FS_MV       2000U    /* Mode C: KA floor display full-scale */

/* ── Pins ────────────────────────────────────────────────── */
#define TOGGLE0_PORT           GPIOB
#define TOGGLE0_PIN            GPIO0
#define TOGGLE1_PORT           GPIOB
#define TOGGLE1_PIN            GPIO1
#define LED_PORT               GPIOB
#define LED_PIN                GPIO6   /* red: brake/fault */
#define LED2_PORT              GPIOB
#define LED2_PIN               GPIO5   /* green: drive/keep-alive. CONFIRM PB5 free on QFN-32 schematic */
#define LED2_BLINK_MS          250U    /* keep-alive floor: slow green blink */

/* LM311 ideal-diode reverse-detect flag -> MCU (current-direction brake detect) */
#define IDIR_PORT              GPIOA
#define IDIR_PIN               GPIO6   /* PA6; CONFIRM free pin, polarity, and 3.3V level of I_DIR */
#define DAC_BRAKE_CHAN         DAC_CHANNEL1   /* PA4 → Q2 */
#define DAC_KA_CHAN            DAC_CHANNEL2   /* PA5 → GD1 anti-brake demand */
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
    uint16_t      saved_pot_min;      /* feature J: pot CCW zero offset */

    /* Mode C save debounce                                     */
    bool          ka_pending_save;
    uint32_t      ka_save_tick;   /* system tick at toggle exit  */

    bool          brake_active;
    uint16_t      idir_hold;       /* I_DIR brake-hold countdown (ticks) */
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
                                   uint16_t rel_ms_c,
                                   uint16_t pot_min,
                                   uint16_t ramp_curve);
void                brake_led_blink(void);
void                brake_calibrate_pot_min(void);   /* feature J: capture pot CCW zero */
uint16_t            brake_ohms_label_at(uint8_t idx);
uint8_t             brake_ohms_nearest(uint16_t ohms);

#endif /* BRAKE_MODULE_H */
