#ifndef BRAKE_MODULE_H
#define BRAKE_MODULE_H

/* ============================================================
 *  AART Slot Car Braking Module
 *  Target  : STM32G041J6M6 (SOIC-8)
 *  Library : libopencm3 — no ST HAL
 *
 *  Pin assignments:
 *    PA0  ADC_IN0   Rail sense (12.4V ÷ divider → ≤3.3V)
 *    PA3  ADC_IN3   Pot wiper (10k centre-detent, 0–3.3V)
 *    PA1  GPIO IN   Wiper strip contact (active LOW, pull-up)
 *    PA6  TIM3_CH1  Brake FET PWM  → gate driver → Q1
 *    PA7  TIM3_CH2  Antibrake FET PWM → gate driver → Q2
 * ============================================================ */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <stdint.h>
#include <stdbool.h>

/* ── ADC ─────────────────────────────────────────────────── */
#define ADC_MAX               4095U
#define ADC_CENTRE            2048U
#define ADC_DEADBAND          150U      /* ±counts around centre = freewheel */

/* Rail sense divider: R_bottom / (R_top + R_bottom)
 * Default: 4.7k / (15k + 4.7k) = 0.2386
 * V_PA0 = 12.4 × 0.2386 = 2.96V — safe for 3.3V input */
#define ADC_RAIL_DIVIDER_NUM  47U       /* numerator   ×10 = 4.7k */
#define ADC_RAIL_DIVIDER_DEN  197U      /* denominator ×10 = 19.7k */

/* Below this (mV) → force safe brake state */
#define RAIL_UNDERVOLTAGE_MV  5000U

/* ── PWM ─────────────────────────────────────────────────── */
/* TIM3 @ 20 kHz: SYSCLK=64MHz, PSC=0, ARR=3199
 * Period = 64,000,000 / (0+1) / (3199+1) = 20,000 Hz          */
#define PWM_ARR               3199U

/* Brake FET: CCR=0 → FET fully ON (inverted polarity on OC)
 *            CCR=PWM_ARR → FET fully OFF
 * Lower CCR = more braking = shorter off-time = harder short   */
#define BRAKE_CCR_HARD        0U        /* hard short */
#define BRAKE_CCR_SOFT        2720U     /* ~8Ω effective at pot minimum */
#define BRAKE_CCR_OFF         PWM_ARR   /* FET off */

/* Antibrake FET: normal polarity
 * CCR=0 → off, CCR=ANTI_CCR_MAX → ~2V (16% of 12.4V)          */
#define ANTI_CCR_MAX          516U      /* 516/3200 × 12.4V ≈ 2.0V */
#define ANTI_CCR_OFF          0U

/* ── Timing ──────────────────────────────────────────────── */
#define WIPER_DEBOUNCE_MS     10U
#define SYSTICK_FREQ_HZ       1000U     /* 1ms tick */

/* ── GPIO / peripheral aliases ───────────────────────────── */
#define WIPER_PORT  GPIOA
#define WIPER_PIN   GPIO1               /* PA1 — active LOW */
#define ADC_PORT    GPIOA
#define ADC_PIN_RAIL  GPIO0             /* PA0 — rail sense */
#define ADC_PIN_POT   GPIO3             /* PA3 — pot wiper */
#define PWM_PORT    GPIOA
#define PWM_PIN_BRAKE GPIO6             /* PA6 — TIM3_CH1 */
#define PWM_PIN_ANTI  GPIO7             /* PA7 — TIM3_CH2 */

/* ── DMA ─────────────────────────────────────────────────── */
/* ADC1 → DMA1 Channel 1, DMAMUX request 5 (ADC1 on G0)        */
#define ADC_DMA_CHANNEL   DMA_CHANNEL1
#define ADC_DMAMUX_REQ    5             /* DMAMUX1_C0CR: ADC1 = 5 on G041 */

/* ── Types ───────────────────────────────────────────────── */
typedef enum {
    MODE_PASSTHROUGH = 0,
    MODE_BRAKING,
    MODE_FREEWHEEL,
    MODE_ANTIBRAKE,
    MODE_SAFE_BRAKE,
} BrakeMode_t;

typedef struct {
    BrakeMode_t mode;
    uint16_t    pot_raw;
    uint16_t    rail_mv;
    uint16_t    brake_ccr;
    uint16_t    anti_ccr;
    bool        wiper_active;
} BrakeState_t;

/* ── Public API ──────────────────────────────────────────── */
void        brake_init(void);
void        brake_tick(void);           /* call from SysTick or main loop */
BrakeMode_t brake_get_mode(void);
void        brake_force_safe(void);     /* safe to call from fault handler */

/* Exposed for SysTick ISR in main.c */
void        brake_systick_isr(void);

#endif /* BRAKE_MODULE_H */
