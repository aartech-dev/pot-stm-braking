/* ============================================================
 *  brake_module.c — AART Slot Car Braking Module
 *  Library: libopencm3 (no ST HAL)
 *  Target : STM32G041J6M6
 * ============================================================ */

#include "brake_module.h"

/* ── Private state ───────────────────────────────────────── */
static BrakeState_t s_state;
static volatile uint32_t s_tick_ms   = 0;   /* incremented by SysTick ISR */
static volatile bool     s_adc_ready = false;

/* DMA destination — two 16-bit results: [0]=rail(PA0), [1]=pot(PA3) */
static volatile uint16_t s_adc_buf[2];

/* Wiper debounce */
static uint32_t s_wiper_last_change_ms = 0;
static bool     s_wiper_raw_prev       = false;

/* ── Utility: millisecond tick (called from SysTick ISR) ─── */
void brake_systick_isr(void)
{
    s_tick_ms++;
}

static uint32_t now_ms(void)
{
    return s_tick_ms;
}

/* ── Utility: linear map with clamping ───────────────────── */
static uint16_t map_u16(uint16_t val,
                         uint16_t in_lo,  uint16_t in_hi,
                         uint16_t out_lo, uint16_t out_hi)
{
    if (val <= in_lo)  return out_lo;
    if (val >= in_hi)  return out_hi;
    uint32_t span_in  = in_hi  - in_lo;
    uint32_t span_out = out_hi - out_lo;
    return (uint16_t)(out_lo + ((uint32_t)(val - in_lo) * span_out) / span_in);
}

/* ── CCR setters ─────────────────────────────────────────── */
/* Brake channel (TIM3_CH1) uses INVERTED polarity:
 *   CCR=0         → output stays HIGH → gate driver → FET ON  (hard brake)
 *   CCR=PWM_ARR   → output stays LOW  → gate driver → FET OFF
 *
 * This gives us "CCR=0 = maximum braking" which is intuitive,
 * and means the 10k pull-up on the gate driver input defaults
 * to brake-on when the MCU is unpowered or in reset.             */
static void set_brake_ccr(uint16_t ccr)
{
    if (ccr > PWM_ARR) ccr = PWM_ARR;
    timer_set_oc_value(TIM3, TIM_OC1, ccr);
    s_state.brake_ccr = ccr;
}

/* Antibrake channel (TIM3_CH2) normal polarity:
 *   CCR=0              → FET off
 *   CCR=ANTI_CCR_MAX   → ~2V injected                            */
static void set_anti_ccr(uint16_t ccr)
{
    if (ccr > ANTI_CCR_MAX) ccr = ANTI_CCR_MAX;
    timer_set_oc_value(TIM3, TIM_OC2, ccr);
    s_state.anti_ccr = ccr;
}

/* ── Clock setup: HSI16 → PLL → 64 MHz ──────────────────── */
static void clock_setup(void)
{
    /* Use the libopencm3 helper for the G0 PLL configuration.
     * rcc_clock_setup() handles flash wait-states automatically. */
    const struct rcc_clock_scale clock_config = {
        .sysclock_source  = RCC_PLL,
        .pll_source       = RCC_PLLCFGR_PLLSRC_HSI16,
        .pll_div          = RCC_PLLCFGR_PLLM_DIV(1),
        .pll_mul          = RCC_PLLCFGR_PLLN_MUL(8),   /* 16×8 = 128 MHz VCO */
        .pllp_div         = RCC_PLLCFGR_PLLP_DIV(2),
        .pllq_div         = RCC_PLLCFGR_PLLQ_DIV(2),
        .pllr_div         = RCC_PLLCFGR_PLLR_DIV(2),   /* 128/2 = 64 MHz */
        .hpre             = RCC_CFGR_HPRE_NODIV,
        .ppre             = RCC_CFGR_PPRE_NODIV,
        .flash_waitstates = 2,              /* 2 wait states required at 64MHz */
        .ahb_frequency    = 64000000,
        .apb_frequency    = 64000000,
        .voltage_scale    = PWR_SCALE1,
    };
    rcc_clock_setup(&clock_config);
}

/* ── GPIO setup ──────────────────────────────────────────── */
static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);

    /* PA0, PA3 — ADC analog inputs */
    gpio_mode_setup(ADC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                    ADC_PIN_RAIL | ADC_PIN_POT);

    /* PA1 — wiper strip digital input, pull-up (active LOW) */
    gpio_mode_setup(WIPER_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,
                    WIPER_PIN);

    /* PA6, PA7 — TIM3 PWM alternate function (AF1 on G0) */
    gpio_mode_setup(PWM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    PWM_PIN_BRAKE | PWM_PIN_ANTI);
    gpio_set_af(PWM_PORT, GPIO_AF1, PWM_PIN_BRAKE | PWM_PIN_ANTI);
}

/* ── TIM3 PWM setup ──────────────────────────────────────── */
static void timer_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_reset_pulse(RST_TIM3);   /* G0: use rcc reset, not timer_reset */

    /* Up-counting, no prescaler, period = PWM_ARR → 20 kHz */
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM3, 0);
    timer_set_period(TIM3, PWM_ARR);
    timer_enable_preload(TIM3);

    /* CH1 — brake — PWM mode 1, INVERTED polarity
     * OC mode 1: active while CNT < CCR.
     * With inverted polarity the pin is HIGH when CNT < CCR.
     * CCR=0 → pin always HIGH → gate driver always on → brake.  */
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_low(TIM3, TIM_OC1);      /* INVERTED */
    timer_enable_oc_preload(TIM3, TIM_OC1);
    timer_set_oc_value(TIM3, TIM_OC1, BRAKE_CCR_HARD); /* start: brake ON */
    timer_enable_oc_output(TIM3, TIM_OC1);

    /* CH2 — antibrake — PWM mode 1, normal polarity */
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM3, TIM_OC2);     /* normal */
    timer_enable_oc_preload(TIM3, TIM_OC2);
    timer_set_oc_value(TIM3, TIM_OC2, ANTI_CCR_OFF); /* start: off */
    timer_enable_oc_output(TIM3, TIM_OC2);

    timer_generate_event(TIM3, TIM_EGR_UG); /* load preloaded values */
    timer_enable_counter(TIM3);
}

/* ── DMA setup ───────────────────────────────────────────── */
/* On STM32G0 the DMA controller uses a DMAMUX to route
 * peripheral requests to channels. This is different from F0/F1
 * where the mapping is fixed. We must configure DMAMUX1_C0CR
 * to connect ADC1 (request ID 5) to DMA1 Channel 1.            */
static void dma_setup(void)
{
    rcc_periph_clock_enable(RCC_DMA1);

    dma_channel_reset(DMA1, ADC_DMA_CHANNEL);

    /* DMAMUX: route ADC1 (request ID 5) to DMA1 Channel 1.
     * G0 DMAMUX1 base = 0x40020800. Each channel has a 4-byte CxCR register.
     * Channel 0 (= DMA1 Ch1) CxCR is at offset 0x00. Bits[6:0] = request ID.
     * We write directly to avoid pulling in the DMAMUX header.              */
    MMIO32(0x40020800U) = ADC_DMAMUX_REQ;  /* DMAMUX1_C0CR = ADC1 req (5) */

    dma_set_peripheral_address(DMA1, ADC_DMA_CHANNEL,
                               (uint32_t)&ADC_DR(ADC1));
    dma_set_memory_address(DMA1, ADC_DMA_CHANNEL,
                           (uint32_t)s_adc_buf);
    dma_set_number_of_data(DMA1, ADC_DMA_CHANNEL, 2);

    dma_set_read_from_peripheral(DMA1, ADC_DMA_CHANNEL);
    dma_set_peripheral_size(DMA1, ADC_DMA_CHANNEL, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, ADC_DMA_CHANNEL, DMA_CCR_MSIZE_16BIT);
    dma_enable_memory_increment_mode(DMA1, ADC_DMA_CHANNEL);
    dma_enable_circular_mode(DMA1, ADC_DMA_CHANNEL);

    /* Enable transfer-complete interrupt */
    dma_enable_transfer_complete_interrupt(DMA1, ADC_DMA_CHANNEL);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
    nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 1);

    dma_enable_channel(DMA1, ADC_DMA_CHANNEL);
}

/* ── ADC setup ───────────────────────────────────────────── */
/* Scan mode: two channels in sequence (CH0 then CH3), DMA.     */
static void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC);

    adc_power_off(ADC1);
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_PCLK_DIV2); /* 32 MHz ADC clock */
    adc_calibrate(ADC1);                /* must be done while ADC is off */
    adc_power_on(ADC1);

    /* Wait for ADC to be ready */
    while (!adc_is_power_off(ADC1) == false);

    adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
    adc_set_single_conversion_mode(ADC1);   /* triggered manually */
    /* G0 ADC: scan is enabled implicitly when sequence length > 1.
     * No adc_enable_scan_mode() exists for this family.           */
    adc_set_right_aligned(ADC1);

    /* Sampling time: 39.5 ADC cycles — adequate for 10k source */
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMPx_039DOT5CYC);

    /* Sequence: CH0 (rail), CH3 (pot) — length 2 */
    uint8_t channels[2] = { 0, 3 };
    adc_set_regular_sequence(ADC1, 2, channels);

    /* DMA: circular, one request per conversion */
    adc_enable_dma(ADC1);
    adc_enable_dma_circular_mode(ADC1); /* keep requesting after end of sequence */
}

/* ── SysTick setup ───────────────────────────────────────── */
static void systick_setup(void)
{
    /* 64 MHz / 1000 = 64000 ticks per ms */
    systick_set_reload(64000 - 1);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

/* ── ADC trigger helper ──────────────────────────────────── */
static void adc_start_scan(void)
{
    adc_start_conversion_regular(ADC1);
}

/* ── Wiper strip debounce ────────────────────────────────── */
static bool read_wiper_debounced(void)
{
    /* PA1 pulled high; wiper contact pulls to GND → active LOW */
    bool raw = (gpio_get(WIPER_PORT, WIPER_PIN) == 0);

    if (raw != s_wiper_raw_prev) {
        s_wiper_last_change_ms = now_ms();
        s_wiper_raw_prev       = raw;
    }

    if ((now_ms() - s_wiper_last_change_ms) >= WIPER_DEBOUNCE_MS) {
        return raw;
    }

    return s_state.wiper_active; /* return last stable state while bouncing */
}

/* ── Rail voltage estimate (integer mV) ─────────────────── */
static uint16_t rail_raw_to_mv(uint16_t raw)
{
    /* V_PA0 = raw / 4095 * 3300 mV
     * V_rail = V_PA0 / divider_ratio
     * divider_ratio = ADC_RAIL_DIVIDER_NUM / ADC_RAIL_DIVIDER_DEN
     * V_rail = (raw * 3300 * DEN) / (4095 * NUM)
     * All integer, no float.                                      */
    return (uint16_t)(((uint32_t)raw * 3300U * ADC_RAIL_DIVIDER_DEN)
                      / (4095U * ADC_RAIL_DIVIDER_NUM));
}

/* ── Public: initialise ──────────────────────────────────── */
void brake_init(void)
{
    s_state = (BrakeState_t){
        .mode        = MODE_SAFE_BRAKE,
        .brake_ccr   = BRAKE_CCR_HARD,
        .anti_ccr    = ANTI_CCR_OFF,
        .wiper_active = false,
    };
    s_wiper_raw_prev = false;

    clock_setup();
    gpio_setup();
    dma_setup();
    adc_setup();
    timer_setup();
    systick_setup();

    /* Start first ADC scan */
    adc_start_scan();
}

/* ── Public: 1ms control tick ────────────────────────────── */
void brake_tick(void)
{
    static uint32_t last_adc_ms = 0;

    /* Trigger a fresh ADC scan every 1ms */
    if ((now_ms() - last_adc_ms) >= 1U) {
        adc_start_scan();
        last_adc_ms = now_ms();
    }

    /* Update cached readings when DMA has completed */
    if (s_adc_ready) {
        s_state.pot_raw  = s_adc_buf[1];              /* CH3 = pot */
        s_state.rail_mv  = rail_raw_to_mv(s_adc_buf[0]); /* CH0 = rail */
        s_adc_ready      = false;
    }

    /* Rail undervoltage guard */
    if (s_state.rail_mv < RAIL_UNDERVOLTAGE_MV &&
        s_state.rail_mv > 0) {              /* 0 = not yet read */
        brake_force_safe();
        return;
    }

    /* Wiper strip state */
    s_state.wiper_active = read_wiper_debounced();

    if (!s_state.wiper_active) {
        /* Throttle active — be completely passive */
        if (s_state.mode != MODE_PASSTHROUGH) {
            set_brake_ccr(BRAKE_CCR_OFF);
            set_anti_ccr(ANTI_CCR_OFF);
            s_state.mode = MODE_PASSTHROUGH;
        }
        return;
    }

    /* Wiper contacted — throttle released */
    uint16_t pot = s_state.pot_raw;
    uint16_t lo  = ADC_CENTRE - ADC_DEADBAND;  /* 1898 */
    uint16_t hi  = ADC_CENTRE + ADC_DEADBAND;  /* 2198 */

    if (pot < lo) {
        /* ── BRAKING ─────────────────────────────────────
         * pot=0    → CCR=BRAKE_CCR_HARD (0)  → full short
         * pot=lo-1 → CCR=BRAKE_CCR_SOFT      → ~8Ω effective
         * Inverted: lower pot value → lower CCR → more braking */
        uint16_t ccr = map_u16(pot, 0, lo,
                               BRAKE_CCR_HARD, BRAKE_CCR_SOFT);
        set_brake_ccr(ccr);
        set_anti_ccr(ANTI_CCR_OFF);
        s_state.mode = MODE_BRAKING;

    } else if (pot > hi) {
        /* ── ANTIBRAKE ───────────────────────────────────
         * pot=hi+1   → CCR=0          (no injection)
         * pot=ADC_MAX → CCR=ANTI_CCR_MAX (~2V)            */
        uint16_t ccr = map_u16(pot, hi, ADC_MAX,
                               ANTI_CCR_OFF, ANTI_CCR_MAX);
        set_brake_ccr(BRAKE_CCR_OFF);
        set_anti_ccr(ccr);
        s_state.mode = MODE_ANTIBRAKE;

    } else {
        /* ── FREEWHEEL ───────────────────────────────────*/
        set_brake_ccr(BRAKE_CCR_OFF);
        set_anti_ccr(ANTI_CCR_OFF);
        s_state.mode = MODE_FREEWHEEL;
    }
}

/* ── Public: force safe state (call from any fault handler) ─ */
void brake_force_safe(void)
{
    /* Direct register write — no library overhead, ISR-safe */
    TIM3_CCR1            = BRAKE_CCR_HARD;  /* brake FET full on */
    TIM3_CCR2            = ANTI_CCR_OFF;    /* antibrake off */
    s_state.mode         = MODE_SAFE_BRAKE;
    s_state.brake_ccr    = BRAKE_CCR_HARD;
    s_state.anti_ccr     = ANTI_CCR_OFF;
}

/* ── Public: query mode ──────────────────────────────────── */
BrakeMode_t brake_get_mode(void)
{
    return s_state.mode;
}

/* ── ISR: DMA1 Channel 1 transfer complete ───────────────── */
void dma1_channel1_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, ADC_DMA_CHANNEL, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, ADC_DMA_CHANNEL, DMA_TCIF);
        s_adc_ready = true;
    }
}
