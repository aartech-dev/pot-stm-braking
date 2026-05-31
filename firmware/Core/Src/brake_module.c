/* ============================================================
 *  brake_module.c — AART Slot Car Braking Module  Rev 3
 *  Target  : STM32G041K6U6 (QFN-32)
 *  Library : libopencm3
 * ============================================================ */

#include "brake_module.h"

/* ── Private state ───────────────────────────────────────── */
static BrakeCtx_t        s_ctx;
static volatile uint32_t s_tick_ms   = 0;
static volatile bool     s_adc_ready = false;

/* DMA buffer: [0]=rail(CH0)  [1]=pot(CH3)  [2]=button(CH5) */
static volatile uint16_t s_adc_buf[ADC_NUM_CHANNELS];

/* Debounce state */
static uint32_t s_wiper_last_ms = 0;
static uint32_t s_btn_last_ms   = 0;
static bool     s_wiper_prev    = false;
static bool     s_btn_prev      = false;

/* LED blink state */
static uint32_t s_led_last_ms   = 0;
static uint8_t  s_led_blinks    = 0;   /* counts down; 0 = idle */

/* ── SysTick ─────────────────────────────────────────────── */
void brake_systick_isr(void) { s_tick_ms++; }
static uint32_t now_ms(void) { return s_tick_ms; }

/* ── Linear map with clamping ────────────────────────────── */
static uint16_t map_u16(uint16_t v,
                         uint16_t il, uint16_t ih,
                         uint16_t ol, uint16_t oh)
{
    if (v <= il) return ol;
    if (v >= ih) return oh;
    return (uint16_t)(ol + ((uint32_t)(v - il) * (oh - ol)) / (ih - il));
}

/* ── CCR setters ─────────────────────────────────────────── */

/* Brake FET (Q1, N-ch low-side, TIM3_CH1 INVERTED polarity)
 *
 *  The key invariant: higher CCR = less braking.
 *
 *  BRAKE_CCR_HARD (0)       → output pin permanently HIGH
 *                            → GD1 drives Q1 gate HIGH
 *                            → Q1 fully on → dead short across motor
 *
 *  BRAKE_CCR_SOFT (2720)    → output PWM, ~85% off-time
 *                            → Q1 on ~15% of period
 *                            → ~8Ω effective (motor L integrates)
 *
 *  BRAKE_CCR_OFF (PWM_ARR)  → output pin permanently LOW
 *                            → GD1 drives Q1 gate LOW → Q1 off
 *
 *  Safe default: 10k pull-up on GD1 input holds input HIGH when
 *  MCU is unpowered → Q1 on → motor shorted = safe/braked state.  */
static void set_brake_ccr(uint16_t ccr)
{
    if (ccr > PWM_ARR) ccr = PWM_ARR;
    timer_set_oc_value(TIM3, TIM_OC1, ccr);
    s_ctx.brake_ccr = ccr;
}

/* Keep-alive / antibrake FET (Q2, P-ch high-side, TIM3_CH2 normal)
 *
 *  KA_CCR_OFF (0)     → pin LOW  → P-ch Q2 gate at 0V → Q2 off
 *  KA_CCR_MAX (774)   → PWM 774/3200 ≈ 24% duty
 *                     → effective voltage 0.24 × 12.4V ≈ 3V
 *
 *  Used for both antibrake injection (mode A) and
 *  eCom keep-alive (mode C). In mode C, runs continuously.
 *
 *  IMPORTANT: A Schottky diode in series with Q2 output is
 *  recommended. When Q1 is on (dead short) simultaneously,
 *  the diode prevents reverse current through Q2.             */
static void set_ka_ccr(uint16_t ccr)
{
    if (ccr > KA_CCR_MAX) ccr = KA_CCR_MAX;
    timer_set_oc_value(TIM3, TIM_OC2, ccr);
    s_ctx.ka_ccr = ccr;
}

/* ── Flash: load keep-alive CCR ──────────────────────────── */
static uint16_t flash_load_ka_ccr(void)
{
    uint32_t magic = *(volatile uint32_t *)(FLASH_SAVE_ADDR);
    uint32_t ccr   = *(volatile uint32_t *)(FLASH_SAVE_ADDR + 4U);
    if (magic == FLASH_MAGIC && ccr <= KA_CCR_MAX) {
        return (uint16_t)ccr;
    }
    return KA_CCR_OFF;
}

/* ── Flash: save keep-alive CCR ──────────────────────────── */
/* G0 flash requires 64-bit (double-word) aligned writes.
 * We pack magic (lower 32 bits) and ccr (upper 32 bits)
 * into one 64-bit write.                                    */
static void flash_save_ka_ccr(uint16_t ccr)
{
    flash_unlock();
    flash_erase_page(FLASH_SAVE_PAGE);
    flash_program_double_word(FLASH_SAVE_ADDR,
        ((uint64_t)ccr << 32) | (uint64_t)FLASH_MAGIC);
    flash_lock();
    s_ctx.saved_ka_ccr = ccr;
}

/* ── LED blink helpers ───────────────────────────────────── */
static void led_on(void)  { gpio_set(LED_PORT, LED_PIN); }
static void led_off(void) { gpio_clear(LED_PORT, LED_PIN); }

static void led_blink_start(void)
{
    s_led_blinks   = LED_BLINK_COUNT * 2U;  /* each count = one half-period */
    s_led_last_ms  = now_ms();
    led_on();
}

static void led_blink_tick(void)
{
    if (s_led_blinks == 0U) return;
    if ((now_ms() - s_led_last_ms) < LED_BLINK_MS) return;
    s_led_last_ms = now_ms();
    s_led_blinks--;
    if (s_led_blinks & 1U) { led_on(); } else { led_off(); }
}

/* ── Clock: HSI16 → PLL → 64 MHz ────────────────────────── */
static void clock_setup(void)
{
    const struct rcc_clock_scale cfg = {
        .sysclock_source  = RCC_PLL,
        .pll_source       = RCC_PLLCFGR_PLLSRC_HSI16,
        .pll_div          = RCC_PLLCFGR_PLLM_DIV(1),
        .pll_mul          = RCC_PLLCFGR_PLLN_MUL(8),
        .pllp_div         = RCC_PLLCFGR_PLLP_DIV(2),
        .pllq_div         = RCC_PLLCFGR_PLLQ_DIV(2),
        .pllr_div         = RCC_PLLCFGR_PLLR_DIV(2),
        .hpre             = RCC_CFGR_HPRE_NODIV,
        .ppre             = RCC_CFGR_PPRE_NODIV,
        .flash_waitstates = 2,
        .ahb_frequency    = 64000000,
        .apb_frequency    = 64000000,
        .voltage_scale    = PWR_SCALE1,
    };
    rcc_clock_setup(&cfg);
}

/* ── GPIO ────────────────────────────────────────────────── */
static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    /* PA0, PA3, PA5 — ADC analog inputs (rail, pot, button) */
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                    GPIO0 | GPIO3 | GPIO5);

    /* PA1 — wiper strip
     * PA2 — toggle bit 0
     * PA4 — toggle bit 1
     * All: digital input, pull-up, active LOW                */
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,
                    GPIO1 | GPIO2 | GPIO4);

    /* PA6 — TIM3_CH1 (brake PWM)
     * PA7 — TIM3_CH2 (keep-alive PWM)
     * AF1 on G0                                              */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO6 | GPIO7);

    /* PB6 — status LED */
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    led_off();
}

/* ── TIM3 PWM ────────────────────────────────────────────── */
static void timer_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_reset_pulse(RST_TIM3);

    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM3, 0);
    timer_set_period(TIM3, PWM_ARR);
    timer_enable_preload(TIM3);

    /* CH1 — brake — INVERTED: CCR=0 → pin HIGH → FET on (dead short) */
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_low(TIM3, TIM_OC1);   /* INVERTED */
    timer_enable_oc_preload(TIM3, TIM_OC1);
    timer_set_oc_value(TIM3, TIM_OC1, BRAKE_CCR_HARD); /* safe default on boot */
    timer_enable_oc_output(TIM3, TIM_OC1);

    /* CH2 — keep-alive / antibrake — normal polarity */
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM3, TIM_OC2);  /* normal */
    timer_enable_oc_preload(TIM3, TIM_OC2);
    timer_set_oc_value(TIM3, TIM_OC2, KA_CCR_OFF);
    timer_enable_oc_output(TIM3, TIM_OC2);

    timer_generate_event(TIM3, TIM_EGR_UG);
    timer_enable_counter(TIM3);
}

/* ── DMA ─────────────────────────────────────────────────── */
static void dma_setup(void)
{
    rcc_periph_clock_enable(RCC_DMA1);
    dma_channel_reset(DMA1, ADC_DMA_CHANNEL);

    MMIO32(0x40020800U) = ADC_DMAMUX_REQ;  /* DMAMUX1_C0CR → ADC1 */

    dma_set_peripheral_address(DMA1, ADC_DMA_CHANNEL,
                               (uint32_t)&ADC_DR(ADC1));
    dma_set_memory_address(DMA1, ADC_DMA_CHANNEL,
                           (uint32_t)s_adc_buf);
    dma_set_number_of_data(DMA1, ADC_DMA_CHANNEL, ADC_NUM_CHANNELS);
    dma_set_read_from_peripheral(DMA1, ADC_DMA_CHANNEL);
    dma_set_peripheral_size(DMA1, ADC_DMA_CHANNEL, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, ADC_DMA_CHANNEL, DMA_CCR_MSIZE_16BIT);
    dma_enable_memory_increment_mode(DMA1, ADC_DMA_CHANNEL);
    dma_enable_circular_mode(DMA1, ADC_DMA_CHANNEL);
    dma_enable_transfer_complete_interrupt(DMA1, ADC_DMA_CHANNEL);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
    nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 1);
    dma_enable_channel(DMA1, ADC_DMA_CHANNEL);
}

/* ── ADC ─────────────────────────────────────────────────── */
static void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC);
    adc_power_off(ADC1);
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_PCLK_DIV2);
    adc_calibrate(ADC1);
    adc_power_on(ADC1);
    while (!adc_is_power_off(ADC1) == false);

    adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
    adc_set_single_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMPx_039DOT5CYC);

    uint8_t channels[3] = { 0, 3, 5 };  /* rail, pot, button */
    adc_set_regular_sequence(ADC1, 3, channels);

    adc_enable_dma(ADC1);
    adc_enable_dma_circular_mode(ADC1);
}

/* ── SysTick ─────────────────────────────────────────────── */
static void systick_setup(void)
{
    systick_set_reload(64000U - 1U);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

/* ── Debounced input readers ─────────────────────────────── */
static bool read_wiper(void)
{
    bool raw = (gpio_get(WIPER_PORT, WIPER_PIN) == 0U);
    if (raw != s_wiper_prev) {
        s_wiper_last_ms = now_ms();
        s_wiper_prev    = raw;
    }
    if ((now_ms() - s_wiper_last_ms) >= WIPER_DEBOUNCE_MS) return raw;
    return s_ctx.wiper_active;
}

static bool read_btn(void)
{
    bool raw = (s_ctx.btn_raw < BTN_PRESSED_THRESHOLD);
    if (raw != s_btn_prev) {
        s_btn_last_ms = now_ms();
        s_btn_prev    = raw;
    }
    if ((now_ms() - s_btn_last_ms) >= BTN_DEBOUNCE_MS) return raw;
    return s_ctx.btn_pressed;
}

static ModeSelect_t read_toggle(void)
{
    bool t0 = (gpio_get(TOGGLE0_PORT, TOGGLE0_PIN) == 0U);
    bool t1 = (gpio_get(TOGGLE1_PORT, TOGGLE1_PIN) == 0U);
    if (!t0 && !t1) return MODE_SEL_A;
    if ( t0 && !t1) return MODE_SEL_B;
    if (!t0 &&  t1) return MODE_SEL_C;
    return MODE_SEL_B;  /* both asserted = invalid → safe */
}

/* ── Rail voltage ────────────────────────────────────────── */
static uint16_t rail_raw_to_mv(uint16_t raw)
{
    return (uint16_t)(((uint32_t)raw * 3300U * ADC_RAIL_DIVIDER_DEN)
                      / (4095U * ADC_RAIL_DIVIDER_NUM));
}

/* ============================================================
 *  MODE A — Brushed motor with antibrake
 *
 *  Pot mapping:
 *    0       → BRAKE_CCR_SOFT  (min brake ~8Ω)
 *    centre  → BRAKE_CCR_HARD  (dead short, max brake)
 *    past ctr→ Q1 off, Q2 ramps 0→3V (antibrake inject)
 *
 *  Centre detent sits AT maximum braking. There is no freewheel
 *  zone — the deadband around centre all maps to dead short.
 *  This makes the centre detent a natural "full brake" parking
 *  position before the pot enters antibrake territory.
 *
 *  Wiper must be contacted for any output.
 * ============================================================ */
static void tick_mode_a(void)
{
    if (!s_ctx.wiper_active) {
        set_brake_ccr(BRAKE_CCR_OFF);
        set_ka_ccr(KA_CCR_OFF);
        s_ctx.state = STATE_PASSTHROUGH;
        return;
    }

    uint16_t pot = s_ctx.pot_raw;
    uint16_t lo  = ADC_CENTRE - ADC_DEADBAND;   /* 1898 */
    uint16_t hi  = ADC_CENTRE + ADC_DEADBAND;   /* 2198 */

    if (pot <= hi) {
        /* Brake region: 0→centre all maps to brake
         * pot=0      → BRAKE_CCR_SOFT (~8Ω)
         * pot=lo..hi → BRAKE_CCR_HARD (dead short — deadband = full brake)
         * pot=lo     → interpolated between soft and hard                 */
        uint16_t ccr;
        if (pot >= lo) {
            ccr = BRAKE_CCR_HARD;   /* deadband = dead short */
        } else {
            ccr = map_u16(pot, 0, lo, BRAKE_CCR_SOFT, BRAKE_CCR_HARD);
        }
        set_brake_ccr(ccr);
        set_ka_ccr(KA_CCR_OFF);
        s_ctx.state = STATE_BRAKING;

    } else {
        /* Antibrake region: past centre CW
         * pot=hi+1   → KA_CCR_OFF   (0V)
         * pot=ADC_MAX → KA_CCR_MAX  (~3V)                                */
        uint16_t ccr = map_u16(pot, hi, ADC_MAX, KA_CCR_OFF, KA_CCR_MAX);
        set_brake_ccr(BRAKE_CCR_OFF);
        set_ka_ccr(ccr);
        s_ctx.state = STATE_ANTIBRAKE;
    }
}

/* ============================================================
 *  MODE B — Brushed motor, brake only
 *
 *  Full pot range maps to brake resistance:
 *    CCW (0)    → ~8Ω   (BRAKE_CCR_SOFT)
 *    CW  (4095) → dead short (BRAKE_CCR_HARD)
 *
 *  No injection. Q2 always off.
 *  Wiper must be contacted for any output.
 * ============================================================ */
static void tick_mode_b(void)
{
    if (!s_ctx.wiper_active) {
        set_brake_ccr(BRAKE_CCR_OFF);
        set_ka_ccr(KA_CCR_OFF);
        s_ctx.state = STATE_PASSTHROUGH;
        return;
    }

    uint16_t ccr = map_u16(s_ctx.pot_raw, 0, ADC_MAX,
                           BRAKE_CCR_SOFT, BRAKE_CCR_HARD);
    set_brake_ccr(ccr);
    set_ka_ccr(KA_CCR_OFF);
    s_ctx.state = STATE_BRAKING;
}

/* ============================================================
 *  MODE C — Brushless motor, eCom keep-alive
 *
 *  Q2 keep-alive runs CONTINUOUSLY at saved_ka_ccr,
 *  regardless of wiper state. This keeps the eCom MCU
 *  powered at all times — during throttle, during braking,
 *  during coasting.
 *
 *  When wiper contacts (brake condition):
 *    Q1 PWM provides brake resistance (same mapping as mode B)
 *    Q2 keep-alive continues in parallel
 *    NOTE: Schottky diode on Q2 output recommended to block
 *    reverse current when Q1 pulls the track to near-0V.
 *
 *  Capture mode (button held):
 *    Q1 off (brake suspended — safe to dial voltage)
 *    Q2 output follows pot: CCW=0V, CW=3V
 *    User adjusts until eCom wakes without motor spinning
 *    Release button → CCR saved to flash, LED blinks 3×
 * ============================================================ */
static void tick_mode_c(void)
{
    /* ── Button held: capture mode ───────────────────────── */
    if (s_ctx.btn_pressed) {
        s_ctx.capture_active = true;
        /* Pot dials keep-alive directly, brake off */
        uint16_t ccr = map_u16(s_ctx.pot_raw, 0, ADC_MAX,
                               KA_CCR_OFF, KA_CCR_MAX);
        set_brake_ccr(BRAKE_CCR_OFF);
        set_ka_ccr(ccr);
        s_ctx.state = STATE_CAPTURE;
        return;
    }

    /* ── Button just released: save to flash ─────────────── */
    if (s_ctx.capture_active) {
        s_ctx.capture_active = false;
        flash_save_ka_ccr(s_ctx.ka_ccr);
        led_blink_start();
        /* ka_ccr and saved_ka_ccr now agree — fall through to normal */
    }

    /* ── Keep-alive always present ───────────────────────── */
    set_ka_ccr(s_ctx.saved_ka_ccr);

    /* ── Wiper: add brake resistance on top ─────────────── */
    if (s_ctx.wiper_active) {
        uint16_t ccr = map_u16(s_ctx.pot_raw, 0, ADC_MAX,
                               BRAKE_CCR_SOFT, BRAKE_CCR_HARD);
        set_brake_ccr(ccr);
        s_ctx.state = STATE_BRAKING;
    } else {
        set_brake_ccr(BRAKE_CCR_OFF);
        s_ctx.state = STATE_KEEPALIVE_ONLY;
    }
}

/* ── Public: init ────────────────────────────────────────── */
void brake_init(void)
{
    s_ctx = (BrakeCtx_t){ .state = STATE_SAFE_BRAKE };

    clock_setup();
    gpio_setup();
    dma_setup();
    adc_setup();
    timer_setup();
    systick_setup();

    s_ctx.saved_ka_ccr = flash_load_ka_ccr();

    adc_start_conversion_regular(ADC1);
}

/* ── Public: 1ms tick ────────────────────────────────────── */
void brake_tick(void)
{
    static uint32_t last_adc_ms = 0;

    if ((now_ms() - last_adc_ms) >= 1U) {
        adc_start_conversion_regular(ADC1);
        last_adc_ms = now_ms();
    }

    if (s_adc_ready) {
        s_ctx.rail_mv = rail_raw_to_mv(s_adc_buf[0]);
        s_ctx.pot_raw = s_adc_buf[1];
        s_ctx.btn_raw = s_adc_buf[2];
        s_adc_ready   = false;
    }

    /* Undervoltage guard */
    if (s_ctx.rail_mv < RAIL_UNDERVOLTAGE_MV && s_ctx.rail_mv > 0U) {
        brake_force_safe();
        return;
    }

    s_ctx.wiper_active = read_wiper();
    s_ctx.btn_pressed  = read_btn();
    s_ctx.mode_sel     = read_toggle();

    switch (s_ctx.mode_sel) {
        case MODE_SEL_A: tick_mode_a(); break;
        case MODE_SEL_B: tick_mode_b(); break;
        case MODE_SEL_C: tick_mode_c(); break;
        default:         tick_mode_b(); break;
    }

    led_blink_tick();
}

/* ── Public: force safe brake (ISR-safe) ────────────────── */
void brake_force_safe(void)
{
    TIM3_CCR1       = BRAKE_CCR_HARD;
    TIM3_CCR2       = KA_CCR_OFF;
    s_ctx.state     = STATE_SAFE_BRAKE;
    s_ctx.brake_ccr = BRAKE_CCR_HARD;
    s_ctx.ka_ccr    = KA_CCR_OFF;
}

OpState_t    brake_get_state(void)    { return s_ctx.state; }
ModeSelect_t brake_get_mode_sel(void) { return s_ctx.mode_sel; }

/* ── ISR: DMA1 CH1 transfer complete ────────────────────── */
void dma1_channel1_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, ADC_DMA_CHANNEL, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, ADC_DMA_CHANNEL, DMA_TCIF);
        s_adc_ready = true;
    }
}
