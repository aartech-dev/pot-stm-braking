/* ============================================================
 *  brake_module.c — AART Slot Car Braking Module  Rev 5
 *  Target  : STM32G041K6U6 (QFN-32)
 *  Library : libopencm3
 * ============================================================ */

#include "brake_module.h"
#include "uart_debug.h"

/* ── Private state ───────────────────────────────────────── */
static BrakeCtx_t        s_ctx;
static volatile uint32_t s_tick_ms   = 0;
static volatile bool     s_adc_ready = false;

/* DMA: [0]=WHITE(CH0) [1]=BLACK(CH1) [2]=pot(CH3) [3]=btn(CH5) */
static volatile uint16_t s_adc_buf[ADC_NUM_CHANNELS];

/* Button debounce */
static uint32_t s_btn_last_ms = 0;
static bool     s_btn_prev    = false;

/* LED blink */
static uint32_t s_led_last_ms = 0;
static uint8_t  s_led_blinks  = 0;

/* ── SysTick ─────────────────────────────────────────────── */
void brake_systick_isr(void) { s_tick_ms++; }
static uint32_t now_ms(void) { return s_tick_ms; }

/* ── Linear map ──────────────────────────────────────────── */
static uint16_t map_u16(uint16_t v,
                         uint16_t il, uint16_t ih,
                         uint16_t ol, uint16_t oh)
{
    if (v <= il) return ol;
    if (v >= ih) return oh;
    return (uint16_t)(ol + ((uint32_t)(v-il)*(oh-ol))/(ih-il));
}

/* ── CCR setters ─────────────────────────────────────────── */
static void set_brake_ccr(uint16_t ccr)
{
    if (ccr > PWM_ARR) ccr = PWM_ARR;
    timer_set_oc_value(TIM3, TIM_OC1, ccr);
    s_ctx.brake_ccr = ccr;
}

static void set_ka_ccr(uint16_t ccr)
{
    if (ccr > KA_CCR_MAX) ccr = KA_CCR_MAX;
    timer_set_oc_value(TIM3, TIM_OC2, ccr);
    s_ctx.ka_ccr = ccr;
}

/* ── ADC raw → millivolts ────────────────────────────────── */
static uint16_t raw_to_mv(uint16_t raw, uint16_t num, uint16_t den)
{
    return (uint16_t)(((uint32_t)raw * 3300U * den)
                      / (4095U * num));
}

/* ── Keep-alive CCR → millivolts ─────────────────────────── */
static uint16_t ka_ccr_to_mv(uint16_t ccr, uint16_t white_mv)
{
    return (uint16_t)(((uint32_t)ccr * white_mv) / (PWM_ARR + 1U));
}

/* ── Flash load ──────────────────────────────────────────── */
/* Record layout: [63:48]=ramp_ms [47:32]=ka_ccr [31:0]=magic */
static void flash_load(void)
{
    uint32_t magic   = *(volatile uint32_t *)(FLASH_SAVE_ADDR);
    uint32_t payload = *(volatile uint32_t *)(FLASH_SAVE_ADDR + 4U);

    if (magic == FLASH_MAGIC) {
        uint16_t ka_ccr  = (uint16_t)(payload & 0xFFFFU);
        uint16_t ramp_ms = (uint16_t)(payload >> 16U);

        s_ctx.saved_ka_ccr  = (ka_ccr  <= KA_CCR_MAX)  ? ka_ccr  : KA_CCR_OFF;
        s_ctx.saved_ramp_ms = (ramp_ms <= RAMP_MS_MAX)  ? ramp_ms : 0U;
    } else {
        s_ctx.saved_ka_ccr  = KA_CCR_OFF;
        s_ctx.saved_ramp_ms = 0U;
    }
}

/* ── Flash save ──────────────────────────────────────────── */
static void flash_save(uint16_t ka_ccr, uint16_t ramp_ms)
{
    if (ka_ccr  > KA_CCR_MAX)  ka_ccr  = KA_CCR_MAX;
    if (ramp_ms > RAMP_MS_MAX) ramp_ms = RAMP_MS_MAX;

    uint64_t record = ((uint64_t)ramp_ms << 48U)
                    | ((uint64_t)ka_ccr  << 32U)
                    | (uint64_t)FLASH_MAGIC;

    flash_unlock();
    flash_erase_page(FLASH_SAVE_PAGE);
    flash_program_double_word(FLASH_SAVE_ADDR, record);
    flash_lock();

    s_ctx.saved_ka_ccr  = ka_ccr;
    s_ctx.saved_ramp_ms = ramp_ms;
}

/* ── LED blink ───────────────────────────────────────────── */
static void led_on(void)  { gpio_set(LED_PORT, LED_PIN); }
static void led_off(void) { gpio_clear(LED_PORT, LED_PIN); }

static void led_blink_start(void)
{
    s_led_blinks  = LED_BLINK_COUNT * 2U;
    s_led_last_ms = now_ms();
    led_on();
}

static void led_blink_tick(void)
{
    if (!s_led_blinks) return;
    if ((now_ms() - s_led_last_ms) < LED_BLINK_MS) return;
    s_led_last_ms = now_ms();
    if (--s_led_blinks & 1U) { led_on(); } else { led_off(); }
}

/* ── Clock ───────────────────────────────────────────────── */
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

    /* PA0 WHITE, PA1 BLACK, PA3 pot, PA5 button — ADC analog */
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                    GPIO0 | GPIO1 | GPIO3 | GPIO5);

    /* PA2 toggle bit0, PA4 toggle bit1 — digital pull-up     */
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,
                    GPIO2 | GPIO4);

    /* PA6 TIM3_CH1, PA7 TIM3_CH2 — PWM AF1                  */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO6 | GPIO7);

    /* PB6 — status LED                                       */
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

    /* CH1 brake — INVERTED: CCR=0 → HIGH → Q1 ON → dead short */
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_low(TIM3, TIM_OC1);
    timer_enable_oc_preload(TIM3, TIM_OC1);
    timer_set_oc_value(TIM3, TIM_OC1, BRAKE_CCR_HARD);
    timer_enable_oc_output(TIM3, TIM_OC1);

    /* CH2 keep-alive — normal: CCR=0 → Q2 off                */
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM3, TIM_OC2);
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

    uint8_t channels[4] = { 0, 1, 3, 5 };
    adc_set_regular_sequence(ADC1, 4, channels);
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

/* ── Button debounce ─────────────────────────────────────── */
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

/* ── Toggle ──────────────────────────────────────────────── */
static ModeSelect_t read_toggle(void)
{
    bool t0 = (gpio_get(TOGGLE0_PORT, TOGGLE0_PIN) == 0U);
    bool t1 = (gpio_get(TOGGLE1_PORT, TOGGLE1_PIN) == 0U);
    if (!t0 && !t1) return MODE_A;
    if ( t0 && !t1) return MODE_B;
    if (!t0 &&  t1) return MODE_C;
    return MODE_B;
}

/* ── Brake hysteresis ────────────────────────────────────── */
static bool update_brake_active(bool cur, uint16_t black_mv)
{
    const DebugParams_t *p = uart_debug_get_params();
    if (black_mv < p->brake_enter_mv) return true;
    if (black_mv > p->brake_exit_mv)  return false;
    return cur;
}

/* ── Keep-alive suppression ──────────────────────────────── */
static bool ka_should_run(uint16_t black_mv, uint16_t white_mv)
{
    if (s_ctx.saved_ka_ccr == KA_CCR_OFF) return false;
    return (black_mv <= ka_ccr_to_mv(s_ctx.saved_ka_ccr, white_mv));
}

/* ── Exponential brake ramp ──────────────────────────────── */
/* Applies one tick of the exponential ramp toward target.
 * Returns true when target is reached (within 1 CCR count).
 *
 * The ramp works on the CCR value directly. Because the brake
 * FET polarity is INVERTED, a lower CCR = more braking. The
 * ramp therefore needs to decrease CCR from BRAKE_CCR_OFF
 * (PWM_ARR=3199, FET off) down toward the target CCR.
 *
 * Each tick: distance = current - target (always positive since
 * we are ramping downward). Step = distance * alpha / 256.
 * CCR decreases by step each tick.                           */
static bool ramp_tick(uint16_t target, uint16_t alpha)
{
    uint16_t cur = s_ctx.brake_ccr;

    if (cur <= target) {
        set_brake_ccr(target);
        return true;  /* reached */
    }

    uint32_t distance = cur - target;
    uint32_t step     = (distance * alpha) / 256U;

    /* Ensure at least 1 count of progress per tick             */
    if (step == 0U) step = 1U;

    uint32_t next = (step >= distance) ? target : (cur - step);
    set_brake_ccr((uint16_t)next);
    return (next <= target);
}

/* ============================================================
 *  Capture handler — shared by modes A/B and C
 *
 *  Mode A/B: pot dials ramp_ms (CCW=0ms, CW=200ms)
 *            on release: save ramp_ms (ka_ccr unchanged)
 *
 *  Mode C:   pot dials ka_ccr (CCW=0V, CW=2V)
 *            on release: save ka_ccr (ramp_ms unchanged)
 *
 *  Returns true while capture is active (caller should return).
 * ============================================================ */
static bool handle_capture(bool is_mode_c)
{
    if (s_ctx.btn_pressed) {
        s_ctx.capture_active = true;

        if (is_mode_c) {
            /* Dial keep-alive voltage */
            uint16_t ccr = map_u16(s_ctx.pot_raw, 0, ADC_MAX,
                                   KA_CCR_OFF, KA_CCR_MAX);
            set_brake_ccr(BRAKE_CCR_OFF);
            set_ka_ccr(ccr);
        } else {
            /* Dial ramp time — display on LED would be nice but
             * we just track the pot position here              */
            set_brake_ccr(BRAKE_CCR_OFF);
            set_ka_ccr(KA_CCR_OFF);
        }
        s_ctx.state = STATE_CAPTURE;
        return true;
    }

    if (s_ctx.capture_active) {
        s_ctx.capture_active = false;

        if (is_mode_c) {
            /* Save keep-alive CCR, preserve ramp_ms            */
            flash_save(s_ctx.ka_ccr, s_ctx.saved_ramp_ms);
        } else {
            /* Convert pot position to ramp_ms and save         */
            uint16_t ramp_ms = map_u16(s_ctx.pot_raw, 0, ADC_MAX,
                                       0, RAMP_MS_MAX);
            flash_save(s_ctx.saved_ka_ccr, ramp_ms);
        }
        led_blink_start();
    }
    return false;
}

/* ============================================================
 *  MODE A — Brushed motor, positive anti-brake
 *
 *  Brake trigger: V_BLACK < BRAKE_ENTER_MV
 *
 *  Brake region (CCW → centre):
 *    CCR ramps exponentially from BRAKE_CCR_OFF to target
 *    using saved_ramp_ms. Target = map(pot, 0→centre, soft→hard)
 *
 *  Anti-brake region (centre → CW):
 *    Q1 off, Q2 ramps 0→KA_CCR_MAX (~2V positive anti-brake)
 *    No ramp-in for anti-brake — takes effect immediately.
 * ============================================================ */
static void tick_mode_a(void)
{
    if (handle_capture(false)) return;

    if (!s_ctx.brake_active) {
        set_brake_ccr(BRAKE_CCR_OFF);
        set_ka_ccr(KA_CCR_OFF);
        s_ctx.state = STATE_PASSTHROUGH;
        return;
    }

    uint16_t pot = s_ctx.pot_raw;
    uint16_t lo  = ADC_CENTRE - ADC_DEADBAND;
    uint16_t hi  = ADC_CENTRE + ADC_DEADBAND;

    if (pot <= hi) {
        /* Reduced braking / dead short region */
        uint16_t target = (pot >= lo)
            ? BRAKE_CCR_HARD
            : map_u16(pot, 0, lo,
                      uart_debug_get_params()->brake_ccr_soft,
                      BRAKE_CCR_HARD);

        s_ctx.brake_ccr_target = target;
        set_ka_ccr(KA_CCR_OFF);

        uint16_t alpha = RAMP_MS_TO_ALPHA(s_ctx.saved_ramp_ms);
        bool done = ramp_tick(target, alpha);
        s_ctx.state = done ? STATE_BRAKING : STATE_RAMP_IN;

    } else {
        /* Positive anti-brake region */
        uint16_t ccr = map_u16(pot, hi, ADC_MAX,
                               KA_CCR_OFF, KA_CCR_MAX);
        set_brake_ccr(BRAKE_CCR_OFF);
        set_ka_ccr(ccr);
        s_ctx.state = STATE_ANTI_BRAKE;
    }
}

/* ============================================================
 *  MODE B — Brushed motor, reduced braking only
 *
 *  Full pot range → brake resistance with exponential ramp-in.
 *  CCW=8Ω, CW=dead short. No injection ever.
 * ============================================================ */
static void tick_mode_b(void)
{
    if (handle_capture(false)) return;

    if (!s_ctx.brake_active) {
        set_brake_ccr(BRAKE_CCR_OFF);
        set_ka_ccr(KA_CCR_OFF);
        s_ctx.state = STATE_PASSTHROUGH;
        return;
    }

    uint16_t target = map_u16(s_ctx.pot_raw, 0, ADC_MAX,
                              uart_debug_get_params()->brake_ccr_soft,
                              BRAKE_CCR_HARD);
    s_ctx.brake_ccr_target = target;
    set_ka_ccr(KA_CCR_OFF);

    uint16_t alpha = RAMP_MS_TO_ALPHA(s_ctx.saved_ramp_ms);
    bool done = ramp_tick(target, alpha);
    s_ctx.state = done ? STATE_BRAKING : STATE_RAMP_IN;
}

/* ============================================================
 *  MODE C — Brushless motor, eCom keep-alive
 *
 *  No ramp-in. Brushless ESC needs prompt braking.
 *  Keep-alive suppressed when V_BLACK > KA setpoint.
 * ============================================================ */
static void tick_mode_c(void)
{
    if (handle_capture(true)) return;

    /* Keep-alive suppression */
    if (ka_should_run(s_ctx.black_mv, s_ctx.white_mv)) {
        set_ka_ccr(s_ctx.saved_ka_ccr);
    } else {
        set_ka_ccr(KA_CCR_OFF);
    }

    if (s_ctx.brake_active) {
        uint16_t target = map_u16(s_ctx.pot_raw, 0, ADC_MAX,
                                  uart_debug_get_params()->brake_ccr_soft,
                                  BRAKE_CCR_HARD);
        set_brake_ccr(target);
        s_ctx.state = STATE_BRAKING;
    } else {
        set_brake_ccr(BRAKE_CCR_OFF);
        s_ctx.state = (s_ctx.ka_ccr > 0U)
                      ? STATE_KEEPALIVE_ONLY
                      : STATE_PASSTHROUGH;
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

    flash_load();
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
        s_ctx.white_mv = raw_to_mv(s_adc_buf[0], WHITE_DIV_NUM, WHITE_DIV_DEN);
        s_ctx.black_mv = raw_to_mv(s_adc_buf[1], BLACK_DIV_NUM, BLACK_DIV_DEN);
        s_ctx.pot_raw  = s_adc_buf[2];
        s_ctx.btn_raw  = s_adc_buf[3];
        s_adc_ready    = false;
    }

    /* Undervoltage guard */
    if (s_ctx.white_mv < RAIL_UNDERVOLTAGE_MV && s_ctx.white_mv > 0U) {
        brake_force_safe();
        return;
    }

    /* Brake hysteresis */
    s_ctx.brake_active = update_brake_active(s_ctx.brake_active,
                                              s_ctx.black_mv);

    /* When brake releases, reset ramp so next entry starts fresh */
    if (!s_ctx.brake_active && s_ctx.brake_ccr != BRAKE_CCR_OFF) {
        set_brake_ccr(BRAKE_CCR_OFF);
    }

    s_ctx.btn_pressed = read_btn();
    s_ctx.mode_sel    = read_toggle();

    switch (s_ctx.mode_sel) {
        case MODE_A: tick_mode_a(); break;
        case MODE_B: tick_mode_b(); break;
        case MODE_C: tick_mode_c(); break;
        default:     tick_mode_b(); break;
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
const BrakeCtx_t *brake_get_ctx(void) { return &s_ctx; }

/* ── ISR: DMA1 CH1 complete ──────────────────────────────── */
void dma1_channel1_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, ADC_DMA_CHANNEL, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, ADC_DMA_CHANNEL, DMA_TCIF);
        s_adc_ready = true;
    }
}
