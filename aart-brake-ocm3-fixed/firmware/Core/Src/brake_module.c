/* ============================================================
 *  brake_module.c — AART Slot Car Braking Module  Revision 11
 *  Target  : STM32G051K6U6 (QFN-32)
 *  Library : libopencm3
 *
 *  Output stage: DAC1 (PA4) -> Q2 brake (linear).
 *  PWM (PA5, TIM2_CH1) -> filter -> Q1 anti-brake CURRENT SOURCE (3A).
 *  Both FETs operated in linear (triode) region — no PWM.
 *  D_PG (LTC4412) in BLACK controller input path.
 *  R10/R11 BLACK sense on motor side of D_PG.
 * ============================================================ */

#include "brake_module.h"
#include "uart_debug.h"

/* ── Private state ───────────────────────────────────────── */
static BrakeCtx_t        s_ctx;
static volatile uint32_t s_tick_ms   = 0;
static volatile bool     s_adc_ready = false;

/* DMA: [0]=WHITE(CH0) [1]=BLACK(CH1) [2]=pot(CH3) [3]=btn(CH8) */
static volatile uint16_t s_adc_buf[ADC_NUM_CHANNELS];

static uint32_t s_btn_last_ms = 0;
static bool     s_btn_prev    = false;
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

/* ── DAC setters ─────────────────────────────────────────── */
static void set_brake_dac(uint16_t val)
{
    if (val > 4095U) val = 4095U;
    dac_load_data_buffer_single(DAC1, val, DAC_ALIGN_RIGHT, DAC_BRAKE_CHAN);
    dac_software_trigger(DAC1, DAC_BRAKE_CHAN);
    s_ctx.brake_dac = val;
}

/* Anti-brake demand is now a 50kHz PWM duty, not a DAC code.   */
/* The PWM is filtered to DC and sets the current-source ref;   */
/* the current source bounds injection to ~3A regardless of      */
/* motor impedance (a stalled motor on the line is ~a short).    */
static void set_ka_pwm(uint16_t duty)
{
    if (duty > KA_PWM_ARR) duty = KA_PWM_ARR;
    timer_set_oc_value(KA_PWM_TIMER, KA_PWM_OC, duty);
    s_ctx.ka_dac = duty;          /* field reused: stores PWM duty */
}
/* compat shim for existing call sites */
static inline void set_ka_dac(uint16_t val) { set_ka_pwm(val); }

/* ── ADC raw → millivolts ────────────────────────────────── */
static uint16_t raw_to_mv(uint16_t raw, uint16_t num, uint16_t den)
{
    return (uint16_t)(((uint32_t)raw * 3300U * den)
                      / (4095U * num));
}

/* ── Keep-alive floor: should Q1 run? ───────────────────── */
/* Q1 only activates when V_BLACK_to_track would drop below
 * the saved keep-alive DAC floor. Suppressed when controller
 * is delivering power above the floor voltage.              */
static bool ka_should_run(uint16_t black_mv, uint16_t white_mv)
{
    /* Convert saved ka_dac to an approximate voltage on track.
     * Q1 P-ch: gate at DAC voltage, source at WHITE rail.
     * Simple approximation: floor_mv = (1 - dac/4095) * white_mv */
    uint32_t floor_mv = ((uint32_t)(4095U - s_ctx.saved_ka_dac)
                         * white_mv) / 4095U;
    return black_mv < (uint16_t)floor_mv;
}

/* ── LED ─────────────────────────────────────────────────── */
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

/* ── Clock (64MHz for UART accuracy; DAC unaffected by clock) */
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

    /* PA0 WHITE sense, PA1 BLACK sense, PA3 pot — ADC analog   */
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                    GPIO0 | GPIO1 | GPIO3);

    /* PA4 DAC1 (Q2 brake) — analog                            */
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
    /* PA5 = TIM2_CH1 (AF2) — anti-brake current demand PWM    */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO5);

    /* PB0 toggle bit0, PB1 toggle bit1, PB3 capture btn        */
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,
                    GPIO0 | GPIO1 | GPIO3);

    /* PB6 status LED                                            */
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    led_off();
}

/* ── DAC setup ───────────────────────────────────────────── */
static void dac_setup(void)
{
    rcc_periph_clock_enable(RCC_DAC1);

    /* DAC1 CH1 only (brake). CH2 freed — anti-brake is PWM now. */
    dac_trigger_enable(DAC1, DAC_BRAKE_CHAN);
    dac_set_trigger_source(DAC1, DAC_CR_TSEL1_SW);  /* CH1 sw trigger */
    dac_enable(DAC1, DAC_BRAKE_CHAN);

    /* Safe default: Q2 brake off (0V). KA PWM set in ka_pwm_setup. */
    set_brake_dac(DAC_BRAKE_OFF);
}

/* ── Anti-brake demand PWM (TIM2_CH1, PA5 AF2) ──────────── */
/* 50kHz PWM, filtered to DC, sets the current-source demand.  */
/* Duty 0 = 0A (off); KA_PWM_ARR = ~3A. Bench-confirm the timer */
/* clock/prescaler so the period is 50kHz on the target board.  */
static void ka_pwm_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM2);
    timer_set_mode(KA_PWM_TIMER, TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(KA_PWM_TIMER, 0U);
    timer_set_period(KA_PWM_TIMER, KA_PWM_ARR);   /* 50kHz */
    timer_set_oc_mode(KA_PWM_TIMER, KA_PWM_OC, TIM_OCM_PWM1);
    timer_set_oc_value(KA_PWM_TIMER, KA_PWM_OC, KA_PWM_OFF); /* off */
    timer_enable_oc_output(KA_PWM_TIMER, KA_PWM_OC);
    timer_enable_preload(KA_PWM_TIMER);
    timer_enable_counter(KA_PWM_TIMER);
    s_ctx.ka_dac = KA_PWM_OFF;
}

/* ── Voltmeter PWM (TIM3_CH1, PB4 AF1) ──────────────────── */
static void voltmeter_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM3);

    /* PB4 → TIM3_CH1 AF1                                      */
    gpio_mode_setup(VOLTMETER_PWM_PORT, GPIO_MODE_AF,
                    GPIO_PUPD_NONE, VOLTMETER_PWM_PIN);
    gpio_set_af(VOLTMETER_PWM_PORT, GPIO_AF1, VOLTMETER_PWM_PIN);

    timer_set_mode(VOLTMETER_TIM, TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(VOLTMETER_TIM, VOLTMETER_TIM_PSC);
    timer_set_period(VOLTMETER_TIM, VOLTMETER_TIM_ARR);

    /* CH1 PWM mode 1, output enabled                          */
    timer_set_oc_mode(VOLTMETER_TIM, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_value(VOLTMETER_TIM, TIM_OC1, 0U);
    timer_enable_oc_output(VOLTMETER_TIM, TIM_OC1);
    timer_enable_preload(VOLTMETER_TIM);
    timer_enable_counter(VOLTMETER_TIM);
}

/* Update voltmeter display each 1ms tick.
 * Normal mode: display V_WHITE (track supply voltage).
 * Capture active: display anti-brake demand as equivalent voltage.*/
static void voltmeter_update(bool capture_active, uint16_t white_mv,
                              uint16_t preview_dac)
{
    uint32_t duty;
    if (capture_active) {
        /* Show live pot-derived setpoint during button hold.
         * preview_dac is the current capture_* field for this mode.
         * Invert: lower DAC → more conduction → higher display.  */
        uint32_t inverted = (uint32_t)(4095U - preview_dac);
        duty = (inverted * VOLTMETER_TIM_ARR) / 4095U;
    } else {
        /* Scale V_WHITE to 0–ARR. Cap at VOLTMETER_SCALE_MV.   */
        uint32_t mv = (white_mv > VOLTMETER_SCALE_MV)
                      ? VOLTMETER_SCALE_MV : white_mv;
        duty = (mv * VOLTMETER_TIM_ARR) / VOLTMETER_SCALE_MV;
    }
    if (duty > VOLTMETER_TIM_ARR) duty = VOLTMETER_TIM_ARR;
    timer_set_oc_value(VOLTMETER_TIM, TIM_OC1, (uint16_t)duty);
}
static void dma_setup(void)
{
    rcc_periph_clock_enable(RCC_DMA1);
    /* DMAMUX: route ADC1 request (id=5) to DMA1 CH1            */
    MMIO32(0x40020800U) = 5U;

    dma_channel_reset(DMA1, ADC_DMA_CHANNEL);
    dma_set_peripheral_address(DMA1, ADC_DMA_CHANNEL, (uint32_t)&ADC1_DR);
    dma_set_memory_address(DMA1, ADC_DMA_CHANNEL, (uint32_t)s_adc_buf);
    dma_set_number_of_data(DMA1, ADC_DMA_CHANNEL, ADC_NUM_CHANNELS);
    dma_set_read_from_peripheral(DMA1, ADC_DMA_CHANNEL);
    dma_enable_memory_increment_mode(DMA1, ADC_DMA_CHANNEL);
    dma_set_peripheral_size(DMA1, ADC_DMA_CHANNEL, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, ADC_DMA_CHANNEL, DMA_CCR_MSIZE_16BIT);
    dma_enable_circular_mode(DMA1, ADC_DMA_CHANNEL);
    dma_enable_transfer_complete_interrupt(DMA1, ADC_DMA_CHANNEL);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
    dma_enable_channel(DMA1, ADC_DMA_CHANNEL);
}

static void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_off(ADC1);

    /* Calibrate, then set up 4-channel scan                     */
    adc_calibrate(ADC1);

    ADC_CFGR1(ADC1) = ADC_CFGR1_RES_12_BIT | ADC_CFGR1_DMACFG |
                      ADC_CFGR1_DMAEN;
    ADC_SMPR(ADC1)  = ADC_SMPR_SMPx_039DOT5CYC;

    /* CH0=WHITE CH1=BLACK CH3=pot CH8=capture btn (PB0)        */
    ADC_CHSELR(ADC1) = (1U << 0) | (1U << 1) | (1U << 3) | (1U << 8);

    adc_power_on(ADC1);
    /* Wait for ADC ready */
    while (!(ADC_ISR(ADC1) & ADC_ISR_ADRDY)) {}
}

/* ── SysTick ─────────────────────────────────────────────── */
static void systick_setup(void)
{
    systick_set_frequency(1000, 64000000);
    systick_interrupt_enable();
    systick_counter_enable();
}

/* ── Brake hysteresis update ─────────────────────────────── */
static bool update_brake_active(bool cur, uint16_t black_mv)
{
    const DebugParams_t *p = uart_debug_get_params();
    if (black_mv < p->brake_enter_mv) return true;
    if (black_mv > p->brake_exit_mv)  return false;
    return cur;
}

/* ── Toggle and capture button read ─────────────────────── */
static ModeSelect_t read_toggle(void)
{
    bool b0 = !gpio_get(TOGGLE0_PORT, TOGGLE0_PIN);  /* LOW = asserted */
    bool b1 = !gpio_get(TOGGLE1_PORT, TOGGLE1_PIN);
    if (!b0 && !b1) return MODE_A;
    if ( b0 && !b1) return MODE_B;
    if (!b0 &&  b1) return MODE_C;
    return MODE_B;   /* invalid → safe default */
}

static bool read_btn(void)
{
    bool pressed = !gpio_get(CAPTURE_BTN_PORT, CAPTURE_BTN_PIN);
    if (pressed == s_btn_prev) {
        s_btn_last_ms = now_ms();
        s_btn_prev    = pressed;
        return pressed;
    }
    if ((now_ms() - s_btn_last_ms) >= BTN_DEBOUNCE_MS) {
        s_btn_prev    = pressed;
        s_btn_last_ms = now_ms();
    }
    return s_btn_prev;
}

/* ── Exponential ramp-in (brake entry) ───────────────────── */
/* ── Lookup tables ───────────────────────────────────────────
 * Used in Mode B (anti-brake) only.
 * Pot index 0 (CCW) = min brake + max anti-brake/injection
 * Pot index 255 (CW) = max brake + zero injection
 * Exponential shape (gamma=1.8) — fine control at soft end.
 *
 * In Mode A: pot maps LINEARLY to Q2 brake only (no table).
 * In Mode C: ka_dac_table used to show/save KA floor.        */
/* ── Configurable minimum brake resistance (non-linear set) ──
 * Five precomputed tables selected by SET BRAKE_OHMS. Nominal
 * full-scale resistances 2,3,4,6,8Ω (geometric-ish: finer steps
 * at the firm/low-ohm end). Higher DAC = higher Q2 gate voltage
 * = lower Rds = lower resistance. Labels are ESTIMATES; calibrate
 * per motor and regenerate via tools/gen_brake_tables.py.
 * No runtime table math: Mode B is a pure lookup; Mode A/C use
 * the selected soft endpoint in a single linear map.            */
static const uint16_t brake_ohms_label[BRAKE_OHMS_COUNT] = {
    25, 15, 5, 4, 3, 2, 1,
};

/* PROVISIONAL soft endpoints for IRFP250N — CALIBRATE on bench.   */
/* Old values were anchored to a wrong 8Ω≈1860 estimate (25Ω pots  */
/* are used on hardbody homeset 1/32 cars). Regenerate via         */
/* tools/gen_brake_tables.py after measuring.                      */
static const uint16_t brake_soft_dac_by_ohm[BRAKE_OHMS_COUNT] = {
    1700, 1780, 2000, 2080, 2150, 2250, 2400,
};

static const uint16_t brake_tables[BRAKE_OHMS_COUNT][256] = {
  /* ~25 ohm full scale (soft=1700, PROVISIONAL) */
  {
    1700, 1700, 1700, 1700, 1701, 1701, 1702, 1702, 1703, 1703, 1704, 1705, 1706, 1707, 1708, 1709,
    1710, 1711, 1712, 1713, 1714, 1716, 1717, 1718, 1720, 1721, 1723, 1725, 1726, 1728, 1730, 1731,
    1733, 1735, 1737, 1739, 1741, 1743, 1745, 1748, 1750, 1752, 1754, 1757, 1759, 1761, 1764, 1766,
    1769, 1772, 1774, 1777, 1780, 1783, 1785, 1788, 1791, 1794, 1797, 1800, 1803, 1806, 1809, 1813,
    1816, 1819, 1822, 1826, 1829, 1833, 1836, 1840, 1843, 1847, 1850, 1854, 1858, 1862, 1865, 1869,
    1873, 1877, 1881, 1885, 1889, 1893, 1897, 1901, 1906, 1910, 1914, 1918, 1923, 1927, 1931, 1936,
    1940, 1945, 1949, 1954, 1959, 1963, 1968, 1973, 1978, 1982, 1987, 1992, 1997, 2002, 2007, 2012,
    2017, 2022, 2028, 2033, 2038, 2043, 2048, 2054, 2059, 2065, 2070, 2076, 2081, 2087, 2092, 2098,
    2103, 2109, 2115, 2121, 2126, 2132, 2138, 2144, 2150, 2156, 2162, 2168, 2174, 2180, 2186, 2193,
    2199, 2205, 2211, 2218, 2224, 2230, 2237, 2243, 2250, 2256, 2263, 2269, 2276, 2283, 2289, 2296,
    2303, 2310, 2316, 2323, 2330, 2337, 2344, 2351, 2358, 2365, 2372, 2380, 2387, 2394, 2401, 2408,
    2416, 2423, 2430, 2438, 2445, 2453, 2460, 2468, 2475, 2483, 2491, 2498, 2506, 2514, 2521, 2529,
    2537, 2545, 2553, 2561, 2569, 2577, 2585, 2593, 2601, 2609, 2617, 2625, 2634, 2642, 2650, 2658,
    2667, 2675, 2684, 2692, 2700, 2709, 2718, 2726, 2735, 2743, 2752, 2761, 2769, 2778, 2787, 2796,
    2805, 2814, 2823, 2831, 2840, 2849, 2859, 2868, 2877, 2886, 2895, 2904, 2914, 2923, 2932, 2941,
    2951, 2960, 2970, 2979, 2989, 2998, 3008, 3017, 3027, 3036, 3046, 3056, 3066, 3075, 3085, 3095,
  },
  /* ~15 ohm full scale (soft=1780, PROVISIONAL) */
  {
    1780, 1780, 1780, 1780, 1781, 1781, 1782, 1782, 1783, 1783, 1784, 1785, 1785, 1786, 1787, 1788,
    1789, 1790, 1791, 1792, 1793, 1795, 1796, 1797, 1799, 1800, 1802, 1803, 1805, 1806, 1808, 1810,
    1811, 1813, 1815, 1817, 1819, 1821, 1823, 1825, 1827, 1829, 1831, 1833, 1836, 1838, 1840, 1843,
    1845, 1848, 1850, 1853, 1855, 1858, 1860, 1863, 1866, 1869, 1871, 1874, 1877, 1880, 1883, 1886,
    1889, 1892, 1895, 1899, 1902, 1905, 1908, 1912, 1915, 1918, 1922, 1925, 1929, 1932, 1936, 1940,
    1943, 1947, 1951, 1954, 1958, 1962, 1966, 1970, 1974, 1978, 1982, 1986, 1990, 1994, 1998, 2002,
    2007, 2011, 2015, 2019, 2024, 2028, 2033, 2037, 2042, 2046, 2051, 2055, 2060, 2065, 2070, 2074,
    2079, 2084, 2089, 2094, 2099, 2104, 2109, 2114, 2119, 2124, 2129, 2134, 2139, 2144, 2150, 2155,
    2160, 2166, 2171, 2176, 2182, 2187, 2193, 2199, 2204, 2210, 2215, 2221, 2227, 2233, 2238, 2244,
    2250, 2256, 2262, 2268, 2274, 2280, 2286, 2292, 2298, 2304, 2311, 2317, 2323, 2329, 2336, 2342,
    2348, 2355, 2361, 2368, 2374, 2381, 2387, 2394, 2400, 2407, 2414, 2421, 2427, 2434, 2441, 2448,
    2455, 2462, 2469, 2475, 2482, 2490, 2497, 2504, 2511, 2518, 2525, 2532, 2540, 2547, 2554, 2562,
    2569, 2576, 2584, 2591, 2599, 2606, 2614, 2622, 2629, 2637, 2645, 2652, 2660, 2668, 2676, 2683,
    2691, 2699, 2707, 2715, 2723, 2731, 2739, 2747, 2755, 2764, 2772, 2780, 2788, 2796, 2805, 2813,
    2821, 2830, 2838, 2847, 2855, 2864, 2872, 2881, 2889, 2898, 2907, 2915, 2924, 2933, 2941, 2950,
    2959, 2968, 2977, 2986, 2995, 3004, 3013, 3022, 3031, 3040, 3049, 3058, 3067, 3076, 3086, 3095,
  },
  /* ~5 ohm full scale (soft=2000, PROVISIONAL) */
  {
    2000, 2000, 2000, 2000, 2001, 2001, 2001, 2002, 2002, 2003, 2003, 2004, 2004, 2005, 2006, 2007,
    2007, 2008, 2009, 2010, 2011, 2012, 2013, 2014, 2016, 2017, 2018, 2019, 2021, 2022, 2023, 2025,
    2026, 2028, 2029, 2031, 2032, 2034, 2036, 2037, 2039, 2041, 2043, 2044, 2046, 2048, 2050, 2052,
    2054, 2056, 2058, 2060, 2063, 2065, 2067, 2069, 2072, 2074, 2076, 2079, 2081, 2083, 2086, 2088,
    2091, 2094, 2096, 2099, 2101, 2104, 2107, 2110, 2112, 2115, 2118, 2121, 2124, 2127, 2130, 2133,
    2136, 2139, 2142, 2145, 2148, 2152, 2155, 2158, 2161, 2165, 2168, 2171, 2175, 2178, 2182, 2185,
    2189, 2192, 2196, 2199, 2203, 2207, 2210, 2214, 2218, 2222, 2226, 2229, 2233, 2237, 2241, 2245,
    2249, 2253, 2257, 2261, 2265, 2269, 2274, 2278, 2282, 2286, 2290, 2295, 2299, 2303, 2308, 2312,
    2317, 2321, 2326, 2330, 2335, 2339, 2344, 2349, 2353, 2358, 2363, 2367, 2372, 2377, 2382, 2387,
    2391, 2396, 2401, 2406, 2411, 2416, 2421, 2426, 2431, 2437, 2442, 2447, 2452, 2457, 2463, 2468,
    2473, 2479, 2484, 2489, 2495, 2500, 2506, 2511, 2517, 2522, 2528, 2533, 2539, 2545, 2550, 2556,
    2562, 2568, 2573, 2579, 2585, 2591, 2597, 2603, 2609, 2615, 2621, 2627, 2633, 2639, 2645, 2651,
    2657, 2663, 2669, 2676, 2682, 2688, 2694, 2701, 2707, 2714, 2720, 2726, 2733, 2739, 2746, 2752,
    2759, 2765, 2772, 2779, 2785, 2792, 2799, 2805, 2812, 2819, 2826, 2833, 2839, 2846, 2853, 2860,
    2867, 2874, 2881, 2888, 2895, 2902, 2909, 2917, 2924, 2931, 2938, 2945, 2953, 2960, 2967, 2974,
    2982, 2989, 2997, 3004, 3011, 3019, 3026, 3034, 3041, 3049, 3057, 3064, 3072, 3080, 3087, 3095,
  },
  /* ~4 ohm full scale (soft=2080, PROVISIONAL) */
  {
    2080, 2080, 2080, 2080, 2081, 2081, 2081, 2082, 2082, 2082, 2083, 2084, 2084, 2085, 2085, 2086,
    2087, 2088, 2089, 2089, 2090, 2091, 2092, 2093, 2094, 2096, 2097, 2098, 2099, 2100, 2102, 2103,
    2104, 2106, 2107, 2108, 2110, 2111, 2113, 2115, 2116, 2118, 2119, 2121, 2123, 2125, 2127, 2128,
    2130, 2132, 2134, 2136, 2138, 2140, 2142, 2144, 2146, 2148, 2151, 2153, 2155, 2157, 2160, 2162,
    2164, 2167, 2169, 2172, 2174, 2177, 2179, 2182, 2184, 2187, 2189, 2192, 2195, 2198, 2200, 2203,
    2206, 2209, 2212, 2215, 2218, 2220, 2223, 2226, 2230, 2233, 2236, 2239, 2242, 2245, 2248, 2252,
    2255, 2258, 2262, 2265, 2268, 2272, 2275, 2279, 2282, 2286, 2289, 2293, 2296, 2300, 2303, 2307,
    2311, 2315, 2318, 2322, 2326, 2330, 2334, 2337, 2341, 2345, 2349, 2353, 2357, 2361, 2365, 2369,
    2374, 2378, 2382, 2386, 2390, 2395, 2399, 2403, 2407, 2412, 2416, 2421, 2425, 2429, 2434, 2438,
    2443, 2447, 2452, 2457, 2461, 2466, 2471, 2475, 2480, 2485, 2489, 2494, 2499, 2504, 2509, 2514,
    2519, 2524, 2529, 2534, 2539, 2544, 2549, 2554, 2559, 2564, 2569, 2574, 2580, 2585, 2590, 2595,
    2601, 2606, 2611, 2617, 2622, 2628, 2633, 2639, 2644, 2650, 2655, 2661, 2666, 2672, 2678, 2683,
    2689, 2695, 2700, 2706, 2712, 2718, 2724, 2730, 2735, 2741, 2747, 2753, 2759, 2765, 2771, 2777,
    2783, 2790, 2796, 2802, 2808, 2814, 2820, 2827, 2833, 2839, 2845, 2852, 2858, 2865, 2871, 2877,
    2884, 2890, 2897, 2903, 2910, 2916, 2923, 2930, 2936, 2943, 2950, 2956, 2963, 2970, 2976, 2983,
    2990, 2997, 3004, 3011, 3018, 3024, 3031, 3038, 3045, 3052, 3059, 3067, 3074, 3081, 3088, 3095,
  },
  /* ~3 ohm full scale (soft=2150, PROVISIONAL) */
  {
    2150, 2150, 2150, 2150, 2151, 2151, 2151, 2151, 2152, 2152, 2153, 2153, 2154, 2154, 2155, 2156,
    2156, 2157, 2158, 2159, 2160, 2161, 2161, 2162, 2163, 2164, 2166, 2167, 2168, 2169, 2170, 2171,
    2173, 2174, 2175, 2176, 2178, 2179, 2181, 2182, 2184, 2185, 2187, 2188, 2190, 2192, 2193, 2195,
    2197, 2199, 2200, 2202, 2204, 2206, 2208, 2210, 2212, 2214, 2216, 2218, 2220, 2222, 2224, 2226,
    2228, 2231, 2233, 2235, 2238, 2240, 2242, 2245, 2247, 2249, 2252, 2254, 2257, 2259, 2262, 2265,
    2267, 2270, 2273, 2275, 2278, 2281, 2284, 2286, 2289, 2292, 2295, 2298, 2301, 2304, 2307, 2310,
    2313, 2316, 2319, 2322, 2325, 2328, 2332, 2335, 2338, 2341, 2345, 2348, 2351, 2355, 2358, 2361,
    2365, 2368, 2372, 2375, 2379, 2382, 2386, 2390, 2393, 2397, 2401, 2404, 2408, 2412, 2416, 2419,
    2423, 2427, 2431, 2435, 2439, 2443, 2447, 2451, 2455, 2459, 2463, 2467, 2471, 2475, 2479, 2484,
    2488, 2492, 2496, 2501, 2505, 2509, 2514, 2518, 2522, 2527, 2531, 2536, 2540, 2545, 2549, 2554,
    2558, 2563, 2568, 2572, 2577, 2582, 2586, 2591, 2596, 2601, 2605, 2610, 2615, 2620, 2625, 2630,
    2635, 2640, 2645, 2650, 2655, 2660, 2665, 2670, 2675, 2680, 2686, 2691, 2696, 2701, 2706, 2712,
    2717, 2722, 2728, 2733, 2738, 2744, 2749, 2755, 2760, 2766, 2771, 2777, 2782, 2788, 2794, 2799,
    2805, 2811, 2816, 2822, 2828, 2834, 2839, 2845, 2851, 2857, 2863, 2869, 2874, 2880, 2886, 2892,
    2898, 2904, 2910, 2916, 2923, 2929, 2935, 2941, 2947, 2953, 2960, 2966, 2972, 2978, 2985, 2991,
    2997, 3004, 3010, 3016, 3023, 3029, 3036, 3042, 3049, 3055, 3062, 3068, 3075, 3082, 3088, 3095,
  },
  /* ~2 ohm full scale (soft=2250, PROVISIONAL) */
  {
    2250, 2250, 2250, 2250, 2250, 2251, 2251, 2251, 2252, 2252, 2252, 2253, 2253, 2254, 2255, 2255,
    2256, 2256, 2257, 2258, 2259, 2259, 2260, 2261, 2262, 2263, 2264, 2265, 2266, 2267, 2268, 2269,
    2270, 2271, 2272, 2274, 2275, 2276, 2277, 2279, 2280, 2281, 2283, 2284, 2286, 2287, 2289, 2290,
    2292, 2293, 2295, 2297, 2298, 2300, 2302, 2303, 2305, 2307, 2309, 2311, 2312, 2314, 2316, 2318,
    2320, 2322, 2324, 2326, 2328, 2330, 2332, 2335, 2337, 2339, 2341, 2343, 2346, 2348, 2350, 2353,
    2355, 2357, 2360, 2362, 2364, 2367, 2369, 2372, 2374, 2377, 2380, 2382, 2385, 2388, 2390, 2393,
    2396, 2398, 2401, 2404, 2407, 2410, 2412, 2415, 2418, 2421, 2424, 2427, 2430, 2433, 2436, 2439,
    2442, 2445, 2448, 2452, 2455, 2458, 2461, 2464, 2468, 2471, 2474, 2477, 2481, 2484, 2488, 2491,
    2494, 2498, 2501, 2505, 2508, 2512, 2515, 2519, 2523, 2526, 2530, 2533, 2537, 2541, 2545, 2548,
    2552, 2556, 2560, 2564, 2567, 2571, 2575, 2579, 2583, 2587, 2591, 2595, 2599, 2603, 2607, 2611,
    2615, 2619, 2623, 2628, 2632, 2636, 2640, 2644, 2649, 2653, 2657, 2662, 2666, 2670, 2675, 2679,
    2684, 2688, 2692, 2697, 2701, 2706, 2710, 2715, 2720, 2724, 2729, 2734, 2738, 2743, 2748, 2752,
    2757, 2762, 2767, 2771, 2776, 2781, 2786, 2791, 2796, 2801, 2806, 2811, 2815, 2820, 2826, 2831,
    2836, 2841, 2846, 2851, 2856, 2861, 2866, 2872, 2877, 2882, 2887, 2893, 2898, 2903, 2908, 2914,
    2919, 2925, 2930, 2935, 2941, 2946, 2952, 2957, 2963, 2968, 2974, 2979, 2985, 2991, 2996, 3002,
    3008, 3013, 3019, 3025, 3031, 3036, 3042, 3048, 3054, 3060, 3065, 3071, 3077, 3083, 3089, 3095,
  },
  /* ~1 ohm full scale (soft=2400, PROVISIONAL) */
  {
    2400, 2400, 2400, 2400, 2400, 2401, 2401, 2401, 2401, 2402, 2402, 2402, 2403, 2403, 2404, 2404,
    2405, 2405, 2406, 2406, 2407, 2408, 2408, 2409, 2410, 2411, 2411, 2412, 2413, 2414, 2415, 2416,
    2417, 2418, 2418, 2419, 2420, 2422, 2423, 2424, 2425, 2426, 2427, 2428, 2429, 2431, 2432, 2433,
    2434, 2436, 2437, 2438, 2440, 2441, 2443, 2444, 2445, 2447, 2448, 2450, 2451, 2453, 2455, 2456,
    2458, 2459, 2461, 2463, 2464, 2466, 2468, 2470, 2471, 2473, 2475, 2477, 2479, 2481, 2482, 2484,
    2486, 2488, 2490, 2492, 2494, 2496, 2498, 2500, 2502, 2504, 2507, 2509, 2511, 2513, 2515, 2518,
    2520, 2522, 2524, 2527, 2529, 2531, 2534, 2536, 2538, 2541, 2543, 2546, 2548, 2551, 2553, 2556,
    2558, 2561, 2563, 2566, 2568, 2571, 2574, 2576, 2579, 2582, 2584, 2587, 2590, 2593, 2595, 2598,
    2601, 2604, 2607, 2610, 2612, 2615, 2618, 2621, 2624, 2627, 2630, 2633, 2636, 2639, 2642, 2645,
    2648, 2652, 2655, 2658, 2661, 2664, 2667, 2671, 2674, 2677, 2680, 2684, 2687, 2690, 2694, 2697,
    2700, 2704, 2707, 2711, 2714, 2717, 2721, 2724, 2728, 2731, 2735, 2739, 2742, 2746, 2749, 2753,
    2757, 2760, 2764, 2768, 2771, 2775, 2779, 2782, 2786, 2790, 2794, 2798, 2802, 2805, 2809, 2813,
    2817, 2821, 2825, 2829, 2833, 2837, 2841, 2845, 2849, 2853, 2857, 2861, 2865, 2869, 2873, 2877,
    2882, 2886, 2890, 2894, 2898, 2903, 2907, 2911, 2916, 2920, 2924, 2928, 2933, 2937, 2942, 2946,
    2950, 2955, 2959, 2964, 2968, 2973, 2977, 2982, 2986, 2991, 2995, 3000, 3005, 3009, 3014, 3018,
    3023, 3028, 3033, 3037, 3042, 3047, 3051, 3056, 3061, 3066, 3071, 3075, 3080, 3085, 3090, 3095,
  },
};

/* Anti-brake demand table: value = PWM duty -> current (0..~3A).  */
/* idx 0 = max current, fades to 0 (off) by ~65% travel.          */
/* (filtered PWM sets the current-source reference; see doc 7A.7)  */
static const uint16_t ka_dac_table[256] = {
    2480, 2480, 2479, 2478, 2477, 2475, 2474, 2472, 2469, 2467, 2464, 2461, 2458, 2455, 2451, 2447,
    2443, 2439, 2435, 2430, 2425, 2420, 2415, 2409, 2404, 2398, 2392, 2386, 2379, 2373, 2366, 2359,
    2352, 2345, 2337, 2329, 2322, 2314, 2305, 2297, 2289, 2280, 2271, 2262, 2253, 2243, 2234, 2224,
    2214, 2204, 2194, 2184, 2173, 2162, 2151, 2140, 2129, 2118, 2106, 2095, 2083, 2071, 2059, 2046,
    2034, 2021, 2009, 1996, 1983, 1969, 1956, 1942, 1929, 1915, 1901, 1887, 1872, 1858, 1843, 1828,
    1813, 1798, 1783, 1768, 1752, 1737, 1721, 1705, 1689, 1672, 1656, 1640, 1623, 1606, 1589, 1572,
    1555, 1537, 1520, 1502, 1484, 1466, 1448, 1430, 1411, 1393, 1374, 1355, 1336, 1317, 1298, 1278,
    1259, 1239, 1219, 1199, 1179, 1159, 1138, 1118, 1097, 1076, 1055, 1034, 1013,  992,  970,  949,
     927,  905,  883,  861,  838,  816,  793,  771,  748,  725,  702,  678,  655,  631,  608,  584,
     560,  536,  512,  487,  463,  438,  414,  389,  364,  339,  313,  288,  262,  237,  211,  185,
     159,  133,  107,   80,   54,   27,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
};

/* ── Flash load ──────────────────────────────────────────── */
static void flash_load(void)
{
    uint32_t magic = *(volatile uint32_t *)(FLASH_SAVE_ADDR);
    if (magic != FLASH_MAGIC) {
        s_ctx.saved_ka_dac       = DEFAULT_KA_DAC;
        s_ctx.saved_ramp_ms      = 0U;
        s_ctx.saved_release_ms_a = 0U;
        s_ctx.saved_release_ms_b = 0U;
        s_ctx.saved_release_ms_c = 0U;
        return;
    }
    uint32_t w0hi = *(volatile uint32_t *)(FLASH_SAVE_ADDR + 4U);
    uint16_t ka   = (uint16_t)(w0hi & 0xFFFFU);
    uint16_t ramp = (uint16_t)(w0hi >> 16U);

    uint32_t w1hi = *(volatile uint32_t *)(FLASH_SAVE_ADDR + 12U);
    uint16_t b_enter = (uint16_t)(w1hi & 0xFFFFU);
    uint16_t b_exit  = (uint16_t)(w1hi >> 16U);

    uint32_t w2hi = *(volatile uint32_t *)(FLASH_SAVE_ADDR + 20U);
    uint16_t b_ohms  = (uint16_t)(w2hi & 0xFFFFU);
    uint16_t latvian = (uint16_t)(w2hi >> 16U);

    uint32_t w3lo = *(volatile uint32_t *)(FLASH_SAVE_ADDR + 28U);
    uint32_t w3hi = *(volatile uint32_t *)(FLASH_SAVE_ADDR + 32U);
    uint16_t rel_a = (uint16_t)(w3lo & 0xFFFFU);
    uint16_t rel_b = (uint16_t)(w3hi & 0xFFFFU);
    uint16_t rel_c = (uint16_t)(w3hi >> 16U);

    s_ctx.saved_ka_dac       = (ka   <= 4095U)           ? ka   : DEFAULT_KA_DAC;
    s_ctx.saved_ramp_ms      = (ramp <= RAMP_MS_MAX)     ? ramp : 0U;
    s_ctx.saved_release_ms_a = (rel_a <= RELEASE_MS_MAX) ? rel_a : 0U;
    s_ctx.saved_release_ms_b = (rel_b <= RELEASE_MS_MAX) ? rel_b : 0U;
    s_ctx.saved_release_ms_c = (rel_c <= RELEASE_MS_MAX) ? rel_c : 0U;

    uart_debug_load_params(b_enter, b_exit, b_ohms, latvian,
                           rel_a, rel_b, rel_c);
}

void flash_save_all(uint16_t ka_dac,
                    uint16_t ramp_ms,
                    uint16_t b_enter,
                    uint16_t b_exit,
                    uint16_t b_ohms_idx,
                    uint16_t latvian,
                    uint16_t rel_ms_a,
                    uint16_t rel_ms_b,
                    uint16_t rel_ms_c)
{
    if (ka_dac > 4095U)             ka_dac   = DEFAULT_KA_DAC;
    if (ramp_ms > RAMP_MS_MAX)      ramp_ms  = 0U;
    if (b_ohms_idx >= BRAKE_OHMS_COUNT) b_ohms_idx = BRAKE_OHMS_DEFAULT;
    if (rel_ms_a > RELEASE_MS_MAX)  rel_ms_a = 0U;
    if (rel_ms_b > RELEASE_MS_MAX)  rel_ms_b = 0U;
    if (rel_ms_c > RELEASE_MS_MAX)  rel_ms_c = 0U;

    flash_unlock();
    flash_erase_page(FLASH_SAVE_PAGE);
    flash_program_double_word(FLASH_SAVE_ADDR + 0x00U,
        ((uint64_t)ramp_ms << 48U) | ((uint64_t)ka_dac << 32U) | FLASH_MAGIC);
    flash_program_double_word(FLASH_SAVE_ADDR + 0x08U,
        ((uint64_t)0xFFFFFFFFUL << 32U) | ((uint64_t)b_exit << 16U) | b_enter);
    flash_program_double_word(FLASH_SAVE_ADDR + 0x10U,
        ((uint64_t)0xFFFFFFFFUL << 32U) | ((uint64_t)latvian << 16U) | b_ohms_idx);
    flash_program_double_word(FLASH_SAVE_ADDR + 0x18U,
        ((uint64_t)rel_ms_c << 48U) | ((uint64_t)rel_ms_b << 32U) |
        ((uint64_t)0xFFFFU  << 16U) | rel_ms_a);
    flash_lock();

    s_ctx.saved_ka_dac       = ka_dac;
    s_ctx.saved_ramp_ms      = ramp_ms;
    s_ctx.saved_release_ms_a = rel_ms_a;
    s_ctx.saved_release_ms_b = rel_ms_b;
    s_ctx.saved_release_ms_c = rel_ms_c;
}

static void do_save(void)
{
    const DebugParams_t *p = uart_debug_get_params();
    flash_save_all(s_ctx.saved_ka_dac, s_ctx.saved_ramp_ms,
                   p->brake_enter_mv, p->brake_exit_mv,
                   p->brake_soft_dac, p->latvian_dvdt_mv_per_ms,
                   s_ctx.saved_release_ms_a,
                   s_ctx.saved_release_ms_b,
                   s_ctx.saved_release_ms_c);
}

/* ── Ramp tick ───────────────────────────────────────────── */
static bool ramp_tick(uint16_t target, uint16_t alpha)
{
    uint16_t cur = s_ctx.brake_dac;
    if (alpha >= RAMP_ALPHA_INSTANT || cur == target) {
        set_brake_dac(target); return true;
    }
    int32_t err  = (int32_t)target - (int32_t)cur;
    int32_t step = (err * (int32_t)alpha) / 256;
    if (step == 0) step = (err > 0) ? 1 : -1;
    set_brake_dac((uint16_t)((int32_t)cur + step));
    return false;
}

/* ── Release ramp ────────────────────────────────────────── */
static bool release_ramp_tick(uint16_t release_ms)
{
    if (s_ctx.black_mv > (BRAKE_EXIT_MV + RELEASE_SNAP_MV)) {
        set_brake_dac(DAC_BRAKE_OFF); return true;
    }
    if (release_ms == 0U || s_ctx.brake_dac == DAC_BRAKE_OFF) {
        set_brake_dac(DAC_BRAKE_OFF); return true;
    }
    uint32_t step = ((uint32_t)DAC_BRAKE_HARD + release_ms - 1U) / release_ms;
    if (step == 0U) step = 1U;
    if (s_ctx.brake_dac <= (uint16_t)step) {
        set_brake_dac(DAC_BRAKE_OFF); return true;
    }
    set_brake_dac(s_ctx.brake_dac - (uint16_t)step);
    return false;
}

/* ── Active brake table selection ────────────────────────────
 * SET BRAKE_OHMS <1-5> picks one of five precomputed tables.
 * No runtime table math: the chosen table and its soft endpoint
 * are used directly. Higher gate voltage = lower Rds = lower
 * resistance. The 1..5Ω labels are nominal; calibrate per motor.  */
static uint8_t active_ohms_idx(void)
{
    uint8_t idx = uart_debug_get_params()->brake_ohms_idx;
    if (idx >= BRAKE_OHMS_COUNT) idx = BRAKE_OHMS_DEFAULT;
    return idx;
}

/* Public helpers for the UART layer (label table lives here). */
uint16_t brake_ohms_label_at(uint8_t idx)
{
    if (idx >= BRAKE_OHMS_COUNT) idx = BRAKE_OHMS_DEFAULT;
    return brake_ohms_label[idx];
}

/* Map a requested resistance (Ω) to the nearest stored table index. */
uint8_t brake_ohms_nearest(uint16_t ohms)
{
    uint8_t  best_i = BRAKE_OHMS_DEFAULT;
    uint16_t best_d = 0xFFFFU;
    for (uint8_t i = 0; i < BRAKE_OHMS_COUNT; i++) {
        uint16_t lbl = brake_ohms_label[i];
        uint16_t d   = (ohms > lbl) ? (ohms - lbl) : (lbl - ohms);
        if (d < best_d) { best_d = d; best_i = i; }
    }
    return best_i;
}

/* ── Main brake_tick ─────────────────────────────────────── */
void brake_tick(void)
{
    extern volatile bool     s_adc_ready;
    extern volatile uint16_t s_adc_buf[ADC_NUM_CHANNELS];

    if (!s_adc_ready) return;
    s_adc_ready = false;

    s_ctx.white_prev_mv = s_ctx.white_mv;
    s_ctx.white_mv  = raw_to_mv(s_adc_buf[0], WHITE_DIV_NUM, WHITE_DIV_DEN);
    s_ctx.black_mv  = raw_to_mv(s_adc_buf[1], BLACK_DIV_NUM, BLACK_DIV_DEN);
    s_ctx.pot_raw   = s_adc_buf[2];
    s_ctx.pot_idx   = (uint8_t)(s_ctx.pot_raw >> 4);

    /* Toggle read */
    bool t0 = (gpio_get(TOGGLE0_PORT, TOGGLE0_PIN) == 0);
    bool t1 = (gpio_get(TOGGLE1_PORT, TOGGLE1_PIN) == 0);
    ModeSelect_t new_mode;
    if      ( t0 && !t1) new_mode = MODE_B;
    else if (!t0 &&  t1) new_mode = MODE_C;
    else                 new_mode = MODE_A;

    bool leaving_c = (s_ctx.mode_sel == MODE_C && new_mode != MODE_C);
    bool entering_c = (s_ctx.mode_sel != MODE_C && new_mode == MODE_C);

    s_ctx.mode_prev = s_ctx.mode_sel;
    s_ctx.mode_sel  = new_mode;

    /* On leaving Mode C: arm the 500ms debounce save        */
    if (leaving_c) {
        s_ctx.ka_pending_save = true;
        s_ctx.ka_save_tick    = now_ms();
    }

    /* Process pending KA save after debounce period         */
    if (s_ctx.ka_pending_save &&
        (now_ms() - s_ctx.ka_save_tick) >= KA_SAVE_DEBOUNCE_MS) {
        s_ctx.saved_ka_dac    = ka_dac_table[s_ctx.pot_idx];
        s_ctx.ka_pending_save = false;
        do_save();
        led_blink_start();
    }

    /* Latvian brake */
    if (uart_debug_get_params()->latvian_dvdt_mv_per_ms > 0U) {
        int32_t dv = (int32_t)s_ctx.white_prev_mv - (int32_t)s_ctx.white_mv;
        if (dv > (int32_t)uart_debug_get_params()->latvian_dvdt_mv_per_ms) {
            set_ka_dac(DAC_KA_OFF);
            set_brake_dac(DAC_BRAKE_HARD);
            s_ctx.state = STATE_LATVIAN_BRAKE;
            goto done;
        }
    }
    if (s_ctx.state == STATE_LATVIAN_BRAKE &&
        s_ctx.white_mv > RAIL_UNDERVOLTAGE_MV)
        s_ctx.state = STATE_PASSTHROUGH;

    s_ctx.brake_active = update_brake_active(s_ctx.brake_active,
                                              s_ctx.black_mv);

    /* ── MODE_C: KA tuning ─────────────────────────────── */
    if (s_ctx.mode_sel == MODE_C) {
        /* Pot dials KA floor live; voltmeter shows it       */
        uint16_t ka_preview = ka_dac_table[s_ctx.pot_idx];
        set_ka_dac(ka_preview);

        /* Still brake normally during tuning               */
        if (!s_ctx.brake_active) {
            if (s_ctx.state == STATE_BRAKING || s_ctx.state == STATE_RAMP_IN)
                s_ctx.state = STATE_RELEASE;
            if (s_ctx.state == STATE_RELEASE) {
                if (!release_ramp_tick(s_ctx.saved_release_ms_c))
                    goto done;
            }
            set_brake_dac(DAC_BRAKE_OFF);
            s_ctx.state = STATE_KA_TUNING;
        } else {
            uint16_t brk = map_u16(s_ctx.pot_raw, 0, ADC_MAX,
                                   brake_soft_dac_by_ohm[active_ohms_idx()],
                                   DAC_BRAKE_HARD);
            s_ctx.brake_dac_target = brk;
            uint16_t alpha = RAMP_MS_TO_ALPHA(s_ctx.saved_ramp_ms);
            s_ctx.state = ramp_tick(brk, alpha) ? STATE_BRAKING : STATE_RAMP_IN;
        }
        voltmeter_update(true, s_ctx.white_mv, ka_dac_table[s_ctx.pot_idx]);
        led_blink_tick();
        return;
    }

    /* ── MODE_A: brake only, linear pot ───────────────── */
    if (s_ctx.mode_sel == MODE_A) {
        /* Q1: apply saved KA floor whenever V_BLACK low    */
        bool ka_active = ka_should_run(s_ctx.black_mv, s_ctx.white_mv)
                         && (s_ctx.saved_ka_dac > DAC_KA_OFF);
        set_ka_dac(ka_active ? s_ctx.saved_ka_dac : DAC_KA_OFF);

        if (!s_ctx.brake_active) {
            if (s_ctx.state == STATE_BRAKING || s_ctx.state == STATE_RAMP_IN)
                s_ctx.state = STATE_RELEASE;
            if (s_ctx.state == STATE_RELEASE) {
                if (!release_ramp_tick(s_ctx.saved_release_ms_a))
                    goto done;
            }
            set_brake_dac(DAC_BRAKE_OFF);
            s_ctx.state = ka_active ? STATE_KEEPALIVE_ONLY : STATE_PASSTHROUGH;
        } else {
            /* Linear map: pot CCW = selected soft floor, CW = HARD */
            uint16_t brk = map_u16(s_ctx.pot_raw, 0, ADC_MAX,
                                   brake_soft_dac_by_ohm[active_ohms_idx()],
                                   DAC_BRAKE_HARD);
            s_ctx.brake_dac_target = brk;
            set_ka_dac(DAC_KA_OFF);   /* Q1 off during braking  */
            uint16_t alpha = RAMP_MS_TO_ALPHA(s_ctx.saved_ramp_ms);
            s_ctx.state = ramp_tick(brk, alpha) ? STATE_BRAKING : STATE_RAMP_IN;
        }
        goto done;
    }

    /* ── MODE_B: anti-brake table, brushed ────────────── */
    {
        uint8_t  idx     = s_ctx.pot_idx;
        uint16_t brk_tgt = brake_tables[active_ohms_idx()][idx];
        uint16_t ka_tgt  = ka_dac_table[idx];
        uint16_t alpha   = RAMP_MS_TO_ALPHA(s_ctx.saved_ramp_ms);

        if (!s_ctx.brake_active) {
            if (s_ctx.state == STATE_BRAKING || s_ctx.state == STATE_RAMP_IN)
                s_ctx.state = STATE_RELEASE;
            if (s_ctx.state == STATE_RELEASE) {
                if (!release_ramp_tick(s_ctx.saved_release_ms_b)) {
                    set_ka_dac(DAC_KA_OFF);
                    goto done;
                }
            }
            set_brake_dac(DAC_BRAKE_OFF);
            /* Anti-brake: Q1 at table value after brake exit */
            set_ka_dac(ka_tgt);
            s_ctx.state = (ka_tgt > DAC_KA_OFF)
                          ? STATE_ANTI_BRAKE : STATE_PASSTHROUGH;
        } else {
            set_ka_dac(DAC_KA_OFF);
            s_ctx.brake_dac_target = brk_tgt;
            s_ctx.state = ramp_tick(brk_tgt, alpha)
                          ? STATE_BRAKING : STATE_RAMP_IN;
        }
    }

done:
    /* Voltmeter: track voltage normally                     */
    voltmeter_update(false, s_ctx.white_mv, 0U);
    led_blink_tick();
}

/* ── brake_init ──────────────────────────────────────────── */
void brake_init(void)
{
    s_ctx = (BrakeCtx_t){
        .state        = STATE_SAFE_BRAKE,
        .saved_ka_dac = DEFAULT_KA_DAC,
    };
    clock_setup();
    gpio_setup();
    dac_setup();
    ka_pwm_setup();
    voltmeter_setup();
    dma_setup();
    adc_setup();
    systick_setup();
    uart_debug_init();
    flash_load();
    s_ctx.state = STATE_PASSTHROUGH;
}

/* ── brake_force_safe ────────────────────────────────────── */
void brake_force_safe(void)
