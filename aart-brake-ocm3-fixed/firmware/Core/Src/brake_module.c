/* ============================================================
 *  brake_module.c — AART Slot Car Braking Module  Rev 7
 *  Target  : STM32G051K6U6 (QFN-32)
 *  Library : libopencm3
 *
 *  Output stage: DAC1 (PA4) → Q2 brake, DAC2 (PA5) → Q1 KA.
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

static void set_ka_dac(uint16_t val)
{
    if (val > 4095U) val = 4095U;
    dac_load_data_buffer_single(DAC1, val, DAC_ALIGN_RIGHT, DAC_KA_CHAN);
    dac_software_trigger(DAC1, DAC_KA_CHAN);
    s_ctx.ka_dac = val;
}

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

    /* PA4 DAC1 (Q2 brake), PA5 DAC2 (Q1 KA) — analog          */
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                    GPIO4 | GPIO5);

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

    /* Both channels: software trigger, output buffer enabled   */
    dac_trigger_enable(DAC1, DAC_BRAKE_CHAN);
    dac_trigger_enable(DAC1, DAC_KA_CHAN);
    dac_set_trigger_source(DAC1, DAC_CR_TSEL1_SW);  /* CH1 sw trigger */
    dac_set_trigger_source(DAC1, DAC_CR_TSEL2_SW);  /* CH2 sw trigger */
    dac_enable(DAC1, DAC_BRAKE_CHAN);
    dac_enable(DAC1, DAC_KA_CHAN);

    /* Safe defaults: Q2 brake off (0V), Q1 KA off (3.3V)       */
    set_brake_dac(DAC_BRAKE_OFF);
    set_ka_dac(DAC_KA_OFF);
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
 * Capture active: display DAC2 setting as equivalent voltage.  */
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
    2, 3, 4, 6, 8,
};

static const uint16_t brake_soft_dac_by_ohm[BRAKE_OHMS_COUNT] = {
    2300, 2200, 2100, 1950, 1860,
};

static const uint16_t brake_tables[BRAKE_OHMS_COUNT][256] = {
  /* index 0: ~2 ohm full scale (soft=2300) */
  {
    2300, 2300, 2300, 2300, 2300, 2301, 2301, 2301, 2302, 2302, 2302, 2303, 2303, 2304, 2304, 2305,
    2305, 2306, 2307, 2307, 2308, 2309, 2310, 2310, 2311, 2312, 2313, 2314, 2315, 2316, 2317, 2318,
    2319, 2320, 2321, 2322, 2323, 2325, 2326, 2327, 2328, 2330, 2331, 2332, 2334, 2335, 2336, 2338,
    2339, 2341, 2342, 2344, 2345, 2347, 2349, 2350, 2352, 2354, 2355, 2357, 2359, 2361, 2362, 2364,
    2366, 2368, 2370, 2372, 2374, 2376, 2378, 2380, 2382, 2384, 2386, 2388, 2390, 2392, 2394, 2396,
    2399, 2401, 2403, 2405, 2408, 2410, 2412, 2415, 2417, 2420, 2422, 2424, 2427, 2429, 2432, 2434,
    2437, 2440, 2442, 2445, 2447, 2450, 2453, 2455, 2458, 2461, 2464, 2467, 2469, 2472, 2475, 2478,
    2481, 2484, 2487, 2490, 2493, 2496, 2499, 2502, 2505, 2508, 2511, 2514, 2517, 2520, 2523, 2527,
    2530, 2533, 2536, 2540, 2543, 2546, 2550, 2553, 2556, 2560, 2563, 2567, 2570, 2574, 2577, 2581,
    2584, 2588, 2591, 2595, 2599, 2602, 2606, 2610, 2613, 2617, 2621, 2624, 2628, 2632, 2636, 2640,
    2644, 2647, 2651, 2655, 2659, 2663, 2667, 2671, 2675, 2679, 2683, 2687, 2691, 2695, 2700, 2704,
    2708, 2712, 2716, 2720, 2725, 2729, 2733, 2738, 2742, 2746, 2751, 2755, 2759, 2764, 2768, 2773,
    2777, 2782, 2786, 2791, 2795, 2800, 2804, 2809, 2813, 2818, 2823, 2827, 2832, 2837, 2841, 2846,
    2851, 2856, 2861, 2865, 2870, 2875, 2880, 2885, 2890, 2895, 2900, 2904, 2909, 2914, 2919, 2925,
    2930, 2935, 2940, 2945, 2950, 2955, 2960, 2965, 2971, 2976, 2981, 2986, 2992, 2997, 3002, 3007,
    3013, 3018, 3024, 3029, 3034, 3040, 3045, 3051, 3056, 3062, 3067, 3073, 3078, 3084, 3089, 3095,
  },
  /* index 1: ~3 ohm full scale (soft=2200) */
  {
    2200, 2200, 2200, 2200, 2201, 2201, 2201, 2201, 2202, 2202, 2203, 2203, 2204, 2204, 2205, 2205,
    2206, 2207, 2208, 2208, 2209, 2210, 2211, 2212, 2213, 2214, 2215, 2216, 2217, 2218, 2219, 2220,
    2221, 2223, 2224, 2225, 2226, 2228, 2229, 2230, 2232, 2233, 2235, 2236, 2238, 2239, 2241, 2243,
    2244, 2246, 2248, 2249, 2251, 2253, 2255, 2257, 2258, 2260, 2262, 2264, 2266, 2268, 2270, 2272,
    2274, 2276, 2279, 2281, 2283, 2285, 2287, 2290, 2292, 2294, 2297, 2299, 2301, 2304, 2306, 2309,
    2311, 2314, 2316, 2319, 2321, 2324, 2327, 2329, 2332, 2335, 2337, 2340, 2343, 2346, 2348, 2351,
    2354, 2357, 2360, 2363, 2366, 2369, 2372, 2375, 2378, 2381, 2384, 2387, 2391, 2394, 2397, 2400,
    2404, 2407, 2410, 2413, 2417, 2420, 2424, 2427, 2430, 2434, 2437, 2441, 2444, 2448, 2452, 2455,
    2459, 2462, 2466, 2470, 2474, 2477, 2481, 2485, 2489, 2493, 2496, 2500, 2504, 2508, 2512, 2516,
    2520, 2524, 2528, 2532, 2536, 2540, 2544, 2549, 2553, 2557, 2561, 2565, 2570, 2574, 2578, 2582,
    2587, 2591, 2596, 2600, 2604, 2609, 2613, 2618, 2622, 2627, 2631, 2636, 2641, 2645, 2650, 2654,
    2659, 2664, 2669, 2673, 2678, 2683, 2688, 2693, 2697, 2702, 2707, 2712, 2717, 2722, 2727, 2732,
    2737, 2742, 2747, 2752, 2757, 2762, 2768, 2773, 2778, 2783, 2788, 2794, 2799, 2804, 2810, 2815,
    2820, 2826, 2831, 2836, 2842, 2847, 2853, 2858, 2864, 2869, 2875, 2881, 2886, 2892, 2897, 2903,
    2909, 2914, 2920, 2926, 2932, 2937, 2943, 2949, 2955, 2961, 2967, 2973, 2979, 2985, 2990, 2996,
    3002, 3009, 3015, 3021, 3027, 3033, 3039, 3045, 3051, 3057, 3064, 3070, 3076, 3082, 3089, 3095,
  },
  /* index 2: ~4 ohm full scale (soft=2100) */
  {
    2100, 2100, 2100, 2100, 2101, 2101, 2101, 2102, 2102, 2102, 2103, 2103, 2104, 2105, 2105, 2106,
    2107, 2108, 2108, 2109, 2110, 2111, 2112, 2113, 2114, 2115, 2116, 2117, 2119, 2120, 2121, 2122,
    2124, 2125, 2126, 2128, 2129, 2131, 2132, 2134, 2135, 2137, 2139, 2140, 2142, 2144, 2146, 2147,
    2149, 2151, 2153, 2155, 2157, 2159, 2161, 2163, 2165, 2167, 2169, 2171, 2174, 2176, 2178, 2180,
    2183, 2185, 2187, 2190, 2192, 2195, 2197, 2200, 2202, 2205, 2207, 2210, 2213, 2215, 2218, 2221,
    2223, 2226, 2229, 2232, 2235, 2238, 2241, 2244, 2247, 2250, 2253, 2256, 2259, 2262, 2265, 2268,
    2271, 2275, 2278, 2281, 2285, 2288, 2291, 2295, 2298, 2301, 2305, 2308, 2312, 2315, 2319, 2323,
    2326, 2330, 2334, 2337, 2341, 2345, 2349, 2352, 2356, 2360, 2364, 2368, 2372, 2376, 2380, 2384,
    2388, 2392, 2396, 2400, 2404, 2408, 2412, 2417, 2421, 2425, 2429, 2434, 2438, 2442, 2447, 2451,
    2456, 2460, 2465, 2469, 2474, 2478, 2483, 2487, 2492, 2497, 2501, 2506, 2511, 2516, 2520, 2525,
    2530, 2535, 2540, 2545, 2550, 2554, 2559, 2564, 2569, 2575, 2580, 2585, 2590, 2595, 2600, 2605,
    2610, 2616, 2621, 2626, 2632, 2637, 2642, 2648, 2653, 2658, 2664, 2669, 2675, 2680, 2686, 2691,
    2697, 2703, 2708, 2714, 2720, 2725, 2731, 2737, 2743, 2748, 2754, 2760, 2766, 2772, 2778, 2784,
    2790, 2796, 2802, 2808, 2814, 2820, 2826, 2832, 2838, 2844, 2850, 2857, 2863, 2869, 2875, 2882,
    2888, 2894, 2901, 2907, 2913, 2920, 2926, 2933, 2939, 2946, 2952, 2959, 2966, 2972, 2979, 2985,
    2992, 2999, 3006, 3012, 3019, 3026, 3033, 3040, 3046, 3053, 3060, 3067, 3074, 3081, 3088, 3095,
  },
  /* index 3: ~6 ohm full scale (soft=1950) */
  {
    1950, 1950, 1950, 1950, 1951, 1951, 1951, 1952, 1952, 1953, 1953, 1954, 1955, 1955, 1956, 1957,
    1958, 1959, 1960, 1961, 1962, 1963, 1964, 1965, 1966, 1968, 1969, 1970, 1971, 1973, 1974, 1976,
    1977, 1979, 1980, 1982, 1984, 1985, 1987, 1989, 1991, 1993, 1995, 1996, 1998, 2000, 2002, 2005,
    2007, 2009, 2011, 2013, 2015, 2018, 2020, 2022, 2025, 2027, 2030, 2032, 2035, 2037, 2040, 2042,
    2045, 2048, 2051, 2053, 2056, 2059, 2062, 2065, 2068, 2071, 2073, 2077, 2080, 2083, 2086, 2089,
    2092, 2095, 2099, 2102, 2105, 2108, 2112, 2115, 2119, 2122, 2126, 2129, 2133, 2136, 2140, 2144,
    2147, 2151, 2155, 2159, 2162, 2166, 2170, 2174, 2178, 2182, 2186, 2190, 2194, 2198, 2202, 2206,
    2210, 2215, 2219, 2223, 2227, 2232, 2236, 2240, 2245, 2249, 2254, 2258, 2263, 2267, 2272, 2276,
    2281, 2286, 2291, 2295, 2300, 2305, 2310, 2314, 2319, 2324, 2329, 2334, 2339, 2344, 2349, 2354,
    2359, 2364, 2370, 2375, 2380, 2385, 2391, 2396, 2401, 2407, 2412, 2417, 2423, 2428, 2434, 2439,
    2445, 2450, 2456, 2462, 2467, 2473, 2479, 2484, 2490, 2496, 2502, 2508, 2514, 2520, 2525, 2531,
    2537, 2543, 2549, 2556, 2562, 2568, 2574, 2580, 2586, 2593, 2599, 2605, 2611, 2618, 2624, 2631,
    2637, 2643, 2650, 2656, 2663, 2670, 2676, 2683, 2689, 2696, 2703, 2709, 2716, 2723, 2730, 2737,
    2744, 2750, 2757, 2764, 2771, 2778, 2785, 2792, 2799, 2806, 2813, 2821, 2828, 2835, 2842, 2849,
    2857, 2864, 2871, 2879, 2886, 2893, 2901, 2908, 2916, 2923, 2931, 2938, 2946, 2954, 2961, 2969,
    2977, 2984, 2992, 3000, 3008, 3015, 3023, 3031, 3039, 3047, 3055, 3063, 3071, 3079, 3087, 3095,
  },
  /* index 4: ~8 ohm full scale (soft=1860) */
  {
    1860, 1860, 1860, 1860, 1861, 1861, 1861, 1862, 1862, 1863, 1864, 1864, 1865, 1866, 1867, 1868,
    1868, 1869, 1870, 1872, 1873, 1874, 1875, 1876, 1878, 1879, 1880, 1882, 1883, 1885, 1886, 1888,
    1889, 1891, 1893, 1895, 1896, 1898, 1900, 1902, 1904, 1906, 1908, 1910, 1912, 1914, 1917, 1919,
    1921, 1923, 1926, 1928, 1931, 1933, 1936, 1938, 1941, 1943, 1946, 1949, 1951, 1954, 1957, 1960,
    1963, 1965, 1968, 1971, 1974, 1977, 1981, 1984, 1987, 1990, 1993, 1996, 2000, 2003, 2006, 2010,
    2013, 2017, 2020, 2024, 2027, 2031, 2035, 2038, 2042, 2046, 2049, 2053, 2057, 2061, 2065, 2069,
    2073, 2077, 2081, 2085, 2089, 2093, 2097, 2102, 2106, 2110, 2114, 2119, 2123, 2127, 2132, 2136,
    2141, 2145, 2150, 2155, 2159, 2164, 2169, 2173, 2178, 2183, 2188, 2192, 2197, 2202, 2207, 2212,
    2217, 2222, 2227, 2232, 2238, 2243, 2248, 2253, 2258, 2264, 2269, 2274, 2280, 2285, 2291, 2296,
    2302, 2307, 2313, 2318, 2324, 2329, 2335, 2341, 2347, 2352, 2358, 2364, 2370, 2376, 2382, 2388,
    2394, 2400, 2406, 2412, 2418, 2424, 2430, 2436, 2443, 2449, 2455, 2462, 2468, 2474, 2481, 2487,
    2494, 2500, 2507, 2513, 2520, 2526, 2533, 2540, 2546, 2553, 2560, 2567, 2573, 2580, 2587, 2594,
    2601, 2608, 2615, 2622, 2629, 2636, 2643, 2650, 2658, 2665, 2672, 2679, 2686, 2694, 2701, 2708,
    2716, 2723, 2731, 2738, 2746, 2753, 2761, 2768, 2776, 2784, 2791, 2799, 2807, 2815, 2822, 2830,
    2838, 2846, 2854, 2862, 2870, 2878, 2886, 2894, 2902, 2910, 2918, 2926, 2934, 2943, 2951, 2959,
    2967, 2976, 2984, 2992, 3001, 3009, 3018, 3026, 3035, 3043, 3052, 3060, 3069, 3078, 3086, 3095,
  },
};

static const uint16_t ka_dac_table[256] = {
    2480, 2480, 2480, 2480, 2481, 2481, 2481, 2481, 2482, 2482, 2483, 2484, 2484, 2485, 2486, 2487,
    2488, 2489, 2490, 2491, 2492, 2494, 2495, 2496, 2498, 2499, 2501, 2503, 2505, 2507, 2509, 2511,
    2514, 2516, 2519, 2521, 2524, 2527, 2530, 2533, 2536, 2539, 2543, 2546, 2550, 2554, 2558, 2562,
    2566, 2570, 2575, 2580, 2584, 2589, 2594, 2599, 2604, 2610, 2615, 2621, 2626, 2632, 2638, 2645,
    2651, 2658, 2664, 2671, 2678, 2685, 2693, 2700, 2708, 2716, 2724, 2732, 2740, 2749, 2757, 2766,
    2775, 2784, 2793, 2803, 2812, 2822, 2832, 2842, 2852, 2862, 2873, 2884, 2894, 2905, 2917, 2928,
    2940, 2951, 2963, 2975, 2988, 3000, 3013, 3026, 3039, 3052, 3065, 3079, 3092, 3106, 3120, 3134,
    3149, 3163, 3178, 3192, 3207, 3222, 3238, 3253, 3269, 3284, 3300, 3316, 3333, 3349, 3366, 3382,
    3399, 3416, 3434, 3451, 3469, 3487, 3505, 3523, 3541, 3560, 3578, 3597, 3616, 3635, 3655, 3674,
    3694, 3714, 3734, 3754, 3774, 3795, 3815, 3836, 3857, 3878, 3899, 3921, 3942, 3964, 3986, 4008,
    4030, 4052, 4075, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
    4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
    4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
    4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
    4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
    4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
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
                         && (s_ctx.saved_ka_dac < DAC_KA_OFF);
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
            s_ctx.state = (ka_tgt < DAC_KA_OFF)
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
