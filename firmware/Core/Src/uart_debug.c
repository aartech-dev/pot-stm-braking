/* ============================================================
 *  uart_debug.c — USART1 telemetry and command interface
 *  Target  : STM32G051K6U6 (QFN-32)
 *  Library : libopencm3
 * ============================================================ */

#include "uart_debug.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ── Private state ───────────────────────────────────────── */
static DebugParams_t s_params;
static bool          s_scope = false;   /* feature E: binary scope mode */

/* TX ring buffer — non-blocking output                       */
#define TX_BUF_SIZE    256U
static volatile uint8_t  s_tx_buf[TX_BUF_SIZE];
static volatile uint16_t s_tx_head = 0;
static volatile uint16_t s_tx_tail = 0;

/* RX line buffer — command accumulation                      */
#define RX_LINE_SIZE   64U
static char     s_rx_line[RX_LINE_SIZE];
static uint8_t  s_rx_idx = 0;

/* Telemetry pacing — handled by tick % TELEM_INTERVAL_MS in uart_debug_tick */
#define TELEM_INTERVAL_MS  100U
#define SCOPE_INTERVAL_MS  5U     /* feature E: ~200Hz binary frames */

/* ── Utility: integer to decimal string (no printf) ─────── */
/* Writes at most 6 chars + null. Returns pointer past end.  */
static char *u16_to_str(char *p, uint16_t v)
{
    char tmp[6];
    uint8_t i = 0;
    if (v == 0) { *p++ = '0'; return p; }
    while (v > 0) { tmp[i++] = '0' + (v % 10); v /= 10; }
    while (i--) *p++ = tmp[i];
    return p;
}

static char *u32_to_str(char *p, uint32_t v)
{
    char tmp[10];
    uint8_t i = 0;
    if (v == 0) { *p++ = '0'; return p; }
    while (v > 0) { tmp[i++] = '0' + (v % 10); v /= 10; }
    while (i--) *p++ = tmp[i];
    return p;
}

/* ── TX ring buffer helpers ──────────────────────────────── */
static void tx_enqueue(uint8_t b)
{
    uint16_t next = (s_tx_head + 1U) % TX_BUF_SIZE;
    if (next != s_tx_tail) {
        s_tx_buf[s_tx_head] = b;
        s_tx_head = next;
    }
    /* Enable TXE interrupt to drain buffer                   */
    USART_CR1(USART1) |= USART_CR1_TXEIE;
}

static void tx_str(const char *s)
{
    while (*s) tx_enqueue((uint8_t)*s++);
}

static void tx_u16(uint16_t v)
{
    char buf[7]; char *p = buf;
    p = u16_to_str(p, v); *p = '\0';
    tx_str(buf);
}

static void tx_le16(uint16_t v) { tx_enqueue((uint8_t)(v & 0xFFU)); tx_enqueue((uint8_t)(v >> 8U)); }
static void tx_u32(uint32_t v)
{
    char buf[11]; char *p = buf;
    p = u32_to_str(p, v); *p = '\0';
    tx_str(buf);
}

/* ── State name helpers ──────────────────────────────────── */
static const char *state_name(OpState_t s)
{
    switch (s) {
        case STATE_PASSTHROUGH:    return "PASS";
        case STATE_RAMP_IN:        return "RAMP";
        case STATE_BRAKING:        return "BRAKE";
        case STATE_ANTI_BRAKE:     return "ANTI";
        case STATE_KEEPALIVE_ONLY: return "KA";
        case STATE_CAPTURE:        return "CAP";
        case STATE_RAMP_IN:        return "RAMP";
        case STATE_RELEASE:        return "RELEASE";
        case STATE_LATVIAN_BRAKE:  return "LATVIAN";
        case STATE_SAFE_BRAKE:     return "SAFE";
        default:                   return "?";
    }
}

static const char *mode_name(ModeSelect_t m)
{
    switch (m) {
        case MODE_A: return "A";
        case MODE_B: return "B";
        case MODE_C: return "C";
        default:     return "?";
    }
}

/* ── Command parser ──────────────────────────────────────── */
static void process_command(const char *line)
{
    /* Skip leading whitespace                                */
    while (*line == ' ') line++;

    if (strncmp(line, "GET ID", 6) == 0) {
        tx_str("#ID: "); tx_str(FW_ID_STRING); tx_str("\r\n");

    } else if (strncmp(line, "GET", 3) == 0) {
        tx_str("#params: brake_enter="); tx_u16(s_params.brake_enter_mv);
        tx_str("mV brake_exit=");        tx_u16(s_params.brake_exit_mv);
        tx_str(" brake_ohms=");          tx_u16(brake_ohms_label_at((uint8_t)s_params.brake_ohms_idx));
        tx_str(" ka_max=");              tx_u16(s_params.ka_dac_max);
        tx_str(" latvian_dvdt=");        tx_u16(s_params.latvian_dvdt_mv_per_ms);
        tx_str("mV/ms");
        if (s_params.latvian_dvdt_mv_per_ms == 0U) tx_str(" (DISABLED)");
        tx_str(" release_ms_a="); tx_u16(s_params.release_ms_a);
        tx_str("ms release_ms_b="); tx_u16(s_params.release_ms_b);
        tx_str("ms release_ms_c="); tx_u16(s_params.release_ms_c);
        tx_str("ms ramp_curve="); tx_u16(s_params.ramp_curve);
        tx_str(" pot_min="); tx_u16(brake_get_ctx()->saved_pot_min);
        tx_str("\r\n");

    } else if (strncmp(line, "HELP", 4) == 0) {
        tx_str("#Commands:\r\n");
        tx_str("#  SET BRAKE_ENTER <mV>       brake entry threshold\r\n");
        tx_str("#  SET BRAKE_EXIT  <mV>       brake exit threshold\r\n");
        tx_str("#  SET BRAKE_OHMS  <ohm>      min brake R, snaps to 2/3/4/6/8 ohm (default 3)\r\n");
        tx_str("#  SET KA_MAX      <dac>      max keep-alive/anti-brake DAC value (default 2480)\r\n");
        tx_str("#  SET LATVIAN_DVDT <mV/ms>   Latvian brake dV/dt threshold\r\n");
        tx_str("#                             0=disabled, 1000=recommended\r\n");
        tx_str("#  SET RELEASE_MS_A <ms>      trail brake release time mode A (0-500)\r\n");
        tx_str("#  SET RELEASE_MS_B <ms>      trail brake release time mode B (0-500)\r\n");
        tx_str("#  SET RELEASE_MS_C <ms>      trail brake release time mode C (0-500)\r\n");
        tx_str("#  SET RAMP_MS     <ms>       brake ramp-in time (0-200, 0=instant)\r\n");
        tx_str("#  SET RAMP_CURVE  <0|1|2>    brake-in shape: 0=linear 1=exp 2=log\r\n");
        tx_str("#  CAL POT_MIN                save pot CCW zero (turn pot fully CCW first)\r\n");
        tx_str("#  SCOPE <0|1>                0=CSV telemetry, 1=binary scope ~200Hz\r\n");
        tx_str("#  SAVE            save ALL current params to flash\r\n");
        tx_str("#  GET             print current runtime values\r\n");
        tx_str("#  HELP            this list\r\n");

    } else if (strncmp(line, "SAVE", 4) == 0) {
        /* Save all runtime params + current ka_ccr/ramp_ms to flash */
        const BrakeCtx_t *ctx = brake_get_ctx();
        tx_str("#Saving to flash...\r\n");
        flash_save_all(ctx->saved_ka_dac,
                       s_params.ramp_ms,
                       s_params.brake_enter_mv,
                       s_params.brake_exit_mv,
                       s_params.brake_ohms_idx,
                       s_params.latvian_dvdt_mv_per_ms,
                       s_params.release_ms_a,
                       s_params.release_ms_b,
                       s_params.release_ms_c,
                       ctx->saved_pot_min,
                       s_params.ramp_curve);
        tx_str("#Saved: brake_enter="); tx_u16(s_params.brake_enter_mv);
        tx_str("mV brake_exit=");       tx_u16(s_params.brake_exit_mv);
        tx_str(" brake_ohms=");         tx_u16(brake_ohms_label_at((uint8_t)s_params.brake_ohms_idx));
        tx_str(" ka_dac=");              tx_u16(ctx->saved_ka_dac);
        tx_str(" ramp_ms=");            tx_u16(ctx->saved_ramp_ms);
        tx_str(" latvian_dvdt=");       tx_u16(s_params.latvian_dvdt_mv_per_ms);
        tx_str("mV/ms\r\n");
        brake_led_blink();

    } else if (strncmp(line, "SET ", 4) == 0) {
        const char *rest = line + 4;
        uint16_t val = 0;
        const char *sp = strrchr(rest, ' ');
        if (!sp) { tx_str("#ERR: missing value\r\n"); return; }
        val = (uint16_t)atoi(sp + 1);

        if (strncmp(rest, "BRAKE_ENTER", 11) == 0) {
            s_params.brake_enter_mv = val;
            tx_str("#OK brake_enter="); tx_u16(val); tx_str("mV\r\n");
        } else if (strncmp(rest, "BRAKE_EXIT", 10) == 0) {
            s_params.brake_exit_mv = val;
            tx_str("#OK brake_exit="); tx_u16(val); tx_str("mV\r\n");
        } else if (strncmp(rest, "BRAKE_OHMS", 10) == 0) {
            /* Select nearest stored table to requested resistance.
             * Tables are non-linear: 2,3,4,6,8 ohm.                 */
            if (val < BRAKE_OHMS_ARG_MIN) val = BRAKE_OHMS_ARG_MIN;
            if (val > BRAKE_OHMS_ARG_MAX) val = BRAKE_OHMS_ARG_MAX;
            uint8_t idx = brake_ohms_nearest(val);
            s_params.brake_ohms_idx = idx;
            tx_str("#OK brake_ohms="); tx_u16(brake_ohms_label_at(idx));
            tx_str(" ohm (nearest to "); tx_u16(val);
            tx_str(", live now, SAVE to persist)\r\n");
        } else if (strncmp(rest, "KA_MAX", 6) == 0) {
            if (val > 4095U) val = 4095U;
            s_params.ka_dac_max = val;
            tx_str("#OK ka_max="); tx_u16(val); tx_str("\r\n");
        } else if (strncmp(rest, "LATVIAN_DVDT", 12) == 0) {
            /* Clamp: 0=disable, or between MIN and MAX          */
            if (val != 0U) {
                if (val < LATVIAN_DVDT_MIN) val = LATVIAN_DVDT_MIN;
                if (val > LATVIAN_DVDT_MAX) val = LATVIAN_DVDT_MAX;
            }
            s_params.latvian_dvdt_mv_per_ms = val;
            if (val == 0U) {
                tx_str("#OK latvian_dvdt=0 (DISABLED)\r\n");
            } else {
                tx_str("#OK latvian_dvdt="); tx_u16(val);
                tx_str("mV/ms (ENABLED)\r\n");
            }
        } else if (strncmp(rest, "RELEASE_MS_A", 12) == 0) {
            if (val > RELEASE_MS_MAX) val = RELEASE_MS_MAX;
            s_params.release_ms_a = val;
            tx_str("#OK release_ms_a="); tx_u16(val); tx_str("ms\r\n");
        } else if (strncmp(rest, "RELEASE_MS_B", 12) == 0) {
            if (val > RELEASE_MS_MAX) val = RELEASE_MS_MAX;
            s_params.release_ms_b = val;
            tx_str("#OK release_ms_b="); tx_u16(val); tx_str("ms\r\n");
        } else if (strncmp(rest, "RELEASE_MS_C", 12) == 0) {
            if (val > RELEASE_MS_MAX) val = RELEASE_MS_MAX;
            s_params.release_ms_c = val;
            tx_str("#OK release_ms_c="); tx_u16(val); tx_str("ms\r\n");

        } else if (strncmp(rest, "RAMP_MS", 7) == 0) {
            if (val > RAMP_MS_MAX) val = RAMP_MS_MAX;
            s_params.ramp_ms = val;
            tx_str("#OK ramp_ms="); tx_u16(val); tx_str("ms (SAVE to apply)\r\n");

        } else if (strncmp(rest, "RAMP_CURVE", 10) == 0) {
            if (val > RAMP_CURVE_LOG) val = RAMP_CURVE_DEFAULT;
            s_params.ramp_curve = val;
            tx_str("#OK ramp_curve="); tx_u16(val);
            tx_str(" (0=lin 1=exp 2=log, SAVE to persist)\r\n");
        } else {
            tx_str("#ERR: unknown param\r\n");
        }
    } else if (strncmp(line, "CAL POT_MIN", 11) == 0) {
        brake_calibrate_pot_min();
        tx_str("#OK pot_min captured at current CCW position (saved)\r\n");
    } else if (strncmp(line, "SCOPE", 5) == 0) {
        const char *sp = strrchr(line, ' ');
        uint16_t v = sp ? (uint16_t)atoi(sp + 1) : 0U;
        s_scope = (v != 0U);
        tx_str(s_scope ? "#OK scope ON (binary frames)\r\n"
                       : "#OK scope OFF (CSV telemetry)\r\n");
    } else {
        tx_str("#ERR: unknown command (try HELP)\r\n");
    }
}

/* ── ISR: USART1 TX empty + RX not empty ────────────────── */
void usart1_isr(void)
{
    /* TX: drain ring buffer                                  */
    if ((USART_ISR(USART1) & USART_ISR_TXE) &&
        (USART_CR1(USART1) & USART_CR1_TXEIE)) {
        if (s_tx_head != s_tx_tail) {
            USART_TDR(USART1) = s_tx_buf[s_tx_tail];
            s_tx_tail = (s_tx_tail + 1U) % TX_BUF_SIZE;
        } else {
            /* Buffer empty — disable TXE interrupt           */
            USART_CR1(USART1) &= ~USART_CR1_TXEIE;
        }
    }

    /* RX: accumulate characters into line buffer             */
    if (USART_ISR(USART1) & USART_ISR_RXNE) {
        char c = (char)(USART_RDR(USART1) & 0xFFU);
        if (c == '\r' || c == '\n') {
            if (s_rx_idx > 0) {
                s_rx_line[s_rx_idx] = '\0';
                process_command(s_rx_line);
                s_rx_idx = 0;
            }
        } else if (s_rx_idx < RX_LINE_SIZE - 1U) {
            s_rx_line[s_rx_idx++] = c;
        }
    }
}

/* ── Public: init ────────────────────────────────────────── */
void uart_debug_init(void)
{
    /* Initialise runtime params from compile-time defaults   */
    s_params.brake_enter_mv         = BRAKE_ENTER_MV;
    s_params.brake_exit_mv          = BRAKE_EXIT_MV;
    s_params.brake_soft_dac         = DAC_BRAKE_SOFT;
    s_params.brake_ohms_idx         = BRAKE_OHMS_DEFAULT;
    s_params.ka_dac_max             = DAC_KA_MAX;
    s_params.latvian_dvdt_mv_per_ms = LATVIAN_DVDT_DEFAULT;
    s_params.release_ms_a          = 0U;
    s_params.release_ms_b          = 0U;
    s_params.release_ms_c          = 0U;
    s_params.ramp_ms               = 0U;
    s_params.ramp_curve            = RAMP_CURVE_DEFAULT;

    /* Clock enable                                           */
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* PA9 TX, PA10 RX — AF1 (USART1) on G0                  */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9 | GPIO10);

    /* 115200 8N1                                             */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Enable RX interrupt; TX interrupt enabled on demand    */
    nvic_enable_irq(NVIC_USART1_IRQ);
    nvic_set_priority(NVIC_USART1_IRQ, 2);
    USART_CR1(USART1) |= USART_CR1_RXNEIE;

    usart_enable(USART1);

    /* Banner                                                 */
    tx_str("\r\n#AART Brake Module v8 — UART debug active\r\n");
    tx_str("#tick,state,mode,white_mv,black_mv,brake_dac,"
           "ka_ccr,pot_raw,ramp_ms,brake_active,latvian_active\r\n");
    tx_str("#Type HELP for command list\r\n");
}

/* ── Public: tick (call from main loop, self-paced) ─────── */
void uart_debug_tick(const BrakeCtx_t *ctx)
{
    /* Use tick from systick counter via ctx — we need the
     * ms timestamp. Approximate from brake state changes.
     * Actually we'll use a simple static counter.            */
    static uint32_t tick = 0;
    tick++;

    /* Feature E: binary scope mode — compact LE frame ~200Hz.        */
    if (s_scope) {
        if (tick % SCOPE_INTERVAL_MS != 0U) return;
        tx_enqueue(0xA5U); tx_enqueue(0x5AU);          /* sync        */
        tx_le16((uint16_t)tick);
        tx_enqueue((uint8_t)ctx->state);
        tx_enqueue((uint8_t)ctx->mode_sel);
        tx_le16(ctx->white_mv);
        tx_le16(ctx->black_mv);
        tx_le16(ctx->brake_ccr);
        tx_le16(ctx->ka_ccr);
        tx_le16(ctx->pot_raw);
        tx_enqueue((uint8_t)((ctx->brake_active ? 1U : 0U) |
                             (ctx->latvian_active ? 2U : 0U)));
        return;
    }

    /* Only emit every TELEM_INTERVAL_MS calls (main loop
     * runs as fast as possible, ~1ms/call).                  */
    if (tick % TELEM_INTERVAL_MS != 0U) return;

    /* Format: tick,state,mode,white_mv,black_mv,
     *          brake_ccr,ka_ccr,pot_raw,ramp_ms,brake_active */
    tx_u32(tick);           tx_enqueue(',');
    tx_str(state_name(ctx->state));  tx_enqueue(',');
    tx_str(mode_name(ctx->mode_sel));tx_enqueue(',');
    tx_u16(ctx->white_mv);  tx_enqueue(',');
    tx_u16(ctx->black_mv);  tx_enqueue(',');
    tx_u16(ctx->brake_ccr); tx_enqueue(',');
    tx_u16(ctx->ka_ccr);    tx_enqueue(',');
    tx_u16(ctx->pot_raw);   tx_enqueue(',');
    tx_u16(ctx->saved_ramp_ms); tx_enqueue(',');
    tx_enqueue(ctx->brake_active   ? '1' : '0'); tx_enqueue(',');
    tx_enqueue(ctx->latvian_active ? '1' : '0');
    tx_str("\r\n");
}

/* ── Public: load params from flash (called by flash_load) ── */
void uart_debug_load_params(uint16_t brake_enter,    uint16_t brake_exit,
                             uint16_t brake_ohms_idx, uint16_t latvian_dvdt,
                             uint16_t rel_ms_a,       uint16_t rel_ms_b,
                             uint16_t rel_ms_c,       uint16_t ramp_curve)
{
    /* Validate before applying — protect against flash corruption.
     * Release times are also restored directly into the brake context
     * by flash_load(); they are accepted here for signature symmetry
     * and mirrored into s_params so GET reports them.                */
    if (brake_enter > 0U && brake_enter < 5000U)
        s_params.brake_enter_mv = brake_enter;
    if (brake_exit > brake_enter && brake_exit < 16000U)
        s_params.brake_exit_mv = brake_exit;
    if (brake_ohms_idx < BRAKE_OHMS_COUNT)
        s_params.brake_ohms_idx = brake_ohms_idx;
    if (latvian_dvdt == 0U ||
        (latvian_dvdt >= LATVIAN_DVDT_MIN && latvian_dvdt <= LATVIAN_DVDT_MAX))
        s_params.latvian_dvdt_mv_per_ms = latvian_dvdt;
    if (rel_ms_a <= RELEASE_MS_MAX) s_params.release_ms_a = rel_ms_a;
    if (rel_ms_b <= RELEASE_MS_MAX) s_params.release_ms_b = rel_ms_b;
    if (rel_ms_c <= RELEASE_MS_MAX) s_params.release_ms_c = rel_ms_c;
    if (ramp_curve <= RAMP_CURVE_LOG) s_params.ramp_curve = ramp_curve;
}

/* ── Public: get runtime params ─────────────────────────── */
const DebugParams_t *uart_debug_get_params(void)
{
    return &s_params;
}
