#ifndef UART_DEBUG_H
#define UART_DEBUG_H

/* ============================================================
 *  uart_debug.h — AART Slot Car Braking Module
 *
 *  USART1 telemetry and command interface.
 *
 *  Hardware:
 *    PA9   USART1_TX  AF1   → J11 pin 1 (UART/DBG header)
 *    PA10  USART1_RX  AF1   → J11 pin 2
 *    GND               → J11 pin 3
 *    Baud: 115200 8N1
 *
 *  Connect any USB-UART adapter (3.3V logic level).
 *  On macOS: screen /dev/tty.usbserial-XXXX 115200
 *  On Linux: minicom -D /dev/ttyUSB0 -b 115200
 *
 *  ── Telemetry output (every 100ms) ──────────────────────
 *  CSV line: tick,state,mode,white_mv,black_mv,brake_ccr,
 *             ka_ccr,pot_raw,ramp_ms,brake_active
 *  Example:
 *    12340,BRAKE,A,12400,0,1850,0,512,80,1
 *
 *  Header line sent on startup:
 *    #tick,state,mode,white_mv,black_mv,brake_ccr,ka_ccr,pot_raw,ramp_ms,brake_active
 *
 *  ── Command input ────────────────────────────────────────
 *  Send a line (CR or LF terminated) to adjust thresholds
 *  at runtime without reflashing. Changes are NOT saved to
 *  flash — they revert on power cycle. Use the capture
 *  button to permanently save ka_ccr and ramp_ms.
 *
 *  Commands:
 *    SET BRAKE_ENTER <mV>   e.g. SET BRAKE_ENTER 400
 *    SET BRAKE_EXIT  <mV>   e.g. SET BRAKE_EXIT 1200
 *    SET BRAKE_SOFT  <ccr>  e.g. SET BRAKE_SOFT 2400
 *    SET KA_MAX      <ccr>  e.g. SET KA_MAX 300
 *    GET                    print current values
 *    HELP                   print command list
 *
 *  ── ROM bootloader entry ─────────────────────────────────
 *  To enter the STM32 ROM bootloader for field flashing
 *  without an ST-Link:
 *    1. Hold BOOT0 button (J12 -> PA14-BOOT0, pulls it to 3.3V;
 *    2. Press and release NRST button (J13)
 *       requires option bit nBOOT_SEL=0. NOTE: BOOT0 is PA14 on
 *       STM32G0, not PB8 — see brake_module.h.)
 *    3. Release BOOT0 button
 *    4. MCU is now in UART bootloader on PA9/PA10
 *    5. Flash with: stm32flash -w aart_brake.bin /dev/ttyUSB0
 *       or STM32CubeProgrammer in UART mode
 * ============================================================ */

#include "brake_module.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Runtime-adjustable thresholds ───────────────────────── */
/* These mirror the compile-time constants but can be changed
 * at runtime via UART commands. Initialised from the
 * compile-time values in uart_debug_init().                  */
typedef struct {
    uint16_t brake_enter_mv;
    uint16_t brake_exit_mv;
    uint16_t brake_soft_dac;          /* legacy DAC value (superseded by ohms) */
    uint16_t brake_ohms_idx;          /* 0-4 = ~1..5Ω full-scale brake table   */
    uint16_t ka_dac_max;              /* DAC value for maximum KA/anti-brake */
    uint16_t latvian_dvdt_mv_per_ms;  /* 0 = disabled                        */
    uint16_t release_ms_a;
    uint16_t release_ms_b;
    uint16_t release_ms_c;
    uint16_t ramp_ms;                 /* brake ramp-in time (ms), applied on SAVE */
    uint16_t ramp_curve;              /* feature D: 0=linear, 1=exp, 2=log */
} DebugParams_t;

/* ── Public API ──────────────────────────────────────────── */
void uart_debug_init(void);
void uart_debug_tick(const BrakeCtx_t *ctx);
const DebugParams_t *uart_debug_get_params(void);
void uart_debug_load_params(uint16_t brake_enter,    uint16_t brake_exit,
                             uint16_t brake_ohms_idx, uint16_t latvian_dvdt,
                             uint16_t rel_ms_a,       uint16_t rel_ms_b,
                             uint16_t rel_ms_c,       uint16_t ramp_curve);

#endif /* UART_DEBUG_H */
