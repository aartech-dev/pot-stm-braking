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
 *    1. Hold BOOT0 button (J12, pulls PB8/BOOT0 to 3.3V)
 *    2. Press and release NRST button (J13)
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
    uint16_t brake_enter_mv;    /* default BRAKE_ENTER_MV      */
    uint16_t brake_exit_mv;     /* default BRAKE_EXIT_MV       */
    uint16_t brake_ccr_soft;    /* default BRAKE_CCR_SOFT      */
    uint16_t ka_ccr_max;        /* default KA_CCR_MAX          */
} DebugParams_t;

/* ── Public API ──────────────────────────────────────────── */
void uart_debug_init(void);
void uart_debug_tick(const BrakeCtx_t *ctx);  /* call from main loop */
const DebugParams_t *uart_debug_get_params(void);

#endif /* UART_DEBUG_H */
