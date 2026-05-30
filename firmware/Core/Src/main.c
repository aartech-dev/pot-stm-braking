/* ============================================================
 *  main.c — AART Slot Car Braking Module
 *  Library: libopencm3
 * ============================================================ */

#include "brake_module.h"

/* ── SysTick ISR ─────────────────────────────────────────── */
/* libopencm3 declares sys_tick_handler as the weak default.
 * We override it here to drive the millisecond tick.          */
void sys_tick_handler(void)
{
    brake_systick_isr();
}

/* ── Fault handlers ──────────────────────────────────────── */
/* Force the brake on immediately if the firmware crashes.
 * libopencm3 provides weak defaults; we override them.        */
void hard_fault_handler(void)
{
    brake_force_safe();
    while (1) {}
}

void nmi_handler(void)
{
    brake_force_safe();
    while (1) {}
}

/* ── Main ────────────────────────────────────────────────── */
int main(void)
{
    brake_init();

    while (1) {
        brake_tick();

        /* Optional: use brake_get_mode() here to drive a
         * status LED or debug output if a spare pin is available */
    }

    return 0; /* never reached */
}
