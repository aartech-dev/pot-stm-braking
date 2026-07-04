/* ============================================================
 *  main.c — AART Slot Car Braking Module  Revision 11
 *  Target : STM32G051K6U6 (QFN-32)
 *  Library: libopencm3
 * ============================================================ */

#include "brake_module.h"
#include "uart_debug.h"

void sys_tick_handler(void)   { brake_systick_isr(); }
void hard_fault_handler(void) { brake_force_safe(); while (1) {} }
void nmi_handler(void)        { brake_force_safe(); while (1) {} }

int main(void)
{
    brake_init();
    uart_debug_init();

    while (1) {
        brake_tick();
        uart_debug_tick(brake_get_ctx());
    }
    return 0;
}
