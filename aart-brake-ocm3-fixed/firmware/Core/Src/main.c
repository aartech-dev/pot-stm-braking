/* ============================================================
 *  main.c — AART Slot Car Braking Module  Rev 2
 *  Target : STM32G041K6U6 (QFN-32)
 *  Library: libopencm3
 * ============================================================ */

#include "brake_module.h"

void sys_tick_handler(void)   { brake_systick_isr(); }
void hard_fault_handler(void) { brake_force_safe(); while (1) {} }
void nmi_handler(void)        { brake_force_safe(); while (1) {} }

int main(void)
{
    brake_init();
    while (1) {
        brake_tick();
    }
    return 0;
}
