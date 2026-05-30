/* Linker script for STM32G041J6 (SOIC-8)
 * Flash : 32K @ 0x08000000
 * RAM   : 8K  @ 0x20000000
 *
 * libopencm3 dropped per-device .ld files in favour of a generator.
 * This is the manual equivalent: define MEMORY then include the
 * generic section layout from libopencm3.
 */

MEMORY
{
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}

/* Pull in libopencm3's standard section definitions
 * (handles .text, .data, .bss, vector table, etc.) */
INCLUDE cortex-m-generic.ld
