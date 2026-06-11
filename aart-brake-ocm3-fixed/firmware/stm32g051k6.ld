/* Linker script — STM32G051K6U6 (QFN-32)
 * Flash: 32K @ 0x08000000  (pages 0–31, 1K each)
 * RAM  : 8K  @ 0x20000000
 *
 * NOTE: Last page (page 31, 0x08007C00) is reserved for
 * non-volatile keep-alive CCR storage. The linker script
 * keeps firmware out of this region via LENGTH = 31K.
 */
MEMORY
{
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 31K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}

INCLUDE cortex-m-generic.ld
