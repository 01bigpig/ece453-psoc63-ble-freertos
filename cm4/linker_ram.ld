/* CM4 linker script - run from SRAM */
MEMORY
{
    sram (rwx) : ORIGIN = 0x08002000, LENGTH = 0x46000
}

_estack_cm4 = ORIGIN(sram) + LENGTH(sram);

SECTIONS
{
    .text : {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
    } > sram

    .data : {
        *(.data*)
    } > sram

    .bss : {
        *(.bss*)
        *(COMMON)
    } > sram
}
