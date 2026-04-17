/* CM0+ linker script - occupies first 8KB of flash */
MEMORY
{
    flash (rx)  : ORIGIN = 0x10000000, LENGTH = 0x2000   /* 8 KB */
    sram  (rwx) : ORIGIN = 0x08000000, LENGTH = 0x2000   /* 8 KB */
}

_estack_cm0p = ORIGIN(sram) + LENGTH(sram);

SECTIONS
{
    .text : {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
    } > flash

    .data : {
        *(.data*)
    } > sram AT > flash

    .bss : {
        *(.bss*)
        *(COMMON)
    } > sram
}
