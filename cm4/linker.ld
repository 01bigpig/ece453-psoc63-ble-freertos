/* CM4 linker script - starts after CM0+ region */
MEMORY
{
    flash (rx)  : ORIGIN = 0x10002000, LENGTH = 0xFE000  /* ~1016 KB */
    sram  (rwx) : ORIGIN = 0x08002000, LENGTH = 0x46000  /* ~280 KB */
}

_estack_cm4 = ORIGIN(sram) + LENGTH(sram);

SECTIONS
{
    .text : {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
    } > flash

    .data : {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > sram AT > flash

    .bss : {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > sram
}
