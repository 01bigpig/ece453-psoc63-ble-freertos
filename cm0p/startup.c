/* Minimal CM0+ image: just enable CM4 and go to sleep */
#include <stdint.h>

/* CPUSS registers */
#define CPUSS_CM4_CTL       (*(volatile uint32_t *)0x40200008UL)
#define CPUSS_CM4_STATUS    (*(volatile uint32_t *)0x4020000CUL)

/* CM4 vector table address */
#define CM4_APP_ADDR        0x10002000UL

/* IPC struct for CM4 enable (CPUSS_CM4_VECTOR_TABLE_BASE) */
#define CPUSS_CM4_VECTOR_TABLE_BASE (*(volatile uint32_t *)0x40210308UL)

extern uint32_t _estack_cm0p;

void Default_Handler(void) { while(1); }
void Reset_Handler(void);
void NMI_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));

__attribute__((section(".isr_vector")))
const uint32_t vectors_cm0p[] = {
    (uint32_t)&_estack_cm0p,
    (uint32_t)Reset_Handler,
    (uint32_t)NMI_Handler,
    (uint32_t)HardFault_Handler,
};

/* Simple delay */
static void delay(volatile uint32_t count) {
    while (count--) { __asm volatile("nop"); }
}

void Reset_Handler(void) {
    /* Enable CM4 core with vector table at CM4_APP_ADDR */
    /* Write the CM4 vector table base address */
    CPUSS_CM4_VECTOR_TABLE_BASE = CM4_APP_ADDR;

    /* Set CM4 CTL: bit 31 = 1 (PPB lock), bits 1:0 = 0 (enable/reset release) */
    /* Actually, on PSoC 63, we use a different mechanism via IPC */
    /* The simplest approach: write to CPUSS_CM4_CTL to release CM4 from reset */
    uint32_t ctl = CPUSS_CM4_CTL;
    ctl &= ~0x3UL;  /* Clear bits [1:0] to release CM4 from reset */
    CPUSS_CM4_CTL = ctl;

    /* Wait a bit for CM4 to start */
    delay(1000);

    /* CM0+ goes to sleep forever */
    while (1) {
        __asm volatile("wfi");
    }
}
