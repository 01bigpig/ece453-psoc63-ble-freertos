/* PSoC 63 CM4 bare-metal LED blink
 * Board: CY8CPROTO-063-BLE
 * LED:   P6.3 (active low - LOW = LED ON)
 */
#include <stdint.h>

/* GPIO Port 6 registers */
#define GPIO_PRT6_BASE      0x40320300UL
#define GPIO_PRT6_OUT       (*(volatile uint32_t *)(GPIO_PRT6_BASE + 0x00))
#define GPIO_PRT6_OUT_CLR   (*(volatile uint32_t *)(GPIO_PRT6_BASE + 0x04))
#define GPIO_PRT6_OUT_SET   (*(volatile uint32_t *)(GPIO_PRT6_BASE + 0x08))
#define GPIO_PRT6_OUT_INV   (*(volatile uint32_t *)(GPIO_PRT6_BASE + 0x0C))
#define GPIO_PRT6_CFG       (*(volatile uint32_t *)(GPIO_PRT6_BASE + 0x28))

/* HSIOM for Port 6 (optional, default is GPIO mode = 0) */
#define HSIOM_PRT6_PORT_SEL0 (*(volatile uint32_t *)0x40310218UL)

/* Pin 3 bit mask */
#define PIN3_MASK           (1UL << 3)

/* Drive mode: Strong drive (6) for pin 3, each pin uses 3 bits in CFG */
#define PIN3_DM_SHIFT       (3 * 3)  /* pin 3 * 3 bits per pin = bit 9 */
#define PIN3_DM_STRONG      (6UL << PIN3_DM_SHIFT)

extern uint32_t _estack_cm4;

void Default_Handler(void) { while(1); }
void Reset_Handler_CM4(void);
void NMI_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));

__attribute__((section(".isr_vector")))
const uint32_t vectors_cm4[] = {
    (uint32_t)&_estack_cm4,
    (uint32_t)Reset_Handler_CM4,
    (uint32_t)NMI_Handler,
    (uint32_t)HardFault_Handler,
};

/* Simple busy-wait delay (~500ms at 8MHz IMO) */
static void delay_ms(uint32_t ms) {
    /* At 8 MHz, ~8000 cycles per ms. Loop overhead ~4 cycles. */
    volatile uint32_t count = ms * 2000;
    while (count--) {
        __asm volatile("nop");
    }
}

void Reset_Handler_CM4(void) {
    /* Configure P6.3 as strong drive output */
    uint32_t cfg = GPIO_PRT6_CFG;
    cfg &= ~(0x7UL << PIN3_DM_SHIFT);  /* Clear pin 3 drive mode bits */
    cfg |= PIN3_DM_STRONG;              /* Set strong drive */
    GPIO_PRT6_CFG = cfg;

    /* Start with LED off (high, since active low) */
    GPIO_PRT6_OUT_SET = PIN3_MASK;

    /* Blink forever */
    while (1) {
        GPIO_PRT6_OUT_INV = PIN3_MASK;  /* Toggle LED */
        delay_ms(500);
    }
}
