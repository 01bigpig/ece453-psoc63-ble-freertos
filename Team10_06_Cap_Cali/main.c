#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "cy_sysint.h"
#include "cy_sysclk.h"
#include "cycfg_peripherals.h"
#include "cycfg_capsense.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdio.h>

#define CAPSENSE_TASK_STACK_SIZE    (1024)
#define CAPSENSE_TASK_PRIORITY      (2)
#define CALI_TASK_STACK_SIZE        (1024)
#define CALI_TASK_PRIORITY          (2)
#define APP_TASK_STACK_SIZE         (1024)
#define APP_TASK_PRIORITY           (1)

#define LED_BLINK_TIMER_CLOCK_HZ    (10000)

#define LED_BLINK_TIMER_PERIOD      (9999)

#define CALI_PIN                    P10_0

#define LED_OUT_PIN                 P10_1

static volatile bool timer_interrupt_flag = false;
static bool led_blink_active_flag = true;
static uint8_t uart_read_value;

static cyhal_timer_t led_blink_timer;

static const cy_stc_sysint_t capsense_isr_cfg =
{
    .intrSrc = csd_interrupt_IRQn,
    .intrPriority = 3u
};

static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CSD0, &cy_capsense_context);
}

static void capsense_settle(void)
{
    for (int i = 0; i < 5; i++)
    {
        (void)Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        while (Cy_CapSense_IsBusy(&cy_capsense_context))
        {
        }
        (void)Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
        cyhal_system_delay_ms(20);
    }
}

static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void)callback_arg;
    (void)event;
    timer_interrupt_flag = true;
}

static void timer_init(void)
{
    cy_rslt_t result;

    const cyhal_timer_cfg_t cfg =
    {
        .compare_value = 0,
        .period = LED_BLINK_TIMER_PERIOD,
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = false,
        .is_continuous = true,
        .value = 0
    };

    result = cyhal_timer_init(&led_blink_timer, NC, NULL);
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }

    cyhal_timer_configure(&led_blink_timer, &cfg);
    cyhal_timer_set_frequency(&led_blink_timer, LED_BLINK_TIMER_CLOCK_HZ);

    cyhal_timer_register_callback(&led_blink_timer, isr_timer, NULL);
    cyhal_timer_enable_event(&led_blink_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 7, true);
    cyhal_timer_start(&led_blink_timer);
}

static void CapSenseTask(void *arg)
{
    (void)arg;

    bool prev_touched = false;

    for (;;)
    {
        uint32_t saved = __get_BASEPRI();
        __set_BASEPRI(0u);

        (void)Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

        while (Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            taskYIELD();
        }

        __set_BASEPRI(saved);

        (void)Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

        bool touched = Cy_CapSense_IsWidgetActive(
            CY_CAPSENSE_BUTTON0_WDGT_ID,
            &cy_capsense_context);

        if (touched != prev_touched)
        {
            if (touched)
            {
                printf("CapSense PRESSED\r\n");
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
                cyhal_gpio_write(LED_OUT_PIN, 1u);
            }
            else
            {
                printf("CapSense RELEASED\r\n");
                cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
                cyhal_gpio_write(LED_OUT_PIN, 0u);
            }
            prev_touched = touched;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void CaliTask(void *arg)
{
    (void)arg;

    bool prev_pressed = false;

    for (;;)
    {
        bool pressed = !cyhal_gpio_read(CALI_PIN);

        if (pressed != prev_pressed)
        {
            if (pressed)
            {
                printf("Cali button PRESSED\r\n");
            }
            else
            {
                printf("Cali button RELEASED\r\n");
            }
            prev_pressed = pressed;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void AppTask(void *arg)
{
    (void)arg;

    for (;;)
    {
        if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 1) == CY_RSLT_SUCCESS)
        {
            if (uart_read_value == '\r')
            {
                if (led_blink_active_flag)
                {
                    cyhal_timer_stop(&led_blink_timer);
                    printf("LED blinking paused \r\n");
                }
                else
                {
                    cyhal_timer_start(&led_blink_timer);
                    printf("LED blinking resumed\r\n");
                }
                printf("\x1b[1F");
                led_blink_active_flag = !led_blink_active_flag;
            }
        }

        if (timer_interrupt_flag)
        {
            timer_interrupt_flag = false;
            cyhal_gpio_toggle(CYBSP_USER_LED);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    result = cy_retarget_io_init_fc(
        CYBSP_DEBUG_UART_TX,
        CYBSP_DEBUG_UART_RX,
        CYBSP_DEBUG_UART_CTS,
        CYBSP_DEBUG_UART_RTS,
        CY_RETARGET_IO_BAUDRATE);

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cyhal_gpio_init(
        CYBSP_USER_LED,
        CYHAL_GPIO_DIR_OUTPUT,
        CYHAL_GPIO_DRIVE_STRONG,
        CYBSP_LED_STATE_OFF);

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Cali button input */
    result = cyhal_gpio_init(
        CALI_PIN,
        CYHAL_GPIO_DIR_INPUT,
        CYHAL_GPIO_DRIVE_NONE,
        0u);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("Cali pin init failed\r\n");
        CY_ASSERT(0);
    }

    /* LED output on P9.1 — strong drive, starts OFF */
    result = cyhal_gpio_init(
        LED_OUT_PIN,
        CYHAL_GPIO_DIR_OUTPUT,
        CYHAL_GPIO_DRIVE_STRONG,
        0u);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("LED out pin init failed\r\n");
        CY_ASSERT(0);
    }

    printf("\x1b[2J\x1b[;H");
    printf("****************** CapSense + Cali + LED Example ****************** \r\n\n");
    printf("CapSense: Button0\r\n");
    printf("Cali pin: P10.0 (HIGH=idle, LOW=pressed)\r\n");
    printf("LED out:  P10.1 (driven HIGH on capsense touch)\r\n\n");

    init_cycfg_peripherals();

    cy_capsense_status_t cs;

    cs = Cy_CapSense_Init(&cy_capsense_context);
    if (cs != CY_CAPSENSE_STATUS_SUCCESS)
    {
        printf("Cy_CapSense_Init failed: %u\r\n", (unsigned)cs);
        CY_ASSERT(0);
    }

    Cy_SysInt_Init(&capsense_isr_cfg, capsense_isr);
    NVIC_ClearPendingIRQ(capsense_isr_cfg.intrSrc);
    NVIC_EnableIRQ(capsense_isr_cfg.intrSrc);

    uint32_t saved_basepri = __get_BASEPRI();
    __set_BASEPRI(0u);

    cs = Cy_CapSense_Enable(&cy_capsense_context);
    if (cs != CY_CAPSENSE_STATUS_SUCCESS)
    {
        printf("Cy_CapSense_Enable failed: %u\r\n", (unsigned)cs);
        CY_ASSERT(0);
    }

    capsense_settle();

    __set_BASEPRI(saved_basepri);

    printf("Ready — waiting for input...\r\n\n");

    timer_init();

    if (xTaskCreate(CapSenseTask, "CapSenseTask", CAPSENSE_TASK_STACK_SIZE, NULL,
                    CAPSENSE_TASK_PRIORITY, NULL) != pdPASS)
    {
        CY_ASSERT(0);
    }

    if (xTaskCreate(CaliTask, "CaliTask", CALI_TASK_STACK_SIZE, NULL,
                    CALI_TASK_PRIORITY, NULL) != pdPASS)
    {
        CY_ASSERT(0);
    }

    if (xTaskCreate(AppTask, "AppTask", APP_TASK_STACK_SIZE, NULL,
                    APP_TASK_PRIORITY, NULL) != pdPASS)
    {
        CY_ASSERT(0);
    }

    vTaskStartScheduler();

    for (;;)
    {
    }
}