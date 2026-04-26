/*******************************************************************************
 * Integrated IR Tracking System — Team 10
 *
 * Flow:
 *   - ToF always running, monitoring distance
 *   - CapSense touch → print "Calibration Start"
 *   - ToF < 50 mm   → print "System Start", activate IMU + AFE + PWM
 *   - ToF > threshold for N cycles → print "System Stop", deactivate
 *   - AFE/PWM feedback loop runs silently (no auto-print)
 *   - CLI "read" command → shows PWM duty, AFE value, IMU status
 ******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_sysint.h"
#include "cycfg_peripherals.h"
#include "cycfg_capsense.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

/* Sensor drivers */
#include "imu.h"
#include "vl53lx_api.h"
#include "vl53lx_platform.h"
#include "vl53lx_platform_init.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/*******************************************************************************
 * Pin configuration
 ******************************************************************************/
#define TOF_I2C_SCL         P10_0
#define TOF_I2C_SDA         P10_1
#define TOF_XSHUT           NC

#define PWM_LED_PIN         P9_6
#define PWM_DEFAULT_FREQ    1000u

#define AFE_ADC_PIN         P10_4

#define CALI_PIN            P5_4

/*******************************************************************************
 * Tuning
 ******************************************************************************/
#define TOF_ACTIVATE_MM         150     /* < 150 mm → System Start          */
#define TOF_DEACTIVATE_MM       150     /* > 150 mm for 1 s → System Stop   */
#define TOF_LEAVE_COUNT         10      /* 10 readings * 100ms = 1 second   */
#define TOF_BAD_COUNT_THRESH    20

#define AFE_TARGET_MV           2000
#define AFE_HYSTERESIS_MV       100
#define PWM_STEP                2.0f
#define PWM_MIN_DUTY            0.0f
#define PWM_MAX_DUTY            100.0f

/*******************************************************************************
 * System state
 ******************************************************************************/
static volatile bool system_active = false;   /* true = all modules running */

/*******************************************************************************
 * Shared sensor data (read by CLI "read" command)
 ******************************************************************************/
static volatile int32_t  g_tof_mm      = -1;
static volatile int32_t  g_afe_mv      = 0;
static volatile float    g_pwm_duty    = 0.0f;
static volatile bool     g_imu_running = false;

/*******************************************************************************
 * Hardware handles
 ******************************************************************************/
static VL53LX_Dev_t       tof_dev;
static cyhal_adc_t         adc_obj;
static cyhal_adc_channel_t adc_chan;
static cyhal_pwm_t         pwm_obj;

static SemaphoreHandle_t print_mutex;

#define SAFE_PRINTF(...) do { \
    xSemaphoreTake(print_mutex, portMAX_DELAY); \
    printf(__VA_ARGS__); \
    fflush(stdout); \
    xSemaphoreGive(print_mutex); \
} while (0)

/*******************************************************************************
 * CapSense
 ******************************************************************************/
static const cy_stc_sysint_t capsense_isr_cfg = {
    .intrSrc      = csd_interrupt_IRQn,
    .intrPriority = 3u
};

static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CSD0, &cy_capsense_context);
}

static void capsense_settle(void)
{
    for (int i = 0; i < 5; i++) {
        Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        while (Cy_CapSense_IsBusy(&cy_capsense_context)) {}
        Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
        cyhal_system_delay_ms(20);
    }
}

/*******************************************************************************
 * TASK: CapSense — touch → print "Calibration Start"
 ******************************************************************************/
static void capsense_task(void *pv)
{
    (void)pv;
    bool prev = false;

    for (;;) {
        uint32_t saved = __get_BASEPRI();
        __set_BASEPRI(0u);
        Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        while (Cy_CapSense_IsBusy(&cy_capsense_context)) { taskYIELD(); }
        __set_BASEPRI(saved);

        Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

        bool touched = Cy_CapSense_IsWidgetActive(
                           CY_CAPSENSE_BUTTON0_WDGT_ID,
                           &cy_capsense_context);

        if (touched && !prev) {
            SAFE_PRINTF("Calibration Start\r\n");
        }
        prev = touched;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*******************************************************************************
 * ToF reinit (brownout recovery)
 ******************************************************************************/
static void tof_reinit(void)
{
    VL53LX_StopMeasurement(&tof_dev);
    vTaskDelay(pdMS_TO_TICKS(50));

    VL53LX_WrByte(&tof_dev, 0x0000, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    VL53LX_WrByte(&tof_dev, 0x0000, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    if (VL53LX_WaitDeviceBooted(&tof_dev) != VL53LX_ERROR_NONE) return;
    if (VL53LX_DataInit(&tof_dev) != VL53LX_ERROR_NONE)         return;
    VL53LX_PerformRefSpadManagement(&tof_dev);
    VL53LX_SetDistanceMode(&tof_dev, VL53LX_DISTANCEMODE_LONG);
    VL53LX_SetMeasurementTimingBudgetMicroSeconds(&tof_dev, 50000);
    if (VL53LX_StartMeasurement(&tof_dev) != VL53LX_ERROR_NONE) return;
}

/*******************************************************************************
 * TASK: ToF — always running, controls system_active
 ******************************************************************************/
static void tof_task(void *pv)
{
    (void)pv;
    uint32_t bad_cnt  = 0;
    uint32_t far_cnt  = 0;

    for (;;) {
        uint8_t ready = 0;
        VL53LX_Error st = VL53LX_GetMeasurementDataReady(&tof_dev, &ready);
        if (st != VL53LX_ERROR_NONE) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (ready) {
            VL53LX_MultiRangingData_t data;
            st = VL53LX_GetMultiRangingData(&tof_dev, &data);
            if (st != VL53LX_ERROR_NONE) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            if (data.NumberOfObjectsFound == 0 ||
                data.RangeData[0].RangeMilliMeter >= 8190)
            {
                bad_cnt++;
                g_tof_mm = -1;
                if (bad_cnt >= TOF_BAD_COUNT_THRESH) {
                    tof_reinit();
                    bad_cnt = 0;
                }
            } else {
                bad_cnt = 0;
                g_tof_mm = data.RangeData[0].RangeMilliMeter;
            }

            VL53LX_ClearInterruptAndStartMeasurement(&tof_dev);

            /* ── Activate/deactivate logic ── */
            if (!system_active) {
                /* Object close → System Start */
                if (g_tof_mm > 0 && g_tof_mm < TOF_ACTIVATE_MM) {
                    system_active = true;
                    far_cnt = 0;

                    /* Start modules */
                    imu_capture_start_seconds(3600);
                    g_imu_running = true;
                    g_pwm_duty = 50.0f;
                    cyhal_pwm_set_duty_cycle(&pwm_obj, g_pwm_duty, PWM_DEFAULT_FREQ);
                    cyhal_pwm_start(&pwm_obj);

                    SAFE_PRINTF("System Start\r\n");
                }
            } else {
                /* Object left → count up, then System Stop */
                if (g_tof_mm < 0 || g_tof_mm > TOF_DEACTIVATE_MM) {
                    far_cnt++;
                    if (far_cnt >= TOF_LEAVE_COUNT) {
                        system_active = false;

                        imu_capture_stop();
                        g_imu_running = false;
                        cyhal_pwm_stop(&pwm_obj);
                        g_pwm_duty = 0.0f;

                        far_cnt = 0;
                        SAFE_PRINTF("System Stop\r\n");
                    }
                } else {
                    far_cnt = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*******************************************************************************
 * TASK: IMU — init sensor, runs silently when active
 ******************************************************************************/
static void imu_wrapper_task(void *pv)
{
    task_imu(pv);   /* loops forever inside imu.c */
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

/*******************************************************************************
 * TASK: AFE + PWM feedback — runs silently when active
 ******************************************************************************/
static void afe_pwm_task(void *pv)
{
    (void)pv;

    for (;;) {
        if (!system_active) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Read IR sensor */
        int32_t mv = cyhal_adc_read_uv(&adc_chan) / 1000;
        g_afe_mv = mv;

        /* Closed-loop: keep IR near target */
        int32_t err = mv - AFE_TARGET_MV;
        if (err > AFE_HYSTERESIS_MV) {
            g_pwm_duty -= PWM_STEP;
            if (g_pwm_duty < PWM_MIN_DUTY) g_pwm_duty = PWM_MIN_DUTY;
            cyhal_pwm_set_duty_cycle(&pwm_obj, g_pwm_duty, PWM_DEFAULT_FREQ);
        } else if (err < -AFE_HYSTERESIS_MV) {
            g_pwm_duty += PWM_STEP;
            if (g_pwm_duty > PWM_MAX_DUTY) g_pwm_duty = PWM_MAX_DUTY;
            cyhal_pwm_set_duty_cycle(&pwm_obj, g_pwm_duty, PWM_DEFAULT_FREQ);
        }

        vTaskDelay(pdMS_TO_TICKS(200));   /* 5 Hz */
    }
}

/*******************************************************************************
 * CLI — mode-based menu
 ******************************************************************************/
#define MAX_INPUT_LEN 64

#define MODE_MENU  0
#define MODE_IR    1
#define MODE_IMU   2

static volatile int cli_mode = MODE_MENU;
static void uart_drain(void);

static void cli_enter_mode_banner(int mode, uint32_t seconds, const char *label)
{
    /* Print the banner BEFORE flipping mode so live_print_task can't squeeze
     * a status line in between. No uart_drain here — calling getc after
     * mode flip would deadlock against live_print_task on the HAL lock. */
    SAFE_PRINTF("-- %s Mode: printing for %lu s --\r\n",
                label, (unsigned long)seconds);
    cli_mode = mode;
}

static void print_menu(void)
{
    SAFE_PRINTF("========== MENU ==========\r\n");
    SAFE_PRINTF("  ir <sec>   - IR tracking, auto-exit after <sec> seconds\r\n");
    SAFE_PRINTF("  imu <sec>  - IMU motion, auto-exit after <sec> seconds\r\n");
    SAFE_PRINTF("  stop       - Force system stop\r\n");
    SAFE_PRINTF("==========================\r\n");
    SAFE_PRINTF("> ");
}

/* Parse "<cmd> <num>" — returns true if matches and writes seconds. */
static bool parse_cmd_seconds(const char *in, const char *cmd, uint32_t *out)
{
    size_t n = strlen(cmd);
    if (strncmp(in, cmd, n) != 0) return false;
    const char *p = in + n;
    /* allow "imu10s", "imu 10", "imu 10s" */
    while (*p == ' ' || *p == '\t') p++;
    if (*p < '0' || *p > '9') return false;
    uint32_t v = 0;
    while (*p >= '0' && *p <= '9') { v = v * 10 + (uint32_t)(*p - '0'); p++; }
    *out = v;
    return true;
}

/* Drain any garbage from UART RX. cli_task is the SOLE UART user — no
 * other task calls cyhal_uart_getc/putc, so no extra locking needed. */
static void uart_drain(void)
{
    uint8_t c;
    while (cyhal_uart_getc(&cy_retarget_io_uart_obj, &c, 0) == CY_RSLT_SUCCESS) {}
}

/* Print one IR/IMU status line. Called from cli_task only — single owner. */
static void print_one_status_line(int mode)
{
    if (mode == MODE_IR) {
        long afe  = (long)(cyhal_adc_read_uv(&adc_chan) / 1000);
        long duty = (long)g_pwm_duty;
        int32_t err = (int32_t)(afe - AFE_TARGET_MV);
        g_afe_mv = (int32_t)afe;

        const char *tag = "IR OK   ";
        if (err > AFE_HYSTERESIS_MV)       tag = "IR HIGH ";
        else if (err < -AFE_HYSTERESIS_MV) tag = "IR LOW  ";
        SAFE_PRINTF("%s afe=%ld mV  pwm=%ld%%  active=%d\r\n",
                    tag, afe, duty, system_active ? 1 : 0);
    } else if (mode == MODE_IMU) {
        SAFE_PRINTF("Motion: %s  imu=%d  active=%d\r\n",
                    (const char *)g_motion_state,
                    g_imu_running ? 1 : 0,
                    system_active ? 1 : 0);
    }
}

/* Run a timed mode entirely inside cli_task — no other task touches UART
 * during the window, so there is zero contention possible. */
static void run_timed_mode(int mode, uint32_t seconds, const char *label)
{
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(seconds * 1000U);

    cli_enter_mode_banner(mode, seconds, label);
    SAFE_PRINTF("[DBG] loop start, end_tick=%lu now=%lu\r\n",
                (unsigned long)end, (unsigned long)xTaskGetTickCount());

    /* Print status every 500 ms until the deadline. */
    while ((int32_t)(xTaskGetTickCount() - end) < 0) {
        print_one_status_line(mode);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    SAFE_PRINTF("[DBG] loop exited, now=%lu\r\n",
                (unsigned long)xTaskGetTickCount());

    /* No uart_drain here — calling cyhal_uart_getc right after a printf
     * burst can hang the SCB UART. Whatever the user typed is left in
     * the FIFO; the menu input loop below will consume it. */
    cli_mode = MODE_MENU;
    SAFE_PRINTF("\r\n-- Time up, back to menu --\r\n");
    print_menu();
    SAFE_PRINTF("[DBG] cleanup done\r\n");
}

/*
 * cli_task: SOLE owner of UART RX. live_print_task only prints — never reads.
 * Timed-mode auto-exit is handled by run_timed_mode via plain vTaskDelay.
 */
static void cli_task(void *pv)
{
    (void)pv;
    char    in[MAX_INPUT_LEN];
    uint8_t idx = 0;

    print_menu();

    for (;;) {
        /* ---- Menu input ---- */
        uint8_t c;
        if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &c, 0) == CY_RSLT_SUCCESS) {
            if (c == '\r' || c == '\n') {
                SAFE_PRINTF("\r\n");
                if (idx > 0) {
                    in[idx] = '\0';
                    idx = 0;

                    uint32_t secs = 0;
                    if (parse_cmd_seconds(in, "ir", &secs) && secs > 0) {
                        run_timed_mode(MODE_IR, secs, "IR");
                    } else if (parse_cmd_seconds(in, "imu", &secs) && secs > 0) {
                        if (!g_imu_running) {
                            imu_capture_start_seconds(secs + 5U);
                            g_imu_running = true;
                        }
                        run_timed_mode(MODE_IMU, secs, "IMU");
                    } else if (strcmp(in, "stop") == 0) {
                        system_active = false;
                        imu_capture_stop();
                        g_imu_running = false;
                        cyhal_pwm_stop(&pwm_obj);
                        g_pwm_duty = 0.0f;
                        SAFE_PRINTF("System Stop (forced)\r\n> ");
                    } else {
                        SAFE_PRINTF("Unknown command. Use: ir <sec> | imu <sec> | stop\r\n");
                        print_menu();
                    }
                } else {
                    SAFE_PRINTF("> ");
                }
            } else if ((c == '\b' || c == 127) && idx > 0) {
                idx--;
                SAFE_PRINTF("\b \b");
            } else if (idx < MAX_INPUT_LEN - 1 && c >= 0x20) {
                in[idx++] = (char)c;
                SAFE_PRINTF("%c", c);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/*******************************************************************************
 * Hardware init
 ******************************************************************************/
static void init_adc(void)
{
    cy_rslt_t r;
    r = cyhal_adc_init(&adc_obj, AFE_ADC_PIN, NULL);
    CY_ASSERT(r == CY_RSLT_SUCCESS);

    const cyhal_adc_channel_config_t ch_cfg = {
        .enable_averaging   = false,
        .min_acquisition_ns = 1000u,
        .enabled            = true
    };
    r = cyhal_adc_channel_init_diff(&adc_chan, &adc_obj,
                                     AFE_ADC_PIN, CYHAL_ADC_VNEG,
                                     &ch_cfg);
    CY_ASSERT(r == CY_RSLT_SUCCESS);
}

static void init_pwm(void)
{
    cy_rslt_t r = cyhal_pwm_init(&pwm_obj, PWM_LED_PIN, NULL);
    CY_ASSERT(r == CY_RSLT_SUCCESS);
    cyhal_pwm_set_duty_cycle(&pwm_obj, 0.0f, PWM_DEFAULT_FREQ);
    cyhal_pwm_start(&pwm_obj);
}

static void init_tof(void)
{
    VL53LX_Error st;

    tof_dev.pin_scl   = TOF_I2C_SCL;
    tof_dev.pin_sda   = TOF_I2C_SDA;
    tof_dev.pin_xshut = TOF_XSHUT;

    st = VL53LX_platform_init(&tof_dev, 0x29, 0, 100);
    if (st) { printf("[ToF] platform_init err: %d\r\n", st); CY_ASSERT(0); }

    printf("[ToF] Waiting for boot...\r\n");
    st = VL53LX_WaitDeviceBooted(&tof_dev);
    if (st) { printf("[ToF] boot err: %d\r\n", st); CY_ASSERT(0); }

    st = VL53LX_DataInit(&tof_dev);
    if (st) { printf("[ToF] DataInit err: %d\r\n", st); CY_ASSERT(0); }

    printf("[ToF] RefSPAD calibration...\r\n");
    st = VL53LX_PerformRefSpadManagement(&tof_dev);
    if (st) { printf("[ToF] RefSPAD warning: %d (continuing)\r\n", st); }

    VL53LX_SetDistanceMode(&tof_dev, VL53LX_DISTANCEMODE_LONG);
    VL53LX_SetMeasurementTimingBudgetMicroSeconds(&tof_dev, 50000);

    st = VL53LX_StartMeasurement(&tof_dev);
    if (st) { printf("[ToF] StartMeasurement err: %d\r\n", st); CY_ASSERT(0); }

    printf("[ToF] Ready.\r\n");
}

static void init_capsense(void)
{
    init_cycfg_peripherals();

    cy_capsense_status_t cs = Cy_CapSense_Init(&cy_capsense_context);
    CY_ASSERT(cs == CY_CAPSENSE_STATUS_SUCCESS);

    Cy_SysInt_Init(&capsense_isr_cfg, capsense_isr);
    NVIC_ClearPendingIRQ(capsense_isr_cfg.intrSrc);
    NVIC_EnableIRQ(capsense_isr_cfg.intrSrc);

    uint32_t saved = __get_BASEPRI();
    __set_BASEPRI(0u);

    cs = Cy_CapSense_Enable(&cy_capsense_context);
    CY_ASSERT(cs == CY_CAPSENSE_STATUS_SUCCESS);

    capsense_settle();
    __set_BASEPRI(saved);

    printf("[CAP] Ready.\r\n");
}

/*******************************************************************************
 * main
 ******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    __enable_irq();

    cy_retarget_io_init(CYBSP_DEBUG_UART_TX,
                        CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Clear terminal: reset attributes, clear screen, cursor home */
    printf("\033[0m\033[2J\033[H");

    printf("\r\n========================================\r\n");
    printf("  Team 10 - IR Tracking System\r\n");
    printf("========================================\r\n\r\n");
    printf("[BOOT] BSP/UART OK\r\n");

    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    cyhal_gpio_init(CALI_PIN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_PULLUP, 1u);

    print_mutex = xSemaphoreCreateMutex();
    configASSERT(print_mutex != NULL);

    printf("[BOOT] init_pwm...\r\n");
    init_pwm();
    printf("[BOOT] init_pwm done\r\n");

    printf("[BOOT] init_adc...\r\n");
    init_adc();
    printf("[BOOT] init_adc done\r\n");

    printf("[BOOT] init_capsense...\r\n");
    init_capsense();
    printf("[BOOT] init_capsense done\r\n");

    printf("[BOOT] init_tof...\r\n");
    init_tof();
    printf("[BOOT] init_tof done\r\n");

    /* CLI is now mode-based, no FreeRTOS CLI commands needed */

    printf("[BOOT] create tasks...\r\n");
    configASSERT(xTaskCreate(tof_task,         "ToF", configMINIMAL_STACK_SIZE * 8,
                NULL, 4, NULL) == pdPASS);

    configASSERT(xTaskCreate(capsense_task,    "CAP", configMINIMAL_STACK_SIZE * 4,
                NULL, 3, NULL) == pdPASS);

    configASSERT(xTaskCreate(imu_wrapper_task, "IMU", configMINIMAL_STACK_SIZE * 8,
                NULL, 2, NULL) == pdPASS);

    configASSERT(xTaskCreate(afe_pwm_task,     "AFE", configMINIMAL_STACK_SIZE * 4,
                NULL, 2, NULL) == pdPASS);

    configASSERT(xTaskCreate(cli_task,         "CLI", configMINIMAL_STACK_SIZE * 8,
                NULL, 1, NULL) == pdPASS);

    printf("[BOOT] scheduler start\r\n");
    vTaskStartScheduler();

    for (;;) {}
}
