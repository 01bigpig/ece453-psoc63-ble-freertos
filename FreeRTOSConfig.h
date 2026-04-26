/*
 * FreeRTOS Kernel V10.6.2
 * Integrated IR Tracking System — Team 10
 *
 * Based on Team10_06_Cap_Cali config, with increased heap for all modules.
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "cy_utils.h"
#include "cycfg_system.h"

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
extern uint32_t SystemCoreClock;
#define configCPU_CLOCK_HZ                      SystemCoreClock
#define configTICK_RATE_HZ                      1000u
#define configMAX_PRIORITIES                    7
#define configMINIMAL_STACK_SIZE                128
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               10
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5

/* Memory — 64 KB heap for all sensor tasks + driver buffers */
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   (64 * 1024)
#define configAPPLICATION_ALLOCATED_HEAP        0

/* CLI output buffer */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE       256

/* Hooks */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* Stats */
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* Co-routines */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

/* Timers */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               3
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE * 2)

/* Interrupt priorities (PSoC 6: __NVIC_PRIO_BITS = 3) */
#define configKERNEL_INTERRUPT_PRIORITY         0xFF
#if defined(CY_XMC4XXX_DEVICES)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    0x04
#else
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    0x20
#endif
#define configMAX_API_CALL_INTERRUPT_PRIORITY   configMAX_SYSCALL_INTERRUPT_PRIORITY

/* API includes */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xResumeFromISR                  1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     0
#define INCLUDE_xTaskGetIdleTaskHandle          0
#define INCLUDE_eTaskGetState                   0
#define INCLUDE_xEventGroupSetBitFromISR        1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 0
#define INCLUDE_xTaskGetHandle                  0
#define INCLUDE_xTaskResumeFromISR              1

/* Assert */
#if defined(NDEBUG)
#define configASSERT(x) CY_UNUSED_PARAMETER(x)
#else
#define configASSERT(x) if ((x) == 0) { taskDISABLE_INTERRUPTS(); CY_HALT(); }
#endif

/* CMSIS handler names */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* Heap scheme — heap_4 for flexible alloc/free */
#define HEAP_ALLOCATION_TYPE1  (1)
#define HEAP_ALLOCATION_TYPE2  (2)
#define HEAP_ALLOCATION_TYPE3  (3)
#define HEAP_ALLOCATION_TYPE4  (4)
#define HEAP_ALLOCATION_TYPE5  (5)
#define NO_HEAP_ALLOCATION     (0)

#define configHEAP_ALLOCATION_SCHEME  (HEAP_ALLOCATION_TYPE4)

/* Tickless idle */
#if defined(CY_CFG_PWR_SYS_IDLE_MODE) && \
    ((CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_SLEEP) || \
     (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP))
extern void vApplicationSleep(uint32_t xExpectedIdleTime);
#define portSUPPRESS_TICKS_AND_SLEEP(xIdleTime) vApplicationSleep(xIdleTime)
#define configUSE_TICKLESS_IDLE 2
#else
#define configUSE_TICKLESS_IDLE 0
#endif

#if (CY_CFG_PWR_DEEPSLEEP_LATENCY > 0)
#define configEXPECTED_IDLE_TIME_BEFORE_SLEEP CY_CFG_PWR_DEEPSLEEP_LATENCY
#endif

/* Newlib reentrancy */
#if defined(__llvm__) && !defined(__ARMCC_VERSION)
#define configUSE_PICOLIBC_TLS  1
#else
#define configUSE_NEWLIB_REENTRANT 1
#endif

#endif /* FREERTOS_CONFIG_H */
