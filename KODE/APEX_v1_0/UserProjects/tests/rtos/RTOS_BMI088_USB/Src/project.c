#include "project.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "vt100.h"

#include "peripherals/spi.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Task pool storage — pools are created in setup() before osKernelStart()
 * ========================================================================= */

TASK_POOL_ALLOCATE(TASK_BMI088_RtosTest);

/* =========================================================================
 * USB helpers
 * ========================================================================= */

static char log_buf[512];

static void usb_print(const char *s) {
    CDC_Transmit_FS((uint8_t *)s, (uint16_t)strlen(s));
    osDelay(2);
}

/* =========================================================================
 * Test orchestrator task
 * ========================================================================= */

void TASK_BMI088_RtosTest(void *argument) {
    TASK_BMI088_RtosTest_ARGS *args = (TASK_BMI088_RtosTest_ARGS *)argument;

    /* ------------------------------------------------------------------
     * 1. Wait for TASK_BMI088_Init to complete (bit 0 of init_done_flags)
     * ------------------------------------------------------------------ */
    uint32_t ev = osEventFlagsWait(args->init_done_flags, 0x01U,
                                   osFlagsWaitAny, 10000U);

    if (ev & 0x80000000U) {
        while (!cdc_port_open) { osDelay(10); }
        osDelay(50);
        usb_print(VT100_SCREEN_CLEAR);
        usb_print(VT100_FG_RED "FATAL: TASK_BMI088_Init timed out (10 s)\r\n" VT100_RESET);
        osThreadExit_Cstm();
    }

    if (*args->init_return_state != BMI_OK) {
        while (!cdc_port_open) { osDelay(10); }
        osDelay(50);
        usb_print(VT100_SCREEN_CLEAR);
        snprintf(log_buf, sizeof(log_buf),
                 VT100_FG_RED "FATAL: BMI088_Init returned %d\r\n" VT100_RESET,
                 (int)(*args->init_return_state));
        usb_print(log_buf);
        osThreadExit_Cstm();
    }

    /* ------------------------------------------------------------------
     * 2. Start background Acc + Gyr read tasks (they loop indefinitely,
     *    publishing measurements to their respective data topics).
     * ------------------------------------------------------------------ */
    static BMI_STATE acc_read_state = BMI_OK;
    static BMI_STATE gyr_read_state = BMI_OK;

    TASK_BMI088_ReadAcc_ARGS acc_args = {
        .imu          = &bmi088,
        .dt           = &bmi088_acc_topic,
        .return_state = &acc_read_state,
    };
    TASK_BMI088_ReadGyr_ARGS gyr_args = {
        .imu          = &bmi088,
        .dt           = &bmi088_gyr_topic,
        .return_state = &gyr_read_state,
    };

    osThreadAttr_t attr = { .priority = (osPriority_t)osPriorityNormal };

    attr.name = "BMI_ReadAcc";
    OS_THREAD_NEW_CSTM(TASK_BMI088_ReadAcc, acc_args, attr, osWaitForever);

    attr.name = "BMI_ReadGyr";
    OS_THREAD_NEW_CSTM(TASK_BMI088_ReadGyr, gyr_args, attr, osWaitForever);

    /* ------------------------------------------------------------------
     * 3. Warm-up: allow topics to get initialised on the first task tick
     * ------------------------------------------------------------------ */
    osDelay(300);

    /* ------------------------------------------------------------------
     * 4. Inject test context (sensor handle + live topic pointers)
     * ------------------------------------------------------------------ */
    BMI088_rtos_test_set_context(&bmi088, &bmi088_config,
                                 &bmi088_acc_topic, &bmi088_gyr_topic);

    /* ------------------------------------------------------------------
     * 5. Configure and run all 13 test cases
     * ------------------------------------------------------------------ */
    TEST_configure_cases(BMI088_rtos_test_cases, BMI088_RTOS_TEST_N_TESTS,
                         (const bool[]) {
                             true,   /* T0  ReadRegister_RTOS (lock=true) — ACC id  */
                             true,   /* T1  ReadRegister_RTOS (lock=true) — GYR id  */
                             true,   /* T2  WriteRegister_RTOS + Read roundtrip      */
                             true,   /* T3  ReadMultiple_RTOS — 6-byte ACC burst     */
                             true,   /* T4  ReadID_RTOS — both chip IDs              */
                             true,   /* T5  SoftReset_RTOS ACC + re-enable           */
                             true,   /* T6  SoftReset_RTOS GYR                       */
                             true,   /* T7  _NoLock bracket (manual sem acquire)     */
                             true,   /* T8  Concurrent ACC+GYR reads (2 tasks)       */
                             true,   /* T9  Semaphore serialization (block/unblock)  */
                             true,   /* T10 TASK_BMI088_ReadAcc data_topic coherence */
                             true,   /* T11 TASK_BMI088_ReadGyr data_topic coherence */
                             true,   /* T12 ApplyConfig_RTOS while tasks are running */
                         });

    for (int i = 0; i < BMI088_RTOS_TEST_N_TESTS; i++) {
        if (BMI088_rtos_test_cases[i].case_info.result != R_SKIP) {
            BMI088_rtos_test_cases[i].func(&BMI088_rtos_test_cases[i].case_info);
        }
    }

    /* ------------------------------------------------------------------
     * 6. Wait for USB serial monitor to open
     * ------------------------------------------------------------------ */
    while (!cdc_port_open) { osDelay(10); }
    osDelay(50);

    /* ------------------------------------------------------------------
     * 7. Print full test result table
     * ------------------------------------------------------------------ */
    usb_print(VT100_SCREEN_CLEAR);

    TEST_print_case_result(BMI088_rtos_test_cases, BMI088_RTOS_TEST_N_TESTS,
                           usb_print,
                           "BMI088 RTOS Driver Test",
                           "L1 primitives | race conditions | data_topic");

    /* ------------------------------------------------------------------
     * 8. Live data loop (ACC + GYR via data_topic subscriptions)
     * ------------------------------------------------------------------ */
    usb_print("\r\n" VT100_FG_CYAN "--- Live data (ACC / GYR) ---" VT100_RESET "\r\n");

    data_sub_t acc_sub = {0};
    data_sub_t gyr_sub = {0};

    if (bmi088_acc_topic) {
        data_sub_attach(&acc_sub, bmi088_acc_topic, DATA_ATTACH_FROM_NOW);
    }
    if (bmi088_gyr_topic) {
        data_sub_attach(&gyr_sub, bmi088_gyr_topic, DATA_ATTACH_FROM_NOW);
    }

    for (;;) {
        float3_t acc = {0.0f, 0.0f, 0.0f};
        float3_t gyr = {0.0f, 0.0f, 0.0f};

        if (bmi088_acc_topic) {
            data_sub_wait_for_data(&acc_sub, 200);
            data_sub_read(&acc_sub, &acc);
        }
        if (bmi088_gyr_topic) {
            data_sub_wait_for_data(&gyr_sub, 200);
            data_sub_read(&gyr_sub, &gyr);
        }

        snprintf(log_buf, sizeof(log_buf),
                 "  ACC  X=%+10.3f  Y=%+10.3f  Z=%+10.3f  m/s2\r\n"
                 "  GYR  X=%+10.3f  Y=%+10.3f  Z=%+10.3f  deg/s\r\n",
                 acc.x, acc.y, acc.z,
                 gyr.x, gyr.y, gyr.z);
        usb_print(log_buf);

        /* Overwrite the 2 live-data lines on the next iteration */
        usb_print(VT100_CURSOR_UP(2));
    }
}

/* =========================================================================
 * setup() — called between osKernelInitialize() and osKernelStart()
 * ========================================================================= */

void setup(void) {
    /* ---- Scheduler cleanup infrastructure (must be first) ---- */
    Init_cleanup();

    /* ---- BMI088 driver task pools ---- */
    TASK_POOL_CREATE(TASK_BMI088_Init);
    TASK_POOL_CREATE(TASK_BMI088_ReadAcc);
    TASK_POOL_CREATE(TASK_BMI088_ReadGyr);
    TASK_POOL_CREATE(TASK_BMI088_ReadTemp);    /* kept in pool even if unused */

    /* ---- Test helper task pools ---- */
    BMI088_rtos_test_init_pools();

    /* ---- Orchestrator task pool ---- */
    TASK_POOL_CREATE(TASK_BMI088_RtosTest);

    /* ---- Event flags for init synchronisation (static — never freed) ---- */
    static StaticEventGroup_t init_flags_cb;
    osEventFlagsAttr_t init_flags_attr = {
        .name    = "bmi088_init_flags",
        .cb_mem  = &init_flags_cb,
        .cb_size = sizeof(init_flags_cb),
    };
    osEventFlagsId_t init_done_flags = osEventFlagsNew(&init_flags_attr);

    /* ---- Static storage for init return status ---- */
    static BMI_STATE bmi_init_state = BMI_OK;

    /* ---- Launch TASK_BMI088_Init (async, runs after osKernelStart) ---- */
    TASK_BMI088_Init_ARGS init_args = {
        .imu          = &bmi088,
        .hspi         = &hspi1,
        .cs_acc_bank  = GPIOA,          /* PA4 — ACC chip-select */
        .cs_acc_pin   = GPIO_PIN_4,
        .cs_gyr_bank  = GPIOB,          /* PB2 — GYR chip-select */
        .cs_gyr_pin   = GPIO_PIN_2,
        .cfg          = &bmi088_config,
        .return_state = &bmi_init_state,
        .done_flags   = init_done_flags,
    };
    osThreadAttr_t attr = {
        .name     = "BMI088_Init",
        .priority = (osPriority_t)osPriorityAboveNormal,
    };
    OS_THREAD_NEW_CSTM(TASK_BMI088_Init, init_args, attr, osWaitForever);

    /* ---- Launch test orchestrator (waits for init, then runs tests) ---- */
    TASK_BMI088_RtosTest_ARGS orch_args = {
        .init_done_flags   = init_done_flags,
        .init_return_state = &bmi_init_state,
    };
    osThreadAttr_t orch_attr = {
        .name     = "BMI_RtosTest",
        .priority = (osPriority_t)osPriorityNormal,
    };
    OS_THREAD_NEW_CSTM(TASK_BMI088_RtosTest, orch_args, orch_attr, osWaitForever);
}

/* =========================================================================
 * loop() — unused in RTOS mode
 * ========================================================================= */

void loop(void) {
    /* All logic is in TASK_BMI088_RtosTest */
}
