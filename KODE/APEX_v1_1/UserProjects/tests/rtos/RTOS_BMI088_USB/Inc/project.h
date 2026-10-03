#ifndef PROJECT_H
#define PROJECT_H

#include "main_config.h"
#include "drivers_config.h"
#include "scheduler.h"

/* =========================================================================
 * Test orchestrator task
 * =========================================================================
 * Runs after osKernelStart:
 *   1. Waits for TASK_BMI088_Init to complete (via done_flags, bit 0).
 *   2. Starts TASK_BMI088_ReadAcc and TASK_BMI088_ReadGyr background tasks.
 *   3. Lets them warm up for 200 ms so their data topics are populated.
 *   4. Runs the full BMI088_rtos_test suite (T0 – T12).
 *   5. Waits for USB port to open, then prints results.
 *   6. Loops printing live ACC / GYR data every 100 ms.
 * ========================================================================= */

typedef struct TASK_BMI088_RtosTest_ARGS {
    osEventFlagsId_t  init_done_flags;      /**< Set by TASK_BMI088_Init on completion (bit 0) */
    BMI_STATE        *init_return_state;    /**< Pointer to the state written by TASK_BMI088_Init */
} TASK_BMI088_RtosTest_ARGS;

TASK_POOL_CONFIGURE(TASK_BMI088_RtosTest, 1, 3072);
void TASK_BMI088_RtosTest(void *argument);

/* =========================================================================
 * Standard project entry points
 * ========================================================================= */

void setup(void);
void loop(void);

#endif // PROJECT_H
