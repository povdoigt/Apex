#include "project.h"

#include "vt100.h"

#include <stdbool.h>
#include <stdio.h>

/* Init du capteur : couvre les resets (1 + 30 ms) et les changements de mode (2 x 5 ms). */
#define BMI088_INIT_TIMEOUT_MS  1000U

/* ========================================================================
 * setup() – execution unique dans le thread applicatif, kernel demarre
 * ======================================================================== */
void setup(void) {

    /* Init par la tache BMI088_Init (pool de la suite) : cree le semaphore du
       capteur. En cas d'echec, chaque cas le signale ("Contexte absent"). */
    BMI_STATE init = BMI088_rtos_test_init_imu(&bmi088, &bmi088_config, BMI088_INIT_TIMEOUT_MS);
    BMI088_rtos_test_set_context(&bmi088, &bmi088_config);

    TEST_configure_cases(BMI088_rtos_test_cases, BMI088_rtos_test_N_TESTS, (const bool[]) {
        /* A. Suite sequentielle transposee */
        true,   /* T0  Chip IDs               */
        true,   /* T1  ACC SoftReset          */
        true,   /* T2  GYR SoftReset          */
        true,   /* T3  ACC Config R/W         */
        true,   /* T4  GYR Config R/W         */
        true,   /* T5  ACC SelfTest           */
        true,   /* T6  GYR BIST               */
        true,   /* T7  ACC Temperature        */
        /* B. Init et parametres */
        true,   /* T8  Re-init / Init Args    */
        true,   /* T9  Uninit Sensor          */
        true,   /* T10 Invalid Args           */
        /* C. Semaphore du capteur */
        true,   /* T11 NoLock Under Lock      */
        true,   /* T12 Lock Timeout           */
        /* D. Tache BMI088_Acquire */
        true,   /* T13 Acquire Nominal        */
        true,   /* T14 Acquire Invalid        */
        true,   /* T15 Acquire Stall          */
        true,   /* T16 Acquire Lock Errors    */
        true,   /* T17 Acquire Subset/Stop    */
        /* E. Concurrence */
        true,   /* T18 Concurrent Readers     */
        true,   /* T19 Re-init Acquiring      */
        true,   /* T20 Range Chg Acquiring    */
        true,   /* T21 Reset Atomic           */
        /* F. Ressources */
        true,   /* T22 Stack Headroom         */
    });

    /* Execute all tests sequentially, in this thread, and fill the results. */
    TEST_perform_cases(BMI088_rtos_test_cases, BMI088_rtos_test_N_TESTS);

    /* Results are printed once the host terminal is open (DTR=1). */
    TEST_wait_host();
    TEST_usb_print(VT100_SCREEN_CLEAR);

    const char suite_name[32] = "BMI088 RTOS Tests";
    char suite_desc[128];
    snprintf(suite_desc, sizeof(suite_desc), "Suite de tests RTOS pour le driver BMI088 (%u cas), init capteur: %s",
             (unsigned)BMI088_rtos_test_N_TESTS, (init == BMI_OK) ? "OK" : "ECHEC");
    TEST_print_case_result(BMI088_rtos_test_cases, BMI088_rtos_test_N_TESTS,
                           TEST_usb_print, suite_name, suite_desc);
}
