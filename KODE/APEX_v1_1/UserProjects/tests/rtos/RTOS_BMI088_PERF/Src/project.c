#include "project.h"

#include "test.h"
#include "vt100.h"

/* Init du capteur : couvre les resets (1 + 30 ms) et les changements de mode (2 x 5 ms). */
#define BMI088_INIT_TIMEOUT_MS  1000U

/* ========================================================================
 * setup() – execution unique dans le thread applicatif, kernel demarre
 * ======================================================================== */
void setup(void) {

    /* Init par la tache BMI088_Init (pool du banc) : cree le semaphore du capteur. */
    BMI_STATE init = BMI088_rtos_bench_init_imu(&bmi088, &bmi088_config, BMI088_INIT_TIMEOUT_MS);

    /* Mesures d'abord, impression ensuite : l'USB ne perturbe pas les chiffres. */
    if (init == BMI_OK) {
        BMI088_rtos_bench_run(&bmi088, &bmi088_config);
    }

    /* Results are printed once the host terminal is open (DTR=1). */
    TEST_wait_host();
    TEST_usb_print(VT100_SCREEN_CLEAR);

    BMI088_rtos_bench_print(TEST_usb_print, init);
}
