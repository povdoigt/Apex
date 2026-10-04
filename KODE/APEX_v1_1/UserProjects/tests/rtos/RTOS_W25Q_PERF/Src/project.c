#include "project.h"

#include "test.h"
#include "vt100.h"

/* Init de la puce : couvre une ecriture non volatile des registres de statut. */
#define W25Q_INIT_TIMEOUT_MS    1000U

/* ========================================================================
 * setup() – execution unique dans le thread applicatif, kernel demarre
 * ======================================================================== */
void setup(void) {

    /* Init par la tache W25Q_Init (pool du banc) : cree le semaphore de la puce. */
    W25Q_STATE init = W25Q_rtos_bench_init_chip(&w25q, &w25q_config, W25Q_INIT_TIMEOUT_MS);

    /* Mesures d'abord, impression ensuite : l'USB ne perturbe pas les chiffres. */
    if (init == W25Q_OK) {
        W25Q_rtos_bench_run(&w25q);
    }

    /* Results are printed once the host terminal is open (DTR=1). */
    TEST_wait_host();
    TEST_usb_print(VT100_SCREEN_CLEAR);

    W25Q_rtos_bench_print(TEST_usb_print, init);
}
