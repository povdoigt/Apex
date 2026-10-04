#include "project.h"

#include "vt100.h"

#include <stdbool.h>
#include <stdio.h>

/* Init de la puce : couvre une ecriture non volatile des registres de statut. */
#define W25Q_INIT_TIMEOUT_MS    1000U

/* ========================================================================
 * setup() – execution unique dans le thread applicatif, kernel demarre
 * ======================================================================== */
void setup(void) {

    /* Init par la tache W25Q_Init (pool de la suite) : cree le semaphore de
       la puce. En cas d'echec, chaque cas le signale ("Contexte absent"). */
    W25Q_STATE init = W25Q_rtos_test_init_chip(&w25q, &w25q_config, W25Q_INIT_TIMEOUT_MS);
    W25Q_rtos_test_set_context(&w25q);

    TEST_configure_cases(W25Q_rtos_test_cases, W25Q_rtos_test_N_TESTS, (const bool[]) {
        /* A. Communication */
        true,   /* T0  ID Check               */
        true,   /* T1  Read Status            */
        /* B. Configuration */
        true,   /* T2  Cfg To Status          */
        true,   /* T3  Cfg Applied            */
        true,   /* T4  Cfg Invalid            */
        true,   /* T5  Cfg Keep               */
        /* C. Primitives commande */
        true,   /* T6  Cmd Invalid            */
        true,   /* T7  Write Enable Latch     */
        true,   /* T8  Addr Mode Tracking     */
        /* D. Effacement secteur */
        true,   /* T9  Erase Verify           */
        /* E. Lecture / ecriture */
        true,   /* T10 Aligned R/W            */
        true,   /* T11 Cross-page R/W         */
        true,   /* T12 Cross-sector R/W       */
        true,   /* T13 Unaligned R/W          */
        true,   /* T14 Multi-sector R/W       */
        true,   /* T15 AND without Erase      */
        true,   /* T16 R/W in 3B Mode         */
        /* F. Effacement etendu */
        true,   /* T17 Erase Addr Length      */
        true,   /* T18 Block Erase 32 KB      */
        true,   /* T19 Block Erase 64 KB      */
        true,   /* T20 Sector Isolation       */
        true,   /* T21 Busy Timeout           */
        /* G. Cas limites R/W */
        true,   /* T22 Write Size=0           */
        true,   /* T23 Write Near End         */
        true,   /* T24 Read Near End          */
        true,   /* T25 Addr Out Of Range      */
        /* H. Reset */
        true,   /* T26 Soft Reset             */
        /* I. Semaphore de la puce */
        true,   /* T27 Lock Error Paths       */
        true,   /* T28 NoLock Under Lock      */
        true,   /* T29 Lock Timeout           */
        true,   /* T30 Uninit Chip            */
        true,   /* T31 Re-init Concurrent     */
        /* J. Integration scheduler */
        true,   /* T32 Args Copy/Detached     */
        true,   /* T33 Join Timeout           */
        /* K. Concurrence */
        true,   /* T34 Concurrent Writes      */
        true,   /* T35 Concurrent Erases      */
        true,   /* T36 Read During Write      */
        true,   /* T37 Wait Shares Lock       */
        /* L. Ressources */
        true,   /* T38 Stack Headroom         */
    });

    /* Execute all tests sequentially, in this thread, and fill the results. */
    TEST_perform_cases(W25Q_rtos_test_cases, W25Q_rtos_test_N_TESTS);

    /* Results are printed once the host terminal is open (DTR=1). */
    TEST_wait_host();
    TEST_usb_print(VT100_SCREEN_CLEAR);

    const char suite_name[32] = "W25Q RTOS Tests";
    char suite_desc[128];
    snprintf(suite_desc, sizeof(suite_desc), "Suite de tests RTOS pour le driver W25Q (%u cas), init puce: %s",
             (unsigned)W25Q_rtos_test_N_TESTS, (init == W25Q_OK) ? "OK" : "ECHEC");
    TEST_print_case_result(W25Q_rtos_test_cases, W25Q_rtos_test_N_TESTS,
                           TEST_usb_print, suite_name, suite_desc);
}
