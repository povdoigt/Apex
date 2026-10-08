#include "project.h"

#include "vt100.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdio.h>

/* Every case of every table is enabled. */
static const bool all_enabled[32] = {
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true,
};

/* ========================================================================
 * setup_pre_kernel() – avant osKernelStart(), aucune tache ne tourne
 * ======================================================================== */
void setup_pre_kernel(void) {
    DT_rtos_test_pre_kernel();
}

/* ========================================================================
 * setup() – execution unique dans le thread applicatif, kernel demarre
 *
 * Les suites sequentielles (circular_buffer, data_topic, data_packet) sont
 * rejouees telles quelles sous RTOS : sections critiques par BASEPRI,
 * semaphores crees et supprimes par attach / detach. Puis la suite propre
 * au RTOS (concurrence entre taches, notification, avant noyau).
 * ======================================================================== */
void setup(void) {

    DT_rtos_test_init();

    _Static_assert(DT_seq_test_N_TESTS <= 32 && CB_seq_test_N_TESTS <= 32 &&
                   DP_seq_test_N_TESTS <= 32 && DT_rtos_test_N_TESTS <= 32, "all_enabled too short");
    TEST_configure_cases(DT_seq_test_cases,  DT_seq_test_N_TESTS,  all_enabled);
    TEST_configure_cases(CB_seq_test_cases,  CB_seq_test_N_TESTS,  all_enabled);
    TEST_configure_cases(DP_seq_test_cases,  DP_seq_test_N_TESTS,  all_enabled);
    TEST_configure_cases(DT_rtos_test_cases, DT_rtos_test_N_TESTS, all_enabled);

    /* Execute all tests sequentially, in this thread, and fill the results. */
    TEST_perform_cases(DT_rtos_test_cases, DT_rtos_test_N_TESTS);
    TEST_perform_cases(DT_seq_test_cases,  DT_seq_test_N_TESTS);
    TEST_perform_cases(CB_seq_test_cases,  CB_seq_test_N_TESTS);
    TEST_perform_cases(DP_seq_test_cases,  DP_seq_test_N_TESTS);

    /* Stack never used by this thread over the whole run (bytes). */
    const unsigned long stack_free = (unsigned long)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

    /* Results are printed once the host terminal is open (DTR=1). */
    TEST_wait_host();
    TEST_usb_print(VT100_SCREEN_CLEAR);

    /* TEST_print_case_result lit 32 / 128 octets : tableaux de cette taille. */
    const char rtos_name[32] = "DT RTOS Tests",    rtos_desc[128] = "Suite RTOS de data_topic et circular_buffer";
    const char dt_name[32]   = "DT Seq sous RTOS", dt_desc[128]   = "Suite sequentielle de data_topic, rejouee sous RTOS";
    const char cb_name[32]   = "CB Seq sous RTOS", cb_desc[128]   = "Suite sequentielle de circular_buffer, rejouee sous RTOS";
    const char dp_name[32]   = "DP Seq sous RTOS", dp_desc[128]   = "Suite sequentielle de data_packet, rejouee sous RTOS";
    TEST_print_case_result(DT_rtos_test_cases, DT_rtos_test_N_TESTS, TEST_usb_print, rtos_name, rtos_desc);
    TEST_print_case_result(DT_seq_test_cases,  DT_seq_test_N_TESTS,  TEST_usb_print, dt_name,   dt_desc);
    TEST_print_case_result(CB_seq_test_cases,  CB_seq_test_N_TESTS,  TEST_usb_print, cb_name,   cb_desc);
    TEST_print_case_result(DP_seq_test_cases,  DP_seq_test_N_TESTS,  TEST_usb_print, dp_name,   dp_desc);

    char line[96];
    snprintf(line, sizeof(line), "\r\nPile du thread de test jamais utilisee : %lu octets\r\nEND_OF_REPORT\r\n",
             stack_free);
    TEST_usb_print(line);
}
