#include "project.h"

#include "vt100.h"

#include <stdbool.h>

/* ========================================================================
 * setup() – execution unique apres init des peripheriques
 * ======================================================================== */
void setup(void) {

    TEST_configure_cases(DP_seq_test_cases, DP_seq_test_N_TESTS, (const bool[]) {
        true,   /* T0  Init arguments, init/free    */
        true,   /* T1  Nominal packet               */
        true,   /* T2  Time window                  */
        true,   /* T3  Copy publish, absent = 0     */
        true,   /* T4  Source freed while in use    */
        true,   /* T5  Concurrency with an ISR      */
    });

    /* Execute all tests sequentially and fill the results. */
    TEST_perform_cases(DP_seq_test_cases, DP_seq_test_N_TESTS);

    /* Results are printed once the host terminal is open (DTR=1). */
    TEST_wait_host();
    TEST_usb_print(VT100_SCREEN_CLEAR);

    const char suite_name[32]   = "DP Sequential Tests";
    const char suite_desc[128]  = "Suite de tests sequentiels pour data_packet";
    TEST_print_case_result(DP_seq_test_cases, DP_seq_test_N_TESTS,
                           TEST_usb_print, suite_name, suite_desc);
}

/* ========================================================================
 * loop() – rien a faire, les tests sont executes une seule fois dans setup()
 * ======================================================================== */
void loop(void) {

}
