#include "project.h"

#include "vt100.h"

#include <stdbool.h>

/* ========================================================================
 * setup() – execution unique apres init des peripheriques
 * ======================================================================== */
void setup(void) {

    TEST_configure_cases(DT_seq_test_cases, DT_seq_test_N_TESTS, (const bool[]) {
        true,   /* T0  NULL args                    */
        true,   /* T1  Pub/Read FIFO                */
        true,   /* T2  Empty read                   */
        true,   /* T3  Attach FROM_OLDEST           */
        true,   /* T4  Attach FROM_NOW              */
        true,   /* T5  num_to_read tracking         */
        true,   /* T6  Two independent subscribers  */
        true,   /* T7  Data loss detection          */
        true,   /* T8  Sync                         */
        true,   /* T9  Detach / Reattach            */
        true,   /* T10 Peek non-destructive         */
        true,   /* T11 REJECT_NEW refused           */
        true,   /* T12 Zero-copy ptr                */
        true,   /* T13 Free detaches                */
        true,   /* T14 Loss boundaries              */
        true,   /* T15 Loss seen by peek(idx > 0)   */
        true,   /* T16 Loss seen by _ptr accesses   */
        true,   /* T17 Peek beyond while lagging    */
        true,   /* T18 Detach from the middle       */
        true,   /* T19 pub_seq wrap-around          */
        true,   /* T20 OLDEST after wraps, cap 1    */
        true,   /* T21 Elem sizes + guards          */
        true,   /* T22 Attach refusals              */
        true,   /* T23 ISR publisher, torn read     */
        true,   /* T24 ISR publisher, loss count    */
        true,   /* T25 Detach after faulty re-init  */
    });

    /* Execute all tests sequentially and fill the results. */
    TEST_perform_cases(DT_seq_test_cases, DT_seq_test_N_TESTS);

    /* Results are printed once the host terminal is open (DTR=1). */
    TEST_wait_host();
    TEST_usb_print(VT100_SCREEN_CLEAR);

    const char suite_name[32]   = "DT Sequential Tests";
    const char suite_desc[128]  = "Suite de tests sequentiels pour data_topic";
    TEST_print_case_result(DT_seq_test_cases, DT_seq_test_N_TESTS,
                           TEST_usb_print, suite_name, suite_desc);
}

/* ========================================================================
 * loop() – rien a faire, les tests sont executes une seule fois dans setup()
 * ======================================================================== */
void loop(void) {

}
