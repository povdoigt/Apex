#include "project.h"

#include "vt100.h"

#include <stdbool.h>

/* ========================================================================
 * setup() – execution unique apres init des peripheriques
 * ======================================================================== */
void setup(void) {

    TEST_configure_cases(CB_seq_test_cases, CB_seq_test_N_TESTS, (const bool[]) {
        true,   /* T0  NULL args                    */
        true,   /* T1  FIFO order                   */
        true,   /* T2  Empty read                   */
        true,   /* T3  Full REJECT_NEW              */
        true,   /* T4  Full OVERWRITE_OLDEST        */
        true,   /* T5  Reset                        */
        true,   /* T6  Peek absolute                */
        true,   /* T7  Peek relative                */
        true,   /* T8  Wrap-around                  */
        true,   /* T9  Float elem_size              */
        true,   /* T10 Fill/Drain cycle x2          */
        true,   /* T11 Capacity 1                   */
        true,   /* T12 Successive overwrites        */
        true,   /* T13 Leaving the full state       */
        true,   /* T14 Odd elem sizes + guards      */
        true,   /* T15 Pointer accessors            */
        true,   /* T16 cb_wrap extremes             */
        true,   /* T17 Remaining bad arguments      */
        true,   /* T18 Push in place                */
        true,   /* T19 Random vs reference model    */
        true,   /* T20 Push from ISR (TIM5)         */
        true,   /* T21 Copies vs ISR (TIM5)         */
    });

    /* Execute all tests sequentially and fill the results. */
    TEST_perform_cases(CB_seq_test_cases, CB_seq_test_N_TESTS);

    /* Results are printed once the host terminal is open (DTR=1). */
    TEST_wait_host();
    TEST_usb_print(VT100_SCREEN_CLEAR);

    const char suite_name[32]   = "CB Sequential Tests";
    const char suite_desc[128]  = "Suite de tests sequentiels pour circular_buffer";
    TEST_print_case_result(CB_seq_test_cases, CB_seq_test_N_TESTS,
                           TEST_usb_print, suite_name, suite_desc);
}

/* ========================================================================
 * loop() – rien a faire, les tests sont executes une seule fois dans setup()
 * ======================================================================== */
void loop(void) {

}
