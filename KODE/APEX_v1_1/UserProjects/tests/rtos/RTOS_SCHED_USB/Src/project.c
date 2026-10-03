#include "project.h"

#include "vt100.h"

#include <stdbool.h>

/* ========================================================================
 * setup() – execution unique dans le thread applicatif, kernel demarre
 * ======================================================================== */
void setup(void) {

    SCHED_rtos_test_init();

    TEST_configure_cases(SCHED_rtos_test_cases, SCHED_rtos_test_N_TESTS, (const bool[]) {
        true,   /* T0  Spawn/join/ret               */
        true,   /* T1  Args copied                  */
        true,   /* T2  Lazy creation                */
        true,   /* T3  Detached + ret               */
        true,   /* T4  Priorities                   */
        true,   /* T5  join_all                     */
        true,   /* T6  Nested jobs                  */
        true,   /* T7  Prewarm                      */
        true,   /* T8  Busy count/headroom          */
        true,   /* T9  Persistent                   */
        true,   /* T10 Spawn rejects                */
        true,   /* T11 Join rejects                 */
        true,   /* T12 Pool exhaustion              */
        true,   /* T13 Stale handle                 */
        true,   /* T14 Endurance x20000             */
        true,   /* T15 Join bit 30 (E1)             */
        true,   /* T16 Join timeout (E2)            */
        true,   /* T17 Both done (E8)               */
        true,   /* T18 join_all retry (E4)          */
        true,   /* T19 Join after reuse (E5)        */
        true,   /* T20 Bit in flight (E6)           */
        true,   /* T21 Prio bounds (F3)             */
        true,   /* T22 Wake no inversion (F1)       */
        true,   /* T23 Spawn latency (F2)           */
        true,   /* T24 Joiner sleeps                */
        true,   /* T25 Concurrent spawners (G1)     */
        true,   /* T26 ISR rejects (H1)             */
    });

    /* Execute all tests sequentially, in this thread, and fill the results. */
    TEST_perform_cases(SCHED_rtos_test_cases, SCHED_rtos_test_N_TESTS);

    /* Results are printed once the host terminal is open (DTR=1). */
    TEST_wait_host();
    TEST_usb_print(VT100_SCREEN_CLEAR);

    const char suite_name[32]  = "Scheduler RTOS Tests";
    const char suite_desc[128] = "Suite de tests RTOS pour la lib scheduler (27 cas)";
    TEST_print_case_result(SCHED_rtos_test_cases, SCHED_rtos_test_N_TESTS,
                           TEST_usb_print, suite_name, suite_desc);
}
