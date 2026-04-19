#include "project.h"

#include "BMI088_seq_test.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "vt100.h"

#include <string.h>
#include <stdbool.h>
#include <sys/types.h>

static void usb_print(const char *s) {
    CDC_Transmit_FS((uint8_t *)s, strlen(s));
    HAL_Delay(1);
}

/* ========================================================================
 * setup() – exécution unique après l'init des périphériques
 * Chaque bloc #if correspond à une suite activée dans tests_config.h.
 * ======================================================================== */
void setup(void) {

    /* ------------------------------------------------------------------
     * Attente de l'ouverture du terminal série côté PC (DTR=1).
     * ------------------------------------------------------------------ */
    while (!cdc_port_open) {
        HAL_Delay(10);
    }
    HAL_Delay(50); /* stabilisation du terminal */

    usb_print(VT100_SCREEN_CLEAR);

    uint32_t total_tests = 0;
    uint32_t total_pass = 0;
    uint32_t total_fail = 0;

    /* ==================================================================
     * Suite : circular_buffer
     * ================================================================== */
#if (APEX_TEST_ENABLE_CB == 1)
    total_tests += CB_seq_test_N_TESTS;
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
    });
    TEST_perform_cases(CB_seq_test_cases, CB_seq_test_N_TESTS);
    TEST_get_pass_fail_count(CB_seq_test_cases, CB_seq_test_N_TESTS, &total_pass, &total_fail);
    const char cb_suite_name[32]   = "CB Sequential Tests";
    const char cb_suite_desc[128]  = "Suite de tests sequentiels pour circular_buffer (11 cas)";
    TEST_print_case_result(CB_seq_test_cases, CB_seq_test_N_TESTS, usb_print, cb_suite_name, cb_suite_desc);
	usb_print("\n");
#endif /* APEX_TEST_ENABLE_CB */

    /* ==================================================================
     * Suite : data_topic
     * ================================================================== */
#if (APEX_TEST_ENABLE_DT == 1)
    total_tests += DT_seq_test_N_TESTS;
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
        true,   /* T11 REJECT_NEW policy            */
    });
    TEST_perform_cases(DT_seq_test_cases, DT_seq_test_N_TESTS);
    TEST_get_pass_fail_count(DT_seq_test_cases, DT_seq_test_N_TESTS, &total_pass, &total_fail);
    const char dt_suite_name[32]   = "DT Sequential Tests";
    const char dt_suite_desc[128]  = "Suite de tests sequentiels pour data_topic";
    TEST_print_case_result(DT_seq_test_cases, DT_seq_test_N_TESTS, usb_print, dt_suite_name, dt_suite_desc);
	usb_print("\n");
#endif /* APEX_TEST_ENABLE_DT */

    /* ==================================================================
     * Suite : BMI088  (nécessite APEX_ENABLE_BMI088=1 + matériel)
     * ================================================================== */
#if (APEX_TEST_ENABLE_BMI088 == 1)
    total_tests += BMI088_seq_test_N_TESTS;
    BMI088_seq_test_set_context(&bmi088, &bmi088_config);
    TEST_configure_cases(BMI088_seq_test_cases, BMI088_seq_test_N_TESTS, (const bool[]) {
        true,   /* T0 */
        true,   /* T1 */
        true,   /* T2 */
        true,   /* T3 */
        true,   /* T4 */
        true,   /* T5 */
        true,   /* T6 */
        true,   /* T7 */
    });
    TEST_perform_cases(BMI088_seq_test_cases, BMI088_seq_test_N_TESTS);
    TEST_get_pass_fail_count(BMI088_seq_test_cases, BMI088_seq_test_N_TESTS, &total_pass, &total_fail);
    const char bmi088_suite_name[32]   = "BMI088 Sequential Tests";
    const char bmi088_suite_desc[128]  = "Suite de tests sequentiels pour BMI088 (8 cas)";
    TEST_print_case_result(BMI088_seq_test_cases, BMI088_seq_test_N_TESTS, usb_print, bmi088_suite_name, bmi088_suite_desc);
	usb_print("\n");
#endif /* APEX_TEST_ENABLE_BMI088 */

    /* ==================================================================
     * Suite : W25Q512  (nécessite APEX_ENABLE_W25Q512=1 + matériel)
     * ================================================================== */
#if (APEX_TEST_ENABLE_W25Q == 1)
    total_tests += W25Q_seq_test_N_TESTS;
    W25Q_seq_test_set_context(&w25q);
    TEST_configure_cases(W25Q_seq_test_cases, W25Q_seq_test_N_TESTS, (const bool[]) {
        true,	/* T0  */
        true,	/* T1  */
        true,	/* T2  */
        true,	/* T3  */
        true,	/* T4  */
        true,	/* T5  */
        true,	/* T6  */
        true,	/* T7  */
        true,	/* T8  */
        true,	/* T9  */
        true,	/* T10 */
        true,	/* T11 */
        true,	/* T12 */
        true,	/* T13 */
        true,	/* T14 */
    });
    TEST_perform_cases(W25Q_seq_test_cases, W25Q_seq_test_N_TESTS);
    TEST_get_pass_fail_count(W25Q_seq_test_cases, W25Q_seq_test_N_TESTS, &total_pass, &total_fail);
    const char w25q_suite_name[32]   = "W25Q Sequential Tests";
    const char w25q_suite_desc[128]  = "Suite de tests sequentiels pour W25Q512 (15 cas)";
    TEST_print_case_result(W25Q_seq_test_cases, W25Q_seq_test_N_TESTS,
                           usb_print, w25q_suite_name, w25q_suite_desc);
	usb_print("\n");
#endif /* APEX_TEST_ENABLE_W25Q */


    /* ==================================================================
     * Résumé global
     * ================================================================== */
    char log_buf[256];
	uint32_t total_skip = total_tests - total_pass - total_fail;
    snprintf(log_buf, sizeof(log_buf),
		VT100_FG_MAGENTA "===== Global Test Summary =====" VT100_RESET "\r\n"
                         "Total: %ld tests"
						 " %s  %ld passed  " VT100_RESET
						 " %s  %ld failed  " VT100_RESET
						 " %s  %ld skipped  " VT100_RESET "\r\n",
		total_tests,
		total_pass ? VT100_FG_BLACK VT100_BG_GREEN  : "", total_pass,
		total_fail ? VT100_FG_BLACK VT100_BG_RED    : "", total_fail,
		total_skip ? VT100_FG_BLACK VT100_BG_YELLOW : "", total_skip);
    usb_print(log_buf);
}

/* ========================================================================
 * loop() – rien à faire, les tests sont exécutés une seule fois dans setup()
 * ======================================================================== */
void loop(void) {

}
