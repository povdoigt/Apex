#include "project.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"



static void usb_print(const char *s) {
    CDC_Transmit_FS((uint8_t *)s, strlen(s));
    HAL_Delay(1);
}



/* ========================================================================
 * setup() – execution unique apres init des peripheriques
 * ======================================================================== */
void setup(void) {
    W25Q_seq_test_set_context(&w25q);

    TEST_configure_cases(W25Q_seq_test_cases, W25Q_seq_test_N_TESTS, (const bool[]) {
        true,	/* T0  Write zero size */
        true,	/* T1  Unaligned RW */
        true,	/* T2  AND behavior */
        true,	/* T3  Erase limits */
        true,	/* T4  Write limits */
        true,	/* T5  Read limits */
        true,	/* T6  Robustesse unaligned RW */
        true,	/* T7  Robustesse multi-sector RW */
		true,	/* T8  Read status */
		true,	/* T9  Erase 32 KB */
		true,	/* T10 Erase 64 KB */
		true,	/* T11 Soft reset */
		true,	/* T12 Multi-sector write */
		true,	/* T13 End of flash clamp */
		true	/* T14 Write size=0 */
    });

	// Execute all tests sequentially and fill the results in the test cases.
	TEST_perform_cases(W25Q_seq_test_cases, W25Q_seq_test_N_TESTS);

    /* Attend que le serial monitor soit ouvert cote PC (DTR=1).
     * Sans ca, les premiers caracteres seraient perdus avant
     * que le terminal ne soit pret a les recevoir.            */
    while (!cdc_port_open) {
        HAL_Delay(10);
    }
    HAL_Delay(50); /* stabilisation du terminal */

	// Print the results of all test cases using the usb_print function.
	const char suite_name[32] = "W25Q Sequential Tests";
	const char suite_desc[128] = "Suite de tests sequenciels pour le driver W25Q";
	TEST_print_case_result(W25Q_seq_test_cases, W25Q_seq_test_N_TESTS, usb_print, suite_name, suite_desc);
}

/* ========================================================================
 * loop() – Do nothing more as tests are done in setup() and results printed there.
 * ======================================================================== */
void loop(void) {

}
