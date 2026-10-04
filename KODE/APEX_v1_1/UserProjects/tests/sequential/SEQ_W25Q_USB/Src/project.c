#include "project.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "vt100.h"



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
        /* A. Communication */
        true,	/* T0  ID Check */
        true,	/* T1  Read Status */
        /* B. Configuration */
        true,	/* T2  Cfg To Status */
        true,	/* T3  Cfg Applied */
        true,	/* T4  Cfg Invalid */
        true,	/* T5  Cfg Keep */
        /* C. Primitives commande */
        true,	/* T6  Cmd Invalid */
        true,	/* T7  Write Enable Latch */
        true,	/* T8  Addr Mode Tracking */
        /* D. Effacement secteur */
        true,	/* T9  Erase Verify */
        /* E. Lecture / ecriture */
        true,	/* T10 Aligned R/W */
        true,	/* T11 Cross-page R/W */
        true,	/* T12 Cross-sector R/W */
        true,	/* T13 Unaligned R/W */
        true,	/* T14 Multi-sector R/W */
        true,	/* T15 AND without Erase */
        true,	/* T16 R/W in 3B Mode */
        /* F. Effacement etendu */
        true,	/* T17 Erase Addr Length */
        true,	/* T18 Block Erase 32 KB */
        true,	/* T19 Block Erase 64 KB */
        true,	/* T20 Sector Isolation */
        true,	/* T21 Busy Timeout */
        /* G. Cas limites R/W */
        true,	/* T22 Write Size=0 */
        true,	/* T23 Write Near End */
        true,	/* T24 Read Near End */
        true,	/* T25 Addr Out Of Range */
        /* H. Reset */
        true,	/* T26 Soft Reset */
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

    usb_print(VT100_SCREEN_CLEAR);

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
