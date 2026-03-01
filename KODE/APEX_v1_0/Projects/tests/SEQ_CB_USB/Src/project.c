#include "project.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <string.h>
#include <stdbool.h>

static void usb_print(const char *s) {
    CDC_Transmit_FS((uint8_t *)s, strlen(s));
    HAL_Delay(1);
}

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
    });

    /* Execute all tests sequentially and fill the results. */
    TEST_perform_cases(CB_seq_test_cases, CB_seq_test_N_TESTS);

    /* Attend que le serial monitor soit ouvert cote PC (DTR=1).
     * Sans ca, les premiers caracteres seraient perdus avant
     * que le terminal ne soit pret a les recevoir.            */
    while (!cdc_port_open) {
        HAL_Delay(10);
    }
    HAL_Delay(50); /* stabilisation du terminal */

    /* Print the results via USB CDC. */
    const char suite_name[32]   = "CB Sequential Tests";
    const char suite_desc[128]  = "Suite de tests sequentiels pour circular_buffer (11 cas)";
    TEST_print_case_result(CB_seq_test_cases, CB_seq_test_N_TESTS,
                           usb_print, suite_name, suite_desc);
}

/* ========================================================================
 * loop() – rien a faire, les tests sont executes une seule fois dans setup()
 * ======================================================================== */
void loop(void) {

}
