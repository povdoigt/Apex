#include "project.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "vt100.h"

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

    /* Execute all tests sequentially and fill the results. */
    TEST_perform_cases(DT_seq_test_cases, DT_seq_test_N_TESTS);

    /* Attend que le serial monitor soit ouvert cote PC (DTR=1).
     * Sans ca, les premiers caracteres seraient perdus avant
     * que le terminal ne soit pret a les recevoir.            */
    while (!cdc_port_open) {
        HAL_Delay(10);
    }
    HAL_Delay(50); /* stabilisation du terminal */

    usb_print(VT100_SCREEN_CLEAR);

    /* Print the results via USB CDC. */
    const char suite_name[32]   = "DT Sequential Tests";
    const char suite_desc[128]  = "Suite de tests sequentiels pour data_topic (12 cas)";
    TEST_print_case_result(DT_seq_test_cases, DT_seq_test_N_TESTS,
                           usb_print, suite_name, suite_desc);
}

/* ========================================================================
 * loop() – rien a faire, les tests sont executes une seule fois dans setup()
 * ======================================================================== */
void loop(void) {

}
