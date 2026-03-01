#include "project.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "vt100.h"

/* ========================================================================
 * Utilitaires
 * ======================================================================== */
static void usb_print(const char *s) {
    CDC_Transmit_FS((uint8_t *)s, strlen(s));
	HAL_Delay(1);
}

static char log_buf[256];

/* ========================================================================
 * setup() — execution unique apres init des peripheriques
 * ======================================================================== */
void setup(void) {
	BMI088_test_set_context(&bmi088, &bmi088_config);

    TEST_configure_cases(BMI088_test_cases, BMI088_test_N_TESTS, (const bool[]) {
        true,	/* T0  chip IDs */
        true,	/* T1  ACC soft reset */
        true,	/* T2  GYR soft reset */
        true,	/* T3  ACC config R/W */
        true,	/* T4  GYR config R/W */
        true,	/* T5  ACC self-test */
        true,	/* T6  GYR BIST */
        false	/* T7  ACC temperature */
    });

    // Execute all tests sequentially and fill the results in the test cases.
    for (int i = 0; i < BMI088_test_N_TESTS; i++) {
        if (BMI088_test_cases[i].case_info.result != R_SKIP) {
            BMI088_test_cases[i].func(&BMI088_test_cases[i].case_info);
        }
    }

    /* Attend que le serial monitor soit ouvert cote PC (DTR=1).
     * Sans ca, les premiers caracteres seraient perdus avant
     * que le terminal ne soit pret a les recevoir.            */
    while (!cdc_port_open) {
        HAL_Delay(10);
    }
    HAL_Delay(50); /* stabilisation du terminal */

	const char title[32] = "BMI088 Sequential Driver Test";
	const char desc[128] = "IDs, reset, config R/W, self-test, temperature";

	TEST_print_case_result(BMI088_test_cases, BMI088_test_N_TESTS, usb_print, title, desc);
    
	/* En-tete fixe de la section live (imprime une seule fois) */
	usb_print("\r\n" VT100_FG_CYAN "--- Mesures live ---" VT100_RESET "\r\n");
}

/* ========================================================================
 * loop() — rapport periodique + mesures live
 *   - Tableau de tests : affiche une seule fois (premier appel).
 *   - Mesures live     : ecrase les deux lignes de donnees a chaque appel
 *                        via VT100_CURSOR_UP(2) sans toucher au reste.
 * ======================================================================== */
void loop(void) {

    /* --- Mesures live (mises a jour a chaque appel) --- */
    float3_t acc = {0}, gyr = {0};
	float temp = 0.0f;
    BMI088_ReadAcc(&bmi088, &acc);
    BMI088_ReadGyr(&bmi088, &gyr);
	BMI088_ReadTemp(&bmi088, &temp);

    snprintf(log_buf, sizeof(log_buf),
             "  ACC  X=%+10.3f  Y=%+10.3f  Z=%+10.3f  m/s2\r\n"
             "  GYR  X=%+10.3f  Y=%+10.3f  Z=%+10.3f  deg/s\r\n"
			 "  TEMP %+6.2f °C\r\n",
             acc.x, acc.y, acc.z,
             gyr.x, gyr.y, gyr.z, temp);
    usb_print(log_buf);

    HAL_Delay(10);

	/* Remonte de 2 lignes pour ecraser uniquement les valeurs ACC/GYR/TEMP */
	usb_print(VT100_CURSOR_UP(3));
}