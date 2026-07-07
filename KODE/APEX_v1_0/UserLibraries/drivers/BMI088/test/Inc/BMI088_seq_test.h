#ifndef BMI088_SEQ_TEST_H
#define BMI088_SEQ_TEST_H

#include "BMI088.h"
#include "test.h"

#define BMI088_seq_test_N_TESTS 8

extern TEST_case_table_t BMI088_seq_test_cases[BMI088_seq_test_N_TESTS];


void BMI088_seq_test_set_context(bmi088_t *bmi, const bmi_config_t *config);


/* ========================================================================
 * T0 — Identifiants chip ACC + GYR (BMI088_ReadID)
 *   Lit les deux chip IDs en une seule transaction SPI (burst interne).
 *   Verifie ACC_CHIP_ID=0x1E et GYR_CHIP_ID=0x0F.
 * ======================================================================== */
void BMI088_seq_test_t0_chip_ids(TEST_case_t *tc);

/* ========================================================================
 * T1 — Soft reset accelerometre (BMI088_SoftReset)
 *   Envoie la commande de reset ACC (0xB6 @ 0x7E), attend ≥50 ms,
 *   reengage PWR_CTRL+PWR_CONF, puis relit le chip ID.
 * ======================================================================== */
void BMI088_seq_test_t1_acc_soft_reset(TEST_case_t *tc);

/* ========================================================================
 * T2 — Soft reset gyroscope (BMI088_SoftReset)
 *   Envoie 0xB6 @ 0x14, attend ≥30 ms (spec), relit GYR_CHIP_ID.
 * ======================================================================== */
void BMI088_seq_test_t2_gyr_soft_reset(TEST_case_t *tc);

/* ========================================================================
 * T3 — Ecriture / relecture config accelerometre (BMI088_ApplyConfig)
 *   Applique une config differente de la config nominale, relit les
 *   registres ACC_CONF (0x40) et ACC_RANGE (0x41) et compare.
 *   Restaure ensuite la config nominale (bmi088_config).
 * ======================================================================== */
void BMI088_seq_test_t3_acc_config_rw(TEST_case_t *tc);

/* ========================================================================
 * T4 — Ecriture / relecture config gyroscope (BMI088_ApplyConfig)
 *   Applique ±500 dps / BW_47_HZ, relit GYR_RANGE (0x0F) + GYR_BW (0x10).
 *   Restaure ensuite la config nominale.
 * ======================================================================== */
void BMI088_seq_test_t4_gyr_config_rw(TEST_case_t *tc);

/* ========================================================================
 * T5 — Self-test accelerometre (procedure Bosch AN §4.4.1)
 *   Config obligatoire : ±24 g, ODR=1600 Hz, BWP=Normal.
 *   1. Active mode positif (0x0D) → attend 50 ms → lit 6 octets bruts.
 *   2. Active mode negatif (0x09) → attend 50 ms → lit 6 octets bruts.
 *   3. delta = pos - neg ; doit depasser ACC_ST_MIN_LSB (1366) sur X, Y, Z.
 *   4. Desactive le self-test et restaure la config nominale dans tous les cas.
 * ======================================================================== */
void BMI088_seq_test_t5_acc_self_test(TEST_case_t *tc);

/* ========================================================================
 * T6 — Built-in self-test gyroscope (registre GYR_SELF_TEST 0x3C)
 *   Bits datasheet : [0]=trig_bist  [1]=bist_rdy  [2]=bist_fail  [4]=rate_ok
 *   1. Ecrit trig_bist=1.
 *   2. Attend que bist_rdy (bit 1) passe a 1 (timeout GYR_BIST_TIMEOUT ms).
 *   3. Verifie rate_ok (bit 4)=1 et bist_fail (bit 2)=0.
 * ======================================================================== */
void BMI088_seq_test_t6_gyr_bist(TEST_case_t *tc);

/* ========================================================================
 * T7 — Lecture de la temperature interne de l'accelerometre
 *   BMI088_ReadTemp lit les registres BMI_TEMP_MSB/LSB et applique
 *   la formule Bosch (t = raw * 0.125 + 23 °C).
 *   Critere : valeur dans [-40, +85] °C.
 * ======================================================================== */
void BMI088_seq_test_t7_acc_temperature(TEST_case_t *tc);

#endif /* BMI088_seq_test_H */
