#ifndef BMI088_TEST_COMMON_H
#define BMI088_TEST_COMMON_H

/*
 * Elements partages par les suites sequentielle (BMI088_seq_test) et RTOS
 * (BMI088_rtos_test) : constantes, configurations de test et verifications.
 *
 * Les deux suites portent la meme numerotation T0-T7, appliquent les memes
 * configurations et les memes criteres : un test RTOS qui echoue alors que son
 * jumeau sequentiel passe designe la couche RTOS (DMA, semaphore, taches).
 *
 * A n'inclure que depuis un fichier de test.
 */

#include "BMI088.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* ========================================================================
 * Constantes de test
 * ======================================================================== */

/* Identifiants de puce attendus (datasheet Rev.1.9) */
#define BMI_ACC_CHIP_ID_EXP  BMI_ACC_CHIP_ID_VALUE   /* 0x1E */
#define BMI_GYR_CHIP_ID_EXP  BMI_GYR_CHIP_ID_VALUE   /* 0x0F */

/* Self-test accelerometre (procedure Bosch AN §4.4.1)
 *   - Plage obligatoire : ±24 g  →  1 LSB = 24 / 2^15 ≈ 0.000732 g
 *   - Seuil minimum : 1000 mg = 1366 LSB (Bosch AN p.12)             */
#define ACC_ST_MIN_LSB   1366
#define ACC_ST_SETTLE_MS 50U    /* attente apres chaque changement de mode */

/* Self-test gyro (registre 0x3C) — bits datasheet
 *   bit 0 : trig_bist   bit 1 : bist_rdy
 *   bit 2 : bist_fail   bit 4 : rate_ok                              */
#define GYR_BIST_TRIG    0x01U
#define GYR_BIST_RDY     0x02U
#define GYR_BIST_FAIL    0x04U
#define GYR_BIST_OK      0x10U
#define GYR_BIST_TIMEOUT 200U   /* ms */
#define GYR_BIST_POLL_MS 5U

/* Plage de temperature valide */
#define TEMP_MIN_C  (-40.0f)
#define TEMP_MAX_C  ( 85.0f)

/* Carte immobile : norme de l'acceleration autour de 1 g, rotation faible. */
#if (BMI_ACCEL_UNIT_MS2)
#define ACC_G_NOMINAL   9.81f
#else
#define ACC_G_NOMINAL   1.0f
#endif
#define ACC_G_MIN       (0.75f * ACC_G_NOMINAL)
#define ACC_G_MAX       (1.25f * ACC_G_NOMINAL)
#if (BMI_GYRO_UNIT_DPS)
#define GYR_REST_MAX    10.0f              /* deg/s */
#else
#define GYR_REST_MAX    (10.0f * 0.0174533f) /* rad/s */
#endif

/* ========================================================================
 * Configurations de test : derivees de la configuration nominale, seule la
 * partie testee change.
 * ======================================================================== */

/* T3 : ACC +-12 g, ODR 200 Hz, BWP OSR2 ; GYR nominal. */
static inline bmi_reg_config_t bmi_test_cfg_acc(const bmi_reg_config_t *nom) {
    bmi_reg_config_t c = *nom;
    c.acc_range = BMI_ACC_RANGE_12G;
    c.acc_bwp   = BMI_ACC_CONF_BWP_OSR2;
    c.acc_odr   = BMI_ACC_CONF_ODR_200_HZ;
    c.acc_pwr   = BMI_ACC_PWR_CONF_ACTIVE;
    c.acc_ctrl  = BMI_ACC_PWR_CTRL_ENABLE;
    return c;
}
#define BMI_TEST_ACC_CONF_EXP   ((uint8_t)BMI_ACC_CONF_BWP_OSR2 | (uint8_t)BMI_ACC_CONF_ODR_200_HZ)
#define BMI_TEST_ACC_RANGE_EXP  ((uint8_t)BMI_ACC_RANGE_12G)

/* T4 : GYR +-500 dps, BW 47 Hz ; ACC nominal. */
static inline bmi_reg_config_t bmi_test_cfg_gyr(const bmi_reg_config_t *nom) {
    bmi_reg_config_t c = *nom;
    c.gyr_range = BMI_GYR_RANGE_500;
    c.gyr_bw    = BMI_GYR_BANDWIDTH_BW_47_HZ;
    c.gyr_mode  = BMI_GYR_LPM1_MODE_NORMAL;
    return c;
}
#define BMI_TEST_GYR_RANGE_EXP  ((uint8_t)BMI_GYR_RANGE_500)
#define BMI_TEST_GYR_BW_EXP     ((uint8_t)BMI_GYR_BANDWIDTH_BW_47_HZ)

/* T5 : configuration imposee par le self-test ACC (+-24 g, 1600 Hz, Normal). */
static inline bmi_reg_config_t bmi_test_cfg_selftest(const bmi_reg_config_t *nom) {
    bmi_reg_config_t c = *nom;
    c.acc_range = BMI_ACC_RANGE_24G;
    c.acc_bwp   = BMI_ACC_CONF_BWP_NORMAL;
    c.acc_odr   = BMI_ACC_CONF_ODR_1600_HZ;
    c.acc_pwr   = BMI_ACC_PWR_CONF_ACTIVE;
    c.acc_ctrl  = BMI_ACC_PWR_CTRL_ENABLE;
    return c;
}

/* ========================================================================
 * Verifications
 * ======================================================================== */

static inline const char *bmi_str(BMI_STATE s) {
    switch (s) {
        case BMI_OK:           return "OK";
        case BMI_SPI_ERR:      return "SPI_ERR";
        case BMI_INVALID_ARG:  return "INVALID_ARG";
        case BMI_BUSY:         return "BUSY";
        case BMI_TIMEOUT:      return "TIMEOUT";
        case BMI_UNKNOWN_ERR:  return "UNKNOWN_ERR";
        case BMI_SEM_ERR:      return "SEM_ERR";
        case BMI_ID_ERR:       return "ID_ERR";
        case BMI_CFG_ERR:      return "CFG_ERR";
        case BMI_LOCK_TIMEOUT: return "LOCK_TIMEOUT";
        default:               return "?";
    }
}

/* T5 : ecart positif - negatif sur chaque axe, en LSB. Vrai si les trois
   axes depassent ACC_ST_MIN_LSB. */
static inline bool bmi_selftest_delta(const uint8_t raw_p[6], const uint8_t raw_n[6], int32_t d[3]) {
    for (int k = 0; k < 3; k++) {
        const int16_t p = (int16_t)((raw_p[2 * k + 1] << 8) | raw_p[2 * k]);
        const int16_t n = (int16_t)((raw_n[2 * k + 1] << 8) | raw_n[2 * k]);
        d[k] = (int32_t)p - n;
    }
    return d[0] >= ACC_ST_MIN_LSB && d[1] >= ACC_ST_MIN_LSB && d[2] >= ACC_ST_MIN_LSB;
}

/* T6 : registre GYR_SELF_TEST final -> BIST reussi. */
static inline bool bmi_bist_ok(uint8_t reg) {
    return (reg & GYR_BIST_RDY) && !(reg & GYR_BIST_FAIL) && (reg & GYR_BIST_OK);
}

static inline float bmi_norm(const float3_t *v) {
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

/* Temperature au dixieme, sans printf flottant : "-12.3". */
static inline void bmi_fmt_temp(char *buf, unsigned n, float t) {
    int32_t d = (int32_t)(t * 10.0f + (t >= 0.0f ? 0.5f : -0.5f));
    const char *sign = (d < 0) ? "-" : "";
    if (d < 0) d = -d;
    snprintf(buf, n, "%s%ld.%ld", sign, (long)(d / 10), (long)(d % 10));
}

#endif /* BMI088_TEST_COMMON_H */
