#include "BMI088_seq_test.h"
#include "test.h"

#include <stdio.h>

// Pointer need to be set
static bmi088_t *bmi088 = NULL;
static const bmi_config_t *bmi088_config = NULL;

void BMI088_seq_test_set_context(bmi088_t *bmi, const bmi_config_t *config) {
    bmi088 = bmi;
    bmi088_config = config;
}

TEST_case_table_t BMI088_seq_test_cases[BMI088_seq_test_N_TESTS] = {
    { .case_info = { .name = "T0 Chip IDs" }       , .func = BMI088_seq_test_t0_chip_ids        },
    { .case_info = { .name = "T1 ACC SoftReset" }  , .func = BMI088_seq_test_t1_acc_soft_reset  },
    { .case_info = { .name = "T2 GYR SoftReset" }  , .func = BMI088_seq_test_t2_gyr_soft_reset  },
    { .case_info = { .name = "T3 ACC Config R/W" } , .func = BMI088_seq_test_t3_acc_config_rw   },
    { .case_info = { .name = "T4 GYR Config R/W" } , .func = BMI088_seq_test_t4_gyr_config_rw   },
    { .case_info = { .name = "T5 ACC SelfTest" }   , .func = BMI088_seq_test_t5_acc_self_test   },
    { .case_info = { .name = "T6 GYR BIST" }       , .func = BMI088_seq_test_t6_gyr_bist        },
    { .case_info = { .name = "T7 ACC Temperature" }, .func = BMI088_seq_test_t7_acc_temperature }
};

/* ========================================================================
 * Constantes de test
 * ======================================================================== */

/* Identifiants de puce attendus (datasheet Rev.1.9) */
#define BMI_ACC_CHIP_ID_EXP  BMI_ACC_CHIP_ID_VALUE   /* 0x1E */
#define BMI_GYR_CHIP_ID_EXP  0x0FU

/* Self-test accelerometre (procedure Bosch AN §4.4.1)
 *   - Plage obligatoire : ±24 g  →  1 LSB = 24 / 2^15 ≈ 0.000732 g
 *   - Seuil minimum : 1000 mg = 1366 LSB (Bosch AN p.12)             */
#define ACC_ST_MIN_LSB   1366

/* Self-test gyro (registre 0x3C) — bits datasheet
 *   bit 0 : trig_bist   bit 1 : bist_rdy
 *   bit 2 : bist_fail   bit 4 : rate_ok                              */
#define GYR_BIST_TRIG    0x01U
#define GYR_BIST_RDY     0x02U
#define GYR_BIST_FAIL    0x04U
#define GYR_BIST_OK      0x10U
#define GYR_BIST_TIMEOUT 200U   /* ms */

/* Plage de temperature valide */
#define TEMP_MIN_C  (-40.0f)
#define TEMP_MAX_C  ( 85.0f)

static const char *bmi_str(BMI_STATE s) {
    switch (s) {
        case BMI_OK:          return "OK";
        case BMI_SPI_ERR:     return "SPI_ERR";
        case BMI_INVALID_ARG: return "INVALID_ARG";
        case BMI_BUSY:        return "BUSY";
        case BMI_TIMEOUT:     return "TIMEOUT";
        case BMI_UNKNOWN_ERR: return "UNKNOWN_ERR";
        case BMI_SEM_ERR:     return "SEM_ERR";
        default:              return "?";
    }
}


void BMI088_seq_test_t0_chip_ids(TEST_case_t *tc) {
    uint8_t acc_id = 0, gyr_id = 0;
    BMI_STATE st = BMI088_ReadID(bmi088, &acc_id, &gyr_id);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ReadID: %s", bmi_str(st));
        return;
    }
    if (acc_id != BMI_ACC_CHIP_ID_EXP || gyr_id != BMI_GYR_CHIP_ID_EXP) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail),
                 "ACC=0x%02X(exp:0x%02X) GYR=0x%02X(exp:0x%02X)",
                 acc_id, BMI_ACC_CHIP_ID_EXP, gyr_id, BMI_GYR_CHIP_ID_EXP);
        return;
    }
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "ACC_ID=0x%02X GYR_ID=0x%02X", acc_id, gyr_id);
}


void BMI088_seq_test_t1_acc_soft_reset(TEST_case_t *tc) {
    BMI_STATE st = BMI088_SoftReset(bmi088, false /* ACC */);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "SoftReset: %s", bmi_str(st));
        return;
    }
    HAL_Delay(50);

    /* Apres reset : ACC est en SUSPEND ; il faut le remettre en ACTIVE    */
    BMI088_WriteRegister(bmi088, false, BMI_ACC_PWR_CTRL, (uint8_t)BMI_ACC_PWR_CTRL_ENABLE);
    HAL_Delay(5);
    BMI088_WriteRegister(bmi088, false, BMI_ACC_PWR_CONF, (uint8_t)BMI_ACC_PWR_CONF_ACTIVE);
    HAL_Delay(5);

    uint8_t id = 0;
    st = BMI088_ReadRegister(bmi088, false, BMI_ACC_CHIP_ID, &id);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ReadID post-reset: %s", bmi_str(st));
        return;
    }
    if (id != BMI_ACC_CHIP_ID_EXP) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail),
                 "ID=0x%02X apres reset (exp:0x%02X)", id, BMI_ACC_CHIP_ID_EXP);
        return;
    }
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "ID=0x%02X intact apres reset", id);
}


void BMI088_seq_test_t2_gyr_soft_reset(TEST_case_t *tc) {
    BMI_STATE st = BMI088_SoftReset(bmi088, true /* GYR */);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "SoftReset: %s", bmi_str(st));
        return;
    }
    HAL_Delay(30);

    uint8_t id = 0;
    st = BMI088_ReadRegister(bmi088, true, BMI_GYR_CHIP_ID, &id);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ReadID post-reset: %s", bmi_str(st));
        return;
    }
    if (id != BMI_GYR_CHIP_ID_EXP) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail),
                 "ID=0x%02X apres reset (exp:0x%02X)", id, BMI_GYR_CHIP_ID_EXP);
        return;
    }
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "ID=0x%02X intact apres reset", id);
}


void BMI088_seq_test_t3_acc_config_rw(TEST_case_t *tc) {
    /* Config a ecrire : +-12 g, ODR=200 Hz, BWP=OSR2 */
    const bmi_config_t test_cfg = {
        .acc_range = BMI_ACC_RANGE_12G,
        .acc_bwp   = BMI_ACC_CONF_BWP_OSR2,
        .acc_odr   = BMI_ACC_CONF_ODR_200_HZ,
        .acc_pwr   = BMI_ACC_PWR_CONF_ACTIVE,
        .acc_ctrl  = BMI_ACC_PWR_CTRL_ENABLE,
        /* GYR inchange : copie de la config nominale */
        .gyr_range = bmi088_config->gyr_range,
        .gyr_bw    = bmi088_config->gyr_bw,
        .gyr_mode  = bmi088_config->gyr_mode,
    };
    BMI_STATE st = BMI088_ApplyConfig(bmi088, &test_cfg);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ApplyConfig: %s", bmi_str(st));
        goto t3_restore;
    }
    HAL_Delay(2);

    uint8_t conf_r = 0, range_r = 0;
    st = BMI088_ReadRegister(bmi088, false, BMI_ACC_CONF,  &conf_r);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ReadConf: %s", bmi_str(st));
        goto t3_restore;
    }
    st = BMI088_ReadRegister(bmi088, false, BMI_ACC_RANGE, &range_r);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ReadRange: %s", bmi_str(st));
        goto t3_restore;
    }

    {
        const uint8_t conf_exp  = (uint8_t)BMI_ACC_CONF_BWP_OSR2 | (uint8_t)BMI_ACC_CONF_ODR_200_HZ;
        const uint8_t range_exp = (uint8_t)BMI_ACC_RANGE_12G;
        if (conf_r != conf_exp || range_r != range_exp) {
            tc->result = R_FAIL;
            snprintf(tc->detail, sizeof(tc->detail),
                     "CONF: got=0x%02X exp=0x%02X  RANGE: got=0x%02X exp=0x%02X",
                     conf_r, conf_exp, range_r, range_exp);
        } else {
            tc->result = R_PASS;
            snprintf(tc->detail, sizeof(tc->detail),
                     "ACC_CONF=0x%02X ACC_RANGE=0x%02X OK", conf_r, range_r);
        }
    }

t3_restore:
    BMI088_ApplyConfig(bmi088, bmi088_config);
    HAL_Delay(2);
}


void BMI088_seq_test_t4_gyr_config_rw(TEST_case_t *tc) {
    const bmi_config_t test_cfg = {
        /* ACC inchange */
        .acc_range = bmi088_config->acc_range,
        .acc_bwp   = bmi088_config->acc_bwp,
        .acc_odr   = bmi088_config->acc_odr,
        .acc_pwr   = bmi088_config->acc_pwr,
        .acc_ctrl  = bmi088_config->acc_ctrl,
        /* GYR : ±500 dps, BW=47 Hz */
        .gyr_range = BMI_GYR_RANGE_500,
        .gyr_bw    = BMI_GYR_BANDWIDTH_BW_47_HZ,
        .gyr_mode  = BMI_GYR_LPM1_MODE_NORMAL,
    };
    BMI_STATE st = BMI088_ApplyConfig(bmi088, &test_cfg);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ApplyConfig: %s", bmi_str(st));
        goto t4_restore;
    }
    HAL_Delay(2);

    uint8_t range_r = 0, bw_r = 0;
    st = BMI088_ReadRegister(bmi088, true, BMI_GYR_RANGE,     &range_r);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ReadRange: %s", bmi_str(st));
        goto t4_restore;
    }
    st = BMI088_ReadRegister(bmi088, true, BMI_GYR_BANDWIDTH, &bw_r);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ReadBW: %s", bmi_str(st));
        goto t4_restore;
    }

    {
        const uint8_t range_exp = (uint8_t)BMI_GYR_RANGE_500;
        const uint8_t bw_exp    = (uint8_t)BMI_GYR_BANDWIDTH_BW_47_HZ;

        range_r &= BMI_GYR_RANGE_MASK;
        bw_r    &= BMI_GYR_BANDWIDTH_BW_MASK;

        if (range_r != range_exp || bw_r != bw_exp) {
            tc->result = R_FAIL;
            snprintf(tc->detail, sizeof(tc->detail),
                     "RANGE: got=0x%02X exp=0x%02X  BW: got=0x%02X exp=0x%02X",
                     range_r, range_exp, bw_r, bw_exp);
        } else {
            tc->result = R_PASS;
            snprintf(tc->detail, sizeof(tc->detail),
                     "GYR_RANGE=0x%02X GYR_BW=0x%02X OK", range_r, bw_r);
        }
    }

t4_restore:
    BMI088_ApplyConfig(bmi088, bmi088_config);
    HAL_Delay(2);
}


void BMI088_seq_test_t5_acc_self_test(TEST_case_t *tc) {
    /* Config self-test : +-24g obligatoire, 1600 Hz, Normal           */
    const bmi_config_t st_cfg = {
        .acc_range = BMI_ACC_RANGE_24G,
        .acc_bwp   = BMI_ACC_CONF_BWP_NORMAL,
        .acc_odr   = BMI_ACC_CONF_ODR_1600_HZ,
        .acc_pwr   = BMI_ACC_PWR_CONF_ACTIVE,
        .acc_ctrl  = BMI_ACC_PWR_CTRL_ENABLE,
        .gyr_range = bmi088_config->gyr_range,
        .gyr_bw    = bmi088_config->gyr_bw,
        .gyr_mode  = bmi088_config->gyr_mode,
    };
    BMI_STATE st = BMI088_ApplyConfig(bmi088, &st_cfg);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ApplyConfig: %s", bmi_str(st));
        goto t5_cleanup;
    }
    HAL_Delay(2);

    /* Mesure positive */
    st = BMI088_WriteRegister(bmi088, false, BMI_ACC_SELF_TEST, (uint8_t)BMI_ACC_SELF_TEST_POS);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "WriteSTpos: %s", bmi_str(st));
        goto t5_cleanup;
    }
    HAL_Delay(50);

    uint8_t raw_p[6] = {0};
    st = BMI088_ReadMultiple(bmi088, false, BMI_ACC_X_LSB, raw_p, 6);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ReadAcc(pos): %s", bmi_str(st));
        goto t5_cleanup;
    }

    /* Mesure negative */
    st = BMI088_WriteRegister(bmi088, false, BMI_ACC_SELF_TEST, (uint8_t)BMI_ACC_SELF_TEST_NEG);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "WriteSTneg: %s", bmi_str(st));
        goto t5_cleanup;
    }
    HAL_Delay(50);

    uint8_t raw_n[6] = {0};
    st = BMI088_ReadMultiple(bmi088, false, BMI_ACC_X_LSB, raw_n, 6);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ReadAcc(neg): %s", bmi_str(st));
        goto t5_cleanup;
    }

    {
        const int16_t xp = (int16_t)((raw_p[1] << 8) | raw_p[0]);
        const int16_t yp = (int16_t)((raw_p[3] << 8) | raw_p[2]);
        const int16_t zp = (int16_t)((raw_p[5] << 8) | raw_p[4]);
        const int16_t xn = (int16_t)((raw_n[1] << 8) | raw_n[0]);
        const int16_t yn = (int16_t)((raw_n[3] << 8) | raw_n[2]);
        const int16_t zn = (int16_t)((raw_n[5] << 8) | raw_n[4]);

        const int32_t dx = (int32_t)xp - xn;
        const int32_t dy = (int32_t)yp - yn;
        const int32_t dz = (int32_t)zp - zn;

        if (dx < ACC_ST_MIN_LSB || dy < ACC_ST_MIN_LSB || dz < ACC_ST_MIN_LSB) {
            tc->result = R_FAIL;
            snprintf(tc->detail, sizeof(tc->detail),
                     "dX=%ld dY=%ld dZ=%ld (min=%d LSB)",
                     (long)dx, (long)dy, (long)dz, ACC_ST_MIN_LSB);
        } else {
            tc->result = R_PASS;
            snprintf(tc->detail, sizeof(tc->detail),
                     "dX=%ld dY=%ld dZ=%ld >= %d LSB OK",
                     (long)dx, (long)dy, (long)dz, ACC_ST_MIN_LSB);
        }
    }

t5_cleanup:
    BMI088_WriteRegister(bmi088, false, BMI_ACC_SELF_TEST, (uint8_t)BMI_ACC_SELF_TEST_OFF);
    HAL_Delay(50);
    BMI088_ApplyConfig(bmi088, bmi088_config);
    HAL_Delay(2);
}


void BMI088_seq_test_t6_gyr_bist(TEST_case_t *tc) {
    BMI_STATE st = BMI088_WriteRegister(bmi088, true, BMI_GYR_SELF_TEST, GYR_BIST_TRIG);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "TriggerBIST: %s", bmi_str(st));
        return;
    }

    uint32_t t_start = HAL_GetTick();
    uint8_t bist_reg = 0;
    do {
        HAL_Delay(5);
        st = BMI088_ReadRegister(bmi088, true, BMI_GYR_SELF_TEST, &bist_reg);
        if (st != BMI_OK) {
            tc->result = R_FAIL;
            snprintf(tc->detail, sizeof(tc->detail), "PollBIST: %s", bmi_str(st));
            return;
        }
    } while (!(bist_reg & GYR_BIST_RDY) && (HAL_GetTick() - t_start) < GYR_BIST_TIMEOUT);

    if (!(bist_reg & GYR_BIST_RDY)) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail),
                 "Timeout %u ms, reg=0x%02X", GYR_BIST_TIMEOUT, bist_reg);
        return;
    }
    if ((bist_reg & GYR_BIST_FAIL) || !(bist_reg & GYR_BIST_OK)) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail),
                 "rate_ok=%d bist_fail=%d (reg=0x%02X)",
                 (bist_reg & GYR_BIST_OK)   ? 1 : 0,
                 (bist_reg & GYR_BIST_FAIL)  ? 1 : 0,
                 bist_reg);
        return;
    }
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "rate_ok=1 bist_fail=0 (reg=0x%02X)", bist_reg);
}


void BMI088_seq_test_t7_acc_temperature(TEST_case_t *tc) {
    float temp_c = 0.0f;
    BMI_STATE st = BMI088_ReadTemp(bmi088, &temp_c);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ReadTemp: %s", bmi_str(st));
        return;
    }
    if (temp_c < TEMP_MIN_C || temp_c > TEMP_MAX_C) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail),
                 "T=%.1f C hors plage [%.0f, %.0f]", temp_c, TEMP_MIN_C, TEMP_MAX_C);
        return;
    }
    tc->result = R_PASS;
    /* evite -u _printf_float : conversion entiere */
    int32_t ti = (int32_t)temp_c;
    int32_t tf = (int32_t)((temp_c - (float)ti) * 10.0f);
    if (tf < 0) tf = -tf;
    snprintf(tc->detail, sizeof(tc->detail),
             "T=%ld.%ld C dans [%.0f, %.0f] OK",
             (long)ti, (long)tf, TEMP_MIN_C, TEMP_MAX_C);
}
