#include "BMI088_seq_test.h"
#include "BMI088_test_common.h"
#include "test.h"
#include "tools.h"

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

void BMI088_seq_test_t0_chip_ids(TEST_case_t *tc) {
    uint8_t acc_id = 0, gyr_id = 0;
    BMI_STATE st = BMI088_ReadID(bmi088, &acc_id, &gyr_id);
    TEST_ASSERT(st == BMI_OK, "ReadID: %s", bmi_str(st));
    TEST_ASSERT(acc_id == BMI_ACC_CHIP_ID_EXP && gyr_id == BMI_GYR_CHIP_ID_EXP,
                "ACC=0x%02X(exp:0x%02X) GYR=0x%02X(exp:0x%02X)",
                acc_id, BMI_ACC_CHIP_ID_EXP, gyr_id, BMI_GYR_CHIP_ID_EXP);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "ACC_ID=0x%02X GYR_ID=0x%02X", acc_id, gyr_id);
}


void BMI088_seq_test_t1_acc_soft_reset(TEST_case_t *tc) {
    /* SoftReset attend la fin du reset et repasse l'ACC en SPI (lecture fictive). */
    BMI_STATE st = BMI088_SoftReset(bmi088, false /* ACC */);
    TEST_ASSERT(st == BMI_OK, "SoftReset: %s", bmi_str(st));

    uint8_t id = 0;
    st = BMI088_ReadRegister(bmi088, false, BMI_ACC_CHIP_ID, &id);
    TEST_ASSERT(st == BMI_OK, "ReadID post-reset: %s", bmi_str(st));
    TEST_ASSERT(id == BMI_ACC_CHIP_ID_EXP, "ID=0x%02X apres reset (exp:0x%02X)", id, BMI_ACC_CHIP_ID_EXP);

    /* Le reset remet la config ACC a ses valeurs par defaut : on la reapplique. */
    st = BMI088_ApplyConfig(bmi088, &bmi088_config->reg);
    TEST_ASSERT(st == BMI_OK, "ApplyConfig apres reset: %s", bmi_str(st));
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "ID=0x%02X apres reset, config reappliquee", id);
}


void BMI088_seq_test_t2_gyr_soft_reset(TEST_case_t *tc) {
    BMI_STATE st = BMI088_SoftReset(bmi088, true /* GYR */);
    TEST_ASSERT(st == BMI_OK, "SoftReset: %s", bmi_str(st));

    uint8_t id = 0;
    st = BMI088_ReadRegister(bmi088, true, BMI_GYR_CHIP_ID, &id);
    TEST_ASSERT(st == BMI_OK, "ReadID post-reset: %s", bmi_str(st));
    TEST_ASSERT(id == BMI_GYR_CHIP_ID_EXP, "ID=0x%02X apres reset (exp:0x%02X)", id, BMI_GYR_CHIP_ID_EXP);

    st = BMI088_ApplyConfig(bmi088, &bmi088_config->reg);
    TEST_ASSERT(st == BMI_OK, "ApplyConfig apres reset: %s", bmi_str(st));
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "ID=0x%02X apres reset, config reappliquee", id);
}


void BMI088_seq_test_t3_acc_config_rw(TEST_case_t *tc) {
    /* Config a ecrire : +-12 g, ODR=200 Hz, BWP=OSR2 ; GYR inchange */
    const bmi_reg_config_t test_cfg = bmi_test_cfg_acc(&bmi088_config->reg);
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
        const uint8_t conf_exp  = BMI_TEST_ACC_CONF_EXP;
        const uint8_t range_exp = BMI_TEST_ACC_RANGE_EXP;
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
    BMI088_ApplyConfig(bmi088, &bmi088_config->reg);
    HAL_Delay(2);
}


void BMI088_seq_test_t4_gyr_config_rw(TEST_case_t *tc) {
    /* GYR : ±500 dps, BW=47 Hz ; ACC inchange */
    const bmi_reg_config_t test_cfg = bmi_test_cfg_gyr(&bmi088_config->reg);
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
        const uint8_t range_exp = BMI_TEST_GYR_RANGE_EXP;
        const uint8_t bw_exp    = BMI_TEST_GYR_BW_EXP;

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
    BMI088_ApplyConfig(bmi088, &bmi088_config->reg);
    HAL_Delay(2);
}


void BMI088_seq_test_t5_acc_self_test(TEST_case_t *tc) {
    /* Config self-test : +-24g obligatoire, 1600 Hz, Normal */
    const bmi_reg_config_t st_cfg = bmi_test_cfg_selftest(&bmi088_config->reg);
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
    HAL_Delay(ACC_ST_SETTLE_MS);

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
    HAL_Delay(ACC_ST_SETTLE_MS);

    uint8_t raw_n[6] = {0};
    st = BMI088_ReadMultiple(bmi088, false, BMI_ACC_X_LSB, raw_n, 6);
    if (st != BMI_OK) {
        tc->result = R_FAIL;
        snprintf(tc->detail, sizeof(tc->detail), "ReadAcc(neg): %s", bmi_str(st));
        goto t5_cleanup;
    }

    {
        int32_t d[3];
        const bool ok = bmi_selftest_delta(raw_p, raw_n, d);
        const int32_t dx = d[0], dy = d[1], dz = d[2];

        if (!ok) {
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
    HAL_Delay(ACC_ST_SETTLE_MS);
    BMI088_ApplyConfig(bmi088, &bmi088_config->reg);
    HAL_Delay(2);
}


void BMI088_seq_test_t6_gyr_bist(TEST_case_t *tc) {
    BMI_STATE st = BMI088_WriteRegister(bmi088, true, BMI_GYR_SELF_TEST, GYR_BIST_TRIG);
    TEST_ASSERT(st == BMI_OK, "TriggerBIST: %s", bmi_str(st));

    uint32_t t_start = HAL_GetTick();
    uint8_t bist_reg = 0;
    do {
        HAL_Delay(5);
        st = BMI088_ReadRegister(bmi088, true, BMI_GYR_SELF_TEST, &bist_reg);
        TEST_ASSERT(st == BMI_OK, "PollBIST: %s", bmi_str(st));
    } while (!(bist_reg & GYR_BIST_RDY) && (HAL_GetTick() - t_start) < GYR_BIST_TIMEOUT);

    TEST_ASSERT(bist_reg & GYR_BIST_RDY, "Timeout %u ms, reg=0x%02X", GYR_BIST_TIMEOUT, bist_reg);
    TEST_ASSERT(!(bist_reg & GYR_BIST_FAIL) && (bist_reg & GYR_BIST_OK),
                "rate_ok=%d bist_fail=%d (reg=0x%02X)",
                (bist_reg & GYR_BIST_OK)  ? 1 : 0,
                (bist_reg & GYR_BIST_FAIL) ? 1 : 0,
                bist_reg);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "rate_ok=1 bist_fail=0 (reg=0x%02X)", bist_reg);
}


void BMI088_seq_test_t7_acc_temperature(TEST_case_t *tc) {
    float temp_c = 0.0f;
    BMI_STATE st = BMI088_ReadTemp(bmi088, &temp_c);
    TEST_ASSERT(st == BMI_OK, "ReadTemp: %s", bmi_str(st));
    TEST_ASSERT(temp_c >= TEMP_MIN_C && temp_c <= TEMP_MAX_C,
                "T=%.1f C hors plage [%.0f, %.0f]", temp_c, TEMP_MIN_C, TEMP_MAX_C);
    tc->result = R_PASS;
    /* evite -u _printf_float : conversion entiere */
    int32_t ti = (int32_t)temp_c;
    int32_t tf = (int32_t)((temp_c - (float)ti) * 10.0f);
    if (tf < 0) tf = -tf;
    snprintf(tc->detail, sizeof(tc->detail),
             "T=%ld.%ld C dans [%.0f, %.0f] OK",
             (long)ti, (long)tf, TEMP_MIN_C, TEMP_MAX_C);
}
