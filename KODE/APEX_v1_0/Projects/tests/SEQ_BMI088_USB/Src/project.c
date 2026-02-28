#include "project.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "utils/vt100.h"

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

/* ========================================================================
 * Banc de test
 * ======================================================================== */
#define N_TESTS  8

typedef enum { R_PASS = 0, R_FAIL, R_SKIP } result_t;

typedef struct {
    const char *name;
    result_t    result;
    char        detail[96];
} test_case_t;

static test_case_t tc[N_TESTS];
static char        log_buf[512];

/* ========================================================================
 * Utilitaires
 * ======================================================================== */
static void usb_print(const char *s) {
    CDC_Transmit_FS((uint8_t *)s, strlen(s));
    HAL_Delay(5);
}

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

/* ========================================================================
 * T0 — Identifiants chip ACC + GYR (BMI088_ReadID)
 *   Lit les deux chip IDs en une seule transaction SPI (burst interne).
 *   Verifie ACC_CHIP_ID=0x1E et GYR_CHIP_ID=0x0F.
 * ======================================================================== */
static void t0_chip_ids(void) {
    tc[0].name = "T0 Chip IDs";
    uint8_t acc_id = 0, gyr_id = 0;
    BMI_STATE st = BMI088_ReadID(&bmi088, &acc_id, &gyr_id);
    if (st != BMI_OK) {
        tc[0].result = R_FAIL;
        snprintf(tc[0].detail, sizeof(tc[0].detail), "ReadID: %s", bmi_str(st));
        return;
    }
    if (acc_id != BMI_ACC_CHIP_ID_EXP || gyr_id != BMI_GYR_CHIP_ID_EXP) {
        tc[0].result = R_FAIL;
        snprintf(tc[0].detail, sizeof(tc[0].detail),
                 "ACC=0x%02X(exp:0x%02X) GYR=0x%02X(exp:0x%02X)",
                 acc_id, BMI_ACC_CHIP_ID_EXP, gyr_id, BMI_GYR_CHIP_ID_EXP);
        return;
    }
    tc[0].result = R_PASS;
    snprintf(tc[0].detail, sizeof(tc[0].detail),
             "ACC_ID=0x%02X GYR_ID=0x%02X", acc_id, gyr_id);
}

/* ========================================================================
 * T1 — Soft reset accelerometre (BMI088_SoftReset)
 *   Envoie la commande de reset ACC (0xB6 @ 0x7E), attend ≥50 ms,
 *   reengage PWR_CTRL+PWR_CONF, puis relit le chip ID.
 * ======================================================================== */
static void t1_acc_soft_reset(void) {
    tc[1].name = "T1 ACC SoftReset";
    BMI_STATE st = BMI088_SoftReset(&bmi088, false /* ACC */);
    if (st != BMI_OK) {
        tc[1].result = R_FAIL;
        snprintf(tc[1].detail, sizeof(tc[1].detail), "SoftReset: %s", bmi_str(st));
        return;
    }
    HAL_Delay(50);

    /* Apres reset : ACC est en SUSPEND ; il faut le remettre en ACTIVE    */
    BMI088_WriteRegister(&bmi088, false, BMI_ACC_PWR_CTRL, (uint8_t)BMI_ACC_PWR_CTRL_ENABLE);
    HAL_Delay(5);
    BMI088_WriteRegister(&bmi088, false, BMI_ACC_PWR_CONF, (uint8_t)BMI_ACC_PWR_CONF_ACTIVE);
    HAL_Delay(5);

    uint8_t id = 0;
    st = BMI088_ReadRegister(&bmi088, false, BMI_ACC_CHIP_ID, &id);
    if (st != BMI_OK) {
        tc[1].result = R_FAIL;
        snprintf(tc[1].detail, sizeof(tc[1].detail), "ReadID post-reset: %s", bmi_str(st));
        return;
    }
    if (id != BMI_ACC_CHIP_ID_EXP) {
        tc[1].result = R_FAIL;
        snprintf(tc[1].detail, sizeof(tc[1].detail),
                 "ID=0x%02X apres reset (exp:0x%02X)", id, BMI_ACC_CHIP_ID_EXP);
        return;
    }
    tc[1].result = R_PASS;
    snprintf(tc[1].detail, sizeof(tc[1].detail), "ID=0x%02X intact apres reset", id);
}

/* ========================================================================
 * T2 — Soft reset gyroscope (BMI088_SoftReset)
 *   Envoie 0xB6 @ 0x14, attend ≥30 ms (spec), relit GYR_CHIP_ID.
 * ======================================================================== */
static void t2_gyr_soft_reset(void) {
    tc[2].name = "T2 GYR SoftReset";
    BMI_STATE st = BMI088_SoftReset(&bmi088, true /* GYR */);
    if (st != BMI_OK) {
        tc[2].result = R_FAIL;
        snprintf(tc[2].detail, sizeof(tc[2].detail), "SoftReset: %s", bmi_str(st));
        return;
    }
    HAL_Delay(30);

    uint8_t id = 0;
    st = BMI088_ReadRegister(&bmi088, true, BMI_GYR_CHIP_ID, &id);
    if (st != BMI_OK) {
        tc[2].result = R_FAIL;
        snprintf(tc[2].detail, sizeof(tc[2].detail), "ReadID post-reset: %s", bmi_str(st));
        return;
    }
    if (id != BMI_GYR_CHIP_ID_EXP) {
        tc[2].result = R_FAIL;
        snprintf(tc[2].detail, sizeof(tc[2].detail),
                 "ID=0x%02X apres reset (exp:0x%02X)", id, BMI_GYR_CHIP_ID_EXP);
        return;
    }
    tc[2].result = R_PASS;
    snprintf(tc[2].detail, sizeof(tc[2].detail), "ID=0x%02X intact apres reset", id);
}

/* ========================================================================
 * T3 — Ecriture / relecture config accelerometre (BMI088_ApplyConfig)
 *   Applique une config differente de la config nominale, relit les
 *   registres ACC_CONF (0x40) et ACC_RANGE (0x41) et compare.
 *   Restaure ensuite la config nominale (bmi088_config).
 * ======================================================================== */
static void t3_acc_config_rw(void) {
    tc[3].name = "T3 ACC Config RW";

    /* Config a ecrire : ±12 g, ODR=200 Hz, BWP=OSR2 */
    const bmi_config_t test_cfg = {
        .acc_range = BMI_ACC_RANGE_12G,
        .acc_bwp   = BMI_ACC_CONF_BWP_OSR2,
        .acc_odr   = BMI_ACC_CONF_ODR_200_HZ,
        .acc_pwr   = BMI_ACC_PWR_CONF_ACTIVE,
        .acc_ctrl  = BMI_ACC_PWR_CTRL_ENABLE,
        /* GYR inchange : copie de la config nominale */
        .gyr_range = bmi088_config.gyr_range,
        .gyr_bw    = bmi088_config.gyr_bw,
        .gyr_mode  = bmi088_config.gyr_mode,
    };
    BMI_STATE st = BMI088_ApplyConfig(&bmi088, &test_cfg);
    if (st != BMI_OK) {
        tc[3].result = R_FAIL;
        snprintf(tc[3].detail, sizeof(tc[3].detail), "ApplyConfig: %s", bmi_str(st));
        goto t3_restore;
    }
    HAL_Delay(2);

    uint8_t conf_r = 0, range_r = 0;
    st = BMI088_ReadRegister(&bmi088, false, BMI_ACC_CONF,  &conf_r);
    if (st != BMI_OK) {
        tc[3].result = R_FAIL;
        snprintf(tc[3].detail, sizeof(tc[3].detail), "ReadConf: %s", bmi_str(st));
        goto t3_restore;
    }
    st = BMI088_ReadRegister(&bmi088, false, BMI_ACC_RANGE, &range_r);
    if (st != BMI_OK) {
        tc[3].result = R_FAIL;
        snprintf(tc[3].detail, sizeof(tc[3].detail), "ReadRange: %s", bmi_str(st));
        goto t3_restore;
    }

    {
        const uint8_t conf_exp  = (uint8_t)BMI_ACC_CONF_BWP_OSR2 | (uint8_t)BMI_ACC_CONF_ODR_200_HZ;
        const uint8_t range_exp = (uint8_t)BMI_ACC_RANGE_12G;
        if (conf_r != conf_exp || range_r != range_exp) {
            tc[3].result = R_FAIL;
            snprintf(tc[3].detail, sizeof(tc[3].detail),
                     "CONF: got=0x%02X exp=0x%02X  RANGE: got=0x%02X exp=0x%02X",
                     conf_r, conf_exp, range_r, range_exp);
        } else {
            tc[3].result = R_PASS;
            snprintf(tc[3].detail, sizeof(tc[3].detail),
                     "ACC_CONF=0x%02X ACC_RANGE=0x%02X OK", conf_r, range_r);
        }
    }

t3_restore:
    BMI088_ApplyConfig(&bmi088, &bmi088_config);
    HAL_Delay(2);
}

/* ========================================================================
 * T4 — Ecriture / relecture config gyroscope (BMI088_ApplyConfig)
 *   Applique ±500 dps / BW_47_HZ, relit GYR_RANGE (0x0F) + GYR_BW (0x10).
 *   Restaure ensuite la config nominale.
 * ======================================================================== */
static void t4_gyr_config_rw(void) {
    tc[4].name = "T4 GYR Config RW";

    const bmi_config_t test_cfg = {
        /* ACC inchange */
        .acc_range = bmi088_config.acc_range,
        .acc_bwp   = bmi088_config.acc_bwp,
        .acc_odr   = bmi088_config.acc_odr,
        .acc_pwr   = bmi088_config.acc_pwr,
        .acc_ctrl  = bmi088_config.acc_ctrl,
        /* GYR : ±500 dps, BW=47 Hz */
        .gyr_range = BMI_GYR_RANGE_500,
        .gyr_bw    = BMI_GYR_BANDWIDTH_BW_47_HZ,
        .gyr_mode  = BMI_GYR_LPM1_MODE_NORMAL,
    };
    BMI_STATE st = BMI088_ApplyConfig(&bmi088, &test_cfg);
    if (st != BMI_OK) {
        tc[4].result = R_FAIL;
        snprintf(tc[4].detail, sizeof(tc[4].detail), "ApplyConfig: %s", bmi_str(st));
        goto t4_restore;
    }
    HAL_Delay(2);

    uint8_t range_r = 0, bw_r = 0;
    st = BMI088_ReadRegister(&bmi088, true, BMI_GYR_RANGE,     &range_r);
    if (st != BMI_OK) {
        tc[4].result = R_FAIL;
        snprintf(tc[4].detail, sizeof(tc[4].detail), "ReadRange: %s", bmi_str(st));
        goto t4_restore;
    }
    st = BMI088_ReadRegister(&bmi088, true, BMI_GYR_BANDWIDTH, &bw_r);
    if (st != BMI_OK) {
        tc[4].result = R_FAIL;
        snprintf(tc[4].detail, sizeof(tc[4].detail), "ReadBW: %s", bmi_str(st));
        goto t4_restore;
    }

    {
        const uint8_t range_exp = (uint8_t)BMI_GYR_RANGE_500;
        const uint8_t bw_exp    = (uint8_t)BMI_GYR_BANDWIDTH_BW_47_HZ;
        if (range_r != range_exp || bw_r != bw_exp) {
            tc[4].result = R_FAIL;
            snprintf(tc[4].detail, sizeof(tc[4].detail),
                     "RANGE: got=0x%02X exp=0x%02X  BW: got=0x%02X exp=0x%02X",
                     range_r, range_exp, bw_r, bw_exp);
        } else {
            tc[4].result = R_PASS;
            snprintf(tc[4].detail, sizeof(tc[4].detail),
                     "GYR_RANGE=0x%02X GYR_BW=0x%02X OK", range_r, bw_r);
        }
    }

t4_restore:
    BMI088_ApplyConfig(&bmi088, &bmi088_config);
    HAL_Delay(2);
}

/* ========================================================================
 * T5 — Self-test accelerometre (procedure Bosch AN §4.4.1)
 *   Config obligatoire : ±24 g, ODR=1600 Hz, BWP=Normal.
 *   1. Active mode positif (0x0D) → attend 50 ms → lit 6 octets bruts.
 *   2. Active mode negatif (0x09) → attend 50 ms → lit 6 octets bruts.
 *   3. delta = pos - neg ; doit depasser ACC_ST_MIN_LSB (1366) sur X, Y, Z.
 *   4. Desactive le self-test et restaure la config nominale dans tous les cas.
 * ======================================================================== */
static void t5_acc_self_test(void) {
    tc[5].name = "T5 ACC SelfTest";

    /* Config self-test : ±24g obligatoire, 1600 Hz, Normal           */
    const bmi_config_t st_cfg = {
        .acc_range = BMI_ACC_RANGE_24G,
        .acc_bwp   = BMI_ACC_CONF_BWP_NORMAL,
        .acc_odr   = BMI_ACC_CONF_ODR_1600_HZ,
        .acc_pwr   = BMI_ACC_PWR_CONF_ACTIVE,
        .acc_ctrl  = BMI_ACC_PWR_CTRL_ENABLE,
        .gyr_range = bmi088_config.gyr_range,
        .gyr_bw    = bmi088_config.gyr_bw,
        .gyr_mode  = bmi088_config.gyr_mode,
    };
    BMI_STATE st = BMI088_ApplyConfig(&bmi088, &st_cfg);
    if (st != BMI_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "ApplyConfig: %s", bmi_str(st));
        goto t5_cleanup;
    }
    HAL_Delay(2);

    /* Mesure positive */
    st = BMI088_WriteRegister(&bmi088, false, BMI_ACC_SELF_TEST, (uint8_t)BMI_ACC_SELF_TEST_POS);
    if (st != BMI_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "WriteSTpos: %s", bmi_str(st));
        goto t5_cleanup;
    }
    HAL_Delay(50);

    uint8_t raw_p[6] = {0};
    st = BMI088_ReadMultiple(&bmi088, false, BMI_ACC_X_LSB, raw_p, 6);
    if (st != BMI_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "ReadAcc(pos): %s", bmi_str(st));
        goto t5_cleanup;
    }

    /* Mesure negative */
    st = BMI088_WriteRegister(&bmi088, false, BMI_ACC_SELF_TEST, (uint8_t)BMI_ACC_SELF_TEST_NEG);
    if (st != BMI_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "WriteSTneg: %s", bmi_str(st));
        goto t5_cleanup;
    }
    HAL_Delay(50);

    uint8_t raw_n[6] = {0};
    st = BMI088_ReadMultiple(&bmi088, false, BMI_ACC_X_LSB, raw_n, 6);
    if (st != BMI_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "ReadAcc(neg): %s", bmi_str(st));
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
            tc[5].result = R_FAIL;
            snprintf(tc[5].detail, sizeof(tc[5].detail),
                     "dX=%ld dY=%ld dZ=%ld (min=%d LSB)",
                     (long)dx, (long)dy, (long)dz, ACC_ST_MIN_LSB);
        } else {
            tc[5].result = R_PASS;
            snprintf(tc[5].detail, sizeof(tc[5].detail),
                     "dX=%ld dY=%ld dZ=%ld >= %d LSB OK",
                     (long)dx, (long)dy, (long)dz, ACC_ST_MIN_LSB);
        }
    }

t5_cleanup:
    BMI088_WriteRegister(&bmi088, false, BMI_ACC_SELF_TEST, (uint8_t)BMI_ACC_SELF_TEST_OFF);
    HAL_Delay(50);
    BMI088_ApplyConfig(&bmi088, &bmi088_config);
    HAL_Delay(2);
}

/* ========================================================================
 * T6 — Built-in self-test gyroscope (registre GYR_SELF_TEST 0x3C)
 *   Bits datasheet : [0]=trig_bist  [1]=bist_rdy  [2]=bist_fail  [4]=rate_ok
 *   1. Ecrit trig_bist=1.
 *   2. Attend que bist_rdy (bit 1) passe a 1 (timeout GYR_BIST_TIMEOUT ms).
 *   3. Verifie rate_ok (bit 4)=1 et bist_fail (bit 2)=0.
 * ======================================================================== */
static void t6_gyr_bist(void) {
    tc[6].name = "T6 GYR BIST";
    BMI_STATE st = BMI088_WriteRegister(&bmi088, true, BMI_GYR_SELF_TEST, GYR_BIST_TRIG);
    if (st != BMI_OK) {
        tc[6].result = R_FAIL;
        snprintf(tc[6].detail, sizeof(tc[6].detail), "TriggerBIST: %s", bmi_str(st));
        return;
    }

    uint32_t t_start = HAL_GetTick();
    uint8_t bist_reg = 0;
    do {
        HAL_Delay(5);
        st = BMI088_ReadRegister(&bmi088, true, BMI_GYR_SELF_TEST, &bist_reg);
        if (st != BMI_OK) {
            tc[6].result = R_FAIL;
            snprintf(tc[6].detail, sizeof(tc[6].detail), "PollBIST: %s", bmi_str(st));
            return;
        }
    } while (!(bist_reg & GYR_BIST_RDY) && (HAL_GetTick() - t_start) < GYR_BIST_TIMEOUT);

    if (!(bist_reg & GYR_BIST_RDY)) {
        tc[6].result = R_FAIL;
        snprintf(tc[6].detail, sizeof(tc[6].detail),
                 "Timeout %u ms, reg=0x%02X", GYR_BIST_TIMEOUT, bist_reg);
        return;
    }
    if ((bist_reg & GYR_BIST_FAIL) || !(bist_reg & GYR_BIST_OK)) {
        tc[6].result = R_FAIL;
        snprintf(tc[6].detail, sizeof(tc[6].detail),
                 "rate_ok=%d bist_fail=%d (reg=0x%02X)",
                 (bist_reg & GYR_BIST_OK)   ? 1 : 0,
                 (bist_reg & GYR_BIST_FAIL)  ? 1 : 0,
                 bist_reg);
        return;
    }
    tc[6].result = R_PASS;
    snprintf(tc[6].detail, sizeof(tc[6].detail),
             "rate_ok=1 bist_fail=0 (reg=0x%02X)", bist_reg);
}

/* ========================================================================
 * T7 — Lecture de la temperature interne de l'accelerometre
 *   BMI088_ReadTemp lit les registres BMI_TEMP_MSB/LSB et applique
 *   la formule Bosch (t = raw * 0.125 + 23 °C).
 *   Critere : valeur dans [-40, +85] °C.
 * ======================================================================== */
static void t7_acc_temperature(void) {
    tc[7].name = "T7 ACC Temp";
    float temp_c = 0.0f;
    BMI_STATE st = BMI088_ReadTemp(&bmi088, &temp_c);
    if (st != BMI_OK) {
        tc[7].result = R_FAIL;
        snprintf(tc[7].detail, sizeof(tc[7].detail), "ReadTemp: %s", bmi_str(st));
        return;
    }
    if (temp_c < TEMP_MIN_C || temp_c > TEMP_MAX_C) {
        tc[7].result = R_FAIL;
        snprintf(tc[7].detail, sizeof(tc[7].detail),
                 "T=%.1f C hors plage [%.0f, %.0f]", temp_c, TEMP_MIN_C, TEMP_MAX_C);
        return;
    }
    tc[7].result = R_PASS;
    /* evite -u _printf_float : conversion entiere */
    int32_t ti = (int32_t)temp_c;
    int32_t tf = (int32_t)((temp_c - (float)ti) * 10.0f);
    if (tf < 0) tf = -tf;
    snprintf(tc[7].detail, sizeof(tc[7].detail),
             "T=%ld.%ld C dans [%.0f, %.0f] OK",
             (long)ti, (long)tf, TEMP_MIN_C, TEMP_MAX_C);
}

/* ========================================================================
 * setup() — execution unique apres init des peripheriques
 * ======================================================================== */
void setup(void) {
    t0_chip_ids();
    t1_acc_soft_reset();
    t2_gyr_soft_reset();
    t3_acc_config_rw();
    t4_gyr_config_rw();
    t5_acc_self_test();
    t6_gyr_bist();
    t7_acc_temperature();

    /* Attend que le serial monitor soit ouvert cote PC (DTR=1).
     * Sans ca, les premiers caracteres seraient perdus avant
     * que le terminal ne soit pret a les recevoir.            */
    while (!cdc_port_open) {
        HAL_Delay(10);
    }
    HAL_Delay(50); /* stabilisation du terminal */
}

/* ========================================================================
 * loop() — rapport periodique + mesures live
 *   - Tableau de tests : affiche une seule fois (premier appel).
 *   - Mesures live     : ecrase les deux lignes de donnees a chaque appel
 *                        via VT100_CURSOR_UP(2) sans toucher au reste.
 * ======================================================================== */
void loop(void) {
    static bool first_loop = true;

    if (first_loop) {
        uint32_t n_pass = 0, n_fail = 0;
        for (int i = 0; i < N_TESTS; i++) {
            if      (tc[i].result == R_PASS) n_pass++;
            else if (tc[i].result == R_FAIL) n_fail++;
        }

        usb_print(VT100_SCREEN_CLEAR);

        snprintf(log_buf, sizeof(log_buf),
                 VT100_FG_CYAN "===== BMI088 Sequential Driver - Test Suite =====" VT100_RESET "\r\n"
                 "8 cas de test (IDs, reset, config R/W, self-test, temperature)\r\n\r\n");
        usb_print(log_buf);

        for (int i = 0; i < N_TESTS; i++) {
            const char *col = (tc[i].result == R_PASS) ? VT100_FG_GREEN
                            : (tc[i].result == R_FAIL) ? VT100_FG_RED
                            :                            VT100_FG_YELLOW;
            const char *tag = (tc[i].result == R_PASS) ? "PASS"
                            : (tc[i].result == R_FAIL) ? "FAIL" : "SKIP";
            snprintf(log_buf, sizeof(log_buf),
                     "  %s[%s]" VT100_RESET " %-20s %s\r\n",
                     col, tag, tc[i].name, tc[i].detail);
            usb_print(log_buf);
        }

        const char *vcol = (n_fail == 0) ? VT100_BG_GREEN VT100_FG_BLACK
                                         : VT100_BG_RED   VT100_FG_BLACK;
        snprintf(log_buf, sizeof(log_buf),
                 "\r\n%s  %lu/%d PASS   %lu FAIL  " VT100_RESET "\r\n",
                 vcol, (unsigned long)n_pass, N_TESTS, (unsigned long)n_fail);
        usb_print(log_buf);

        /* En-tete fixe de la section live (imprime une seule fois) */
        usb_print("\r\n" VT100_FG_CYAN "--- Mesures live ---" VT100_RESET "\r\n");

        first_loop = false;
    } else {
        /* Remonte de 2 lignes pour ecraser uniquement les valeurs ACC/GYR */
        usb_print(VT100_CURSOR_UP(2));
    }

    /* --- Mesures live (mises a jour a chaque appel) --- */
    float3_t acc = {0}, gyr = {0};
    BMI088_ReadAcc(&bmi088, &acc);
    BMI088_ReadGyr(&bmi088, &gyr);

    snprintf(log_buf, sizeof(log_buf),
             "  ACC  X=%+10.3f  Y=%+10.3f  Z=%+10.3f  m/s2\r\n"
             "  GYR  X=%+10.3f  Y=%+10.3f  Z=%+10.3f  deg/s\r\n",
             acc.x, acc.y, acc.z,
             gyr.x, gyr.y, gyr.z);
    usb_print(log_buf);

    HAL_Delay(10);
}