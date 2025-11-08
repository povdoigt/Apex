/* bmi088.c — Niveaux 0 & 1
 * APEX avionics — Bosch BMI088 (ACC + GYR)
 */

#include "drivers/BMI088.h"

#include "stm32f4xx_hal.h"
#include "utils/tools.h"
#include "utils/types.h"

/* -------------------------------------------------------------------------- */
/*                           Niveau 0 : SPI Primitives                         */
/* -------------------------------------------------------------------------- */

void BMI088_SPI_Begin(bmi088_t *imu, bool is_gyr) {
    HAL_GPIO_WritePin(is_gyr ? imu->cs_gyr_bank : imu->cs_acc_bank,
                      is_gyr ? imu->cs_gyr_pin  : imu->cs_acc_pin,
                      GPIO_PIN_RESET);
}

void BMI088_SPI_End(bmi088_t *imu, bool is_gyr) {
    HAL_GPIO_WritePin(is_gyr ? imu->cs_gyr_bank : imu->cs_acc_bank,
                      is_gyr ? imu->cs_gyr_pin  : imu->cs_acc_pin,
                      GPIO_PIN_SET);
}

BMI_STATE BMI088_SPI_Tx(bmi088_t *imu, uint8_t *tx_buf, uint16_t tx_len) {
    if (!imu || !imu->spi || (!tx_buf && tx_len)) { return BMI_INVALID_ARG; }
    return (HAL_SPI_Transmit(imu->spi, tx_buf, tx_len, HAL_MAX_DELAY) == HAL_OK) ? BMI_OK : BMI_SPI_ERR;
}

BMI_STATE BMI088_SPI_Rx(bmi088_t *imu, uint8_t *rx_buf, uint16_t rx_len) {
    if (!imu || !imu->spi || (!rx_buf && rx_len)) { return BMI_INVALID_ARG; }
    return (HAL_SPI_Receive(imu->spi, rx_buf, rx_len, HAL_MAX_DELAY) == HAL_OK) ? BMI_OK : BMI_SPI_ERR;
}

/* -------------------------------------------------------------------------- */
/*                       Niveau 1 : Primitives capteur                         */
/* -------------------------------------------------------------------------- */
/* Rappel protocole (datasheet):
 *  - Gyro: 16-bit frames. Byte0 = (R/W bit0 + addr[6:0]), Byte1 = DATA.
 *          Burst : auto-increment si CS maintenu bas. (pas de dummy)
 *  - Acc : lecture avec 1 dummy byte AVANT la vraie donnée.
 *          => single read = lire 2 octets après l'adresse et garder le 2e.
 *          => burst N = lire (N+1) octets après l'adresse (jeter le 1er).
 */

BMI_STATE BMI088_ReadRegister(bmi088_t *imu, bool is_gyr, uint8_t reg_addr, uint8_t *value) {
    if (!imu || !value) return BMI_INVALID_ARG;

    BMI_STATE st;

    /* Gyroscope: 1 (addr+R)           + 1 (data) */
    /* Accelero:  1 (addr+R) + 1 dummy + 1 (data) */
    uint8_t addr = reg_addr | BMI_READ_MASK;

    BMI088_SPI_Begin(imu, is_gyr);
    st = BMI088_SPI_Tx(imu, &addr, 1);
    if (st != BMI_OK) { BMI088_SPI_End(imu, is_gyr); return st; }
    if (!is_gyr) {
        /* Pour l'accéléromètre, il faut lire un octet dummy avant la donnée */
        st = BMI088_SPI_Rx(imu, value, 1);
        if (st != BMI_OK) { BMI088_SPI_End(imu, is_gyr); return st; }
    }
    st = BMI088_SPI_Rx(imu, value, 1);
    BMI088_SPI_End(imu, is_gyr);
    if (st != BMI_OK) return st;

    /* Respecter tIDLE_wacc ≥ 2 µs avant une autre lecture/écriture (datasheet) */
    TIM_Delay_Micro(2);
    return st;
}

BMI_STATE BMI088_WriteRegister(bmi088_t *imu, bool is_gyr, uint8_t reg_addr, uint8_t value) {
    if (!imu) return BMI_INVALID_ARG;

    BMI_STATE st = BMI_OK;
    uint8_t frame[2] = { reg_addr & ~BMI_READ_MASK, value };

    BMI088_SPI_Begin(imu, is_gyr);
    st = BMI088_SPI_Tx(imu, frame, 2);
    BMI088_SPI_End(imu, is_gyr);
    if (st != BMI_OK) return st;

    /* Respecter tIDLE_wacc ≥ 2 µs avant une autre lecture/écriture (datasheet) */
    TIM_Delay_Micro(2);
    return st;
}

BMI_STATE BMI088_ReadMultiple(bmi088_t *imu, bool is_gyr, uint8_t start_reg, uint8_t *data, uint16_t len) {
    if (!imu || !data || !len) return BMI_INVALID_ARG;
    BMI_STATE st = BMI_OK;

    uint8_t addr = start_reg | BMI_READ_MASK;

    BMI088_SPI_Begin(imu, is_gyr);
    st = BMI088_SPI_Tx(imu, &addr, 1);
    if (st != BMI_OK) { BMI088_SPI_End(imu, is_gyr); return st; }
    if (!is_gyr) {
        /* Pour l'accéléromètre, lire 1 octet dummy avant les vraies données */
        st = BMI088_SPI_Rx(imu, data, 1);
        if (st != BMI_OK) { BMI088_SPI_End(imu, is_gyr); return st; }
    }
    st = BMI088_SPI_Rx(imu, data, len);
    BMI088_SPI_End(imu, is_gyr);
    if (st != BMI_OK) return st;

    /* Respecter tIDLE_wacc ≥ 2 µs avant une autre lecture/écriture (datasheet) */
    TIM_Delay_Micro(2);
    return st; 
}

BMI_STATE BMI088_ReadID(bmi088_t *imu, uint8_t *acc_id, uint8_t *gyr_id) {
    if (!imu || !acc_id || !gyr_id) {
        return BMI_INVALID_ARG;
    }

    /* Gyroscope CHIP_ID @ BMI_GYR_CHIP_ID */
    BMI_STATE st = BMI088_ReadRegister(imu, true, BMI_GYR_CHIP_ID, gyr_id);
    if (st != BMI_OK) { return st; }

    /* Accelerometer CHIP_ID @ BMI_ACC_CHIP_ID */
    st = BMI088_ReadRegister(imu, false, BMI_ACC_CHIP_ID, acc_id);
    return st;
}

BMI_STATE BMI088_SoftReset(bmi088_t *imu, bool is_gyr) {
    if (!imu) {
        return BMI_INVALID_ARG;
    }

    uint8_t reg;
    uint8_t cmd;

    if (is_gyr) {
        reg = BMI_GYR_SOFTRESET;      /* 0x14U */
        cmd = BMI_GYR_SOFTRESET_CMD;  /* 0b10110110 */
    } else {
        reg = BMI_ACC_SOFTRESET;      /* 0x7EU */
        cmd = BMI_ACC_SOFTRESET_CMD;  /* 0b10110110 */
    }

    return BMI088_WriteRegister(imu, is_gyr, reg, cmd);
}

/* -------------------------------------------------------------------------- */
/*                        Niveau 2 : Logique périphérique                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Calcule le facteur de conversion pour l'accéléromètre en m/s².
 * @param range Valeur du registre BMI_ACC_RANGE.
 * @retval Facteur de conversion (m/s² par LSB).
 * @note  D’après la formule Bosch, pages 21–22 de la datasheet Rev.1.9.
 */
static inline float BMI088_ComputeAccSensitivity(bmi_acc_range_t range) {
    return (9.81f / 32768.0f) * powf(2.0f, (float)range + 1.0f) * 1.5f;
}

/**
 * @brief Calcule le facteur de conversion pour le gyroscope en °/s.
 * @param range Valeur du registre BMI_GYR_RANGE.
 * @retval Facteur de conversion (°/s par LSB).
 * @note  D’après la formule Bosch, page 28 de la datasheet Rev.1.9.
 */
static inline float BMI088_ComputeGyrSensitivity(bmi_gyr_range_t range) {
    return 2000.0f / (32768.0f * powf(2.0f, (float)range));
}

/**
 * @brief Initialise la partie accéléromètre du BMI088.
 * @param imu Pointeur sur la structure de périphérique BMI088.
 * @param cfg Configuration à appliquer.
 * @retval BMI_STATE BMI_OK si succès, code d’erreur sinon.
 */
static inline BMI_STATE BMI088_InitAccelerometer(bmi088_t *imu) {
    BMI_STATE st;

    /* Soft-reset accéléromètre */
    st = BMI088_SoftReset(imu, false);
    if (st != BMI_OK) { return st; }
    HAL_Delay(1);

    /* Vérification ID accéléromètre */
    uint8_t acc_id = 0;
    st = BMI088_ReadRegister(imu, false, BMI_ACC_CHIP_ID_VALUE, &acc_id);
    if (st != BMI_OK) { return st; }
    if (acc_id != BMI_ACC_CHIP_ID) { return BMI_UNKNOWN_ERR; }

    /* Activation alimentation accéléromètre */
    st = BMI088_WriteRegister(imu, false, BMI_ACC_PWR_CTRL, BMI_ACC_PWR_CTRL_ENABLE);
    if (st != BMI_OK) { return st; }
    TIM_Delay_Micro(450);

    return BMI_OK;
}

/**
 * @brief Initialise la partie gyroscope du BMI088.
 * @param imu Pointeur sur la structure de périphérique BMI088.
 * @param cfg Configuration à appliquer.
 * @retval BMI_STATE BMI_OK si succès, code d’erreur sinon.
 */
static inline BMI_STATE BMI088_InitGyroscope(bmi088_t *imu) {
    BMI_STATE st;

    /* Soft-reset gyroscope */
    st = BMI088_SoftReset(imu, true);
    if (st != BMI_OK) { return st; }
    HAL_Delay(30);

    /* Vérification ID gyroscope */
    uint8_t gyr_id = 0;
    st = BMI088_ReadRegister(imu, true, BMI_GYR_CHIP_ID_VALUE, &gyr_id);
    if (st != BMI_OK) { return st; }
    if (gyr_id != 0x0F) { return BMI_UNKNOWN_ERR; }

    return BMI_OK;
}

/* -------------------------------------------------------------------------- */
/*                               BMI088_Init                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise le capteur BMI088 (accéléromètre + gyroscope).
 * @param imu         Pointeur sur la structure BMI088.
 * @param hspi        Handle SPI utilisé.
 * @param cs_acc_bank Port GPIO du chip select accéléromètre.
 * @param cs_acc_pin  Pin GPIO du chip select accéléromètre.
 * @param cs_gyr_bank Port GPIO du chip select gyroscope.
 * @param cs_gyr_pin  Pin GPIO du chip select gyroscope.
 * @param cfg         Pointeur vers la configuration à appliquer.
 * @retval BMI_STATE  BMI_OK si succès, code d’erreur sinon.
 */
BMI_STATE BMI088_Init(bmi088_t *imu,
                      SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_acc_bank, uint16_t cs_acc_pin,
                      GPIO_TypeDef *cs_gyr_bank, uint16_t cs_gyr_pin,
                      const bmi_config_t *cfg)
{
    if (!imu || !hspi || !cfg) { return BMI_INVALID_ARG; }

    imu->spi          = hspi;
    imu->cs_acc_bank  = cs_acc_bank;
    imu->cs_acc_pin   = cs_acc_pin;
    imu->cs_gyr_bank  = cs_gyr_bank;
    imu->cs_gyr_pin   = cs_gyr_pin;

    BMI_STATE st;

    /* Initialisation séparée des sous-blocs */
    st = BMI088_InitAccelerometer(imu);
    if (st != BMI_OK) { return st; }

    st = BMI088_InitGyroscope(imu);
    if (st != BMI_OK) { return st; }

    st = BMI088_ApplyConfig(imu, cfg);
    if (st != BMI_OK) { return st; }

    /* Calcul des facteurs de conversion */
    imu->acc_conv = BMI088_ComputeAccSensitivity(cfg->acc_range);
    imu->gyr_conv = BMI088_ComputeGyrSensitivity(cfg->gyr_range);

    return BMI_OK;
}


/* -------------------------------------------------------------------------- */
/*                            BMI088_ApplyConfig                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Applique une configuration complète au BMI088.
 * @param imu Pointeur sur la structure de périphérique BMI088.
 * @param cfg Pointeur sur la configuration à appliquer.
 * @retval BMI_STATE  BMI_OK si succès, code d’erreur sinon.
 */
BMI_STATE BMI088_ApplyConfig(bmi088_t *imu, const bmi_config_t *cfg) {
    if (!imu || !cfg) { return BMI_INVALID_ARG; }

    BMI_STATE st;

    /* --- Accelerometer configuration --- */
    st = BMI088_WriteRegister(imu, false, BMI_ACC_CONF,
        ((cfg->acc_bwp & BMI_ACC_CONF_BWP_MASK) |
         (cfg->acc_odr & BMI_ACC_CONF_ODR_MASK)));
    if (st != BMI_OK) { return st; }

    st = BMI088_WriteRegister(imu, false, BMI_ACC_RANGE,
        (cfg->acc_range & BMI_ACC_RANGE_MASK));
    if (st != BMI_OK) { return st; }

    st = BMI088_WriteRegister(imu, false, BMI_ACC_PWR_CONF,
        (cfg->acc_pwr & BMI_ACC_PWR_CONF_MASK));
    if (st != BMI_OK) { return st; }

    st = BMI088_WriteRegister(imu, false, BMI_ACC_PWR_CTRL,
        (cfg->acc_ctrl & BMI_ACC_PWR_CTRL_MASK));
    if (st != BMI_OK) { return st; }

    /* --- Gyroscope configuration --- */
    st = BMI088_WriteRegister(imu, true, BMI_GYR_BANDWIDTH,
        (cfg->gyr_bw & BMI_GYR_BANDWIDTH_BW_MASK));
    if (st != BMI_OK) { return st; }

    st = BMI088_WriteRegister(imu, true, BMI_GYR_RANGE,
        (cfg->gyr_range & BMI_GYR_RANGE_MASK));
    if (st != BMI_OK) { return st; }

    st = BMI088_WriteRegister(imu, true, BMI_GYR_LPM1,
        (cfg->gyr_mode & BMI_GYR_LPM1_MODE_MASK));
    if (st != BMI_OK) { return st; }

    imu->config = *cfg;
    return BMI_OK;
}

/* -------------------------------------------------------------------------- */
/*                            BMI088_ReadAcc                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Lit les mesures d’accélération du BMI088 et les convertit en m/s².
 * @param imu Pointeur sur la structure de périphérique BMI088.
 * @param accel Pointeur vers la structure de sortie des données d’accélération.
 * @retval BMI_STATE  BMI_OK si succès, code d’erreur sinon.
 */
BMI_STATE BMI088_ReadAcc(bmi088_t *imu, float3_t *accel) {
    if (!imu || !accel) { return BMI_INVALID_ARG; }

    uint8_t raw[6];
    BMI_STATE st = BMI088_ReadMultiple(imu, false, BMI_ACC_X_LSB, raw, 6);
    if (st != BMI_OK) { return st; }

    int16_t rx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t rz = (int16_t)((raw[5] << 8) | raw[4]);

    accel->x = rx * imu->acc_conv;
    accel->y = ry * imu->acc_conv;
    accel->z = rz * imu->acc_conv;

    return BMI_OK;
}

/* -------------------------------------------------------------------------- */
/*                            BMI088_ReadGyr                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Lit les mesures de rotation du BMI088 et les convertit en °/s.
 * @param imu Pointeur sur la structure de périphérique BMI088.
 * @param gyr Pointeur vers la structure de sortie des données de rotation.
 * @retval BMI_STATE  BMI_OK si succès, code d’erreur sinon.
 */
BMI_STATE BMI088_ReadGyr(bmi088_t *imu, float3_t *gyr) {
    if (!imu || !gyr) { return BMI_INVALID_ARG; }

    uint8_t raw[6];
    BMI_STATE st = BMI088_ReadMultiple(imu, true, BMI_GYR_RATE_X_LSB, raw, 6);
    if (st != BMI_OK) { return st; }

    int16_t rx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t rz = (int16_t)((raw[5] << 8) | raw[4]);

    gyr->x = rx * imu->gyr_conv;
    gyr->y = ry * imu->gyr_conv;
    gyr->z = rz * imu->gyr_conv;

    return BMI_OK;
}

/* -------------------------------------------------------------------------- */
/*                            BMI088_ReadTemp                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Lit la température interne du capteur et la convertit en °C.
 * @param imu Pointeur sur la structure de périphérique BMI088.
 * @param temp_c Pointeur vers la température convertie.
 * @retval BMI_STATE  BMI_OK si succès, code d’erreur sinon.
 */
BMI_STATE BMI088_ReadTemp(bmi088_t *imu, float *temp_c) {
    if (!imu || !temp_c) { return BMI_INVALID_ARG; }

    uint8_t raw[2];
    BMI_STATE st = BMI088_ReadMultiple(imu, false, BMI_TEMP_MSB, raw, 2);
    if (st != BMI_OK) { return st; }

    // 11-bit signed value (two's complement)
    // LSB: bits [7:5] of raw[1], MSB: raw[0]
    int16_t t_lsb_raw = (int16_t)((raw[1] & BMI_TEMP_LSB_MASK) >> 5);
    int16_t t_msb_raw = (int16_t)(raw[0] << 3);
    int16_t t_raw = t_msb_raw | t_lsb_raw;
    if (t_msb_raw > 0x3E) {
        if (t_msb_raw < 0xC1) {
            t_raw -= 2048;  // two's complement adjustment for negative values
        } else {
            return BMI_UNKNOWN_ERR;  // invalid temperature reading
        }
    }

    // Convert to °C using formula from datasheet (section 5.3.7 page 28)
    *temp_c = (float)t_raw * 0.125f + 23.0f;
    return BMI_OK;
}

