/**
 * @file BMI088.c
 * @brief Bosch BMI088 dual IMU driver — couche sequentielle (HAL bloquant)
 *        et helpers partages avec la couche RTOS (BMI088_rtos.c).
 *
 * @details
 *  - Niveau 0 : transaction SPI (HAL bloquant, timeout borne).
 *  - Niveau 1 : acces registres (simple / burst).
 *  - Niveau 2 : init, configuration, lecture des mesures.
 *
 * Rappel protocole (datasheet) :
 *  - Gyro : octet 0 = adresse (bit 7 = lecture), puis les donnees ; burst par
 *           auto-increment tant que CS reste bas.
 *  - Acc  : un octet dummy est renvoye AVANT la premiere donnee lue.
 * Une lecture est donc une seule transaction full-duplex :
 *   TX [addr][0...]   RX [x][(dummy)][data...]
 */

#include "BMI088.h"

#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/*                             Helpers partages                               */
/* -------------------------------------------------------------------------- */

BMI_STATE BMI088_CheckConfig(const bmi_reg_config_t *reg) {
    if (!reg) return BMI_INVALID_ARG;

    if ((unsigned)reg->acc_range > (unsigned)BMI_ACC_RANGE_24G) return BMI_INVALID_ARG;
    if (reg->acc_bwp != BMI_ACC_CONF_BWP_OSR4 && reg->acc_bwp != BMI_ACC_CONF_BWP_OSR2 &&
        reg->acc_bwp != BMI_ACC_CONF_BWP_NORMAL) return BMI_INVALID_ARG;
    if ((unsigned)reg->acc_odr < (unsigned)BMI_ACC_CONF_ODR_12_5_HZ ||
        (unsigned)reg->acc_odr > (unsigned)BMI_ACC_CONF_ODR_1600_HZ) return BMI_INVALID_ARG;
    if (reg->acc_pwr != BMI_ACC_PWR_CONF_ACTIVE && reg->acc_pwr != BMI_ACC_PWR_CONF_SUSPEND) return BMI_INVALID_ARG;
    if (reg->acc_ctrl != BMI_ACC_PWR_CTRL_ENABLE && reg->acc_ctrl != BMI_ACC_PWR_CTRL_DISABLE) return BMI_INVALID_ARG;

    if ((unsigned)reg->gyr_range > (unsigned)BMI_GYR_RANGE_125) return BMI_INVALID_ARG;
    if ((unsigned)reg->gyr_bw > (unsigned)BMI_GYR_BANDWIDTH_BW_32_HZ) return BMI_INVALID_ARG;
    if (reg->gyr_mode != BMI_GYR_LPM1_MODE_NORMAL && reg->gyr_mode != BMI_GYR_LPM1_MODE_SUSPEND &&
        reg->gyr_mode != BMI_GYR_LPM1_MODE_DEEPSUSPEND) return BMI_INVALID_ARG;

    return BMI_OK;
}

/* Datasheet : pleine echelle = 1.5 g * 2^(range + 1). */
float BMI088_AccSensitivity(bmi_acc_range_t range) {
    float factor = (1.0f / 32768.0f) * powf(2.0f, (float)range + 1.0f) * 1.5f;
#if (BMI_ACCEL_UNIT_MS2)
    return factor * 9.81f;
#else
    return factor;
#endif
}

/* Datasheet : pleine echelle = 2000 deg/s / 2^range. */
float BMI088_GyrSensitivity(bmi_gyr_range_t range) {
    float factor = 2000.0f / (32768.0f * powf(2.0f, (float)range));
#if (BMI_GYRO_UNIT_DPS)
    return factor;
#else
    return factor * (3.14159265f / 180.0f);
#endif
}

void BMI088_DecodeXYZ(const uint8_t raw[6], float conv, float3_t *out) {
    out->x = (float)(int16_t)((raw[1] << 8) | raw[0]) * conv;
    out->y = (float)(int16_t)((raw[3] << 8) | raw[2]) * conv;
    out->z = (float)(int16_t)((raw[5] << 8) | raw[4]) * conv;
}

/* Datasheet : valeur 11 bits signee = MSB * 8 + LSB[7:5], au-dela de 1023 on
   retranche 2048 ; T = valeur * 0.125 + 23 degC. */
float BMI088_DecodeTemp(const uint8_t raw[2]) {
    int32_t t = ((int32_t)raw[0] << 3) | ((raw[1] & BMI_TEMP_LSB_MASK) >> 5);
    if (t > 1023) t -= 2048;
    return (float)t * 0.125f + 23.0f;
}

void BMI088_DelayUs(uint32_t us) {
    /* DWT_CTRL survit au reset systeme mais pas DEMCR (une sonde SWD peut remettre
       TRCENA a 0) : CYCCNTENA seul laisserait CYCCNT fige et cette boucle sans fin. */
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u || (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    }
    const uint32_t start  = DWT->CYCCNT;
    const uint32_t cycles = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < cycles) {
    }
}

/* -------------------------------------------------------------------------- */
/*                         Niveau 0 : transaction SPI                         */
/* -------------------------------------------------------------------------- */

static inline void BMI088_SPI_Begin(bmi088_t *imu, bool is_gyr) {
    const bmi_bus_config_t *bus = &imu->config.bus;
    HAL_GPIO_WritePin(is_gyr ? bus->cs_gyr_bank : bus->cs_acc_bank,
                      is_gyr ? bus->cs_gyr_pin  : bus->cs_acc_pin, GPIO_PIN_RESET);
}

static inline void BMI088_SPI_End(bmi088_t *imu, bool is_gyr) {
    const bmi_bus_config_t *bus = &imu->config.bus;
    HAL_GPIO_WritePin(is_gyr ? bus->cs_gyr_bank : bus->cs_acc_bank,
                      is_gyr ? bus->cs_gyr_pin  : bus->cs_acc_pin, GPIO_PIN_SET);
}

/* Une transaction complete, CS compris, puis le temps mort tIDLE. */
static BMI_STATE BMI088_SPI_Xfer(bmi088_t *imu, bool is_gyr, const uint8_t *tx, uint8_t *rx, uint16_t len) {
    HAL_StatusTypeDef hs;

    BMI088_SPI_Begin(imu, is_gyr);
    if (rx != NULL) {
        hs = HAL_SPI_TransmitReceive(imu->config.bus.hspi, tx, rx, len, BMI_SPI_TIMEOUT_MS);
    } else {
        hs = HAL_SPI_Transmit(imu->config.bus.hspi, tx, len, BMI_SPI_TIMEOUT_MS);
    }
    BMI088_SPI_End(imu, is_gyr);

    BMI088_DelayUs(BMI_IDLE_NORMAL_US);
    return (hs == HAL_OK) ? BMI_OK : BMI_SPI_ERR;
}

/* -------------------------------------------------------------------------- */
/*                       Niveau 1 : acces registres                           */
/* -------------------------------------------------------------------------- */

BMI_STATE BMI088_ReadMultiple(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t *data, uint16_t len) {
    if (!imu || !imu->config.bus.hspi || !data || len == 0u || len > BMI_BURST_MAX) return BMI_INVALID_ARG;

    /* Gyro : [addr][data...] ; Acc : [addr][dummy][data...] */
    const uint16_t skip = is_gyr ? 1u : 2u;
    uint8_t tx[BMI_BURST_MAX + 2u] = { (uint8_t)(reg | BMI_READ_MASK) };
    uint8_t rx[BMI_BURST_MAX + 2u];

    BMI_STATE st = BMI088_SPI_Xfer(imu, is_gyr, tx, rx, (uint16_t)(skip + len));
    if (st != BMI_OK) return st;

    memcpy(data, &rx[skip], len);
    return BMI_OK;
}

BMI_STATE BMI088_ReadRegister(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t *value) {
    return BMI088_ReadMultiple(imu, is_gyr, reg, value, 1u);
}

BMI_STATE BMI088_WriteRegister(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t value) {
    if (!imu || !imu->config.bus.hspi) return BMI_INVALID_ARG;

    const uint8_t frame[2] = { (uint8_t)(reg & ~BMI_READ_MASK), value };
    return BMI088_SPI_Xfer(imu, is_gyr, frame, NULL, sizeof(frame));
}

BMI_STATE BMI088_ReadID(bmi088_t *imu, uint8_t *acc_id, uint8_t *gyr_id) {
    if (!imu || !acc_id || !gyr_id) return BMI_INVALID_ARG;

    BMI_STATE st = BMI088_ReadRegister(imu, true, BMI_GYR_CHIP_ID, gyr_id);
    if (st != BMI_OK) return st;
    return BMI088_ReadRegister(imu, false, BMI_ACC_CHIP_ID, acc_id);
}

/* -------------------------------------------------------------------------- */
/*          Sequences communes (reset, config, mise en route) via bmi_io_t    */
/* -------------------------------------------------------------------------- */

BMI_STATE BMI088_SoftReset_io(bmi088_t *imu, const bmi_io_t *io, bool is_gyr) {
    BMI_STATE st = is_gyr
        ? io->write(imu, true,  BMI_GYR_SOFTRESET, BMI_GYR_SOFTRESET_CMD)
        : io->write(imu, false, BMI_ACC_SOFTRESET, BMI_ACC_SOFTRESET_CMD);
    if (st != BMI_OK) return st;

    if (is_gyr) {
        io->delay_ms(BMI_GYR_SOFTRESET_DELAY_MS);
        return BMI_OK;
    }

    /* L'ACC redemarre en mode I2C : un front montant de CS (lecture fictive)
       le repasse en SPI. C'est ce qui manquait quand le reset "ne marchait pas". */
    io->delay_ms(BMI_ACC_SOFTRESET_DELAY_MS);
    uint8_t dummy;
    return io->read(imu, false, BMI_ACC_CHIP_ID, &dummy, 1u);
}

/* Ecrit puis attend : utilise pour les changements de mode d'alimentation. */
static BMI_STATE BMI088_WriteWait_io(bmi088_t *imu, const bmi_io_t *io, bool is_gyr, uint8_t reg, uint8_t value, uint32_t wait_ms) {
    BMI_STATE st = io->write(imu, is_gyr, reg, value);
    if (st == BMI_OK) io->delay_ms(wait_ms);
    return st;
}

/* Relit un registre et le compare sous masque. */
static BMI_STATE BMI088_Verify_io(bmi088_t *imu, const bmi_io_t *io, bool is_gyr, uint8_t reg, uint8_t mask, uint8_t expected) {
    uint8_t v = 0;
    BMI_STATE st = io->read(imu, is_gyr, reg, &v, 1u);
    if (st != BMI_OK) return st;
    return ((v & mask) == (expected & mask)) ? BMI_OK : BMI_CFG_ERR;
}

BMI_STATE BMI088_ApplyConfig_io(bmi088_t *imu, const bmi_io_t *io, const bmi_reg_config_t *reg) {
    BMI_STATE st = BMI088_CheckConfig(reg);
    if (st != BMI_OK) return st;

    uint8_t v = 0;

    /* --- ACC : actif et alimente pendant l'ecriture de la config. En suspend,
           deux ecritures doivent etre espacees de 450 us : on l'evite. --- */
    st = io->read(imu, false, BMI_ACC_PWR_CONF, &v, 1u);
    if (st != BMI_OK) return st;
    if ((v & BMI_ACC_PWR_CONF_MASK) != BMI_ACC_PWR_CONF_ACTIVE) {
        st = BMI088_WriteWait_io(imu, io, false, BMI_ACC_PWR_CONF, BMI_ACC_PWR_CONF_ACTIVE, BMI_ACC_PWR_DELAY_MS);
        if (st != BMI_OK) return st;
    }
    st = io->read(imu, false, BMI_ACC_PWR_CTRL, &v, 1u);
    if (st != BMI_OK) return st;
    if ((v & BMI_ACC_PWR_CTRL_MASK) != BMI_ACC_PWR_CTRL_ENABLE) {
        st = BMI088_WriteWait_io(imu, io, false, BMI_ACC_PWR_CTRL, BMI_ACC_PWR_CTRL_ENABLE, BMI_ACC_PWR_DELAY_MS);
        if (st != BMI_OK) return st;
    }

    const uint8_t acc_conf = (uint8_t)((reg->acc_bwp & BMI_ACC_CONF_BWP_MASK) | (reg->acc_odr & BMI_ACC_CONF_ODR_MASK));
    st = io->write(imu, false, BMI_ACC_CONF, acc_conf);
    if (st != BMI_OK) return st;
    st = io->write(imu, false, BMI_ACC_RANGE, (uint8_t)(reg->acc_range & BMI_ACC_RANGE_MASK));
    if (st != BMI_OK) return st;

    st = BMI088_Verify_io(imu, io, false, BMI_ACC_CONF, 0xFFu, acc_conf);
    if (st != BMI_OK) return st;
    st = BMI088_Verify_io(imu, io, false, BMI_ACC_RANGE, BMI_ACC_RANGE_MASK, reg->acc_range);
    if (st != BMI_OK) return st;

    /* Etat d'alimentation final, s'il n'est pas "actif + alimente". */
    if (reg->acc_ctrl != BMI_ACC_PWR_CTRL_ENABLE) {
        st = BMI088_WriteWait_io(imu, io, false, BMI_ACC_PWR_CTRL, reg->acc_ctrl, BMI_ACC_PWR_DELAY_MS);
        if (st != BMI_OK) return st;
    }
    if (reg->acc_pwr != BMI_ACC_PWR_CONF_ACTIVE) {
        st = BMI088_WriteWait_io(imu, io, false, BMI_ACC_PWR_CONF, reg->acc_pwr, BMI_ACC_PWR_DELAY_MS);
        if (st != BMI_OK) return st;
    }
    st = BMI088_Verify_io(imu, io, false, BMI_ACC_PWR_CONF, BMI_ACC_PWR_CONF_MASK, reg->acc_pwr);
    if (st != BMI_OK) return st;
    st = BMI088_Verify_io(imu, io, false, BMI_ACC_PWR_CTRL, BMI_ACC_PWR_CTRL_MASK, reg->acc_ctrl);
    if (st != BMI_OK) return st;

    /* --- GYR : en (deep) suspend seuls LPM1 et le reset sont accessibles. --- */
    st = io->read(imu, true, BMI_GYR_LPM1, &v, 1u);
    if (st != BMI_OK) return st;
    if ((v & BMI_GYR_LPM1_MODE_MASK) != BMI_GYR_LPM1_MODE_NORMAL) {
        st = BMI088_WriteWait_io(imu, io, true, BMI_GYR_LPM1, BMI_GYR_LPM1_MODE_NORMAL, BMI_GYR_WAKEUP_DELAY_MS);
        if (st != BMI_OK) return st;
    }

    st = io->write(imu, true, BMI_GYR_RANGE, (uint8_t)(reg->gyr_range & BMI_GYR_RANGE_MASK));
    if (st != BMI_OK) return st;
    /* Bit 7 de GYR_BANDWIDTH : lecture seule, toujours a 1. */
    st = io->write(imu, true, BMI_GYR_BANDWIDTH, (uint8_t)(0x80u | (reg->gyr_bw & BMI_GYR_BANDWIDTH_BW_MASK)));
    if (st != BMI_OK) return st;

    st = BMI088_Verify_io(imu, io, true, BMI_GYR_RANGE, BMI_GYR_RANGE_MASK, reg->gyr_range);
    if (st != BMI_OK) return st;
    st = BMI088_Verify_io(imu, io, true, BMI_GYR_BANDWIDTH, BMI_GYR_BANDWIDTH_BW_MASK, reg->gyr_bw);
    if (st != BMI_OK) return st;

    if (reg->gyr_mode != BMI_GYR_LPM1_MODE_NORMAL) {
        st = BMI088_WriteWait_io(imu, io, true, BMI_GYR_LPM1, reg->gyr_mode, BMI_ACC_PWR_DELAY_MS);
        if (st != BMI_OK) return st;
    }
    st = BMI088_Verify_io(imu, io, true, BMI_GYR_LPM1, BMI_GYR_LPM1_MODE_MASK, reg->gyr_mode);
    if (st != BMI_OK) return st;

    imu->config.reg = *reg;
    imu->acc_conv   = BMI088_AccSensitivity(reg->acc_range);
    imu->gyr_conv   = BMI088_GyrSensitivity(reg->gyr_range);
    return BMI_OK;
}

BMI_STATE BMI088_Bringup_io(bmi088_t *imu, const bmi_io_t *io, const bmi_reg_config_t *reg) {
    const bmi_bus_config_t *bus = &imu->config.bus;
    HAL_GPIO_WritePin(bus->cs_acc_bank, bus->cs_acc_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bus->cs_gyr_bank, bus->cs_gyr_pin, GPIO_PIN_SET);

    /* ACC : lecture fictive (passage en SPI apres la mise sous tension), reset,
       puis identification. */
    uint8_t id = 0;
    BMI_STATE st = io->read(imu, false, BMI_ACC_CHIP_ID, &id, 1u);
    if (st != BMI_OK) return st;
    st = BMI088_SoftReset_io(imu, io, false);
    if (st != BMI_OK) return st;
    st = io->read(imu, false, BMI_ACC_CHIP_ID, &id, 1u);
    if (st != BMI_OK) return st;
    if (id != BMI_ACC_CHIP_ID_VALUE) return BMI_ID_ERR;

    /* GYR : reset puis identification. */
    st = BMI088_SoftReset_io(imu, io, true);
    if (st != BMI_OK) return st;
    st = io->read(imu, true, BMI_GYR_CHIP_ID, &id, 1u);
    if (st != BMI_OK) return st;
    if (id != BMI_GYR_CHIP_ID_VALUE) return BMI_ID_ERR;

    return BMI088_ApplyConfig_io(imu, io, reg);
}

/* -------------------------------------------------------------------------- */
/*                   Niveau 2 : logique capteur (sequentiel)                  */
/* -------------------------------------------------------------------------- */

/* HAL_Delay(n) attend au moins n ms. */
static void seq_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

static const bmi_io_t seq_io = {
    .read     = BMI088_ReadMultiple,
    .write    = BMI088_WriteRegister,
    .delay_ms = seq_delay_ms,
};

BMI_STATE BMI088_SoftReset(bmi088_t *imu, bool is_gyr) {
    if (!imu) return BMI_INVALID_ARG;
    return BMI088_SoftReset_io(imu, &seq_io, is_gyr);
}

BMI_STATE BMI088_ApplyConfig(bmi088_t *imu, const bmi_reg_config_t *reg) {
    if (!imu) return BMI_INVALID_ARG;
    return BMI088_ApplyConfig_io(imu, &seq_io, reg);
}

BMI_STATE BMI088_Init(bmi088_t *imu, const bmi_config_t *config) {
    if (!imu || !config || !config->bus.hspi || !config->bus.cs_acc_bank || !config->bus.cs_gyr_bank) {
        return BMI_INVALID_ARG;
    }
    BMI_STATE st = BMI088_CheckConfig(&config->reg);
    if (st != BMI_OK) return st;

    imu->config.bus = config->bus;
    return BMI088_Bringup_io(imu, &seq_io, &config->reg);
}

BMI_STATE BMI088_ReadAcc(bmi088_t *imu, float3_t *accel) {
    if (!imu || !accel) return BMI_INVALID_ARG;

    uint8_t raw[6];
    BMI_STATE st = BMI088_ReadMultiple(imu, false, BMI_ACC_X_LSB, raw, sizeof(raw));
    if (st != BMI_OK) return st;

    BMI088_DecodeXYZ(raw, imu->acc_conv, accel);
    return BMI_OK;
}

BMI_STATE BMI088_ReadGyr(bmi088_t *imu, float3_t *gyr) {
    if (!imu || !gyr) return BMI_INVALID_ARG;

    uint8_t raw[6];
    BMI_STATE st = BMI088_ReadMultiple(imu, true, BMI_GYR_RATE_X_LSB, raw, sizeof(raw));
    if (st != BMI_OK) return st;

    BMI088_DecodeXYZ(raw, imu->gyr_conv, gyr);
    return BMI_OK;
}

BMI_STATE BMI088_ReadTemp(bmi088_t *imu, float *temp_c) {
    if (!imu || !temp_c) return BMI_INVALID_ARG;

    uint8_t raw[2];
    BMI_STATE st = BMI088_ReadMultiple(imu, false, BMI_TEMP_MSB, raw, sizeof(raw));
    if (st != BMI_OK) return st;

    *temp_c = BMI088_DecodeTemp(raw);
    return BMI_OK;
}
