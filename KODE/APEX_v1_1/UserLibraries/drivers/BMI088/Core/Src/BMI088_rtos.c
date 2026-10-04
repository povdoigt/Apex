/**
 *******************************************************************************
 * @file    BMI088_rtos.c
 * @brief   RTOS layer of the BMI088 driver. @see BMI088_rtos.h
 *
 * @details
 * Multi-step sequences (reset, configuration, bring-up) are shared with the
 * sequential layer through bmi_io_t: here they run on the _NoLock primitives,
 * with the sensor semaphore held around the whole sequence.
 *******************************************************************************
 */

#include "BMI088_rtos.h"

#if (APEX_CFG_SCHED_RTOS == 1)

#include "spi.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/*                          Level 0 : SPI transaction                         */
/* -------------------------------------------------------------------------- */

/* One complete transaction on the shared bus (bus semaphore + CS), then the
   2 us idle time the sensor needs before its next access. */
static BMI_STATE BMI088_SPI_Xfer_RTOS(bmi088_t *imu, bool is_gyr, const uint8_t *tx, uint8_t *rx, uint16_t len) {
    const bmi_bus_config_t *bus = &imu->config.bus;
    GPIO_TypeDef *cs_bank = is_gyr ? bus->cs_gyr_bank : bus->cs_acc_bank;
    uint16_t      cs_pin  = is_gyr ? bus->cs_gyr_pin  : bus->cs_acc_pin;

    if (SPI_Begin_DMA_RTOS(bus->hspi, cs_bank, cs_pin) != HAL_OK) {
        return BMI_SEM_ERR;
    }
    HAL_StatusTypeDef hs = (rx != NULL)
        ? SPI_TransmitReceive_DMA_RTOS(bus->hspi, tx, rx, len)
        : SPI_Transmit_DMA_RTOS(bus->hspi, tx, len);
    HAL_StatusTypeDef he = SPI_End_DMA_RTOS(bus->hspi, cs_bank, cs_pin);

    BMI088_DelayUs(BMI_IDLE_NORMAL_US);

    if (hs != HAL_OK) return BMI_SPI_ERR;
    return (he == HAL_OK) ? BMI_OK : BMI_SEM_ERR;
}

/* Sensor semaphore. `lock` false means the caller already owns it.
   osErrorResource is what a 0 timeout returns when the semaphore is taken. */
static inline BMI_STATE BMI088_Lock(bmi088_t *imu, bool lock, uint32_t timeout_ms) {
    if (!lock) return BMI_OK;
    if (imu->sem_id == NULL) return BMI_SEM_ERR;
    switch (osSemaphoreAcquire(imu->sem_id, timeout_ms)) {
    case osOK:            return BMI_OK;
    case osErrorTimeout:
    case osErrorResource: return BMI_LOCK_TIMEOUT;
    default:              return BMI_SEM_ERR;
    }
}

static inline BMI_STATE BMI088_Unlock(bmi088_t *imu, bool lock, BMI_STATE st) {
    if (lock && osSemaphoreRelease(imu->sem_id) != osOK) {
        return BMI_SEM_ERR;
    }
    return st;
}

/* -------------------------------------------------------------------------- */
/*                        Level 1 : register access                           */
/* -------------------------------------------------------------------------- */

BMI_STATE BMI088_ReadMultiple_RTOS_base(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t *data, uint16_t len,
                                        uint32_t timeout_ms, bool lock_sem) {
    if (!imu || !imu->config.bus.hspi || !data || len == 0u || len > BMI_BURST_MAX) return BMI_INVALID_ARG;

    /* One full-duplex DMA transfer. Gyro: [addr][data...]; Acc: [addr][dummy][data...] */
    const uint16_t skip = is_gyr ? 1u : 2u;
    uint8_t tx[BMI_BURST_MAX + 2u] = { (uint8_t)(reg | BMI_READ_MASK) };
    uint8_t rx[BMI_BURST_MAX + 2u];

    BMI_STATE st = BMI088_Lock(imu, lock_sem, timeout_ms);
    if (st != BMI_OK) return st;
    st = BMI088_SPI_Xfer_RTOS(imu, is_gyr, tx, rx, (uint16_t)(skip + len));
    st = BMI088_Unlock(imu, lock_sem, st);
    if (st != BMI_OK) return st;

    memcpy(data, &rx[skip], len);
    return BMI_OK;
}

BMI_STATE BMI088_WriteRegister_RTOS_base(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t value,
                                         uint32_t timeout_ms, bool lock_sem) {
    if (!imu || !imu->config.bus.hspi) return BMI_INVALID_ARG;

    const uint8_t frame[2] = { (uint8_t)(reg & ~BMI_READ_MASK), value };

    BMI_STATE st = BMI088_Lock(imu, lock_sem, timeout_ms);
    if (st != BMI_OK) return st;
    st = BMI088_SPI_Xfer_RTOS(imu, is_gyr, frame, NULL, sizeof(frame));
    return BMI088_Unlock(imu, lock_sem, st);
}

BMI_STATE BMI088_ReadID_RTOS_base(bmi088_t *imu, uint8_t *acc_id, uint8_t *gyr_id, uint32_t timeout_ms, bool lock_sem) {
    if (!imu || !acc_id || !gyr_id) return BMI_INVALID_ARG;

    BMI_STATE st = BMI088_Lock(imu, lock_sem, timeout_ms);
    if (st != BMI_OK) return st;
    st = BMI088_ReadRegister_RTOS_NoLock(imu, true, BMI_GYR_CHIP_ID, gyr_id);
    if (st == BMI_OK) st = BMI088_ReadRegister_RTOS_NoLock(imu, false, BMI_ACC_CHIP_ID, acc_id);
    return BMI088_Unlock(imu, lock_sem, st);
}

/* bmi_io_t adapters: the shared sequences run on the _NoLock primitives. */
static BMI_STATE rtos_io_read(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t *data, uint16_t len) {
    return BMI088_ReadMultiple_RTOS_NoLock(imu, is_gyr, reg, data, len);
}
static BMI_STATE rtos_io_write(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t value) {
    return BMI088_WriteRegister_RTOS_NoLock(imu, is_gyr, reg, value);
}
/* osDelay(n) may return after n - 1 ms (tick phase): one more tick. */
static void rtos_io_delay_ms(uint32_t ms) {
    osDelay(ms + 1u);
}

static const bmi_io_t rtos_io = {
    .read     = rtos_io_read,
    .write    = rtos_io_write,
    .delay_ms = rtos_io_delay_ms,
};

BMI_STATE BMI088_SoftReset_RTOS_base(bmi088_t *imu, bool is_gyr, uint32_t timeout_ms, bool lock_sem) {
    if (!imu || !imu->config.bus.hspi) return BMI_INVALID_ARG;

    BMI_STATE st = BMI088_Lock(imu, lock_sem, timeout_ms);
    if (st != BMI_OK) return st;
    st = BMI088_SoftReset_io(imu, &rtos_io, is_gyr);
    return BMI088_Unlock(imu, lock_sem, st);
}

/* -------------------------------------------------------------------------- */
/*                          Level 2 : sensor logic                            */
/* -------------------------------------------------------------------------- */

BMI_STATE BMI088_Init_RTOS(bmi088_t *imu, const bmi_config_t *config, uint32_t timeout_ms) {
    if (!imu || !config || !config->bus.hspi || !config->bus.cs_acc_bank || !config->bus.cs_gyr_bank) {
        return BMI_INVALID_ARG;
    }
    BMI_STATE st = BMI088_CheckConfig(&config->reg);
    if (st != BMI_OK) return st;

    /* Created once: re-creating it over a semaphore another job holds or waits on
       would corrupt the kernel's lists. A re-init takes it like any other operation. */
    if (imu->sem_id == NULL) {
        imu->sem_id = osSemaphoreNew(1u, 1u, &(const osSemaphoreAttr_t){
            .name    = "bmi088",
            .cb_mem  = &imu->sem,
            .cb_size = sizeof(imu->sem),
        });
        if (imu->sem_id == NULL) {
            return BMI_SEM_ERR;
        }
    }

    st = BMI088_Lock(imu, true, timeout_ms);
    if (st != BMI_OK) return st;

    /* Under the lock: the bus config is read by every transaction. */
    imu->config.bus = config->bus;
    st = BMI088_Bringup_io(imu, &rtos_io, &config->reg);

    return BMI088_Unlock(imu, true, st);
}

BMI_STATE BMI088_ApplyConfig_RTOS(bmi088_t *imu, const bmi_reg_config_t *reg, uint32_t timeout_ms) {
    if (!imu) return BMI_INVALID_ARG;
    BMI_STATE st = BMI088_CheckConfig(reg);
    if (st != BMI_OK) return st;

    st = BMI088_Lock(imu, true, timeout_ms);
    if (st != BMI_OK) return st;
    st = BMI088_ApplyConfig_io(imu, &rtos_io, reg);
    return BMI088_Unlock(imu, true, st);
}

/* A burst read and its conversion factor, taken under the same lock so a
   concurrent ApplyConfig cannot pair new data with an old range. */
static BMI_STATE BMI088_ReadXYZ_RTOS(bmi088_t *imu, bool is_gyr, float3_t *out, uint32_t timeout_ms) {
    if (!imu || !out) return BMI_INVALID_ARG;

    uint8_t raw[6];
    BMI_STATE st = BMI088_Lock(imu, true, timeout_ms);
    if (st != BMI_OK) return st;
    st = BMI088_ReadMultiple_RTOS_NoLock(imu, is_gyr, is_gyr ? BMI_GYR_RATE_X_LSB : BMI_ACC_X_LSB, raw, sizeof(raw));
    const float conv = is_gyr ? imu->gyr_conv : imu->acc_conv;
    st = BMI088_Unlock(imu, true, st);
    if (st != BMI_OK) return st;

    BMI088_DecodeXYZ(raw, conv, out);
    return BMI_OK;
}

BMI_STATE BMI088_ReadAcc_RTOS(bmi088_t *imu, float3_t *accel, uint32_t timeout_ms) {
    return BMI088_ReadXYZ_RTOS(imu, false, accel, timeout_ms);
}

BMI_STATE BMI088_ReadGyr_RTOS(bmi088_t *imu, float3_t *gyro, uint32_t timeout_ms) {
    return BMI088_ReadXYZ_RTOS(imu, true, gyro, timeout_ms);
}

BMI_STATE BMI088_ReadTemp_RTOS(bmi088_t *imu, float *temp_c, uint32_t timeout_ms) {
    if (!imu || !temp_c) return BMI_INVALID_ARG;

    uint8_t raw[2];
    BMI_STATE st = BMI088_ReadMultiple_RTOS(imu, false, BMI_TEMP_MSB, raw, sizeof(raw), timeout_ms);
    if (st != BMI_OK) return st;

    *temp_c = BMI088_DecodeTemp(raw);
    return BMI_OK;
}

/* -------------------------------------------------------------------------- */
/*                                   Tasks                                    */
/* -------------------------------------------------------------------------- */

TASK_DEFINE(BMI088_Init) {
    return BMI088_Init_RTOS(args->imu, &args->config, args->timeout_ms);
}

TASK_DEFINE(BMI088_ApplyConfig) {
    return BMI088_ApplyConfig_RTOS(args->imu, &args->reg, args->timeout_ms);
}

static void acq_error(bmi_acq_stats_t *stats, BMI_STATE st) {
    if (stats != NULL) {
        stats->errors++;
        stats->last_error = st;
    }
}

TASK_DEFINE(BMI088_Acquire) {
    if (args->imu == NULL || args->period_ms == 0u ||
        (args->acc_topic == NULL && args->gyr_topic == NULL && args->temp_topic == NULL)) {
        return BMI_INVALID_ARG;
    }
    const uint32_t temp_div = (args->temp_divider != 0u) ? args->temp_divider : 1u;
    uint32_t n    = 0u;
    uint32_t next = osKernelGetTickCount();

    while (args->stop == NULL || !*args->stop) {
        const uint32_t ts = osKernelGetTickCount();
        BMI_STATE st;

        if (args->acc_topic != NULL) {
            float3_ts_t s = { .ts = ts };
            st = BMI088_ReadAcc_RTOS(args->imu, &s.data, args->timeout_ms);
            if (st == BMI_OK) (void)data_topic_publish(args->acc_topic, &s);
            else              acq_error(args->stats, st);
        }
        if (args->gyr_topic != NULL) {
            float3_ts_t s = { .ts = ts };
            st = BMI088_ReadGyr_RTOS(args->imu, &s.data, args->timeout_ms);
            if (st == BMI_OK) (void)data_topic_publish(args->gyr_topic, &s);
            else              acq_error(args->stats, st);
        }
        if (args->temp_topic != NULL && (n % temp_div) == 0u) {
            float_ts_t s = { .ts = ts };
            st = BMI088_ReadTemp_RTOS(args->imu, &s.data, args->timeout_ms);
            if (st == BMI_OK) (void)data_topic_publish(args->temp_topic, &s);
            else              acq_error(args->stats, st);
        }

        n++;
        if (args->stats != NULL) args->stats->cycles++;

        /* Next slot on the grid; if it is already past, count the missed
           periods and restart the grid from now (no catch-up burst). */
        next += args->period_ms;
        const uint32_t now  = osKernelGetTickCount();
        const int32_t  lead = (int32_t)(next - now);
        if (lead < 0) {
            if (args->stats != NULL) args->stats->overruns += (now - next) / args->period_ms + 1u;
            next = now;
        } else if (lead > 0) {
            (void)osDelayUntil(next);
        }
    }
    return BMI_OK;
}

#endif /* APEX_CFG_SCHED_RTOS == 1 */
