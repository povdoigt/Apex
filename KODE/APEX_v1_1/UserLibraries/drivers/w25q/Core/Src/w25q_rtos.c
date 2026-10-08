/**
 *******************************************************************************
 * @file    w25q_rtos.c
 * @brief   RTOS layer of the W25Q driver. @see w25q_rtos.h
 *
 * @details
 * The one `goto` per function that survives releases the chip semaphore: a real
 * resource the function owns. The task bodies themselves are business logic and
 * a `return`; the framework handles result publication and completion.
 *******************************************************************************
 */

#include "w25q_rtos.h"

#if (APEX_CFG_SCHED_RTOS == 1)

/* -------------------------------------------------------------------------- */
/*                          Level 0 : SPI transaction                         */
/* -------------------------------------------------------------------------- */

static inline W25Q_STATE W25Q_SPI_Begin_RTOS(W25Q_t *chip) {
    return SPI_Begin_DMA_RTOS(chip->config.bus.hspi, chip->config.bus.cs_bank, chip->config.bus.cs_pin) == HAL_OK ? W25Q_OK : W25Q_SEM_ERR;
}
static inline W25Q_STATE W25Q_SPI_Tx_RTOS(W25Q_t *chip, const uint8_t *tx_buf, uint16_t tx_len) {
    return SPI_Transmit_DMA_RTOS(chip->config.bus.hspi, tx_buf, tx_len) == HAL_OK ? W25Q_OK : W25Q_SPI_ERR;
}
static inline W25Q_STATE W25Q_SPI_Rx_RTOS(W25Q_t *chip, uint8_t *rx_buf, uint16_t rx_len) {
    return SPI_Receive_DMA_RTOS(chip->config.bus.hspi, rx_buf, rx_len) == HAL_OK ? W25Q_OK : W25Q_SPI_ERR;
}
static inline W25Q_STATE W25Q_SPI_End_RTOS(W25Q_t *chip) {
    return SPI_End_DMA_RTOS(chip->config.bus.hspi, chip->config.bus.cs_bank, chip->config.bus.cs_pin) == HAL_OK ? W25Q_OK : W25Q_SEM_ERR;
}

/* Chip semaphore. `lock` false means the caller already owns it.
   osErrorResource is what a 0 timeout returns when the semaphore is taken. */
static inline W25Q_STATE W25Q_Lock(W25Q_t *chip, bool lock, uint32_t timeout_ms) {
    if (!lock) return W25Q_OK;
    switch (osSemaphoreAcquire(chip->sem_id, timeout_ms)) {
    case osOK:            return W25Q_OK;
    case osErrorTimeout:
    case osErrorResource: return W25Q_LOCK_TIMEOUT;
    default:              return W25Q_SEM_ERR;
    }
}
static inline W25Q_STATE W25Q_Unlock(W25Q_t *chip, bool lock, W25Q_STATE st) {
    if (lock && osSemaphoreRelease(chip->sem_id) != osOK) {
        return W25Q_SEM_ERR;
    }
    return st;
}

/* -------------------------------------------------------------------------- */
/*                         Level 1 : command primitives                       */
/* -------------------------------------------------------------------------- */

W25Q_STATE W25Q_WaitForReady_RTOS_base(W25Q_t *chip, uint32_t timeout_ms, bool lock_sem) {
    const uint32_t start = HAL_GetTick();
    for (;;) {
        /* The semaphore is retaken on each poll, within what is left of the timeout. */
        uint32_t elapsed = HAL_GetTick() - start;
        W25Q_STATE st = W25Q_ReadStatus_RTOS_base(chip, 1, (elapsed < timeout_ms) ? timeout_ms - elapsed : 0u, lock_sem);
        if (st != W25Q_OK) return st;
        if (!W25Q_STATUS_REG(chip, W25Q_SR1_BUSY_BIT)) return W25Q_OK;

        elapsed = HAL_GetTick() - start;
        if (elapsed >= timeout_ms) return W25Q_BUSY_TIMEOUT;

        /* Spin through short operations (page program), then poll once per tick:
           a poll is almost all fixed cost, spinning would starve every lower
           priority task for the whole erase. Neither the bus nor, with lock_sem,
           the chip semaphore is held while sleeping. */
        if (elapsed >= W25Q_RTOS_SPIN_MS) {
            osDelay(1u);
        }
    }
}

W25Q_STATE W25Q_SendCmd_RTOS_base(W25Q_t *chip, uint8_t cmd, uint32_t timeout_ms, bool lock_sem) {
    W25Q_STATE st;

    if (!W25Q_IsCmdValid(cmd)) {
        return W25Q_PARAM_ERR;
    }

    st = W25Q_Lock(chip, lock_sem, timeout_ms);
    if (st != W25Q_OK) return st;

    if (W25Q_IsCmdRequiresBusyCheck(cmd)) {
        st = W25Q_WaitForReady_RTOS_NoLock(chip, timeout_ms);
        if (st != W25Q_OK) goto release;
    }

    /* WRITE_ENABLE itself is excluded to avoid infinite recursion. */
    if (W25Q_IsCmdRequiresWEL(cmd) && cmd != W25Q_WRITE_ENABLE &&
        !W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
        st = W25Q_SendCmd_RTOS_NoLock(chip, W25Q_WRITE_ENABLE, timeout_ms);
        if (st != W25Q_OK) goto release;
    }

    st = W25Q_SPI_Begin_RTOS(chip);
    if (st != W25Q_OK) goto release;
    st = W25Q_SPI_Tx_RTOS(chip, &cmd, 1);
    W25Q_STATE end_st = W25Q_SPI_End_RTOS(chip);
    if (st == W25Q_OK) st = end_st;
    if (st != W25Q_OK) goto release;

    if (W25Q_IsCmdNeedWaitAfter(cmd)) {
        st = W25Q_WaitForReady_RTOS_NoLock(chip, timeout_ms);
        if (st != W25Q_OK) goto release;
    }

    /* Keep the cached ADS in step: it sets the address length of SendCmdAddr. */
    switch (cmd) {
    case W25Q_ENABLE_4B_MODE:  chip->status_reg |=  (1UL << W25Q_SR3_ADS_BIT); break;
    case W25Q_DISABLE_4B_MODE: chip->status_reg &= ~(1UL << W25Q_SR3_ADS_BIT); break;
    case W25Q_RESET:
        /* SRs reloaded from their non-volatile values, ADS from ADP (SR1 already re-read by WAIT_AFTER). */
        st = W25Q_ReadStatus_RTOS_NoLock(chip, 2);
        if (st == W25Q_OK) st = W25Q_ReadStatus_RTOS_NoLock(chip, 3);
        break;
    default: break;
    }

release:
    return W25Q_Unlock(chip, lock_sem, st);
}

W25Q_STATE W25Q_SendCmdAddr_RTOS_base(W25Q_t *chip, uint8_t cmd, uint32_t addr, uint32_t timeout_ms, bool lock_sem) {
    W25Q_STATE st;

    if (!W25Q_IsCmdValid(cmd)) {
        return W25Q_PARAM_ERR;
    }

    st = W25Q_Lock(chip, lock_sem, timeout_ms);
    if (st != W25Q_OK) return st;

    if (W25Q_IsCmdRequiresBusyCheck(cmd)) {
        st = W25Q_WaitForReady_RTOS_NoLock(chip, timeout_ms);
        if (st != W25Q_OK) goto release;
    }

    if (W25Q_IsCmdRequiresWEL(cmd) && !W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
        st = W25Q_SendCmd_RTOS_NoLock(chip, W25Q_WRITE_ENABLE, timeout_ms);
        if (st != W25Q_OK) goto release;
    }

    /* Address length: fixed for the 4-byte opcodes, otherwise set by the current mode (ADS).
       Read under the lock: a concurrent B7h/E9h/reset may change it. */
    const uint8_t addr_len = (W25Q_IsCmdAddr4B(cmd) || W25Q_STATUS_REG(chip, W25Q_SR3_ADS_BIT)) ? 4u : 3u;
    if (addr_len == 3u && addr > 0x00FFFFFFUL) {
        st = W25Q_PARAM_ERR;
        goto release;
    }
    uint8_t tx[5] = { cmd };
    for (uint8_t i = 0; i < addr_len; i++) {
        tx[1 + i] = (uint8_t)(addr >> (8u * (addr_len - 1u - i)));
    }

    st = W25Q_SPI_Begin_RTOS(chip);
    if (st != W25Q_OK) goto release;
    st = W25Q_SPI_Tx_RTOS(chip, tx, (uint16_t)(1u + addr_len));
    W25Q_STATE end_st = W25Q_SPI_End_RTOS(chip);
    if (st == W25Q_OK) st = end_st;
    if (st != W25Q_OK) goto release;

    if (W25Q_IsCmdNeedWaitAfter(cmd)) {
        st = W25Q_WaitForReady_RTOS_NoLock(chip, timeout_ms);
    }

release:
    return W25Q_Unlock(chip, lock_sem, st);
}

W25Q_STATE W25Q_ReadStatus_RTOS_base(W25Q_t *chip, uint8_t sr_index, uint32_t timeout_ms, bool lock_sem) {
    static const uint8_t sr_cmd[3] = { W25Q_READ_SR1, W25Q_READ_SR2, W25Q_READ_SR3 };
    W25Q_STATE st;
    uint8_t status = 0;

    if (sr_index < 1u || sr_index > 3u) {
        return W25Q_PARAM_ERR;
    }
    sr_index--;

    st = W25Q_Lock(chip, lock_sem, timeout_ms);
    if (st != W25Q_OK) return st;

    st = W25Q_SPI_Begin_RTOS(chip);
    if (st != W25Q_OK) goto release;
    st = W25Q_SPI_Tx_RTOS(chip, &sr_cmd[sr_index], 1);
    if (st == W25Q_OK) st = W25Q_SPI_Rx_RTOS(chip, &status, 1);
    W25Q_STATE end_st = W25Q_SPI_End_RTOS(chip);
    if (st == W25Q_OK) st = end_st;
    if (st != W25Q_OK) goto release;

    chip->status_reg &= ~((uint32_t)0xFF << (sr_index * 8));
    chip->status_reg |=  ((uint32_t)status << (sr_index * 8));

release:
    return W25Q_Unlock(chip, lock_sem, st);
}

W25Q_STATE W25Q_WriteStatus_RTOS_base(W25Q_t *chip, uint8_t sr_index, uint8_t value, W25Q_SR_WRITE mode, uint32_t timeout_ms, bool lock_sem) {
    static const uint8_t sr_cmd[3] = { W25Q_WRITE_SR1, W25Q_WRITE_SR2, W25Q_WRITE_SR3 };
    W25Q_STATE st;

    if (sr_index < 1u || sr_index > 3u) {
        return W25Q_PARAM_ERR;
    }
    sr_index--;

    const uint8_t tx[2] = { sr_cmd[sr_index], value };

    st = W25Q_Lock(chip, lock_sem, timeout_ms);
    if (st != W25Q_OK) return st;

    /* A status-register write needs WEL (06h, non-volatile) or 50h (volatile).
       SendCmd already waits for BUSY=0 before either. */
    st = W25Q_SendCmd_RTOS_NoLock(chip,
            (mode == W25Q_SR_WRITE_VOLATILE) ? W25Q_ENABLE_VOLATILE_SR : W25Q_WRITE_ENABLE, timeout_ms);
    if (st != W25Q_OK) goto release;

    st = W25Q_SPI_Begin_RTOS(chip);
    if (st != W25Q_OK) goto release;
    st = W25Q_SPI_Tx_RTOS(chip, tx, sizeof(tx));
    W25Q_STATE end_st = W25Q_SPI_End_RTOS(chip);
    if (st == W25Q_OK) st = end_st;
    if (st != W25Q_OK) goto release;

    /* Read-only bits (BUSY, WEL, SUS, ADS) are not written: keep their cached value. */
    uint32_t writable = ((uint32_t)0xFF << (sr_index * 8)) & ~W25Q_SR_READONLY_MASK;
    chip->status_reg = (chip->status_reg & ~writable) | (((uint32_t)value << (sr_index * 8)) & writable);

    st = W25Q_WaitForReady_RTOS_NoLock(chip, timeout_ms);

release:
    return W25Q_Unlock(chip, lock_sem, st);
}

W25Q_STATE W25Q_ReadID_RTOS_base(W25Q_t *chip, uint8_t *id, uint32_t timeout_ms, bool lock_sem) {
    const uint8_t cmd = W25Q_READ_JEDEC_ID;
    W25Q_STATE st;

    st = W25Q_Lock(chip, lock_sem, timeout_ms);
    if (st != W25Q_OK) return st;

    st = W25Q_SPI_Begin_RTOS(chip);
    if (st != W25Q_OK) goto release;
    st = W25Q_SPI_Tx_RTOS(chip, &cmd, 1);
    if (st == W25Q_OK) st = W25Q_SPI_Rx_RTOS(chip, id, 3);
    W25Q_STATE end_st = W25Q_SPI_End_RTOS(chip);
    if (st == W25Q_OK) st = end_st;

release:
    return W25Q_Unlock(chip, lock_sem, st);
}

/* -------------------------------------------------------------------------- */
/*                          Level 2 : device logic                            */
/* -------------------------------------------------------------------------- */

W25Q_STATE W25Q_PageProgram_RTOS(W25Q_t *chip, const uint8_t *data, uint32_t addr, uint16_t len, uint32_t timeout_ms) {
    W25Q_STATE st = W25Q_Lock(chip, true, timeout_ms);
    if (st != W25Q_OK) return st;

    st = W25Q_WaitForReady_RTOS_NoLock(chip, timeout_ms);
    if (st != W25Q_OK) goto release;
    if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
        st = W25Q_SendCmd_RTOS_NoLock(chip, W25Q_WRITE_ENABLE, timeout_ms);
        if (st != W25Q_OK) goto release;
    }

    len = len > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE : len;
    const uint8_t cmd[5] = {
        W25Q_PAGE_PROGRAM_4B,
        (uint8_t)(addr >> 24),
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >>  8),
        (uint8_t)(addr >>  0),
    };

    st = W25Q_SPI_Begin_RTOS(chip);
    if (st != W25Q_OK) goto release;
    st = W25Q_SPI_Tx_RTOS(chip, cmd, sizeof(cmd));
    if (st == W25Q_OK) st = W25Q_SPI_Tx_RTOS(chip, data, len);
    W25Q_STATE end_st = W25Q_SPI_End_RTOS(chip);
    if (st == W25Q_OK) st = end_st;
    if (st != W25Q_OK) goto release;

    st = W25Q_WaitForReady_RTOS_NoLock(chip, timeout_ms);

release:
    return W25Q_Unlock(chip, true, st);
}

/* -------------------------------------------------------------------------- */
/*                                   Tasks                                    */
/* -------------------------------------------------------------------------- */

TASK_DEFINE(W25Q_SendCmd) {
    return W25Q_SendCmd_RTOS(args->chip, args->cmd, args->timeout_ms);
}

TASK_DEFINE(W25Q_SendCmdAddr) {
    return W25Q_SendCmdAddr_RTOS(args->chip, args->cmd, args->addr, args->timeout_ms);
}

TASK_DEFINE(W25Q_Init) {
    W25Q_t              *chip = args->chip;
    const W25Q_config_t *cfg  = &args->config;
    uint32_t             mask, bits;

    if (chip == NULL || cfg->bus.hspi == NULL || cfg->bus.cs_bank == NULL) {
        return W25Q_PARAM_ERR;
    }
    W25Q_STATE st = W25Q_ConfigToStatus(&cfg->reg, &mask, &bits);
    if (st != W25Q_OK) return st;

    /* Created once: re-creating it over a semaphore another job holds or waits on
       would corrupt the kernel's lists. A re-init takes it like any other operation. */
    if (chip->sem_id == NULL) {
        chip->sem_id = osSemaphoreNew(1u, 1u, &(const osSemaphoreAttr_t){
            .name    = "w25q_bus",
            .cb_mem  = &chip->sem,
            .cb_size = sizeof(chip->sem),
        });
        if (chip->sem_id == NULL) {
            return W25Q_SEM_ERR;
        }
    }

    st = W25Q_Lock(chip, true, args->timeout_ms);
    if (st != W25Q_OK) return st;

    /* Under the lock: the bus config and the cache are read by every transaction. */
    chip->config     = *cfg;
    chip->status_reg = 0;
    HAL_GPIO_WritePin(cfg->bus.cs_bank, cfg->bus.cs_pin, GPIO_PIN_SET);

    /* Check ID. */
    uint8_t id[3] = { 0 };
    W25Q_ReadID_RTOS_NoLock(chip, id); // Dummy read to ensure the SPI bus is ready
    st = W25Q_ReadID_RTOS_NoLock(chip, id);
    if (st != W25Q_OK) goto release;
    if (id[0] != W25Q_MANUFACTURER_ID) { st = W25Q_CHIP_ERR; goto release; }
    if (W25Q_V_FULL_DEVICE_ID != (uint32_t)((id[1] << 8) | id[2])) { st = W25Q_PARAM_ERR; goto release; }

    /* Read the current configuration. */
    for (uint8_t sr = 1u; sr <= 3u; sr++) {
        st = W25Q_ReadStatus_RTOS_NoLock(chip, sr);
        if (st != W25Q_OK) goto release;
    }

    /* Write only the registers whose configured bits differ: no needless NV write,
       bits not covered by the config (SRP, SRL, LB...) are written back unchanged. */
    for (uint8_t sr = 1u; sr <= 3u; sr++) {
        uint8_t shift = (uint8_t)((sr - 1u) * 8u);
        uint8_t cur   = (uint8_t)(chip->status_reg >> shift);
        uint8_t m     = (uint8_t)(mask >> shift);
        uint8_t want  = (uint8_t)((cur & ~m) | ((uint8_t)(bits >> shift) & m));
        if (want != cur) {
            st = W25Q_WriteStatus_RTOS_NoLock(chip, sr, want, cfg->reg.sr_write, args->timeout_ms);
            if (st != W25Q_OK) goto release;
        }
    }

    /* Current address mode (ADS): read-only bit, switched by command (ADP only acts at power-up). */
    if (cfg->reg.addr_mode != W25Q_CFG_KEEP) {
        bool want_4b = (cfg->reg.addr_mode == W25Q_CFG_ADS_4B);
        if (W25Q_STATUS_REG(chip, W25Q_SR3_ADS_BIT) != want_4b) {
            st = W25Q_SendCmd_RTOS_NoLock(chip, want_4b ? W25Q_ENABLE_4B_MODE : W25Q_DISABLE_4B_MODE, args->timeout_ms);
            if (st != W25Q_OK) goto release;
        }
        mask |= 1UL << W25Q_SR3_ADS_BIT;
        if (want_4b) bits |= 1UL << W25Q_SR3_ADS_BIT;
    }

    /* Verify: a write can be silently refused (SRP / WP pin / lock bits). */
    for (uint8_t sr = 1u; sr <= 3u; sr++) {
        st = W25Q_ReadStatus_RTOS_NoLock(chip, sr);
        if (st != W25Q_OK) goto release;
    }
    if ((chip->status_reg & mask) != bits) st = W25Q_CHIP_ERR;

release:
    return W25Q_Unlock(chip, true, st);
}

TASK_DEFINE(W25Q_Write) {
    /* Clamping and walking mutate `args` freely: it is this slot's private copy. */
    if (args->addr >= W25Q_FLASH_SIZE_BYTES) {
        return W25Q_PARAM_ERR;
    }
    if (args->len > (uint32_t)W25Q_FLASH_SIZE_BYTES - args->addr) {
        args->len = (uint32_t)W25Q_FLASH_SIZE_BYTES - args->addr;
    }

    while (args->len > 0u) {
        uint32_t room  = W25Q_MEM_PAGE_SIZE - (args->addr % W25Q_MEM_PAGE_SIZE);
        uint16_t chunk = (uint16_t)((args->len < room) ? args->len : room);

        W25Q_STATE st = W25Q_PageProgram_RTOS(args->chip, args->buf, args->addr, chunk, args->timeout_ms);
        if (st != W25Q_OK) {
            return st;
        }

        args->len  -= chunk;
        args->addr += chunk;
        args->buf  += chunk;
    }

    return W25Q_OK;
}

TASK_DEFINE(W25Q_Read) {
    W25Q_t    *chip = args->chip;
    W25Q_STATE st;

    if (args->addr >= W25Q_FLASH_SIZE_BYTES) {
        return W25Q_PARAM_ERR;
    }
    if (args->len > (uint32_t)W25Q_FLASH_SIZE_BYTES - args->addr) {
        args->len = (uint32_t)W25Q_FLASH_SIZE_BYTES - args->addr;
    }

    st = W25Q_Lock(chip, true, args->timeout_ms);
    if (st != W25Q_OK) return st;

    st = W25Q_WaitForReady_RTOS_NoLock(chip, args->timeout_ms);
    if (st != W25Q_OK) goto release;

    const uint8_t cmd[5] = {
        W25Q_READ_DATA_4B,
        (uint8_t)(args->addr >> 24),
        (uint8_t)(args->addr >> 16),
        (uint8_t)(args->addr >>  8),
        (uint8_t)(args->addr >>  0),
    };

    st = W25Q_SPI_Begin_RTOS(chip);
    if (st != W25Q_OK) goto release;

    st = W25Q_SPI_Tx_RTOS(chip, cmd, sizeof(cmd));

    /* The SPI layer takes a 16-bit length: stream the span in chunks, CS held low. */
    while (st == W25Q_OK && args->len > 0u) {
        uint16_t chunk = (uint16_t)((args->len > 0xFFFFu) ? 0xFFFFu : args->len);
        st = W25Q_SPI_Rx_RTOS(chip, args->buf, chunk);
        args->buf += chunk;
        args->len -= chunk;
    }

    /* End the transaction whatever happened, without masking a transfer error. */
    W25Q_STATE end_st = W25Q_SPI_End_RTOS(chip);
    if (st == W25Q_OK) st = end_st;

release:
    return W25Q_Unlock(chip, true, st);
}

#endif /* APEX_CFG_SCHED_RTOS == 1 */
