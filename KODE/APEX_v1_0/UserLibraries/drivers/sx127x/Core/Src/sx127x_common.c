#include "sx127x_common.h"
#include <math.h>
#include <stdint.h>





// =========================== Level 0: SPI Read/Write functions ===========================

static inline void sx127x_SPI_Begin(sx127x_base_chip_t *chip) {
    HAL_GPIO_WritePin(chip->config.csPinBank, chip->config.csPin, GPIO_PIN_RESET);
}

static inline sx127x_status_t sx127x_SPI_Tx(sx127x_base_chip_t *chip, const uint8_t *tx_buf, uint16_t tx_len) {
    if (!chip || (!tx_buf && tx_len)) { return sx127x_STATUS_ERROR; }
    HAL_StatusTypeDef status = HAL_SPI_Transmit(chip->config.spiHandle, tx_buf, tx_len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? sx127x_STATUS_OK : sx127x_STATUS_ERROR;
}

static inline sx127x_status_t sx127x_SPI_Rx(sx127x_base_chip_t *chip, uint8_t *rx_buf, uint16_t rx_len) {
    if (!chip || (!rx_buf && rx_len)) { return sx127x_STATUS_ERROR; }
    HAL_StatusTypeDef status = HAL_SPI_Receive(chip->config.spiHandle, rx_buf, rx_len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? sx127x_STATUS_OK : sx127x_STATUS_ERROR;
}

static inline void sx127x_SPI_End(sx127x_base_chip_t *chip) {
    HAL_GPIO_WritePin(chip->config.csPinBank, chip->config.csPin, GPIO_PIN_SET);
}





// ================================== Level 1: Basic Read/Write functions ==================================

sx127x_status_t sx127x_RegWrite(sx127x_base_chip_t *chip, uint8_t reg, uint8_t value) {
    uint8_t tx_buf[2] = { reg | sx127x_REG_WRITE_MSK, value };

    sx127x_SPI_Begin(chip);
    sx127x_status_t status = sx127x_SPI_Tx(chip, tx_buf, 2);
    sx127x_SPI_End(chip);
    return status;
}

sx127x_status_t sx127x_RegWriteMulti(sx127x_base_chip_t *chip, uint8_t reg, const uint8_t *buffer, uint8_t buf_size) {
    if (buf_size == 0) {
        return sx127x_STATUS_OK;
    }
    
    uint8_t tx_reg = reg | sx127x_REG_WRITE_MSK;

    sx127x_status_t status;
    sx127x_SPI_Begin(chip);
    status = sx127x_SPI_Tx(chip, &tx_reg, 1);
    if (status != sx127x_STATUS_OK) {
        sx127x_SPI_End(chip);
        return status;
    }
    status = sx127x_SPI_Tx(chip, buffer, buf_size);
    sx127x_SPI_End(chip);
    return status;
}

sx127x_status_t sx127x_RegRead(sx127x_base_chip_t *chip, uint8_t reg, uint8_t *value) {
    uint8_t tx_buf[1] = { reg & ~sx127x_REG_WRITE_MSK };

    sx127x_SPI_Begin(chip);
    sx127x_status_t status = sx127x_SPI_Tx(chip, tx_buf, 1);
    if (status != sx127x_STATUS_OK) {
        sx127x_SPI_End(chip);
        return status;
    }
    status = sx127x_SPI_Rx(chip, value, 1);
    sx127x_SPI_End(chip);
    return status;
}

sx127x_status_t sx127x_RegReadMulti(sx127x_base_chip_t *chip, uint8_t reg, uint8_t *buffer, uint8_t buf_size) {
    if (buf_size == 0) {
        return sx127x_STATUS_OK;
    }

    uint8_t tx_reg = reg & ~sx127x_REG_WRITE_MSK;

    sx127x_status_t status;
    sx127x_SPI_Begin(chip);
    status = sx127x_SPI_Tx(chip, &tx_reg, 1);
    if (status != sx127x_STATUS_OK) {
        sx127x_SPI_End(chip);
        return status;
    }
    status = sx127x_SPI_Rx(chip, buffer, buf_size);
    sx127x_SPI_End(chip);
    return status;
}





// ================================== Level 2: High level function ==================================

void __sx127x_SetPwrvalue(sx127x_base_chip_t *chip, float Pout, uint8_t *pMaxPower, uint8_t *pOutputPower);


sx127x_status_t sx127x_InitBase(sx127x_base_chip_t *chip, sx127x_base_config_t config) {
    if (!chip || !config.spiHandle || !config.csPinBank) { return sx127x_STATUS_ERROR; }

    chip->config = config;

    // Set CS pin high (if not already set)
    HAL_GPIO_WritePin(chip->config.csPinBank, chip->config.csPin, GPIO_PIN_SET);

    // Small delay to ensure the chip is ready after initialization
    HAL_Delay(10);

    uint8_t value;
    sx127x_status_t status;

    // Check the version register to verify communication
    status = sx127x_RegRead(chip, sx127x_REG_42_VERSION, &value);
    if (status != sx127x_STATUS_OK || (value != sx127x_VERSION_ID)) { return sx127x_STATUS_ERROR; }

    // Set OCP
    status = sx127x_SetOcp(chip, config.ocp_current_mA);
    if (status != sx127x_STATUS_OK) { return status; }

	// Set lna boost
	value = sx127x_REG_0C_LNA_LNA_GAIN_G1			// Gain set to G1 (highest)
		  | sx127x_REG_0C_LNA_LNA_BOOST_LF_DFT		// Normal LNA current (LF)
		  | sx127x_REG_0C_LNA_LNA_BOOST_HF_BOOST;	// LNA current boosted by 150% (HF)
	status = sx127x_RegWrite(chip, sx127x_REG_0C_LNA, value);
	if (status != sx127x_STATUS_OK) { return status; }

    // Check and set the frequency band
    status = sx127x_SetFrequency(chip, config.frequency);
    if (status != sx127x_STATUS_OK) { return status; }

    // Set power level
    status = sx127x_SetTxPower(chip, config.power_dBm);

    return status;
}

sx127x_status_t sx127x_GetBand(sx127x_base_chip_t *chip, uint8_t *band) {
    if (!chip || !band) { return sx127x_STATUS_ERROR; }

    if (sx127x_BAND_1_MIN_FREQ <= chip->config.frequency && chip->config.frequency <= sx127x_BAND_1_MAX_FREQ) {
        *band = sx127x_BAND_1;
    } else if (sx127x_BAND_2_MIN_FREQ <= chip->config.frequency && chip->config.frequency <= sx127x_BAND_2_MAX_FREQ) {
        *band = sx127x_BAND_2;
    } else if (sx127x_BAND_3_MIN_FREQ <= chip->config.frequency && chip->config.frequency <= sx127x_BAND_3_MAX_FREQ) {
        *band = sx127x_BAND_3;
    } else {
        return sx127x_STATUS_ERROR;
    }

    return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_SetFrequency(sx127x_base_chip_t *chip, uint32_t frequency) {
    if (!chip) { return sx127x_STATUS_ERROR; }

    // Check and set the frequency band
    uint8_t band;
    if (sx127x_GetBand(chip, &band) != sx127x_STATUS_OK) { return sx127x_STATUS_ERROR; }
    frequency = (uint32_t)((float)frequency / sx127x_FSTEP);

    uint8_t freq_bytes[3] = {
        (uint8_t)(frequency >> 16),
        (uint8_t)(frequency >>  8),
        (uint8_t)(frequency >>  0)
    };
    return sx127x_RegWriteMulti(chip, sx127x_REG_06_FRF_MSB, freq_bytes, 3);
}

sx127x_status_t sx127x_SetTxPower(sx127x_base_chip_t *chip, float power_dBm) {
    if (!chip) { return sx127x_STATUS_ERROR; }

    if (power_dBm < sx127x_OUTPUT_POWER_MIN || power_dBm > sx127x_OUTPUT_POWER_MAX) { return sx127x_STATUS_ERROR; }

    uint8_t value;
    value = 0x00;
    value |= sx127x_REG_4D_PA_DAC_DEFAULT;
    value |= power_dBm == 20 ? sx127x_REG_4D_PA_DAC_PA_DAC_HP : sx127x_REG_4D_PA_DAC_PA_DAC_DFT;
    sx127x_RegWrite(chip, sx127x_REG_4D_PA_DAC, value);

    if (power_dBm > 17) {
        power_dBm = 17; // Limit to 17 dBm max output power
    }
    if (power_dBm > 15) {
        // Use PA Boost
        value = 0x00;
        value |= sx127x_REG_09_PA_CONFIG_PA_SELECT_PA_BOOST;
        value |= sx127x_REG_09_PA_CONFIG_MAX_POWER_7; // Pmax = 15.0 dBm
        value |= (uint8_t)(power_dBm - 2); // OutputPower
        return sx127x_RegWrite(chip, sx127x_REG_09_PA_CONFIG, value);
    } else {
        // Use RFO (does not work for any reason... - use PA Boost anyway)
        uint8_t pMaxPower, pOutputPower;
        __sx127x_SetPwrvalue(chip, power_dBm, &pMaxPower, &pOutputPower);
        value = 0x00;
        value |= sx127x_REG_09_PA_CONFIG_PA_SELECT_RFO;
        value |= pMaxPower;      // [6-4] MaxPower
        value |= pOutputPower;   // [3-0] OutputPower
        return sx127x_RegWrite(chip, sx127x_REG_09_PA_CONFIG, value);
    }
}

sx127x_status_t sx127x_SetOcp(sx127x_base_chip_t *chip, float ocp_current_mA) {
    if (!chip) { return sx127x_STATUS_ERROR; }

    if (ocp_current_mA < sx127x_OCP_IMAX_MIN || ocp_current_mA > sx127x_OCP_IMAX_MAX) { return sx127x_STATUS_ERROR; }

    uint8_t ocp_trim = 0;
    if (ocp_current_mA <= 120) {
        ocp_trim = (uint8_t)((ocp_current_mA - 45) / 5);
    } else {
        ocp_trim = (uint8_t)((ocp_current_mA + 30) / 10);
    }
    ocp_trim &= sx127x_REG_0B_OCP_OCP_TRIM_MSK; // Set bits [4-0] for OcpTrim value
    ocp_trim |= sx127x_REG_0B_OCP_OCP_ON_ON;   // Enable OCP

    return sx127x_RegWrite(chip, sx127x_REG_0B_OCP, ocp_trim);
}

void __sx127x_SetPwrvalue(sx127x_base_chip_t *chip, float Pout, uint8_t *pMaxPower, uint8_t *pOutputPower) {
    /**
    -4.2 <= Pout <= 15 dBm
    Pout = -4.2 + n0 * 0.6 + n1 with n0 in [[0; 7]] and n1 in [[0; 15]]
    Find n0 and n1 such as Pout = -4.2 + n0 * 0.6 + n1
    Pout = -4.2 + n0 * 0.6 + n1
    Ptemp = Pout + 4.2

    Ptemp || n0 | n1
    ----------------
    00.0 || 0  | 0 
    00.2 || -  | -    03.0 || 0  | 3    15.0 || 0  | 15  ----------------
    00.4 || -  | -    03.2 || 2  | 2    15.2 || 2  | 14   18.0 || 5  | 15
    00.6 || 1  | 0    03.4 || 4  | 1    15.4 || 4  | 13   18.2 || 7  | 14
    00.8 || -  | -    03.6 || 1  | 3    15.6 || 1  | 15   18.4 || -  | - 
    ----------------  03.8 || 3  | 2    15.8 || 3  | 14   18.6 || 6  | 15
    01.0 || 0  | 1   ----------------  ----------------   18.8 || -  | - 
    01.2 || 2  | 0          ...        ----------------  ----------------
    01.4 || -  | -          ...         16.0 || 5  | 13   19.0 || -  | - 
    01.6 || 1  | 1          ...         16.2 || 2  | 15   19.2 || 7  | 15
    01.8 || 3  | 0          ...         16.4 || 4  | 14 
    ----------------        ...         16.6 || 2  | 15 
    ----------------  ----------------  16.8 || 3  | 15 
    02.0 || 0  | 2    15.0 || 0  | 14  ---------------- 
    02.2 || 2  | 1    15.2 || 2  | 13   17.0 || 5  | 14 
    02.4 || 4  | 0    15.4 || 4  | 12   17.2 || 7  | 13 
    02.6 || 1  | 2    15.6 || 1  | 14   17.4 || -  | -  
    02.8 || 3  | 1    15.8 || 3  | 13   17.6 || 6  | 14 
    ----------------  ----------------  17.8 || -  | -  

    if 2.0 <= Ptemp <= 15.8:
        m = fabs(Ptemp)
        f = x - m
        if f = 0.0 -> n0 = 0, n1 = m
        if f = 0.2 -> n0 = 2, n1 = m - 1
        if f = 0.4 -> n0 = 4, n1 = m - 2
        if f = 0.6 -> n0 = 1, n1 = m
        if f = 0.8 -> n0 = 3, n1 = m - 1
    */

    uint8_t n0, n1;

    float Ptemp = Pout + 4.2f;
    if (Ptemp < 1.0f) {
        if (Ptemp < 0.3f) {
            n0 = 0; n1 = 0;
        } else if (Ptemp < 0.8f) {
            n0 = 1; n1 = 0;
        } else {
            n0 = 0; n1 = 1;
        }
    } else if (Ptemp <  2.0f) {
        if (Ptemp < 1.1f) {
            n0 = 0; n1 = 1;
        } else if (Ptemp < 1.4f) {
            n0 = 2; n1 = 0;
        } else if (Ptemp < 1.7f) {
            n0 = 1; n1 = 1;
        } else {
            n0 = 3; n1 = 0;
        }
    } else if (Ptemp < 16.0f) {
        uint8_t m = (uint8_t)fabs(Ptemp); // integer part
        float f = Ptemp - (float)(m * 1); // fractional part
        if (f < 0.1f) {
            n0 = 0; n1 = m;
        } else if (f < 0.3f) {
            n0 = 2; n1 = m - 1;
        } else if (f < 0.5f) {
            n0 = 4; n1 = m - 2;
        } else if (f < 0.7f) {
            n0 = 1; n1 = m;
        } else {
            n0 = 3; n1 = m - 1;
        }
    } else if (Ptemp < 17.0f) {
        if (Ptemp < 16.1f) {
            n0 = 5; n1 = 13;
        } else if (Ptemp < 16.3f) {
            n0 = 2; n1 = 15;
        } else if (Ptemp < 16.5f) {
            n0 = 4; n1 = 14;
        } else if (Ptemp < 16.7f) {
            n0 = 2; n1 = 15;
        } else {
            n0 = 3; n1 = 15;
        }
    } else if (Ptemp < 18.0f) {
        if (Ptemp < 17.1f) {
            n0 = 5; n1 = 14;
        } else if (Ptemp < 17.4f) {
            n0 = 7; n1 = 13;
        } else if (Ptemp < 17.8f) {
            n0 = 6; n1 = 14;
        } else {
            n0 = 7; n1 = 15;
        }
    } else if (Ptemp < 19.0f) {
        if (Ptemp < 18.1f) {
            n0 = 5; n1 = 15;
        } else if (Ptemp < 18.4f) {
            n0 = 7; n1 = 14;
        } else if (Ptemp < 18.9f) {
            n0 = 6; n1 = 15;
        } else {
            n0 = 7; n1 = 15;
        }
    } else {
        n0 = 7; n1 = 15;
    }

    *pMaxPower = (n0 << 4) & sx127x_REG_09_PA_CONFIG_MAX_POWER_MSK;    // [6-4] MaxPower
    *pOutputPower = n1 & sx127x_REG_09_PA_CONFIG_OUTPUT_POWER_MSK;     // [3-0] OutputPower
}




/* ============================== FreeRTOS ============================== */

#if (APEX_CFG_SCHED_RTOS == 1)


/* Niveau 0: Primitives SPI */

static inline sx127x_status_t sx127x_SPI_Begin_RTOS(sx127x_base_chip_t *chip) {
    return SPI_Begin_DMA_RTOS(chip->config.spiHandle, chip->config.csPinBank, chip->config.csPin) == HAL_OK ? sx127x_STATUS_OK : sx127x_STATUS_SEM_ERROR;
}

static inline sx127x_status_t sx127x_SPI_Tx_RTOS(sx127x_base_chip_t *chip, const uint8_t *tx_buf, uint16_t tx_len) {
    return SPI_Transmit_DMA_RTOS(chip->config.spiHandle, (uint8_t *)tx_buf, tx_len) == HAL_OK ? sx127x_STATUS_OK : sx127x_STATUS_SPI_TIMEOUT;
}

static inline sx127x_status_t sx127x_SPI_Rx_RTOS(sx127x_base_chip_t *chip, uint8_t *rx_buf, uint16_t rx_len) {
    return SPI_Receive_DMA_RTOS(chip->config.spiHandle, rx_buf, rx_len) == HAL_OK ? sx127x_STATUS_OK : sx127x_STATUS_SPI_TIMEOUT;
}

static inline sx127x_status_t sx127x_SPI_End_RTOS(sx127x_base_chip_t *chip) {
    return SPI_End_DMA_RTOS(chip->config.spiHandle, chip->config.csPinBank, chip->config.csPin) == HAL_OK ? sx127x_STATUS_OK : sx127x_STATUS_SEM_ERROR;
}

/* Niveau 1: Primitives de lecture/écriture */

sx127x_status_t sx127x_RegWrite_RTOS_base(sx127x_base_chip_t *chip, uint8_t reg, uint8_t val, bool lock_sem) {
    sx127x_status_t st;
    uint8_t tx_buf[2] = { reg | sx127x_REG_WRITE_MSK, val };

    if (lock_sem) {
        if (osSemaphoreAcquire(chip->sem_id, osWaitForever) != osOK) { return sx127x_STATUS_SEM_ERROR; }
    }

    st = sx127x_SPI_Begin_RTOS(chip);
    if (st != sx127x_STATUS_OK) { goto exit_flag; }
    st = sx127x_SPI_Tx_RTOS(chip, tx_buf, sizeof(tx_buf));
    if (st != sx127x_STATUS_OK) { sx127x_SPI_End_RTOS(chip); goto exit_flag; }
    st = sx127x_SPI_End_RTOS(chip);

exit_flag:
    if (lock_sem) {
        if (osSemaphoreRelease(chip->sem_id) != osOK) { return sx127x_STATUS_SEM_ERROR; }
    }
    return st;
}

sx127x_status_t sx127x_RegRead_RTOS_base(sx127x_base_chip_t *chip, uint8_t reg, uint8_t *val, bool lock_sem) {
    sx127x_status_t st;
    uint8_t tx_buf[1] = { reg & ~sx127x_REG_WRITE_MSK };

    if (lock_sem) {
        if (osSemaphoreAcquire(chip->sem_id, osWaitForever) != osOK) { return sx127x_STATUS_SEM_ERROR; }
    }

    st = sx127x_SPI_Begin_RTOS(chip);
    if (st != sx127x_STATUS_OK) { goto exit_flag; }
    st = sx127x_SPI_Tx_RTOS(chip, tx_buf, sizeof(tx_buf));
    if (st != sx127x_STATUS_OK) { sx127x_SPI_End_RTOS(chip); goto exit_flag; }
    st = sx127x_SPI_Rx_RTOS(chip, val, 1);
    if (st != sx127x_STATUS_OK) { sx127x_SPI_End_RTOS(chip); goto exit_flag; }
    st = sx127x_SPI_End_RTOS(chip);

exit_flag:
    if (lock_sem) {
        if (osSemaphoreRelease(chip->sem_id) != osOK) { return sx127x_STATUS_SEM_ERROR; }
    }
    return st;
}

sx127x_status_t sx127x_RegWriteMulti_RTOS_base(sx127x_base_chip_t *chip, uint8_t reg, const uint8_t *buffer, uint8_t buf_size, bool lock_sem) {
    sx127x_status_t st;
    uint8_t tx_reg = reg | sx127x_REG_WRITE_MSK;

    if (lock_sem) {
        if (osSemaphoreAcquire(chip->sem_id, osWaitForever) != osOK) { return sx127x_STATUS_SEM_ERROR; }
    }

    st = sx127x_SPI_Begin_RTOS(chip);
    if (st != sx127x_STATUS_OK) { goto exit_flag; }
    st = sx127x_SPI_Tx_RTOS(chip, &tx_reg, 1);
    if (st != sx127x_STATUS_OK) { sx127x_SPI_End_RTOS(chip); goto exit_flag; }
    st = sx127x_SPI_Tx_RTOS(chip, (uint8_t *)buffer, buf_size);
    if (st != sx127x_STATUS_OK) { sx127x_SPI_End_RTOS(chip); goto exit_flag; }
    st = sx127x_SPI_End_RTOS(chip);

exit_flag:
    if (lock_sem) {
        if (osSemaphoreRelease(chip->sem_id) != osOK) { return sx127x_STATUS_SEM_ERROR; }
    }
    return st;
}

sx127x_status_t sx127x_RegReadMulti_RTOS_base(sx127x_base_chip_t *chip, uint8_t reg, uint8_t *buffer, uint8_t buf_size, bool lock_sem) {
    sx127x_status_t st;
    uint8_t tx_reg = reg & ~sx127x_REG_WRITE_MSK;

    if (lock_sem) {
        if (osSemaphoreAcquire(chip->sem_id, osWaitForever) != osOK) { return sx127x_STATUS_SEM_ERROR; }
    }

    st = sx127x_SPI_Begin_RTOS(chip);
    if (st != sx127x_STATUS_OK) { goto exit_flag; }
    st = sx127x_SPI_Tx_RTOS(chip, &tx_reg, 1);
    if (st != sx127x_STATUS_OK) { sx127x_SPI_End_RTOS(chip); goto exit_flag; }
    st = sx127x_SPI_Rx_RTOS(chip, buffer, buf_size);
    if (st != sx127x_STATUS_OK) { sx127x_SPI_End_RTOS(chip); goto exit_flag; }
    st = sx127x_SPI_End_RTOS(chip);

exit_flag:
    if (lock_sem) {
        if (osSemaphoreRelease(chip->sem_id) != osOK) { return sx127x_STATUS_SEM_ERROR; }
    }
    return st;
}

/* Niveau 2: Fonctions de plus haut niveau */

sx127x_status_t sx127x_InitBase_RTOS(sx127x_base_chip_t *chip, sx127x_base_config_t config) {
    if (!chip || !config.spiHandle || !config.csPinBank) { return sx127x_STATUS_ERROR; }

    chip->config = config;

    // Creattion of the semaphore
    chip->sem_id = osSemaphoreNew(1, 0, &(osSemaphoreAttr_t){
        .name = "SX127x_Semaphore",
        .cb_mem = &(chip->sem),
        .cb_size = sizeof(chip->sem)
    });
    if (chip->sem_id == NULL) {
        return sx127x_STATUS_SEM_ERROR;
    }

    // Set CS pin high (if not already set)
    HAL_GPIO_WritePin(chip->config.csPinBank, chip->config.csPin, GPIO_PIN_SET);

    // Small delay to ensure the chip is ready after initialization
    osDelay(10);

    uint8_t value;
    sx127x_status_t status;

    // Check the version register to verify communication
    status = sx127x_RegRead_RTOS_NoLock(chip, sx127x_REG_42_VERSION, &value);
    if (status != sx127x_STATUS_OK || (value != sx127x_VERSION_ID)) { status = sx127x_STATUS_ERROR; goto exit_flag; }

    // Set OCP
    status = sx127x_SetOcp_RTOS(chip, config.ocp_current_mA);
    if (status != sx127x_STATUS_OK) { goto exit_flag; }

    // Set lna boost
    value = sx127x_REG_0C_LNA_LNA_GAIN_G1			// Gain set to G1 (highest)
          | sx127x_REG_0C_LNA_LNA_BOOST_LF_DFT		// Normal LNA current (LF)
          | sx127x_REG_0C_LNA_LNA_BOOST_HF_BOOST;	// LNA current boosted by 150% (HF)
    status = sx127x_RegWrite_RTOS_NoLock(chip, sx127x_REG_0C_LNA, value);
    if (status != sx127x_STATUS_OK) { goto exit_flag; }

    // Check and set the frequency band
    status = sx127x_SetFrequency_RTOS(chip, config.frequency);
    if (status != sx127x_STATUS_OK) { goto exit_flag; }

    // Set power level
    status = sx127x_SetTxPower_RTOS(chip, config.power_dBm);

exit_flag:

    if (osSemaphoreRelease(chip->sem_id) != osOK) { return sx127x_STATUS_SEM_ERROR; }

    return status;
}

sx127x_status_t sx127x_SetFrequency_RTOS(sx127x_base_chip_t *chip, uint32_t frequency) {
    if (!chip) { return sx127x_STATUS_ERROR; }

    // Check and set the frequency band
    uint8_t band;
    if (sx127x_GetBand(chip, &band) != sx127x_STATUS_OK) { return sx127x_STATUS_ERROR; }
    frequency = (uint32_t)((float)frequency / sx127x_FSTEP);

    uint8_t freq_bytes[3] = {
        (uint8_t)(frequency >> 16),
        (uint8_t)(frequency >>  8),
        (uint8_t)(frequency >>  0)
    };

    return sx127x_RegWriteMulti_RTOS_NoLock(chip, sx127x_REG_06_FRF_MSB, freq_bytes, 3);
}

sx127x_status_t sx127x_SetTxPower_RTOS(sx127x_base_chip_t *chip, float power_dBm) {
    if (!chip) { return sx127x_STATUS_ERROR; }

    if (power_dBm < sx127x_OUTPUT_POWER_MIN || power_dBm > sx127x_OUTPUT_POWER_MAX) { return sx127x_STATUS_ERROR; }

    uint8_t value;
    sx127x_status_t status;
    value = 0x00;
    value |= sx127x_REG_4D_PA_DAC_DEFAULT;
    value |= power_dBm == 20 ? sx127x_REG_4D_PA_DAC_PA_DAC_HP : sx127x_REG_4D_PA_DAC_PA_DAC_DFT;
    status = sx127x_RegWrite_RTOS_NoLock(chip, sx127x_REG_4D_PA_DAC, value);
    if (status != sx127x_STATUS_OK) { return status; }

    if (power_dBm > 17) {
        power_dBm = 17; // Limit to 17 dBm max output power
    }
    if (power_dBm > 15) {
        // Use PA Boost
        value = 0x00;
        value |= sx127x_REG_09_PA_CONFIG_PA_SELECT_PA_BOOST;
        value |= sx127x_REG_09_PA_CONFIG_MAX_POWER_7; // Pmax = 15.0 dBm
        value |= (uint8_t)(power_dBm - 2); // OutputPower
        status = sx127x_RegWrite_RTOS_NoLock(chip, sx127x_REG_09_PA_CONFIG, value);
    } else {
        // Use RFO (does not work for any reason... - use PA Boost anyway)
        uint8_t pMaxPower, pOutputPower;
        __sx127x_SetPwrvalue(chip, power_dBm, &pMaxPower, &pOutputPower);
        value = 0x00;
        value |= sx127x_REG_09_PA_CONFIG_PA_SELECT_RFO;
        value |= pMaxPower;      // [6-4] MaxPower
        value |= pOutputPower;   // [3-0] OutputPower
        status = sx127x_RegWrite_RTOS_NoLock(chip, sx127x_REG_09_PA_CONFIG, value);
    }

    return status;
}

sx127x_status_t sx127x_SetOcp_RTOS(sx127x_base_chip_t *chip, float ocp_current_mA) {
    if (!chip) { return sx127x_STATUS_ERROR; }

    if (ocp_current_mA < sx127x_OCP_IMAX_MIN || ocp_current_mA > sx127x_OCP_IMAX_MAX) { return sx127x_STATUS_ERROR; }

    uint8_t ocp_trim = 0;
    if (ocp_current_mA <= 120) {
        ocp_trim = (uint8_t)((ocp_current_mA - 45) / 5);
    } else {
        ocp_trim = (uint8_t)((ocp_current_mA + 30) / 10);
    }
    ocp_trim &= sx127x_REG_0B_OCP_OCP_TRIM_MSK; // Set bits [4-0] for OcpTrim value
    ocp_trim |= sx127x_REG_0B_OCP_OCP_ON_ON;   // Enable OCP

    return sx127x_RegWrite_RTOS_NoLock(chip, sx127x_REG_0B_OCP, ocp_trim);
}

#endif
