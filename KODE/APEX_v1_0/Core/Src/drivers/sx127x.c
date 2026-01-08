#include "drivers/sx127x.h"
#include "cmsis_os2.h"
#include "drivers/sx127x/sx127x_common.h"
#include "stm32f4xx_hal.h"
#include <string.h>

sx127x_status_t sx127x_Init(sx127x_chip_t *chip, sx127x_base_config_t base_config, sx127x_modulation_t modulation, sx127x_mod_config_t config) {
    if (!chip) { return sx127x_STATUS_ERROR; }
    
    sx127x_status_t status;

    status = sx127x_InitBase(&chip->base_chip, base_config);
    if (status != sx127x_STATUS_OK) { return status; }

    switch (modulation) {
        case sx127x_MODULATION_LORA:
            status = sx127x_LORA_Config(&chip->mod_chip.lora, &chip->base_chip, config.lora);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
        case sx127x_MODULATION_FSK:
            status = sx127x_FSK_Config(&chip->mod_chip.fsk_ook, &chip->base_chip, config.fsk_ook);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
        case sx127x_MODULATION_OOK:
            status = sx127x_OOK_Config(&chip->mod_chip.fsk_ook, &chip->base_chip, config.fsk_ook);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
            
        default:
            return sx127x_STATUS_ERROR;
    }
    
    return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_TxSend(sx127x_chip_t *chip, const uint8_t *data, uint16_t len) {
    if (!chip || !data) { return sx127x_STATUS_ERROR; }

    sx127x_status_t status;

    switch (chip->base_chip.modulation) {
        case sx127x_MODULATION_LORA:
            status = sx127x_LORA_TxSend(&chip->mod_chip.lora, data, len);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
        case sx127x_MODULATION_FSK:
        case sx127x_MODULATION_OOK:
            status = sx127x_FSK_OOK_TxSend(&chip->mod_chip.fsk_ook, data, len);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
        default:
            return sx127x_STATUS_ERROR;
    }

    return sx127x_STATUS_OK;
}

sx127x_status_t sx127x_RxReceive(sx127x_chip_t *chip, uint8_t *data, uint16_t *len) {
    if (!chip || !data || !len) { return sx127x_STATUS_ERROR; }

    sx127x_status_t status;

    switch (chip->base_chip.modulation) {
        case sx127x_MODULATION_LORA:
            status = sx127x_LORA_RxReceive(&chip->mod_chip.lora, data, (uint8_t *)len);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
        case sx127x_MODULATION_FSK:
        case sx127x_MODULATION_OOK:
            status = sx127x_FSK_OOK_RxReceive(&chip->mod_chip.fsk_ook, data, len);
            if (status != sx127x_STATUS_OK) { return status; }
            break;
        default:
            return sx127x_STATUS_ERROR;
    }

    return sx127x_STATUS_OK;
}








sx127x_status_t sx127x_Init_RTOS(sx127x_chip_t *chip, sx127x_base_config_t base_config, sx127x_modulation_t modulation, sx127x_mod_config_t config) {
    if (!chip) { return sx127x_STATUS_ERROR; }
    
    sx127x_status_t status;

    status = sx127x_InitBase_RTOS(&chip->base_chip, base_config);
    if (status != sx127x_STATUS_OK) { return status; }

    switch (modulation) {
        case sx127x_MODULATION_LORA:
            status = sx127x_LORA_Config_RTOS(&chip->mod_chip.lora, &chip->base_chip, config.lora);
            break;
        case sx127x_MODULATION_FSK:
            status = sx127x_FSK_Config_RTOS(&chip->mod_chip.fsk_ook, &chip->base_chip, config.fsk_ook);
            break;
        case sx127x_MODULATION_OOK:
            status = sx127x_OOK_Config_RTOS(&chip->mod_chip.fsk_ook, &chip->base_chip, config.fsk_ook);
            break;
            
        default:
            status = sx127x_STATUS_ERROR;
    }
    
    return status;
}

sx127x_status_t sx127x_TxSend_RTOS(sx127x_chip_t *chip, const uint8_t *data, uint16_t len) {
    if (!chip || !data) { return sx127x_STATUS_ERROR; }

    sx127x_status_t status;

    switch (chip->base_chip.modulation) {
        case sx127x_MODULATION_LORA:
            status = sx127x_LORA_TxSend_RTOS(&chip->mod_chip.lora, data, len);
            break;
        case sx127x_MODULATION_FSK:
        case sx127x_MODULATION_OOK:
            status = sx127x_FSK_OOK_TxSend_RTOS(&chip->mod_chip.fsk_ook, data, len);
            break;
        default:
            status = sx127x_STATUS_ERROR;
    }

    return status;
}

sx127x_status_t sx127x_RxReceive_RTOS(sx127x_chip_t *chip, uint8_t *data, uint16_t *len) {
    if (!chip || !data || !len) { return sx127x_STATUS_ERROR; }

    sx127x_status_t status;

    switch (chip->base_chip.modulation) {
        case sx127x_MODULATION_LORA:
            status = sx127x_LORA_RxReceive_RTOS(&chip->mod_chip.lora, data, (uint8_t *)len);
            break;
        case sx127x_MODULATION_FSK:
        case sx127x_MODULATION_OOK:
            status = sx127x_FSK_OOK_RxReceive_RTOS(&chip->mod_chip.fsk_ook, data, len);
            break;
        default:
            status = sx127x_STATUS_ERROR;
    }

    return status;
}


TASK_POOL_ALLOCATE(TASK_sx127x_Init);
void TASK_sx127x_Init(void *argument) {
    TASK_sx127x_Init_ARGS *args = (TASK_sx127x_Init_ARGS *)argument;
    *(args->return_state) = sx127x_Init_RTOS(args->chip, *(args->base_config), args->modulation, *(args->config));
    osEventFlagsSet(args->done_flags, 1);
    osThreadExit_Cstm();
}

TASK_POOL_ALLOCATE(TASK_sx127x_TxSend);
void TASK_sx127x_TxSend(void *argument) {
    TASK_sx127x_TxSend_ARGS *args = (TASK_sx127x_TxSend_ARGS *)argument;
    *(args->return_state) = sx127x_TxSend_RTOS(args->chip, args->data, args->len);
    osEventFlagsSet(args->done_flags, 1);
    osThreadExit_Cstm();
}

TASK_POOL_ALLOCATE(TASK_sx127x_RxReceive);
void TASK_sx127x_RxReceive(void *argument) {
    TASK_sx127x_RxReceive_ARGS *args = (TASK_sx127x_RxReceive_ARGS *)argument;
    *(args->return_state) = sx127x_RxReceive_RTOS(args->chip, args->data, args->len);
    osEventFlagsSet(args->done_flags, 1);
    osThreadExit_Cstm();
}




// ================================== task test ==================================

TASK_POOL_ALLOCATE(TASK_sx127x_Init_TxRx);
void TASK_sx127x_Init_TxRx(void *argument) {
    TASK_sx127x_Init_TxRx_ARGS *args = (TASK_sx127x_Init_TxRx_ARGS *)argument;

	sx127x_status_t sx127x_status;
	StaticEventGroup_t sx127x_done_event_cb;
	osEventFlagsId_t sx127x_done_event_id = osEventFlagsNew(&(osEventFlagsAttr_t){
		.name = "sx127x_done_event",
		.cb_mem = &sx127x_done_event_cb,
		.cb_size = sizeof(sx127x_done_event_cb)
	});
	osEventFlagsClear(sx127x_done_event_id, 0xFFFFFFFF);
	osThreadAttr_t sx127x_attr = {
		.name = "TASK_sx127x_Init",
		.priority = (osPriority_t)osPriorityNormal,
	};
	TASK_sx127x_Init_ARGS sx127x_init_args = {
		.chip = args->chip,
		.base_config = &(sx127x_base_config_t){
			.spiHandle = &hspi1,
			.csPinBank = CS_LORA_GPIO_Port,
			.csPin = CS_LORA_Pin,
			.frequency = 869500000,
			.ocp_current_mA = 240.0,
			.power_dBm = 20.0,
		},
#ifdef IS_LORA
        .modulation = sx127x_MODULATION_LORA,
        .config = &(sx127x_mod_config_t){
            .lora = {
                .implicitHeader = false,
                .bandwidth = sx127x_LORA_REG_1D_MODEM_CONFIG1_BW_125KHZ,
                .codingRate = sx127x_LORA_REG_1D_MODEM_CONFIG1_CR_4_5,
                .spreadingFactor = sx127x_LORA_REG_1E_MODEM_CONFIG2_SF_128CPS,
                .crcEnabled = true,
            }
        },
#else
		.modulation = sx127x_MODULATION_FSK,
		.config = &(sx127x_mod_config_t){
			.fsk_ook = {
				.bitrate = 200000,
				.RxBw = SX127X_FSK_OOK_RxBw_125_0kHz,
				.modShaping = sx127x_FSK_OOK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_0_5,
				.paRamp = sx127x_FSK_OOK_REG_0A_PA_RAMP_PA_RAMP_40US,				// Default PA ramp time
				.packetCfg = {
					.preamble_len = 5,												// Default preamble length
					.sync_on = true,
					.sync_len = 4,													// Default sync length
					.sync_word = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01},	// Default sync word
					.variable_length = true,
					// .payload_len = 63,												// Default payload length
					.crc_on = true,
				},
				.mod_config.fsk = {
					.fdev = 60000,
				},
			},
		},
#endif
		.return_state = &sx127x_status,
		.done_flags = sx127x_done_event_id,
	};
	OS_THREAD_NEW_CSTM(TASK_sx127x_Init, sx127x_init_args, sx127x_attr, osWaitForever);

	osEventFlagsWait(sx127x_done_event_id, 1, osFlagsWaitAll, osWaitForever);


	#ifdef IS_LORA
	uint32_t delay = 500;
	#else
	uint32_t delay = 30;
	#endif

	uint32_t dt_last_rx = delay * 1.5;

	#ifdef IS_SYNC
	bool sync_done = true;
	#else
	bool sync_done = false;
	#endif

    #ifdef IS_SYNC
    char buff_tx[255] = "Bonjour depuis APEX-1!";
    #else
    char buff_tx[255] = "Bonjour depuis APEX-2!";
    #endif
    uint16_t tx_len = strlen(buff_tx);

    sx127x_status_t tx_status;
    TASK_sx127x_TxSend_ARGS tx_args = {
        .chip = args->chip,
        .data = (uint8_t *)buff_tx,
        .len = tx_len,
        .return_state = &tx_status,
        .done_flags = NULL,
    };
    osThreadAttr_t tx_attr = {
        .name = "TASK_sx127x_TxSend",
        .priority = (osPriority_t)osPriorityNormal,
    };

    char buff_rx[256];
    uint16_t rx_len = 0;

    sx127x_status_t rx_status;
    StaticEventGroup_t rx_done_event_cb;
    osEventFlagsId_t rx_done_event_id = osEventFlagsNew(&(osEventFlagsAttr_t){
        .name = "sx127x_rx_done_event",
        .cb_mem = &rx_done_event_cb,
        .cb_size = sizeof(rx_done_event_cb)
    });
    osEventFlagsClear(rx_done_event_id, 0xFFFFFFFF);
    TASK_sx127x_RxReceive_ARGS rx_args = {
        .chip = args->chip,
        .data = (uint8_t *)buff_rx,
        .len = &rx_len,
        .return_state = &rx_status,
        .done_flags = rx_done_event_id,
    };
    osThreadAttr_t rx_attr = {
        .name = "TASK_sx127x_RxReceive",
        .priority = (osPriority_t)osPriorityNormal,
    };

    // /* APEX FSK test */
    // // code émetteur
    // if (sync_done && HAL_GetTick() - t0 >= delay) {
    // 	t0 = HAL_GetTick();
    // 	sprintf(buff1, "%2u: APEX-1: ACC: %s, %s, %s", i++, accx, accy, accz);
    // 	len = strlen(buff1);
    // 	len = (len > 255) ? 255 : len;
    // 	sx127x_status = sx127x_TxSend(&sx127x_chip, (uint8_t*)buff1, 255);
    // 	// HAL_Delay(10);
    // 	// HAL_GPIO_TogglePin(LED0B_GPIO_Port, LED0B_Pin);
    // 	// HAL_Delay(10);
    // 	// HAL_GPIO_TogglePin(LED0B_GPIO_Port, LED0B_Pin);
    // }
    // // code récepteur
    // dt_last_rx = HAL_GetTick() - t1;
    // HAL_GPIO_WritePin(LED0G_GPIO_Port, LED0G_Pin, dt_last_rx < delay * 1.5 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    // sx127x_status = sx127x_RxReceive(&sx127x_chip, (uint8_t*)buff1, &len);
    // if (sx127x_status == sx127x_STATUS_OK && len > 0) {
    // 	if (!sync_done) {
    // 		sync_done = true;
    // 		t0 = HAL_GetTick() + delay / 2; // duty cycle de 50%
    // 	}
    // 	// HAL_Delay(10);
    // 	// HAL_GPIO_TogglePin(LED0G_GPIO_Port, LED0G_Pin);
    // 	// HAL_Delay(10);
    // 	// HAL_GPIO_TogglePin(LED0G_GPIO_Port, LED0G_Pin);
    // 	sprintf(buff2, "(dt: %4u ms), %.*s\n", HAL_GetTick() - t1, len, buff1);
    // 	CDC_Transmit_FS((uint8_t*)buff2, strlen(buff2));
    // 	t1 = HAL_GetTick();
    // }

    uint32_t t0 = HAL_GetTick();

    for (;;) {
        // Emission
        if (sync_done && HAL_GetTick() - t0 >= delay) {
            t0 = HAL_GetTick();
            OS_THREAD_NEW_CSTM(TASK_sx127x_TxSend, tx_args, tx_attr, osWaitForever);
        }

        // Réception
        dt_last_rx = HAL_GetTick() - dt_last_rx;
        HAL_GPIO_WritePin(LED0G_GPIO_Port, LED0G_Pin, dt_last_rx < delay * 1.5 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        OS_THREAD_NEW_CSTM(TASK_sx127x_RxReceive, rx_args, rx_attr, osWaitForever);
        osEventFlagsWait(rx_done_event_id, 1, osFlagsWaitAll, osWaitForever);
        if (rx_status == sx127x_STATUS_OK && rx_len > 0) {
            if (!sync_done) {
                sync_done = true;
                t0 = HAL_GetTick() + delay / 2; // duty cycle de 50%
            }
            dt_last_rx = HAL_GetTick();
        }

        osDelay(1);
    }
}


