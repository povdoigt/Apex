#include "drivers/sx127x.h"
#include "drivers/sx127x/sx127x_common.h"

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