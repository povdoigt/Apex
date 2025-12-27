#include "drivers/sx127x.h"
#include "drivers/sx127x/sx127x_common.h"
#include <stdint.h>




sx127x_status_t sx127x_Init(sx127x_chip_t *chip, SPI_HandleTypeDef *spiHandle,
							GPIO_TypeDef *csPinBank, uint16_t csPin, sx127x_config_t config) {
    sx127x_status_t status;
    status = __sx127x_Init(chip, spiHandle, csPinBank, csPin,
                           config.frequency, config.ocp_current_mA, config.power_dBm);
    if (status != sx127x_STATUS_OK) { return status; }

    if (config.isLoRa) {
        return sx127x_LORA_Init(chip, config.modeConfig.lora);
    } else {
        return sx127x_FSK_Init(chip, config.modeConfig.fsk);
    }
}