# ifndef SX127x_H
# define SX127x_H

#include "sx127x/sx127x_common.h"
#include "sx127x/sx127x_lora.h"
#include "sx127x/sx127x_fsk_ook.h"

typedef struct sx127x_chip_t {
    sx127x_base_chip_t base_chip;
    union {
        sx127x_LORA_chip_t lora;
        sx127x_FSK_OOK_chip_t fsk_ook;
    } mod_chip;
} sx127x_chip_t;

typedef union sx127x_mod_config_t {
    sx127x_LORA_config_t lora;
    sx127x_FSK_OOK_config_t fsk_ook;
} sx127x_mod_config_t;

sx127x_status_t sx127x_Init(sx127x_chip_t *chip, sx127x_base_config_t base_config, sx127x_modulation_t modulation, sx127x_mod_config_t config);

sx127x_status_t sx127x_TxSend(sx127x_chip_t *chip, const uint8_t *data, uint16_t len);
sx127x_status_t sx127x_RxReceive(sx127x_chip_t *chip, uint8_t *data, uint16_t *len);

# endif // SX127x_H