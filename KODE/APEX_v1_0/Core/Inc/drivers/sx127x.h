# ifndef sx127x_h
# define sx127x_h

#include "sx127x/sx127x_common.h"
#include "sx127x/sx127x_lora.h"
#include "sx127x/sx127x_fsk.h"



typedef union sx127x_config_mode_t {
	sx127x_lora_config_t	lora;
	sx127x_fsk_config_t		fsk;
} sx127x_config_mode_t;

typedef struct sx127x_config_t {
	uint32_t				frequency;	// in Hz
	bool					isLoRa;		// true: LoRa mode, false: FSK/OOK mode
	sx127x_config_mode_t	modeConfig;	// Mode specific configuration
} sx127x_config_t;

sx127x_status_t sx127x_Init(sx127x_chip_t *chip, SPI_HandleTypeDef *spiHandle,
							GPIO_TypeDef *csPinBank, uint16_t csPin, sx127x_config_t *config);

# endif // sx127x_h