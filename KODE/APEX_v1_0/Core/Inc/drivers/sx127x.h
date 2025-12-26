# ifndef sx127x_h
# define sx127x_h

# include "sx127x/sx127x_lora.h"
# include "sx127x/sx127x_fsk.h"

// Max number of octets the LORA Rx/Tx FIFO can hold
#define sx127x_FIFO_SIZE 255

// The crystal oscillator frequency of the module
#define sx127x_FXOSC 32000000.0

// The Frequency Synthesizer step = sx127x_LORA_FXOSC / 2^^19
#define sx127x_FSTEP  (sx127x_FXOSC / (1 << 19))

// The Frequency range of the module (3 bands: 137MHz - 1020MHz)
// Available bands for sx1276/7/8, sx1279 is not supported
#define sx127x_BAND_3_MIN_FREQ  137000000
#define sx127x_BAND_3_MAX_FREQ  175000000
#define sx127x_BAND_2_MIN_FREQ  410000000
#define sx127x_BAND_2_MAX_FREQ  525000000
#define sx127x_BAND_1_MIN_FREQ  862000000
#define sx127x_BAND_1_MAX_FREQ 1020000000




// Register Write Not Read mas
#define sx127x_REG_WRITE_MASK	0x80
#define sx127x_REG_READ_MASK	0x00


// Common Register names (FSK/OOK and LoRa Mode)  
#define sx127x_REG_00_FIFO			0x00
#define sx127x_REG_01_OP_MODE		0x01
// REG 02~05 are mode specific
#define sx127x_REG_06_FRF_MSB   	0x06
#define sx127x_REG_07_FRF_MID   	0x07
#define sx127x_REG_08_FRF_LSB   	0x08
#define sx127x_REG_09_PA_CONFIG 	0x09
#define sx127x_REG_0A_PA_RAMP   	0x0a
#define sx127x_REG_0B_OCP       	0x0b
#define sx127x_REG_0C_LNA       	0x0c
// REG 0D~40 are mode specific
#define sx127x_REG_40_DIO_MAPPING1	0x40
#define sx127x_REG_41_DIO_MAPPING2	0x41
#define sx127x_REG_42_VERSION     	0x42
// REG 44 is mode specific, 43 and 45~4A are reserved
#define sx127x_REG_4B_TCXO       	0x4b
#define sx127x_REG_4D_PA_DAC     	0x4d
#define sx127x_REG_5B_FORMER_TEMP	0x5b
#define sx127x_REG_61_AGC_REF    	0x61
#define sx127x_REG_62_AGC_THRESH1	0x62
#define sx127x_REG_63_AGC_THRESH2	0x63
#define sx127x_REG_64_AGC_THRESH3	0x64
// REG 65~6F are reserved
#define sx127x_REG_70_PLL_HOP     	0x70





typedef struct sx127x_Chip {
	SPI_HandleTypeDef    *spiHandle;		// SPI Handle
	GPIO_TypeDef 	     *csPinBank;		// Chip Select Pin Bank
	uint16_t 		      csPin;			// Chip Select Pin
	GPIO_TypeDef 	     *resetPinBank;		// Reset Pin Bank
	uint16_t 		      resetPin;			// Reset Pin
} sx127x_Chip;



# endif // sx127x_h