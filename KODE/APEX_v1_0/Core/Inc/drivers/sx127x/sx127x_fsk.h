
# ifndef RFM96W_FSK_h
# define RFM96W_FSK_h
// include
#include "stm32f4xx_hal.h"
#include "peripherals/spi.h"
#include "sx127x_common.h"
#include "utils/scheduler.h"
#include <stdbool.h>

// Register names (FSK/OOK Mode)
// REG 00~01 are common
#define sx127x_FSK_REG_02_BITRATE_MSB   0x02
#define sx127x_FSK_REG_03_BITRATE_LSB   0x03
#define sx127x_FSK_REG_04_FDEV_MSB      0x04
#define sx127x_FSK_REG_05_FDEV_LSB      0x05
// REG 06~09 are common
#define sx127x_FSK_REG_0A_PA_RAMP       0x0a





// ====================== 0x01 RegOpMode ======================
// [7] LongRangeMode:
typedef enum sx127x_FSK_REG_01_OP_MODE_LRM {
    sx127x_FSK_REG_01_OP_MODE_LRM_FSK_OOK   = 0b00000000,   // FSK/OOK mode
    sx127x_FSK_REG_01_OP_MODE_LRM_LoRa      = 0b10000000,   // LoRa mode
} sx127x_FSK_REG_01_OP_MODE_LongRangeMode;
#define sx127x_FSK_REG_01_OP_MODE_LRM_MASK 0b10000000
// [6-5] ModulationType:
typedef enum sx127x_FSK_REG_01_OP_MODE_MOD_TYPE {
    sx127x_FSK_REG_01_OP_MODE_MOD_TYPE_FSK  = 0b00000000,   // FSK
    sx127x_FSK_REG_01_OP_MODE_MOD_TYPE_OOK  = 0b00100000,   // OOK
    // 10 reserved
    // 11 reserved 
} sx127x_FSK_REG_01_OP_MODE_ModulationType;
#define sx127x_FSK_REG_01_OP_MODE_MOD_TYPE_MASK 0b01100000
// [4] Reserved
// [3] LowFrequencyModeOn:
typedef enum sx127x_FSK_REG_01_OP_MODE_LFMO {
    sx127x_FSK_REG_01_OP_MODE_LFMO_OFF  = 0b00000000,   // HF mode (> 525MHz)
    sx127x_FSK_REG_01_OP_MODE_LFMO_ON   = 0b00001000,   // LF mode (<= 525MHz)
} sx127x_FSK_REG_01_OP_MODE_LowFrequencyModeOn;
#define sx127x_FSK_REG_01_OP_MODE_LFMO_MASK 0b00001000
// [2-0] Mode:
typedef enum sx127x_FSK_REG_01_OP_MODE_MODE {
    sx127x_FSK_REG_01_OP_MODE_MODE_SLEEP    = 0b00000000,   // Sleep
    sx127x_FSK_REG_01_OP_MODE_MODE_STDBY    = 0b00000001,   // Standby
    sx127x_FSK_REG_01_OP_MODE_MODE_FSTX     = 0b00000010,   // FSTX
    sx127x_FSK_REG_01_OP_MODE_MODE_TX       = 0b00000011,   // Tx
    sx127x_FSK_REG_01_OP_MODE_MODE_FSRX     = 0b00000100,   // FSRX
    sx127x_FSK_REG_01_OP_MODE_MODE_RX       = 0b00000101,   // Rx
    // 110 reserved
    // 111 reserved
} sx127x_FSK_REG_01_OP_MODE_Mode;
#define sx127x_FSK_REG_01_OP_MODE_MODE_MASK 0b00000111





// ====================== 0x0a RegPaRamp ======================
// [7] Unused
// [6-5] ModulationShaping:
typedef enum sx127x_FSK_REG_0A_PA_RAMP_MOD_SHAPING {
    sx127x_FSK_REG_0A_PA_RAMP_MOD_SHAPING_OFF           = 0b00000000,   // No shaping
    sx127x_FSK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_1_0    = 0b00100000,   // FSK, BT = 1.0
    sx127x_FSK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_0_5    = 0b01000000,   // FSK, BT = 0.5
    sx127x_FSK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_0_3    = 0b01100000,   // FSK, BT = 0.3
} sx127x_FSK_REG_0A_PA_RAMP_ModulationShaping;
// For OOK modulation
#define sx127x_FSK_REG_0A_PA_RAMP_MOD_SHAPING_OOK_FC    sx127x_FSK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_1_0    // OOK, fc = bit rate
#define sx127x_FSK_REG_0A_PA_RAMP_MOD_SHAPING_OOK_FC_2  sx127x_FSK_REG_0A_PA_RAMP_MOD_SHAPING_FSK_BT_0_5    // OOK, fc = 2*bit rate
#define sx127x_FSK_REG_0A_PA_RAMP_MOD_SHAPING_MASK 0b01100000
// [4] Reserved
// [3-0] PA Ramp:
typedef enum sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP {
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_3_4MS    = 0b00000000,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_2MS      = 0b00000001,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_1MS      = 0b00000010,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_500US    = 0b00000011,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_250US    = 0b00000100,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_125US    = 0b00000101,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_100US    = 0b00000110,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_62US     = 0b00000111,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_50US     = 0b00001000,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_40US     = 0b00001001,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_31US     = 0b00001010,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_25US     = 0b00001011,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_20US     = 0b00001100,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_15US     = 0b00001101,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_12US     = 0b00001110,
    sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_10US     = 0b00001111,
} sx127x_FSK_REG_0A_PA_RAMP_PaRamp;
#define sx127x_FSK_REG_0A_PA_RAMP_PA_RAMP_MASK 0b00001111









typedef struct sx127x_fsk_config_t {
    uint32_t                                        bitrate;            // in bps
    uint32_t                                        fdev;               // in Hz
    sx127x_FSK_REG_0A_PA_RAMP_ModulationShaping     modulationShaping;
    sx127x_FSK_REG_0A_PA_RAMP_PaRamp                paRamp;
} sx127x_fsk_config_t;



sx127x_status_t sx127x_FSK_Init(sx127x_chip_t *sx127x_chip, sx127x_fsk_config_t *config);


# endif // RFM96W_FSK_h