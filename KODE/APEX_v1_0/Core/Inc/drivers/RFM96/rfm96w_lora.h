// RFM96.h
//
// Definitions for HopeRF RFM96 LoRa radios
//
// Portions adapted from RadioHead RH_RF95.h
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2014 Mike McCauley
// $Id: RH_RF95.h,v 1.22 2019/07/14 00:18:48 mikem Exp $
// 


#ifndef RFM96W_LORA_h
#define RFM96W_LORA_h

#include "stm32f4xx_hal.h"
#include "peripherals/spi.h"
#include "utils/scheduler.h"
#include <stdbool.h>

/* Public define -------------------------------------------------------------*/

// Max number of octets the LORA Rx/Tx FIFO can hold
#define RFM96_LORA_FIFO_SIZE 255

// The crystal oscillator frequency of the module
#define RFM96_LORA_FXOSC 32000000.0

// The Frequency Synthesizer step = RFM96_LORA_FXOSC / 2^^19
#define RFM96_LORA_FSTEP  (RFM96_LORA_FXOSC / (1 << 19))

// ATTENTION A UTILISER
#define RFM96_LORA_FREQUENCE 868.25 // Frequency in MHz



// Register Write Not Read mas
#define RFM96_LORA_REG_WRITE_MASK                        0x80
#define RFM96_LORA_REG_READ_MASK                         0x00


// Register names (LoRa Mode, from table 85)
#define RFM96_LORA_REG_00_FIFO                           0x00
#define RFM96_LORA_LORA_REG_01_OP_MODE                   0x01
#define RFM96_LORA_LORA_REG_02_RESERVED                  0x02
#define RFM96_LORA_LORA_REG_03_RESERVED                  0x03
#define RFM96_LORA_LORA_REG_04_RESERVED                  0x04
#define RFM96_LORA_LORA_REG_05_RESERVED                  0x05
#define RFM96_LORA_LORA_REG_06_FRF_MSB                   0x06
#define RFM96_LORA_LORA_REG_07_FRF_MID                   0x07
#define RFM96_LORA_LORA_REG_08_FRF_LSB                   0x08
#define RFM96_LORA_LORA_REG_09_PA_CONFIG                 0x09
#define RFM96_LORA_LORA_REG_0A_PA_RAMP                   0x0a
#define RFM96_LORA_LORA_REG_0B_OCP                       0x0b
#define RFM96_LORA_LORA_REG_0C_LNA                       0x0c
#define RFM96_LORA_LORA_REG_0D_FIFO_ADDR_PTR             0x0d
#define RFM96_LORA_LORA_REG_0E_FIFO_TX_BASE_ADDR         0x0e
#define RFM96_LORA_LORA_REG_0F_FIFO_RX_BASE_ADDR         0x0f
#define RFM96_LORA_LORA_REG_10_FIFO_RX_CURRENT_ADDR      0x10
#define RFM96_LORA_LORA_REG_11_IRQ_FLAGS_MASK            0x11
#define RFM96_LORA_LORA_REG_12_IRQ_FLAGS                 0x12
#define RFM96_LORA_LORA_REG_13_RX_NB_BYTES               0x13
#define RFM96_LORA_LORA_REG_14_RX_HEADER_CNT_VALUE_MSB   0x14
#define RFM96_LORA_LORA_REG_15_RX_HEADER_CNT_VALUE_LSB   0x15
#define RFM96_LORA_LORA_REG_16_RX_PACKET_CNT_VALUE_MSB   0x16
#define RFM96_LORA_LORA_REG_17_RX_PACKET_CNT_VALUE_LSB	0x17
#define RFM96_LORA_LORA_REG_18_MODEM_STAT                0x18
#define RFM96_LORA_LORA_REG_19_PKT_SNR_VALUE             0x19
#define RFM96_LORA_LORA_REG_1A_PKT_RSSI_VALUE            0x1a
#define RFM96_LORA_LORA_REG_1B_RSSI_VALUE                0x1b
#define RFM96_LORA_LORA_REG_1C_HOP_CHANNEL               0x1c
#define RFM96_LORA_LORA_REG_1D_MODEM_CONFIG1             0x1d
#define RFM96_LORA_LORA_REG_1E_MODEM_CONFIG2             0x1e
#define RFM96_LORA_LORA_REG_1F_SYMB_TIMEOUT_LSB          0x1f
#define RFM96_LORA_LORA_REG_20_PREAMBLE_MSB              0x20
#define RFM96_LORA_LORA_REG_21_PREAMBLE_LSB              0x21
#define RFM96_LORA_LORA_REG_22_PAYLOAD_LENGTH            0x22
#define RFM96_LORA_LORA_REG_23_MAX_PAYLOAD_LENGTH        0x23
#define RFM96_LORA_LORA_REG_24_HOP_PERIOD                0x24
#define RFM96_LORA_LORA_REG_25_FIFO_RX_BYTE_ADDR         0x25
#define RFM96_LORA_LORA_REG_26_MODEM_CONFIG3             0x26

#define RFM96_LORA_LORA_REG_27_PPM_CORRECTION            0x27
#define RFM96_LORA_LORA_REG_28_FEI_MSB                   0x28
#define RFM96_LORA_LORA_REG_29_FEI_MID                   0x29
#define RFM96_LORA_LORA_REG_2A_FEI_LSB                   0x2a
#define RFM96_LORA_LORA_REG_2C_RSSI_WIDEBAND             0x2c
#define RFM96_LORA_LORA_REG_31_DETECT_OPTIMIZ            0x31
#define RFM96_LORA_LORA_REG_33_INVERT_IQ                 0x33
#define RFM96_LORA_LORA_REG_37_DETECTION_THRESHOLD       0x37
#define RFM96_LORA_LORA_REG_39_SYNC_WORD                 0x39

#define RFM96_LORA_LORA_REG_40_DIO_MAPPING1              0x40
#define RFM96_LORA_LORA_REG_41_DIO_MAPPING2              0x41
#define RFM96_LORA_LORA_REG_42_VERSION                   0x42

#define RFM96_LORA_LORA_REG_4B_TCXO           			0x4b
#define RFM96_LORA_LORA_REG_4D_PA_DAC         			0x4d
#define RFM96_LORA_LORA_REG_5B_FORMER_TEMP    			0x5b
#define RFM96_LORA_LORA_REG_61_AGC_REF        			0x61
#define RFM96_LORA_LORA_REG_62_AGC_THRESH1    			0x62
#define RFM96_LORA_LORA_REG_63_AGC_THRESH2    			0x63
#define RFM96_LORA_LORA_REG_64_AGC_THRESH3    			0x64

// RFM96_LORA_LORA_REG_01_OP_MODE                             0x01
#define RFM96_LORA_LORA_LONG_RANGE_MODE                  0x80
#define RFM96_LORA_LORA_ACCESS_SHARED_REG                0x40
#define RFM96_LORA_LORA_LOW_FREQUENCY_MODE               0x08
#define RFM96_LORA_LORA_MODE                             0x07
#define RFM96_LORA_LORA_MODE_SLEEP                       0x00
#define RFM96_LORA_LORA_MODE_STDBY                       0x01
#define RFM96_LORA_LORA_MODE_FSTX                        0x02
#define RFM96_LORA_LORA_MODE_TX                          0x03
#define RFM96_LORA_LORA_MODE_FSRX                        0x04
#define RFM96_LORA_LORA_MODE_RXCONTINUOUS                0x05
#define RFM96_LORA_LORA_MODE_RXSINGLE                    0x06
#define RFM96_LORA_LORA_MODE_CAD                         0x07

// RFM96_LORA_LORA_REG_09_PA_CONFIG                           0x09
#define RFM96_LORA_LORA_PA_SELECT                        0x80
#define RFM96_LORA_LORA_MAX_POWER                        0x70
#define RFM96_LORA_LORA_OUTPUT_POWER                     0x0f

// RFM96_LORA_LORA_REG_0A_PA_RAMP                             0x0a
#define RFM96_LORA_LORA_LOW_PN_TX_PLL_OFF                0x10
#define RFM96_LORA_LORA_PA_RAMP                          0x0f
#define RFM96_LORA_LORA_PA_RAMP_3_4MS                    0x00
#define RFM96_LORA_LORA_PA_RAMP_2MS                      0x01
#define RFM96_LORA_LORA_PA_RAMP_1MS                      0x02
#define RFM96_LORA_LORA_PA_RAMP_500US                    0x03
#define RFM96_LORA_LORA_PA_RAMP_250US                    0x04
#define RFM96_LORA_LORA_PA_RAMP_125US                    0x05
#define RFM96_LORA_LORA_PA_RAMP_100US                    0x06
#define RFM96_LORA_LORA_PA_RAMP_62US                     0x07
#define RFM96_LORA_LORA_PA_RAMP_50US                     0x08
#define RFM96_LORA_LORA_PA_RAMP_40US                     0x09
#define RFM96_LORA_LORA_PA_RAMP_31US                     0x0a
#define RFM96_LORA_LORA_PA_RAMP_25US                     0x0b
#define RFM96_LORA_LORA_PA_RAMP_20US                     0x0c
#define RFM96_LORA_LORA_PA_RAMP_15US                     0x0d
#define RFM96_LORA_LORA_PA_RAMP_12US                     0x0e
#define RFM96_LORA_LORA_PA_RAMP_10US                     0x0f

// RFM96_LORA_LORA_REG_0B_OCP                                 0x0b
#define RFM96_LORA_LORA_OCP_ON                           0x20
#define RFM96_LORA_LORA_OCP_TRIM                         0x1f

// RFM96_LORA_LORA_REG_0C_LNA                                 0x0c
#define RFM96_LORA_LORA_LNA_GAIN                         0xe0
#define RFM96_LORA_LORA_LNA_GAIN_G1                      0x20
#define RFM96_LORA_LORA_LNA_GAIN_G2                      0x40
#define RFM96_LORA_LORA_LNA_GAIN_G3                      0x60
#define RFM96_LORA_LORA_LNA_GAIN_G4                      0x80
#define RFM96_LORA_LORA_LNA_GAIN_G5                      0xa0
#define RFM96_LORA_LORA_LNA_GAIN_G6                      0xc0
#define RFM96_LORA_LORA_LNA_BOOST_LF                     0x18
#define RFM96_LORA_LORA_LNA_BOOST_LF_DEFAULT             0x00
#define RFM96_LORA_LORA_LNA_BOOST_HF                     0x03
#define RFM96_LORA_LORA_LNA_BOOST_HF_DEFAULT             0x00
#define RFM96_LORA_LORA_LNA_BOOST_HF_150PC               0x11

// RFM96_LORA_LORA_REG_11_IRQ_FLAGS_MASK                      0x11
#define RFM96_LORA_LORA_RX_TIMEOUT_MASK                  0x80
#define RFM96_LORA_LORA_RX_DONE_MASK                     0x40
#define RFM96_LORA_LORA_PAYLOAD_CRC_ERROR_MASK           0x20
#define RFM96_LORA_LORA_VALID_HEADER_MASK                0x10
#define RFM96_LORA_LORA_TX_DONE_MASK                     0x08
#define RFM96_LORA_LORA_CAD_DONE_MASK                    0x04
#define RFM96_LORA_LORA_FHSS_CHANGE_CHANNEL_MASK         0x02
#define RFM96_LORA_LORA_CAD_DETECTED_MASK                0x01

// RFM96_LORA_LORA_REG_12_IRQ_FLAGS                           0x12
#define RFM96_LORA_LORA_RX_TIMEOUT                       0x80
#define RFM96_LORA_LORA_RX_DONE                          0x40
#define RFM96_LORA_LORA_PAYLOAD_CRC_ERROR                0x20
#define RFM96_LORA_LORA_VALID_HEADER                     0x10
#define RFM96_LORA_LORA_TX_DONE                          0x08
#define RFM96_LORA_LORA_CAD_DONE                         0x04
#define RFM96_LORA_LORA_FHSS_CHANGE_CHANNEL              0x02
#define RFM96_LORA_LORA_CAD_DETECTED                     0x01

// RFM96_LORA_LORA_REG_18_MODEM_STAT                          0x18
#define RFM96_LORA_LORA_RX_CODING_RATE                   0xe0
#define RFM96_LORA_LORA_MODEM_STATUS_CLEAR               0x10
#define RFM96_LORA_LORA_MODEM_STATUS_HEADER_INFO_VALID   0x08
#define RFM96_LORA_LORA_MODEM_STATUS_RX_ONGOING          0x04
#define RFM96_LORA_LORA_MODEM_STATUS_SIGNAL_SYNCHRONIZED 0x02
#define RFM96_LORA_LORA_MODEM_STATUS_SIGNAL_DETECTED     0x01

// RFM96_LORA_LORA_REG_1C_HOP_CHANNEL                         0x1c
#define RFM96_LORA_LORA_PLL_TIMEOUT                      0x80
#define RFM96_LORA_LORA_RX_PAYLOAD_CRC_IS_ON             0x40
#define RFM96_LORA_LORA_FHSS_PRESENT_CHANNEL             0x3f

// RFM96_LORA_LORA_REG_1D_MODEM_CONFIG1                       0x1d
#define RFM96_LORA_LORA_BW_7_8KHZ                             0x00
#define RFM96_LORA_BW_10_4KHZ                            0x01
#define RFM96_LORA_BW_15_6KHZ                            0x02
#define RFM96_LORA_BW_20_8KHZ                            0x03
#define RFM96_LORA_BW_31_25KHZ                           0x04
#define RFM96_LORA_BW_41_7KHZ                            0x05
#define RFM96_LORA_BW_62_5KHZ                            0x06
#define RFM96_LORA_BW_125KHZ                             0x07
#define RFM96_LORA_BW_250KHZ                             0x08
#define RFM96_LORA_BW_500KHZ                             0x09
// #define RFM96_LORA_CODING_RATE                           0x0e
// #define RFM96_LORA_CODING_RATE_4_5                       0x02
// #define RFM96_LORA_CODING_RATE_4_6                       0x04
// #define RFM96_LORA_CODING_RATE_4_7                       0x06
// #define RFM96_LORA_CODING_RATE_4_8                       0x08
#define RFM96_LORA_IMPLICIT_HEADER_MODE_ON               0x01
#define RFM96_LORA_IMPLICIT_HEADER_MODE_OFF              0x00

// RFM96_LORA_LORA_REG_1E_MODEM_CONFIG2                       0x1e
#define RFM96_LORA_SPREADING_FACTOR_64CPS                0x60	// 255 bytes ->   13 ms
#define RFM96_LORA_SPREADING_FACTOR_128CPS               0x70	// 255 bytes ->  398 ms
#define RFM96_LORA_SPREADING_FACTOR_256CPS               0x80	// 255 bytes ->  701 ms
#define RFM96_LORA_SPREADING_FACTOR_512CPS               0x90	// 255 bytes -> 1256 ms
#define RFM96_LORA_SPREADING_FACTOR_1024CPS              0xa0	// 255 bytes -> 2264 ms
#define RFM96_LORA_SPREADING_FACTOR_2048CPS              0xb0	// 255 bytes -> 4115 ms
#define RFM96_LORA_SPREADING_FACTOR_4096CPS              0xc0	// 255 bytes -> 7735 ms
#define RFM96_LORA_TX_CONTINUOUS_MODE                    0x08

#define RFM96_LORA_PAYLOAD_CRC_ON                        0x04
#define RFM96_LORA_PAYLOAD_CRC_OFF                       0x00
#define RFM96_LORA_SYM_TIMEOUT_MSB                       0x03

// RFM96_LORA_LORA_REG_26_MODEM_CONFIG3
#define RFM96_LORA_MOBILE_NODE                           0x08 // HopeRF term
#define RFM96_LORA_LOW_DATA_RATE_OPTIMIZE                0x08 // Semtechs term
#define RFM96_LORA_AGC_AUTO_ON                           0x04

// RFR96_REG_42_VERSION                             0x42
#define RFM96_LORA_VERSION                               0x12 // 0x11 is also found

// RFM96_LORA_LORA_REG_4B_TCXO                                0x4b
#define RFM96_LORA_TCXO_TCXO_INPUT_ON                    0x10

// RFM96_LORA_LORA_REG_4D_PA_DAC                              0x4d
#define RFM96_LORA_PA_DAC_DISABLE                        0x04
#define RFM96_LORA_PA_DAC_ENABLE                         0x07

/* Public function prototypes ------------------------------------------------*/

typedef struct RFM96_LORA_Chip {
	SPI_HandleTypeDef    *spiHandle;		// SPI Handle
	GPIO_TypeDef 	     *csPinBank;		// Chip Select Pin Bank
	uint16_t 		      csPin;			// Chip Select Pin
	GPIO_TypeDef 	     *resetPinBank;		// Reset Pin Bank
	uint16_t 		      resetPin;			// Reset Pin
} RFM96_LORA_Chip;


/*
Macro that call the ASYNC_SPI_TxRx_DMA_init function with the RFM96 chip's parameters
It requires:
	- SCHEUDLER struct named "scheduler"
	- TASK struct named "task"
	- ASYNC_RFM96_LORA_..._CONTEXT struct named "context" which is the context of the task
... to be defined in the current scope.
Parameters:
	- (uint8_t*)txBuf: the buffer to transmit
	- (uint8_t*)rxBuf: the buffer to receive
	- (size_t)txLen: the length of the buffer to transmit
	- (size_t)rxLen: the length of the buffer to receive
*/
#define ASYNC_SPI_TxRx_DMA_static_init_RFM96(txBuf, rxBuf, txLen, rxLen) \
	ASYNC_SPI_TxRx_DMA_static_init(task, \
							context->RFM96_LORA_chip->spiHandle, \
							context->RFM96_LORA_chip->csPinBank, context->RFM96_LORA_chip->csPin, \
							txBuf, rxBuf, txLen, rxLen, self)


void	RFM96_LORA_Init(RFM96_LORA_Chip        *RFM96_LORA_chip,
			       SPI_HandleTypeDef *spiHandle,
			       GPIO_TypeDef      *csPinBank,
			       uint16_t           csPin,
				   GPIO_TypeDef      *resetPinBank,
				   uint16_t           resetPin,
				   double			   frequency);
void	RFM96_LORA_Reset(RFM96_LORA_Chip *RFM96_LORA_chip);
uint8_t	RFM96_LORA_GetVersion(RFM96_LORA_Chip *RFM96_LORA_chip);
void	RFM96_LORA_LoRaStandBy(RFM96_LORA_Chip *RFM96_LORA_chip);
void	RFM96_LORA_LoRaSleep(RFM96_LORA_Chip *RFM96_LORA_chip);
void	RFM96_LORA_SetFrequency(RFM96_LORA_Chip *RFM96_LORA_chip, double frequency);
double	RFM96_LORA_GetFrequency(RFM96_LORA_Chip *RFM96_LORA_chip);
void 	RFM96_LORA_SetTxPower(RFM96_LORA_Chip *RFM96_LORA_chip, int8_t power);
void	RFM96_LORA_SetOCP(RFM96_LORA_Chip *RFM96_LORA_chip, uint8_t mA);
void	RFM96_LORA_BeginPacket(RFM96_LORA_Chip *RFM96_LORA_chip);
void	RFM96_LORA_EndPacket(RFM96_LORA_Chip *RFM96_LORA_chip);
void	RFM96_LORA_Write(RFM96_LORA_Chip *RFM96_LORA_chip, uint8_t *data, uint16_t len);
void	RFM96_LORA_WriteString(RFM96_LORA_Chip *RFM96_LORA_chip, char *data);
void	RFM96_LORA_Print(RFM96_LORA_Chip *RFM96_LORA_chip, char *data);
int 	RFM96_LORA_ParsePacket(RFM96_LORA_Chip *RFM96_LORA_chip);
void	RFM96_LORA_Read(RFM96_LORA_Chip *RFM96_LORA_chip, uint8_t *buff, int size);
void	RFM96_LORA_ReadRegisters(RFM96_LORA_Chip *RFM96_LORA_chip, uint8_t reg, uint8_t *data, uint16_t len);
uint8_t RFM96_LORA_ReadRegister(RFM96_LORA_Chip *RFM96_LORA_chip, uint8_t reg);
void	RFM96_LORA_WriteRegisters(RFM96_LORA_Chip *RFM96_LORA_chip, uint8_t reg, uint8_t *data, uint16_t len);
void	RFM96_LORA_WriteRegister(RFM96_LORA_Chip *RFM96_LORA_chip, uint8_t reg, uint8_t data);
void	RFM96_LORA_TransmitReceive(RFM96_LORA_Chip *RFM96_LORA_chip, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t tx_size, uint16_t rx_size);


// =======================================================================


// #define ASYNC_RFM96_LORA_BeginPacket_NUMBER 10

// typedef enum ASYNC_RFM96_LORA_BeginPacket_State {
// 	ASYNC_RFM96_LORA_BeginPacket_WAIT_READY,
// 	ASYNC_RFM96_LORA_BeginPacket_LORA_STANDBY,
// 	ASYNC_RFM96_LORA_BeginPacket_GET_CONFIG,
// 	ASYNC_RFM96_LORA_BeginPacket_SET_CONFIG,
// 	ASYNC_RFM96_LORA_BeginPacket_SET_FIFO_ADDR_PTR,
// 	ASYNC_RFM96_LORA_BeginPacket_PAYLOAD_LENGTH,
// 	ASYNC_RFM96_LORA_BeginPacket_END
// } ASYNC_RFM96_LORA_BeginPacket_State;

// typedef struct ASYNC_RFM96_LORA_BeginPacket_CONTEXT {
// 	RFM96_LORA_Chip *RFM96_LORA_chip;		// RFM96 chip

// 	uint8_t tx_buf[2];
// 	uint8_t rx_buf;

// 	bool is_done;			// Pointer to a boolean to indicate if sub-task are done

// 	ASYNC_RFM96_LORA_BeginPacket_State state;// Current state of the task
// 	ASYNC_RFM96_LORA_BeginPacket_State next_state; // Next state of the task
// } ASYNC_RFM96_LORA_BeginPacket_CONTEXT;

// extern TASK_POOL_CREATE(ASYNC_RFM96_LORA_BeginPacket);

// void ASYNC_RFM96_LORA_BeginPacket_init(TASK *self, RFM96_LORA_Chip *RFM96_LORA_chip);
// TASK_RETURN ASYNC_RFM96_LORA_BeginPacket(SCHEDULER *scheduler, TASK *self);


// // =======================================================================


// #define ASYNC_RFM96_LORA_EndPacket_NUMBER 10

// typedef enum ASYNC_RFM96_LORA_EndPacket_State {
// 	ASYNC_RFM96_LORA_EndPacket_WAIT_READY,
// 	ASYNC_RFM96_LORA_EndPacket_SET_TX_MODE,
// 	ASYNC_RFM96_LORA_EndPacket_ASK_TX_DONE,
// 	ASYNC_RFM96_LORA_EndPacket_WAIT_TX_DONE,
// 	ASYNC_RFM96_LORA_EndPacket_CLEAR_IRQ_FLAGS,
// 	ASYNC_RFM96_LORA_EndPacket_END
// } ASYNC_RFM96_LORA_EndPacket_State;

// typedef struct ASYNC_RFM96_LORA_EndPacket_CONTEXT {
// 	RFM96_LORA_Chip *RFM96_LORA_chip;		// RFM96 chip
// 	bool is_done;				// Pointer to a boolean to indicate if sub-task are done

// 	uint8_t tx_buf[2];			// Transmit buffer
// 	uint8_t rx_buf;				// Receive buffer

// 	ASYNC_RFM96_LORA_EndPacket_State state; // Current state of the task
// 	ASYNC_RFM96_LORA_EndPacket_State next_state; // Next state of the task
// } ASYNC_RFM96_LORA_EndPacket_CONTEXT;

// extern TASK_POOL_CREATE(ASYNC_RFM96_LORA_EndPacket);

// void ASYNC_RFM96_LORA_EndPacket_init(TASK *self, RFM96_LORA_Chip *RFM96_LORA_chip);
// TASK_RETURN ASYNC_RFM96_LORA_EndPacket(SCHEDULER *scheduler, TASK *self);


// // =======================================================================


// #define ASYNC_RFM96_LORA_WriteFIFO_NUMBER 10

// typedef enum ASYNC_RFM96_LORA_WriteFIFO_State {
// 	ASYNC_RFM96_LORA_WriteFIFO_WAIT_READY,
// 	ASYNC_RFM96_LORA_WriteFIFO_GET_CURRENT_LEN,
// 	ASYNC_RFM96_LORA_WriteFIFO_GET_MAX_LEN,
// 	ASYNC_RFM96_LORA_WriteFIFO_WRITE_FIFO,
// 	ASYNC_RFM96_LORA_WriteFIFO_UPDATE_LEN,
// 	ASYNC_RFM96_LORA_WriteFIFO_END
// } ASYNC_RFM96_LORA_WriteFIFO_State;

// typedef struct ASYNC_RFM96_LORA_WriteFIFO_CONTEXT {
// 	RFM96_LORA_Chip *RFM96_LORA_chip;		// RFM96 chip
// 	uint8_t tx_fifo_buf[257];		// Transmit FIFO buffer (1 byte for the register address + 256 bytes for the FIFO data)
// 	uint8_t tx_fifo_size;		// Transmit FIFO size
// 	uint8_t tx_buf[2];			// Transmit buffer

// 	bool is_done;				// Pointer to a boolean to indicate if sub-task are done

// 	uint8_t current_len;		// Current length of the FIFO
// 	uint8_t max_len;			// Max length of the FIFO

// 	ASYNC_RFM96_LORA_WriteFIFO_State state; // Current state of the task
// 	ASYNC_RFM96_LORA_WriteFIFO_State next_state; // Next state of the task
// } ASYNC_RFM96_LORA_WriteFIFO_CONTEXT;

// extern TASK_POOL_CREATE(ASYNC_RFM96_LORA_WriteFIFO);

// void ASYNC_RFM96_LORA_WriteFIFO_init(TASK *self, RFM96_LORA_Chip *RFM96_LORA_chip, uint8_t *tx_buf, uint16_t tx_size);
// TASK_RETURN ASYNC_RFM96_LORA_WriteFIFO(SCHEDULER *scheduler, TASK *self);


// // =======================================================================


// #define ASYNC_RFM96_LORA_SendPacket_NUMBER 10

// typedef enum ASYNC_RFM96_LORA_SendPacket_State {
// 	ASYNC_RFM96_LORA_SendPacket_WAIT_READY,
// 	ASYNC_RFM96_LORA_SendPacket_BEGIN_PACKET,
// 	ASYNC_RFM96_LORA_SendPacket_WRITE_FIFO,
// 	ASYNC_RFM96_LORA_SendPacket_END_PACKET,
// 	ASYNC_RFM96_LORA_SendPacket_END
// } ASYNC_RFM96_LORA_SendPacket_State;

// typedef struct ASYNC_RFM96_LORA_SendPacket_CONTEXT {
// 	RFM96_LORA_Chip *RFM96_LORA_chip;		// RFM96 chip
// 	uint8_t *tx_buf;			// Transmit buffer
// 	uint8_t tx_size;			// Transmit size

// 	bool is_done;				// Pointer to a boolean to indicate if sub-task are done

// 	ASYNC_RFM96_LORA_SendPacket_State state; // Current state of the task
// 	ASYNC_RFM96_LORA_SendPacket_State next_state; // Next state of the task
// } ASYNC_RFM96_LORA_SendPacket_CONTEXT;

// extern TASK_POOL_CREATE(ASYNC_RFM96_LORA_SendPacket);

// void ASYNC_RFM96_LORA_SendPacket_init(TASK *self, RFM96_LORA_Chip *RFM96_LORA_chip, uint8_t *tx_buf, uint8_t tx_size);
// TASK_RETURN ASYNC_RFM96_LORA_SendPacket(SCHEDULER *scheduler, TASK *self);


// // ====================================================================


// #define ASYNC_RFM96_LORA_SetRxMode_NUMBER 5

// typedef enum ASYNC_RFM96_LORA_SetRxMode_State {
// 	ASYNC_RFM96_LORA_SetRxMode_Wait,
// 	ASYNC_RFM96_LORA_SetRxMode_GetConfig1,
// 	ASYNC_RFM96_LORA_SetRxMode_SetConfig1_Mode_FIFO,
// 	ASYNC_RFM96_LORA_SetRxMode_DONE
// } ASYNC_RFM96_LORA_SetRxMode_State;

// typedef struct ASYNC_RFM96_LORA_SetRxMode_CONTEXT {
// 	RFM96_LORA_Chip *RFM96_LORA_chip;

// 	uint8_t tx0[2];
// 	uint8_t tx1[2];
// 	uint8_t tx2[2];
// 	uint8_t rx;

// 	bool is_done[3];

// 	ASYNC_RFM96_LORA_SetRxMode_State state; // Current state of the task
// 	ASYNC_RFM96_LORA_SetRxMode_State next_state; // Next state of the task
// } ASYNC_RFM96_LORA_SetRxMode_CONTEXT;

// extern TASK_POOL_CREATE(ASYNC_RFM96_LORA_SetRxMode);

// void ASYNC_RFM96_LORA_SetRxMode_init(TASK *self, RFM96_LORA_Chip *RFM96_LORA_chip);

// TASK_RETURN ASYNC_RFM96_LORA_SetRxMode(SCHEDULER *scheduler, TASK *self);


// // ====================================================================


// #define ASYNC_RFM96_LORA_ParsePacket_NUMBER 5

// typedef enum ASYNC_RFM96_LORA_ParsePacket_State {
// 	ASYNC_RFM96_LORA_ParsePacket_Wait,
// 	ASYNC_RFM96_LORA_ParsePacket_GetIRQ,
// 	ASYNC_RFM96_LORA_ParsePacket_ResetIRQ,
	
// 	ASYNC_RFM96_LORA_ParsePacket_GetLen,
// 	ASYNC_RFM96_LORA_ParsePacket_IsPacketAvailable,

// 	ASYNC_RFM96_LORA_ParsePacket_GetRxAddr,
// 	ASYNC_RFM96_LORA_ParsePacket_SetRxAddr,
// 	ASYNC_RFM96_LORA_ParsePacket_GetData,
// 	ASYNC_RFM96_LORA_ParsePacket_Done,
// 	// IF RX RECEIVED
// } ASYNC_RFM96_LORA_ParsePacket_State;

// typedef struct ASYNC_RFM96_LORA_ParsePacket_CONTEXT {
// 	RFM96_LORA_Chip *RFM96_LORA_chip;

// 	uint8_t *len;
// 	uint8_t i;
// 	uint8_t *data;

// 	uint8_t tx[2];
// 	uint8_t rx;

// 	bool is_done;

// 	ASYNC_RFM96_LORA_ParsePacket_State state; // Current state of the task
// 	ASYNC_RFM96_LORA_ParsePacket_State next_state; // Next state of the task
// } ASYNC_RFM96_LORA_ParsePacket_CONTEXT;

// extern TASK_POOL_CREATE(ASYNC_RFM96_LORA_ParsePacket);

// void ASYNC_RFM96_LORA_ParsePacket_init(TASK *self, RFM96_LORA_Chip *RFM96_LORA_chip, uint8_t *data, uint8_t *len);

// TASK_RETURN ASYNC_RFM96_LORA_ParsePacket(SCHEDULER *scheduler, TASK *self);

#endif // RFM96W_LORA_h