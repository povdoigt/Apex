/**
 *******************************************
 * @file    w25q_mem.h
 * @author  Dmitriy Semenov / Crazy_Geeks
 * @version 0.1b
 * @date	12-August-2021
 * @brief   Header for W25Qxxx lib
 * @note 	https://github.com/Crazy-Geeks/STM32-W25Q-QSPI
 *******************************************
 *
 * @note https://ru.mouser.com/datasheet/2/949/w25q256jv_spi_revg_08032017-1489574.pdf
 * @note https://www.st.com/resource/en/application_note/DM00227538-.pdf
*/

#ifndef W25Q_QSPI_W25Q_MEM_H_
#define W25Q_QSPI_W25Q_MEM_H_

#include <stdbool.h>
#include "stm32f4xx_hal.h"

#include "peripherals/spi.h"

#include "utils/scheduler.h"

/**
 * @addtogroup W25Q_Driver
 * @brief W25Q QSPI Driver
 * @{
 */

/**
 * @defgroup W25Q_Param W25Q Chip's Parameters
 * @brief User's chip parameters
 * @{
 */
// YOUR CHIP'S SETTINGS
/// Mem size in MB-bit
#define W25Q_MEM_FLASH_SIZE 512U 	// 512 MB-bit
/// Mem big block size in KB
#define W25Q_MEM_BLOCK_SIZE 64U		// 64 KB: 256 pages
/// Mem small block size in KB
#define W25Q_MEM_SBLOCK_SIZE 32U	// 32 KB: 128 pages
/// Mem sector size in KB
#define W25Q_MEM_SECTOR_SIZE 4U		// 4 KB : 16 pages
/// Mem page size in bytes
#define W25Q_MEM_PAGE_SIZE  256U		// 256 byte : 1 page
/// Blocks count
#define W25Q_BLOCK_COUNT (W25Q_MEM_FLASH_SIZE * 2) // 1024 blocks
/// Sector count
#define W25Q_SECTOR_COUNT (W25Q_BLOCK_COUNT * 16)  // 16'384 sectors
/// Pages count
#define W25Q_PAGE_COUNT (W25Q_SECTOR_COUNT * 16)	 // 262'144 pages



#define W25Q_MANUFACTURER_ID	0xEFU	// W25Q Manufacturer ID
#define W25Q_Q_FULL_DEVICE_ID	0x4018U // W25Q....IQ/JQ Device ID
#define W25Q_M_FULL_DEVICE_ID	0x7018U // W25Q....IM/JM Device ID
#define W25Q_HALF_DEVICE_ID		0x17U	// W25Q Half Device ID

/**
 * @defgroup W25Q_Commands W25Q Chip's Commands
 * @brief W25Q Chip commands from datasheet
 * @{
 */
#define W25Q_WRITE_ENABLE				0x06U	// sets WEL bit, must be set before any write/program/erase
#define W25Q_WRITE_DISABLE				0x04U	// resets WEL bit (state after power-up)
#define W25Q_ENABLE_VOLATILE_SR			0x50U	// check 7.1 in datasheet
#define W25Q_READ_SR1					0x05U	// read status-register 1
#define W25Q_READ_SR2					0x35U	// read status-register 2
#define W25Q_READ_SR3					0x15U	// read ststus-register 3
#define W25Q_WRITE_SR1					0x01U	// write status-register 1 (8.2.5)
#define W25Q_WRITE_SR2					0x31U	// write status-register 2 (8.2.5)
#define W25Q_WRITE_SR3					0x11U	// write status-register 3 (8.2.5)
#define W25Q_READ_EXT_ADDR_REG			0xC8U	// read extended addr reg (only in 3-byte mode)
#define W25Q_WRITE_EXT_ADDR_REG			0xC8U	// write extended addr reg (only in 3-byte mode)
#define W25Q_ENABLE_4B_MODE				0xB7U	// enable 4-byte mode (128+ MB address)
#define W25Q_DISABLE_4B_MODE			0xE9U	// disable 4-byte mode (<=128MB)
#define W25Q_READ_DATA					0x03U	// read data by standard SPI
#define W25Q_READ_DATA_4B				0x13U	// read data by standard SPI in 4-byte mode
#define W25Q_FAST_READ					0x0BU	// highest FR speed (8.2.12)
#define W25Q_FAST_READ_4B				0x0CU	// fast read in 4-byte mode
#define W25Q_FAST_READ_DUAL_OUT			0x3BU	// fast read in dual-SPI OUTPUT (8.2.14)
#define W25Q_FAST_READ_DUAL_OUT_4B		0x3CU	// fast read in dual-SPI OUTPUT in 4-byte mode
#define W25Q_FAST_READ_QUAD_OUT			0x6BU	// fast read in quad-SPI OUTPUT (8.2.16)
#define W25Q_FAST_READ_QUAD_OUT_4B		0x6CU	// fast read in quad-SPI OUTPUT in 4-byte mode
#define W25Q_FAST_READ_DUAL_IO			0xBBU	// fast read in dual-SPI I/O (address transmits by both lines)
#define W25Q_FAST_READ_DUAL_IO_4B		0xBCU	// fast read in dual-SPI I/O in 4-byte mode
#define W25Q_FAST_READ_QUAD_IO			0xEBU	// fast read in quad-SPI I/O (address transmits by quad lines)
#define W25Q_FAST_READ_QUAD_IO_4B		0xECU	// fast read in quad-SPI I/O in 4-byte mode
#define W25Q_SET_BURST_WRAP				0x77U	// use with quad-I/O (8.2.22)
#define W25Q_PAGE_PROGRAM				0x02U	// program page (256bytes) by single SPI line
#define W25Q_PAGE_PROGRAM_4B			0x12U	// program page by single SPI in 4-byte mode
#define W25Q_PAGE_PROGRAM_QUAD_INP		0x32U	// program page (256bytes) by quad SPI lines
#define W25Q_PAGE_PROGRAM_QUAD_INP_4B	0x34U	// program page by quad SPI in 4-byte mode
#define W25Q_SECTOR_ERASE				0x20U	// sets all 4Kbyte sector with 0xFF (erases it)
#define W25Q_SECTOR_ERASE_4B			0x21U	// sets all 4Kbyte sector with 0xFF in 4-byte mode
#define W25Q_32KB_BLOCK_ERASE			0x52U	// sets all 32Kbyte block with 0xFF
#define W25Q_64KB_BLOCK_ERASE			0xD8U	// sets all 64Kbyte block with 0xFF
#define W25Q_64KB_BLOCK_ERASE_4B		0xDCU	// sets all 64Kbyte sector with 0xFF in 4-byte mode
#define W25Q_CHIP_ERASE					0xC7U	// fill all the chip with 0xFF
// #define W25Q_CHIP_ERASE					0x60U	// another way to erase chip
#define W25Q_ERASEPROG_SUSPEND			0x75U	// suspend erase/program operation (can be applied only when SUS=0, BYSY=1)
#define W25Q_ERASEPROG_RESUME			0x7AU	// resume erase/program operation (if SUS=1, BUSY=0)
#define W25Q_POWERDOWN					0xB9U	// powers down the chip (power-up by reading ID)
#define W25Q_POWERUP					0xABU	// release power-down
#define W25Q_DEVID						0xABU	// read Device ID (same as powerup)
#define W25Q_FULLID						0x90U	// read Manufacturer ID & Device ID
#define W25Q_FULLID_DUAL_IO				0x92U	// read Manufacturer ID & Device ID by dual I/O
#define W25Q_FULLID_QUAD_IO				0x94U	// read Manufacturer ID & Device ID by quad I/O
#define W25Q_READ_UID					0x4BU	// read unique chip 64-bit ID
#define W25Q_READ_JEDEC_ID				0x9FU	// read JEDEC-standard ID
#define W25Q_READ_SFDP					0x5AU	// read SFDP register parameters
#define W25Q_ERASE_SECURITY_REG			0x44U	// erase security registers
#define W25Q_PROG_SECURITY_REG			0x42U	// program security registers
#define W25Q_READ_SECURITY_REG			0x48U	// read security registers
#define W25Q_IND_BLOCK_LOCK				0x36U	// make block/sector read-only
#define W25Q_IND_BLOCK_UNLOCK			0x39U	// disable block/sector protection
#define W25Q_READ_BLOCK_LOCK			0x3DU	// check block/sector protection
#define W25Q_GLOBAL_LOCK				0x7EU	// global read-only protection enable
#define W25Q_GLOBAL_UNLOCK				0x98U	// global read-only protection disable
#define W25Q_ENABLE_RESET				0x66U	// enable software-reset ability
#define W25Q_RESET						0x99U	// make software reset

// V	- Volatile bit
// NV	- Non-volatile bit
// Status Register Bits	(SR1)
#define W25Q_SR1_BUSY_BIT		0	// Erase/Write in progress					(Read only)
#define W25Q_SR1_WEL_BIT		1	// Write enable latch (1 - write allowed) 	(Read only)
#define W25Q_SR1_BP0_BIT		2	// Block protect 0							(V/NV)
#define W25Q_SR1_BP1_BIT		3	// Block protect 1							(V/NV)
#define W25Q_SR1_BP2_BIT		4	// Block protect 2							(V/NV)
#define W25Q_SR1_BP3_BIT		5	// Block protect 3							(V/NV)
#define W25Q_SR1_TB_BIT			6	// Top/Bottom protect						(V/NV)
#define W25Q_SR1_SRP_BIT		7	// Status register protect 0				(V/NV)

// Status Register Bits	(SR2)
#define W25Q_SR2_SRL_BIT		8	// Status register protect 1				(V/NV)
#define W25Q_SR2_QE_BIT			9	// Quad SPI mode (1 - quad SPI enabled)		(V/NV)
// #define W25Q_SR2_RESERVED_BIT	10	// Reserved bit
#define W25Q_SR2_LB1_BIT		11	// Lock bit 1								(NV)
#define W25Q_SR2_LB2_BIT		12	// Lock bit 2								(NV)
#define W25Q_SR2_LB3_BIT		13	// Lock bit 3								(NV)
#define W25Q_SR2_CMP_BIT		14	// Complement protect						(V/NV)
#define W25Q_SR2_SUS_BIT		15	// Suspend Status							(Read only)

// Status Register Bits	(SR3)
#define W25Q_SR3_ADS_BIT		16	// Current addr mode (0-3 byte / 1-4 byte)	(Read only)
#define W25Q_SR3_ADP_BIT		17	// Power-up addr mode (0-3 byte / 1-4 byte)	(NV)
#define W25Q_SR3_WPS_BIT		18	// Write protect selection					(V/NV)
// #define W25Q_SR3_RESERVED1_BIT	19	// Reserved bit
// #define W25Q_SR3_RESERVED2_BIT	20	// Reserved bit
#define W25Q_SR3_DRV0_BIT		21	// Output driver strength 0 				(V/NV)
#define W25Q_SR3_DRV1_BIT		22	// Output driver strength 1 				(V/NV)
// #define W25Q_SR3_RESERVED3_BIT	23	// Reserved bit

// Extended Address Register Bits
#define W25Q_EAR_A24_BIT		0	// Address bit 24							(V)
#define W25Q_EAR_A25_BIT		1	// Address bit 25							(V)
#define W25Q_EAR_A26_BIT		2	// Address bit 26							(V)
#define W25Q_EAR_A27_BIT		3	// Address bit 27							(V)
#define W25Q_EAR_A28_BIT		4	// Address bit 28							(V)
#define W25Q_EAR_A29_BIT		5	// Address bit 29							(V)
#define W25Q_EAR_A30_BIT		6	// Address bit 30							(V)
#define W25Q_EAR_A31_BIT		7	// Address bit 31							(V)


/**
 * @enum W25Q_STATE
 * @brief W25Q Return State
 * Lib's functions status returns
 * @{
 */
typedef enum {
	W25Q_OK          			= 0, // Chip OK - Execution fine
	W25Q_BUSY        			= 1, // Chip busy
	W25Q_PARAM_ERR   			= 2, // Function parameters error
	W25Q_CHIP_ERR    			= 3, // Chip error
	W25Q_SPI_ERR     			= 4, // SPI Bus err
	W25Q_CHIP_IGNORE 			= 5, // Chip ignore state
	W25Q_CHIP_OFF_ON_REQUEST	= 6, // Chip off on request
} W25Q_STATE;
/** @} */

// typedef bool W25Q_STATUS_REG_BITS[24];

/**
 * @struct W25Q_STATUS_REG
 * @brief  W25Q Status Registers
 * @TODO: Mem protected recognition
 *
 * Structure to check chip's status registers
 * @{
 */
typedef struct {
	bool BUSY;  // Erase/Write in progress
	bool WEL;	// Write enable latch (1 - write allowed)
	bool QE;	// Quad SPI mode
	bool SUS; 	// Suspend Status
	bool ADS; 	// Current addr mode (0-3 byte / 1-4 byte)
	bool ADP; 	// Power-up addr mode
	bool SLEEP; // Sleep Status
} W25Q_STATUS_REG;
/** @} */

typedef struct W25Q_Chip {
	SPI_HandleTypeDef		*hspi;				// SPI Handle
	GPIO_TypeDef 	        *csPinBank;			// Chip Select Pin Bank
	uint16_t				 csPin;				// Chip Select Pin
	uint32_t				 status_reg;		// Status register value

	uint8_t			   		 tx_buf[256];		// Transmit buffer
	uint8_t	   				 rx_buf[256];		// Receive buffer

	bool 				     ASYNC_busy;
	// Is mandatory to use in async functions that do the following actions:
	// - Page Program
	// - Quad Page Program
	// - Sector Erase
	// - Block Erase
	// - Chip Erase
	// - Write Status Register
    // - Erase/Program Security Register
} W25Q_Chip;

#define W25Q_STATUS_REG(chip, bit)		((chip)->status_reg & (1 << (bit)) ? 1 : 0)

// typedef enum ASYNC_W25Q_State {
// 	ASYNC_W25Q_WAIT_W25Q,
// 	ASYNC_W25Q_START,
// 	ASYNC_W25Q_WAIT,
// 	ASYNC_W25Q_END,
// } ASYNC_W25Q_State;

// typedef enum ASYNC_W25Q_WaitAndProceed_State {
// 	ASYNC_W25Q_WaitAndProceed_WAIT_W25Q,
// 	ASYNC_W25Q_WaitAndProceed_START,
// 	ASYNC_W25Q_WaitAndProceed_WAIT_READY,
// 	ASYNC_W25Q_WaitAndProceed_WAIT_WEL,
// 	ASYNC_W25Q_WaitAndProceed_TxRx,
// 	ASYNC_W25Q_WaitAndProceed_END,
// } ASYNC_W25Q_WaitAndProceed_State;


// /*
// 	Macro that call the ASYNC_SPI_TxRx_DMA_init function with the W25Q chip's parameters
// 	It requires:
// 		- SCHEUDLER struct named "scheduler"
// 		- TASK struct named "task"
// 		- ASYNC_W25Q_..._CONTEXT struct named "context" witch is the context of the task
	
// 	... to be defined in the current scope.

// 	Parameters:
// 		- (uint8_t*)txBuf: the buffer to transmit
// 		- (uint8_t*)rxBuf: the buffer to receive
// 		- (size_t)txLen: the length of the buffer to transmit
// 		- (size_t)rxLen: the length of the buffer to receive
// */
// #define ASYNC_SPI_TxRx_DMA_init_W25Q(txBuf, rxBuf, txLen, rxLen) \
// 	ASYNC_SPI_TxRx_DMA_init(task, \
// 							context->w25q_chip->hspi, \
// 							context->w25q_chip->csPinBank, context->w25q_chip->csPin, \
// 							txBuf, rxBuf, txLen, rxLen, self)

W25Q_STATE W25Q_Init(W25Q_Chip			*w25q_chip,
					 SPI_HandleTypeDef	*hspi_flag,
					 GPIO_TypeDef		*csPinBank,
					 uint16_t			 csPin,
					 uint32_t			 id);
W25Q_STATE W25Q_WaitForReady(W25Q_Chip *w25q_chip);
W25Q_STATE W25Q_ReadID(W25Q_Chip *w25q_chip, uint8_t *id);
W25Q_STATE W25Q_Reset(W25Q_Chip *w25q_chip);
W25Q_STATE W25Q_ReadStatusReg(W25Q_Chip *w25q_chip);
W25Q_STATE W25Q_WriteStatuReg1(W25Q_Chip *w25q_chip, uint8_t data);
W25Q_STATE W25Q_WriteStatuReg2(W25Q_Chip *w25q_chip, uint8_t data);
W25Q_STATE W25Q_WriteStatuReg3(W25Q_Chip *w25q_chip, uint8_t data);
W25Q_STATE W25Q_EraseSector(W25Q_Chip *w25q_chip, uint32_t addr);
W25Q_STATE W25Q_Erase32K(W25Q_Chip *w25q_chip, uint32_t addr);
W25Q_STATE W25Q_Erase64K(W25Q_Chip *w25q_chip, uint32_t addr);
W25Q_STATE W25Q_EraseAll(W25Q_Chip *w25q_chip);
W25Q_STATE W25Q_WriteEnable(W25Q_Chip *w25q_chip);
W25Q_STATE W25Q_WriteVolatileEnable(W25Q_Chip *w25q_chip);
W25Q_STATE W25Q_WriteDisable(W25Q_Chip *w25q_chip);
W25Q_STATE W25Q_TransmitReceive(W25Q_Chip *w25q_chip, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t tx_len, uint16_t rx_len);
W25Q_STATE W25Q_PageProgram(W25Q_Chip *w25q_chip, uint8_t *buffer, uint32_t addr, uint16_t buf_size);
W25Q_STATE W25Q_WriteData(W25Q_Chip *w25q_chip, uint8_t *buffer, uint32_t addr, uint32_t buf_size);
W25Q_STATE W25Q_ReadData(W25Q_Chip *w25q_chip, uint8_t *buffer, uint32_t addr, uint32_t buf_size);


#ifdef __cplusplus
}
#endif

#endif /* W25Q_QSPI_W25Q_MEM_H_ */
