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
#define W25Q_MEM_FLASH_SIZE 516U 	// 516 MB-bit
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

/**@}*/

/**
 * @enum W25Q_STATE
 * @brief W25Q Return State
 * Lib's functions status returns
 * @{
 */
typedef enum {
	W25Q_OK          = 0, // Chip OK - Execution fine
	W25Q_BUSY        = 1, // Chip busy
	W25Q_PARAM_ERR   = 2, // Function parameters error
	W25Q_CHIP_ERR    = 3, // Chip error
	W25Q_SPI_ERR     = 4, // SPI Bus err
	W25Q_CHIP_IGNORE = 5, // Chip ignore state
} W25Q_STATE;
/** @} */

typedef enum {
	W25Q_BUSY_BIT =  0, // Erase/Write in progress
	W25Q_WEL_BIT  =  1, // Write enable latch (1 - write allowed)
	W25Q_BP0_BIT  =  2, // Block protect 0
	W25Q_BP1_BIT  =  3, // Block protect 1
	W25Q_BP2_BIT  =  4, // Block protect 2
	W25Q_TB_BIT   =  5, // Top/Bottom protect
	W25Q_SEC_BIT  =  6, // Sector protect
	W25Q_SRP_BIT  =  7, // Status register protect 0
	W25Q_SRL_BIT  =  8, // Status register protect 1
	W25Q_QE_BIT   =  9, // Quad SPI mode
	// Reserved bit
	W25Q_LB1_BIT  = 11, // Lock bit 1
	W25Q_LB2_BIT  = 12, // Lock bit 2
	W25Q_LB3_BIT  = 13, // Lock bit 3
	W25Q_CMP_BIT  = 14, // Complement protect
	W25Q_SUS_BIT  = 15, // Suspend Status
	// Reserved bit
	// Reserved bit
	W25Q_WPS_BIT  = 18, // Write protect selection
	// Reserved bit
	// Reserved bit
	W25Q_DRV2_BIT = 21, // Output driver strength 2
	W25Q_DRV1_BIT = 22, // Output driver strength 1
	// Reserved bit
} W25Q_STATUS_REG_BITS;

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
	bool                  	 status_bits[24];	// Status Register
	W25Q_STATE	  	      	 statue;  			// Statue

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


typedef enum ASYNC_W25Q_State {
	ASYNC_W25Q_WAIT_W25Q,
	ASYNC_W25Q_START,
	ASYNC_W25Q_WAIT,
	ASYNC_W25Q_END,
} ASYNC_W25Q_State;

typedef enum ASYNC_W25Q_WaitAndProceed_State {
	ASYNC_W25Q_WaitAndProceed_WAIT_W25Q,
	ASYNC_W25Q_WaitAndProceed_START,
	ASYNC_W25Q_WaitAndProceed_WAIT_READY,
	ASYNC_W25Q_WaitAndProceed_WAIT_WEL,
	ASYNC_W25Q_WaitAndProceed_TxRx,
	ASYNC_W25Q_WaitAndProceed_END,
} ASYNC_W25Q_WaitAndProceed_State;


/*
	Macro that call the ASYNC_SPI_TxRx_DMA_init function with the W25Q chip's parameters
	It requires:
		- SCHEUDLER struct named "scheduler"
		- TASK struct named "task"
		- ASYNC_W25Q_..._CONTEXT struct named "context" witch is the context of the task
	
	... to be defined in the current scope.

	Parameters:
		- (uint8_t*)txBuf: the buffer to transmit
		- (uint8_t*)rxBuf: the buffer to receive
		- (size_t)txLen: the length of the buffer to transmit
		- (size_t)rxLen: the length of the buffer to receive
*/
#define ASYNC_SPI_TxRx_DMA_init_W25Q(txBuf, rxBuf, txLen, rxLen) \
	ASYNC_SPI_TxRx_DMA_init(task, \
							context->w25q_chip->hspi, \
							context->w25q_chip->csPinBank, context->w25q_chip->csPin, \
							txBuf, rxBuf, txLen, rxLen, self)

void W25Q_Init(W25Q_Chip			*w25q_chip,
			   SPI_HandleTypeDef	*hspi_flag,
			   GPIO_TypeDef			*csPinBank,
			   uint16_t              csPin,
			   uint32_t              id);
bool W25Q_IsReady(W25Q_Chip *w25q_chip);
void W25Q_WaitForReady(W25Q_Chip *w25q_chip);
void W25Q_ReadID(W25Q_Chip *w25q_chip, uint8_t *id);
void W25Q_ReadStatusReg(W25Q_Chip *w25q_chip);
void W25Q_WriteStatuReg1(W25Q_Chip *w25q_chip, uint8_t data);
void W25Q_WriteStatuReg2(W25Q_Chip *w25q_chip, uint8_t data);
void W25Q_WriteStatuReg3(W25Q_Chip *w25q_chip, uint8_t data);
void W25Q_EarseSector(W25Q_Chip *w25q_chip, uint32_t addr);
void W25Q_EarseAll(W25Q_Chip *w25q_chip);
void W25Q_WriteEnable(W25Q_Chip *w25q_chip);
void W25Q_WriteDisable(W25Q_Chip *w25q_chip);
void W25Q_TransmitReceive2(W25Q_Chip* w25q_chip, uint8_t* tx_buf, uint8_t* rx_buf, uint16_t tx_len, uint16_t rx_len);
void W25Q_TransmitReceive(W25Q_Chip *w25q_chip, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t tx_len, uint16_t rx_len);
void W25Q_PageProgram(W25Q_Chip *w25q_chip, uint8_t *data, uint32_t addr, uint16_t data_size);
void W25Q_WriteData(W25Q_Chip *w25q_chip, uint8_t *data_buf, uint32_t addr, uint32_t data_size);
void W25Q_ReadData(W25Q_Chip *w25q_chip, uint8_t *data_buf, uint32_t addr, uint32_t data_size);




// =======================================================================

#define ASYNC_W25Q_ReadStatusReg_NUMBER 10

typedef struct ASYNC_W25Q_ReadStatusReg_CONTEXT {
	W25Q_Chip *w25q_chip;
	ASYNC_W25Q_State state;

	bool dma_complete[3];
	bool *is_done;

	uint8_t tx_buf[3];
	uint8_t rx_buf[3];
} ASYNC_W25Q_ReadStatusReg_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_ReadStatusReg);

void ASYNC_W25Q_ReadStatusReg_init(TASK *self, W25Q_Chip *w25q_chip);
TASK_RETURN ASYNC_W25Q_ReadStatusReg(SCHEDULER *scheduler, TASK *self);

// =======================================================================

#define ASYNC_W25Q_WriteEnable_NUMBER 10

typedef struct ASYNC_W25Q_WriteEnable_CONTEXT {
	W25Q_Chip *w25q_chip;
} ASYNC_W25Q_WriteEnable_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_WriteEnable);

void ASYNC_W25Q_WriteEnable_init(TASK *self, W25Q_Chip *w25q_chip);
TASK_RETURN ASYNC_W25Q_WriteEnable(SCHEDULER *scheduler, TASK *self);

// =======================================================================

#define ASYNC_W25Q_WaitForReady_NUMBER 10

typedef struct ASYNC_W25Q_WaitForReady_CONTEXT {
	W25Q_Chip *w25q_chip;
	ASYNC_W25Q_State state;
	bool read_reg_status_done;
} ASYNC_W25Q_WaitForReady_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_WaitForReady);

void ASYNC_W25Q_WaitForReady_init(TASK *self, W25Q_Chip *w25q_chip);
TASK_RETURN ASYNC_W25Q_WaitForReady(SCHEDULER *scheduler, TASK *self);

// =======================================================================

#define ASYNC_W25Q_EraseSector_NUMBER 10

typedef struct ASYNC_W25Q_EraseSector_CONTEXT {
	W25Q_Chip *w25q_chip;
	uint32_t addr;
	bool is_ready;
	uint8_t tx_buf[5];
	ASYNC_W25Q_WaitAndProceed_State state;
} ASYNC_W25Q_EraseSector_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_EraseSector);

void ASYNC_W25Q_EraseSector_init(TASK *self, W25Q_Chip *w25q_chip, uint32_t addr);
TASK_RETURN ASYNC_W25Q_EraseSector(SCHEDULER *scheduler, TASK *self);

// =======================================================================

#define ASYNC_W25Q_EraseAll_NUMBER 10

typedef struct ASYNC_W25Q_EraseAll_CONTEXT {
	W25Q_Chip *w25q_chip;
	bool is_ready;
	uint8_t tx_buf[1];
	ASYNC_W25Q_WaitAndProceed_State state;
} ASYNC_W25Q_EraseAll_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_EraseAll);

void ASYNC_W25Q_EraseAll_init(TASK *self, W25Q_Chip *w25q_chip);
TASK_RETURN ASYNC_W25Q_EraseAll(SCHEDULER *scheduler, TASK *self);

// =======================================================================

#define ASYNC_W25Q_ReadData_Smol_NUMBER 10

typedef struct ASYNC_W25Q_ReadData_Smol_CONTEXT {
	W25Q_Chip	*w25q_chip;

	uint8_t		*data;
	uint16_t	 data_size;
	uint32_t 	 addr;

	uint8_t		 tx_buf[5];

	bool is_ready;

	ASYNC_W25Q_WaitAndProceed_State state;
} ASYNC_W25Q_ReadData_Smol_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_ReadData_Smol);

void ASYNC_W25Q_ReadData_Smol_init(TASK					*self,
								   W25Q_Chip			*w25q_chip,
								   uint8_t	            *data,
								   uint16_t 		     data_size,
								   uint32_t				 addr);
TASK_RETURN ASYNC_W25Q_ReadData_Smol(SCHEDULER *scheduler, TASK *self);

// =======================================================================

#define ASYNC_W25Q_ReadData_NUMBER 10

typedef struct ASYNC_W25Q_ReadData_CONTEXT {
	W25Q_Chip *w25q_chip;
	
	uint8_t *data;
	uint16_t data_size;
	uint32_t based_addr;
	size_t last_data_size;

	bool is_ready;

	ASYNC_W25Q_State state;
} ASYNC_W25Q_ReadData_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_ReadData);

void ASYNC_W25Q_ReadData_init(TASK				*self,
							  W25Q_Chip			*w25q_chip,
							  uint8_t	 		*data,
							  uint16_t 			 data_size,
							  uint32_t			 addr);
TASK_RETURN ASYNC_W25Q_ReadData(SCHEDULER *scheduler, TASK *self);


// =======================================================================

#define ASYNC_W25Q_PageProgram_NUMBER 10

typedef struct ASYNC_W25Q_PageProgram_CONTEXT {
	W25Q_Chip *w25q_chip;

	// GMBNC_FAT_PTR data_fat_ptr;
	uint32_t addr;
	uint16_t data_size; // 1-256 bytes
	bool is_ready;

	uint8_t *tx_buf; // 4 bytes addr + data

	ASYNC_W25Q_WaitAndProceed_State state;
} ASYNC_W25Q_PageProgram_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_PageProgram);

void ASYNC_W25Q_PageProgram_init(TASK				*self,
							     W25Q_Chip  		*w25q_chip,
								 uint8_t   			*data,
								 uint16_t 		 	 data_size,
							     uint32_t    		 addr);
TASK_RETURN ASYNC_W25Q_PageProgram(SCHEDULER *scheduler, TASK *self);

// =======================================================================

#define ASYNC_W25Q_WriteData_NUMBER 10

typedef struct ASYNC_W25Q_WriteData_CONTEXT {
	W25Q_Chip *w25q_chip;
	
	uint8_t *data;
	uint16_t data_size;
	uint32_t based_addr;
	size_t last_data_size;

	bool is_ready;

	ASYNC_W25Q_State state;
} ASYNC_W25Q_WriteData_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_WriteData);

void ASYNC_W25Q_WriteData_init(TASK					*self,
							   W25Q_Chip 			*w25q_chip,
							   uint8_t	 			*data,
							   size_t	 		 	 data_size,
							   uint32_t			 	 addr);
TASK_RETURN ASYNC_W25Q_WriteData(SCHEDULER *scheduler, TASK *self);

// =======================================================================

#define ASYNC_W25Q_ScanIfSectorErased_NUMBER 10

typedef struct ASYNC_W25Q_ScanIfSectorErased_CONTEXT {
	W25Q_Chip *w25q_chip;

	uint32_t addr;
	bool is_ready;
	bool *is_erased;

	uint8_t data[W25Q_MEM_PAGE_SIZE * 16]; // 1 sector (16 pages of 256 bytes)
	
	ASYNC_W25Q_State state;
} ASYNC_W25Q_ScanIfSectorErased_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_ScanIfSectorErased);

void ASYNC_W25Q_ScanIfSectorErased_init(TASK *self, W25Q_Chip *w25q_chip, uint32_t addr, bool *is_erased);
TASK_RETURN ASYNC_W25Q_ScanIfSectorErased(SCHEDULER *scheduler, TASK *self);

// =======================================================================

#define ASYNC_W25Q_ScanIfBlockErased_NUMBER 10

typedef struct ASYNC_W25Q_ScanIfBlockErased_CONTEXT {
	W25Q_Chip *w25q_chip;

	uint32_t addr;
	bool is_ready;
	bool *is_erased;

	size_t i;

	bool *sectors; // 1 block (16 sectors)

	ASYNC_W25Q_State state;
} ASYNC_W25Q_ScanIfBlockErased_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_ScanIfBlockErased);

void ASYNC_W25Q_ScanIfBlockErased_init(TASK *self, W25Q_Chip *w25q_chip, uint32_t addr, bool *sectors, bool *is_erased);
TASK_RETURN ASYNC_W25Q_ScanIfBlockErased(SCHEDULER *scheduler, TASK *self);

// =======================================================================

#define ASYNC_W25Q_ScanIfChipErased_NUMBER 10

typedef struct ASYNC_W25Q_ScanIfChipErased_CONTEXT {
	W25Q_Chip *w25q_chip;

	bool is_ready;
	bool *is_erased;

	bool sectors[16]; // 1 block (16 sectors)
	uint32_t addr;
	size_t i;
	bool *blocks; // 1 chip (1024 blocks)

	ASYNC_W25Q_State state;
} ASYNC_W25Q_ScanIfChipErased_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_ScanIfChipErased);

void ASYNC_W25Q_ScanIfChipErased_init(TASK *self, W25Q_Chip *w25q_chip, bool *blocks, bool *is_erased);
TASK_RETURN ASYNC_W25Q_ScanIfChipErased(SCHEDULER *scheduler, TASK *self);

// =======================================================================

#define ASYNC_W25Q_USB_ScanMemory_NUMBER 1

typedef struct ASYNC_W25Q_USB_ScanMemory_CONTEXT {
	W25Q_Chip *w25q_chip;

	bool is_ready;
	bool *is_erased;

	uint32_t addr;
	size_t i;

	uint8_t data[4096]; // sector of 16 pages of 256 bytes
	// Scan sector by sector

	ASYNC_W25Q_State state;
} ASYNC_W25Q_USB_ScanMemory_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_W25Q_USB_ScanMemory);

void ASYNC_W25Q_USB_ScanMemory_init(TASK *self, W25Q_Chip *w25q_chip);
TASK_RETURN ASYNC_W25Q_USB_ScanMemory(SCHEDULER *scheduler, TASK *self);


// W25Q_STATE W25Q_EnableVolatileSR(void);						 ///< Make Status Register Volatile
// W25Q_STATE W25Q_ReadStatusReg(uint8_t *reg_data, uint8_t reg_num); ///< Read status register to variable
// W25Q_STATE W25Q_WriteStatusReg(uint8_t reg_data, uint8_t reg_num);///< Write status register from variable
// W25Q_STATE W25Q_ReadStatusStruct(W25Q_STATUS_REG *status);	 ///< Read all status registers to struct
// W25Q_STATE W25Q_IsBusy(void);	///< Check chip's busy status

// W25Q_STATE W25Q_ReadSByte(int8_t *buf, uint8_t pageShift, uint32_t pageNum);			///< Read signed 8-bit variable
// W25Q_STATE W25Q_ReadByte(uint8_t *buf, uint8_t pageShift, uint32_t pageNum);			 	///< Read 8-bit variable
// W25Q_STATE W25Q_ReadSWord(int16_t *buf, uint8_t pageShift, uint32_t pageNum);			///< Read signed 16-bit variable
// W25Q_STATE W25Q_ReadWord(uint16_t *buf, uint8_t pageShift, uint32_t pageNum);			///< Read 16-bit variable
// W25Q_STATE W25Q_ReadSLong(int32_t *buf, uint8_t pageShift, uint32_t pageNum);			///< Read signed 32-bit variable
// W25Q_STATE W25Q_ReadLong(uint32_t *buf, uint8_t pageShift, uint32_t pageNum);			///< Read 32-bit variable
// W25Q_STATE W25Q_ReadData(uint8_t *buf, uint16_t len, uint8_t pageShift, uint32_t pageNum);  ///< Read any 8-bit data
// W25Q_STATE W25Q_ReadRaw(uint8_t *buf, uint16_t data_len, uint32_t rawAddr);				///< Read data from raw addr
// W25Q_STATE W25Q_SingleRead(uint8_t *buf, uint32_t len, uint32_t Addr);					///< Read data from raw addr by single line

// W25Q_STATE W25Q_EraseSector(uint32_t SectAddr);			///< Erase 4KB Sector
// W25Q_STATE W25Q_EraseBlock(uint32_t BlockAddr, uint8_t size); ///< Erase 32KB/64KB Sector
// W25Q_STATE W25Q_EraseChip(void);						///< Erase all chip

// W25Q_STATE W25Q_ProgramSByte(int8_t buf, uint8_t pageShift, uint32_t pageNum);			 ///< Program signed 8-bit variable
// W25Q_STATE W25Q_ProgramByte(uint8_t buf, uint8_t pageShift, uint32_t pageNum);			 ///< Program 8-bit variable
// W25Q_STATE W25Q_ProgramSWord(int16_t buf, uint8_t pageShift, uint32_t pageNum);			 ///< Program signed 16-bit variable
// W25Q_STATE W25Q_ProgramWord(uint16_t buf, uint8_t pageShift, uint32_t pageNum);			 ///< Program 16-bit variable
// W25Q_STATE W25Q_ProgramSLong(int32_t buf, uint8_t pageShift, uint32_t pageNum);			 ///< Program signed 32-bit variable
// W25Q_STATE W25Q_ProgramLong(uint32_t buf, uint8_t pageShift, uint32_t pageNum);			 ///< Program 32-bit variable
// W25Q_STATE W25Q_ProgramData(uint8_t *buf, uint16_t len, uint8_t pageShift, uint32_t pageNum); ///< Program any 8-bit data
// W25Q_STATE W25Q_ProgramRaw(uint8_t *buf, uint16_t data_len, uint32_t rawAddr); 					 ///< Program data to raw addr

// W25Q_STATE W25Q_SetBurstWrap(uint8_t WrapSize);		///< Set Burst with Wrap

// W25Q_STATE W25Q_ProgSuspend(void);	///< Pause Programm/Erase operation
// W25Q_STATE W25Q_ProgResume(void);	///< Resume Programm/Erase operation

// W25Q_STATE W25Q_Sleep(void);	///< Set low current consumption
// W25Q_STATE W25Q_WakeUP(void);	///< Wake the chip up from sleep mode

// W25Q_STATE W25Q_ReadID(uint8_t *buf);				///< Read chip ID
// W25Q_STATE W25Q_ReadFullID(uint8_t *buf);			///< Read full chip ID (Manufacturer ID + Device ID)
// W25Q_STATE W25Q_ReadUID(uint8_t *buf);				///< Read unique chip ID
// W25Q_STATE W25Q_ReadJEDECID(uint8_t *buf); 		///< Read ID by JEDEC Standards
// W25Q_STATE W25Q_ReadSFDPRegister(uint8_t *buf); 	///< Read device descriptor (SFDP Standard)

// W25Q_STATE W25Q_EraseSecurityRegisters(uint8_t numReg);							///< Erase security register
// W25Q_STATE W25Q_ProgSecurityRegisters(uint8_t *buf, uint8_t numReg, uint8_t byteAddr);	///< Program security register
// W25Q_STATE W25Q_ReadSecurityRegisters(uint8_t *buf, uint8_t numReg, uint8_t byteAddr);	///< Read security register

// W25Q_STATE W25Q_BlockReadOnly(uint32_t Addr, bool enable);			///< Individual block/sector read-only lock
// W25Q_STATE W25Q_BlockReadOnlyCheck(bool *state, uint32_t Addr);	///< Check block's/sector's read-only lock status
// W25Q_STATE W25Q_GlobalReadOnly(bool enable);		///< Set read-only param to all chip

// W25Q_STATE W25Q_SwReset(bool force);	///< Software reset


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
#define W25Q_ENABLE_RST					0x66U	// enable software-reset ability
#define W25Q_RESET						0x99U	// make software reset


#ifdef __cplusplus
}
#endif

#endif /* W25Q_QSPI_W25Q_MEM_H_ */
