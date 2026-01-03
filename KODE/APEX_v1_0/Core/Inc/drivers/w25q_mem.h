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
#include <stdint.h>
#include "stm32f4xx_hal.h"

#include "peripherals/spi.h"

#include "utils/scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

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
#define W25Q_V_FULL_DEVICE_ID	0x4020U // W25Q....IV/JV Device ID
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


/* --- Flags comportementaux des commandes --- */
#define W25Q_FLAG_VALID        (1u << 0)
#define W25Q_FLAG_BUSY		   (1u << 1)
#define W25Q_FLAG_WEL		   (1u << 2)
#define W25Q_FLAG_DEVICE_BUSY  (1u << 3)
#define W25Q_FLAG_WAIT_AFTER   (1u << 4)

/* --- Codes de retour --- */
typedef enum {
    W25Q_OK = 0,
    W25Q_CHIP_ERR,
    W25Q_SPI_ERR,
    W25Q_PARAM_ERR,
    W25Q_BUSY_TIMEOUT,
	W25Q_SEM_ERR,
} W25Q_STATE;

/* --- Structure principale du périphérique --- */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_bank;
    uint16_t cs_pin;
    uint32_t status_reg;
	StaticSemaphore_t sem;
	osSemaphoreId_t sem_id;
} W25Q_t;

/* --- Table d’attributs de commandes --- */
extern const uint8_t W25Q_CMD_FLAGS[256];

#define W25Q_STATUS_REG(chip, bit) ((chip)->status_reg & (1 << (bit)) ? 1 : 0)
#define W25Q_FLASH_SIZE_BYTES (1 << 26) // 512 MBits = 64 MBytes





/* ============================== Sequential ============================== */

/* Niveau 1 : Primitives */
W25Q_STATE W25Q_SendCmd(W25Q_t *chip, uint8_t cmd);
W25Q_STATE W25Q_SendCmdAddr(W25Q_t *chip, uint8_t cmd, uint32_t addr);
W25Q_STATE W25Q_ReadStatus(W25Q_t *chip, uint8_t sr_index);
W25Q_STATE W25Q_WriteStatus(W25Q_t *chip, uint8_t sr_index, uint8_t value);
W25Q_STATE W25Q_ReadID(W25Q_t *chip, uint8_t *id);

/* Niveau 2 : Logique périphérique */
W25Q_STATE W25Q_Init(W25Q_t *chip, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_bank, uint16_t cs_pin);
W25Q_STATE W25Q_WriteData(W25Q_t *chip, const uint8_t *data, uint32_t addr, uint32_t data_size);
W25Q_STATE W25Q_ReadData(W25Q_t *chip, uint8_t *data, uint32_t addr, uint32_t data_size);




/* ============================== FreeRTOS ============================== */

/* Niveau 1 : Primitives */
W25Q_STATE W25Q_SendCmd_RTOS_base(W25Q_t *chip, uint8_t cmd, bool lock_sem);
W25Q_STATE W25Q_SendCmdAddr_RTOS_base(W25Q_t *chip, uint8_t cmd, uint32_t addr, bool lock_sem);
W25Q_STATE W25Q_ReadStatus_RTOS_base(W25Q_t *chip, uint8_t sr_index, bool lock_sem);
W25Q_STATE W25Q_WriteStatus_RTOS_base(W25Q_t *chip, uint8_t sr_index, uint8_t value, bool lock_sem);
W25Q_STATE W25Q_ReadID_RTOS_base(W25Q_t *chip, uint8_t *id, bool lock_sem);

#define W25Q_WaitForReady_RTOS_NoLock(chip)					W25Q_WaitForReady_RTOS_base(chip, false);

#define W25Q_SendCmd_RTOS_NoLock(chip, cmd)					W25Q_SendCmd_RTOS_base(chip, cmd, false);
#define W25Q_SendCmdAddr_RTOS_NoLock(chip, cmd, addr)		W25Q_SendCmdAddr_RTOS_base(chip, cmd, addr, false);
#define W25Q_ReadStatus_RTOS_NoLock(chip, sr_index)			W25Q_ReadStatus_RTOS_base(chip, sr_index, false);
#define W25Q_WriteStatus_RTOS_NoLock(chip, sr_index, value)	W25Q_WriteStatus_RTOS_base(chip, sr_index, value, false);
#define W25Q_ReadID_RTOS_NoLock(wchip, id)					W25Q_ReadID_RTOS_base(wchip, id, false);

#define W25Q_WaitForReady_RTOS(chip)					W25Q_WaitForReady_RTOS_base(chip, true);
#define W25Q_SendCmd_RTOS(chip, cmd)					W25Q_SendCmd_RTOS_base(chip, cmd, true);
#define W25Q_SendCmdAddr_RTOS(chip, cmd, addr)			W25Q_SendCmdAddr_RTOS_base(chip, cmd, addr, true);
#define W25Q_ReadStatus_RTOS(chip, sr_index)			W25Q_ReadStatus_RTOS_base(chip, sr_index, true);
#define W25Q_WriteStatus_RTOS(chip, sr_index, value)	W25Q_WriteStatus_RTOS_base(chip, sr_index, value, true);
#define W25Q_ReadID_RTOS(wchip, id)						W25Q_ReadID_RTOS_base(wchip, id, true);

/* Niveau 1 : Primitives en mode TASK (version lock par défaut) */
typedef struct TASK_W25Q_SendCmd_ARGS {
	W25Q_t *chip;
	uint8_t cmd;
	W25Q_STATE *result;
	osEventFlagsId_t done_flags;
} TASK_W25Q_SendCmd_ARGS;
TASK_POOL_CONFIGURE(TASK_W25Q_SendCmd, 5, 384);
void TASK_W25Q_SendCmd(void *argument);

typedef struct TASK_W25Q_SendCmdAddr_ARGS {
	W25Q_t *chip;
	uint8_t cmd;
	uint32_t addr;
	W25Q_STATE *result;
	osEventFlagsId_t done_flags;
} TASK_W25Q_SendCmdAddr_ARGS;
TASK_POOL_CONFIGURE(TASK_W25Q_SendCmdAddr, 5, 384);
void TASK_W25Q_SendCmdAddr(void *argument);

/* Niveau 2 : Logique périphérique */
typedef struct TASK_W25Q_Init_ARGS {
	W25Q_t *chip;
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_bank;
	uint16_t cs_pin;
	W25Q_STATE *result;
	osEventFlagsId_t done_flags;
} TASK_W25Q_Init_ARGS;
TASK_POOL_CONFIGURE(TASK_W25Q_Init, 1, 512);
void TASK_W25Q_Init(void *argument);

typedef struct TASK_W25Q_WriteData_ARGS {
	W25Q_t *chip;
	uint8_t *buffer;
	uint32_t addr;
	uint32_t buf_size;
	W25Q_STATE *result;
	osEventFlagsId_t done_flags;
} TASK_W25Q_WriteData_ARGS;
TASK_POOL_CONFIGURE(TASK_W25Q_WriteData, 5, 512);
void TASK_W25Q_WriteData(void *argument);

typedef struct TASK_W25Q_ReadData_ARGS {
	W25Q_t *chip;
	uint8_t *buffer;
	uint32_t addr;
	uint32_t buf_size;
	W25Q_STATE *result;
	osEventFlagsId_t done_flags;
} TASK_W25Q_ReadData_ARGS;
TASK_POOL_CONFIGURE(TASK_W25Q_ReadData, 5, 384);
void TASK_W25Q_ReadData(void *argument);






// ============================================== Fonction de test ==============================================

void W25Q_ReadWriteTest(W25Q_t *W25Q_t);




// A enlevé plus tard pour gagner de la place
typedef struct TASK_W25Q_ReadWriteTest_ARGS {
	W25Q_t *chip;
} TASK_W25Q_ReadWriteTest_ARGS;
TASK_POOL_CONFIGURE(TASK_W25Q_ReadWriteTest, 1, 8192);
void TASK_W25Q_ReadWriteTest(void *argument);




#ifdef __cplusplus
}
#endif

#endif /* W25Q_QSPI_W25Q_MEM_H_ */
