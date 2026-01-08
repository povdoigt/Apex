/**
 *******************************************
 * @file    w25q_mem.c
 * @author  Dmitriy Semenov / Crazy_Geeks
 * @version 0.1b
 * @date    12-August-2021
 * @brief   Source file for W25Qxxx lib
 * @note    https://github.com/Crazy-Geeks/STM32-W25Q-QSPI
 *******************************************
 *
 * @note https://ru.mouser.com/datasheet/2/949/w25q256jv_spi_revg_08032017-1489574.pdf
 * @note https://www.st.com/resource/en/application_note/DM00227538-.pdf
 */

 /**
  * @addtogroup W25Q_Driver
  * @{
  */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os2.h"

#include "drivers/w25q_mem.h"
#include "peripherals/spi.h"
#include "utils/scheduler.h"



const uint8_t W25Q_CMD_FLAGS[256] = {

    /* ----- Status / ID / Register access --------------------------------- */
    // [W25Q_READ_SR1]              	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,		// commented due to not used
    // [W25Q_READ_SR2]              	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_READ_SR3]              	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_WRITE_SR1]             	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_WAIT_AFTER | W25Q_FLAG_DEVICE_BUSY,
    // [W25Q_WRITE_SR2]             	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_WAIT_AFTER | W25Q_FLAG_DEVICE_BUSY,
    // [W25Q_WRITE_SR3]             	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_WAIT_AFTER | W25Q_FLAG_DEVICE_BUSY,
    // [W25Q_READ_JEDEC_ID]         	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,		// commented due to not used
    // [W25Q_READ_UID]              	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_READ_SFDP]             	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    [W25Q_READ_SECURITY_REG]     	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    [W25Q_PROG_SECURITY_REG]     	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY | W25Q_FLAG_WAIT_AFTER,
    [W25Q_ERASE_SECURITY_REG]    	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY | W25Q_FLAG_WAIT_AFTER,

    /* ----- Read data commands -------------------------------------------- */
    // [W25Q_READ_DATA]             	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,		// commented due to not used
    // [W25Q_READ_DATA_4B]          	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ]             	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_4B]          	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_DUAL_OUT]    	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_DUAL_IO]     	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_QUAD_OUT]    	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_QUAD_IO]     	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_DUAL_OUT_4B] 	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_DUAL_IO_4B]  	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_QUAD_OUT_4B] 	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_QUAD_IO_4B]  	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,

    /* ----- Program / Erase ----------------------------------------------- */
    // [W25Q_PAGE_PROGRAM]          	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,		// commented due to not used
    // [W25Q_PAGE_PROGRAM_4B]       	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    // [W25Q_PAGE_PROGRAM_QUAD_INP] 	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    // [W25Q_PAGE_PROGRAM_QUAD_INP_4B]	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,

    [W25Q_SECTOR_ERASE]          	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    [W25Q_SECTOR_ERASE_4B]       	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    [W25Q_32KB_BLOCK_ERASE]      	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    [W25Q_64KB_BLOCK_ERASE]      	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    [W25Q_64KB_BLOCK_ERASE_4B]   	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    [W25Q_CHIP_ERASE]            	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,

    /* ----- Write enable / disable & protection --------------------------- */
    [W25Q_WRITE_ENABLE]          	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    [W25Q_WRITE_DISABLE]         	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    [W25Q_ENABLE_VOLATILE_SR]    	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    [W25Q_READ_BLOCK_LOCK]       	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,

    /* ----- Suspend / Resume ---------------------------------------------- */
    [W25Q_ERASEPROG_SUSPEND]     	= W25Q_FLAG_VALID,
    [W25Q_ERASEPROG_RESUME]      	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,

    /* ----- Address mode / reset / power ---------------------------------- */
    [W25Q_ENABLE_4B_MODE]        	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WAIT_AFTER,
    [W25Q_DISABLE_4B_MODE]       	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WAIT_AFTER,
    [W25Q_ENABLE_RESET]          	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    [W25Q_RESET]                 	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WAIT_AFTER,
    [W25Q_POWERDOWN]             	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WAIT_AFTER,
    [W25Q_POWERUP]               	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WAIT_AFTER,
};

static inline bool W25Q_IsCmdValid(uint8_t cmd)				{ return (W25Q_CMD_FLAGS[cmd] & W25Q_FLAG_VALID      ); }
static inline bool W25Q_IsCmdRequiresBusyCheck(uint8_t cmd)	{ return (W25Q_CMD_FLAGS[cmd] & W25Q_FLAG_BUSY       ); }
static inline bool W25Q_IsCmdRequiresWEL(uint8_t cmd)		{ return (W25Q_CMD_FLAGS[cmd] & W25Q_FLAG_WEL        ); }
static inline bool W25Q_IsCmdSetsDeviceBusy(uint8_t cmd)	{ return (W25Q_CMD_FLAGS[cmd] & W25Q_FLAG_DEVICE_BUSY); }
static inline bool W25Q_IsCmdNeedWaitAfter(uint8_t cmd)		{ return (W25Q_CMD_FLAGS[cmd] & W25Q_FLAG_WAIT_AFTER ); }



/* -------------------------------------------------------------------------- */
/*                          Niveau 0 : SPI transaction                        */
/* -------------------------------------------------------------------------- */


static inline void W25Q_SPI_Begin(W25Q_t *chip) {
	HAL_GPIO_WritePin(chip->cs_bank, chip->cs_pin, GPIO_PIN_RESET);
}
static inline W25Q_STATE W25Q_SPI_Tx(W25Q_t *chip, const uint8_t *tx_buf, uint16_t tx_len) {
	return HAL_SPI_Transmit(chip->hspi, tx_buf, tx_len, HAL_MAX_DELAY) == HAL_OK ? W25Q_OK : W25Q_SPI_ERR;
}
static inline W25Q_STATE W25Q_SPI_Rx(W25Q_t *chip, uint8_t *rx_buf, uint16_t rx_len) {
	return HAL_SPI_Receive(chip->hspi, rx_buf, rx_len, HAL_MAX_DELAY) == HAL_OK ? W25Q_OK : W25Q_SPI_ERR;
}
static inline void W25Q_SPI_End(W25Q_t *chip) {
	HAL_GPIO_WritePin(chip->cs_bank, chip->cs_pin, GPIO_PIN_SET);
}





/* -------------------------------------------------------------------------- */
/*                          Niveau 1 : Command primitives                     */
/* -------------------------------------------------------------------------- */

static inline W25Q_STATE W25Q_WaitForReady(W25Q_t *chip) {
	W25Q_STATE st;
	do {
		st = W25Q_ReadStatus(chip, 1);
		if (st != W25Q_OK) return st;
	} while (W25Q_STATUS_REG(chip, W25Q_SR1_BUSY_BIT));
	return W25Q_OK;
}

/**
 * @brief Envoie une commande simple (sans adresse) au composant W25Q.
 *
 * Cette fonction gère automatiquement :
 *   - La vérification de validité de la commande à partir de la table W25Q_CMD_FLAGS.
 *   - L’attente de fin d’opération précédente (BUSY=0) si nécessaire.
 *   - L’activation de la possibilité d’écriture (WRITE ENABLE) si la commande le requiert.
 *   - L’attente de fin d’opération interne si la commande rend le composant occupé.
 *
 * Principe :
 *   - Certaines commandes (ex: ERASE, PROGRAM, WRITE_SR) requièrent que le périphérique
 *     soit prêt (BUSY=0) avant leur exécution, et mettent le périphérique en état occupé
 *     après leur envoi (BUSY=1). Ces contraintes sont indiquées par les flags de la table.
 *   - Le driver gère ces conditions automatiquement : aucune logique externe n’est nécessaire.
 *
 * Contraintes :
 *   - La commande doit exister dans la table W25Q_CMD_FLAGS, sinon la fonction retourne une erreur.
 *   - Si la commande requiert un Write Enable (WEL=1), celui-ci est activé automatiquement.
 *   - Cette fonction ne prend pas d’adresse : pour les commandes nécessitant un argument d’adresse
 *     (ex: ERASE 4KB, PROGRAM 4B, etc.), utiliser W25Q_SendCmdAddr().
 *
 * Exemples :
 *   - W25Q_SendCmd(chip, W25Q_WRITE_ENABLE);
 *   - W25Q_SendCmd(chip, W25Q_CHIP_ERASE);
 *   - W25Q_SendCmd(chip, W25Q_ENABLE_4B_MODE);
 *
 * @param chip  Pointeur vers la structure W25Q.
 * @param cmd   Code de la commande SPI à envoyer (ex: 0x06 pour WRITE_ENABLE).
 * @return      W25Q_OK si succès, ou un code d’erreur (W25Q_SPI_ERR, W25Q_PARAM_ERR, etc.).
 */
W25Q_STATE W25Q_SendCmd(W25Q_t *chip, uint8_t cmd) {
    W25Q_STATE st;

    if (!W25Q_IsCmdValid(cmd)) {
		return W25Q_PARAM_ERR;
	}

    if (W25Q_IsCmdRequiresBusyCheck(cmd)) {
		st = W25Q_WaitForReady(chip);
		if (st != W25Q_OK) return st;
	}

    if (W25Q_IsCmdRequiresWEL(cmd) && cmd != W25Q_WRITE_ENABLE) {
		// No need to check status again, already done above with WaitForReady
		// Even if W25Q_WRITE_ENABLE will return false with W25Q_IsCmdRequiresWEL(cmd),
		// we skip it here to avoid infinite recursion.
		if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
			st = W25Q_SendCmd(chip, W25Q_WRITE_ENABLE);
			if (st != W25Q_OK) return st;
		}
	}

	W25Q_SPI_Begin(chip);
	st = W25Q_SPI_Tx(chip, &cmd, 1);
    if (st != W25Q_OK) return st;
	W25Q_SPI_End(chip);

    if (W25Q_IsCmdNeedWaitAfter(cmd)) {
        st = W25Q_WaitForReady(chip);
		if (st != W25Q_OK) return st;
	}

    return st;
}

/**
 * @brief Envoie une commande accompagnée d’une adresse 32 bits.
 *
 * Cette fonction gère automatiquement :
 *   - L’attente de fin d’opération précédente (BUSY=0) si nécessaire.
 *   - L’envoi de la commande suivie de l’adresse (big endian : A31→A0).
 *   - L’activation automatique du Write Enable si la commande le requiert.
 *   - L’attente de fin d’opération interne si la commande met le périphérique occupé.
 *
 * Principe :
 *   - Les commandes de type lecture ou écriture adressée (READ_DATA_4B, ERASE_4K, PROGRAM_4B)
 *     nécessitent l’envoi d’un code de commande suivi d’une adresse 32 bits.
 *   - Le composant commence à répondre immédiatement après le dernier bit d’adresse,
 *     sans délai (bit n°40 du flux SPI).
 *   - Le contenu de MOSI après cette phase n’est pas échantillonné tant que CS reste LOW.
 *
 * Contraintes :
 *   - Le code commande doit être reconnu dans la table W25Q_CMD_FLAGS.
 *   - Le champ d’adresse est toujours transmis sur 4 octets (MSB→LSB).
 *   - Si la commande requiert un Write Enable (WEL=1), il est activé automatiquement.
 *   - Les flags DEVICE_BUSY et BUSY_REQ0 définissent les conditions de synchronisation.
 *
 * Exemples :
 *   - W25Q_SendCmdAddr(chip, W25Q_PAGE_PROGRAM_4B, 0x00123456);
 *   - W25Q_SendCmdAddr(chip, W25Q_SECTOR_ERASE_4B, 0x00080000);
 *   - W25Q_SendCmdAddr(chip, W25Q_READ_DATA_4B, 0x00000000);
 *
 * @param chip  Pointeur vers la structure W25Q.
 * @param cmd   Code de la commande SPI à envoyer (ex: 0x13 pour READ_DATA_4B).
 * @param addr  Adresse mémoire sur 32 bits (A31→A0).
 * @return      W25Q_OK si succès, ou un code d’erreur (W25Q_SPI_ERR, W25Q_PARAM_ERR, etc.).
 */
W25Q_STATE W25Q_SendCmdAddr(W25Q_t *chip, uint8_t cmd, uint32_t addr) {
    W25Q_STATE st;

    if (!W25Q_IsCmdValid(cmd)) {
		return W25Q_PARAM_ERR;
	}

    if (W25Q_IsCmdRequiresBusyCheck(cmd)) {
		st = W25Q_WaitForReady(chip);
		if (st != W25Q_OK) return st;
	}

    if (W25Q_IsCmdRequiresWEL(cmd)) {
		// No need to check status again, already done above with WaitForReady
		if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
			st =W25Q_SendCmd(chip, W25Q_WRITE_ENABLE);
			if (st != W25Q_OK) return st;
		}
	}

    uint8_t tx[5] = {
        cmd,
        (uint8_t)(addr >> 24),
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >>  8),
        (uint8_t)(addr >>  0)
    };

	W25Q_SPI_Begin(chip);
	st = W25Q_SPI_Tx(chip, tx, sizeof(tx));
	if (st != W25Q_OK) return st;
	W25Q_SPI_End(chip);
    if (st != W25Q_OK) return st;

    if (W25Q_IsCmdNeedWaitAfter(cmd)) {
        st = W25Q_WaitForReady(chip);
		if (st != W25Q_OK) return st;
	}

    return st;
}

W25Q_STATE W25Q_ReadStatus(W25Q_t *chip, uint8_t sr_index) {
	W25Q_STATE st;
	uint8_t cmd;
	uint8_t status;
	sr_index--;

	switch (sr_index) {
	case 0:
		cmd = W25Q_READ_SR1;
		break;
	case 1:
		cmd = W25Q_READ_SR2;
		break;
	case 2:
		cmd = W25Q_READ_SR3;
		break;
	default:
		return W25Q_PARAM_ERR;
	}

	// st = W25Q_SPI_TxRx(chip, &cmd, &status, 1, 1);
	W25Q_SPI_Begin(chip);
	st = W25Q_SPI_Tx(chip, &cmd, 1);
	if (st != W25Q_OK) { W25Q_SPI_End(chip); return st; }
	st = W25Q_SPI_Rx(chip, &status, 1);
	if (st != W25Q_OK) { W25Q_SPI_End(chip); return st; }
	W25Q_SPI_End(chip);

	chip->status_reg &= ~(  0xFF << (sr_index * 8));
	chip->status_reg |=  (status << (sr_index * 8));

	return W25Q_OK;
}

W25Q_STATE W25Q_WriteStatus(W25Q_t *chip, uint8_t sr_index, uint8_t value) {
	W25Q_STATE st;
	uint8_t tx_buf[2] = { 0 };
	sr_index--;

	switch (sr_index) {
	case 0:
		tx_buf[0] = W25Q_WRITE_SR1;
		break;
	case 1:
		tx_buf[0] = W25Q_WRITE_SR2;
		break;
	case 2:
		tx_buf[0] = W25Q_WRITE_SR3;
		break;
	default:
		return W25Q_PARAM_ERR;
	}

	tx_buf[1] = value;

	W25Q_SPI_Begin(chip);
	st = W25Q_SPI_Tx(chip, tx_buf, sizeof(tx_buf));
	W25Q_SPI_End(chip);
	if (st != W25Q_OK) return st;

	chip->status_reg &= ~( 0xFF << (sr_index * 8));
	chip->status_reg |=  (value << (sr_index * 8));

	return W25Q_OK;
}

W25Q_STATE W25Q_ReadID(W25Q_t *chip, uint8_t *id) {
	uint8_t cmd = W25Q_READ_JEDEC_ID;
	W25Q_STATE st;

	W25Q_SPI_Begin(chip);
	st = W25Q_SPI_Tx(chip, &cmd, 1);
	if (st != W25Q_OK) { W25Q_SPI_End(chip); return st; }
	st = W25Q_SPI_Rx(chip, id, 3);
	if (st != W25Q_OK) { W25Q_SPI_End(chip); return st; }
	W25Q_SPI_End(chip);

	return W25Q_OK;
}





/* -------------------------------------------------------------------------- */
/*                        Niveau 2 : Fonctions logiques                        */
/* -------------------------------------------------------------------------- */

W25Q_STATE W25Q_Init(W25Q_t *chip, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_bank, uint16_t cs_pin) {
	chip->hspi = hspi;
	chip->cs_bank = cs_bank;
	chip->cs_pin = cs_pin;

	// chip->sem_id = osSemaphoreNew(1, 1, &(osSemaphoreAttr_t) {
	// 	.name = "W25Q_SEM",
	// 	.cb_mem = &chip->sem,
	// 	.cb_size = sizeof(chip->sem)
	// });
	// if (chip->sem_id == NULL) return W25Q_SEM_ERR;

	W25Q_STATE st;

	// Read and check ID
	uint8_t id_buf[3];
	st = W25Q_ReadID(chip, id_buf);
	if (st != W25Q_OK) return st;
	if (id_buf[0] != W25Q_MANUFACTURER_ID) return W25Q_CHIP_ERR;
	if (W25Q_V_FULL_DEVICE_ID != (uint32_t)((id_buf[1] << 8) | id_buf[2])) return W25Q_PARAM_ERR;

	// Load configuration
	uint32_t status_reg = 0; // Initialize status register to 0
	status_reg |= (1 << W25Q_SR3_ADP_BIT); // Set addr mode to 4-byte

	st = W25Q_WriteStatus(chip, 1, (uint8_t)(status_reg >>  0));
	if (st != W25Q_OK) return st;
	st = W25Q_WriteStatus(chip, 2, (uint8_t)(status_reg >>  8));
	if (st != W25Q_OK) return st;
	st = W25Q_WriteStatus(chip, 3, (uint8_t)(status_reg >> 16));
	if (st != W25Q_OK) return st;

	// Verify configuration
	st = W25Q_ReadStatus(chip, 1);
	if (st != W25Q_OK) return st;
	st = W25Q_ReadStatus(chip, 2);
	if (st != W25Q_OK) return st;
	st = W25Q_ReadStatus(chip, 3);
	if (st != W25Q_OK) return st;

	if (!W25Q_STATUS_REG(chip, W25Q_SR3_ADS_BIT)) {
		// Try to enable 4-byte mode if not set already
		st = W25Q_SendCmd(chip, W25Q_ENABLE_4B_MODE);
	}

	return st;
}

// Data size must be <= 256 (page size)
static W25Q_STATE W25Q_PageProgram(W25Q_t *chip, const uint8_t *data, uint32_t addr, uint16_t data_size) {
	W25Q_STATE st;

	st = W25Q_WaitForReady(chip);
	if (st != W25Q_OK) return st;
	if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
		st = W25Q_SendCmd(chip, W25Q_WRITE_ENABLE);
		if (st != W25Q_OK) return st;
	}
	data_size = data_size > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE : data_size;
	uint8_t cmd[5] = {
		W25Q_PAGE_PROGRAM_4B,	// Command
		(uint8_t)(addr >> 24),	// Address
		(uint8_t)(addr >> 16),	// Address
		(uint8_t)(addr >>  8),	// Address
		(uint8_t)(addr >>  0)	// Address
	};

	W25Q_SPI_Begin(chip);
	st = W25Q_SPI_Tx(chip, cmd, sizeof(cmd));
	if (st != W25Q_OK) { W25Q_SPI_End(chip); return st; }
	st = W25Q_SPI_Tx(chip, data, data_size);
	if (st != W25Q_OK) { W25Q_SPI_End(chip); return st; }
	W25Q_SPI_End(chip);

	st = W25Q_WaitForReady(chip);
	return st;
}

W25Q_STATE W25Q_WriteData(W25Q_t *chip, const uint8_t *data, uint32_t addr, uint32_t data_size) {
	W25Q_STATE st;

	addr = addr > W25Q_FLASH_SIZE_BYTES ? W25Q_FLASH_SIZE_BYTES : addr;
	data_size = (uint64_t)(data_size + addr) > (uint64_t)W25Q_FLASH_SIZE_BYTES ? (W25Q_FLASH_SIZE_BYTES - addr) : data_size;

	while (data_size > 0) {
		uint32_t relative_addr = addr % W25Q_MEM_PAGE_SIZE;
		uint16_t data_size_page = (data_size + relative_addr) > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE - relative_addr : data_size;
		
		st = W25Q_PageProgram(chip, data, addr, data_size_page);
		if (st != W25Q_OK) return st;

		data_size -= data_size_page;
		addr += data_size_page;
		data += data_size_page;
	}
	return W25Q_OK;
}

// Data size must be <= flash size
W25Q_STATE W25Q_ReadData(W25Q_t *chip, uint8_t *data_buf, uint32_t addr, uint32_t data_size) {
	W25Q_STATE state;
	state =	W25Q_WaitForReady(chip);

	addr = addr > W25Q_FLASH_SIZE_BYTES ? W25Q_FLASH_SIZE_BYTES : addr;
	data_size = (uint64_t)(data_size + addr) > (uint64_t)W25Q_FLASH_SIZE_BYTES ? (W25Q_FLASH_SIZE_BYTES - addr) : data_size;

	uint8_t cmd[5] = {
		W25Q_READ_DATA_4B,		// Command
		(uint8_t)(addr >> 24),	// Address
		(uint8_t)(addr >> 16),	// Address
		(uint8_t)(addr >>  8),	// Address
		(uint8_t)(addr >>  0),	// Address
	};

	W25Q_SPI_Begin(chip);
	state = W25Q_SPI_Tx(chip, cmd, sizeof(cmd));
	if (state != W25Q_OK) { W25Q_SPI_End(chip); return state; }
	state = W25Q_SPI_Rx(chip, data_buf, data_size);
	if (state != W25Q_OK) { W25Q_SPI_End(chip); return state; }
	W25Q_SPI_End(chip);

	return state;
}




/* ============================== FreeRTOS ============================== */




/* -------------------------------------------------------------------------- */
/*                          Niveau 0 : SPI transaction                        */
/* -------------------------------------------------------------------------- */

static inline W25Q_STATE W25Q_SPI_Begin_RTOS(W25Q_t *chip) {
	return SPI_Begin_DMA_RTOS(chip->hspi, chip->cs_bank, chip->cs_pin) == HAL_OK ? W25Q_OK : W25Q_SEM_ERR; 
}
static inline W25Q_STATE W25Q_SPI_Tx_RTOS(W25Q_t *chip, uint8_t *tx_buf, uint16_t tx_len) {
	return SPI_Transmit_DMA_RTOS(chip->hspi, tx_buf, tx_len) == HAL_OK ? W25Q_OK : W25Q_SPI_ERR;
}
static inline W25Q_STATE W25Q_SPI_Rx_RTOS(W25Q_t *chip, uint8_t *rx_buf, uint16_t rx_len) {
	return SPI_Receive_DMA_RTOS(chip->hspi, rx_buf,  rx_len) == HAL_OK ? W25Q_OK : W25Q_SPI_ERR;
}
static inline W25Q_STATE W25Q_SPI_End_RTOS(W25Q_t *chip) {
	return SPI_End_DMA_RTOS(chip->hspi, chip->cs_bank, chip->cs_pin) == HAL_OK ? W25Q_OK : W25Q_SEM_ERR; 
}





/* -------------------------------------------------------------------------- */
/*                          Niveau 1 : Command primitives                     */
/* -------------------------------------------------------------------------- */

static inline W25Q_STATE W25Q_WaitForReady_RTOS_base(W25Q_t *chip, bool lock_sem) {
	W25Q_STATE st;
	do {
		st = W25Q_ReadStatus_RTOS_base(chip, 1, lock_sem);
		if (st != W25Q_OK) return st;
		osDelay(1);
	} while (W25Q_STATUS_REG(chip, W25Q_SR1_BUSY_BIT));
	return W25Q_OK;
}

W25Q_STATE W25Q_SendCmd_RTOS_base(W25Q_t *chip, uint8_t cmd, bool lock_sem) {
	W25Q_STATE st;
	bool need_release = false;

	if (!W25Q_IsCmdValid(cmd)) {
		return W25Q_PARAM_ERR;
	}

	if (W25Q_IsCmdSetsDeviceBusy(cmd)) {
		if (lock_sem) {
			if (osSemaphoreAcquire(chip->sem_id, osWaitForever) != osOK) {
				return W25Q_SEM_ERR;
			}
			need_release = true;
		}
	}

	if (W25Q_IsCmdRequiresBusyCheck(cmd)) {
		st = W25Q_WaitForReady_RTOS_NoLock(chip);
		if (st != W25Q_OK) { goto exit_release; }
	}

	if (W25Q_IsCmdRequiresWEL(cmd) && cmd != W25Q_WRITE_ENABLE) {
		// No need to check status again, already done above with WaitForReady
		// Even if W25Q_WRITE_ENABLE will return false with W25Q_IsCmdRequiresWEL(cmd),
		// we skip it here to avoid infinite recursion.
		if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
			st = W25Q_SendCmd_RTOS_NoLock(chip, W25Q_WRITE_ENABLE);
			if (st != W25Q_OK) { goto exit_release; }
		}
	}

	st = W25Q_SPI_Begin_RTOS(chip);
	if (st != W25Q_OK) { goto exit_release; }
	st = W25Q_SPI_Tx_RTOS(chip, &cmd, 1);
	if (st != W25Q_OK) { W25Q_SPI_End_RTOS(chip); goto exit_release; }
	st = W25Q_SPI_End_RTOS(chip);
	if (st != W25Q_OK) { goto exit_release; }

	if (W25Q_IsCmdNeedWaitAfter(cmd)) {
		st = W25Q_WaitForReady_RTOS_NoLock(chip);
		if (st != W25Q_OK) { goto exit_release; }
	}

exit_release:
	if (need_release) {
		if (osSemaphoreRelease(chip->sem_id) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	return st;
}

W25Q_STATE W25Q_SendCmdAddr_RTOS_base(W25Q_t *chip, uint8_t cmd, uint32_t addr, bool lock_sem) {
	W25Q_STATE st;
	bool need_release = false;

	if (!W25Q_IsCmdValid(cmd)) {
		return W25Q_PARAM_ERR;
	}

	if (W25Q_IsCmdSetsDeviceBusy(cmd)) {
		if (lock_sem) {
			if (osSemaphoreAcquire(chip->sem_id, osWaitForever) != osOK) {
				return W25Q_SEM_ERR;
			}
			need_release = true;
		}
	}

	if (W25Q_IsCmdRequiresBusyCheck(cmd)) {
		st = W25Q_WaitForReady_RTOS_NoLock(chip);
		if (st != W25Q_OK) { goto exit_release; }
	}

	if (W25Q_IsCmdRequiresWEL(cmd)) {
		// No need to check status again, already done above with WaitForReady
		if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
			st =W25Q_SendCmd_RTOS_NoLock(chip, W25Q_WRITE_ENABLE);
			if (st != W25Q_OK) { goto exit_release; }
		}
	}

	uint8_t tx[5] = {
		cmd,
		(uint8_t)(addr >> 24),
		(uint8_t)(addr >> 16),
		(uint8_t)(addr >>  8),
		(uint8_t)(addr >>  0)
	};

	st = W25Q_SPI_Begin_RTOS(chip);
	if (st != W25Q_OK) { goto exit_release; }
	st = W25Q_SPI_Tx_RTOS(chip, tx, sizeof(tx));
	if (st != W25Q_OK) { W25Q_SPI_End_RTOS(chip); goto exit_release; }
	st = W25Q_SPI_End_RTOS(chip);
	if (st != W25Q_OK) { goto exit_release; }

	if (W25Q_IsCmdNeedWaitAfter(cmd)) {
		st = W25Q_WaitForReady_RTOS_NoLock(chip);
		if (st != W25Q_OK) { goto exit_release; }
	}

exit_release:
	if (need_release) {
		if (osSemaphoreRelease(chip->sem_id) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	return st;
}

W25Q_STATE W25Q_ReadStatus_RTOS_base(W25Q_t *chip, uint8_t sr_index, bool lock_sem) {
	W25Q_STATE st = W25Q_OK;
	uint8_t cmd;
	uint8_t status;
	sr_index--;

	switch (sr_index) {
	case 0:
		cmd = W25Q_READ_SR1;
		break;
	case 1:
		cmd = W25Q_READ_SR2;
		break;
	case 2:
		cmd = W25Q_READ_SR3;
		break;
	default:
		return W25Q_PARAM_ERR;
	}

	if (lock_sem) {
		if (osSemaphoreAcquire(chip->sem_id, osWaitForever) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	st = W25Q_SPI_Begin_RTOS(chip);
	if (st != W25Q_OK) { goto exit_flag; }
	st = W25Q_SPI_Tx_RTOS(chip, &cmd, 1);
	if (st != W25Q_OK) { W25Q_SPI_End_RTOS(chip); goto exit_flag; }
	st = W25Q_SPI_Rx_RTOS(chip, &status, 1);
	if (st != W25Q_OK) { W25Q_SPI_End_RTOS(chip); goto exit_flag; }
	st = W25Q_SPI_End_RTOS(chip);
	if (st != W25Q_OK) { goto exit_flag; }

	chip->status_reg &= ~(  0xFF << (sr_index * 8));
	chip->status_reg |=  (status << (sr_index * 8));

exit_flag:
	if (lock_sem) {
		if (osSemaphoreRelease(chip->sem_id) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	return st;
}

W25Q_STATE W25Q_WriteStatus_RTOS_base(W25Q_t *chip, uint8_t sr_index, uint8_t value, bool lock_sem) {
	W25Q_STATE st = W25Q_OK;
	uint8_t tx_buf[2] = { 0 };
	sr_index--;

	switch (sr_index) {
	case 0:
		tx_buf[0] = W25Q_WRITE_SR1;
		break;
	case 1:
		tx_buf[0] = W25Q_WRITE_SR2;
		break;
	case 2:
		tx_buf[0] = W25Q_WRITE_SR3;
		break;
	default:
		return W25Q_PARAM_ERR;
	}

	tx_buf[1] = value;

	if (lock_sem) {
		if (osSemaphoreAcquire(chip->sem_id, osWaitForever) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	st = W25Q_SPI_Begin_RTOS(chip);
	if (st != W25Q_OK) { goto exit_flag; }
	st = W25Q_SPI_Tx_RTOS(chip, tx_buf, sizeof(tx_buf));
	if (st != W25Q_OK) { W25Q_SPI_End_RTOS(chip); goto exit_flag; }
	st = W25Q_SPI_End_RTOS(chip);
	if (st != W25Q_OK) { goto exit_flag; }

	chip->status_reg &= ~( 0xFF << (sr_index * 8));
	chip->status_reg |=  (value << (sr_index * 8));

exit_flag:
	if (lock_sem) {
		if (osSemaphoreRelease(chip->sem_id) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	return st;
}

W25Q_STATE W25Q_ReadID_RTOS_base(W25Q_t *chip, uint8_t *id, bool lock_sem) {
	W25Q_STATE st = W25Q_OK;
	uint8_t cmd = W25Q_READ_JEDEC_ID;

	if (lock_sem) {
		if (osSemaphoreAcquire(chip->sem_id, osWaitForever) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	st = W25Q_SPI_Begin_RTOS(chip);
	if (st != W25Q_OK) { goto exit_flag; }
	st = W25Q_SPI_Tx_RTOS(chip, &cmd, 1);
	if (st != W25Q_OK) { W25Q_SPI_End_RTOS(chip); goto exit_flag; }
	st = W25Q_SPI_Rx_RTOS(chip, id, 3);
	if (st != W25Q_OK) { W25Q_SPI_End_RTOS(chip); goto exit_flag; }
	st = W25Q_SPI_End_RTOS(chip);

exit_flag:
	if (lock_sem) {
		if (osSemaphoreRelease(chip->sem_id) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	return st;
}






/* -------------------------------------------------------------------------- */
/*                          Niveau 1 : Command primitives TASK                */
/* -------------------------------------------------------------------------- */

TASK_POOL_ALLOCATE(TASK_W25Q_SendCmd);
void TASK_W25Q_SendCmd(void *argument) {
	TASK_W25Q_SendCmd_ARGS *args = (TASK_W25Q_SendCmd_ARGS*)argument;
	*(args->result) = W25Q_SendCmd_RTOS(args->chip, args->cmd);
	if (args->done_flags) { osEventFlagsSet(args->done_flags, 1); }
	osThreadExit_Cstm();
}

TASK_POOL_ALLOCATE(TASK_W25Q_SendCmdAddr);
void TASK_W25Q_SendCmdAddr(void *argument) {
	TASK_W25Q_SendCmdAddr_ARGS *args = (TASK_W25Q_SendCmdAddr_ARGS*)argument;
	*(args->result) = W25Q_SendCmdAddr_RTOS(args->chip, args->cmd, args->addr);
	if (args->done_flags) { osEventFlagsSet(args->done_flags, 1); }
	osThreadExit_Cstm();
}





/* -------------------------------------------------------------------------- */
/*                        Niveau 2 : Fonctions logiques                        */
/* -------------------------------------------------------------------------- */

TASK_POOL_ALLOCATE(TASK_W25Q_Init);
void TASK_W25Q_Init(void *argument) {
	TASK_W25Q_Init_ARGS *args = (TASK_W25Q_Init_ARGS*)argument;

	args->chip->hspi = args->hspi;
	args->chip->cs_bank = args->cs_bank;
	args->chip->cs_pin = args->cs_pin;

	args->chip->sem_id = osSemaphoreNew(1, 1, &(osSemaphoreAttr_t) {
		.name = "W25Q_SEM",
		.cb_mem = &args->chip->sem,
		.cb_size = sizeof(args->chip->sem)
	});
	if (args->chip->sem_id == NULL) { *(args->result) = W25Q_SEM_ERR; goto exit_flag; }

	// Read and check ID
	uint8_t id_buf[3];
	*(args->result) = W25Q_ReadID_RTOS(args->chip, id_buf);
	if (*(args->result) != W25Q_OK) { goto exit_flag; }
	if (id_buf[0] != W25Q_MANUFACTURER_ID) { *(args->result) = W25Q_CHIP_ERR; goto exit_flag; }
	if (W25Q_V_FULL_DEVICE_ID != (uint32_t)((id_buf[1] << 8) | id_buf[2])) { *(args->result) = W25Q_PARAM_ERR; osThreadExit_Cstm();}

	// Load configuration
	uint32_t status_reg = 0; // Initialize status register to 0
	status_reg |= (1 << W25Q_SR3_ADP_BIT); // Set addr mode to 4-byte

	*(args->result) = W25Q_WriteStatus_RTOS(args->chip, 1, (uint8_t)(status_reg >>  0));
	if (*(args->result) != W25Q_OK) { goto exit_flag; }
	*(args->result) = W25Q_WriteStatus_RTOS(args->chip, 2, (uint8_t)(status_reg >>  8));
	if (*(args->result) != W25Q_OK) { goto exit_flag; }
	*(args->result) = W25Q_WriteStatus_RTOS(args->chip, 3, (uint8_t)(status_reg >> 16));
	if (*(args->result) != W25Q_OK) { goto exit_flag; }

	// Verify configuration
	*(args->result) = W25Q_ReadStatus_RTOS(args->chip, 1);
	if (*(args->result) != W25Q_OK) { goto exit_flag; }
	*(args->result) = W25Q_ReadStatus_RTOS(args->chip, 2);
	if (*(args->result) != W25Q_OK) { goto exit_flag; }
	*(args->result) = W25Q_ReadStatus_RTOS(args->chip, 3);
	if (*(args->result) != W25Q_OK) { goto exit_flag; }

	if (!W25Q_STATUS_REG(args->chip, W25Q_SR3_ADS_BIT)) {
		// Try to enable 4-byte mode if not set already
		*(args->result) = W25Q_SendCmd_RTOS(args->chip, W25Q_ENABLE_4B_MODE);
	}

exit_flag:
	if (args->done_flags) { osEventFlagsSet(args->done_flags, 1); }

	osThreadExit_Cstm();
}

static W25Q_STATE W25Q_PageProgram_RTOS(W25Q_t *chip, uint8_t *buffer, uint32_t addr, uint16_t buf_size) {
	W25Q_STATE st;
	if (osSemaphoreAcquire(chip->sem_id, osWaitForever) != osOK) {
		return W25Q_SEM_ERR;
	}
	st = W25Q_WaitForReady_RTOS_NoLock(chip);
	if (st != W25Q_OK) { goto exit_flag; }
	if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
		st = W25Q_SendCmd_RTOS_NoLock(chip, W25Q_WRITE_ENABLE);
		if (st != W25Q_OK) { goto exit_flag; }
	}
	buf_size = buf_size > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE : buf_size;
	uint8_t cmd[5] = {
		W25Q_PAGE_PROGRAM_4B,	// Command
		(uint8_t)(addr >> 24),	// Address
		(uint8_t)(addr >> 16),	// Address
		(uint8_t)(addr >>  8),	// Address
		(uint8_t)(addr >>  0)	// Address
	};

	st = W25Q_SPI_Begin_RTOS(chip);
	if (st != W25Q_OK) { goto exit_flag; }
	st = W25Q_SPI_Tx_RTOS(chip, cmd, sizeof(cmd));
	if (st != W25Q_OK) { W25Q_SPI_End_RTOS(chip); goto exit_flag; }
	st = W25Q_SPI_Tx_RTOS(chip, buffer, buf_size);
	if (st != W25Q_OK) { W25Q_SPI_End_RTOS(chip); goto exit_flag; }
	st = W25Q_SPI_End_RTOS(chip);
	if (st != W25Q_OK) { goto exit_flag; }

	st = W25Q_WaitForReady_RTOS_NoLock(chip);

exit_flag:
	if (osSemaphoreRelease(chip->sem_id) != osOK) {
		return W25Q_SEM_ERR;
	}

	return st;
}

TASK_POOL_ALLOCATE(TASK_W25Q_WriteData);
void TASK_W25Q_WriteData(void *argument) {
	TASK_W25Q_WriteData_ARGS *args = (TASK_W25Q_WriteData_ARGS*)argument;

	uint32_t flash_size = W25Q_MEM_FLASH_SIZE * 1000000 / 8; // MBites to bytes
	args->buf_size = (args->buf_size + args->addr) > flash_size ? flash_size - args->addr : args->buf_size;

	while (args->buf_size > 0) {
		uint32_t relative_addr = args->addr % W25Q_MEM_PAGE_SIZE;
		uint16_t data_size_page = (args->buf_size + relative_addr) > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE - relative_addr : args->buf_size;

		*(args->result) = W25Q_PageProgram_RTOS(args->chip, args->buffer, args->addr, data_size_page);
		if (*(args->result) != W25Q_OK) { goto exit_flag; }

		args->buf_size -= data_size_page;
		args->addr += data_size_page;
		args->buffer += data_size_page;
	}
	*(args->result) = W25Q_OK;

exit_flag:
	if (args->done_flags) osEventFlagsSet(args->done_flags, 1);

	osThreadExit_Cstm();
}

TASK_POOL_ALLOCATE(TASK_W25Q_ReadData);
void TASK_W25Q_ReadData(void *argument) {
	TASK_W25Q_ReadData_ARGS *args = (TASK_W25Q_ReadData_ARGS*)argument;

	uint32_t flash_size = W25Q_MEM_FLASH_SIZE * 1000000 / 8; // MBites to bytes

	if (osSemaphoreAcquire(args->chip->sem_id, osWaitForever) != osOK) {
		*(args->result) = W25Q_SEM_ERR;
		goto exit_flag;
	}

	*(args->result) = W25Q_WaitForReady_RTOS_NoLock(args->chip);
	if (*(args->result) != W25Q_OK) { goto exit_release; }
	if (!W25Q_STATUS_REG(args->chip, W25Q_SR1_WEL_BIT)) {
		*(args->result) = W25Q_SendCmd_RTOS_NoLock(args->chip, W25Q_WRITE_ENABLE);
		if (*(args->result) != W25Q_OK) { goto exit_release; }
	}

	args->buf_size = args->buf_size > flash_size ? flash_size : args->buf_size;
	uint8_t cmd[5] = {
		W25Q_READ_DATA_4B,				// Command
		(uint8_t)(args->addr >> 24),	// Address
		(uint8_t)(args->addr >> 16),	// Address
		(uint8_t)(args->addr >>  8),	// Address
		(uint8_t)(args->addr >>  0),	// Address
	};

	*(args->result) = W25Q_SPI_Begin_RTOS(args->chip);
	if (*(args->result) != W25Q_OK) { goto exit_release; }
	*(args->result) = W25Q_SPI_Tx_RTOS(args->chip, cmd, sizeof(cmd));
	if (*(args->result) != W25Q_OK) { W25Q_SPI_End_RTOS(args->chip); goto exit_release; }
	*(args->result) = W25Q_SPI_Rx_RTOS(args->chip, args->buffer, args->buf_size);
	if (*(args->result) != W25Q_OK) { W25Q_SPI_End_RTOS(args->chip); goto exit_release; }
	*(args->result) = W25Q_SPI_End_RTOS(args->chip);

exit_release:
	if (osSemaphoreRelease(args->chip->sem_id) != osOK) {
		*(args->result) = W25Q_SEM_ERR;
	}

exit_flag:
	if (args->done_flags) { osEventFlagsSet(args->done_flags, 1); }

	osThreadExit_Cstm();
}





/* -------------------------------------------------------------------------- */
/*                              Fonction de test                              */
/* -------------------------------------------------------------------------- */

void W25Q_ReadWriteTest(W25Q_t *chip) {
	W25Q_STATE state = W25Q_OK;

	uint8_t rx_data[4096 + 512 + 5] = { 0 };
	uint8_t tx_data_origine[538] = "Hello, W25Q256! This is a test of the W25Q256 flash memory chip. \
Let's see if it works properly. We will write this data to the flash memory and then read it back to verify\
the integrity of the data. If everything goes well, we should see the same data we wrote.\
This message is intentionally made longer to test the page programming and reading capabilities of the chip.\
We will also check if the data spans multiple pages and sectors to ensure that the implementation is robust.\
Thank you for your attention and happy coding!";

	uint8_t tx_data[538] = { 0 };
	memcpy(tx_data, tx_data_origine, 538);
	
	state = W25Q_ReadData(chip, rx_data, 0x00000000, 4096 + 512);
	assert_param(state == W25Q_OK);

	state = W25Q_SendCmdAddr(chip, W25Q_SECTOR_ERASE_4B, 0x00000000);
	assert_param(state == W25Q_OK);

	state = W25Q_SendCmdAddr(chip, W25Q_SECTOR_ERASE_4B, 0X00001000);
	assert_param(state == W25Q_OK);

	state = W25Q_ReadData(chip, rx_data, 0x00000000, 4096 + 512);
	assert_param(state == W25Q_OK);

	state = W25Q_WriteData(chip, tx_data, 0x00000f0f, 538);
	assert_param(state == W25Q_OK);

	state = W25Q_ReadData(chip, rx_data, 0x00000000, 4096 + 512);
	assert_param(state == W25Q_OK);

	assert_param(memcmp(rx_data + 0x00000f0f, tx_data_origine, 538) == 0);
}



TASK_POOL_ALLOCATE(TASK_W25Q_ReadWriteTest);
void TASK_W25Q_ReadWriteTest(void *argument) {
	TASK_W25Q_ReadWriteTest_ARGS *args = (TASK_W25Q_ReadWriteTest_ARGS*)argument;

	W25Q_STATE state[7] = { W25Q_OK };
	uint8_t rx_data[4096 + 512] = { 0 };
	uint8_t tx_data_origine[538] = "Hello, W25Q256! This is a test of the W25Q256 flash memory chip. \
Let's see if it works properly. We will write this data to the flash memory and then read it back to verify\
the integrity of the data. If everything goes well, we should see the same data we wrote.\
This message is intentionally made longer to test the page programming and reading capabilities of the chip.\
We will also check if the data spans multiple pages and sectors to ensure that the implementation is robust.\
Thank you for your attention and happy coding!";

	uint8_t tx_data[538] = { 0 };
	memcpy(tx_data, tx_data_origine, 538);

	
	osThreadAttr_t attr = { 0 };

	TASK_W25Q_ReadData_ARGS read_args = { .chip = args->chip };
	TASK_W25Q_SendCmdAddr_ARGS erase_args = { .chip = args->chip, .cmd = W25Q_SECTOR_ERASE_4B };
	TASK_W25Q_WriteData_ARGS write_args = { .chip = args->chip };

	StaticEventGroup_t done_flags_mem[7];
	osEventFlagsId_t done_flags[7];

	done_flags[0] = osEventFlagsNew(&((osEventFlagsAttr_t){
		.name = "W25Q_DoneFlags1",
		.cb_mem = &done_flags_mem[0],
		.cb_size = sizeof(done_flags_mem[0])
	}));
	done_flags[1] = osEventFlagsNew(&((osEventFlagsAttr_t){
		.name = "W25Q_DoneFlags2",
		.cb_mem = &done_flags_mem[1],
		.cb_size = sizeof(done_flags_mem[1])
	}));
	done_flags[2] = osEventFlagsNew(&((osEventFlagsAttr_t){
		.name = "W25Q_DoneFlags3",
		.cb_mem = &done_flags_mem[2],
		.cb_size = sizeof(done_flags_mem[2])
	}));
	done_flags[3] = osEventFlagsNew(&((osEventFlagsAttr_t){
		.name = "W25Q_DoneFlags4",
		.cb_mem = &done_flags_mem[3],
		.cb_size = sizeof(done_flags_mem[3])
	}));
	done_flags[4] = osEventFlagsNew(&((osEventFlagsAttr_t){
		.name = "W25Q_DoneFlags5",
		.cb_mem = &done_flags_mem[4],
		.cb_size = sizeof(done_flags_mem[4])
	}));
	done_flags[5] = osEventFlagsNew(&((osEventFlagsAttr_t){
		.name = "W25Q_DoneFlags6",
		.cb_mem = &done_flags_mem[5],
		.cb_size = sizeof(done_flags_mem[5])
	}));
	osEventFlagsClear(done_flags[0], 0xFFFFFFFF);
	osEventFlagsClear(done_flags[1], 0xFFFFFFFF);
	osEventFlagsClear(done_flags[2], 0xFFFFFFFF);
	osEventFlagsClear(done_flags[3], 0xFFFFFFFF);
	osEventFlagsClear(done_flags[4], 0xFFFFFFFF);
	osEventFlagsClear(done_flags[5], 0xFFFFFFFF);


	
	// Initial read
	read_args.addr = 0x00000000;
	read_args.buffer = rx_data;
	read_args.buf_size = 4096 + 512;
	read_args.result = &state[0];
	read_args.done_flags = done_flags[0];

	attr.name = "W25Q_Read1";
	OS_THREAD_NEW_CSTM(TASK_W25Q_ReadData, read_args, attr, osWaitForever);
	osEventFlagsWait(done_flags[0], 1, osFlagsWaitAll, osWaitForever);
	assert_param(state[0] == W25Q_OK);

	// Erase sectors 1 and 2
	erase_args.addr = 0x00000000;
	erase_args.result = &state[1];
	erase_args.done_flags = done_flags[1];

	attr.name = "W25Q_Erase1";
	OS_THREAD_NEW_CSTM(TASK_W25Q_SendCmdAddr, erase_args, attr, osWaitForever);

	erase_args.addr = 0x00001000;
	erase_args.result = &state[2];
	erase_args.done_flags = done_flags[2];

	attr.name = "W25Q_Erase2";
	OS_THREAD_NEW_CSTM(TASK_W25Q_SendCmdAddr, erase_args, attr, osWaitForever);

	osEventFlagsWait(done_flags[1], 1, osFlagsWaitAll, osWaitForever);
	osEventFlagsWait(done_flags[2], 1, osFlagsWaitAll, osWaitForever);
	assert_param(state[1] == W25Q_OK);
	assert_param(state[2] == W25Q_OK);

	// Read after erase
	read_args.result = &state[3]; // reuse previous args
	read_args.done_flags = done_flags[3];

	attr.name = "W25Q_Read2";

	OS_THREAD_NEW_CSTM(TASK_W25Q_ReadData, read_args, attr, osWaitForever);
	osEventFlagsWait(done_flags[3], 1, osFlagsWaitAll, osWaitForever);
	assert_param(state[3] == W25Q_OK);

	// Write data
	write_args.addr = 0x00000f0f;
	write_args.buffer = tx_data;
	write_args.buf_size = 538;
	write_args.result = &state[4];
	write_args.done_flags = done_flags[4];

	attr.name = "W25Q_Write";

	OS_THREAD_NEW_CSTM(TASK_W25Q_WriteData, write_args, attr, osWaitForever);
	osEventFlagsWait(done_flags[4], 1, osFlagsWaitAll, osWaitForever);
	assert_param(state[4] == W25Q_OK);

	// Final read
	read_args.result = &state[5]; // reuse previous args
	read_args.done_flags = done_flags[5];

	attr.name = "W25Q_Read3";

	OS_THREAD_NEW_CSTM(TASK_W25Q_ReadData, read_args, attr, osWaitForever);
	osEventFlagsWait(done_flags[5], 1, osFlagsWaitAll, osWaitForever);
	assert_param(state[5] == W25Q_OK);

	// Verify data
	assert_param(memcmp(tx_data, tx_data_origine, 538) == 0);
	assert_param(memcmp(rx_data + 0x00000f0f, tx_data_origine, 538) == 0);

	osThreadExit_Cstm();
}
