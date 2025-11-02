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

#include "FreeRTOS.h"
#include "cmsis_os.h"
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
    [W25Q_PROG_SECURITY_REG]     	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    [W25Q_ERASE_SECURITY_REG]    	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,

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

/**
 * @brief Effectue une transaction SPI complète avec le composant W25Q.
 *
 * Cette fonction combine la phase de transmission (commande + adresse)
 * et la phase de réception (lecture de données ou de registres) dans une
 * seule transaction SPI, en gardant le signal CS bas pendant toute la durée.
 *
 * Principe :
 *   - Le protocole SPI du W25Q est séquentiel :
 *       [ Commande ][ Adresse (optionnelle) ][ Données lues ou écrites ]
 *   - Pendant la phase de lecture (MISO actif), le composant n’échantillonne
 *     plus MOSI. Les octets transmis après la commande ne servent qu’à générer
 *     les cycles d’horloge nécessaires au décalage des données.
 *   - Le contenu de ces octets « dummy » n’a donc aucune importance pour le
 *     périphérique : aucune initialisation particulière du buffer TX n’est requise.
 *
 * Contraintes :
 *   - Le buffer TX (tx_buf) doit être dimensionné pour contenir à la fois
 *     la commande, l’adresse et les octets d’horloge de lecture :
 *       → taille minimale = tx_len + rx_len
 *   - Le buffer RX (rx_buf) doit être de la même taille.
 *   - Les rx_len premiers octets reçus correspondent à l’écho de la commande
 *     et de l’adresse, et peuvent être ignorés.
 *   - CS doit rester à l’état bas pendant toute la transaction.
 *
 * Exemples :
 *   - Lecture d’un registre :  tx = [0x05, 0x00], rx = [x, SR1]
 *   - Lecture de données 4B :  tx = [0x13, A3, A2, A1, A0, 0x00, 0x00, ...]
 *                              rx = [x, x, x, x, x, D0, D1, D2, ...]
 *
 * @param chip    Pointeur sur la structure du périphérique W25Q.
 * @param tx_buf  Buffer à transmettre (commande + adresse + octets de lecture).
 * @param rx_buf  Buffer de réception (même taille que tx_buf).
 * @param tx_len  Longueur de la phase commande/adresse (en octets).
 * @param rx_len  Longueur de la phase lecture (en octets).
 * @return        W25Q_OK en cas de succès, W25Q_SPI_ERR en cas d’échec SPI.
 */
static W25Q_STATE W25Q_SPI_TxRx(W25Q_Chip	*chip,
                        		uint8_t		*tx_buf,
                        		uint8_t		*rx_buf,
                        		uint16_t	 tx_len,
                        		uint16_t	 rx_len) {
	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(chip->cs_bank, chip->cs_pin, GPIO_PIN_RESET);

	status = HAL_OK;

	if (tx_len > 0 && tx_buf != NULL) {
		status = HAL_SPI_Transmit(chip->hspi, tx_buf, tx_len, HAL_MAX_DELAY);
	}

	if (status == HAL_OK && rx_len > 0 && rx_buf != NULL) {
		uint8_t *rx_ptr = rx_buf + tx_len;
		status = HAL_SPI_Receive(chip->hspi, rx_ptr, rx_len, HAL_MAX_DELAY);
	}

	HAL_GPIO_WritePin(chip->cs_bank, chip->cs_pin, GPIO_PIN_SET);

	return (status == HAL_OK) ? W25Q_OK : W25Q_SPI_ERR;
}





/* -------------------------------------------------------------------------- */
/*                          Niveau 1 : Command primitives                     */
/* -------------------------------------------------------------------------- */

static inline W25Q_STATE W25Q_WaitForReady(W25Q_Chip *chip) {
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
W25Q_STATE W25Q_SendCmd(W25Q_Chip *chip, uint8_t cmd) {
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

    st = W25Q_SPI_TxRx(chip, &cmd, NULL, 1, 0);
    if (st != W25Q_OK) return st;

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
W25Q_STATE W25Q_SendCmdAddr(W25Q_Chip *chip, uint8_t cmd, uint32_t addr) {
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

    st = W25Q_SPI_TxRx(chip, tx, NULL, 5, 0);
    if (st != W25Q_OK) return st;

    if (W25Q_IsCmdNeedWaitAfter(cmd)) {
        st = W25Q_WaitForReady(chip);
		if (st != W25Q_OK) return st;
	}

    return st;
}

W25Q_STATE W25Q_ReadStatus(W25Q_Chip *chip, uint8_t sr_index) {
	W25Q_STATE st;
	uint8_t tx_buf[2] = { 0 };
	uint8_t rx_buf[2] = { 0 };
	sr_index--;

	switch (sr_index) {
	case 0:
		tx_buf[0] = W25Q_READ_SR1;
		break;
	case 1:
		tx_buf[0] = W25Q_READ_SR2;
		break;
	case 2:
		tx_buf[0] = W25Q_READ_SR3;
		break;
	default:
		return W25Q_PARAM_ERR;
	}

	st = W25Q_SPI_TxRx(chip, tx_buf, rx_buf, 1, 1);
	if (st != W25Q_OK) return st;

	chip->status_reg &= ~(     0xFF << (sr_index * 8));
	chip->status_reg |=  (rx_buf[1] << (sr_index * 8));

	return W25Q_OK;
}

W25Q_STATE W25Q_WriteStatus(W25Q_Chip *chip, uint8_t sr_index, uint8_t value) {
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

	st = W25Q_SPI_TxRx(chip, tx_buf, NULL, 2, 0);
	if (st != W25Q_OK) return st;

	chip->status_reg &= ~( 0xFF << (sr_index * 8));
	chip->status_reg |=  (value << (sr_index * 8));

	return W25Q_OK;
}

W25Q_STATE W25Q_ReadID(W25Q_Chip *w25q_chip, uint8_t *id) {
	uint8_t txBuf[4] = { W25Q_READ_JEDEC_ID, 0, 0, 0 };
	uint8_t rxBuf[4]; // First byte is dummy

	W25Q_STATE st = W25Q_SPI_TxRx(w25q_chip, txBuf, rxBuf, 1, 3);
	if (st != W25Q_OK) return st;

	id[0] = rxBuf[1];
	id[1] = rxBuf[2];
	id[2] = rxBuf[3];

	return W25Q_OK;
}





/* -------------------------------------------------------------------------- */
/*                        Niveau 2 : Fonctions logiques                        */
/* -------------------------------------------------------------------------- */

W25Q_STATE W25Q_Init(W25Q_Chip *w25q_chip, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_bank, uint16_t cs_pin) {
	w25q_chip->hspi = hspi;
	w25q_chip->cs_bank = cs_bank;
	w25q_chip->cs_pin = cs_pin;

	w25q_chip->sem_id = osSemaphoreNew(1, 1, NULL);
	if (w25q_chip->sem_id == NULL) return W25Q_SEM_ERR;

	W25Q_STATE st;

	// Read and check ID
	uint8_t id_buf[3];
	st = W25Q_ReadID(w25q_chip, id_buf);
	if (st != W25Q_OK) return st;
	if (id_buf[0] != W25Q_MANUFACTURER_ID) return W25Q_CHIP_ERR;
	if (W25Q_V_FULL_DEVICE_ID != (uint32_t)((id_buf[1] << 8) | id_buf[2])) return W25Q_PARAM_ERR;

	// Load configuration
	uint32_t status_reg = 0; // Initialize status register to 0
	status_reg |= (1 << W25Q_SR3_ADP_BIT); // Set addr mode to 4-byte

	st = W25Q_WriteStatus(w25q_chip, 1, (uint8_t)(status_reg >>  0));
	if (st != W25Q_OK) return st;
	st = W25Q_WriteStatus(w25q_chip, 2, (uint8_t)(status_reg >>  8));
	if (st != W25Q_OK) return st;
	st = W25Q_WriteStatus(w25q_chip, 3, (uint8_t)(status_reg >> 16));
	if (st != W25Q_OK) return st;

	// Verify configuration
	st = W25Q_ReadStatus(w25q_chip, 1);
	if (st != W25Q_OK) return st;
	st = W25Q_ReadStatus(w25q_chip, 2);
	if (st != W25Q_OK) return st;
	st = W25Q_ReadStatus(w25q_chip, 3);
	if (st != W25Q_OK) return st;

	if (!W25Q_STATUS_REG(w25q_chip, W25Q_SR3_ADS_BIT)) {
		// Try to enable 4-byte mode if not set already
		st = W25Q_SendCmd(w25q_chip, W25Q_ENABLE_4B_MODE);
	}

	return st;
}

// Data must have the first 5 bytes reserved for command and address
// Data size must be <= 256 + 5 bytes = 261 bytes (1 page + command and address)
static W25Q_STATE W25Q_PageProgram(W25Q_Chip *w25q_chip, uint8_t *buffer, uint32_t addr, uint16_t buf_size) {
	W25Q_STATE st;

	uint32_t flash_size = W25Q_MEM_FLASH_SIZE * 1000000 / 8; // MBites to bytes

	st = W25Q_WaitForReady(w25q_chip);
	if (st != W25Q_OK) return st;
	if (!W25Q_STATUS_REG(w25q_chip, W25Q_SR1_WEL_BIT)) {
		st = W25Q_SendCmd(w25q_chip, W25Q_WRITE_ENABLE);
		if (st != W25Q_OK) return st;
	}
	buf_size = (buf_size - 5) > flash_size ? flash_size + 5 : buf_size;
	*(buffer + 0) = W25Q_PAGE_PROGRAM_4B;	// Command
	*(buffer + 1) = (uint8_t)(addr >> 24);	// Address
	*(buffer + 2) = (uint8_t)(addr >> 16);	// Address
	*(buffer + 3) = (uint8_t)(addr >>  8);	// Address
	*(buffer + 4) = (uint8_t)(addr >>  0);	// Address

	st = W25Q_SPI_TxRx(w25q_chip, buffer, NULL, buf_size, 0);

	return st;
}

// Data must have the first 5 bytes reserved for command and address
// Data size must be <= flash size + 5 bytes (data + command and address)
// Data will be modified during the process but the content will remain the same after the function
W25Q_STATE W25Q_WriteData(W25Q_Chip *w25q_chip, uint8_t *buffer, uint32_t addr, uint32_t buf_size) {
	W25Q_STATE st;

	uint32_t flash_size = W25Q_MEM_FLASH_SIZE * 1000000 / 8; // MBites to bytes
	uint32_t data_size = buf_size - 5; // Exclude command and address size
	data_size = (data_size + addr) > flash_size ? flash_size - addr : data_size;
	uint8_t *base = buffer; // Save based pointer

	while (data_size > 0) {
		uint32_t relative_addr = addr % W25Q_MEM_PAGE_SIZE;
		uint16_t data_size_page = (data_size + relative_addr) > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE - relative_addr : data_size;
		
		if (buffer != base) {
			memcpy(base, buffer, 5);	// Store last 5 bytes of data to reserved space
			st = W25Q_PageProgram(w25q_chip, buffer, addr, data_size_page + 5);
			memcpy(buffer, base, 5);	// Restore last 5 bytes of data from reserved space
		} else {
			st = W25Q_PageProgram(w25q_chip, buffer, addr, data_size_page + 5);
		}
		if (st != W25Q_OK) return st;

		data_size -= data_size_page;
		addr += data_size_page;
		buffer += data_size_page;
	}
	return W25Q_OK;
}

// Data must have the first 5 bytes reserved for command and address
// Data size must be <= flash size + 5 bytes (data + command and address)
W25Q_STATE W25Q_ReadData(W25Q_Chip *w25q_chip, uint8_t *data_buf, uint32_t addr, uint32_t data_size) {
	W25Q_STATE state;
	state =	W25Q_WaitForReady(w25q_chip);

	*(data_buf + 0) = W25Q_READ_DATA_4B;		// Command
	*(data_buf + 1) = (uint8_t)(addr >> 24);	// Address
	*(data_buf + 2) = (uint8_t)(addr >> 16);	// Address
	*(data_buf + 3) = (uint8_t)(addr >>  8);	// Address
	*(data_buf + 4) = (uint8_t)(addr >>  0);	// Address

	state = W25Q_SPI_TxRx(w25q_chip, data_buf, data_buf, 5, data_size);

	return state;
}




/* ============================== FreeRTOS ============================== */




/* -------------------------------------------------------------------------- */
/*                          Niveau 0 : SPI transaction                        */
/* -------------------------------------------------------------------------- */

static W25Q_STATE W25Q_SPI_TxRx_RTOS(W25Q_Chip	*chip,
                         			 uint8_t	*tx_buf,
                         			 uint8_t	*rx_buf,
                         			 uint16_t	 tx_len,
                         			 uint16_t	 rx_len) {
	uint8_t *rx_ptr = NULL;
	if (rx_buf != NULL && rx_len > 0) {
		rx_ptr = rx_buf + tx_len;
	}

	HAL_StatusTypeDef status = TASK_HAL_SPI_TransmitThenReceive_DMA(
		chip->hspi,
		chip->cs_bank,
		chip->cs_pin,
		tx_buf,
		tx_len,
		rx_ptr,
		rx_len
	);

	return (status == HAL_OK) ? W25Q_OK : W25Q_SPI_ERR;
}





/* -------------------------------------------------------------------------- */
/*                          Niveau 1 : Command primitives                     */
/* -------------------------------------------------------------------------- */

static inline W25Q_STATE W25Q_WaitForReady_RTOS_base(W25Q_Chip *chip, bool lock_sem) {
	W25Q_STATE st;
	do {
		st = W25Q_ReadStatus_RTOS_base(chip, 1, lock_sem);
		if (st != W25Q_OK) return st;
	} while (W25Q_STATUS_REG(chip, W25Q_SR1_BUSY_BIT));
	return W25Q_OK;
}

W25Q_STATE W25Q_SendCmd_RTOS_base(W25Q_Chip *chip, uint8_t cmd, bool lock_sem) {
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
		if (st != W25Q_OK) goto exit_release;
	}

	if (W25Q_IsCmdRequiresWEL(cmd)) {
		// No need to check status again, already done above with WaitForReady
		if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
			st = W25Q_SendCmd_RTOS_NoLock(chip, W25Q_WRITE_ENABLE);
			if (st != W25Q_OK) goto exit_release;
		}
	}

	st = W25Q_SPI_TxRx_RTOS(chip, &cmd, NULL, 1, 0);
	if (st != W25Q_OK) goto exit_release;

	if (W25Q_IsCmdNeedWaitAfter(cmd)) {
		st = W25Q_WaitForReady_RTOS_NoLock(chip);
		if (st != W25Q_OK) goto exit_release;
	}

exit_release:
	if (need_release) {
		if (osSemaphoreRelease(chip->sem_id) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	return st;
}

W25Q_STATE W25Q_SendCmdAddr_RTOS_base(W25Q_Chip *chip, uint8_t cmd, uint32_t addr, bool lock_sem) {
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
		if (st != W25Q_OK) goto exit_release;
	}

	if (W25Q_IsCmdRequiresWEL(cmd)) {
		// No need to check status again, already done above with WaitForReady
		if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
			st =W25Q_SendCmd_RTOS_NoLock(chip, W25Q_WRITE_ENABLE);
			if (st != W25Q_OK) goto exit_release;
		}
	}

	uint8_t tx[5] = {
		cmd,
		(uint8_t)(addr >> 24),
		(uint8_t)(addr >> 16),
		(uint8_t)(addr >>  8),
		(uint8_t)(addr >>  0)
	};

	st = W25Q_SPI_TxRx_RTOS(chip, tx, NULL, 5, 0);
	if (st != W25Q_OK) goto exit_release;

	if (W25Q_IsCmdNeedWaitAfter(cmd)) {
		st = W25Q_WaitForReady_RTOS_NoLock(chip);
		if (st != W25Q_OK) goto exit_release;
	}

exit_release:
	if (need_release) {
		if (osSemaphoreRelease(chip->sem_id) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	return st;
}

W25Q_STATE W25Q_ReadStatus_RTOS_base(W25Q_Chip *chip, uint8_t sr_index, bool lock_sem) {
	W25Q_STATE st = W25Q_OK;
	uint8_t tx_buf[2] = { 0 };
	uint8_t rx_buf[2] = { 0 };
	sr_index--;

	switch (sr_index) {
	case 0:
		tx_buf[0] = W25Q_READ_SR1;
		break;
	case 1:
		tx_buf[0] = W25Q_READ_SR2;
		break;
	case 2:
		tx_buf[0] = W25Q_READ_SR3;
		break;
	default:
		return W25Q_PARAM_ERR;
	}

	if (lock_sem) {
		if (osSemaphoreAcquire(chip->sem_id, osWaitForever) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	st = W25Q_SPI_TxRx_RTOS(chip, tx_buf, rx_buf, 1, 1);
	if (st != W25Q_OK) { goto exit_flag; }

	chip->status_reg &= ~(     0xFF << (sr_index * 8));
	chip->status_reg |=  (rx_buf[1] << (sr_index * 8));


exit_flag:
	if (lock_sem) {
		if (osSemaphoreRelease(chip->sem_id) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	return st;
}

W25Q_STATE W25Q_WriteStatus_RTOS_base(W25Q_Chip *chip, uint8_t sr_index, uint8_t value, bool lock_sem) {
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

	st = W25Q_SPI_TxRx_RTOS(chip, tx_buf, NULL, 2, 0);
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

W25Q_STATE W25Q_ReadID_RTOS_base(W25Q_Chip *w25q_chip, uint8_t *id, bool lock_sem) {
	W25Q_STATE st = W25Q_OK;
	uint8_t txBuf[4] = { W25Q_READ_JEDEC_ID, 0, 0, 0 };
	uint8_t rxBuf[4]; // First byte is dummy

	if (lock_sem) {
		if (osSemaphoreAcquire(w25q_chip->sem_id, osWaitForever) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	st = W25Q_SPI_TxRx_RTOS(w25q_chip, txBuf, rxBuf, 1, 3);
	if (st != W25Q_OK) { goto exit_flag; }

	id[0] = rxBuf[1];
	id[1] = rxBuf[2];
	id[2] = rxBuf[3];

exit_flag:
	if (lock_sem) {
		if (osSemaphoreRelease(w25q_chip->sem_id) != osOK) {
			return W25Q_SEM_ERR;
		}
	}

	return st;
}


// static inline W25Q_STATE W25Q_WaitForReady_RTOS_NoLock(W25Q_Chip *chip) { return W25Q_WaitForReady_RTOS_base(chip, false); }

// inline W25Q_STATE W25Q_SendCmd_RTOS_NoLock(W25Q_Chip *chip, uint8_t cmd) { return W25Q_SendCmd_RTOS_base(chip, cmd, false); }
// inline W25Q_STATE W25Q_SendCmdAddr_RTOS_NoLock(W25Q_Chip *chip, uint8_t cmd, uint32_t addr) { return W25Q_SendCmdAddr_RTOS_base(chip, cmd, addr, false); }
// inline W25Q_STATE W25Q_ReadStatus_RTOS_NoLock(W25Q_Chip *chip, uint8_t sr_index) { return W25Q_ReadStatus_RTOS_base(chip, sr_index, false); }
// inline W25Q_STATE W25Q_WriteStatus_RTOS_NoLock(W25Q_Chip *chip, uint8_t sr_index, uint8_t value) { return W25Q_WriteStatus_RTOS_base(chip, sr_index, value, false); }
// inline W25Q_STATE W25Q_ReadID_RTOS_NoLock(W25Q_Chip *w25q_chip, uint8_t *id) { return W25Q_ReadID_RTOS_base(w25q_chip, id, false); }


// static inline W25Q_STATE W25Q_WaitForReady_RTOS(W25Q_Chip *chip) { return W25Q_WaitForReady_RTOS_base(chip, true); }

// inline W25Q_STATE W25Q_SendCmd_RTOS(W25Q_Chip *chip, uint8_t cmd) { return W25Q_SendCmd_RTOS_base(chip, cmd, true); }
// inline W25Q_STATE W25Q_SendCmdAddr_RTOS(W25Q_Chip *chip, uint8_t cmd, uint32_t addr) { return W25Q_SendCmdAddr_RTOS_base(chip, cmd, addr, true); }
// inline W25Q_STATE W25Q_ReadStatus_RTOS(W25Q_Chip *chip, uint8_t sr_index) { return W25Q_ReadStatus_RTOS_base(chip, sr_index, true); }
// inline W25Q_STATE W25Q_WriteStatus_RTOS(W25Q_Chip *chip, uint8_t sr_index, uint8_t value) { return W25Q_WriteStatus_RTOS_base(chip, sr_index, value, true); }
// inline W25Q_STATE W25Q_ReadID_RTOS(W25Q_Chip *w25q_chip, uint8_t *id) { return W25Q_ReadID_RTOS_base(w25q_chip, id, true); }






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

	args->chip->sem_id = osSemaphoreNew(1, 1, NULL);
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

static W25Q_STATE W25Q_PageProgram_RTOS(W25Q_Chip *w25q_chip, uint8_t *buffer, uint32_t addr, uint16_t buf_size) {
	W25Q_STATE st;
	if (osSemaphoreAcquire(w25q_chip->sem_id, osWaitForever) != osOK) {
		return W25Q_SEM_ERR;
	}
	st = W25Q_WaitForReady_RTOS_NoLock(w25q_chip);
	if (st != W25Q_OK) { goto exit_flag; }
	if (!W25Q_STATUS_REG(w25q_chip, W25Q_SR1_WEL_BIT)) {
		st = W25Q_SendCmd_RTOS_NoLock(w25q_chip, W25Q_WRITE_ENABLE);
		if (st != W25Q_OK) { goto exit_flag; }
	}
	buf_size = (buf_size - 5) > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE + 5 : buf_size;
	*(buffer + 0) = W25Q_PAGE_PROGRAM_4B;	// Command
	*(buffer + 1) = (uint8_t)(addr >> 24);	// Address
	*(buffer + 2) = (uint8_t)(addr >> 16);	// Address
	*(buffer + 3) = (uint8_t)(addr >>  8);	// Address
	*(buffer + 4) = (uint8_t)(addr >>  0);	// Address

	st = W25Q_SPI_TxRx_RTOS(w25q_chip, buffer, NULL, buf_size, 0);
	if (st != W25Q_OK) { goto exit_flag; }

	st = W25Q_WaitForReady_RTOS_NoLock(w25q_chip);

exit_flag:
	if (osSemaphoreRelease(w25q_chip->sem_id) != osOK) {
		return W25Q_SEM_ERR;
	}

	return st;
}

TASK_POOL_ALLOCATE(TASK_W25Q_WriteData);
void TASK_W25Q_WriteData(void *argument) {
	TASK_W25Q_WriteData_ARGS *args = (TASK_W25Q_WriteData_ARGS*)argument;

	uint32_t flash_size = W25Q_MEM_FLASH_SIZE * 1000000 / 8; // MBites to bytes
	uint32_t data_size = args->buf_size - 5; // Exclude command and address size
	data_size = (data_size + args->addr) > flash_size ? flash_size - args->addr : data_size;
	uint8_t *base = args->buffer; // Save based pointer

	while (data_size > 0) {
		uint32_t relative_addr = args->addr % W25Q_MEM_PAGE_SIZE;
		uint16_t data_size_page = (data_size + relative_addr) > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE - relative_addr : data_size;

		if (args->buffer != base) {
			memcpy(base, args->buffer, 5);	// Store last 5 bytes of data to reserved space
			*(args->result) = W25Q_PageProgram_RTOS(args->chip, args->buffer, args->addr, data_size_page + 5);
			memcpy(args->buffer, base, 5);	// Restore last 5 bytes of data from reserved space
		} else {
			*(args->result) = W25Q_PageProgram_RTOS(args->chip, args->buffer, args->addr, data_size_page + 5);
		}
		if (*(args->result) != W25Q_OK) { goto exit_flag; }

		data_size -= data_size_page;
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

	args->buf_size = (args->buf_size - 5) > flash_size ? flash_size + 5 : args->buf_size;
	*(args->buffer + 0) = W25Q_READ_DATA_4B;			// Command
	*(args->buffer + 1) = (uint8_t)(args->addr >> 24);	// Address
	*(args->buffer + 2) = (uint8_t)(args->addr >> 16);	// Address
	*(args->buffer + 3) = (uint8_t)(args->addr >>  8);	// Address
	*(args->buffer + 4) = (uint8_t)(args->addr >>  0);	// Address

	*(args->result) = W25Q_SPI_TxRx_RTOS(args->chip, args->buffer, args->buffer, 5, args->buf_size - 5);

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

void W25Q_ReadWriteTest(W25Q_Chip *chip) {
	W25Q_STATE state = W25Q_OK;

	uint8_t rx_data[4096 + 512 + 5] = { 0 };
	uint8_t tx_data_origine[538] = "Hello, W25Q256! This is a test of the W25Q256 flash memory chip. \
Let's see if it works properly. We will write this data to the flash memory and then read it back to verify\
the integrity of the data. If everything goes well, we should see the same data we wrote.\
This message is intentionally made longer to test the page programming and reading capabilities of the chip.\
We will also check if the data spans multiple pages and sectors to ensure that the implementation is robust.\
Thank you for your attention and happy coding!";

	uint8_t tx_data[538 + 5] = { 0 };
	memcpy(tx_data + 5, tx_data_origine, 538);
	
	state = W25Q_ReadData(chip, rx_data, 0x00000000, 4096 + 512 + 5);
	assert_param(state == W25Q_OK);

	state = W25Q_SendCmdAddr(chip, W25Q_SECTOR_ERASE_4B, 0x00000000);
	assert_param(state == W25Q_OK);

	state = W25Q_SendCmdAddr(chip, W25Q_SECTOR_ERASE_4B, 0X00001000);
	assert_param(state == W25Q_OK);

	state = W25Q_ReadData(chip, rx_data, 0x00000000, 4096 + 512 + 5);
	assert_param(state == W25Q_OK);

	state = W25Q_WriteData(chip, tx_data, 0x00000f0f, 538 + 5);
	assert_param(state == W25Q_OK);

	state = W25Q_ReadData(chip, rx_data, 0x00000000, 4096 + 512 + 5);
	assert_param(state == W25Q_OK);

	assert_param(memcmp(tx_data + 5, tx_data_origine, 538) == 0);
	assert_param(memcmp(rx_data + 0x00000f0f + 5, tx_data_origine, 538) == 0);
}



TASK_POOL_ALLOCATE(TASK_W25Q_ReadWriteTest);
void TASK_W25Q_ReadWriteTest(void *argument) {
	TASK_W25Q_ReadWriteTest_ARGS *args = (TASK_W25Q_ReadWriteTest_ARGS*)argument;

	W25Q_STATE state[7] = { W25Q_OK };
	uint8_t rx_data[4096 + 512 + 5] = { 0 };
	uint8_t tx_data_origine[538] = "Hello, W25Q256! This is a test of the W25Q256 flash memory chip. \
Let's see if it works properly. We will write this data to the flash memory and then read it back to verify\
the integrity of the data. If everything goes well, we should see the same data we wrote.\
This message is intentionally made longer to test the page programming and reading capabilities of the chip.\
We will also check if the data spans multiple pages and sectors to ensure that the implementation is robust.\
Thank you for your attention and happy coding!";

	uint8_t tx_data[538 + 5] = { 0 };
	memcpy(tx_data + 5, tx_data_origine, 538);

	
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
	read_args.buf_size = 4096 + 512 + 5;
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

	erase_args.addr = 0x11111111;
	erase_args.result = &state[6];
	erase_args.done_flags = done_flags[6];

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
	write_args.buf_size = 538 + 5;
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
	assert_param(memcmp(tx_data + 5, tx_data_origine, 538) == 0);
	assert_param(memcmp(rx_data + 0x00000f0f + 5, tx_data_origine, 538) == 0);

	osThreadExit_Cstm();
}





// TASK_POOL_CREATE(ASYNC_W25Q_ReadStatusReg);

// void ASYNC_W25Q_ReadStatusReg_init(TASK *self, W25Q_Chip *w25q_chip) {
// 	ASYNC_W25Q_ReadStatusReg_CONTEXT *context = (ASYNC_W25Q_ReadStatusReg_CONTEXT*)self->context;

// 	context->w25q_chip = w25q_chip;

// 	context->state = ASYNC_W25Q_START;
// 	// can skip the waiting state if the chip is busy because this
// 	// task is not going to change the internal state of the chip

// 	memset(context->dma_complete, false, sizeof(bool) * 3);

// 	context->tx_buf[0] = W25Q_READ_SR1;
// 	context->tx_buf[1] = W25Q_READ_SR2;
// 	context->tx_buf[2] = W25Q_READ_SR3;
// }

// TASK_RETURN ASYNC_W25Q_ReadStatusReg(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_ReadStatusReg_CONTEXT *context = (ASYNC_W25Q_ReadStatusReg_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_W25Q_WAIT_W25Q: { break;} // what ever if w25q is taken by another task
// 	case ASYNC_W25Q_START: {	// if SR1, SR2 or SR3, then read
// 		TASK *task;
// 		for (uint8_t i = 0; i < 3; i++) {	
// 			task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
// 			ASYNC_SPI_TxRx_DMA_init_W25Q(context->tx_buf + i, context->rx_buf + i, 1, 1);
// 			task->is_done = &(context->dma_complete[i]);
// 		}
// 		context->state = ASYNC_W25Q_WAIT;
// 		break;}
// 	case ASYNC_W25Q_WAIT: {
// 		if (context->dma_complete[0] &&
// 		    context->dma_complete[1] &&
// 			context->dma_complete[2]) {
// 			context->state = ASYNC_W25Q_END;
// 		}
// 		break;}
// 	case ASYNC_W25Q_END: {
// 		for (uint8_t i = 0; i < 8; i++) {
// 			context->w25q_chip->status_bits[i +  0] = (context->rx_buf[0] >> i) & 0x01; // SR1
// 			context->w25q_chip->status_bits[i +  8] = (context->rx_buf[1] >> i) & 0x01; // SR2
// 			context->w25q_chip->status_bits[i + 16] = (context->rx_buf[2] >> i) & 0x01; // SR3
// 		}
// 		return TASK_RETURN_STOP;
// 		break;}
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_WriteEnable);

// void ASYNC_W25Q_WriteEnable_init(TASK *self, W25Q_Chip *w25q_chip) {
// 	ASYNC_W25Q_WriteEnable_CONTEXT *context = (ASYNC_W25Q_WriteEnable_CONTEXT*)self->context;
// 	context->w25q_chip = w25q_chip;
// }

// TASK_RETURN ASYNC_W25Q_WriteEnable(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_WriteEnable_CONTEXT *context = (ASYNC_W25Q_WriteEnable_CONTEXT*)self->context;

// 	uint8_t tx_buf[1] = { W25Q_WRITE_ENABLE };
// 	TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
// 	ASYNC_SPI_TxRx_DMA_init_W25Q(tx_buf, NULL, 1, 0);
// 	task->is_done = self->is_done;
// 	self->is_done = NULL;

// 	return TASK_RETURN_STOP;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_WaitForReady);

// void ASYNC_W25Q_WaitForReady_init(TASK *self, W25Q_Chip *w25q_chip) {
// 	ASYNC_W25Q_WaitForReady_CONTEXT *context = (ASYNC_W25Q_WaitForReady_CONTEXT*)self->context;

// 	context->w25q_chip = w25q_chip;
// 	context->state = ASYNC_W25Q_START;
// 	// can skip the waiting state if the chip is busy because this
// 	// task is not going to change the internal state of the chip
// }

// TASK_RETURN ASYNC_W25Q_WaitForReady(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_WaitForReady_CONTEXT *context = (ASYNC_W25Q_WaitForReady_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_W25Q_WAIT_W25Q: { break; } // what ever if w25q is taken by another task
// 	case ASYNC_W25Q_START: {
// 		context->read_reg_status_done = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ReadStatusReg, false);
// 		ASYNC_W25Q_ReadStatusReg_init(task, context->w25q_chip);
// 		task->is_done = &(context->read_reg_status_done);
// 		context->state = ASYNC_W25Q_WAIT;
// 		break;}
// 	case ASYNC_W25Q_WAIT: {
// 		if (context->read_reg_status_done) {
// 			if (context->w25q_chip->status_bits[W25Q_BUSY_BIT] == 0) {
// 				context->state = ASYNC_W25Q_END;
// 			} else {
// 				context->state = ASYNC_W25Q_START;
// 			}
// 		}
// 		break;}
// 	case ASYNC_W25Q_END: {
// 		return TASK_RETURN_STOP;
// 		break;}
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_EraseSector);

// void ASYNC_W25Q_EraseSector_init(TASK *self, W25Q_Chip *w25q_chip, uint32_t addr) {
// 	ASYNC_W25Q_EraseSector_CONTEXT *context = (ASYNC_W25Q_EraseSector_CONTEXT*)self->context;

// 	context->w25q_chip = w25q_chip;
// 	context->addr = addr;

// 	context->tx_buf[0] = W25Q_SECTOR_ERASE;
// 	context->tx_buf[1] = (uint8_t)(context->addr >> 24);	// Address
// 	context->tx_buf[2] = (uint8_t)(context->addr >> 16);	// Address
// 	context->tx_buf[3] = (uint8_t)(context->addr >>  8);	// Address
// 	context->tx_buf[4] = (uint8_t)(context->addr >>  0);	// Address

// 	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
// }

// TASK_RETURN ASYNC_W25Q_EraseSector(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_EraseSector_CONTEXT *context = (ASYNC_W25Q_EraseSector_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_W25Q_WaitAndProceed_WAIT_W25Q: {
// 		if (!(context->w25q_chip->ASYNC_busy)) {
// 			context->w25q_chip->ASYNC_busy = true;
// 			context->state = ASYNC_W25Q_WaitAndProceed_START;
// 		}
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_START: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WaitForReady, false);
// 		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
// 		if (context->is_ready) {
// 			if (context->w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
// 				context->is_ready = false;
// 				TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WriteEnable, false);
// 				ASYNC_W25Q_WriteEnable_init(task, context->w25q_chip);
// 				task->is_done = &(context->is_ready);
// 				context->state = ASYNC_W25Q_WaitAndProceed_WAIT_WEL;
// 			} else {
// 				context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
// 			}
// 		}
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {
// 		if (context->is_ready) {
// 			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
// 		}
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_TxRx: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
// 		ASYNC_SPI_TxRx_DMA_init_W25Q(context->tx_buf, NULL, 5, 0);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WaitAndProceed_END;
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_END: {
// 		if (context->is_ready) {
// 			context->w25q_chip->ASYNC_busy = false;
// 			return TASK_RETURN_STOP;
// 		}
// 		break;}
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_EraseAll);

// void ASYNC_W25Q_EraseAll_init(TASK *self, W25Q_Chip *w25q_chip) {
// 	ASYNC_W25Q_EraseAll_CONTEXT *context = (ASYNC_W25Q_EraseAll_CONTEXT*)self->context;

// 	context->w25q_chip = w25q_chip;

// 	context->tx_buf[0] = W25Q_CHIP_ERASE;

// 	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
// }

// TASK_RETURN ASYNC_W25Q_EraseAll(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_EraseAll_CONTEXT *context = (ASYNC_W25Q_EraseAll_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_W25Q_WaitAndProceed_WAIT_W25Q: {
// 		if (!(context->w25q_chip->ASYNC_busy)) {
// 			context->w25q_chip->ASYNC_busy = true;
// 			context->state = ASYNC_W25Q_WaitAndProceed_START;
// 		}
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_START: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WaitForReady, false);
// 		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
// 		if (context->is_ready) {
// 			if (context->w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
// 				context->is_ready = false;
// 				TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WriteEnable, false);
// 				ASYNC_W25Q_WriteEnable_init(task, context->w25q_chip);
// 				task->is_done = &(context->is_ready);
// 				context->state = ASYNC_W25Q_WaitAndProceed_WAIT_WEL;
// 			} else {
// 				context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
// 			}
// 		}
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {
// 		if (context->is_ready) {
// 			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
// 		}
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_TxRx: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
// 		ASYNC_SPI_TxRx_DMA_init_W25Q(context->tx_buf, NULL, 1, 0);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WaitAndProceed_END;
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_END: {
// 		if (context->is_ready) {
// 			context->w25q_chip->ASYNC_busy = false;
// 			return TASK_RETURN_STOP;
// 		}
// 		break;}
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_ReadData_Smol);

// void ASYNC_W25Q_ReadData_Smol_init(TASK					*self,
// 								   W25Q_Chip			*w25q_chip,
// 								   uint8_t	            *data,
// 								   uint16_t 		     data_size,
// 								   uint32_t				 addr) {
// 	ASYNC_W25Q_ReadData_Smol_CONTEXT *context = (ASYNC_W25Q_ReadData_Smol_CONTEXT*)self->context;

// 	context->w25q_chip = w25q_chip;

// 	context->data = data;
// 	context->data_size = data_size;
// 	context->addr = addr;

// 	context->tx_buf[0] = W25Q_READ_DATA;
// 	// context->tx_buf[1] = (uint8_t)(context->addr >> 24);	// Address
// 	// context->tx_buf[2] = (uint8_t)(context->addr >> 16);	// Address
// 	// context->tx_buf[3] = (uint8_t)(context->addr >>  8);	// Address
// 	// context->tx_buf[4] = (uint8_t)(context->addr >>  0);	// Address

// 	context->tx_buf[1] = (uint8_t)(context->addr >> 16);	// Address
// 	context->tx_buf[2] = (uint8_t)(context->addr >>  8);	// Address
// 	context->tx_buf[3] = (uint8_t)(context->addr >>  0);	// Address

// 	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
// }

// TASK_RETURN ASYNC_W25Q_ReadData_Smol(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_ReadData_Smol_CONTEXT *context = (ASYNC_W25Q_ReadData_Smol_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_W25Q_WAIT_W25Q: {
// 		if (!(context->w25q_chip->ASYNC_busy)) {
// 			context->w25q_chip->ASYNC_busy = true;
// 			context->state = ASYNC_W25Q_WaitAndProceed_START;
// 		}
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_START: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WaitForReady, false);
// 		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
// 		if (context->is_ready) {
// 			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
// 		}
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {break;} // case never happens
// 	case ASYNC_W25Q_WaitAndProceed_TxRx: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
// 		ASYNC_SPI_TxRx_DMA_init_W25Q(context->tx_buf, context->data, 4, context->data_size);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WaitAndProceed_END;
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_END: {
// 		if (context->is_ready) {
// 			context->w25q_chip->ASYNC_busy = false;
// 			return TASK_RETURN_STOP;
// 		}
// 		break;}
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_ReadData);

// void ASYNC_W25Q_ReadData_init(TASK				*self,
// 							  W25Q_Chip			*w25q_chip,
// 							  uint8_t	 		*data,
// 							  uint16_t 			 data_size,
// 							  uint32_t			 addr) {
// 	ASYNC_W25Q_ReadData_CONTEXT *context = (ASYNC_W25Q_ReadData_CONTEXT*)self->context;

// 	context->w25q_chip = w25q_chip;
// 	context->data = data;
// 	context->data_size = data_size;
// 	context->based_addr = addr;

// 	context->last_data_size = data_size;

// 	context->state = ASYNC_W25Q_START;
// }

// TASK_RETURN ASYNC_W25Q_ReadData(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_ReadData_CONTEXT *context = (ASYNC_W25Q_ReadData_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
// 	case ASYNC_W25Q_START: {
// 		context->is_ready = false;

// 		uint32_t addr = context->based_addr + (context->data_size - context->last_data_size);

// 		// Limit the data read size to 256 bytes in order to not saturate the SPI bus and the memory allocator
// 		size_t size_to_copy = W25Q_MEM_PAGE_SIZE * 16;
// 		size_to_copy = context->last_data_size > size_to_copy ? size_to_copy : context->last_data_size; // Limit to the last data size

// 		size_t offset = context->data_size - context->last_data_size;
// 		uint8_t *data_block = context->data + offset;
// 		context->last_data_size -= size_to_copy;

// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ReadData_Smol, false);
// 		ASYNC_W25Q_ReadData_Smol_init(task, context->w25q_chip, data_block, size_to_copy, addr);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WAIT;
// 		break;}
// 	case ASYNC_W25Q_WAIT: {
// 		if (context->is_ready) {
// 			if (context->last_data_size) {
// 				context->state = ASYNC_W25Q_START;
// 			} else {
// 				context->state = ASYNC_W25Q_END;
// 			}
// 		}
// 		break; }
// 	case ASYNC_W25Q_END: {
// 		return TASK_RETURN_STOP;
// 		break; }
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_PageProgram);

// void ASYNC_W25Q_PageProgram_init(TASK				*self,
// 							     W25Q_Chip  		*w25q_chip,
// 								 uint8_t   			*data,
// 								 uint16_t 		 	 data_size,
// 							     uint32_t    		 addr) {
// 	ASYNC_W25Q_PageProgram_CONTEXT *context = (ASYNC_W25Q_PageProgram_CONTEXT*)self->context;
	
// 	context->w25q_chip = w25q_chip;

// 	size_t flash_size = W25Q_MEM_FLASH_SIZE * 1000000; // MBytes to bytes

// 	context->addr = addr;
// 	data_size = data_size > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE : data_size;
// 	data_size = flash_size < addr ? 0 : data_size;
// 	data_size = data_size + addr > flash_size ? flash_size - addr : data_size;
// 	context->data_size = data_size;

// 	uint8_t tx_buf_start[5] = { W25Q_PAGE_PROGRAM,
// 								// (uint8_t)(context->addr >> 24),		// Address
// 								(uint8_t)(context->addr >> 16),		// Address
// 								(uint8_t)(context->addr >>  8),		// Address
// 								(uint8_t)(context->addr >>  0) };	// Address

// 	context->tx_buf = GMS_alloc(&GMS_memory, data_size + 4);
// 	memcpy(context->tx_buf, tx_buf_start, 4);
// 	memcpy(context->tx_buf + 4, data, data_size);

// 	context->state = ASYNC_W25Q_WaitAndProceed_WAIT_W25Q;
// }

// TASK_RETURN ASYNC_W25Q_PageProgram(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_PageProgram_CONTEXT *context = (ASYNC_W25Q_PageProgram_CONTEXT*)self->context;	

// 	switch (context->state) {
// 	case ASYNC_W25Q_WaitAndProceed_WAIT_W25Q: {
// 		if (context->data_size == 0) { // No data
// 			GMS_free(&GMS_memory, context->tx_buf);
// 			return TASK_RETURN_STOP;
// 		}
// 		if (!context->w25q_chip->ASYNC_busy) {
// 			context->state = ASYNC_W25Q_WaitAndProceed_START;
// 			context->w25q_chip->ASYNC_busy = true;
// 		}
// 		break; }
// 	case ASYNC_W25Q_WaitAndProceed_START: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WaitForReady, false);
// 		ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WaitAndProceed_WAIT_READY;
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_WAIT_READY: {
// 		if (context->is_ready) {
// 			if (context->w25q_chip->status_bits[W25Q_WEL_BIT] == 0) {
// 				context->is_ready = false;
// 				TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WriteEnable, false);
// 				ASYNC_W25Q_WriteEnable_init(task, context->w25q_chip);
// 				task->is_done = &(context->is_ready);
// 				context->state = ASYNC_W25Q_WaitAndProceed_WAIT_WEL;
// 			} else {
// 				context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
// 			}
// 		}
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_WAIT_WEL: {
// 		if (context->is_ready) {
// 			context->state = ASYNC_W25Q_WaitAndProceed_TxRx;
// 		}
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_TxRx: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SPI_TxRx_DMA, false);
// 		ASYNC_SPI_TxRx_DMA_init_W25Q(context->tx_buf, NULL, context->data_size + 4, 0);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WaitAndProceed_END;
// 		break;}
// 	case ASYNC_W25Q_WaitAndProceed_END: {
// 		if (context->is_ready) {
// 			context->w25q_chip->ASYNC_busy = false;
// 			GMS_free(&GMS_memory, context->tx_buf);
// 			return TASK_RETURN_STOP;
// 		}
// 		break;}
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_WriteData);

// void ASYNC_W25Q_WriteData_init(TASK					*self,
// 							   W25Q_Chip 			*w25q_chip,
// 							   uint8_t	 			*data,
// 							   size_t	 		 	 data_size,
// 							   uint32_t			 	 addr) {
// 	ASYNC_W25Q_WriteData_CONTEXT *context = (ASYNC_W25Q_WriteData_CONTEXT*)self->context;

// 	context->w25q_chip = w25q_chip;
// 	context->data = data;
// 	context->data_size = data_size;
// 	context->based_addr = addr;

// 	context->last_data_size = data_size;

// 	context->state = ASYNC_W25Q_START;
// }

// TASK_RETURN ASYNC_W25Q_WriteData(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_WriteData_CONTEXT *context = (ASYNC_W25Q_WriteData_CONTEXT*)self->context;	

// 	switch (context->state) {
// 	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
// 	case ASYNC_W25Q_START: {
// 		context->is_ready = false;

// 		uint32_t addr = context->based_addr + (context->data_size - context->last_data_size);

// 		size_t offset = addr % W25Q_MEM_PAGE_SIZE;
// 		size_t size_to_copy = W25Q_MEM_PAGE_SIZE - offset;
// 		size_to_copy = context->last_data_size > size_to_copy ? size_to_copy : context->last_data_size;

// 		size_t offset_data = context->data_size - context->last_data_size;
// 		uint8_t *data_block = context->data + offset_data;
// 		context->last_data_size -= size_to_copy;

// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_PageProgram, false);
// 		ASYNC_W25Q_PageProgram_init(task, context->w25q_chip, data_block, size_to_copy, addr);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WAIT;
// 		break; }
// 	case ASYNC_W25Q_WAIT: {
// 		if (context->is_ready) {
// 			if (context->last_data_size) {
// 				context->state = ASYNC_W25Q_START;
// 			} else {
// 				context->state = ASYNC_W25Q_END;
// 			}
// 		}
// 		break; }
// 	case ASYNC_W25Q_END: {
// 		return TASK_RETURN_STOP;
// 		break; }
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_ScanIfSectorErased);

// void ASYNC_W25Q_ScanIfSectorErased_init(TASK *self, W25Q_Chip *w25q_chip, uint32_t addr, bool *is_erased) {
// 	ASYNC_W25Q_ScanIfSectorErased_CONTEXT *context = (ASYNC_W25Q_ScanIfSectorErased_CONTEXT*)self->context;

// 	context->w25q_chip = w25q_chip;
// 	context->addr = addr / 4096 * 4096; // align to sector start

// 	context->is_erased = is_erased;
// 	*context->is_erased = true;

// 	context->state = ASYNC_W25Q_START;
// }

// TASK_RETURN ASYNC_W25Q_ScanIfSectorErased(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_ScanIfSectorErased_CONTEXT *context = (ASYNC_W25Q_ScanIfSectorErased_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
// 	case ASYNC_W25Q_START: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ReadData, false);
// 		ASYNC_W25Q_ReadData_init(task, context->w25q_chip, context->data, 4096, context->addr);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WAIT;
// 		break; }
// 	case ASYNC_W25Q_WAIT: {
// 		if (context->is_ready) {
// 			context->state = ASYNC_W25Q_END;
// 		}
// 		break; }
// 	case ASYNC_W25Q_END: {
// 		for (uint16_t i = 0; i < W25Q_MEM_PAGE_SIZE; i++) {
// 			if (context->data[i] != 0xFF) {
// 				*(context->is_erased) = false;
// 				break;
// 			}
// 		}
// 		return TASK_RETURN_STOP;
// 		break; }
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_ScanIfBlockErased);

// void ASYNC_W25Q_ScanIfBlockErased_init(TASK *self, W25Q_Chip *w25q_chip, uint32_t addr, bool *sectors, bool *is_erased) {
// 	ASYNC_W25Q_ScanIfBlockErased_CONTEXT *context = (ASYNC_W25Q_ScanIfBlockErased_CONTEXT*)self->context;

// 	context->w25q_chip = w25q_chip;
// 	context->addr = addr / (4096 * 16) * (4096 * 16); // align to block start
// 	context->i = 0;

// 	context->sectors = sectors;
// 	memset(context->sectors, true, sizeof(bool) * 16); // 16 sectors in a block
// 	context->is_erased = is_erased;
// 	*context->is_erased = true;

// 	context->state = ASYNC_W25Q_START;
// }

// TASK_RETURN ASYNC_W25Q_ScanIfBlockErased(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_ScanIfBlockErased_CONTEXT *context = (ASYNC_W25Q_ScanIfBlockErased_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
// 	case ASYNC_W25Q_START: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ScanIfSectorErased, false);
// 		ASYNC_W25Q_ScanIfSectorErased_init(task, context->w25q_chip, context->addr, context->sectors + context->i);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WAIT;
// 		break; }
// 	case ASYNC_W25Q_WAIT: {
// 		if (context->is_ready) {
// 			*(context->is_erased) &= context->sectors[context->i];
// 			if (context->i == 16) {
// 				context->state = ASYNC_W25Q_END;
// 			} else {
// 				context->i++;
// 				context->addr += 4096;
// 				context->state = ASYNC_W25Q_START;
// 			}
// 		}
// 		break; }
// 	case ASYNC_W25Q_END: {
// 		return TASK_RETURN_STOP;
// 		break; }
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_ScanIfChipErased);

// void ASYNC_W25Q_ScanIfChipErased_init(TASK *self, W25Q_Chip *w25q_chip, bool *blocks, bool *is_erased) {
// 	ASYNC_W25Q_ScanIfChipErased_CONTEXT *context = (ASYNC_W25Q_ScanIfChipErased_CONTEXT*)self->context;

// 	context->w25q_chip = w25q_chip;

// 	context->is_erased = is_erased;
// 	*context->is_erased = true;

// 	context->blocks = blocks;
// 	memset(context->blocks, true, sizeof(bool) * 1024); // 1024 blocks
// 	context->addr = 0; // start from the beginning
// 	context->i = 0;

// 	context->state = ASYNC_W25Q_START;
// }

// TASK_RETURN ASYNC_W25Q_ScanIfChipErased(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_ScanIfChipErased_CONTEXT *context = (ASYNC_W25Q_ScanIfChipErased_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
// 	case ASYNC_W25Q_START: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ScanIfBlockErased, false);
// 		ASYNC_W25Q_ScanIfBlockErased_init(task, context->w25q_chip, context->addr, context->sectors, context->blocks + context->i);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WAIT;
// 		break; }
// 	case ASYNC_W25Q_WAIT: {
// 		if (context->is_ready) {
// 			*(context->is_erased) &= context->blocks[context->i];
// 			// if (context->i == 1024) {
// 			if (context->i == 512) {
// 				context->state = ASYNC_W25Q_END;
// 			} else {
// 				context->i++;
// 				context->addr += 4096 * 16;
// 				context->state = ASYNC_W25Q_START;

// 				// char str[64];
// 				// sprintf(str, "Scanning block %d/%d\r\n", context->i, 1024);
// 				// CDC_Transmit_FS((uint8_t*)str, strlen(str));
// 			}
// 		}
// 		break; }
// 	case ASYNC_W25Q_END: {
// 		return TASK_RETURN_STOP;
// 		break; }
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_W25Q_USB_ScanMemory);

// void ASYNC_W25Q_USB_ScanMemory_init(TASK *self, W25Q_Chip *w25q_chip) {
// 	ASYNC_W25Q_USB_ScanMemory_CONTEXT *context = (ASYNC_W25Q_USB_ScanMemory_CONTEXT*)self->context;

// 	context->w25q_chip = w25q_chip;
// 	context->is_erased = false;

// 	context->addr = 0; // start from the beginning

// 	context->state = ASYNC_W25Q_START;
// }

// TASK_RETURN ASYNC_W25Q_USB_ScanMemory(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_W25Q_USB_ScanMemory_CONTEXT *context = (ASYNC_W25Q_USB_ScanMemory_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_W25Q_WAIT_W25Q: { break; } // case never happens
// 	case ASYNC_W25Q_START: {
// 		context->is_ready = false;
// 		TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ReadData, false);
// 		ASYNC_W25Q_ReadData_init(task, context->w25q_chip, context->data, 4096, context->addr);
// 		task->is_done = &(context->is_ready);
// 		context->state = ASYNC_W25Q_WAIT;
// 		break; }
// 	case ASYNC_W25Q_WAIT: {
// 		if (context->is_ready) {
// 			context->state = ASYNC_W25Q_END;
// 		}
// 		break; }
// 	case ASYNC_W25Q_END: {
// 		HAL_Delay(10);
// 		CDC_Transmit_FS(context->data, 4096);
// 		context->addr += 4096;
// 		if (context->addr % 0x2000000 == 0) {
// 			HAL_Delay(1000); // set a delay to avoid flooding the USB
// 			__NOP();
// 		}
// 		// TODO : CHANGER A 64 MO
// 		if (context->addr >= 0x1000000) { // A CHANGER (OU PAS)
// 		// if (context->addr >= 0x2000) {
// 			HAL_Delay(1);
// 			char str[64];
// 			// sprintf(str, "End of memory scan\r\n");
// 			// CDC_Transmit_FS((uint8_t*)str, strlen(str));
// 			return TASK_RETURN_STOP; // end of memory
// 		} else {
// 			context->state = ASYNC_W25Q_START; // continue scanning
// 		}
// 		break; }
// 	}
// 	return TASK_RETURN_IDLE;
// }


