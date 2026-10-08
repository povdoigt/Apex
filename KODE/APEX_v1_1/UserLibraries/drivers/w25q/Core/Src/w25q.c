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

#include "w25q.h"


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
    // [W25Q_READ_DATA_4B]          	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_ADDR_4B,
    // [W25Q_FAST_READ]             	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_4B]          	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_ADDR_4B,
    // [W25Q_FAST_READ_DUAL_OUT]    	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_DUAL_IO]     	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_QUAD_OUT]    	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_QUAD_IO]     	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    // [W25Q_FAST_READ_DUAL_OUT_4B] 	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_ADDR_4B,
    // [W25Q_FAST_READ_DUAL_IO_4B]  	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_ADDR_4B,
    // [W25Q_FAST_READ_QUAD_OUT_4B] 	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_ADDR_4B,
    // [W25Q_FAST_READ_QUAD_IO_4B]  	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_ADDR_4B,

    /* ----- Program / Erase ----------------------------------------------- */
    // [W25Q_PAGE_PROGRAM]          	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,		// commented due to not used
    // [W25Q_PAGE_PROGRAM_4B]       	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY | W25Q_FLAG_ADDR_4B,
    // [W25Q_PAGE_PROGRAM_QUAD_INP] 	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    // [W25Q_PAGE_PROGRAM_QUAD_INP_4B]	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY | W25Q_FLAG_ADDR_4B,

    [W25Q_SECTOR_ERASE]          	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    [W25Q_SECTOR_ERASE_4B]       	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY | W25Q_FLAG_ADDR_4B,
    [W25Q_32KB_BLOCK_ERASE]      	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    [W25Q_64KB_BLOCK_ERASE]      	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY,
    [W25Q_64KB_BLOCK_ERASE_4B]   	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WEL | W25Q_FLAG_DEVICE_BUSY | W25Q_FLAG_ADDR_4B,
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
    [W25Q_ENABLE_4B_MODE]        	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,		// immediate, does not set BUSY
    [W25Q_DISABLE_4B_MODE]       	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,		// immediate, does not set BUSY
    [W25Q_ENABLE_RESET]          	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY,
    [W25Q_RESET]                 	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WAIT_AFTER,
    [W25Q_POWERDOWN]             	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WAIT_AFTER,
    [W25Q_POWERUP]               	= W25Q_FLAG_VALID | W25Q_FLAG_BUSY | W25Q_FLAG_WAIT_AFTER,
};




/* -------------------------------------------------------------------------- */
/*                          Niveau 0 : SPI transaction                        */
/* -------------------------------------------------------------------------- */


static inline void W25Q_SPI_Begin(W25Q_t *chip) {
	HAL_GPIO_WritePin(chip->config.bus.cs_bank, chip->config.bus.cs_pin, GPIO_PIN_RESET);
}
static inline W25Q_STATE W25Q_SPI_Tx(W25Q_t *chip, const uint8_t *tx_buf, uint16_t tx_len) {
	return HAL_SPI_Transmit(chip->config.bus.hspi, tx_buf, tx_len, HAL_MAX_DELAY) == HAL_OK ? W25Q_OK : W25Q_SPI_ERR;
}
static inline W25Q_STATE W25Q_SPI_Rx(W25Q_t *chip, uint8_t *rx_buf, uint16_t rx_len) {
	return HAL_SPI_Receive(chip->config.bus.hspi, rx_buf, rx_len, HAL_MAX_DELAY) == HAL_OK ? W25Q_OK : W25Q_SPI_ERR;
}
static inline void W25Q_SPI_End(W25Q_t *chip) {
	HAL_GPIO_WritePin(chip->config.bus.cs_bank, chip->config.bus.cs_pin, GPIO_PIN_SET);
}





/* -------------------------------------------------------------------------- */
/*                          Niveau 1 : Command primitives                     */
/* -------------------------------------------------------------------------- */

W25Q_STATE W25Q_WaitForReady(W25Q_t *chip, uint32_t timeout_ms) {
	W25Q_STATE st;
	uint32_t start = HAL_GetTick();
	do {
		st = W25Q_ReadStatus(chip, 1);
		if (st != W25Q_OK) return st;
		if (W25Q_STATUS_REG(chip, W25Q_SR1_BUSY_BIT) && (HAL_GetTick() - start) >= timeout_ms) {
			return W25Q_BUSY_TIMEOUT;
		}
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
 *   - W25Q_SendCmd(chip, W25Q_WRITE_ENABLE, 10);
 *   - W25Q_SendCmd(chip, W25Q_CHIP_ERASE, 400000);
 *   - W25Q_SendCmd(chip, W25Q_ENABLE_4B_MODE, 10);
 *
 * @param chip  Pointeur vers la structure W25Q.
 * @param cmd   Code de la commande SPI à envoyer (ex: 0x06 pour WRITE_ENABLE).
 * @param timeout_ms Attente max (ms) de la fin d'opération (BUSY=0). Doit couvrir l'opération la plus lente envoyée (ex: chip erase ~400 s).
 * @return      W25Q_OK si succès, ou un code d’erreur (W25Q_SPI_ERR, W25Q_PARAM_ERR, etc.).
 */
W25Q_STATE W25Q_SendCmd(W25Q_t *chip, uint8_t cmd, uint32_t timeout_ms) {
    W25Q_STATE st;

    if (!W25Q_IsCmdValid(cmd)) {
		return W25Q_PARAM_ERR;
	}

    if (W25Q_IsCmdRequiresBusyCheck(cmd)) {
		st = W25Q_WaitForReady(chip, timeout_ms);
		if (st != W25Q_OK) return st;
	}

    if (W25Q_IsCmdRequiresWEL(cmd) && cmd != W25Q_WRITE_ENABLE) {
		// No need to check status again, already done above with WaitForReady
		// Even if W25Q_WRITE_ENABLE will return false with W25Q_IsCmdRequiresWEL(cmd),
		// we skip it here to avoid infinite recursion.
		if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
			st = W25Q_SendCmd(chip, W25Q_WRITE_ENABLE, timeout_ms);
			if (st != W25Q_OK) return st;
		}
	}

	W25Q_SPI_Begin(chip);
	st = W25Q_SPI_Tx(chip, &cmd, 1);
	W25Q_SPI_End(chip);
    if (st != W25Q_OK) return st;

    if (W25Q_IsCmdNeedWaitAfter(cmd)) {
        st = W25Q_WaitForReady(chip, timeout_ms);
		if (st != W25Q_OK) return st;
	}

	// Keep the cached ADS in step: it sets the address length of SendCmdAddr
	switch (cmd) {
	case W25Q_ENABLE_4B_MODE:	chip->status_reg |=  (1UL << W25Q_SR3_ADS_BIT); break;
	case W25Q_DISABLE_4B_MODE:	chip->status_reg &= ~(1UL << W25Q_SR3_ADS_BIT); break;
	case W25Q_RESET:
		// SRs reloaded from their non-volatile values, ADS from ADP (SR1 already re-read by WAIT_AFTER)
		st = W25Q_ReadStatus(chip, 2);
		if (st != W25Q_OK) return st;
		st = W25Q_ReadStatus(chip, 3);
		break;
	default: break;
	}

    return st;
}

/**
 * @brief Envoie une commande accompagnée d’une adresse (3 ou 4 octets).
 *
 * Cette fonction gère automatiquement :
 *   - L’attente de fin d’opération précédente (BUSY=0) si nécessaire.
 *   - L’envoi de la commande suivie de l’adresse (big endian, MSB en premier).
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
 *   - Longueur d’adresse : 4 octets pour les opcodes W25Q_FLAG_ADDR_4B (21h, DCh...),
 *     sinon selon le mode courant ADS suivi dans chip->status_reg (20h, 52h, D8h...).
 *     En mode 3 octets, une adresse > 0xFFFFFF est rejetée (W25Q_PARAM_ERR).
 *   - Si la commande requiert un Write Enable (WEL=1), il est activé automatiquement.
 *   - Les flags DEVICE_BUSY et BUSY_REQ0 définissent les conditions de synchronisation.
 *
 * Exemples :
 *   - W25Q_SendCmdAddr(chip, W25Q_PAGE_PROGRAM_4B, 0x00123456, 10);
 *   - W25Q_SendCmdAddr(chip, W25Q_SECTOR_ERASE_4B, 0x00080000, 500);
 *   - W25Q_SendCmdAddr(chip, W25Q_READ_DATA_4B, 0x00000000, 10);
 *
 * @param chip  Pointeur vers la structure W25Q.
 * @param cmd   Code de la commande SPI à envoyer (ex: 0x13 pour READ_DATA_4B).
 * @param addr  Adresse mémoire (A31→A0, limitée à A23→A0 en mode 3 octets).
 * @param timeout_ms Attente max (ms) de la fin d'opération (BUSY=0). Doit couvrir l'opération la plus lente envoyée (ex: sector erase ~400 ms).
 * @return      W25Q_OK si succès, ou un code d’erreur (W25Q_SPI_ERR, W25Q_PARAM_ERR, etc.).
 */
W25Q_STATE W25Q_SendCmdAddr(W25Q_t *chip, uint8_t cmd, uint32_t addr, uint32_t timeout_ms) {
    W25Q_STATE st;

    if (!W25Q_IsCmdValid(cmd)) {
		return W25Q_PARAM_ERR;
	}

	// Address length: fixed for the 4-byte opcodes, otherwise set by the current mode (ADS)
	uint8_t addr_len = (W25Q_IsCmdAddr4B(cmd) || W25Q_STATUS_REG(chip, W25Q_SR3_ADS_BIT)) ? 4u : 3u;
	if (addr_len == 3u && addr > 0x00FFFFFFUL) return W25Q_PARAM_ERR;

    if (W25Q_IsCmdRequiresBusyCheck(cmd)) {
		st = W25Q_WaitForReady(chip, timeout_ms);
		if (st != W25Q_OK) return st;
	}

    if (W25Q_IsCmdRequiresWEL(cmd)) {
		// No need to check status again, already done above with WaitForReady
		if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
			st =W25Q_SendCmd(chip, W25Q_WRITE_ENABLE, timeout_ms);
			if (st != W25Q_OK) return st;
		}
	}

    uint8_t tx[5] = { cmd };
	for (uint8_t i = 0; i < addr_len; i++) {
		tx[1 + i] = (uint8_t)(addr >> (8u * (addr_len - 1u - i)));
	}

	W25Q_SPI_Begin(chip);
	st = W25Q_SPI_Tx(chip, tx, (uint16_t)(1u + addr_len));
	W25Q_SPI_End(chip);
    if (st != W25Q_OK) return st;

    if (W25Q_IsCmdNeedWaitAfter(cmd)) {
        st = W25Q_WaitForReady(chip, timeout_ms);
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

W25Q_STATE W25Q_WriteStatus(W25Q_t *chip, uint8_t sr_index, uint8_t value, W25Q_SR_WRITE mode, uint32_t timeout_ms) {
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

	// A status-register write needs WEL (06h, non-volatile) or 50h (volatile).
	// SendCmd already waits for BUSY=0 before either.
	st = W25Q_SendCmd(chip, (mode == W25Q_SR_WRITE_VOLATILE) ? W25Q_ENABLE_VOLATILE_SR : W25Q_WRITE_ENABLE, timeout_ms);
	if (st != W25Q_OK) return st;

	W25Q_SPI_Begin(chip);
	st = W25Q_SPI_Tx(chip, tx_buf, sizeof(tx_buf));
	W25Q_SPI_End(chip);
	if (st != W25Q_OK) return st;

	// Read-only bits (BUSY, WEL, SUS, ADS) are not written: keep their cached value
	uint32_t writable = ((uint32_t)0xFF << (sr_index * 8)) & ~W25Q_SR_READONLY_MASK;
	chip->status_reg = (chip->status_reg & ~writable) | (((uint32_t)value << (sr_index * 8)) & writable);

	return W25Q_WaitForReady(chip, timeout_ms);
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

/* Applique un champ binaire (OFF=1 / ON=2 dans les enums de config) sur un bit de status_reg. */
static inline void W25Q_CfgBit(uint8_t field, uint8_t bit, uint32_t *mask, uint32_t *bits) {
	if (field == W25Q_CFG_KEEP) return;
	*mask |= (1UL << bit);
	if (field == 2u) *bits |= (1UL << bit);
}

W25Q_STATE W25Q_ConfigToStatus(const W25Q_reg_config_t *reg, uint32_t *mask, uint32_t *bits) {
	*mask = 0;
	*bits = 0;

	// SR1 [5:2] BP3..BP0
	if (reg->block_protect != W25Q_CFG_KEEP) {
		if (reg->block_protect > W25Q_CFG_BP(15)) return W25Q_PARAM_ERR;
		*mask |= 0xFUL << W25Q_SR1_BP0_BIT;
		*bits |= (uint32_t)(reg->block_protect - 1u) << W25Q_SR1_BP0_BIT;
	}

	// Champs binaires : la valeur 2 de chaque enum correspond au bit à 1
	if (reg->top_bottom           > W25Q_CFG_TB_BOTTOM)       return W25Q_PARAM_ERR;
	if (reg->complement           > W25Q_CFG_CMP_ON)          return W25Q_PARAM_ERR;
	if (reg->quad_enable          > W25Q_CFG_QE_ON)           return W25Q_PARAM_ERR;
	if (reg->power_up_addr_mode   > W25Q_CFG_ADP_4B)          return W25Q_PARAM_ERR;
	if (reg->addr_mode            > W25Q_CFG_ADS_4B)          return W25Q_PARAM_ERR;
	if (reg->write_protect_scheme > W25Q_CFG_WPS_INDIVIDUAL)  return W25Q_PARAM_ERR;
	W25Q_CfgBit(reg->top_bottom,           W25Q_SR1_TB_BIT,  mask, bits);
	W25Q_CfgBit(reg->complement,           W25Q_SR2_CMP_BIT, mask, bits);
	W25Q_CfgBit(reg->quad_enable,          W25Q_SR2_QE_BIT,  mask, bits);
	W25Q_CfgBit(reg->power_up_addr_mode,   W25Q_SR3_ADP_BIT, mask, bits);
	W25Q_CfgBit(reg->write_protect_scheme, W25Q_SR3_WPS_BIT, mask, bits);

	// SR3 [6:5] DRV1..DRV0
	if (reg->drive_strength != W25Q_CFG_KEEP) {
		if (reg->drive_strength > W25Q_CFG_DRV_25) return W25Q_PARAM_ERR;
		*mask |= 0x3UL << W25Q_SR3_DRV0_BIT;
		*bits |= (uint32_t)(reg->drive_strength - 1u) << W25Q_SR3_DRV0_BIT;
	}

	if (reg->sr_write > W25Q_SR_WRITE_VOLATILE) return W25Q_PARAM_ERR;

	return W25Q_OK;
}

W25Q_STATE W25Q_Init(W25Q_t *chip, W25Q_config_t config, uint32_t timeout_ms) {
	if (!chip || !config.bus.hspi || !config.bus.cs_bank) return W25Q_PARAM_ERR;

	W25Q_STATE st;
	uint32_t mask, bits;

	st = W25Q_ConfigToStatus(&config.reg, &mask, &bits);
	if (st != W25Q_OK) return st;

	chip->config = config;
	chip->status_reg = 0;

	// Set CS pin high (if not already set)
	HAL_GPIO_WritePin(config.bus.cs_bank, config.bus.cs_pin, GPIO_PIN_SET);

	// Read and check ID
	uint8_t id_buf[3];
	W25Q_ReadID(chip, id_buf); // Dummy
	st = W25Q_ReadID(chip, id_buf);
	if (st != W25Q_OK) return st;
	if (id_buf[0] != W25Q_MANUFACTURER_ID) return W25Q_CHIP_ERR;
	if (W25Q_V_FULL_DEVICE_ID != (uint32_t)((id_buf[1] << 8) | id_buf[2])) return W25Q_PARAM_ERR;

	// Read the current configuration
	for (uint8_t sr = 1; sr <= 3; sr++) {
		st = W25Q_ReadStatus(chip, sr);
		if (st != W25Q_OK) return st;
	}

	// Write only the registers whose configured bits differ: no needless NV write,
	// bits not covered by the config (SRP, SRL, LB...) are written back unchanged.
	for (uint8_t sr = 1; sr <= 3; sr++) {
		uint8_t shift = (uint8_t)((sr - 1u) * 8u);
		uint8_t cur   = (uint8_t)(chip->status_reg >> shift);
		uint8_t m     = (uint8_t)(mask >> shift);
		uint8_t want  = (uint8_t)((cur & ~m) | ((uint8_t)(bits >> shift) & m));
		if (want != cur) {
			st = W25Q_WriteStatus(chip, sr, want, config.reg.sr_write, timeout_ms);
			if (st != W25Q_OK) return st;
		}
	}

	// Current address mode (ADS): read-only bit, switched by command (ADP only acts at power-up)
	if (config.reg.addr_mode != W25Q_CFG_KEEP) {
		bool want_4b = (config.reg.addr_mode == W25Q_CFG_ADS_4B);
		if (W25Q_STATUS_REG(chip, W25Q_SR3_ADS_BIT) != want_4b) {
			st = W25Q_SendCmd(chip, want_4b ? W25Q_ENABLE_4B_MODE : W25Q_DISABLE_4B_MODE, timeout_ms);
			if (st != W25Q_OK) return st;
		}
		mask |= 1UL << W25Q_SR3_ADS_BIT;
		if (want_4b) bits |= 1UL << W25Q_SR3_ADS_BIT;
	}

	// Verify: a write can be silently refused (SRP / WP pin / lock bits)
	for (uint8_t sr = 1; sr <= 3; sr++) {
		st = W25Q_ReadStatus(chip, sr);
		if (st != W25Q_OK) return st;
	}
	if ((chip->status_reg & mask) != bits) return W25Q_CHIP_ERR;

	return W25Q_OK;
}

// Data size must be <= 256 (page size)
static W25Q_STATE W25Q_PageProgram(W25Q_t *chip, const uint8_t *data, uint32_t addr, uint16_t data_size, uint32_t timeout_ms) {
	W25Q_STATE st;

	st = W25Q_WaitForReady(chip, timeout_ms);
	if (st != W25Q_OK) return st;
	if (!W25Q_STATUS_REG(chip, W25Q_SR1_WEL_BIT)) {
		st = W25Q_SendCmd(chip, W25Q_WRITE_ENABLE, timeout_ms);
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

	st = W25Q_WaitForReady(chip, timeout_ms);
	return st;
}

W25Q_STATE W25Q_WriteData(W25Q_t *chip, const uint8_t *data, uint32_t addr, uint32_t data_size, uint32_t timeout_ms) {
	W25Q_STATE st;

	if (addr >= W25Q_FLASH_SIZE_BYTES) return W25Q_PARAM_ERR;
	if (data_size > (uint32_t)W25Q_FLASH_SIZE_BYTES - addr) data_size = (uint32_t)W25Q_FLASH_SIZE_BYTES - addr;

	while (data_size > 0) {
		uint32_t relative_addr = addr % W25Q_MEM_PAGE_SIZE;
		uint16_t data_size_page = (data_size + relative_addr) > W25Q_MEM_PAGE_SIZE ? W25Q_MEM_PAGE_SIZE - relative_addr : data_size;
		
		st = W25Q_PageProgram(chip, data, addr, data_size_page, timeout_ms);
		if (st != W25Q_OK) return st;

		data_size -= data_size_page;
		addr += data_size_page;
		data += data_size_page;
	}
	return W25Q_OK;
}

// Data size must be <= flash size
W25Q_STATE W25Q_ReadData(W25Q_t *chip, uint8_t *data_buf, uint32_t addr, uint32_t data_size, uint32_t timeout_ms) {
	W25Q_STATE state;

	if (addr >= W25Q_FLASH_SIZE_BYTES) return W25Q_PARAM_ERR;
	if (data_size > (uint32_t)W25Q_FLASH_SIZE_BYTES - addr) data_size = (uint32_t)W25Q_FLASH_SIZE_BYTES - addr;

	state = W25Q_WaitForReady(chip, timeout_ms);
	if (state != W25Q_OK) return state;

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
	// The SPI layer takes a 16-bit length: stream the span in chunks, CS held low.
	while (data_size > 0) {
		uint16_t chunk = (data_size > 0xFFFFU) ? 0xFFFFU : (uint16_t)data_size;
		state = W25Q_SPI_Rx(chip, data_buf, chunk);
		if (state != W25Q_OK) { W25Q_SPI_End(chip); return state; }
		data_buf += chunk;
		data_size -= chunk;
	}
	W25Q_SPI_End(chip);

	return state;
}




