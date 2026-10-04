#ifndef W25Q_TEST_COMMON_H
#define W25Q_TEST_COMMON_H

/*
 * Elements partages par les suites sequentielle (w25q_seq_test) et RTOS
 * (w25q_rtos_test) : carte des zones flash, constantes et verifications.
 *
 * Les deux suites portent la meme numerotation T0-T26, utilisent les memes
 * zones et les memes motifs : un test RTOS qui echoue alors que son jumeau
 * sequentiel passe designe la couche RTOS (DMA, semaphore, taches).
 *
 * A n'inclure que depuis un fichier de test.
 */

#include "w25q.h"

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================
 * Adresses de test – une zone dediee par test (pas de collision entre cas
 * de test, pas de dependances entre eux).
 *
 * Plage reservee aux tests communs : 0x000000 - 0x03FFFF, plus la zone
 * >16 MB et le dernier secteur. Les tests propres a une suite prennent
 * leurs zones a partir de 0x040000.
 * ======================================================================== */
#define PAGE_BYTES    W25Q_MEM_PAGE_SIZE              /* 256 octets         */
#define SECTOR_BYTES  (W25Q_MEM_SECTOR_SIZE * 1024U)  /* 4 096 octets       */

/*                              base        usage                           */
#define ADDR_SEC0   0x000000UL  /* T9  - Erase verify                      */
#define ADDR_SEC1   0x001000UL  /* T10 - R/W aligne (1 page complete)      */
#define ADDR_SEC2   0x002000UL  /* T11 - R/W cross-page                    */
#define ADDR_SEC3   0x003000UL  /* T12 - R/W cross-secteur (moitie 1/2)    */
#define ADDR_SEC4   0x004000UL  /* T12 - R/W cross-secteur (moitie 2/2)    */
#define ADDR_SEC5   0x005000UL  /* T20 - Isolation secteur (source)        */
#define ADDR_SEC6   0x006000UL  /* T20 - Isolation secteur (voisin intact) */
#define ADDR_SEC7   0x007000UL  /* T15 - Comportement AND sans effacement  */
#define ADDR_SEC8   0x008000UL  /* T13 - Ecriture adresse non-alignee      */
#define ADDR_SEC9   0x009000UL  /* T22 - WriteData taille zero             */
#define ADDR_SEC10  0x00A000UL  /* T21 - Timeout BUSY                      */
#define ADDR_SEC11  0x00B000UL  /* T17 - Erase 20h en mode 3 octets        */
#define ADDR_SEC12  0x00C000UL  /* T17 - Erase 20h en mode 4 octets        */

/* T11 : 128 B demarrant 64 B avant la frontiere page-7/page-8 du sec2.   *
 *   page 7 : 0x0027C0-0x0027FF (64 B), page 8 : 0x002800-0x00283F (64 B) *
 *   WriteData doit emettre 2 PageProgram internes.                        */
#define ADDR_CROSS_PAGE   (ADDR_SEC2 + 8u * PAGE_BYTES - 64u)  /* 0x0027C0 */
#define SIZE_CROSS_PAGE   128U

/* T12 : 32 B demarrant 16 B avant la frontiere sec3/sec4.                *
 *   fin sec3 : 0x003FF0-0x003FFF (16 B), debut sec4 : 0x004000-0x00400F  */
#define ADDR_CROSS_SECTOR (ADDR_SEC4 - 16u)                    /* 0x003FF0 */
#define SIZE_CROSS_SECTOR 32U

/* T20 : 16 derniers B de sec5 + 16 premiers B de sec6                    */
#define ADDR_ISOL_END5    (ADDR_SEC6 - 16u)                    /* 0x005FF0 */
#define ADDR_ISOL_BEG6    ADDR_SEC6                            /* 0x006000 */
#define SIZE_ISOL         16U

/* T13 : 100 B a partir de +50 dans sec8 (non multiple de 256)            */
#define ADDR_UNALIGNED    (ADDR_SEC8 + 50u)
#define SIZE_UNALIGNED    100U

/* T18 : 32 KB block erase – 1er bloc 32 KB apres les secteurs de test    */
#define ADDR_BLK32   0x010000UL
/* T19 : 64 KB block erase – 1er bloc 64 KB apres ADDR_BLK32              */
#define ADDR_BLK64   0x020000UL
/* T14 : ecriture multi-secteurs – PAGE_BYTES*17 = 4352 B sur 2 secteurs  */
#define ADDR_MULTI   0x030000UL
#define SIZE_LARGE   (PAGE_BYTES * 17U)  /* 256 * 17 = 4352 B              */
/* T23/T24 : fin de flash – 128 B avant la limite des 64 MB               */
#define ADDR_NEAR_END  (W25Q_FLASH_SIZE_BYTES - 128U)
/* T24 : valeur sentinelle pour detecter un debordement du buffer de lecture */
#define SENTINEL       0x5AU
/* T8/T16 : zone au-dela de 16 MB (inaccessible en adressage 3 octets)    */
#define ADDR_ABOVE_16MB  0x01000000UL
#define SIZE_ADDR_MODE   32U

/* Bits de SR1 qui evoluent seuls (BUSY, WEL), exclus des comparaisons    */
#define SR_VOLATILE_MASK ((1UL << W25Q_SR1_BUSY_BIT) | (1UL << W25Q_SR1_WEL_BIT))
/* Attente max BUSY par appel : couvre le block erase 64 KB (max 2 s datasheet) */
#define W25Q_TEST_TIMEOUT_MS  3000U

/* ========================================================================
 * Verifications
 * ======================================================================== */

static inline const char *state_str(W25Q_STATE s) {
    switch (s) {
        case W25Q_OK:           return "OK";
        case W25Q_CHIP_ERR:     return "CHIP_ERR";
        case W25Q_SPI_ERR:      return "SPI_ERR";
        case W25Q_PARAM_ERR:    return "PARAM_ERR";
        case W25Q_BUSY_TIMEOUT: return "TIMEOUT";
        case W25Q_SEM_ERR:      return "SEM_ERR";
        case W25Q_LOCK_TIMEOUT: return "LOCK_TIMEOUT";
        default:                return "?";
    }
}

/* Compte les mismatches ; stocke addr/exp/got du premier ecart. */
static inline uint32_t count_mm(const uint8_t *exp, const uint8_t *got, uint32_t n,
                                uint32_t base,
                                uint32_t *first_addr, uint8_t *first_exp, uint8_t *first_got) {
    uint32_t mm = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (exp[i] != got[i]) {
            if (!mm) { *first_addr = base + i; *first_exp = exp[i]; *first_got = got[i]; }
            mm++;
        }
    }
    return mm;
}

/* Verifie que tous les octets valent expected_byte. */
static inline bool verify_uniform(const uint8_t *buf, uint32_t n,
                                  uint8_t expected_byte, uint32_t base,
                                  uint32_t *first_addr, uint8_t *first_got) {
    for (uint32_t i = 0; i < n; i++) {
        if (buf[i] != expected_byte) {
            *first_addr = base + i; *first_got = buf[i];
            return false;
        }
    }
    return true;
}

#endif /* W25Q_TEST_COMMON_H */
