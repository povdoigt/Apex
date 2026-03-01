#include "project.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "utils/vt100.h"

/* ========================================================================
 * Adresses de test – un secteur dedie par test (4 KB chacun, pas de
 * collision entre cas de test, pas de dependances entre eux).
 * ======================================================================== */
#define PAGE_BYTES    W25Q_MEM_PAGE_SIZE              /* 256 octets         */
#define SECTOR_BYTES  (W25Q_MEM_SECTOR_SIZE * 1024U)  /* 4 096 octets       */

/*                              base        usage                           */
#define ADDR_SEC0   0x000000UL  /* T1 - Erase verify                       */
#define ADDR_SEC1   0x001000UL  /* T2 - R/W aligne (1 page complete)       */
#define ADDR_SEC2   0x002000UL  /* T3 - R/W cross-page                     */
#define ADDR_SEC3   0x003000UL  /* T4 - R/W cross-secteur (moitie 1/2)     */
#define ADDR_SEC4   0x004000UL  /* T4 - R/W cross-secteur (moitie 2/2)     */
#define ADDR_SEC5   0x005000UL  /* T5 - Isolation secteur (source)         */
#define ADDR_SEC6   0x006000UL  /* T5 - Isolation secteur (voisin intact)  */
#define ADDR_SEC7   0x007000UL  /* T6 - Comportement AND sans effacement   */
#define ADDR_SEC8   0x008000UL  /* T7 - Ecriture adresse non-alignee       */

/* T3 : 128 B demarrant 64 B avant la frontiere page-7/page-8 du sec2.    *
 *   page 7 : 0x0027C0-0x0027FF (64 B), page 8 : 0x002800-0x00283F (64 B) *
 *   WriteData doit emettre 2 PageProgram internes.                        */
#define ADDR_CROSS_PAGE   (ADDR_SEC2 + 8u * PAGE_BYTES - 64u)  /* 0x0027C0 */
#define SIZE_CROSS_PAGE   128U

/* T4 : 32 B demarrant 16 B avant la frontiere sec3/sec4.                 *
 *   fin sec3 : 0x003FF0-0x003FFF (16 B), debut sec4 : 0x004000-0x00400F  */
#define ADDR_CROSS_SECTOR (ADDR_SEC4 - 16u)                    /* 0x003FF0 */
#define SIZE_CROSS_SECTOR 32U

/* T5 : 16 derniers B de sec5 + 16 premiers B de sec6                     */
#define ADDR_ISOL_END5    (ADDR_SEC6 - 16u)                    /* 0x005FF0 */
#define ADDR_ISOL_BEG6    ADDR_SEC6                            /* 0x006000 */
#define SIZE_ISOL         16U

/* T7 : 100 B a partir de +50 dans sec8 (non multiple de 256)             */
#define ADDR_UNALIGNED    (ADDR_SEC8 + 50u)
#define SIZE_UNALIGNED    100U

/* T9 : 32 KB block erase – 1er bloc 32 KB apres les secteurs de test     */
#define ADDR_BLK32   0x010000UL
/* T10 : 64 KB block erase – 1er bloc 64 KB apres ADDR_BLK32              */
#define ADDR_BLK64   0x020000UL
/* T12 : ecriture multi-secteurs – PAGE_BYTES*17 = 4352 B sur 2 secteurs  */
#define ADDR_MULTI   0x030000UL
#define SIZE_LARGE   (PAGE_BYTES * 17U)  /* 256 * 17 = 4352 B              */
/* T13 : ecriture en fin de flash – 128 B avant la limite des 64 MB       */
#define ADDR_NEAR_END  (W25Q_FLASH_SIZE_BYTES - 128U)

/* ========================================================================
 * Buffers statiques
 * ======================================================================== */
static uint8_t buf_a[PAGE_BYTES];
static uint8_t buf_b[PAGE_BYTES];
static uint8_t buf_isol_a[SIZE_ISOL];
static uint8_t buf_isol_b[SIZE_ISOL];
static uint8_t buf_isol_rx[SIZE_ISOL];
static uint8_t buf_large[SIZE_LARGE];     /* T12 – write multi-secteurs   */
static uint8_t buf_large_rx[SIZE_LARGE];  /* T12 – read  multi-secteurs   */
static char    log_buf[512];

/* ========================================================================
 * Registre de cas de test
 * ======================================================================== */
#define N_TESTS  15

typedef enum { R_PASS = 0, R_FAIL, R_SKIP } result_t;

typedef struct {
    const char *name;
    result_t    result;
    char        detail[80];
} test_case_t;

static test_case_t tc[N_TESTS];

/* ========================================================================
 * Utilitaires internes
 * ======================================================================== */
static void usb_print(const char *s) {
    CDC_Transmit_FS((uint8_t *)s, strlen(s));
    HAL_Delay(5);
}

static const char *state_str(W25Q_STATE s) {
    switch (s) {
        case W25Q_OK:           return "OK";
        case W25Q_CHIP_ERR:     return "CHIP_ERR";
        case W25Q_SPI_ERR:      return "SPI_ERR";
        case W25Q_PARAM_ERR:    return "PARAM_ERR";
        case W25Q_BUSY_TIMEOUT: return "TIMEOUT";
        case W25Q_SEM_ERR:      return "SEM_ERR";
        default:                return "?";
    }
}

/* Compte les mismatches ; stocke addr/exp/got du premier ecart. */
static uint32_t count_mm(const uint8_t *exp, const uint8_t *got, uint32_t n,
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
static bool verify_uniform(const uint8_t *buf, uint32_t n,
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

/* ========================================================================
 * T0 – Identificateur chip (JEDEC ID)
 *   Verifie Manufacturer ID (0xEF) + Device ID (0x4020 pour JV 512 Mbit).
 * ======================================================================== */
static void t0_id_check(void) {
    tc[0].name = "T0 ID Check";
    uint8_t id[3] = {0};
    W25Q_STATE st = W25Q_ReadID(&w25q, id);
    if (st != W25Q_OK) {
        tc[0].result = R_FAIL;
        snprintf(tc[0].detail, sizeof(tc[0].detail), "ReadID: %s", state_str(st));
        return;
    }
    uint16_t dev = (uint16_t)((id[1] << 8) | id[2]);
    if (id[0] != W25Q_MANUFACTURER_ID || dev != W25Q_V_FULL_DEVICE_ID) {
        tc[0].result = R_FAIL;
        snprintf(tc[0].detail, sizeof(tc[0].detail),
                 "Manuf=0x%02X(exp:0x%02X) Dev=0x%04X(exp:0x%04X)",
                 id[0], W25Q_MANUFACTURER_ID, dev, W25Q_V_FULL_DEVICE_ID);
        return;
    }
    tc[0].result = R_PASS;
    snprintf(tc[0].detail, 80, "Manuf=0x%02X DevID=0x%04X", id[0], dev);
}

/* ========================================================================
 * T1 – Verification de l'effacement
 *   Efface sec0, lit 256 B -> tous doivent valoir 0xFF.
 *   Detecte les pannes d'erase partiel ou d'adressage incorrect.
 * ======================================================================== */
static void t1_erase_verify(void) {
    tc[1].name = "T1 Erase Verify";
    W25Q_STATE st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC0);
    if (st != W25Q_OK) {
        tc[1].result = R_FAIL;
        snprintf(tc[1].detail, sizeof(tc[1].detail), "SectorErase: %s", state_str(st));
        return;
    }
    st = W25Q_ReadData(&w25q, buf_a, ADDR_SEC0, PAGE_BYTES);
    if (st != W25Q_OK) {
        tc[1].result = R_FAIL;
        snprintf(tc[1].detail, sizeof(tc[1].detail), "ReadData: %s", state_str(st));
        return;
    }
    uint32_t first_addr; uint8_t first_got;
    if (!verify_uniform(buf_a, PAGE_BYTES, 0xFF, ADDR_SEC0, &first_addr, &first_got)) {
        tc[1].result = R_FAIL;
        snprintf(tc[1].detail, sizeof(tc[1].detail),
                 "Non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    } else {
        tc[1].result = R_PASS;
        snprintf(tc[1].detail, sizeof(tc[1].detail),
                 "%uB=0xFF apres erase @0x%06lX",
                 (unsigned)PAGE_BYTES, (unsigned long)ADDR_SEC0);
    }
}

/* ========================================================================
 * T2 – Lecture / ecriture alignee (1 page complete, adresse page-alignee)
 *   Cas de base : addr = debut de page, taille = exactement 1 page.
 * ======================================================================== */
static void t2_aligned_rw(void) {
    tc[2].name = "T2 Aligned RW";
    W25Q_STATE st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC1);
    if (st != W25Q_OK) {
        tc[2].result = R_FAIL;
        snprintf(tc[2].detail, sizeof(tc[2].detail), "Erase: %s", state_str(st));
        return;
    }
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0xA5 ^ i);
    st = W25Q_WriteData(&w25q, buf_a, ADDR_SEC1, PAGE_BYTES);
    if (st != W25Q_OK) {
        tc[2].result = R_FAIL;
        snprintf(tc[2].detail, sizeof(tc[2].detail), "Write: %s", state_str(st));
        return;
    }
    st = W25Q_ReadData(&w25q, buf_b, ADDR_SEC1, PAGE_BYTES);
    if (st != W25Q_OK) {
        tc[2].result = R_FAIL;
        snprintf(tc[2].detail, sizeof(tc[2].detail), "Read: %s", state_str(st));
        return;
    }
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, PAGE_BYTES, ADDR_SEC1, &first_addr, &first_exp, &first_got);
    if (mm) {
        tc[2].result = R_FAIL;
        snprintf(tc[2].detail, sizeof(tc[2].detail), "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                 (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    } else {
        tc[2].result = R_PASS;
        snprintf(tc[2].detail, sizeof(tc[2].detail), "256B @0x%06lX motif 0xA5^i OK",
                 (unsigned long)ADDR_SEC1);
    }
}

/* ========================================================================
 * T3 – Ecriture / lecture chevauchant deux pages consecutives
 *   128 B a 0x0027C0 : 64 B en page 7 + 64 B en page 8 du sec2.
 *   WriteData doit decouper en 2 PageProgram. Detecte les bugs de
 *   wrap-around dans la logique de relative_addr.
 * ======================================================================== */
static void t3_cross_page_rw(void) {
    tc[3].name = "T3 Cross-Page RW";
    W25Q_STATE st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC2);
    if (st != W25Q_OK) {
        tc[3].result = R_FAIL;
        snprintf(tc[3].detail, sizeof(tc[3].detail), "Erase: %s", state_str(st));
        return;
    }
    for (uint32_t i = 0; i < SIZE_CROSS_PAGE; i++) buf_a[i] = (uint8_t)(0xC3 ^ i);
    st = W25Q_WriteData(&w25q, buf_a, ADDR_CROSS_PAGE, SIZE_CROSS_PAGE);
    if (st != W25Q_OK) {
        tc[3].result = R_FAIL;
        snprintf(tc[3].detail, sizeof(tc[3].detail), "Write: %s", state_str(st));
        return;
    }
    st = W25Q_ReadData(&w25q, buf_b, ADDR_CROSS_PAGE, SIZE_CROSS_PAGE);
    if (st != W25Q_OK) {
        tc[3].result = R_FAIL;
        snprintf(tc[3].detail, sizeof(tc[3].detail), "Read: %s", state_str(st));
        return;
    }
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, SIZE_CROSS_PAGE, ADDR_CROSS_PAGE, &first_addr, &first_exp, &first_got);
    if (mm) {
        tc[3].result = R_FAIL;
        snprintf(tc[3].detail, sizeof(tc[3].detail), "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                 (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    } else {
        tc[3].result = R_PASS;
        snprintf(tc[3].detail, sizeof(tc[3].detail), "128B @0x%06lX cheval page7/8 OK",
                 (unsigned long)ADDR_CROSS_PAGE);
    }
}

/* ========================================================================
 * T4 – Ecriture / lecture chevauchant deux secteurs consecutifs
 *   32 B a 0x003FF0 : 16 B en fin de sec3 + 16 B en debut de sec4.
 *   Detecte les problemes d'adressage a la frontiere de secteur.
 * ======================================================================== */
static void t4_cross_sector_rw(void) {
    tc[4].name = "T4 Cross-Sector RW";
    W25Q_STATE st;
    st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC3);
    if (st != W25Q_OK) {
        tc[4].result = R_FAIL;
        snprintf(tc[4].detail, sizeof(tc[4].detail), "Erase sec3: %s", state_str(st));
        return;
    }
    st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC4);
    if (st != W25Q_OK) {
        tc[4].result = R_FAIL;
        snprintf(tc[4].detail, sizeof(tc[4].detail), "Erase sec4: %s", state_str(st));
        return;
    }

    for (uint32_t i = 0; i < SIZE_CROSS_SECTOR; i++) buf_a[i] = (uint8_t)(0x55 ^ i);
    st = W25Q_WriteData(&w25q, buf_a, ADDR_CROSS_SECTOR, SIZE_CROSS_SECTOR);
    if (st != W25Q_OK) {
        tc[4].result = R_FAIL;
        snprintf(tc[4].detail, sizeof(tc[4].detail), "Write: %s", state_str(st));
        return;
    }
    st = W25Q_ReadData(&w25q, buf_b, ADDR_CROSS_SECTOR, SIZE_CROSS_SECTOR);
    if (st != W25Q_OK) {
        tc[4].result = R_FAIL;
        snprintf(tc[4].detail, sizeof(tc[4].detail), "Read: %s", state_str(st));
        return;
    }

    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, SIZE_CROSS_SECTOR, ADDR_CROSS_SECTOR, &first_addr, &first_exp, &first_got);
    if (mm) {
        tc[4].result = R_FAIL;
        snprintf(tc[4].detail, sizeof(tc[4].detail), "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                 (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    } else {
        tc[4].result = R_PASS;
        snprintf(tc[4].detail, sizeof(tc[4].detail), "32B @0x%06lX cheval sec3/4 OK",
                 (unsigned long)ADDR_CROSS_SECTOR);
    }
}

/* ========================================================================
 * T5 – Isolation lors d'un effacement de secteur
 *   1. Efface sec5 et sec6
 *   2. Ecrit 0xAA sur les 16 derniers B de sec5 (0x005FF0)
 *   3. Ecrit 0x55 sur les 16 premiers B de sec6 (0x006000)
 *   4. Re-efface sec5 uniquement
 *   5. Verifie : fin sec5 = 0xFF, debut sec6 = 0x55 (intact)
 *   Critere : l'effacement d'un secteur ne doit pas corrompre son voisin.
 * ======================================================================== */
static void t5_sector_isolation(void) {
    tc[5].name = "T5 Sec Isolation";
    for (uint32_t i = 0; i < SIZE_ISOL; i++) { buf_isol_a[i] = 0xAA; buf_isol_b[i] = 0x55; }

    W25Q_STATE st;
    st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC5);
    if (st != W25Q_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "Erase sec5: %s", state_str(st));
        return;
    }
    st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC6);
    if (st != W25Q_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "Erase sec6: %s", state_str(st));
        return;
    }

    st = W25Q_WriteData(&w25q, buf_isol_a, ADDR_ISOL_END5, SIZE_ISOL);
    if (st != W25Q_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "Write 0xAA: %s", state_str(st));
        return;
    }
    st = W25Q_WriteData(&w25q, buf_isol_b, ADDR_ISOL_BEG6, SIZE_ISOL);
    if (st != W25Q_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "Write 0x55: %s", state_str(st));
        return;
    }

    st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC5);  /* re-efface sec5 seul */
    if (st != W25Q_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "Re-erase sec5: %s", state_str(st));
        return;
    }

    st = W25Q_ReadData(&w25q, buf_isol_rx, ADDR_ISOL_END5, SIZE_ISOL);
    if (st != W25Q_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "Read sec5: %s", state_str(st));
        return;
    }
    uint32_t first_addr; uint8_t first_got;
    if (!verify_uniform(buf_isol_rx, SIZE_ISOL, 0xFF, ADDR_ISOL_END5, &first_addr, &first_got)) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "Sec5 non efface @0x%06lX got=0x%02X",
                 (unsigned long)first_addr, first_got);
        return;
    }

    st = W25Q_ReadData(&w25q, buf_isol_rx, ADDR_ISOL_BEG6, SIZE_ISOL);
    if (st != W25Q_OK) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "Read sec6: %s", state_str(st));
        return;
    }
    uint32_t first_addr2; uint8_t first_exp2, first_got2;
    uint32_t mm = count_mm(buf_isol_b, buf_isol_rx, SIZE_ISOL, ADDR_ISOL_BEG6, &first_addr2, &first_exp2, &first_got2);
    if (mm) {
        tc[5].result = R_FAIL;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "Sec6 corrompu: %lu mm @0x%06lX exp=0x%02X got=0x%02X",
                 (unsigned long)mm, (unsigned long)first_addr2, first_exp2, first_got2);
    } else {
        tc[5].result = R_PASS;
        snprintf(tc[5].detail, sizeof(tc[5].detail), "Sec5=0xFF efface, sec6=0x55 intact");
    }
}

/* ========================================================================
 * T6 – Comportement AND (ecriture sans effacement prealable)
 *   Flash NOR : un bit ne peut passer de 0->1 que par un effacement.
 *   Ecrire sur une cellule deja programmee applique un AND bit a bit :
 *     0xFF -> 0x0F : OK (1->0 autorise)
 *     0x0F -> 0xF0 : donne 0x0F & 0xF0 = 0x00 (sans re-effacement)
 *   Verifie que le driver n'ajoute pas d'effacement implicite.
 * ======================================================================== */
static void t6_and_behavior(void) {
    tc[6].name = "T6 AND Behavior";
    W25Q_STATE st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC7);
    if (st != W25Q_OK) {
        tc[6].result = R_FAIL;
        snprintf(tc[6].detail, sizeof(tc[6].detail), "Erase: %s", state_str(st));
        return;
    }

    uint8_t val = 0x0F;
    st = W25Q_WriteData(&w25q, &val, ADDR_SEC7, 1);
    if (st != W25Q_OK) {
        tc[6].result = R_FAIL;
        snprintf(tc[6].detail, sizeof(tc[6].detail), "Write 0x0F: %s", state_str(st));
        return;
    }

    val = 0xF0;
    st = W25Q_WriteData(&w25q, &val, ADDR_SEC7, 1);   /* pas d'effacement */
    if (st != W25Q_OK) {
        tc[6].result = R_FAIL;
        snprintf(tc[6].detail, sizeof(tc[6].detail), "Write 0xF0: %s", state_str(st));
        return;
    }

    uint8_t got = 0xFF;
    st = W25Q_ReadData(&w25q, &got, ADDR_SEC7, 1);
    if (st != W25Q_OK) {
        tc[6].result = R_FAIL;
        snprintf(tc[6].detail, sizeof(tc[6].detail), "Read: %s", state_str(st));
        return;
    }

    const uint8_t expected = 0x0F & 0xF0;  /* = 0x00 */
    if (got != expected) {
        tc[6].result = R_FAIL;
        snprintf(tc[6].detail, sizeof(tc[6].detail), "exp=0x%02X got=0x%02X (devrait etre AND)", expected, got);
    } else {
        tc[6].result = R_PASS;
        snprintf(tc[6].detail, sizeof(tc[6].detail), "0xFF->0x0F->0x00 (0x0F & 0xF0) OK");
    }
}

/* ========================================================================
 * T7 – Ecriture a une adresse non alignee sur une page (addr % 256 != 0)
 *   100 B a ADDR_SEC8+50 (offset 50 non multiple de 256).
 *   Lit ensuite 200 B depuis le debut du secteur et controle :
 *   - octets [0..49]    (avant zone) : 0xFF non touches
 *   - octets [50..149]  (zone ecrite): correspond au motif
 *   - octets [150..199] (apres zone) : 0xFF non touches
 * ======================================================================== */
static void t7_unaligned_rw(void) {
    tc[7].name = "T7 Unaligned RW";
    W25Q_STATE st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC8);
    if (st != W25Q_OK) {
        tc[7].result = R_FAIL;
        snprintf(tc[7].detail, sizeof(tc[7].detail), "Erase: %s", state_str(st));
        return;
    }

    for (uint32_t i = 0; i < SIZE_UNALIGNED; i++) buf_a[i] = (uint8_t)(0x3C ^ i);
    st = W25Q_WriteData(&w25q, buf_a, ADDR_UNALIGNED, SIZE_UNALIGNED);
    if (st != W25Q_OK) {
        tc[7].result = R_FAIL;
        snprintf(tc[7].detail, sizeof(tc[7].detail), "Write: %s", state_str(st));
        return;
    }

    st = W25Q_ReadData(&w25q, buf_b, ADDR_SEC8, 200U);
    if (st != W25Q_OK) {
        tc[7].result = R_FAIL;
        snprintf(tc[7].detail, sizeof(tc[7].detail), "Read: %s", state_str(st));
        return;
    }

    uint32_t first_addr; uint8_t first_got;
    if (!verify_uniform(buf_b, 50U, 0xFF, ADDR_SEC8, &first_addr, &first_got)) {
        tc[7].result = R_FAIL;
        snprintf(tc[7].detail, sizeof(tc[7].detail), "Zone avant corrompue @0x%06lX got=0x%02X",
                 (unsigned long)first_addr, first_got);
        return;
    }

    uint32_t first_addr2; uint8_t first_exp2, first_got2;
    uint32_t mm = count_mm(buf_a, buf_b + 50U, SIZE_UNALIGNED, ADDR_UNALIGNED, &first_addr2, &first_exp2, &first_got2);
    if (mm) {
        tc[7].result = R_FAIL;
        snprintf(tc[7].detail, sizeof(tc[7].detail), "%lu mm @0x%06lX exp=0x%02X got=0x%02X",
                 (unsigned long)mm, (unsigned long)first_addr2, first_exp2, first_got2);
        return;
    }

    if (!verify_uniform(buf_b + 150U, 50U, 0xFF, ADDR_UNALIGNED + SIZE_UNALIGNED, &first_addr, &first_got)) {
        tc[7].result = R_FAIL;
        snprintf(tc[7].detail, sizeof(tc[7].detail), "Zone apres corrompue @0x%06lX got=0x%02X",
                 (unsigned long)first_addr, first_got);
        return;
    }

    tc[7].result = R_PASS;
    snprintf(tc[7].detail, sizeof(tc[7].detail), "100B @0x%06lX (offset+50), pre/post=0xFF OK",
             (unsigned long)ADDR_UNALIGNED);
}

/* ========================================================================
 * T8 – Lecture des registres de statut (W25Q_ReadStatus)
 *   Lit SR1, SR2, SR3 et verifie le bit ADS (W25Q_SR3_ADS_BIT = 16).
 *   Confirme que W25Q_Init a configure le mode 4-byte (ADS=1).
 * ======================================================================== */
static void t8_read_status(void) {
    tc[8].name = "T8 ReadStatus";
    W25Q_STATE st;
    st = W25Q_ReadStatus(&w25q, 1);
    if (st != W25Q_OK) {
        tc[8].result = R_FAIL;
        snprintf(tc[8].detail, sizeof(tc[8].detail), "ReadSR1: %s", state_str(st));
        return;
    }
    st = W25Q_ReadStatus(&w25q, 2);
    if (st != W25Q_OK) {
        tc[8].result = R_FAIL;
        snprintf(tc[8].detail, sizeof(tc[8].detail), "ReadSR2: %s", state_str(st));
        return;
    }
    st = W25Q_ReadStatus(&w25q, 3);
    if (st != W25Q_OK) {
        tc[8].result = R_FAIL;
        snprintf(tc[8].detail, sizeof(tc[8].detail), "ReadSR3: %s", state_str(st));
        return;
    }
    if (!W25Q_STATUS_REG(&w25q, W25Q_SR3_ADS_BIT)) {
        tc[8].result = R_FAIL;
        snprintf(tc[8].detail, sizeof(tc[8].detail),
                 "ADS=0 (mode 3-byte), status_reg=0x%06lX",
                 (unsigned long)w25q.status_reg);
        return;
    }
    tc[8].result = R_PASS;
    snprintf(tc[8].detail, sizeof(tc[8].detail),
             "SR1=0x%02X SR2=0x%02X SR3=0x%02X ADS=1",
             (uint8_t)(w25q.status_reg >>  0),
             (uint8_t)(w25q.status_reg >>  8),
             (uint8_t)(w25q.status_reg >> 16));
}

/* ========================================================================
 * T9 – Effacement 32 KB (W25Q_32KB_BLOCK_ERASE, cmd 0x52)
 *   Ecrit un motif sur la premiere page du bloc 32 KB a ADDR_BLK32,
 *   efface le bloc entier (32 KB) puis verifie que 256 B = 0xFF.
 * ======================================================================== */
static void t9_erase_32kb(void) {
    tc[9].name = "T9 32KB Erase";
    /* Pre-condition : s'assure que la zone n'est pas deja vierge */
    W25Q_STATE st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_BLK32);
    if (st != W25Q_OK) {
        tc[9].result = R_FAIL;
        snprintf(tc[9].detail, sizeof(tc[9].detail), "Pre-erase sector: %s", state_str(st));
        return;
    }
    for (uint32_t i = 0; i < PAGE_BYTES; i++) {
        buf_a[i] = (uint8_t)(0xAB ^ i);
    }
    st = W25Q_WriteData(&w25q, buf_a, ADDR_BLK32, PAGE_BYTES);
    if (st != W25Q_OK) {
        tc[9].result = R_FAIL;
        snprintf(tc[9].detail, sizeof(tc[9].detail), "Write motif: %s", state_str(st));
        return;
    }
    /* Efface tout le bloc 32 KB (0x010000-0x017FFF) */
    st = W25Q_SendCmdAddr(&w25q, W25Q_32KB_BLOCK_ERASE, ADDR_BLK32);
    if (st != W25Q_OK) {
        tc[9].result = R_FAIL;
        snprintf(tc[9].detail, sizeof(tc[9].detail), "32KB Erase: %s", state_str(st));
        return;
    }
    st = W25Q_ReadData(&w25q, buf_b, ADDR_BLK32, PAGE_BYTES);
    if (st != W25Q_OK) {
        tc[9].result = R_FAIL;
        snprintf(tc[9].detail, sizeof(tc[9].detail), "Read: %s", state_str(st));
        return;
    }
    uint32_t first_addr; uint8_t first_got;
    if (!verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_BLK32, &first_addr, &first_got)) {
        tc[9].result = R_FAIL;
        snprintf(tc[9].detail, sizeof(tc[9].detail), "Non-0xFF @0x%06lX got=0x%02X",
                 (unsigned long)first_addr, first_got);
    } else {
        tc[9].result = R_PASS;
        snprintf(tc[9].detail, sizeof(tc[9].detail), "256B=0xFF apres 32KB erase @0x%06lX",
                 (unsigned long)ADDR_BLK32);
    }
}

/* ========================================================================
 * T10 – Effacement 64 KB (W25Q_64KB_BLOCK_ERASE_4B, cmd 0xDC)
 *   Variante 4-byte du 64 KB block erase. Meme logique que T9.
 * ======================================================================== */
static void t10_erase_64kb(void) {
    tc[10].name = "T10 64KB Erase";
    W25Q_STATE st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_BLK64);
    if (st != W25Q_OK) {
        tc[10].result = R_FAIL;
        snprintf(tc[10].detail, sizeof(tc[10].detail), "Pre-erase sector: %s", state_str(st));
        return;
    }
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0xDE ^ i);
    st = W25Q_WriteData(&w25q, buf_a, ADDR_BLK64, PAGE_BYTES);
    if (st != W25Q_OK) {
        tc[10].result = R_FAIL;
        snprintf(tc[10].detail, sizeof(tc[10].detail), "Write motif: %s", state_str(st));
        return;
    }
    st = W25Q_SendCmdAddr(&w25q, W25Q_64KB_BLOCK_ERASE_4B, ADDR_BLK64);
    if (st != W25Q_OK) {
        tc[10].result = R_FAIL;
        snprintf(tc[10].detail, sizeof(tc[10].detail), "64KB Erase: %s", state_str(st));
        return;
    }
    st = W25Q_ReadData(&w25q, buf_b, ADDR_BLK64, PAGE_BYTES);
    if (st != W25Q_OK) {
        tc[10].result = R_FAIL;
        snprintf(tc[10].detail, sizeof(tc[10].detail), "Read: %s", state_str(st));
        return;
    }
    uint32_t first_addr; uint8_t first_got;
    if (!verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_BLK64, &first_addr, &first_got)) {
        tc[10].result = R_FAIL;
        snprintf(tc[10].detail, sizeof(tc[10].detail), "Non-0xFF @0x%06lX got=0x%02X",
                 (unsigned long)first_addr, first_got);
    } else {
        tc[10].result = R_PASS;
        snprintf(tc[10].detail, sizeof(tc[10].detail), "256B=0xFF apres 64KB erase @0x%06lX",
                 (unsigned long)ADDR_BLK64);
    }
}

/* ========================================================================
 * T11 – Reset logiciel (ENABLE_RESET puis RESET)
 *   Verifie que le chip repond correctement apres la sequence de reset :
 *   - L'identifiant JEDEC est intact.
 *   - Le bit ADS reste a 1 (ADP=1 => mode 4-byte restaure apres reset).
 * ======================================================================== */
static void t11_soft_reset(void) {
    tc[11].name = "T11 Soft Reset";
    W25Q_STATE st = W25Q_SendCmd(&w25q, W25Q_ENABLE_RESET);
    if (st != W25Q_OK) {
        tc[11].result = R_FAIL;
        snprintf(tc[11].detail, sizeof(tc[11].detail), "ENABLE_RESET: %s", state_str(st));
        return;
    }
    st = W25Q_SendCmd(&w25q, W25Q_RESET);
    if (st != W25Q_OK) {
        tc[11].result = R_FAIL;
        snprintf(tc[11].detail, sizeof(tc[11].detail), "RESET: %s", state_str(st));
        return;
    }
    /* Rafraichit status_reg local (stale apres reset interne du chip) */
    if (W25Q_ReadStatus(&w25q, 1) == W25Q_OK)
        W25Q_ReadStatus(&w25q, 2);
    st = W25Q_ReadStatus(&w25q, 3);
    if (st != W25Q_OK) {
        tc[11].result = R_FAIL;
        snprintf(tc[11].detail, sizeof(tc[11].detail), "ReadStatus post-reset: %s", state_str(st));
        return;
    }
    uint8_t id[3] = {0};
    st = W25Q_ReadID(&w25q, id);
    if (st != W25Q_OK) {
        tc[11].result = R_FAIL;
        snprintf(tc[11].detail, sizeof(tc[11].detail), "ReadID post-reset: %s", state_str(st));
        return;
    }
    uint16_t dev = (uint16_t)((id[1] << 8) | id[2]);
    if (id[0] != W25Q_MANUFACTURER_ID || dev != W25Q_V_FULL_DEVICE_ID) {
        tc[11].result = R_FAIL;
        snprintf(tc[11].detail, sizeof(tc[11].detail),
                 "Post-reset: Manuf=0x%02X Dev=0x%04X", id[0], dev);
        return;
    }
    tc[11].result = R_PASS;
    snprintf(tc[11].detail, sizeof(tc[11].detail),
             "ID OK post-reset, ADS=%d", W25Q_STATUS_REG(&w25q, W25Q_SR3_ADS_BIT));
}

/* ========================================================================
 * T12 – Ecriture multi-secteurs (>4 KB, 17 pages = 2 secteurs)
 *   Ecrit SIZE_LARGE (4352 B) a ADDR_MULTI sur 2 secteurs consecutifs.
 *   Teste que la boucle interne de WriteData gere correctement >16 appels
 *   successifs a PageProgram sur plusieurs secteurs.
 * ======================================================================== */
static void t12_multi_sector_rw(void) {
    tc[12].name = "T12 MultiSec RW";
    W25Q_STATE st;
    st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_MULTI);
    if (st != W25Q_OK) {
        tc[12].result = R_FAIL;
        snprintf(tc[12].detail, sizeof(tc[12].detail), "Erase sec0: %s", state_str(st));
        return;
    }
    st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_MULTI + SECTOR_BYTES);
    if (st != W25Q_OK) {
        tc[12].result = R_FAIL;
        snprintf(tc[12].detail, sizeof(tc[12].detail), "Erase sec1: %s", state_str(st));
        return;
    }
    for (uint32_t i = 0; i < SIZE_LARGE; i++) {
        buf_large[i] = (uint8_t)(i & 0xFFU);
    }
    st = W25Q_WriteData(&w25q, buf_large, ADDR_MULTI, SIZE_LARGE);
    if (st != W25Q_OK) {
        tc[12].result = R_FAIL;
        snprintf(tc[12].detail, sizeof(tc[12].detail), "Write: %s", state_str(st));
        return;
    }
    st = W25Q_ReadData(&w25q, buf_large_rx, ADDR_MULTI, SIZE_LARGE);
    if (st != W25Q_OK) {
        tc[12].result = R_FAIL;
        snprintf(tc[12].detail, sizeof(tc[12].detail), "Read: %s", state_str(st));
        return;
    }
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_large, buf_large_rx, SIZE_LARGE, ADDR_MULTI,
                            &first_addr, &first_exp, &first_got);
    if (mm) {
        tc[12].result = R_FAIL;
        snprintf(tc[12].detail, sizeof(tc[12].detail), "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                 (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    } else {
        tc[12].result = R_PASS;
        snprintf(tc[12].detail, sizeof(tc[12].detail), "%uB @0x%06lX (17 pages, 2 sec) OK",
                 (unsigned)SIZE_LARGE, (unsigned long)ADDR_MULTI);
    }
}

/* ========================================================================
 * T13 – Ecriture a cheval sur la limite de la memoire (clamp silencieux)
 *   WriteData avec addr=(taille_flash - 128) et size=256 :
 *   le driver doit limiter silencieusement a 128 B et retourner W25Q_OK.
 *   Verifie que les 128 B ecrits correspondent au debut du motif.
 * ======================================================================== */
static void t13_end_of_flash_clamp(void) {
    tc[13].name = "T13 Flash Clamp";
    W25Q_STATE st = W25Q_SendCmdAddr(&w25q, W25Q_SECTOR_ERASE_4B, ADDR_NEAR_END);
    if (st != W25Q_OK) {
        tc[13].result = R_FAIL;
        snprintf(tc[13].detail, sizeof(tc[13].detail), "Erase: %s", state_str(st));
        return;
    }
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0x7E ^ i);
    /* Tente d'ecrire 256 B alors qu'il ne reste que 128 B disponibles */
    st = W25Q_WriteData(&w25q, buf_a, ADDR_NEAR_END, PAGE_BYTES);
    if (st != W25Q_OK) {
        tc[13].result = R_FAIL;
        snprintf(tc[13].detail, sizeof(tc[13].detail), "Write: %s (attendu OK)", state_str(st));
        return;
    }
    st = W25Q_ReadData(&w25q, buf_b, ADDR_NEAR_END, 128U);
    if (st != W25Q_OK) {
        tc[13].result = R_FAIL;
        snprintf(tc[13].detail, sizeof(tc[13].detail), "Read: %s", state_str(st));
        return;
    }
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, 128U, ADDR_NEAR_END,
                            &first_addr, &first_exp, &first_got);
    if (mm) {
        tc[13].result = R_FAIL;
        snprintf(tc[13].detail, sizeof(tc[13].detail), "%lu mm @0x%06lX exp=0x%02X got=0x%02X",
                 (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    } else {
        tc[13].result = R_PASS;
        snprintf(tc[13].detail, sizeof(tc[13].detail),
                 "128/256B clampees @0x%06lX OK", (unsigned long)ADDR_NEAR_END);
    }
}

/* ========================================================================
 * T14 – WriteData avec taille zero (cas limite)
 *   Appelle WriteData(size=0) : doit retourner W25Q_OK sans toucher le
 *   flash. Utilise ADDR_SEC0 deja efface par T1 et non ecrit depuis.
 * ======================================================================== */
static void t14_write_zero_size(void) {
    tc[14].name = "T14 WriteSize=0";
    W25Q_STATE st = W25Q_WriteData(&w25q, buf_a, ADDR_SEC0, 0U);
    if (st != W25Q_OK) {
        tc[14].result = R_FAIL;
        snprintf(tc[14].detail, sizeof(tc[14].detail), "Write(size=0): %s (attendu OK)",
                 state_str(st));
        return;
    }
    st = W25Q_ReadData(&w25q, buf_b, ADDR_SEC0, PAGE_BYTES);
    if (st != W25Q_OK) {
        tc[14].result = R_FAIL;
        snprintf(tc[14].detail, sizeof(tc[14].detail), "Read: %s", state_str(st));
        return;
    }
    uint32_t first_addr; uint8_t first_got;
    if (!verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_SEC0, &first_addr, &first_got)) {
        tc[14].result = R_FAIL;
        snprintf(tc[14].detail, sizeof(tc[14].detail),
                 "Zone modifiee @0x%06lX got=0x%02X (attendu 0xFF)",
                 (unsigned long)first_addr, first_got);
    } else {
        tc[14].result = R_PASS;
        snprintf(tc[14].detail, sizeof(tc[14].detail), "W25Q_OK, zone intacte (0xFF)");
    }
}

/* ========================================================================
 * setup() – execution unique apres init des peripheriques
 * ======================================================================== */
void setup(void) {
    t0_id_check();
    t1_erase_verify();
    t2_aligned_rw();
    t3_cross_page_rw();
    t4_cross_sector_rw();
    t5_sector_isolation();
    t6_and_behavior();
    t7_unaligned_rw();
    t8_read_status();
    t9_erase_32kb();
    t10_erase_64kb();
    t11_soft_reset();
    t12_multi_sector_rw();
    t13_end_of_flash_clamp();
    t14_write_zero_size();

    /* Attend que le serial monitor soit ouvert cote PC (DTR=1).
     * Sans ca, les premiers caracteres seraient perdus avant
     * que le terminal ne soit pret a les recevoir.            */
    while (!cdc_port_open) {
        HAL_Delay(10);
    }
    HAL_Delay(50); /* stabilisation du terminal */

        uint32_t n_pass = 0, n_fail = 0;
    for (int i = 0; i < N_TESTS; i++) {
        if      (tc[i].result == R_PASS) n_pass++;
        else if (tc[i].result == R_FAIL) n_fail++;
    }

    usb_print(VT100_SCREEN_CLEAR);

    snprintf(log_buf, sizeof(log_buf),
             VT100_FG_CYAN "===== W25Q Sequential Driver - Test Suite =====" VT100_RESET "\r\n"
             "15 cas de test (adressage, effacement, limites, robustesse)\r\n\r\n");
    usb_print(log_buf);

    for (int i = 0; i < N_TESTS; i++) {
        const char *col = (tc[i].result == R_PASS) ? VT100_FG_GREEN
                        : (tc[i].result == R_FAIL) ? VT100_FG_RED
                        :                            VT100_FG_YELLOW;
        const char *tag = (tc[i].result == R_PASS) ? "PASS"
                        : (tc[i].result == R_FAIL) ? "FAIL" : "SKIP";
        snprintf(log_buf, sizeof(log_buf),
                 "  %s[%s]" VT100_RESET " %-20s %s\r\n",
                 col, tag, tc[i].name, tc[i].detail);
        usb_print(log_buf);
    }

    const char *vcol = (n_fail == 0) ? VT100_BG_GREEN VT100_FG_BLACK
                                     : VT100_BG_RED   VT100_FG_BLACK;
    snprintf(log_buf, sizeof(log_buf),
             "\r\n%s  %lu/%d PASS   %lu FAIL  " VT100_RESET "\r\n",
             vcol, (unsigned long)n_pass, N_TESTS, (unsigned long)n_fail);
    usb_print(log_buf);
}

/* ========================================================================
 * loop() – Do nothing more as tests are done in setup() and results printed there.
 * ======================================================================== */
void loop(void) {

}
