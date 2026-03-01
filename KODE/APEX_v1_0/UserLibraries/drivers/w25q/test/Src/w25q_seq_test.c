#include "w25q_seq_test.h"

#include <stdio.h>

// Pointer need to be set
static W25Q_t *w25q = NULL;

void W25Q_seq_test_set_context(W25Q_t *w25q_ctx) {
    w25q = w25q_ctx;
}

TEST_case_table_t W25Q_seq_test_cases[W25Q_seq_test_N_TESTS] = {
    { .case_info = { .name = "T0 ID Check" }           , .func = W25Q_seq_test_t0_id_check            },
    { .case_info = { .name = "T1 Erase Verify" }       , .func = W25Q_seq_test_t1_erase_verify        },
    { .case_info = { .name = "T2 Aligned R/W" }        , .func = W25Q_seq_test_t2_aligned_rw          },
    { .case_info = { .name = "T3 Cross-page R/W" }     , .func = W25Q_seq_test_t3_cross_page_rw       },
    { .case_info = { .name = "T4 Cross-sector R/W" }   , .func = W25Q_seq_test_t4_cross_sector_rw     },
    { .case_info = { .name = "T5 Sector Isolation" }   , .func = W25Q_seq_test_t5_sector_isolation    },
    { .case_info = { .name = "T6 AND without Erase" }  , .func = W25Q_seq_test_t6_and_behavior        },
    { .case_info = { .name = "T7 Unaligned Write" }    , .func = W25Q_seq_test_t7_unaligned_rw        },
    { .case_info = { .name = "T8 Block Erase 32 KB" }  , .func = W25Q_seq_test_t8_read_status         },
    { .case_info = { .name = "T9 Block Erase 64 KB" }  , .func = W25Q_seq_test_t9_erase_32kb          },
    { .case_info = { .name = "T10 Multi-sector Write" }, .func = W25Q_seq_test_t10_erase_64kb         },
    { .case_info = { .name = "T11 Multi-sector Read" } , .func = W25Q_seq_test_t11_soft_reset         },
    { .case_info = { .name = "T12 Write Near End" }    , .func = W25Q_seq_test_t12_multi_sector_rw    },
    { .case_info = { .name = "T13 Read Near End" }     , .func = W25Q_seq_test_t13_end_of_flash_clamp },
    { .case_info = { .name = "T14 Write Size=0" }      , .func = W25Q_seq_test_t14_write_zero_size    }
};


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

/* ========================================================================
 * Utilitaires internes
 * ======================================================================== */

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



void W25Q_seq_test_t0_id_check(TEST_case_t *tc) {
    uint8_t id[3] = {0};
    W25Q_STATE st = W25Q_ReadID(w25q, id);
    TEST_ASSERT(st == W25Q_OK, "ReadID: %s", state_str(st));

    uint16_t dev = (uint16_t)((id[1] << 8) | id[2]);
    TEST_ASSERT(id[0] == W25Q_MANUFACTURER_ID, "Manuf=0x%02X != 0x%02X", id[0], W25Q_MANUFACTURER_ID);
    TEST_ASSERT(dev == W25Q_V_FULL_DEVICE_ID, "Dev=0x%04X != 0x%04X", dev, W25Q_V_FULL_DEVICE_ID);

    tc->result = R_PASS;
    snprintf(tc->detail, 80, "Manuf=0x%02X DevID=0x%04X", id[0], dev);
}

void W25Q_seq_test_t1_erase_verify(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC0);
    TEST_ASSERT(st == W25Q_OK, "SectorErase: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_a, ADDR_SEC0, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "ReadData: %s", state_str(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_a, PAGE_BYTES, 0xFF, ADDR_SEC0, &first_addr, &first_got),
                "Non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "%uB=0xFF apres erase @0x%06lX",
             (unsigned)PAGE_BYTES, (unsigned long)ADDR_SEC0);
}

void W25Q_seq_test_t2_aligned_rw(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC1);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0xA5 ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_SEC1, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_SEC1, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, PAGE_BYTES, ADDR_SEC1, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "256B @0x%06lX motif 0xA5^i OK",
             (unsigned long)ADDR_SEC1);
}

void W25Q_seq_test_t3_cross_page_rw(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC2);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));
    for (uint32_t i = 0; i < SIZE_CROSS_PAGE; i++) buf_a[i] = (uint8_t)(0xC3 ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_CROSS_PAGE, SIZE_CROSS_PAGE);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_CROSS_PAGE, SIZE_CROSS_PAGE);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, SIZE_CROSS_PAGE, ADDR_CROSS_PAGE, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "128B @0x%06lX cheval page7/8 OK",
             (unsigned long)ADDR_CROSS_PAGE);
}

void W25Q_seq_test_t4_cross_sector_rw(TEST_case_t *tc) {
    W25Q_STATE st;
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC3);
    TEST_ASSERT(st == W25Q_OK, "Erase sec3: %s", state_str(st));
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC4);
    TEST_ASSERT(st == W25Q_OK, "Erase sec4: %s", state_str(st));
    for (uint32_t i = 0; i < SIZE_CROSS_SECTOR; i++) buf_a[i] = (uint8_t)(0x55 ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_CROSS_SECTOR, SIZE_CROSS_SECTOR);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_CROSS_SECTOR, SIZE_CROSS_SECTOR);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, SIZE_CROSS_SECTOR, ADDR_CROSS_SECTOR, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "32B @0x%06lX cheval sec3/4 OK",
             (unsigned long)ADDR_CROSS_SECTOR);
}

void W25Q_seq_test_t5_sector_isolation(TEST_case_t *tc) {
    for (uint32_t i = 0; i < SIZE_ISOL; i++) { buf_isol_a[i] = 0xAA; buf_isol_b[i] = 0x55; }

    W25Q_STATE st;
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC5);
    TEST_ASSERT(st == W25Q_OK, "Erase sec5: %s", state_str(st));
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC6);
    TEST_ASSERT(st == W25Q_OK, "Erase sec6: %s", state_str(st));
    st = W25Q_WriteData(w25q, buf_isol_a, ADDR_ISOL_END5, SIZE_ISOL);
    TEST_ASSERT(st == W25Q_OK, "Write 0xAA: %s", state_str(st));
    st = W25Q_WriteData(w25q, buf_isol_b, ADDR_ISOL_BEG6, SIZE_ISOL);
    TEST_ASSERT(st == W25Q_OK, "Write 0x55: %s", state_str(st));
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC5);  /* re-efface sec5 seul */
    TEST_ASSERT(st == W25Q_OK, "Re-erase sec5: %s", state_str(st));

    st = W25Q_ReadData(w25q, buf_isol_rx, ADDR_ISOL_END5, SIZE_ISOL);
    TEST_ASSERT(st == W25Q_OK, "Read sec5: %s", state_str(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_isol_rx, SIZE_ISOL, 0xFF, ADDR_ISOL_END5, &first_addr, &first_got),
                "Sec5 non efface @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);

    st = W25Q_ReadData(w25q, buf_isol_rx, ADDR_ISOL_BEG6, SIZE_ISOL);
    TEST_ASSERT(st == W25Q_OK, "Read sec6: %s", state_str(st));
    uint32_t first_addr2; uint8_t first_exp2, first_got2;
    uint32_t mm = count_mm(buf_isol_b, buf_isol_rx, SIZE_ISOL, ADDR_ISOL_BEG6, &first_addr2, &first_exp2, &first_got2);
    TEST_ASSERT(mm == 0, "Sec6 corrompu: %lu mm @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr2, first_exp2, first_got2);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "Sec5=0xFF efface, sec6=0x55 intact");
}

void W25Q_seq_test_t6_and_behavior(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC7);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));

    uint8_t val = 0x0F;
    st = W25Q_WriteData(w25q, &val, ADDR_SEC7, 1);
    TEST_ASSERT(st == W25Q_OK, "Write 0x0F: %s", state_str(st));

    val = 0xF0;
    st = W25Q_WriteData(w25q, &val, ADDR_SEC7, 1);   /* pas d'effacement */
    TEST_ASSERT(st == W25Q_OK, "Write 0xF0: %s", state_str(st));

    uint8_t got = 0xFF;
    st = W25Q_ReadData(w25q, &got, ADDR_SEC7, 1);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));

    const uint8_t expected = 0x0F & 0xF0;  /* = 0x00 */
    TEST_ASSERT(got == expected, "exp=0x%02X got=0x%02X (devrait etre AND)", expected, got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "0xFF->0x0F->0x00 (0x0F & 0xF0) OK");
}

void W25Q_seq_test_t7_unaligned_rw(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC8);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));

    for (uint32_t i = 0; i < SIZE_UNALIGNED; i++) buf_a[i] = (uint8_t)(0x3C ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_UNALIGNED, SIZE_UNALIGNED);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", state_str(st));

    st = W25Q_ReadData(w25q, buf_b, ADDR_SEC8, 200U);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));

    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, 50U, 0xFF, ADDR_SEC8, &first_addr, &first_got),
                "Zone avant corrompue @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);

    uint32_t first_addr2; uint8_t first_exp2, first_got2;
    uint32_t mm = count_mm(buf_a, buf_b + 50U, SIZE_UNALIGNED, ADDR_UNALIGNED, &first_addr2, &first_exp2, &first_got2);
    TEST_ASSERT(mm == 0, "%lu mm @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr2, first_exp2, first_got2);

    TEST_ASSERT(verify_uniform(buf_b + 150U, 50U, 0xFF, ADDR_UNALIGNED + SIZE_UNALIGNED, &first_addr, &first_got),
                "Zone apres corrompue @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);

    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "100B @0x%06lX (offset+50), pre/post=0xFF OK",
             (unsigned long)ADDR_UNALIGNED);
}

void W25Q_seq_test_t8_read_status(TEST_case_t *tc) {
    W25Q_STATE st;
    st = W25Q_ReadStatus(w25q, 1);
    TEST_ASSERT(st == W25Q_OK, "ReadSR1: %s", state_str(st));
    st = W25Q_ReadStatus(w25q, 2);
    TEST_ASSERT(st == W25Q_OK, "ReadSR2: %s", state_str(st));
    st = W25Q_ReadStatus(w25q, 3);
    TEST_ASSERT(st == W25Q_OK, "ReadSR3: %s", state_str(st));
    TEST_ASSERT(W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT),
                "ADS=0 (mode 3-byte), status_reg=0x%06lX", (unsigned long)w25q->status_reg);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "SR1=0x%02X SR2=0x%02X SR3=0x%02X ADS=1",
             (uint8_t)(w25q->status_reg >>  0),
             (uint8_t)(w25q->status_reg >>  8),
             (uint8_t)(w25q->status_reg >> 16));
}

void W25Q_seq_test_t9_erase_32kb(TEST_case_t *tc) {
    /* Pre-condition : s'assure que la zone n'est pas deja vierge */
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_BLK32);
    TEST_ASSERT(st == W25Q_OK, "Pre-erase sector: %s", state_str(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) {
        buf_a[i] = (uint8_t)(0xAB ^ i);
    }
    st = W25Q_WriteData(w25q, buf_a, ADDR_BLK32, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Write motif: %s", state_str(st));
    /* Efface tout le bloc 32 KB (0x010000-0x017FFF) */
    st = W25Q_SendCmdAddr(w25q, W25Q_32KB_BLOCK_ERASE, ADDR_BLK32);
    TEST_ASSERT(st == W25Q_OK, "32KB Erase: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_BLK32, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_BLK32, &first_addr, &first_got),
                "Non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "256B=0xFF apres 32KB erase @0x%06lX",
             (unsigned long)ADDR_BLK32);
}

void W25Q_seq_test_t10_erase_64kb(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_BLK64);
    TEST_ASSERT(st == W25Q_OK, "Pre-erase sector: %s", state_str(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0xDE ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_BLK64, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Write motif: %s", state_str(st));
    st = W25Q_SendCmdAddr(w25q, W25Q_64KB_BLOCK_ERASE_4B, ADDR_BLK64);
    TEST_ASSERT(st == W25Q_OK, "64KB Erase: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_BLK64, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_BLK64, &first_addr, &first_got),
                "Non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "256B=0xFF apres 64KB erase @0x%06lX",
             (unsigned long)ADDR_BLK64);
}

void W25Q_seq_test_t11_soft_reset(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmd(w25q, W25Q_ENABLE_RESET);
    TEST_ASSERT(st == W25Q_OK, "ENABLE_RESET: %s", state_str(st));
    st = W25Q_SendCmd(w25q, W25Q_RESET);
    TEST_ASSERT(st == W25Q_OK, "RESET: %s", state_str(st));
    /* Rafraichit status_reg local (stale apres reset interne du chip) */
    if (W25Q_ReadStatus(w25q, 1) == W25Q_OK)
        W25Q_ReadStatus(w25q, 2);
    st = W25Q_ReadStatus(w25q, 3);
    TEST_ASSERT(st == W25Q_OK, "ReadStatus post-reset: %s", state_str(st));
    uint8_t id[3] = {0};
    st = W25Q_ReadID(w25q, id);
    TEST_ASSERT(st == W25Q_OK, "ReadID post-reset: %s", state_str(st));
    uint16_t dev = (uint16_t)((id[1] << 8) | id[2]);
    TEST_ASSERT(id[0] == W25Q_MANUFACTURER_ID && dev == W25Q_V_FULL_DEVICE_ID,
                "Post-reset: Manuf=0x%02X Dev=0x%04X", id[0], dev);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "ID OK post-reset, ADS=%d", W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT));
}

void W25Q_seq_test_t12_multi_sector_rw(TEST_case_t *tc) {
    W25Q_STATE st;
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_MULTI);
    TEST_ASSERT(st == W25Q_OK, "Erase sec0: %s", state_str(st));
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_MULTI + SECTOR_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Erase sec1: %s", state_str(st));
    for (uint32_t i = 0; i < SIZE_LARGE; i++) {
        buf_large[i] = (uint8_t)(i & 0xFFU);
    }
    st = W25Q_WriteData(w25q, buf_large, ADDR_MULTI, SIZE_LARGE);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_large_rx, ADDR_MULTI, SIZE_LARGE);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_large, buf_large_rx, SIZE_LARGE, ADDR_MULTI,
                            &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "%uB @0x%06lX (17 pages, 2 sec) OK",
             (unsigned)SIZE_LARGE, (unsigned long)ADDR_MULTI);
}

void W25Q_seq_test_t13_end_of_flash_clamp(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_NEAR_END);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0x7E ^ i);
    /* Tente d'ecrire 256 B alors qu'il ne reste que 128 B disponibles */
    st = W25Q_WriteData(w25q, buf_a, ADDR_NEAR_END, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Write: %s (attendu OK)", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_NEAR_END, 128U);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, 128U, ADDR_NEAR_END,
                            &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "128/256B clampees @0x%06lX OK", (unsigned long)ADDR_NEAR_END);
}

void W25Q_seq_test_t14_write_zero_size(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_WriteData(w25q, buf_a, ADDR_SEC0, 0U);
    TEST_ASSERT(st == W25Q_OK, "Write(size=0): %s (attendu OK)", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_SEC0, PAGE_BYTES);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_SEC0, &first_addr, &first_got),
                "Zone modifiee @0x%06lX got=0x%02X (attendu 0xFF)",
                (unsigned long)first_addr, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "W25Q_OK, zone intacte (0xFF)");
}
