#include "w25q_seq_test.h"
#include "w25q_test_common.h"

#include <stdio.h>

// Pointer need to be set
static W25Q_t *w25q = NULL;

void W25Q_seq_test_set_context(W25Q_t *w25q_ctx) {
    w25q = w25q_ctx;
}

TEST_case_table_t W25Q_seq_test_cases[W25Q_seq_test_N_TESTS] = {
    /* A. Communication */
    { .case_info = { .name = "T0 ID Check"            } , .func = W25Q_seq_test_t0_id_check           },
    { .case_info = { .name = "T1 Read Status"         } , .func = W25Q_seq_test_t1_read_status        },
    /* B. Configuration */
    { .case_info = { .name = "T2 Cfg To Status"       } , .func = W25Q_seq_test_t2_cfg_to_status      },
    { .case_info = { .name = "T3 Cfg Applied"         } , .func = W25Q_seq_test_t3_cfg_applied        },
    { .case_info = { .name = "T4 Cfg Invalid"         } , .func = W25Q_seq_test_t4_cfg_invalid        },
    { .case_info = { .name = "T5 Cfg Keep"            } , .func = W25Q_seq_test_t5_cfg_keep           },
    /* C. Primitives commande */
    { .case_info = { .name = "T6 Cmd Invalid"         } , .func = W25Q_seq_test_t6_cmd_invalid        },
    { .case_info = { .name = "T7 Write Enable Latch"  } , .func = W25Q_seq_test_t7_wel                },
    { .case_info = { .name = "T8 Addr Mode Tracking"  } , .func = W25Q_seq_test_t8_addr_mode_tracking },
    /* D. Effacement secteur */
    { .case_info = { .name = "T9 Erase Verify"        } , .func = W25Q_seq_test_t9_erase_verify       },
    /* E. Lecture / ecriture */
    { .case_info = { .name = "T10 Aligned R/W"        } , .func = W25Q_seq_test_t10_aligned_rw        },
    { .case_info = { .name = "T11 Cross-page R/W"     } , .func = W25Q_seq_test_t11_cross_page_rw     },
    { .case_info = { .name = "T12 Cross-sector R/W"   } , .func = W25Q_seq_test_t12_cross_sector_rw   },
    { .case_info = { .name = "T13 Unaligned R/W"      } , .func = W25Q_seq_test_t13_unaligned_rw      },
    { .case_info = { .name = "T14 Multi-sector R/W"   } , .func = W25Q_seq_test_t14_multi_sector_rw   },
    { .case_info = { .name = "T15 AND without Erase"  } , .func = W25Q_seq_test_t15_and_behavior      },
    { .case_info = { .name = "T16 R/W in 3B Mode"     } , .func = W25Q_seq_test_t16_rw_3b_mode        },
    /* F. Effacement etendu */
    { .case_info = { .name = "T17 Erase Addr Length"  } , .func = W25Q_seq_test_t17_erase_addr_len    },
    { .case_info = { .name = "T18 Block Erase 32 KB"  } , .func = W25Q_seq_test_t18_erase_32kb        },
    { .case_info = { .name = "T19 Block Erase 64 KB"  } , .func = W25Q_seq_test_t19_erase_64kb        },
    { .case_info = { .name = "T20 Sector Isolation"   } , .func = W25Q_seq_test_t20_sector_isolation  },
    { .case_info = { .name = "T21 Busy Timeout"       } , .func = W25Q_seq_test_t21_busy_timeout      },
    /* G. Cas limites R/W */
    { .case_info = { .name = "T22 Write Size=0"       } , .func = W25Q_seq_test_t22_write_zero_size   },
    { .case_info = { .name = "T23 Write Near End"     } , .func = W25Q_seq_test_t23_write_end_clamp   },
    { .case_info = { .name = "T24 Read Near End"      } , .func = W25Q_seq_test_t24_read_end_clamp    },
    { .case_info = { .name = "T25 Addr Out Of Range"  } , .func = W25Q_seq_test_t25_addr_out_of_range },
    /* H. Reset */
    { .case_info = { .name = "T26 Soft Reset"         } , .func = W25Q_seq_test_t26_soft_reset        },
};


/* ========================================================================
 * Buffers statiques
 * ======================================================================== */
static uint8_t buf_a[PAGE_BYTES];
static uint8_t buf_b[PAGE_BYTES];
static uint8_t buf_isol_a[SIZE_ISOL];
static uint8_t buf_isol_b[SIZE_ISOL];
static uint8_t buf_isol_rx[SIZE_ISOL];
static uint8_t buf_large[SIZE_LARGE];     /* T14 – write multi-secteurs   */
static uint8_t buf_large_rx[SIZE_LARGE];  /* T14 – read  multi-secteurs   */

/* ========================================================================
 * Utilitaires internes
 * ======================================================================== */

/* Relit SR1, SR2, SR3 dans w25q->status_reg. */
static W25Q_STATE read_all_status(void) {
    W25Q_STATE st = W25Q_OK;
    for (uint8_t sr = 1; sr <= 3 && st == W25Q_OK; sr++) st = W25Q_ReadStatus(w25q, sr);
    return st;
}

/* Ecrit un motif sur un secteur deja programme puis l'efface avec 20h (longueur
   d'adresse selon ADS) : retourne true si le secteur vise est bien vierge. */
static bool erase_20h_hits(uint32_t addr, W25Q_STATE *st_out) {
    uint8_t pat[16], rx[16];
    for (uint32_t i = 0; i < sizeof(pat); i++) pat[i] = (uint8_t)(0x4D ^ i);
    *st_out = W25Q_WriteData(w25q, pat, addr, sizeof(pat), W25Q_TEST_TIMEOUT_MS);
    if (*st_out == W25Q_OK) *st_out = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE, addr, W25Q_TEST_TIMEOUT_MS);
    if (*st_out == W25Q_OK) *st_out = W25Q_ReadData(w25q, rx, addr, sizeof(rx), W25Q_TEST_TIMEOUT_MS);
    if (*st_out != W25Q_OK) return false;
    uint32_t first_addr; uint8_t first_got;
    return verify_uniform(rx, sizeof(rx), 0xFF, addr, &first_addr, &first_got);
}

/* ======================= A. Communication ============================== */

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

void W25Q_seq_test_t1_read_status(TEST_case_t *tc) {
    W25Q_STATE st;
    st = W25Q_ReadStatus(w25q, 1);
    TEST_ASSERT(st == W25Q_OK, "ReadSR1: %s", state_str(st));
    st = W25Q_ReadStatus(w25q, 2);
    TEST_ASSERT(st == W25Q_OK, "ReadSR2: %s", state_str(st));
    st = W25Q_ReadStatus(w25q, 3);
    TEST_ASSERT(st == W25Q_OK, "ReadSR3: %s", state_str(st));
    TEST_ASSERT(!W25Q_STATUS_REG(w25q, W25Q_SR1_BUSY_BIT), "BUSY=1 au repos, SR1=0x%02X",
                (uint8_t)w25q->status_reg);

    uint32_t cached = w25q->status_reg;
    st = W25Q_ReadStatus(w25q, 0);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "ReadStatus(0): %s (attendu PARAM_ERR)", state_str(st));
    st = W25Q_ReadStatus(w25q, 4);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "ReadStatus(4): %s (attendu PARAM_ERR)", state_str(st));
    TEST_ASSERT(w25q->status_reg == cached, "status_reg modifie par un index invalide");

    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "SR1=0x%02X SR2=0x%02X SR3=0x%02X, index 0/4 rejetes",
             (uint8_t)(w25q->status_reg >>  0),
             (uint8_t)(w25q->status_reg >>  8),
             (uint8_t)(w25q->status_reg >> 16));
}


/* ======================= B. Configuration ============================== */

void W25Q_seq_test_t2_cfg_to_status(TEST_case_t *tc) {
    uint32_t mask, bits;

    W25Q_reg_config_t keep = { 0 };
    W25Q_STATE st = W25Q_ConfigToStatus(&keep, &mask, &bits);
    TEST_ASSERT(st == W25Q_OK, "Config KEEP: %s", state_str(st));
    TEST_ASSERT(mask == 0 && bits == 0, "Config KEEP: mask=0x%06lX bits=0x%06lX (attendu 0)",
                (unsigned long)mask, (unsigned long)bits);

    W25Q_reg_config_t full = {
        .sr_write             = W25Q_SR_WRITE_VOLATILE,
        .block_protect        = W25Q_CFG_BP(5),
        .top_bottom           = W25Q_CFG_TB_BOTTOM,
        .complement           = W25Q_CFG_CMP_ON,
        .quad_enable          = W25Q_CFG_QE_OFF,
        .addr_mode            = W25Q_CFG_ADS_4B,
        .power_up_addr_mode   = W25Q_CFG_ADP_4B,
        .write_protect_scheme = W25Q_CFG_WPS_STATUS_BITS,
        .drive_strength       = W25Q_CFG_DRV_50,
    };
    const uint32_t exp_mask = (0xFUL << W25Q_SR1_BP0_BIT) | (1UL << W25Q_SR1_TB_BIT)
                            | (1UL << W25Q_SR2_CMP_BIT)   | (1UL << W25Q_SR2_QE_BIT)
                            | (1UL << W25Q_SR3_ADP_BIT)   | (1UL << W25Q_SR3_WPS_BIT)
                            | (0x3UL << W25Q_SR3_DRV0_BIT);
    const uint32_t exp_bits = (5UL << W25Q_SR1_BP0_BIT)   | (1UL << W25Q_SR1_TB_BIT)
                            | (1UL << W25Q_SR2_CMP_BIT)   | (1UL << W25Q_SR3_ADP_BIT)
                            | (0x2UL << W25Q_SR3_DRV0_BIT);
    st = W25Q_ConfigToStatus(&full, &mask, &bits);
    TEST_ASSERT(st == W25Q_OK, "Config complete: %s", state_str(st));
    TEST_ASSERT(mask == exp_mask, "mask=0x%06lX != 0x%06lX", (unsigned long)mask, (unsigned long)exp_mask);
    TEST_ASSERT(bits == exp_bits, "bits=0x%06lX != 0x%06lX", (unsigned long)bits, (unsigned long)exp_bits);
    TEST_ASSERT(!(mask & (1UL << W25Q_SR3_ADS_BIT)), "ADS dans le masque d'ecriture");

    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "KEEP->0, complete->mask=0x%06lX bits=0x%06lX",
             (unsigned long)mask, (unsigned long)bits);
}

void W25Q_seq_test_t3_cfg_applied(TEST_case_t *tc) {
    const W25Q_config_t ref = w25q->config;
    W25Q_STATE st = W25Q_Init(w25q, ref, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Re-init: %s", state_str(st));

    uint32_t mask, bits;
    st = W25Q_ConfigToStatus(&ref.reg, &mask, &bits);
    TEST_ASSERT(st == W25Q_OK, "ConfigToStatus: %s", state_str(st));
    st = read_all_status();
    TEST_ASSERT(st == W25Q_OK, "ReadStatus: %s", state_str(st));
    TEST_ASSERT((w25q->status_reg & mask) == bits, "SR=0x%06lX, attendu 0x%06lX sur mask 0x%06lX",
                (unsigned long)w25q->status_reg, (unsigned long)bits, (unsigned long)mask);

    int ads = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    if (ref.reg.addr_mode != W25Q_CFG_ADS_KEEP) {
        int want = (ref.reg.addr_mode == W25Q_CFG_ADS_4B);
        TEST_ASSERT(ads == want, "ADS=%d != config (%d)", ads, want);
    }

    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "SR=0x%06lX conforme (mask 0x%06lX), ADS=%d",
             (unsigned long)w25q->status_reg, (unsigned long)mask, ads);
}

void W25Q_seq_test_t4_cfg_invalid(TEST_case_t *tc) {
    const W25Q_config_t ref = w25q->config;
    W25Q_config_t cfg;

    cfg = ref;
    cfg.reg.addr_mode = (W25Q_CFG_ADS)(W25Q_CFG_ADS_4B + 1);
    W25Q_STATE st = W25Q_Init(w25q, cfg, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "addr_mode invalide: %s (attendu PARAM_ERR)", state_str(st));

    cfg = ref;
    cfg.reg.block_protect = (uint8_t)(W25Q_CFG_BP(15) + 1u);
    st = W25Q_Init(w25q, cfg, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "block_protect invalide: %s (attendu PARAM_ERR)", state_str(st));

    cfg = ref;
    cfg.bus.hspi = NULL;
    st = W25Q_Init(w25q, cfg, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "hspi NULL: %s (attendu PARAM_ERR)", state_str(st));

    /* Un rejet ne doit pas ecraser la config en place */
    TEST_ASSERT(w25q->config.bus.hspi == ref.bus.hspi && w25q->config.reg.addr_mode == ref.reg.addr_mode,
                "chip->config modifiee par un Init rejete");
    uint8_t id[3] = {0};
    st = W25Q_ReadID(w25q, id);
    TEST_ASSERT(st == W25Q_OK, "ReadID apres rejet: %s", state_str(st));
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "3 configs invalides -> PARAM_ERR, config intacte");
}

void W25Q_seq_test_t5_cfg_keep(TEST_case_t *tc) {
    const W25Q_config_t ref = w25q->config;
    W25Q_STATE st = read_all_status();
    TEST_ASSERT(st == W25Q_OK, "ReadStatus avant: %s", state_str(st));
    uint32_t before = w25q->status_reg;

    W25Q_config_t cfg = { .bus = ref.bus };   /* reg : tout a W25Q_CFG_KEEP */
    W25Q_STATE st_keep = W25Q_Init(w25q, cfg, W25Q_TEST_TIMEOUT_MS);
    uint32_t after = w25q->status_reg;
    W25Q_STATE st_ref = W25Q_Init(w25q, ref, W25Q_TEST_TIMEOUT_MS);

    TEST_ASSERT(st_keep == W25Q_OK, "Init KEEP: %s", state_str(st_keep));
    TEST_ASSERT((before & ~SR_VOLATILE_MASK) == (after & ~SR_VOLATILE_MASK),
                "SR modifies: avant=0x%06lX apres=0x%06lX", (unsigned long)before, (unsigned long)after);
    TEST_ASSERT(st_ref == W25Q_OK, "Restauration config: %s", state_str(st_ref));
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "SR inchanges (0x%06lX)", (unsigned long)after);
}


/* ======================= C. Primitives commande ======================== */

void W25Q_seq_test_t6_cmd_invalid(TEST_case_t *tc) {
    const uint8_t bad = 0x00;   /* absent de W25Q_CMD_FLAGS */
    W25Q_STATE st = W25Q_SendCmd(w25q, bad, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "SendCmd(0x00): %s (attendu PARAM_ERR)", state_str(st));
    st = W25Q_SendCmdAddr(w25q, bad, ADDR_SEC0, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "SendCmdAddr(0x00): %s (attendu PARAM_ERR)", state_str(st));
    st = W25Q_WriteStatus(w25q, 0, 0x00, W25Q_SR_WRITE_VOLATILE, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "WriteStatus(0): %s (attendu PARAM_ERR)", state_str(st));
    st = W25Q_WriteStatus(w25q, 4, 0x00, W25Q_SR_WRITE_VOLATILE, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "WriteStatus(4): %s (attendu PARAM_ERR)", state_str(st));

    uint8_t id[3] = {0};
    st = W25Q_ReadID(w25q, id);
    TEST_ASSERT(st == W25Q_OK && id[0] == W25Q_MANUFACTURER_ID,
                "ReadID apres rejets: %s, Manuf=0x%02X", state_str(st), id[0]);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "SendCmd/SendCmdAddr/WriteStatus -> PARAM_ERR");
}

void W25Q_seq_test_t7_wel(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmd(w25q, W25Q_WRITE_ENABLE, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "WRITE_ENABLE: %s", state_str(st));
    st = W25Q_ReadStatus(w25q, 1);
    TEST_ASSERT(st == W25Q_OK, "ReadSR1: %s", state_str(st));
    TEST_ASSERT(W25Q_STATUS_REG(w25q, W25Q_SR1_WEL_BIT), "WEL=0 apres 06h, SR1=0x%02X",
                (uint8_t)w25q->status_reg);

    st = W25Q_SendCmd(w25q, W25Q_WRITE_DISABLE, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "WRITE_DISABLE: %s", state_str(st));
    st = W25Q_ReadStatus(w25q, 1);
    TEST_ASSERT(st == W25Q_OK, "ReadSR1: %s", state_str(st));
    TEST_ASSERT(!W25Q_STATUS_REG(w25q, W25Q_SR1_WEL_BIT), "WEL=1 apres 04h, SR1=0x%02X",
                (uint8_t)w25q->status_reg);

    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "06h -> WEL=1, 04h -> WEL=0");
}

void W25Q_seq_test_t8_addr_mode_tracking(TEST_case_t *tc) {
    const W25Q_config_t ref = w25q->config;
    W25Q_config_t cfg = ref;

    /* Toutes les operations d'abord, puis restauration, puis verifications :
       un TEST_ASSERT ne doit pas laisser la puce dans un mode non configure. */
    cfg.reg.addr_mode = W25Q_CFG_ADS_3B;
    W25Q_STATE st3 = W25Q_Init(w25q, cfg, W25Q_TEST_TIMEOUT_MS);
    int ads3 = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);

    /* SR3 reecrit tel quel avec ADS force a 1 : le bit est en lecture seule */
    uint8_t sr3 = (uint8_t)(w25q->status_reg >> 16);
    W25Q_STATE ws = W25Q_WriteStatus(w25q, 3, (uint8_t)(sr3 | (1u << (W25Q_SR3_ADS_BIT - 16))),
                                     W25Q_SR_WRITE_VOLATILE, W25Q_TEST_TIMEOUT_MS);
    int ws_tracked = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    W25Q_STATE ws_rd = W25Q_ReadStatus(w25q, 3);
    int ws_read = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);

    /* Mode 3 octets : 20h limite a 16 MB, 21h (4-byte) non limite */
    W25Q_STATE oor3 = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE, ADDR_ABOVE_16MB, W25Q_TEST_TIMEOUT_MS);
    W25Q_STATE op4 = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_ABOVE_16MB, W25Q_TEST_TIMEOUT_MS);
    W25Q_STATE op4_wait = W25Q_WaitForReady(w25q, W25Q_TEST_TIMEOUT_MS);

    /* SendCmd seul (sans Init, qui relit SR3) */
    W25Q_STATE sc_b7 = W25Q_SendCmd(w25q, W25Q_ENABLE_4B_MODE, W25Q_TEST_TIMEOUT_MS);
    int b7_tracked = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    W25Q_STATE rs_b7 = W25Q_ReadStatus(w25q, 3);
    int b7_read = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    W25Q_STATE sc_e9 = W25Q_SendCmd(w25q, W25Q_DISABLE_4B_MODE, W25Q_TEST_TIMEOUT_MS);
    int e9_tracked = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    W25Q_STATE rs_e9 = W25Q_ReadStatus(w25q, 3);
    int e9_read = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);

    W25Q_STATE st_ref = W25Q_Init(w25q, ref, W25Q_TEST_TIMEOUT_MS);

    TEST_ASSERT(st3 == W25Q_OK, "Init 3B: %s", state_str(st3));
    TEST_ASSERT(ads3 == 0, "ADS=%d apres config 3B", ads3);
    TEST_ASSERT(ws == W25Q_OK && ws_rd == W25Q_OK, "WriteStatus SR3: %s, ReadStatus: %s",
                state_str(ws), state_str(ws_rd));
    TEST_ASSERT(ws_tracked == 0 && ws_read == 0, "ADS apres WriteStatus: suivi=%d relu=%d (attendu 0)",
                ws_tracked, ws_read);
    TEST_ASSERT(oor3 == W25Q_PARAM_ERR, "20h en 3B >16MB: %s (attendu PARAM_ERR)", state_str(oor3));
    TEST_ASSERT(op4 == W25Q_OK && op4_wait == W25Q_OK, "21h en 3B >16MB: %s, attente: %s (attendu OK)",
                state_str(op4), state_str(op4_wait));
    TEST_ASSERT(sc_b7 == W25Q_OK && rs_b7 == W25Q_OK, "SendCmd(B7h): %s, ReadStatus: %s",
                state_str(sc_b7), state_str(rs_b7));
    TEST_ASSERT(b7_tracked == 1 && b7_read == 1, "ADS apres B7h: suivi=%d relu=%d (attendu 1)",
                b7_tracked, b7_read);
    TEST_ASSERT(sc_e9 == W25Q_OK && rs_e9 == W25Q_OK, "SendCmd(E9h): %s, ReadStatus: %s",
                state_str(sc_e9), state_str(rs_e9));
    TEST_ASSERT(e9_tracked == 0 && e9_read == 0, "ADS apres E9h: suivi=%d relu=%d (attendu 0)",
                e9_tracked, e9_read);
    TEST_ASSERT(st_ref == W25Q_OK, "Restauration config: %s", state_str(st_ref));
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "ADS suivi (Init/WriteStatus/B7h/E9h), 20h/21h en 3B OK");
}


/* ======================= D. Effacement secteur ========================= */

void W25Q_seq_test_t9_erase_verify(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC0, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "SectorErase: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_a, ADDR_SEC0, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "ReadData: %s", state_str(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_a, PAGE_BYTES, 0xFF, ADDR_SEC0, &first_addr, &first_got),
                "Non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "%uB=0xFF apres erase @0x%06lX",
             (unsigned)PAGE_BYTES, (unsigned long)ADDR_SEC0);
}


/* ======================= E. Lecture / ecriture ========================= */

void W25Q_seq_test_t10_aligned_rw(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC1, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0xA5 ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_SEC1, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_SEC1, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, PAGE_BYTES, ADDR_SEC1, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "256B @0x%06lX motif 0xA5^i OK",
             (unsigned long)ADDR_SEC1);
}

void W25Q_seq_test_t11_cross_page_rw(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC2, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));
    for (uint32_t i = 0; i < SIZE_CROSS_PAGE; i++) buf_a[i] = (uint8_t)(0xC3 ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_CROSS_PAGE, SIZE_CROSS_PAGE, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_CROSS_PAGE, SIZE_CROSS_PAGE, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, SIZE_CROSS_PAGE, ADDR_CROSS_PAGE, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "128B @0x%06lX cheval page7/8 OK",
             (unsigned long)ADDR_CROSS_PAGE);
}

void W25Q_seq_test_t12_cross_sector_rw(TEST_case_t *tc) {
    W25Q_STATE st;
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC3, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase sec3: %s", state_str(st));
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC4, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase sec4: %s", state_str(st));
    for (uint32_t i = 0; i < SIZE_CROSS_SECTOR; i++) buf_a[i] = (uint8_t)(0x55 ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_CROSS_SECTOR, SIZE_CROSS_SECTOR, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_CROSS_SECTOR, SIZE_CROSS_SECTOR, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, buf_b, SIZE_CROSS_SECTOR, ADDR_CROSS_SECTOR, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "%lu mm, 1er @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "32B @0x%06lX cheval sec3/4 OK",
             (unsigned long)ADDR_CROSS_SECTOR);
}

void W25Q_seq_test_t13_unaligned_rw(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC8, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));

    for (uint32_t i = 0; i < SIZE_UNALIGNED; i++) buf_a[i] = (uint8_t)(0x3C ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_UNALIGNED, SIZE_UNALIGNED, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", state_str(st));

    st = W25Q_ReadData(w25q, buf_b, ADDR_SEC8, 200U, W25Q_TEST_TIMEOUT_MS);
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

void W25Q_seq_test_t14_multi_sector_rw(TEST_case_t *tc) {
    W25Q_STATE st;
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_MULTI, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase sec0: %s", state_str(st));
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_MULTI + SECTOR_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase sec1: %s", state_str(st));
    for (uint32_t i = 0; i < SIZE_LARGE; i++) {
        buf_large[i] = (uint8_t)(i & 0xFFU);
    }
    st = W25Q_WriteData(w25q, buf_large, ADDR_MULTI, SIZE_LARGE, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_large_rx, ADDR_MULTI, SIZE_LARGE, W25Q_TEST_TIMEOUT_MS);
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

void W25Q_seq_test_t15_and_behavior(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC7, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));

    uint8_t val = 0x0F;
    st = W25Q_WriteData(w25q, &val, ADDR_SEC7, 1, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write 0x0F: %s", state_str(st));

    val = 0xF0;
    st = W25Q_WriteData(w25q, &val, ADDR_SEC7, 1, W25Q_TEST_TIMEOUT_MS);   /* pas d'effacement */
    TEST_ASSERT(st == W25Q_OK, "Write 0xF0: %s", state_str(st));

    uint8_t got = 0xFF;
    st = W25Q_ReadData(w25q, &got, ADDR_SEC7, 1, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));

    const uint8_t expected = 0x0F & 0xF0;  /* = 0x00 */
    TEST_ASSERT(got == expected, "exp=0x%02X got=0x%02X (devrait etre AND)", expected, got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "0xFF->0x0F->0x00 (0x0F & 0xF0) OK");
}

void W25Q_seq_test_t16_rw_3b_mode(TEST_case_t *tc) {
    const W25Q_config_t ref = w25q->config;
    W25Q_config_t cfg = ref;
    const uint32_t half = SIZE_ADDR_MODE / 2u;
    uint8_t rx3[SIZE_ADDR_MODE], rx4[SIZE_ADDR_MODE];

    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_ABOVE_16MB, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase >16MB: %s", state_str(st));
    for (uint32_t i = 0; i < SIZE_ADDR_MODE; i++) buf_a[i] = (uint8_t)(0xE1 ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_ABOVE_16MB, half, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write 1re moitie: %s", state_str(st));

    /* Operations en 3B, restauration, puis verifications */
    cfg.reg.addr_mode = W25Q_CFG_ADS_3B;
    W25Q_STATE st3 = W25Q_Init(w25q, cfg, W25Q_TEST_TIMEOUT_MS);
    W25Q_STATE wr3 = W25Q_WriteData(w25q, buf_a + half, ADDR_ABOVE_16MB + half, half, W25Q_TEST_TIMEOUT_MS);
    W25Q_STATE rd3 = W25Q_ReadData(w25q, rx3, ADDR_ABOVE_16MB, SIZE_ADDR_MODE, W25Q_TEST_TIMEOUT_MS);
    W25Q_STATE st_ref = W25Q_Init(w25q, ref, W25Q_TEST_TIMEOUT_MS);
    W25Q_STATE rd4 = W25Q_ReadData(w25q, rx4, ADDR_ABOVE_16MB, SIZE_ADDR_MODE, W25Q_TEST_TIMEOUT_MS);

    TEST_ASSERT(st3 == W25Q_OK, "Init 3B: %s", state_str(st3));
    TEST_ASSERT(wr3 == W25Q_OK, "Write >16MB en 3B: %s", state_str(wr3));
    TEST_ASSERT(rd3 == W25Q_OK, "Read >16MB en 3B: %s", state_str(rd3));
    TEST_ASSERT(st_ref == W25Q_OK, "Restauration config: %s", state_str(st_ref));
    TEST_ASSERT(rd4 == W25Q_OK, "Read >16MB apres restauration: %s", state_str(rd4));
    uint32_t first_addr; uint8_t first_exp, first_got;
    uint32_t mm = count_mm(buf_a, rx3, SIZE_ADDR_MODE, ADDR_ABOVE_16MB, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "Lecture en 3B: %lu mm @0x%07lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    mm = count_mm(buf_a, rx4, SIZE_ADDR_MODE, ADDR_ABOVE_16MB, &first_addr, &first_exp, &first_got);
    TEST_ASSERT(mm == 0, "Lecture apres restauration: %lu mm @0x%07lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr, first_exp, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "%uB @0x%07lX ecrits 4B+3B, relus identiques",
             (unsigned)SIZE_ADDR_MODE, (unsigned long)ADDR_ABOVE_16MB);
}


/* ======================= F. Effacement etendu ========================== */

void W25Q_seq_test_t17_erase_addr_len(TEST_case_t *tc) {
    const W25Q_config_t ref = w25q->config;
    W25Q_config_t cfg = ref;

    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC11, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase sec11: %s", state_str(st));
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC12, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase sec12: %s", state_str(st));

    /* Operations dans les deux modes, restauration, puis verifications */
    cfg.reg.addr_mode = W25Q_CFG_ADS_3B;
    W25Q_STATE st3 = W25Q_Init(w25q, cfg, W25Q_TEST_TIMEOUT_MS);
    W25Q_STATE er3;
    bool hit3 = erase_20h_hits(ADDR_SEC11, &er3);
    cfg.reg.addr_mode = W25Q_CFG_ADS_4B;
    W25Q_STATE st4 = W25Q_Init(w25q, cfg, W25Q_TEST_TIMEOUT_MS);
    W25Q_STATE er4;
    bool hit4 = erase_20h_hits(ADDR_SEC12, &er4);
    W25Q_STATE st_ref = W25Q_Init(w25q, ref, W25Q_TEST_TIMEOUT_MS);

    TEST_ASSERT(st3 == W25Q_OK, "Init 3B: %s", state_str(st3));
    TEST_ASSERT(er3 == W25Q_OK && hit3, "20h en 3B @0x%06lX: %s, secteur %s",
                (unsigned long)ADDR_SEC11, state_str(er3), hit3 ? "efface" : "non efface");
    TEST_ASSERT(st4 == W25Q_OK, "Init 4B: %s", state_str(st4));
    TEST_ASSERT(er4 == W25Q_OK && hit4, "20h en 4B @0x%06lX: %s, secteur %s",
                (unsigned long)ADDR_SEC12, state_str(er4), hit4 ? "efface" : "non efface");
    TEST_ASSERT(st_ref == W25Q_OK, "Restauration config: %s", state_str(st_ref));
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "20h: 3 octets en 3B, 4 octets en 4B, bon secteur efface");
}

void W25Q_seq_test_t18_erase_32kb(TEST_case_t *tc) {
    /* Pre-condition : s'assure que la zone n'est pas deja vierge */
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_BLK32, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Pre-erase sector: %s", state_str(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) {
        buf_a[i] = (uint8_t)(0xAB ^ i);
    }
    st = W25Q_WriteData(w25q, buf_a, ADDR_BLK32, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write motif: %s", state_str(st));
    /* Efface tout le bloc 32 KB (0x010000-0x017FFF) */
    st = W25Q_SendCmdAddr(w25q, W25Q_32KB_BLOCK_ERASE, ADDR_BLK32, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "32KB Erase: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_BLK32, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_BLK32, &first_addr, &first_got),
                "Non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "256B=0xFF apres 32KB erase @0x%06lX",
             (unsigned long)ADDR_BLK32);
}

void W25Q_seq_test_t19_erase_64kb(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_BLK64, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Pre-erase sector: %s", state_str(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0xDE ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_BLK64, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write motif: %s", state_str(st));
    st = W25Q_SendCmdAddr(w25q, W25Q_64KB_BLOCK_ERASE_4B, ADDR_BLK64, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "64KB Erase: %s", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_BLK64, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_BLK64, &first_addr, &first_got),
                "Non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "256B=0xFF apres 64KB erase @0x%06lX",
             (unsigned long)ADDR_BLK64);
}

void W25Q_seq_test_t20_sector_isolation(TEST_case_t *tc) {
    for (uint32_t i = 0; i < SIZE_ISOL; i++) { buf_isol_a[i] = 0xAA; buf_isol_b[i] = 0x55; }

    W25Q_STATE st;
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC5, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase sec5: %s", state_str(st));
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC6, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase sec6: %s", state_str(st));
    st = W25Q_WriteData(w25q, buf_isol_a, ADDR_ISOL_END5, SIZE_ISOL, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write 0xAA: %s", state_str(st));
    st = W25Q_WriteData(w25q, buf_isol_b, ADDR_ISOL_BEG6, SIZE_ISOL, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write 0x55: %s", state_str(st));
    st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC5, W25Q_TEST_TIMEOUT_MS);  /* re-efface sec5 seul */
    TEST_ASSERT(st == W25Q_OK, "Re-erase sec5: %s", state_str(st));

    st = W25Q_ReadData(w25q, buf_isol_rx, ADDR_ISOL_END5, SIZE_ISOL, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Read sec5: %s", state_str(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_isol_rx, SIZE_ISOL, 0xFF, ADDR_ISOL_END5, &first_addr, &first_got),
                "Sec5 non efface @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);

    st = W25Q_ReadData(w25q, buf_isol_rx, ADDR_ISOL_BEG6, SIZE_ISOL, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Read sec6: %s", state_str(st));
    uint32_t first_addr2; uint8_t first_exp2, first_got2;
    uint32_t mm = count_mm(buf_isol_b, buf_isol_rx, SIZE_ISOL, ADDR_ISOL_BEG6, &first_addr2, &first_exp2, &first_got2);
    TEST_ASSERT(mm == 0, "Sec6 corrompu: %lu mm @0x%06lX exp=0x%02X got=0x%02X",
                (unsigned long)mm, (unsigned long)first_addr2, first_exp2, first_got2);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "Sec5=0xFF efface, sec6=0x55 intact");
}

void W25Q_seq_test_t21_busy_timeout(TEST_case_t *tc) {
    /* L'effacement rend la main sans attendre la fin (pas de WAIT_AFTER) */
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC10, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "SectorErase: %s", state_str(st));
    W25Q_STATE st_short = W25Q_WaitForReady(w25q, 1U);   /* erase 4 KB : ~45 ms typ. */
    W25Q_STATE st_long  = W25Q_WaitForReady(w25q, W25Q_TEST_TIMEOUT_MS);

    TEST_ASSERT(st_short == W25Q_BUSY_TIMEOUT, "WaitForReady(1 ms): %s (attendu TIMEOUT)", state_str(st_short));
    TEST_ASSERT(st_long == W25Q_OK, "WaitForReady(%u ms): %s", (unsigned)W25Q_TEST_TIMEOUT_MS, state_str(st_long));
    TEST_ASSERT(!W25Q_STATUS_REG(w25q, W25Q_SR1_BUSY_BIT), "BUSY=1 apres WaitForReady OK");
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "1 ms -> TIMEOUT, %u ms -> OK", (unsigned)W25Q_TEST_TIMEOUT_MS);
}


/* ======================= G. Cas limites R/W ============================ */

void W25Q_seq_test_t22_write_zero_size(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_SEC9, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0x96 ^ i);
    st = W25Q_WriteData(w25q, buf_a, ADDR_SEC9, 0U, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write(size=0): %s (attendu OK)", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_SEC9, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Read: %s", state_str(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, PAGE_BYTES, 0xFF, ADDR_SEC9, &first_addr, &first_got),
                "Zone modifiee @0x%06lX got=0x%02X (attendu 0xFF)",
                (unsigned long)first_addr, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "W25Q_OK, zone intacte (0xFF)");
}

void W25Q_seq_test_t23_write_end_clamp(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_NEAR_END, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_a[i] = (uint8_t)(0x7E ^ i);
    /* Tente d'ecrire 256 B alors qu'il ne reste que 128 B disponibles */
    st = W25Q_WriteData(w25q, buf_a, ADDR_NEAR_END, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Write: %s (attendu OK)", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, ADDR_NEAR_END, 128U, W25Q_TEST_TIMEOUT_MS);
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

void W25Q_seq_test_t24_read_end_clamp(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmdAddr(w25q, W25Q_SECTOR_ERASE_4B, ADDR_NEAR_END, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Erase: %s", state_str(st));
    for (uint32_t i = 0; i < PAGE_BYTES; i++) buf_b[i] = SENTINEL;
    /* Demande 256 B alors qu'il n'en reste que 128 avant la fin du flash */
    st = W25Q_ReadData(w25q, buf_b, ADDR_NEAR_END, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "Read: %s (attendu OK)", state_str(st));
    uint32_t first_addr; uint8_t first_got;
    TEST_ASSERT(verify_uniform(buf_b, 128U, 0xFF, ADDR_NEAR_END, &first_addr, &first_got),
                "Partie valide non-0xFF @0x%06lX got=0x%02X", (unsigned long)first_addr, first_got);
    TEST_ASSERT(verify_uniform(buf_b + 128U, 128U, SENTINEL, 128U, &first_addr, &first_got),
                "Buffer ecrase apres la limite, off=%lu got=0x%02X", (unsigned long)first_addr, first_got);
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "128/256B lus @0x%06lX, reste du buffer intact", (unsigned long)ADDR_NEAR_END);
}

void W25Q_seq_test_t25_addr_out_of_range(TEST_case_t *tc) {
    /* addr == taille du flash : hors plage, doit etre rejete sans toucher le bus */
    W25Q_STATE st = W25Q_WriteData(w25q, buf_a, W25Q_FLASH_SIZE_BYTES, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "Write hors plage: %s (attendu PARAM_ERR)", state_str(st));
    st = W25Q_ReadData(w25q, buf_b, W25Q_FLASH_SIZE_BYTES, PAGE_BYTES, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_PARAM_ERR, "Read hors plage: %s (attendu PARAM_ERR)", state_str(st));
    /* Le driver doit rester utilisable apres un rejet (CS relache, bus libre) */
    uint8_t id[3] = {0};
    st = W25Q_ReadID(w25q, id);
    TEST_ASSERT(st == W25Q_OK, "ReadID apres rejet: %s", state_str(st));
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail), "addr=taille flash -> PARAM_ERR (R et W)");
}


/* ======================= H. Reset ====================================== */

void W25Q_seq_test_t26_soft_reset(TEST_case_t *tc) {
    W25Q_STATE st = W25Q_SendCmd(w25q, W25Q_ENABLE_RESET, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "ENABLE_RESET: %s", state_str(st));
    st = W25Q_SendCmd(w25q, W25Q_RESET, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(st == W25Q_OK, "RESET: %s", state_str(st));
    /* Le driver doit avoir resynchronise son cache : ADS suivi = ADS relu */
    int ads_tracked = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    st = read_all_status();
    TEST_ASSERT(st == W25Q_OK, "ReadStatus post-reset: %s", state_str(st));
    TEST_ASSERT(ads_tracked == W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT),
                "ADS suivi=%d != relu=%d apres reset", ads_tracked, W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT));
    uint8_t id[3] = {0};
    st = W25Q_ReadID(w25q, id);
    TEST_ASSERT(st == W25Q_OK, "ReadID post-reset: %s", state_str(st));
    uint16_t dev = (uint16_t)((id[1] << 8) | id[2]);
    TEST_ASSERT(id[0] == W25Q_MANUFACTURER_ID && dev == W25Q_V_FULL_DEVICE_ID,
                "Post-reset: Manuf=0x%02X Dev=0x%04X", id[0], dev);
    int ads = W25Q_STATUS_REG(w25q, W25Q_SR3_ADS_BIT);
    int adp = W25Q_STATUS_REG(w25q, W25Q_SR3_ADP_BIT);
    /* Le reset efface ADS et les ecritures volatiles : re-applique la config */
    W25Q_STATE st_cfg = W25Q_Init(w25q, w25q->config, W25Q_TEST_TIMEOUT_MS);
    TEST_ASSERT(ads == adp, "Post-reset: ADS=%d != ADP=%d", ads, adp);
    TEST_ASSERT(st_cfg == W25Q_OK, "Re-init post-reset: %s", state_str(st_cfg));
    tc->result = R_PASS;
    snprintf(tc->detail, sizeof(tc->detail),
             "ID OK post-reset, ADS=ADP=%d, config re-appliquee", ads);
}
