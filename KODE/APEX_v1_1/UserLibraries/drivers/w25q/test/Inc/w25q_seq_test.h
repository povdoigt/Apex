#ifndef W25Q_SEQ_TEST_H
#define W25Q_SEQ_TEST_H

#include "w25q.h"
#include "test.h"

#define W25Q_seq_test_N_TESTS 15

extern TEST_case_table_t W25Q_seq_test_cases[W25Q_seq_test_N_TESTS];

void W25Q_seq_test_set_context(W25Q_t *w25q);



/* ========================================================================
 * T0 – Identificateur chip (JEDEC ID)
 *   Verifie Manufacturer ID (0xEF) + Device ID (0x4020 pour JV 512 Mbit).
 * ======================================================================== */
void W25Q_seq_test_t0_id_check(TEST_case_t *tc);

/* ========================================================================
 * T1 – Verification de l'effacement
 *   Efface sec0, lit 256 B -> tous doivent valoir 0xFF.
 *   Detecte les pannes d'erase partiel ou d'adressage incorrect.
 * ======================================================================== */
void W25Q_seq_test_t1_erase_verify(TEST_case_t *tc);

/* ========================================================================
 * T2 – Lecture / ecriture alignee (1 page complete, adresse page-alignee)
 *   Cas de base : addr = debut de page, taille = exactement 1 page.
 * ======================================================================== */
void W25Q_seq_test_t2_aligned_rw(TEST_case_t *tc);

/* ========================================================================
 * T3 – Ecriture / lecture chevauchant deux pages consecutives
 *   128 B a 0x0027C0 : 64 B en page 7 + 64 B en page 8 du sec2.
 *   WriteData doit decouper en 2 PageProgram. Detecte les bugs de
 *   wrap-around dans la logique de relative_addr.
 * ======================================================================== */
void W25Q_seq_test_t3_cross_page_rw(TEST_case_t *tc);

/* ========================================================================
 * T4 – Ecriture / lecture chevauchant deux secteurs consecutifs
 *   32 B a 0x003FF0 : 16 B en fin de sec3 + 16 B en debut de sec4.
 *   Detecte les problemes d'adressage a la frontiere de secteur.
 * ======================================================================== */
void W25Q_seq_test_t4_cross_sector_rw(TEST_case_t *tc);

/* ========================================================================
 * T5 – Isolation lors d'un effacement de secteur
 *   1. Efface sec5 et sec6
 *   2. Ecrit 0xAA sur les 16 derniers B de sec5 (0x005FF0)
 *   3. Ecrit 0x55 sur les 16 premiers B de sec6 (0x006000)
 *   4. Re-efface sec5 uniquement
 *   5. Verifie : fin sec5 = 0xFF, debut sec6 = 0x55 (intact)
 *   Critere : l'effacement d'un secteur ne doit pas corrompre son voisin.
 * ======================================================================== */
void W25Q_seq_test_t5_sector_isolation(TEST_case_t *tc);

/* ========================================================================
 * T6 – Comportement AND (ecriture sans effacement prealable)
 *   Flash NOR : un bit ne peut passer de 0->1 que par un effacement.
 *   Ecrire sur une cellule deja programmee applique un AND bit a bit :
 *     0xFF -> 0x0F : OK (1->0 autorise)
 *     0x0F -> 0xF0 : donne 0x0F & 0xF0 = 0x00 (sans re-effacement)
 *   Verifie que le driver n'ajoute pas d'effacement implicite.
 * ======================================================================== */
void W25Q_seq_test_t6_and_behavior(TEST_case_t *tc);

/* ========================================================================
 * T7 – Ecriture a une adresse non alignee sur une page (addr % 256 != 0)
 *   100 B a ADDR_SEC8+50 (offset 50 non multiple de 256).
 *   Lit ensuite 200 B depuis le debut du secteur et controle :
 *   - octets [0..49]    (avant zone) : 0xFF non touches
 *   - octets [50..149]  (zone ecrite): correspond au motif
 *   - octets [150..199] (apres zone) : 0xFF non touches
 * ======================================================================== */
void W25Q_seq_test_t7_unaligned_rw(TEST_case_t *tc);

/* ========================================================================
 * T8 – Lecture des registres de statut (W25Q_ReadStatus)
 *   Lit SR1, SR2, SR3 et verifie le bit ADS (W25Q_SR3_ADS_BIT = 16).
 *   Confirme que W25Q_Init a configure le mode 4-byte (ADS=1).
 * ======================================================================== */
void W25Q_seq_test_t8_read_status(TEST_case_t *tc);

/* ========================================================================
 * T9 – Effacement 32 KB (W25Q_32KB_BLOCK_ERASE, cmd 0x52)
 *   Ecrit un motif sur la premiere page du bloc 32 KB a ADDR_BLK32,
 *   efface le bloc entier (32 KB) puis verifie que 256 B = 0xFF.
 * ======================================================================== */
void W25Q_seq_test_t9_erase_32kb(TEST_case_t *tc);

/* ========================================================================
 * T10 – Effacement 64 KB (W25Q_64KB_BLOCK_ERASE_4B, cmd 0xDC)
 *   Variante 4-byte du 64 KB block erase. Meme logique que T9.
 * ======================================================================== */
void W25Q_seq_test_t10_erase_64kb(TEST_case_t *tc);

/* ========================================================================
 * T11 – Reset logiciel (ENABLE_RESET puis RESET)
 *   Verifie que le chip repond correctement apres la sequence de reset :
 *   - L'identifiant JEDEC est intact.
 *   - Le bit ADS reste a 1 (ADP=1 => mode 4-byte restaure apres reset).
 * ======================================================================== */
void W25Q_seq_test_t11_soft_reset(TEST_case_t *tc);

/* ========================================================================
 * T12 – Ecriture multi-secteurs (>4 KB, 17 pages = 2 secteurs)
 *   Ecrit SIZE_LARGE (4352 B) a ADDR_MULTI sur 2 secteurs consecutifs.
 *   Teste que la boucle interne de WriteData gere correctement >16 appels
 *   successifs a PageProgram sur plusieurs secteurs.
 * ======================================================================== */
void W25Q_seq_test_t12_multi_sector_rw(TEST_case_t *tc);

/* ========================================================================
 * T13 – Ecriture a cheval sur la limite de la memoire (clamp silencieux)
 *   WriteData avec addr=(taille_flash - 128) et size=256 :
 *   le driver doit limiter silencieusement a 128 B et retourner W25Q_OK.
 *   Verifie que les 128 B ecrits correspondent au debut du motif.
 * ======================================================================== */
void W25Q_seq_test_t13_end_of_flash_clamp(TEST_case_t *tc);

/* ========================================================================
 * T14 – WriteData avec taille zero (cas limite)
 *   Appelle WriteData(size=0) : doit retourner W25Q_OK sans toucher le
 *   flash. Utilise ADDR_SEC0 deja efface par T1 et non ecrit depuis.
 * ======================================================================== */
void W25Q_seq_test_t14_write_zero_size(TEST_case_t *tc);

#endif /* W25Q_SEQ_TEST_H */