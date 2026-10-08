#ifndef W25Q_SEQ_TEST_H
#define W25Q_SEQ_TEST_H

#include "w25q.h"
#include "test.h"

#define W25Q_seq_test_N_TESTS 27

extern TEST_case_table_t W25Q_seq_test_cases[W25Q_seq_test_N_TESTS];

void W25Q_seq_test_set_context(W25Q_t *w25q);

/*
 * Ordre des tests : du plus bas niveau au plus haut. Chaque test ne s'appuie
 * que sur des fonctions deja validees par les tests precedents, de sorte que
 * le premier echec designe la couche fautive.
 *
 *   A. Communication          T0  - T1
 *   B. Configuration (Init)   T2  - T5
 *   C. Primitives commande    T6  - T8   (dont suivi d'ADS et longueur
 *                                         d'adresse de SendCmdAddr)
 *   D. Effacement secteur     T9
 *   E. Lecture / ecriture     T10 - T16
 *   F. Effacement etendu      T17 - T21
 *   G. Cas limites R/W        T22 - T25
 *   H. Reset                  T26        (re-initialise la puce : en dernier)
 *
 * Chaque test qui ecrit utilise une zone dediee et l'efface lui-meme :
 * aucun test ne depend du contenu laisse par un autre.
 *
 * Non couverts volontairement :
 *   - CHIP_ERASE (C7h) : ~400 s et efface toute la puce.
 *   - Protection memoire (BP/TB/CMP, WPS) et power-down (B9h/ABh).
 *   - Suspend / resume (75h / 7Ah), registres de securite (42h/44h/48h),
 *     verrouillage individuel (WPS=1, 36h/39h) : non exposes par l'API.
 *   - ReadData > 64 KB (decoupage interne en blocs de 0xFFFF) : necessite un
 *     buffer plus grand que la RAM disponible.
 */

/* ======================= A. Communication ============================== */

/* T0 – Identifiant JEDEC (W25Q_ReadID)
 *   Manufacturer ID (0xEF) + Device ID (0x4020 pour JV 512 Mbit). */
void W25Q_seq_test_t0_id_check(TEST_case_t *tc);

/* T1 – Lecture des registres de statut (W25Q_ReadStatus)
 *   SR1, SR2, SR3 lisibles, BUSY=0 au repos.
 *   Index invalides (0 et 4) -> W25Q_PARAM_ERR. */
void W25Q_seq_test_t1_read_status(TEST_case_t *tc);

/* ======================= B. Configuration ============================== */

/* T2 – Traduction config -> registres (W25Q_ConfigToStatus, sans materiel)
 *   Config zero -> masque vide. Config complete -> masque/valeur attendus
 *   bit a bit. addr_mode n'apparait pas dans le masque (ADS en lecture
 *   seule, applique par commande). */
void W25Q_seq_test_t2_cfg_to_status(TEST_case_t *tc);

/* T3 – Config appliquee par W25Q_Init
 *   Re-init avec la config courante -> W25Q_OK (idempotent). Les bits de
 *   SR1-3 couverts par la config et ADS correspondent a chip->config. */
void W25Q_seq_test_t3_cfg_applied(TEST_case_t *tc);

/* T4 – Config invalide rejetee par W25Q_Init
 *   addr_mode hors enum, block_protect > BP(15), hspi NULL :
 *   W25Q_PARAM_ERR sans modifier chip->config, driver toujours utilisable. */
void W25Q_seq_test_t4_cfg_invalid(TEST_case_t *tc);

/* T5 – Config zero-initialisee (tout W25Q_CFG_KEEP)
 *   W25Q_Init avec reg = {0} -> W25Q_OK sans modifier SR1-3 (hors
 *   BUSY/WEL). Restaure la config d'origine. */
void W25Q_seq_test_t5_cfg_keep(TEST_case_t *tc);

/* ======================= C. Primitives commande ======================== */

/* T6 – Parametres invalides des primitives
 *   SendCmd / SendCmdAddr avec un opcode absent de W25Q_CMD_FLAGS,
 *   WriteStatus avec un index invalide -> W25Q_PARAM_ERR, rien n'est
 *   emis sur le bus (ReadID OK ensuite). */
void W25Q_seq_test_t6_cmd_invalid(TEST_case_t *tc);

/* T7 – Write Enable Latch (06h / 04h)
 *   WRITE_ENABLE -> WEL=1, WRITE_DISABLE -> WEL=0. */
void W25Q_seq_test_t7_wel(TEST_case_t *tc);

/* T8 – Suivi d'ADS et longueur d'adresse de SendCmdAddr (sans donnees)
 *   ADS fixe la longueur d'adresse de SendCmdAddr : son suivi dans
 *   chip->status_reg doit etre exact avant tout effacement.
 *   Re-init en 3B -> ADS=0, puis :
 *   - WriteStatus(SR3) avec ADS=1 : ADS suivi et relu restent a 0
 *     (bit en lecture seule) ;
 *   - SendCmdAddr(20h, >16 MB) -> W25Q_PARAM_ERR (3 octets d'adresse) ;
 *   - SendCmdAddr(21h, >16 MB) -> W25Q_OK (opcode 4-byte, non limite) ;
 *   - SendCmd(B7h) puis SendCmd(E9h) : ADS suivi = commande = ADS relu.
 *   Restaure la config avant les verifications. */
void W25Q_seq_test_t8_addr_mode_tracking(TEST_case_t *tc);

/* ======================= D. Effacement secteur ========================= */

/* T9 – Effacement secteur 4 KB (W25Q_SECTOR_ERASE_4B, 21h)
 *   Efface sec0, lit 256 B -> tous a 0xFF. */
void W25Q_seq_test_t9_erase_verify(TEST_case_t *tc);

/* ======================= E. Lecture / ecriture ========================= */

/* T10 – R/W aligne : 1 page complete a une adresse page-alignee. */
void W25Q_seq_test_t10_aligned_rw(TEST_case_t *tc);

/* T11 – R/W a cheval sur deux pages (128 B a 0x0027C0)
 *   WriteData doit decouper en 2 PageProgram. */
void W25Q_seq_test_t11_cross_page_rw(TEST_case_t *tc);

/* T12 – R/W a cheval sur deux secteurs (32 B a 0x003FF0). */
void W25Q_seq_test_t12_cross_sector_rw(TEST_case_t *tc);

/* T13 – R/W a une adresse non alignee (100 B a +50 dans sec8)
 *   Les octets avant et apres la zone ecrite restent a 0xFF. */
void W25Q_seq_test_t13_unaligned_rw(TEST_case_t *tc);

/* T14 – R/W multi-secteurs (17 pages = 4352 B sur 2 secteurs). */
void W25Q_seq_test_t14_multi_sector_rw(TEST_case_t *tc);

/* T15 – Ecriture sans effacement prealable (comportement AND NOR)
 *   0xFF -> 0x0F -> 0xF0 donne 0x00 : pas d'effacement implicite. */
void W25Q_seq_test_t15_and_behavior(TEST_case_t *tc);

/* T16 – R/W en mode 3 octets au-dela de 16 MB
 *   WriteData / ReadData utilisent des opcodes 4-byte (12h / 13h) :
 *   motif ecrit moitie en 4B, moitie en 3B, relu identique dans les deux
 *   modes. Restaure la config avant les verifications. */
void W25Q_seq_test_t16_rw_3b_mode(TEST_case_t *tc);

/* ======================= F. Effacement etendu ========================== */

/* T17 – Longueur d'adresse effective d'un opcode dependant du mode (20h)
 *   En 3B puis en 4B : motif ecrit, SendCmdAddr(20h) -> le secteur vise
 *   (et pas un autre) est efface. Valide l'envoi de 3 ou 4 octets
 *   d'adresse, utilise ensuite par 52h. Restaure la config. */
void W25Q_seq_test_t17_erase_addr_len(TEST_case_t *tc);

/* T18 – Effacement 32 KB (W25Q_32KB_BLOCK_ERASE, 52h)
 *   52h n'a pas de variante 4-byte : adresse dimensionnee selon ADS. */
void W25Q_seq_test_t18_erase_32kb(TEST_case_t *tc);

/* T19 – Effacement 64 KB (W25Q_64KB_BLOCK_ERASE_4B, DCh). */
void W25Q_seq_test_t19_erase_64kb(TEST_case_t *tc);

/* T20 – Isolation lors d'un effacement de secteur
 *   Re-effacer sec5 ne doit pas toucher les premiers octets de sec6. */
void W25Q_seq_test_t20_sector_isolation(TEST_case_t *tc);

/* T21 – Timeout BUSY (W25Q_WaitForReady)
 *   SendCmdAddr(erase) rend la main sans attendre la fin : WaitForReady
 *   avec 1 ms -> W25Q_BUSY_TIMEOUT, puis avec le timeout nominal -> OK. */
void W25Q_seq_test_t21_busy_timeout(TEST_case_t *tc);

/* ======================= G. Cas limites R/W ============================ */

/* T22 – WriteData avec taille zero -> W25Q_OK, flash intacte. */
void W25Q_seq_test_t22_write_zero_size(TEST_case_t *tc);

/* T23 – Ecriture a cheval sur la fin de la memoire
 *   addr = taille - 128, size = 256 -> clamp silencieux a 128 B, W25Q_OK. */
void W25Q_seq_test_t23_write_end_clamp(TEST_case_t *tc);

/* T24 – Lecture a cheval sur la fin de la memoire
 *   Clamp a 128 B, le reste du buffer (sentinelle) n'est pas touche. */
void W25Q_seq_test_t24_read_end_clamp(TEST_case_t *tc);

/* T25 – Adresse hors plage (addr >= taille du flash)
 *   WriteData et ReadData -> W25Q_PARAM_ERR, driver toujours utilisable. */
void W25Q_seq_test_t25_addr_out_of_range(TEST_case_t *tc);

/* ======================= H. Reset ====================================== */

/* T26 – Reset logiciel (66h puis 99h)
 *   ID intact. ADS = ADP apres reset, et le cache status_reg du driver
 *   est resynchronise (ADS suivi = ADS relu). Re-applique la config
 *   ensuite (le reset efface ADS et les ecritures volatiles des SR). */
void W25Q_seq_test_t26_soft_reset(TEST_case_t *tc);

#endif /* W25Q_SEQ_TEST_H */
