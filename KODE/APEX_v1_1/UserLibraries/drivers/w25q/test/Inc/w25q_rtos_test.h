#ifndef W25Q_RTOS_TEST_H
#define W25Q_RTOS_TEST_H

#include "w25q_rtos.h"
#include "test.h"

#if (APEX_CFG_SCHED_RTOS == 1)

#define W25Q_rtos_test_N_TESTS 39

extern TEST_case_table_t W25Q_rtos_test_cases[W25Q_rtos_test_N_TESTS];

/* ========================================================================
 * Contexte d'execution
 *   - Depuis un thread, kernel demarre, a la priorite osPriorityNormal :
 *       W25Q_rtos_test_init_chip(), W25Q_rtos_test_set_context(),
 *       puis TEST_perform_cases().
 *   - La suite declare ses propres pools (TASK_POOL dans le .c), y compris
 *     ceux des taches du driver : le projet n'en declare aucun. Le .c n'est
 *     compile qu'avec APEX_CFG_PROFILE_TEST et APEX_ENABLE_W25Q512 a 1.
 *   - Seule l'API RTOS est appelee, helpers compris (pas de HAL bloquant).
 *   - Les primitives de niveau 1 sont appelees directement depuis le thread
 *     de test ; Init / Write / Read passent par leur tache (seule forme
 *     disponible) ; SendCmd / SendCmdAddr sont exerces sous les deux formes.
 *   - Tout ce qu'un job lit ou ecrit (buffers, ret) est statique : un
 *     TEST_ASSERT en echec rend la main avant la fin des jobs.
 *   - Chaque cas, meme en echec, attend que les pools se vident, puis
 *     verifie que le semaphore de la puce est libre (count = 1) : une fuite
 *     de semaphore fait echouer le cas qui l'a causee.
 *   - Les joins ont tous un timeout : un blocage apparait comme un echec au
 *     lieu de geler la suite.
 * ======================================================================== */

/* Initialise la puce par la tache W25Q_Init (pool de la suite). Cree le
   semaphore de la puce au premier appel ; la W25Q_t doit etre a zero. */
W25Q_STATE W25Q_rtos_test_init_chip(W25Q_t *w25q, const W25Q_config_t *config, uint32_t timeout_ms);

void W25Q_rtos_test_set_context(W25Q_t *w25q);

/*
 * T0-T26 : transposition 1:1 de la suite sequentielle (memes zones, memes
 * motifs, memes verifications ; voir w25q_seq_test.h pour le detail).
 * T27-T38 : propre a la couche RTOS.
 *
 *   A. Communication          T0  - T1
 *   B. Configuration (Init)   T2  - T5
 *   C. Primitives commande    T6  - T8
 *   D. Effacement secteur     T9
 *   E. Lecture / ecriture     T10 - T16
 *   F. Effacement etendu      T17 - T21
 *   G. Cas limites R/W        T22 - T25
 *   H. Reset                  T26        (re-applique la config ensuite)
 *   I. Semaphore de la puce   T27 - T31
 *   J. Integration scheduler  T32 - T33
 *   K. Concurrence            T34 - T37
 *   L. Ressources             T38        (en dernier : mesure apres tout)
 *
 * Non couverts volontairement (en plus de ceux de la suite sequentielle) :
 *   - Ecritures concurrentes sur des zones qui se chevauchent : W25Q_Write
 *     n'est atomique qu'a la page, le resultat est indefini par conception.
 *   - Partage du bus SPI avec un autre composant (SPI_Begin_DMA_RTOS attend
 *     encore le bus sans timeout).
 *   - Erreurs DMA / interruption perdue : non provocables sans injection.
 */

/* ======================= A. Communication ============================== */

/* T0 – Identifiant JEDEC (W25Q_ReadID_RTOS) */
void W25Q_rtos_test_t0_id_check(TEST_case_t *tc);

/* T1 – Registres de statut (W25Q_ReadStatus_RTOS), index 0/4 rejetes */
void W25Q_rtos_test_t1_read_status(TEST_case_t *tc);

/* ======================= B. Configuration ============================== */

/* T2 – Traduction config -> registres (W25Q_ConfigToStatus, sans materiel) */
void W25Q_rtos_test_t2_cfg_to_status(TEST_case_t *tc);

/* T3 – Config appliquee par la tache W25Q_Init (re-init idempotent) */
void W25Q_rtos_test_t3_cfg_applied(TEST_case_t *tc);

/* T4 – Config invalide rejetee par W25Q_Init, chip->config intacte */
void W25Q_rtos_test_t4_cfg_invalid(TEST_case_t *tc);

/* T5 – Config zero-initialisee : SR1-3 inchanges */
void W25Q_rtos_test_t5_cfg_keep(TEST_case_t *tc);

/* ======================= C. Primitives commande ======================== */

/* T6 – Parametres invalides des primitives -> W25Q_PARAM_ERR */
void W25Q_rtos_test_t6_cmd_invalid(TEST_case_t *tc);

/* T7 – Write Enable Latch (06h / 04h), via la tache W25Q_SendCmd */
void W25Q_rtos_test_t7_wel(TEST_case_t *tc);

/* T8 – Suivi d'ADS et longueur d'adresse de SendCmdAddr */
void W25Q_rtos_test_t8_addr_mode_tracking(TEST_case_t *tc);

/* ======================= D. Effacement secteur ========================= */

/* T9 – Effacement secteur 4 KB, via la tache W25Q_SendCmdAddr */
void W25Q_rtos_test_t9_erase_verify(TEST_case_t *tc);

/* ======================= E. Lecture / ecriture ========================= */

/* T10 – R/W aligne : 1 page complete */
void W25Q_rtos_test_t10_aligned_rw(TEST_case_t *tc);

/* T11 – R/W a cheval sur deux pages */
void W25Q_rtos_test_t11_cross_page_rw(TEST_case_t *tc);

/* T12 – R/W a cheval sur deux secteurs */
void W25Q_rtos_test_t12_cross_sector_rw(TEST_case_t *tc);

/* T13 – R/W a une adresse non alignee, voisinage intact */
void W25Q_rtos_test_t13_unaligned_rw(TEST_case_t *tc);

/* T14 – R/W multi-secteurs (17 pages) */
void W25Q_rtos_test_t14_multi_sector_rw(TEST_case_t *tc);

/* T15 – Ecriture sans effacement prealable (AND NOR) */
void W25Q_rtos_test_t15_and_behavior(TEST_case_t *tc);

/* T16 – R/W en mode 3 octets au-dela de 16 MB */
void W25Q_rtos_test_t16_rw_3b_mode(TEST_case_t *tc);

/* ======================= F. Effacement etendu ========================== */

/* T17 – Longueur d'adresse effective de 20h en 3B et 4B */
void W25Q_rtos_test_t17_erase_addr_len(TEST_case_t *tc);

/* T18 – Effacement 32 KB (52h), via la tache W25Q_SendCmdAddr */
void W25Q_rtos_test_t18_erase_32kb(TEST_case_t *tc);

/* T19 – Effacement 64 KB (DCh), via la tache W25Q_SendCmdAddr */
void W25Q_rtos_test_t19_erase_64kb(TEST_case_t *tc);

/* T20 – Isolation lors d'un effacement de secteur */
void W25Q_rtos_test_t20_sector_isolation(TEST_case_t *tc);

/* T21 – Timeout BUSY (W25Q_WaitForReady_RTOS) */
void W25Q_rtos_test_t21_busy_timeout(TEST_case_t *tc);

/* ======================= G. Cas limites R/W ============================ */

/* T22 – Write avec taille zero -> W25Q_OK, flash intacte */
void W25Q_rtos_test_t22_write_zero_size(TEST_case_t *tc);

/* T23 – Ecriture a cheval sur la fin de la memoire (clamp) */
void W25Q_rtos_test_t23_write_end_clamp(TEST_case_t *tc);

/* T24 – Lecture a cheval sur la fin de la memoire (clamp, sentinelle) */
void W25Q_rtos_test_t24_read_end_clamp(TEST_case_t *tc);

/* T25 – Adresse hors plage -> W25Q_PARAM_ERR (Write et Read) */
void W25Q_rtos_test_t25_addr_out_of_range(TEST_case_t *tc);

/* ======================= H. Reset ====================================== */

/* T26 – Reset logiciel (66h puis 99h, via la tache W25Q_SendCmd) */
void W25Q_rtos_test_t26_soft_reset(TEST_case_t *tc);

/* ======================= I. Semaphore de la puce ======================= */

/* T27 – Semaphore rendu sur les chemins d'erreur
 *   Puce occupee par un effacement 64 KB : WaitForReady, SendCmdAddr,
 *   PageProgram et la tache Read, avec 1 ms de timeout, rendent
 *   W25Q_BUSY_TIMEOUT et laissent le semaphore libre (count = 1). */
void W25Q_rtos_test_t27_lock_error_paths(TEST_case_t *tc);

/* T28 – Variantes _NoLock sous semaphore deja detenu
 *   Le thread detient le semaphore : la version verrouillee rend
 *   W25Q_LOCK_TIMEOUT (immediatement avec 0, apres ~20 ms avec 20), les
 *   versions _NoLock fonctionnent sans s'auto-bloquer. */
void W25Q_rtos_test_t28_nolock_held(TEST_case_t *tc);

/* T29 – Timeout du lock face a un autre job
 *   Un job detient le semaphore 200 ms : ReadID(50 ms) et la tache Read
 *   (20 ms) rendent W25Q_LOCK_TIMEOUT dans le delai ; ReadID(1000 ms)
 *   reussit des que le semaphore est rendu. */
void W25Q_rtos_test_t29_lock_timeout(TEST_case_t *tc);

/* T30 – Puce non initialisee (sem_id NULL)
 *   Primitives verrouillees et tache Read -> W25Q_SEM_ERR ; Init avec chip
 *   NULL ou cs_bank NULL -> W25Q_PARAM_ERR ; la vraie puce reste utilisable. */
void W25Q_rtos_test_t30_uninit_chip(TEST_case_t *tc);

/* T31 – Re-init pendant une ecriture
 *   Init (priorite haute) lance pendant un Write de 17 pages : les deux
 *   reussissent, le semaphore n'est pas recree (meme sem_id), les donnees
 *   ecrites sont exactes. */
void W25Q_rtos_test_t31_reinit_concurrent(TEST_case_t *tc);

/* ======================= J. Integration scheduler ====================== */

/* T32 – Arguments copies, job detache
 *   Write lance detache (ret seul) pendant qu'un job detient le semaphore ;
 *   la struct d'arguments modifiee apres le spawn est sans effet, la copie
 *   mutee par le corps ne remonte pas a l'appelant, ret est publie. */
void W25Q_rtos_test_t32_args_detached(TEST_case_t *tc);

/* T33 – Join expire sur un job long
 *   Read lance pendant un effacement 64 KB : join(5 ms) -> osErrorTimeout,
 *   job toujours actif et ret non publie ; join suivant -> osOK, ret = OK,
 *   donnees effacees. */
void W25Q_rtos_test_t33_join_timeout(TEST_case_t *tc);

/* ======================= K. Concurrence ================================ */

/* T34 – 4 Write simultanes sur des zones disjointes, 4 priorites
 *   Adresses non alignees (5 pages chacun) : toutes les donnees exactes. */
void W25Q_rtos_test_t34_concurrent_writes(TEST_case_t *tc);

/* T35 – 2 effacements simultanes (2 taches SendCmdAddr)
 *   Le second attend la fin du premier par son controle BUSY : les deux
 *   secteurs sont vierges. */
void W25Q_rtos_test_t35_concurrent_erases(TEST_case_t *tc);

/* T36 – Lectures prioritaires pendant une ecriture longue
 *   Lectures repetees d'une autre zone tant que le Write de 17 pages
 *   tourne : toutes exactes, ecriture exacte. */
void W25Q_rtos_test_t36_read_during_write(TEST_case_t *tc);

/* T37 – WaitForReady_RTOS ne monopolise pas le semaphore
 *   Un job attend la fin d'un effacement 64 KB ; pendant ce temps le thread
 *   de test lit SR1 sans jamais expirer sur le lock, et voit BUSY = 1. */
void W25Q_rtos_test_t37_wait_shares_lock(TEST_case_t *tc);

/* ======================= L. Ressources ================================= */

/* T38 – Marge de pile des pools de la suite
 *   task_stack_headroom de chaque instance creee >= 128 B. Le minimum est
 *   rapporte pour dimensionner TASK_POOL_SZ. */
void W25Q_rtos_test_t38_stack_headroom(TEST_case_t *tc);

#endif /* APEX_CFG_SCHED_RTOS == 1 */
#endif /* W25Q_RTOS_TEST_H */
