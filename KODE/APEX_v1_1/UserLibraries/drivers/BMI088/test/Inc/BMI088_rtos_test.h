#ifndef BMI088_RTOS_TEST_H
#define BMI088_RTOS_TEST_H

#include "BMI088_rtos.h"
#include "test.h"

#if (APEX_CFG_SCHED_RTOS == 1)

#define BMI088_rtos_test_N_TESTS 23

extern TEST_case_table_t BMI088_rtos_test_cases[BMI088_rtos_test_N_TESTS];

/* ========================================================================
 * Contexte d'execution
 *   - Depuis un thread, kernel demarre, a la priorite osPriorityNormal :
 *       BMI088_rtos_test_init_imu(), BMI088_rtos_test_set_context(),
 *       puis TEST_perform_cases().
 *   - La suite declare ses propres pools (TASK_POOL dans le .c), y compris
 *     ceux des taches du driver : le projet n'en declare aucun. Le .c n'est
 *     compile qu'avec APEX_CFG_PROFILE_TEST et APEX_ENABLE_BMI088 a 1, et
 *     sans APEX_CFG_BMI088_BENCH (le banc definit les memes pools).
 *   - Seule l'API RTOS est appelee, helpers compris (pas de HAL bloquant).
 *   - Carte immobile pendant la suite : plusieurs cas verifient que
 *     l'acceleration vaut ~1 g et que la rotation est faible.
 *   - Tout ce qu'un job lit ou ecrit (stats, topics, ret) est statique : un
 *     TEST_ASSERT en echec rend la main avant la fin des jobs.
 *   - Chaque cas, meme en echec, arrete l'acquisition qu'il a lancee, attend
 *     que les pools se vident, puis verifie que le semaphore du capteur est
 *     libre (count = 1) : une fuite fait echouer le cas qui l'a causee.
 *   - Les joins ont tous un timeout : un blocage apparait comme un echec au
 *     lieu de geler la suite.
 * ======================================================================== */

/* Initialise le capteur par la tache BMI088_Init (pool de la suite). Cree le
   semaphore au premier appel ; le bmi088_t doit etre a zero. */
BMI_STATE BMI088_rtos_test_init_imu(bmi088_t *imu, const bmi_config_t *config, uint32_t timeout_ms);

void BMI088_rtos_test_set_context(bmi088_t *imu, const bmi_config_t *config);

/*
 * T0-T7 : transposition 1:1 de la suite sequentielle (memes configurations,
 * memes criteres ; voir BMI088_seq_test.h pour le detail).
 * T8-T22 : propre a la couche RTOS.
 *
 *   A. Suite sequentielle transposee   T0  - T7
 *   B. Init et parametres              T8  - T10
 *   C. Semaphore du capteur            T11 - T12
 *   D. Tache BMI088_Acquire            T13 - T17
 *   E. Concurrence                     T18 - T21
 *   F. Ressources                      T22   (en dernier : mesure apres tout)
 *
 * Non couverts volontairement :
 *   - Partage du bus SPI1 avec un autre composant (ADXL375, SX127x) : aucun
 *     n'est actif dans ce projet, et SPI_Begin_DMA_RTOS attend encore le bus
 *     sans timeout.
 *   - Erreurs SPI / DMA / interruption perdue : non provocables sans
 *     injection. Le nettoyage verifie toutefois le semaphore apres chaque cas.
 *   - Transfert de BMI_ACC_SELF_TEST en mode suspend (450 us entre deux
 *     ecritures) : le driver evite ce cas en repassant l'ACC actif d'abord.
 */

/* ======================= A. Suite sequentielle transposee =============== */

/* T0 – Identifiants ACC 0x1E / GYR 0x0F (BMI088_ReadID_RTOS) */
void BMI088_rtos_test_t0_chip_ids(TEST_case_t *tc);

/* T1 – Soft reset ACC (BMI088_SoftReset_RTOS), ID relu, config reappliquee
 *      par la tache BMI088_ApplyConfig */
void BMI088_rtos_test_t1_acc_soft_reset(TEST_case_t *tc);

/* T2 – Soft reset GYR, idem */
void BMI088_rtos_test_t2_gyr_soft_reset(TEST_case_t *tc);

/* T3 – Config ACC ecrite par la tache BMI088_ApplyConfig, relue registre par
 *      registre (BMI088_ReadRegister_RTOS), puis restauree */
void BMI088_rtos_test_t3_acc_config_rw(TEST_case_t *tc);

/* T4 – Config GYR ecrite par BMI088_ApplyConfig_RTOS (appel direct), relue */
void BMI088_rtos_test_t4_gyr_config_rw(TEST_case_t *tc);

/* T5 – Self-test ACC (Bosch AN §4.4.1), attentes en osDelay */
void BMI088_rtos_test_t5_acc_self_test(TEST_case_t *tc);

/* T6 – BIST GYR, scrutation en osDelay */
void BMI088_rtos_test_t6_gyr_bist(TEST_case_t *tc);

/* T7 – Temperature interne (BMI088_ReadTemp_RTOS) dans [-40, +85] °C */
void BMI088_rtos_test_t7_acc_temperature(TEST_case_t *tc);

/* ======================= B. Init et parametres ========================== */

/* T8 – Re-init par la tache BMI088_Init : idempotente (meme semaphore,
 *      config et facteurs de conversion identiques). Init rejetee (imu NULL,
 *      CS NULL, config hors plage) -> INVALID_ARG, handle intact. */
void BMI088_rtos_test_t8_reinit(TEST_case_t *tc);

/* T9 – Capteur non initialise (sem_id NULL) : toutes les operations
 *      verrouillees et les taches -> SEM_ERR, aucun semaphore cree. */
void BMI088_rtos_test_t9_uninit(TEST_case_t *tc);

/* T10 – Parametres invalides des primitives (NULL, len 0, len > BURST_MAX,
 *       config hors plage) -> INVALID_ARG, sans prendre le semaphore. */
void BMI088_rtos_test_t10_invalid_args(TEST_case_t *tc);

/* ======================= C. Semaphore du capteur ======================== */

/* T11 – Le thread detient le semaphore : versions verrouillees ->
 *       LOCK_TIMEOUT (0 ms immediat, 20 ms dans le delai), versions _NoLock
 *       fonctionnelles sans s'auto-bloquer. */
void BMI088_rtos_test_t11_nolock_held(TEST_case_t *tc);

/* T12 – Un job detient le semaphore 200 ms : ReadAcc(50 ms) et la tache
 *       ApplyConfig (20 ms) -> LOCK_TIMEOUT ; ReadAcc(1000 ms) reussit des
 *       que le semaphore est rendu. */
void BMI088_rtos_test_t12_lock_timeout(TEST_case_t *tc);

/* ======================= D. Tache BMI088_Acquire ======================== */

/* T13 – Acquisition nominale : 10 ms x 500 ms, ACC + GYR + TEMP (1/5).
 *       Nombre de cycles, horodatage sur la grille, aucune erreur ni
 *       retard, valeurs plausibles (carte immobile), topics coherents. */
void BMI088_rtos_test_t13_acquire_nominal(TEST_case_t *tc);

/* T14 – Arguments invalides (imu NULL, periode 0, aucun topic) ->
 *       INVALID_ARG immediat, rien de publie. */
void BMI088_rtos_test_t14_acquire_invalid(TEST_case_t *tc);

/* T15 – Blocage de 30 ms (semaphore tenu) pendant une acquisition a 1 ms :
 *       retards comptes, pas de rafale de rattrapage, aucune erreur. */
void BMI088_rtos_test_t15_acquire_stall(TEST_case_t *tc);

/* T16 – Semaphore tenu plus longtemps que le timeout de lecture : erreurs
 *       comptees (LOCK_TIMEOUT), la boucle continue puis reprend. */
void BMI088_rtos_test_t16_acquire_lock_errors(TEST_case_t *tc);

/* T17 – GYR seul (ACC et TEMP a NULL) : seul le topic GYR recoit ;
 *       latence d'arret <= une periode + marge. */
void BMI088_rtos_test_t17_acquire_subset_stop(TEST_case_t *tc);

/* ======================= E. Concurrence ================================= */

/* T18 – 3 lecteurs simultanes a 3 priorites (ID, ACC, GYR, TEMP en boucle) :
 *       IDs toujours exacts, valeurs plausibles, aucune erreur. */
void BMI088_rtos_test_t18_concurrent_readers(TEST_case_t *tc);

/* T19 – Re-init (tache BMI088_Init, priorite haute) pendant une acquisition
 *       a 4 ms : meme semaphore, aucune erreur de lecture. Apres la re-init,
 *       l'ACC rend des zeros le temps de redemarrer : seuls zeros et 1 g sont
 *       admis, le temps de stabilisation est mesure (max 100 ms). */
void BMI088_rtos_test_t19_reinit_during_acquire(TEST_case_t *tc);

/* T20 – Changement de plage ACC (24 g -> 3 g -> 24 g) pendant une
 *       acquisition : chaque echantillon vaut 1 g, ou est une donnee de
 *       l'ancienne plage le temps que le capteur la remplace (jamais autre
 *       chose : facteur et donnee brute restent apparies). Le temps de
 *       stabilisation est mesure et rapporte (max 100 ms). */
void BMI088_rtos_test_t20_range_change_during_acquire(TEST_case_t *tc);

/* T21 – Soft reset GYR (30 ms sous semaphore) pendant que le thread lit
 *       l'ID GYR : chaque lecture attend la fin du reset et voit 0x0F. */
void BMI088_rtos_test_t21_reset_atomic(TEST_case_t *tc);

/* ======================= F. Ressources ================================== */

/* T22 – Marge de pile des pools de la suite : task_stack_headroom de chaque
 *       instance creee >= 128 B. Le minimum est rapporte. */
void BMI088_rtos_test_t22_stack_headroom(TEST_case_t *tc);

#endif /* APEX_CFG_SCHED_RTOS == 1 */
#endif /* BMI088_RTOS_TEST_H */
