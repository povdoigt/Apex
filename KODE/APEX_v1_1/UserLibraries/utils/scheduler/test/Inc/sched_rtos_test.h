#ifndef SCHED_RTOS_TEST_H
#define SCHED_RTOS_TEST_H

#include "scheduler.h"
#include "test.h"

#define SCHED_rtos_test_N_TESTS 27

extern TEST_case_table_t SCHED_rtos_test_cases[SCHED_rtos_test_N_TESTS];

/* ========================================================================
 * Contexte d'execution
 *   - Appeler SCHED_rtos_test_init() une fois, puis TEST_perform_cases(),
 *     depuis un thread (kernel demarre) a la priorite osPriorityNormal.
 *   - Les cas utilisent les bits de join 1..30 du thread appelant.
 *   - La suite declare ses propres pools (TASK_POOL dans le .c) : elle ne
 *     touche a aucun pool applicatif.
 *   - Chaque cas, meme en echec, libere ses jobs et attend que les pools se
 *     vident avant de rendre la main : les cas sont independants.
 *   - A executer une seule fois par boot : T9 laisse une tache persistante,
 *     et T2/T23 mesurent la creation paresseuse de pools jamais utilises.
 *   - T26 utilise le vecteur SPI5_IRQHandler (SPI5 non utilise sur APEX).
 * ======================================================================== */
void SCHED_rtos_test_init(void);

/* ========================================================================
 * Fonctionnement nominal
 * ======================================================================== */

/* T0 – Spawn, join, lecture de ret
 *   Echo(42) avec join_bit 1 : join -> osOK, ret == 42, task_running false.
 *   Un second join sur le meme handle expire (le flag est consomme). */
void SCHED_rtos_test_t0_spawn_join(TEST_case_t *tc);

/* T1 – Arguments copies au spawn
 *   Modifier la struct d'arguments apres le spawn n'a aucun effet sur le
 *   job : il travaille sur sa copie privee. */
void SCHED_rtos_test_t1_args_copied(TEST_case_t *tc);

/* T2 – Creation paresseuse
 *   Le thread d'un slot n'existe pas avant le premier spawn, existe apres,
 *   et le meme thread est reutilise au spawn suivant. */
void SCHED_rtos_test_t2_lazy_creation(TEST_case_t *tc);

/* T3 – Job detache avec ret
 *   Sans join_bit, le resultat est quand meme publie dans ret, et
 *   task_running passe a false a la fin du job. */
void SCHED_rtos_test_t3_detached(TEST_case_t *tc);

/* T4 – Priorite de travail et de garage
 *   Le corps tourne a la priorite demandee, le slot retombe a
 *   TASK_PARKED_PRIORITY ensuite, et un job suivant n'herite pas de la
 *   priorite du precedent. */
void SCHED_rtos_test_t4_priorities(TEST_case_t *tc);

/* T5 – task_join_all
 *   Trois jobs de durees differentes sur trois bits : join_all -> osOK et
 *   les trois ret sont ecrits. */
void SCHED_rtos_test_t5_join_all(TEST_case_t *tc);

/* T6 – Sous-jobs
 *   Un job lance et joint lui-meme deux sous-jobs (ses propres bits 1 et 2,
 *   sans collision avec le bit 0 de son garage). */
void SCHED_rtos_test_t6_nested(TEST_case_t *tc);

/* T7 – task_prewarm
 *   Toutes les instances creees, au garage, libres ; un second appel ne
 *   recree rien ; le spawn suivant utilise le thread prechauffe. */
void SCHED_rtos_test_t7_prewarm(TEST_case_t *tc);

/* T8 – task_busy_count et task_stack_headroom
 *   Valeurs coherentes pendant et apres des jobs ; indices hors bornes et
 *   descripteur NULL -> 0. */
void SCHED_rtos_test_t8_busy_headroom(TEST_case_t *tc);

/* T9 – Tache persistante
 *   Le slot reste occupe ; un second spawn sur le pool de 1 est refuse. */
void SCHED_rtos_test_t9_persistent(TEST_case_t *tc);

/* ========================================================================
 * Parametres refuses
 * ======================================================================== */

/* T10 – Spawns refuses
 *   desc/attr NULL, args NULL, priorites None/Error/ISR, join_bit 31 et 255,
 *   join_bit sur tache persistante -> TASK_H_INVALID, sans slot consomme ni
 *   bit de join fuite. */
void SCHED_rtos_test_t10_spawn_rejects(TEST_case_t *tc);

/* T11 – Joins refuses
 *   Handle invalide, handle detache, join depuis un autre thread que le
 *   spawner, join_all avec NULL / 0 / bit duplique / handle detache
 *   -> osErrorParameter. */
void SCHED_rtos_test_t11_join_rejects(TEST_case_t *tc);

/* ========================================================================
 * Capacite et cycle de vie
 * ======================================================================== */

/* T12 – Pool epuise
 *   Le spawn N+1 est refuse sans bloquer ; liberer un job rend un spawn
 *   possible ; tous les ret restent corrects. */
void SCHED_rtos_test_t12_pool_exhaustion(TEST_case_t *tc);

/* T13 – Handle perime
 *   Apres reutilisation de son slot, l'ancien handle n'est plus vu comme
 *   en cours. */
void SCHED_rtos_test_t13_stale_handle(TEST_case_t *tc);

/* T14 – Endurance
 *   20000 cycles spawn/join : chaque ret correct, aucun slot qui fuit. */
void SCHED_rtos_test_t14_endurance(TEST_case_t *tc);

/* ========================================================================
 * Non-regression des correctifs
 * ======================================================================== */

/* T15 – Bit de join 30 (E1)
 *   Le plus haut bit autorise fonctionne de bout en bout. */
void SCHED_rtos_test_t15_join_bit_30(TEST_case_t *tc);

/* T16 – Timeout 0 et timeout court (E2)
 *   task_join(h, 0) sur un job en cours -> osErrorTimeout (et non osError) ;
 *   task_join(h, 20) attend ~20 ms puis osErrorTimeout. */
void SCHED_rtos_test_t16_join_timeout(TEST_case_t *tc);

/* T17 – Deux jobs finis avant le premier join (E8)
 *   Le second join retourne immediatement osOK au lieu de se bloquer. */
void SCHED_rtos_test_t17_both_done(TEST_case_t *tc);

/* T18 – join_all relance apres timeout (E4)
 *   Le bit du job deja fini n'est pas perdu par le premier essai. */
void SCHED_rtos_test_t18_join_all_retry(TEST_case_t *tc);

/* T19 – Join apres reutilisation du slot par un autre thread (E5)
 *   Le job A est fini, son slot est repris par un job d'un autre thread :
 *   task_join(A) -> osOK. */
void SCHED_rtos_test_t19_join_after_reuse(TEST_case_t *tc);

/* T20 – Bit encore utilise (E6)
 *   Apres un join expire, respawn sur le meme bit refuse tant que l'ancien
 *   job tourne, accepte ensuite, et le join suivant attend bien le
 *   nouveau job. */
void SCHED_rtos_test_t20_bit_in_flight(TEST_case_t *tc);

/* ========================================================================
 * Priorites et temps
 * ======================================================================== */

/* T21 – Bornes de priorite (F3)
 *   osPriorityRealtime7 deux fois sur le meme slot (le 2e figeait la carte
 *   a 56), et osPriorityIdle. */
void SCHED_rtos_test_t21_prio_bounds(TEST_case_t *tc);

/* T22 – Reveil du joiner sans inversion (F1)
 *   Joiner High, job Realtime, thread AboveNormal qui monopolise le CPU :
 *   le joiner est reveille en moins de 1 ms apres la fin du corps. */
void SCHED_rtos_test_t22_wake_no_inversion(TEST_case_t *tc);

/* T23 – Latence spawn -> premiere instruction (F2)
 *   Mesure DWT a froid (creation du thread) et a chaud (min/max sur 100
 *   spawns). Echec si le chaud depasse 100 us. */
void SCHED_rtos_test_t23_spawn_latency(TEST_case_t *tc);

/* T24 – Le joiner dort vraiment
 *   Pendant un join de 100 ms, un thread de priorite plus basse que le
 *   joiner progresse : le joiner ne fait pas d'attente active. */
void SCHED_rtos_test_t24_joiner_sleeps(TEST_case_t *tc);

/* ========================================================================
 * Concurrence et contexte
 * ======================================================================== */

/* T25 – Spawners concurrents (G1)
 *   Deux threads de priorites differentes enchainent spawn/join sur le meme
 *   pool : chaque job rend la valeur de son propre spawner. */
void SCHED_rtos_test_t25_concurrent_spawners(TEST_case_t *tc);

/* T26 – Appels depuis une ISR (H1)
 *   Spawn -> TASK_H_INVALID, join et prewarm -> osErrorISR, aucun slot
 *   consomme. */
void SCHED_rtos_test_t26_isr_rejects(TEST_case_t *tc);

#endif /* SCHED_RTOS_TEST_H */
