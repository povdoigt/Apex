#ifndef DT_RTOS_TEST_H
#define DT_RTOS_TEST_H

#include "main_config.h"
#include "data_topic.h"
#include "test.h"

#if (APEX_CFG_SCHED_RTOS == 1)

#include "scheduler.h"

#define DT_rtos_test_N_TESTS 18

extern TEST_case_table_t DT_rtos_test_cases[DT_rtos_test_N_TESTS];

/* ========================================================================
 * Contexte d'execution
 *   - Appeler DT_rtos_test_init() une fois, puis TEST_perform_cases(),
 *     depuis un thread (kernel demarre) a la priorite osPriorityNormal.
 *   - Les cas utilisent les bits de join 1..4 du thread appelant.
 *   - La suite declare ses propres pools (TASK_POOL dans le .c). Le .c n'est
 *     compile qu'avec APEX_CFG_SCHED_RTOS et APEX_CFG_PROFILE_TEST a 1.
 *   - Chaque cas, meme en echec, arrete le timer, attend la fin de ses
 *     taches et libere ses topics avant de rendre la main.
 *   - Les cas a interruption utilisent TIM5 (non utilise sur APEX) via
 *     test_irq.h, priorite NVIC 6.
 *   - T27 demande que le projet appelle DT_rtos_test_pre_kernel() depuis
 *     setup_pre_kernel(), avant osKernelStart().
 *   - Les numeros renvoient aux points de la revue du 04/10/2026
 *     ("Revue data_topic & circular_buffer").
 * ======================================================================== */
void DT_rtos_test_init(void);

/* ========================================================================
 * A. Suite sequentielle rejouee sous RTOS (T0-T24)
 *   Le projet execute aussi la table DT_seq_test_cases sous RTOS : memes
 *   fonctions, memes numeros, memes attendus. Sous RTOS, attach/detach
 *   creent et suppriment en plus le semaphore de l'abonne, et les sections
 *   critiques passent par BASEPRI. Si un cas echoue sous RTOS et passe en
 *   sequentiel, le defaut est dans la couche RTOS.
 * ======================================================================== */

/* ========================================================================
 * B. Publication et notification
 * ======================================================================== */

/* T14 – Abonne plus prioritaire que le publieur (point 1)
 *   Abonne High : wait puis read, 50 fois. Le thread de test (Normal)
 *   publie 1..50. Des que publish rend la main, l'abonne a deja lu la
 *   valeur qui vient d'etre publiee : aucune periode de retard, aucun
 *   DT_EMPTY, valeurs dans l'ordre. */
void DT_rtos_test_t14_priority_subscriber(TEST_case_t *tc);

/* T15 – wait sans donnee (point 2)
 *   Publie 42 puis le lit sans wait : le jeton reste dans le semaphore.
 *   wait(50) doit quand meme expirer apres 50 ms, et read rendre DT_EMPTY.
 *   wait(0) : osErrorTimeout sans donnee, osOK avec. Abonne detache ou
 *   NULL : osErrorParameter. */
void DT_rtos_test_t15_wait_no_data(TEST_case_t *tc);

/* T16 – wait reveille par une publication
 *   Une tache BelowNormal publie 7 apres 30 ms : wait(500) rend osOK au
 *   bout de 28..33 ms et read rend 7. Sans publieur, wait(20) expire au
 *   bout de 20..22 ms. */
void DT_rtos_test_t16_wait_wakeup(TEST_case_t *tc);

/* T17 – Publieurs concurrents (point 8)
 *   Deux taches BelowNormal (time slicing) publient 20000 fois chacune sur
 *   le meme topic, un abonne attache, et TIM5 publie en plus a 20 kHz. Les
 *   executions se chevauchent, et pub_seq vaut exactement 40000 + le nombre
 *   de publications de l'ISR. Revue du 05/10 : sans l'ISR, ce test passait
 *   avec des sections critiques vides (preemption au tick seulement). */
void DT_rtos_test_t17_two_publishers(TEST_case_t *tc);

/* T18 – Abonnement pendant une publication (point 9)
 *   Une tache High reveillee par la publication de 1 attache aussitot un
 *   second abonne FROM_NOW : il n'a rien a lire (DT_EMPTY, pas un slot
 *   jamais ecrit). La publication suivante (2) lui parvient normalement. */
void DT_rtos_test_t18_attach_during_publish(TEST_case_t *tc);

/* T19 – Liste des abonnes modifiee pendant la notification (point 10)
 *   topic A : abonnes B puis A (A en tete) ; topic B : abonne C. La tache
 *   de A (High), reveillee par une publication sur A, se detache et
 *   s'attache au topic B. B a bien recu son jeton, C n'en a recu aucun, et
 *   les deux listes sont coherentes. */
void DT_rtos_test_t19_list_during_notify(TEST_case_t *tc);

/* ========================================================================
 * C. Lecture
 * ======================================================================== */

/* T20 – Pas de lecture dechiree, publieur en interruption (points 5, 6)
 *   TIM5 publie a 20 kHz des trames de 128 o (seq, 30 mots = seq,
 *   seq_end) dans un topic de 4. Le thread de test lit pendant 300 ms "au
 *   bord" (topic plein vu de l'abonne, pause aleatoire) : chaque trame lue
 *   est celle que la publication suivante ecrase. Aucune trame melangee,
 *   OK => seq consecutive, DT_DATA_LOSS => trou, et au moins une perte. */
void DT_rtos_test_t20_torn_read_isr(TEST_case_t *tc);

/* T21 – Pertes signalees sous concurrence (point 3)
 *   Publieur AboveNormal : 1..300 a 1 kHz, topic de 8. Trois abonnes
 *   (High sans pause, BelowNormal 2 ms par lecture, Low 5 ms). Pour chacun :
 *   lus + sautes == 300, chaque trou est annonce par DT_DATA_LOSS, derniere
 *   valeur 300. Le High ne perd rien, les deux lents perdent. */
void DT_rtos_test_t21_loss_accounting(TEST_case_t *tc);

/* ========================================================================
 * D. circular_buffer sous concurrence
 * ======================================================================== */

/* T22 – cb_pop concurrents (point 11)
 *   Deux consommateurs BelowNormal (time slicing) et TIM5 (20 kHz) vident
 *   un buffer de 64 que le thread de test remplit avec 1..20000
 *   (REJECT_NEW, reessai si plein). Chaque valeur est retiree exactement une
 *   fois (nombre et somme), les trois consommateurs ont travaille, count
 *   finit a 0. Revue du 05/10 : sans l'ISR, ce test passait avec des
 *   sections critiques vides. */
void DT_rtos_test_t22_cb_pop_concurrent(TEST_case_t *tc);

/* T23 – cb_push depuis une interruption (point 6)
 *   TIM5 pousse 0, 1, 2... a 20 kHz (REJECT_NEW, refus comptes). Le thread
 *   de test depile pendant 300 ms en verifiant count == head - tail sous
 *   section critique. Valeurs strictement croissantes, trous == refus,
 *   depiles + refus == pousses, au moins un refus. */
void DT_rtos_test_t23_cb_push_isr(TEST_case_t *tc);

/* ========================================================================
 * E. Interruptions et cycle de vie
 * ======================================================================== */

/* T24 – publish depuis une interruption reveille un abonne (point 6)
 *   TIM5 publie 1, 2, 3... a 1 kHz. Abonne High : wait(10) puis read, pendant
 *   200 ms. Aucun timeout, aucune perte, tout est recu dans l'ordre. Depuis
 *   l'ISR, wait rend osErrorISR et attach DT_BAD_ARG. */
void DT_rtos_test_t24_publish_isr_wait(TEST_case_t *tc);

/* T25 – free et re-init sous RTOS (point 14)
 *   free supprime les semaphores (sem == NULL) et detache les abonnes ;
 *   wait sur un abonne detache -> osErrorParameter. Apres re-init, une
 *   publication differee reveille wait normalement. */
void DT_rtos_test_t25_free_reinit(TEST_case_t *tc);

/* T26 – Attache / detache en rafale pendant les publications (point 10)
 *   Un publieur AboveNormal alimente deux topics. Une tache High fait 300
 *   fois : attache au topic A, wait, read, detache, idem sur B. Tout
 *   reussit, et a la fin chaque liste ne contient que son abonne permanent
 *   (chainage et sub_count intacts). */
void DT_rtos_test_t26_attach_churn(TEST_case_t *tc);

/* ========================================================================
 * F. Avant le demarrage du noyau
 * ======================================================================== */

/* A appeler depuis setup_pre_kernel() (voir freertos.c), avant
 * osKernelStart() : initialise un topic, y attache un abonne et publie 3
 * valeurs, noyau arrete. T27 verifie ensuite ce qui a ete releve. */
void DT_rtos_test_pre_kernel(void);

/* T27 – Publication avant le demarrage du noyau (revue du 05/10, B1)
 *   Avant le correctif, la notification d'une publication faite noyau
 *   arrete bloquait la carte (configASSERT de vPortValidateInterruptPriority).
 *   Les 3 publications pre-noyau rendent DT_OK, wait y rend osError, puis,
 *   noyau demarre : wait(0) les voit, read rend 101..103, et une publication
 *   differee reveille wait normalement. Une seule verification par demarrage
 *   (SKIP ensuite). */
void DT_rtos_test_t27_publish_before_kernel(TEST_case_t *tc);

/* ========================================================================
 * G. Attente infinie et liste face a une interruption
 * ======================================================================== */

/* T28 – wait(osWaitForever)
 *   Reveille au bout de ~20 ms par une tache (publication differee), puis
 *   par TIM5 (une publication toutes les 20 ms) : osOK et la bonne valeur. */
void DT_rtos_test_t28_wait_forever(TEST_case_t *tc);

/* T29 – Attache / detache en rafale face a un publieur en interruption
 *   Comme T26, mais TIM5 publie a 20 kHz alternativement sur les deux
 *   topics : l'ISR parcourt les listes pendant que la tache High les
 *   modifie. 3000 attach/wait/read/detach reussis, listes intactes, les
 *   abonnes permanents ont tout compte (num_to_read == publications). */
void DT_rtos_test_t29_churn_vs_isr(TEST_case_t *tc);

/* T30 – Deux taches modifient la meme liste
 *   Low (BelowNormal) attache / detache en boucle sur topic A. High,
 *   reveillee par TIM5 (publications sur B) a n'importe quelle instruction
 *   de Low, attache / detache 2000 fois sur A ; l'ISR parcourt aussi la
 *   liste de A. Aucun echec, et la liste de A ne contient plus que son
 *   abonne permanent. Seul cas ou deux modifications de liste peuvent
 *   s'entrelacer : sans section critique, la liste se corrompt. */
void DT_rtos_test_t30_two_tasks_one_list(TEST_case_t *tc);

/* T31 – Abonne fantome (tache terminee sans detach, memoire reutilisee)
 *   Un abonne encore chaine au milieu de la liste voit sa memoire ecrasee
 *   (0xA5, pointeurs compris). Publication en tache puis en ISR : aucun
 *   plantage, le parcours s'arrete sur le fantome (la tete est notifiee,
 *   pas l'abonne situe au-dela, dont les donnees restent lisibles), chaque
 *   arret est compte dans list_faults. Une nouvelle attache passe, et le
 *   detach du voisin du fantome n'ecrit rien dedans. */
void DT_rtos_test_t31_dangling_subscriber(TEST_case_t *tc);

#endif /* APEX_CFG_SCHED_RTOS */

#endif /* DT_RTOS_TEST_H */
