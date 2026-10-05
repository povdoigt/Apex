#ifndef CB_SEQ_TEST_H
#define CB_SEQ_TEST_H

#include "circular_buffer.h"
#include "test.h"

#define CB_seq_test_N_TESTS 22

extern TEST_case_table_t CB_seq_test_cases[CB_seq_test_N_TESTS];

/* ========================================================================
 * T0 – Protection NULL / arguments invalides
 *   Appelle cb_init, cb_push et cb_pop avec des pointeurs NULL.
 *   Verifie qu'aucun crash ne se produit et que les codes d'erreur
 *   corrects sont retournes (CB_BAD_ARG).
 * ======================================================================== */
void CB_seq_test_t0_null_args(TEST_case_t *tc);

/* ========================================================================
 * T1 – Ordre FIFO basique (push / pop 3 elements)
 *   Pousse {10, 20, 30} et depile dans l'ordre. Verifie :
 *   - cb_push retourne CB_OK pour chaque insertion.
 *   - cb_pop retourne les elements dans l'ordre FIFO exact.
 *   - count == 0 apres epuisement.
 * ======================================================================== */
void CB_seq_test_t1_fifo_order(TEST_case_t *tc);

/* ========================================================================
 * T2 – Lecture sur buffer vide → CB_EMPTY
 *   cb_pop sur un buffer fraichement initialise doit retourner CB_EMPTY
 *   sans modifier la memoire de sortie.
 * ======================================================================== */
void CB_seq_test_t2_empty_read(TEST_case_t *tc);

/* ========================================================================
 * T3 – Buffer plein avec politique CB_REJECT_NEW → CB_FULL
 *   capacity=3, pousse 4 elements. Le 4eme doit retourner CB_FULL.
 *   Les 3 premiers elements doivent rester intacts en memoire.
 * ======================================================================== */
void CB_seq_test_t3_full_reject(TEST_case_t *tc);

/* ========================================================================
 * T4 – Buffer plein avec politique CB_OVERWRITE_OLDEST
 *   capacity=3, pousse {1, 2, 3, 4}. Le 4eme push doit retourner
 *   CB_OVERWROTE_OLDEST. Apres overwrite, le premier pop renvoie 2
 *   (l'element 1 a ete ecrase), puis 3, puis 4.
 * ======================================================================== */
void CB_seq_test_t4_full_overwrite(TEST_case_t *tc);

/* ========================================================================
 * T5 – cb_reset vide logiquement le buffer
 *   Pousse 2 elements, cb_reset, verifie count == 0 et que cb_pop
 *   retourne CB_EMPTY. Puis repousse 1 element pour confirmer que le
 *   buffer est reutilisable apres reset.
 * ======================================================================== */
void CB_seq_test_t5_reset(TEST_case_t *tc);

/* ========================================================================
 * T6 – cb_peek : couverture complete du wrap permissif sur l'index absolu
 *   Logique : cb_peek(cb, idx) lit le slot physique  idx % capacity.
 *   capacity=4, storage[0..3]={10,20,30,40}.
 *   Scenarios verifies :
 *     A) idx in [0,3]         -> acces directs.
 *     B) idx == cap           -> wrap  -> slot 0 = 10.
 *     C) idx == cap+2         -> wrap  -> slot 2 = 30.
 *     D) idx == 2*cap         -> double wrap -> slot 0 = 10.
 *     E) idx == 17            -> 17%4=1 -> slot 1 = 20.
 *     F) non destructif       -> count inchange apres tous les peeks.
 * ======================================================================== */
void CB_seq_test_t6_peek_absolute(TEST_case_t *tc);

/* ========================================================================
 * T7 – cb_peek_relative : couverture complete du wrap permissif
 *   Logique : index_physique = (origin + offset) % capacity  (signe gere).
 *   capacity=5, storage[0..4]={10,20,30,40,50}.
 *   Scenarios verifies :
 *     A) offsets +0..+4 depuis origin=0    -> acces directs.
 *     B) offset == +cap    (0+5)%5=0       -> slot 0 = 10.
 *     C) offset == +cap+2  (0+7)%5=2       -> slot 2 = 30.
 *     D) offset == -cap    (0-5+5)%5=0     -> slot 0 = 10.
 *     E) offset == -1      (0-1+5)%5=4     -> slot 4 = 50.
 *     F) offset == -2      (0-2+5)%5=3     -> slot 3 = 40.
 *     G) origin=2, +3      (2+3)%5=0       -> slot 0 = 10.
 *     H) origin=2, -3      (2-3+5)%5=4     -> slot 4 = 50.
 *     I) origin=1, +11     (1+11)%5=2      -> slot 2 = 30.
 *     J) origin=1, -11     (1-11+15)%5=0   -> slot 0 = 10.
 *     K) non destructif    -> count inchange apres tous les peeks.
 * ======================================================================== */
void CB_seq_test_t7_peek_relative(TEST_case_t *tc);

/* ========================================================================
 * T8 – Wrap-around (head et tail traversent la frontiere de capacite)
 *   capacity=4, pousse {1,2,3,4}, pop {1,2}, pousse {5,6}.
 *   Le head et le tail doivent avoir effectue un wrap.
 *   Verifie que le pop donne {3, 4, 5, 6} dans l'ordre.
 * ======================================================================== */
void CB_seq_test_t8_wraparound(TEST_case_t *tc);

/* ========================================================================
 * T9 – Elem_size variable : type float
 *   Cree un buffer de capacity=4 avec elem_size=sizeof(float).
 *   Pousse {1.5f, 2.5f, 3.5f}, pop et verifie les valeurs en virgule
 *   flottante (tolerance 1e-6).
 * ======================================================================== */
void CB_seq_test_t9_float_elemsize(TEST_case_t *tc);

/* ========================================================================
 * T10 – Cycle remplissage / vidange repete
 *   capacity=8. Cycle 1 : pousse [0..7], pop et verifie [0..7].
 *   Cycle 2 : pousse [8..15], pop et verifie [8..15].
 *   Verifie que le buffer est parfaitement reutilisable apres un cycle
 *   complet (head, tail, count tous remis a zero logiquement).
 * ======================================================================== */
void CB_seq_test_t10_fill_drain_cycle(TEST_case_t *tc);

/* ========================================================================
 * Cas limites et stress (revue du 05/10/2026)
 * ======================================================================== */

/* ========================================================================
 * T11 – Capacite 1, deux politiques
 *   head = tail = 0 en permanence. REJECT_NEW : 5 cycles push / push refuse
 *   (CB_FULL, donnee intacte) / pop / pop vide. OVERWRITE_OLDEST : chaque
 *   push suivant ecrase (CB_OVERWROTE_OLDEST), la derniere valeur reste.
 * ======================================================================== */
void CB_seq_test_t11_capacity_one(TEST_case_t *tc);

/* ========================================================================
 * T12 – Ecrasements successifs (OVERWRITE_OLDEST)
 *   capacity=3, pousse 1..10 : 3 CB_OK puis 7 CB_OVERWROTE_OLDEST, le tail
 *   fait plusieurs tours par le chemin d'ecrasement ; pop -> 8, 9, 10.
 *   Puis 1000 pushes : restent les 3 derniers, dans l'ordre.
 * ======================================================================== */
void CB_seq_test_t12_multi_overwrite(TEST_case_t *tc);

/* ========================================================================
 * T13 – Sortie de l'etat plein (REJECT_NEW)
 *   capacity=4 plein ; 50 fois : pop (ordre FIFO), push accepte, push
 *   suivant refuse. Vidange finale dans l'ordre.
 * ======================================================================== */
void CB_seq_test_t13_reject_recovery(TEST_case_t *tc);

/* ========================================================================
 * T14 – Tailles d'element 1, 3, 13 et 128 octets, avec canaris
 *   Stockage et sorties encadres d'octets temoins. 23 pushes (plusieurs
 *   tours) avec pops intercales, peek de chaque slot : contenu exact octet
 *   par octet, aucun temoin touche (aucune ecriture hors limites).
 * ======================================================================== */
void CB_seq_test_t14_elem_sizes_guards(TEST_case_t *tc);

/* ========================================================================
 * T15 – Acces pointeur cb_peek_ptr / cb_peek_relative_ptr
 *   Adresses dans le stockage au slot attendu (avec wrap), identiques au
 *   contenu copie par cb_peek / cb_peek_relative sur un balayage
 *   origine x offset. NULL pour un buffer NULL ou non initialise.
 * ======================================================================== */
void CB_seq_test_t15_ptr_accessors(TEST_case_t *tc);

/* ========================================================================
 * T16 – cb_wrap aux extremes
 *   capacites 1..1000, origines 0 / SIZE_MAX / SIZE_MAX-1..., offsets
 *   INT_MIN / INT_MIN+1 / INT_MAX / +-capacity... : slot compare a une
 *   reference calculee en 64 bits signes.
 * ======================================================================== */
void CB_seq_test_t16_wrap_extremes(TEST_case_t *tc);

/* ========================================================================
 * T17 – Arguments invalides restants
 *   pop/peek/peek_relative avec sortie NULL, buffer NULL, buffer a zero
 *   (jamais initialise), reset/free/count sur NULL, politique inconnue,
 *   elem_size * capacity au-dela de SIZE_MAX.
 * ======================================================================== */
void CB_seq_test_t17_bad_args(TEST_case_t *tc);

/* ========================================================================
 * T18 – Push d'une donnee deja en place dans le slot head
 *   Element ecrit via cb_peek_ptr(head) puis pousse depuis cette adresse :
 *   CB_OK et valeur relue, sur plusieurs tours.
 * ======================================================================== */
void CB_seq_test_t18_push_in_place(TEST_case_t *tc);

/* ========================================================================
 * T19 – Test aleatoire contre un modele de reference
 *   36 configurations (capacite 1/2/3/7/8/16 x politique x taille 1/4/12),
 *   10000 operations tirees au hasard chacune (graine fixe) : push, pop,
 *   peek, peek_relative, reset. Modele independant (rang de push modulo la
 *   capacite) ; statut, contenu, count et temoins verifies a chaque pas.
 * ======================================================================== */
void CB_seq_test_t19_random_model(TEST_case_t *tc);

/* ========================================================================
 * T20 – cb_push depuis une interruption (TIM5, 20 kHz)
 *   Meme scenario que le T23 RTOS, mais a l'attente active : l'ISR pousse
 *   0, 1, 2... (REJECT_NEW, refus comptes), la boucle principale depile
 *   300 ms. Valeurs croissantes, trous == refus, depiles + refus ==
 *   pousses, count coherent avec head / tail, au moins un refus.
 * ======================================================================== */
void CB_seq_test_t20_isr_push(TEST_case_t *tc);

/* ========================================================================
 * T21 – Copies jamais dechirees face a une ISR (OVERWRITE_OLDEST)
 *   L'ISR pousse a 20 kHz des elements de 64 o (16 mots = numero) dans un
 *   buffer de 4 ; la boucle principale fait cb_peek (slot au hasard) et
 *   cb_pop pendant 300 ms : chaque copie est homogene, les pops sont
 *   strictement croissants, et l'ISR a bien ecrase des elements.
 * ======================================================================== */
void CB_seq_test_t21_isr_no_torn_copy(TEST_case_t *tc);

#endif /* CB_SEQ_TEST_H */
