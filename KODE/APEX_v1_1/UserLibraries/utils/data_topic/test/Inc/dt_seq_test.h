#ifndef DT_SEQ_TEST_H
#define DT_SEQ_TEST_H

#include "data_topic.h"
#include "test.h"

#define DT_seq_test_N_TESTS 30

extern TEST_case_table_t DT_seq_test_cases[DT_seq_test_N_TESTS];

/* ========================================================================
 * T0 – Protection NULL / arguments invalides
 *   Appelle les fonctions API avec des pointeurs NULL ou un abonné
 *   non attaché. Verifie que DT_BAD_ARG est retourné sans crash.
 * ======================================================================== */
void DT_seq_test_t0_null_args(TEST_case_t *tc);

/* ========================================================================
 * T1 – Publish / read FIFO basique (1 abonné)
 *   Publie {10, 20, 30}, lit avec data_sub_read dans l'ordre FIFO.
 *   Verifie les valeurs, les codes de retour DT_OK, et que num_to_read
 *   décroît correctement.
 * ======================================================================== */
void DT_seq_test_t1_publish_read_fifo(TEST_case_t *tc);

/* ========================================================================
 * T2 – DT_EMPTY quand aucune donnée disponible
 *   Attache un abonné FROM_NOW avant toute publication.
 *   data_sub_read et data_sub_peek retournent DT_EMPTY, sortie intacte.
 *   Verifie que num_to_read == 0.
 * ======================================================================== */
void DT_seq_test_t2_empty_read(TEST_case_t *tc);

/* ========================================================================
 * T3 – DATA_ATTACH_FROM_OLDEST récupère l'historique existant
 *   Publie {10, 20, 30} avant d'attacher l'abonné en mode FROM_OLDEST.
 *   Verifie que num_to_read == 3 et que les lectures donnent
 *   {10, 20, 30} dans l'ordre, suivi de DT_EMPTY.
 * ======================================================================== */
void DT_seq_test_t3_attach_from_oldest(TEST_case_t *tc);

/* ========================================================================
 * T4 – DATA_ATTACH_FROM_NOW ignore l'historique existant
 *   Publie {10, 20, 30} avant d'attacher l'abonné en mode FROM_NOW.
 *   Verifie que num_to_read == 0.
 *   Publie ensuite 40 : num_to_read == 1, lecture retourne 40.
 * ======================================================================== */
void DT_seq_test_t4_attach_from_now(TEST_case_t *tc);

/* ========================================================================
 * T5 – data_sub_num_to_read suit le compteur de publications
 *   Attache FROM_NOW, publie 1, 2 puis 3 elements successivement.
 *   Verifica que num_to_read vaut 1, 2, 3 a chaque étape, et décroît
 *   de 1 apres chaque data_sub_read.
 * ======================================================================== */
void DT_seq_test_t5_num_to_read(TEST_case_t *tc);

/* ========================================================================
 * T6 – Indépendance de deux abonnés (tails séparées)
 *   Attache sub1 et sub2 FROM_NOW.
 *   Publie {10, 20, 30}.
 *   sub1 lit {10, 20}. Vérifie que num_to_read(sub2) == 3 (inchangé).
 *   sub2 lit {10, 20, 30} complètement.
 *   num_to_read(sub1) == 1, num_to_read(sub2) == 0 en fin.
 * ======================================================================== */
void DT_seq_test_t6_two_subscribers(TEST_case_t *tc);

/* ========================================================================
 * T7 – DT_DATA_LOSS quand un abonné est dépassé
 *   capacity=3, attache FROM_NOW, publie 1..5 (1 et 2 écrasés).
 *   data_sub_read retourne DT_DATA_LOSS avec 3 (plus ancienne restante),
 *   puis DT_OK avec 4 et 5, puis DT_EMPTY : la perte est signalée une
 *   seule fois. Même chose via data_sub_peek (publie 10..14 : peek rend
 *   DT_DATA_LOSS/12, le read suivant DT_OK/12).
 * ======================================================================== */
void DT_seq_test_t7_data_loss(TEST_case_t *tc);

/* ========================================================================
 * T8 – data_sub_sync(FROM_NOW) et data_sub_sync(FROM_OLDEST)
 *   Publie {10, 20, 30}, sync FROM_NOW : num_to_read == 0, tail == head,
 *   read -> DT_EMPTY, puis 40 publié est lu.
 *   sync FROM_OLDEST : num_to_read == 4 et relecture de {10,20,30,40}.
 * ======================================================================== */
void DT_seq_test_t8_sync(TEST_case_t *tc);

/* ========================================================================
 * T9 – data_sub_detach / sub_count / re-attachement
 *   Attache sub1 et sub2 → sub_count == 2.
 *   Re-attache sub1 au même topic → DT_OK sans effet ; à un 2e topic →
 *   DT_BAD_ARG, sub1 reste sur le premier.
 *   Detache sub1 → sub_count == 1, sub1.attached == 0.
 *   Publie 99 → sub2 lit 99 (DT_OK), sub1 ne peut pas lire.
 *   Re-attache sub1 FROM_NOW → sub_count == 2, num_to_read(sub1) == 0.
 * ======================================================================== */
void DT_seq_test_t9_detach_reattach(TEST_case_t *tc);

/* ========================================================================
 * T10 – data_sub_peek : rang relatif au curseur de l'abonné
 *   Publie 1, 2, attache FROM_NOW, publie {10, 20, 30}.
 *   peek(0..2) = {10, 20, 30} (pas les slots physiques 0..2), peek(3) →
 *   DT_EMPTY sans toucher la sortie. tail et num_to_read inchangés,
 *   read → 10, puis peek(0) → 20.
 * ======================================================================== */
void DT_seq_test_t10_peek_non_destructive(TEST_case_t *tc);

/* ========================================================================
 * T11 – CB_REJECT_NEW refusé, OVERWRITE_OLDEST ne bloque jamais
 *   init(REJECT_NEW) → DT_BAD_ARG, et le topic refuse publish et attach.
 *   Avec OVERWRITE_OLDEST, capacity=2 : 5 tours "remplir puis tout lire"
 *   sans aucun refus (le scénario qui bloquait REJECT_NEW pour toujours).
 * ======================================================================== */
void DT_seq_test_t11_reject_new_policy(TEST_case_t *tc);

/* ========================================================================
 * T12 – Accès zéro copie (peek_ptr / read_ptr)
 *   Publie 1..3. peek_ptr(0) → 1 (la plus ancienne non lue), peek_ptr(2)
 *   → 3, sans consommer. read_ptr → 1 (adresse dans le stockage du topic)
 *   et avance. Sur vide : DT_EMPTY, pointeur de sortie inchangé.
 * ======================================================================== */
void DT_seq_test_t12_zero_copy_ptr(TEST_case_t *tc);

/* ========================================================================
 * T13 – data_topic_free détache les abonnés
 *   Deux abonnés attachés, free : attached == 0, liste vide, read /
 *   publish / attach → DT_BAD_ARG. Après re-init, attache et lecture OK.
 * ======================================================================== */
void DT_seq_test_t13_free_detaches(TEST_case_t *tc);

/* ========================================================================
 * Cas limites et stress (revue du 05/10/2026)
 * ======================================================================== */

/* ========================================================================
 * T14 – Bornes de la perte
 *   capacity=4. lag == capacite : 1..4 lus en DT_OK, aucune perte.
 *   lag == capacite + 1 : exactement une donnee perdue, DT_DATA_LOSS avec
 *   la plus ancienne restante (6), puis 7, 8, 9 en DT_OK, puis DT_EMPTY.
 * ======================================================================== */
void DT_seq_test_t14_loss_boundary(TEST_case_t *tc);

/* ========================================================================
 * T15 – Perte constatee par data_sub_peek(idx > 0)
 *   1..7 publies dans 4 : peek(2) rend DT_DATA_LOSS et 6 (4 + 2), recale
 *   l'abonne (num_to_read == 4) ; peek(0) = 4, peek(3) = 7, peek(4) vide.
 * ======================================================================== */
void DT_seq_test_t15_loss_by_peek_idx(TEST_case_t *tc);

/* ========================================================================
 * T16 – Perte constatee par read_ptr et par peek_ptr
 *   read_ptr signale la perte et rend la plus ancienne restante, adresse
 *   dans le stockage ; peek_ptr(1) apres un nouveau depassement aussi.
 * ======================================================================== */
void DT_seq_test_t16_loss_by_ptr(TEST_case_t *tc);

/* ========================================================================
 * T17 – peek au-dela des donnees presentes pendant un retard
 *   Abonne depasse (lag 7 > 4) : peek(4) et peek(100) rendent DT_EMPTY sans
 *   toucher la sortie ni recaler l'abonne (num_to_read reste 7) ; le read
 *   suivant signale alors la perte.
 * ======================================================================== */
void DT_seq_test_t17_peek_beyond_while_lagging(TEST_case_t *tc);

/* ========================================================================
 * T18 – Detach dans n'importe quel slot du registre
 *   4 abonnes ; detach d'un abonne attache entre d'autres, du dernier
 *   attache, du premier : registre et sub_count verifies a chaque etape, les
 *   restants recoivent les publications. Puis re-attache dans un autre ordre
 *   et 2 detach.
 * ======================================================================== */
void DT_seq_test_t18_detach_middle(TEST_case_t *tc);

/* ========================================================================
 * T19 – Debordement de pub_seq (2^32 publications)
 *   pub_seq force juste sous 2^32 : 12 publications le font repasser par
 *   0 ; num_to_read, lecture, FROM_OLDEST et detection de perte restent
 *   exacts de part et d'autre.
 * ======================================================================== */
void DT_seq_test_t19_pub_seq_wrap(TEST_case_t *tc);

/* ========================================================================
 * T20 – FROM_OLDEST apres plusieurs tours, et topic de capacite 1
 *   1..10 dans 4 puis attache FROM_OLDEST : 7..10 en DT_OK (aucune perte
 *   annoncee). Capacite 1 : perte, peek, FROM_OLDEST, deux abonnes.
 * ======================================================================== */
void DT_seq_test_t20_oldest_after_wraps_cap1(TEST_case_t *tc);

/* ========================================================================
 * T21 – Elements de 13 et 128 octets, avec canaris
 *   Stockage et sortie encadres d'octets temoins ; lectures, perte et peek :
 *   contenus exacts, aucun temoin touche.
 * ======================================================================== */
void DT_seq_test_t21_elem_sizes_guards(TEST_case_t *tc);

/* ========================================================================
 * T22 – Attache : mode invalide, abonne remis a zero encore inscrit
 *   Mode 7 -> DT_BAD_ARG. Un abonne remis a zero sans detach : la
 *   publication atteint toujours les autres ; sa re-attache (FROM_NOW, puis
 *   FROM_OLDEST pour un second) reprend un slot sans doublon et compte une
 *   anomalie par abonne. Idem apres un ecrasement partiel (seul `attached`
 *   remis a zero).
 * ======================================================================== */
void DT_seq_test_t22_attach_refusals(TEST_case_t *tc);

/* ========================================================================
 * T23 – Publieur en interruption, lectures jamais dechirees
 *   TIM5 publie a 20 kHz des trames de 128 o (seq, 30 mots = seq, seq_end)
 *   dans un topic de 4 ; la boucle principale lit 300 ms "au bord" (topic
 *   plein vu de l'abonne, pause aleatoire) : chaque trame lue est celle que
 *   la publication suivante ecrase. Aucune trame melangee, OK => seq
 *   consecutive, DT_DATA_LOSS => trou, et au moins une perte.
 * ======================================================================== */
void DT_seq_test_t23_isr_torn_read(TEST_case_t *tc);

/* ========================================================================
 * T24 – Publieur en interruption, comptabilite des pertes exacte
 *   TIM5 publie 1, 2, 3... a 20 kHz dans un topic de 8 ; la boucle
 *   principale lit avec des pauses. Chaque trou est annonce par
 *   DT_DATA_LOSS, lus + sautes == publies, derniere valeur == derniere
 *   publiee.
 * ======================================================================== */
void DT_seq_test_t24_isr_loss_accounting(TEST_case_t *tc);

/* ========================================================================
 * T25 – Detach apres une re-initialisation fautive du topic
 *   Erreur d'usage : topic reinitialise avec deux abonnes encore attaches,
 *   puis un nouvel abonne s'y attache. Le detach des anciens ne touche pas
 *   le nouveau registre (ni slots, ni sub_count) et compte 2 anomalies dans
 *   list_faults ; le nouvel abonne recoit toujours les publications.
 * ======================================================================== */
void DT_seq_test_t25_stale_detach(TEST_case_t *tc);

/* ========================================================================
 * T26 – Re-initialisation fautive, detach des anciens dans l'ordre inverse
 *   Comme T25, mais le plus ancien abonne se detache le premier. sub_count
 *   reste exact (1), 2 anomalies sont comptees, et le topic accepte encore
 *   de nouvelles attaches (regression N1 du 08/10/2026 : sub_count tombait
 *   a 0 et toute attache suivante etait refusee).
 * ======================================================================== */
void DT_seq_test_t26_stale_detach_reverse(TEST_case_t *tc);

/* ========================================================================
 * T27 – Abonne remis a zero puis rattache a un autre topic
 *   a2 (ta) remis a zero sans detach puis attache a tb. Publication et
 *   attache sur ta : le slot perime est libere une seule fois (list_faults
 *   1), a2 n'est jamais ecrit au titre de ta, le registre de tb n'est pas
 *   touche ; chaque abonne ne lit que son topic. Puis a1 remis a zero passe
 *   sur tb et ta est libere : a1 n'est pas touche, a3/a4 sont detaches.
 * ======================================================================== */
void DT_seq_test_t27_sub_moved(TEST_case_t *tc);

/* ========================================================================
 * T28 – Registre plein
 *   DATA_TOPIC_MAX_SUBS attaches, la suivante rend DT_NO_SLOT sans rien
 *   modifier ; un slot libere par detach est repris ; registre plein avec
 *   un slot perime : l'attache le recupere (1 anomalie) ; tous les inscrits
 *   lisent chaque publication.
 * ======================================================================== */
void DT_seq_test_t28_registry_full(TEST_case_t *tc);

/* ========================================================================
 * T29 – Curseur incoherent (repli de pub_seq - last_seq, revue du 08/10, N3)
 *   Capacite 3. Un abonne qui ne lit rien pendant 2^32 + 2 publications a
 *   le meme retard (2) qu'apres 2 publications, mais un curseur decale de
 *   2^32 mod 3 = 1 slot : on reproduit ce decalage. read, puis peek(1),
 *   rendent DT_DATA_LOSS avec la plus ancienne donnee, et les lectures
 *   suivantes sont exactes.
 * ======================================================================== */
void DT_seq_test_t29_cursor_check(TEST_case_t *tc);

#endif /* DT_SEQ_TEST_H */
