#ifndef DP_SEQ_TEST_H
#define DP_SEQ_TEST_H

#include "data_packet.h"
#include "test.h"

#define DP_seq_test_N_TESTS 7

extern TEST_case_table_t DP_seq_test_cases[DP_seq_test_N_TESTS];

/* Suite de data_packet. Les cas tournent aussi sous RTOS (ils n'utilisent
 * que data_topic et une attente active) : RTOS_DT_USB les rejoue. */

/* ========================================================================
 * T0 – Arguments de data_packer_init, cycle init / free / init
 *   NULL, 0 ou 33 topics, topic NULL ou non initialise, element ou paquet
 *   trop grand, packer pas a zero : DT_BAD_ARG, et aucun abonne laisse sur
 *   les topics sources. data_packer_packet_size exact. init, free, init.
 * ======================================================================== */
void DP_seq_test_t0_init_args(TEST_case_t *tc);

/* ========================================================================
 * T1 – Paquet nominal
 *   3 sources (16, 8 et 36 o) : flags = 0b111, horodatage, charges utiles
 *   concatenees dans l'ordre, sources consommees.
 * ======================================================================== */
void DP_seq_test_t1_basic_packet(TEST_case_t *tc);

/* ========================================================================
 * T2 – Fenetre temporelle
 *   T = 10 ms autour de t : trop vieilles jetees, premiere dans la fenetre
 *   prise, trop jeune gardee pour le paquet suivant, source vide -> flag a
 *   0 et charge utile a zero ; bornes |t - ts| = T/2 et passage de
 *   l'horodatage par 2^32.
 * ======================================================================== */
void DP_seq_test_t2_time_window(TEST_case_t *tc);

/* ========================================================================
 * T3 – Paquets publies par copie, champs absents a zero
 *   10 paquets dans un topic de 3, une source sur deux absente : l'abonne
 *   en retard lit DT_DATA_LOSS puis les 3 derniers paquets intacts, et un
 *   champ absent vaut zero (pas de reste d'un paquet precedent).
 * ======================================================================== */
void DP_seq_test_t3_copy_publish(TEST_case_t *tc);

/* ========================================================================
 * T4 – Source liberee pendant l'usage
 *   data_topic_free d'une source detache l'abonne du packer : son flag
 *   reste a 0 sans erreur, les autres sources sont toujours empaquetees.
 * ======================================================================== */
void DP_seq_test_t4_source_freed(TEST_case_t *tc);

/* ========================================================================
 * T5 – Concurrence avec une interruption (TIM5, 20 kHz)
 *   L'ISR publie la source (8 mots = numero) et relit le topic des paquets
 *   avec un abonne en retard d'exactement `capacity` paquets ; la boucle
 *   principale construit des paquets pendant 300 ms et relit les siens.
 *   Aucun paquet incoherent (mots differents, flag sans charge ou l'inverse),
 *   des deux cotes. Avant le correctif (paquet construit dans le slot head,
 *   source lue par pointeur), les deux etaient possibles.
 * ======================================================================== */
void DP_seq_test_t5_isr_concurrency(TEST_case_t *tc);

/* ========================================================================
 * T6 – Rejet borne face a un flot d'echantillons trop vieux (revue du 08/10, N4)
 *   TIM5 publie sur une source de capacite 4 des echantillons toujours trop
 *   vieux (100 kHz en sequentiel, 40 kHz sous RTOS), plus vite que le packer
 *   ne les rejette. Chaque data_packer_build_publish rend la main en 2 ms au
 *   plus, le champ de cette source est toujours absent. Sans borne sur la
 *   boucle de rejet, le premier appel ne rendrait jamais la main.
 * ======================================================================== */
void DP_seq_test_t6_bounded_discard(TEST_case_t *tc);

#endif /* DP_SEQ_TEST_H */
