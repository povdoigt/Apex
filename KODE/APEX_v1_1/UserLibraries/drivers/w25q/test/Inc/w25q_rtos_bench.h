#ifndef W25Q_RTOS_BENCH_H
#define W25Q_RTOS_BENCH_H

#include "w25q_rtos.h"

#if (APEX_CFG_SCHED_RTOS == 1)

/* ========================================================================
 * Banc de performance de la couche RTOS du driver W25Q
 *
 * Mesures (DWT, en cycles CPU, resumees en min / mediane / p99 / max / moy) :
 *   A1  Couche SPI2 RTOS : cout fixe Begin/End, Tx/Rx DMA selon la taille,
 *       compare au HAL bloquant (meme bus, memes tailles).
 *   A2  Primitives niveau 1 : verrou puce, ReadStatus (avec / sans verrou),
 *       ReadID, WaitForReady puce libre, WREN / WRDI.
 *   A3  Surcout du scheduler : spawn -> debut du corps, fin -> retour du
 *       join, aller-retour a priorite haute / basse, tache SendCmd.
 *   A4  Effacements 4 / 32 / 64 KB (commande + attente BUSY).
 *   A5  PageProgram_RTOS direct, 1 a 256 octets.
 *   A6  Taches Write / Read selon la taille (debit), relecture verifiee.
 *   B   CPU laisse aux taches de priorite inferieure pendant une operation
 *       (tache "spinner" basse priorite, rapportee a un osDelay de reference).
 *   C   Marge de pile de chaque pool apres le banc.
 *
 * Contexte d'execution :
 *   - Depuis un thread a osPriorityNormal, kernel demarre.
 *   - Le .c declare ses pools (TASK_POOL) : il ne doit pas etre lie avec
 *     w25q_rtos_test.c. Il ne compile qu'avec APEX_CFG_W25Q_BENCH a 1 dans
 *     main_config.h, ce qui exclut la suite de tests RTOS.
 *   - Zone flash utilisee : 0x02000000 - 0x0204FFFF (effacee puis ecrite).
 *   - Rien n'est imprime pendant les mesures : appeler
 *     W25Q_rtos_bench_print() une fois le terminal ouvert.
 * ======================================================================== */

/* Initialise la puce par la tache W25Q_Init (pool du banc). Cree le
   semaphore de la puce au premier appel ; la W25Q_t doit etre a zero. */
W25Q_STATE W25Q_rtos_bench_init_chip(W25Q_t *chip, const W25Q_config_t *config, uint32_t timeout_ms);

/* Execute tout le banc et garde les resultats en RAM. */
void W25Q_rtos_bench_run(W25Q_t *chip);

/* Imprime le rapport : tableaux, charge CPU, piles, puis CSV. */
void W25Q_rtos_bench_print(void (*print)(const char *), W25Q_STATE init_st);

#endif /* APEX_CFG_SCHED_RTOS == 1 */
#endif /* W25Q_RTOS_BENCH_H */
