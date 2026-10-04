#include "w25q_rtos_bench.h"

/* Compile seulement pour le projet de banc : il definit les memes pools que
   w25q_rtos_test.c, les deux ne peuvent pas etre lies ensemble. */
#if (APEX_CFG_SCHED_RTOS == 1) && (APEX_CFG_PROFILE_TEST == 1) && (APEX_ENABLE_W25Q512 == 1) && (APEX_CFG_W25Q_BENCH == 1)

#include "bench.h"

#include <stdio.h>
#include <string.h>

/* ========================================================================
 * Parametres
 * ======================================================================== */
#define N_SPL            200U       /* Echantillons des micro-mesures            */
#define N_PP             16U        /* Pages programmees par taille (A5)         */
#define BIG_BYTES        16384U     /* Plus gros transfert Write / Read          */
#define OP_TIMEOUT_MS    3000U      /* Par attente : couvre un effacement 64 KB  */
#define JOIN_TIMEOUT_MS  10000U
#define PRIO_JOB         osPriorityAboveNormal
#define PRIO_SPIN        osPriorityBelowNormal
#define LOAD_REF_MS      100U       /* Duree de la mesure de reference (B)       */
#define HAL_TIMEOUT_MS   100U       /* HAL bloquant : large, -O0 compris         */

/* Zone du banc : loin des zones de la suite de tests (< 0x060000). */
#define BENCH_BASE   0x02000000UL
#define Z_ERASE4K    (BENCH_BASE + 0x00000UL)  /* 9 secteurs 4 KB (8 en A4, 1 en B) */
#define Z_ERASE32K   (BENCH_BASE + 0x10000UL)  /* 2 blocs 32 KB                     */
#define Z_PP         (BENCH_BASE + 0x20000UL)  /* Bloc 64 KB : PageProgram (A5)     */
#define Z_WRITE      (BENCH_BASE + 0x30000UL)  /* Bloc 64 KB : taches Write / Read  */
#define Z_SPARE      (BENCH_BASE + 0x40000UL)  /* Bloc 64 KB : mesures de charge B  */

#define SECTOR_BYTES 4096UL
#define BLK32_BYTES  0x8000UL

/* Codes propres au banc (hors W25Q_STATE). */
#define RT_SPAWN_FAIL    ((task_ret_t)-1)
#define RT_JOIN_TIMEOUT  ((task_ret_t)-2)
#define RT_VERIFY_FAIL   ((task_ret_t)-3)
#define RT_NOT_SET       ((task_ret_t)-99)

#define MAX_RESULTS      64U
#define MAX_LOAD         8U

/* ========================================================================
 * Taches propres au banc
 * ======================================================================== */
typedef struct {
    volatile uint32_t *t_start;   /* DWT a l'entree du corps   */
    volatile uint32_t *t_end;     /* DWT juste avant le return */
} W25QB_Nop_args_t;

/* Corps vide horodate : mesure le surcout du scheduler seul. */
TASK_DECLARE(W25QB_Nop, W25QB_Nop_args_t, 512);

typedef struct {
    volatile uint32_t *count;
    volatile bool     *stop;
} W25QB_Spin_args_t;

/* Compte tant qu'il a le CPU : sa vitesse mesure le CPU laisse aux taches basses. */
TASK_DECLARE(W25QB_Spin, W25QB_Spin_args_t, 512);

TASK_DEFINE(W25QB_Nop) {
    *args->t_start = BENCH_now();
    *args->t_end   = BENCH_now();
    return 0;
}

TASK_DEFINE(W25QB_Spin) {
    while (!*args->stop) {
        (*args->count)++;
    }
    return 0;
}

/* ========================================================================
 * Pools : toute la RAM des taches du banc
 * ======================================================================== */
TASK_POOL(W25Q_Init,    1);
TASK_POOL(W25Q_SendCmd, 1);
TASK_POOL(W25Q_Write,   1);
TASK_POOL(W25Q_Read,    1);
TASK_POOL(W25QB_Nop,    1);
TASK_POOL(W25QB_Spin,   1);

static const task_desc_t *const bench_pools[] = {
    &TASK_W25Q_Init, &TASK_W25Q_SendCmd, &TASK_W25Q_Write,
    &TASK_W25Q_Read, &TASK_W25QB_Nop,    &TASK_W25QB_Spin,
};
#define N_POOLS (sizeof(bench_pools) / sizeof(bench_pools[0]))

/* ========================================================================
 * Etat et buffers – tous statiques
 * ======================================================================== */
static W25Q_t *chip = NULL;

static uint32_t spl_a[N_SPL];
static uint32_t spl_b[N_SPL];
static uint32_t spl_c[N_SPL];

static uint8_t tx_buf[W25Q_MEM_PAGE_SIZE];
static uint8_t src_buf[BIG_BYTES];
static uint8_t dst_buf[BIG_BYTES];

static task_ret_t job_ret;

static BENCH_result_t results[MAX_RESULTS];
static size_t         n_results;

typedef struct {
    const char *name;
    float       dur_ms;
    float       cpu_free_pct;  /* CPU laisse aux taches de priorite inferieure */
} load_result_t;

static load_result_t load_results[MAX_LOAD];
static size_t        n_load;

/* Premiere erreur rencontree, et leur nombre : le banc continue malgre tout. */
static struct {
    const char *where;
    int32_t     code;
    uint32_t    count;
} bench_err;

static void note_err(const char *where, int32_t code) {
    if (bench_err.count++ == 0u) {
        bench_err.where = where;
        bench_err.code  = code;
    }
}

#define CHECK(where_, expr_)                                                   \
    do {                                                                       \
        int32_t st_ = (int32_t)(expr_);                                        \
        if (st_ != 0) note_err((where_), st_);                                 \
    } while (0)

static void add(const char *group, const char *name, uint32_t bytes, uint32_t *samples, uint32_t n) {
    if (n_results < MAX_RESULTS) {
        BENCH_summarize(&results[n_results++], group, name, bytes, samples, n);
    }
}

/* ========================================================================
 * Helpers : jobs du driver et effacement
 * ======================================================================== */
static task_ret_t run_write(uint32_t addr, const uint8_t *buf, uint32_t len) {
    job_ret = RT_NOT_SET;
    task_h_t h = W25Q_Write_spawn(
        &(W25Q_Write_args_t){ .chip = chip, .buf = buf, .addr = addr, .len = len, .timeout_ms = OP_TIMEOUT_MS },
        &(task_attr_t){ .priority = PRIO_JOB, .ret = &job_ret, .join_bit = 1u });
    if (!task_h_valid(h)) return RT_SPAWN_FAIL;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) return RT_JOIN_TIMEOUT;
    return job_ret;
}

static task_ret_t run_read(uint32_t addr, uint8_t *buf, uint32_t len) {
    job_ret = RT_NOT_SET;
    task_h_t h = W25Q_Read_spawn(
        &(W25Q_Read_args_t){ .chip = chip, .buf = buf, .addr = addr, .len = len, .timeout_ms = OP_TIMEOUT_MS },
        &(task_attr_t){ .priority = PRIO_JOB, .ret = &job_ret, .join_bit = 1u });
    if (!task_h_valid(h)) return RT_SPAWN_FAIL;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) return RT_JOIN_TIMEOUT;
    return job_ret;
}

/* Les effacements n'attendent pas la fin (drapeau DEVICE_BUSY) : on l'attend ici. */
static W25Q_STATE erase_wait(uint8_t cmd, uint32_t addr) {
    W25Q_STATE st = W25Q_SendCmdAddr_RTOS(chip, cmd, addr, OP_TIMEOUT_MS);
    if (st == W25Q_OK) st = W25Q_WaitForReady_RTOS(chip, OP_TIMEOUT_MS);
    return st;
}

/* ========================================================================
 * A1 – Couche SPI2 RTOS
 *
 * CS de la puce bas, premier octet 05h (Read SR1) : la puce ignore la suite
 * de MOSI et renvoie SR1 en boucle. Aucun octet ne peut etre pris pour une
 * commande, quelle que soit la taille.
 * ======================================================================== */
static const uint16_t tx_sizes[] = { 1u, 5u, 16u, 64u, 256u };
static const char *const tx_dma_names[] = {
    "Tx DMA 1 o", "Tx DMA 5 o", "Tx DMA 16 o", "Tx DMA 64 o", "Tx DMA 256 o" };
static const char *const tx_hal_names[] = {
    "Tx HAL bloquant 1 o", "Tx HAL bloquant 5 o", "Tx HAL bloquant 16 o",
    "Tx HAL bloquant 64 o", "Tx HAL bloquant 256 o" };

static const uint16_t rx_sizes[] = { 1u, 16u, 256u, 4096u };
static const char *const rx_dma_names[] = {
    "Rx DMA 1 o", "Rx DMA 16 o", "Rx DMA 256 o", "Rx DMA 4096 o" };
static const char *const rx_hal_names[] = {
    "Rx HAL bloquant 1 o", "Rx HAL bloquant 16 o", "Rx HAL bloquant 256 o", "Rx HAL bloquant 4096 o" };

#define N_TX (sizeof(tx_sizes) / sizeof(tx_sizes[0]))
#define N_RX (sizeof(rx_sizes) / sizeof(rx_sizes[0]))

static void bench_a1_spi(void) {
    static const char G[] = "A1 Couche SPI2 RTOS (DMA) vs HAL bloquant";
    SPI_HandleTypeDef *hspi   = chip->config.bus.hspi;
    GPIO_TypeDef      *cs_bnk = chip->config.bus.cs_bank;
    uint16_t           cs_pin = chip->config.bus.cs_pin;
    uint32_t t0, t1;

    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = W25Q_READ_SR1;

    /* Le verrou de la puce est garde tout le groupe : on parle au bus en direct. */
    if (osSemaphoreAcquire(chip->sem_id, OP_TIMEOUT_MS) != osOK) {
        note_err("A1 verrou puce", W25Q_LOCK_TIMEOUT);
        return;
    }

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "DWT (mesure a vide)", 0u, spl_a, N_SPL);

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        CHECK("A1 Begin", SPI_Begin_DMA_RTOS(hspi, cs_bnk, cs_pin));
        CHECK("A1 End",   SPI_End_DMA_RTOS(hspi, cs_bnk, cs_pin));
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "Begin + End (bus + CS)", 0u, spl_a, N_SPL);

    for (uint32_t k = 0; k < N_TX; k++) {
        for (uint32_t i = 0; i < N_SPL; i++) {
            CHECK("A1 Begin", SPI_Begin_DMA_RTOS(hspi, cs_bnk, cs_pin));
            t0 = BENCH_now();
            CHECK("A1 Tx DMA", SPI_Transmit_DMA_RTOS(hspi, tx_buf, tx_sizes[k]));
            t1 = BENCH_now();
            CHECK("A1 End", SPI_End_DMA_RTOS(hspi, cs_bnk, cs_pin));
            spl_a[i] = t1 - t0;
        }
        add(G, tx_dma_names[k], tx_sizes[k], spl_a, N_SPL);
    }

    for (uint32_t k = 0; k < N_TX; k++) {
        for (uint32_t i = 0; i < N_SPL; i++) {
            CHECK("A1 Begin", SPI_Begin_DMA_RTOS(hspi, cs_bnk, cs_pin));
            t0 = BENCH_now();
            CHECK("A1 Tx HAL", HAL_SPI_Transmit(hspi, tx_buf, tx_sizes[k], HAL_TIMEOUT_MS));
            t1 = BENCH_now();
            CHECK("A1 End", SPI_End_DMA_RTOS(hspi, cs_bnk, cs_pin));
            spl_a[i] = t1 - t0;
        }
        add(G, tx_hal_names[k], tx_sizes[k], spl_a, N_SPL);
    }

    for (uint32_t k = 0; k < N_RX; k++) {
        for (uint32_t i = 0; i < N_SPL; i++) {
            CHECK("A1 Begin", SPI_Begin_DMA_RTOS(hspi, cs_bnk, cs_pin));
            CHECK("A1 Tx DMA", SPI_Transmit_DMA_RTOS(hspi, tx_buf, 1u));
            t0 = BENCH_now();
            CHECK("A1 Rx DMA", SPI_Receive_DMA_RTOS(hspi, dst_buf, rx_sizes[k]));
            t1 = BENCH_now();
            CHECK("A1 End", SPI_End_DMA_RTOS(hspi, cs_bnk, cs_pin));
            spl_a[i] = t1 - t0;
        }
        add(G, rx_dma_names[k], rx_sizes[k], spl_a, N_SPL);
    }

    for (uint32_t k = 0; k < N_RX; k++) {
        for (uint32_t i = 0; i < N_SPL; i++) {
            CHECK("A1 Begin", SPI_Begin_DMA_RTOS(hspi, cs_bnk, cs_pin));
            CHECK("A1 Tx HAL", HAL_SPI_Transmit(hspi, tx_buf, 1u, HAL_TIMEOUT_MS));
            t0 = BENCH_now();
            CHECK("A1 Rx HAL", HAL_SPI_Receive(hspi, dst_buf, rx_sizes[k], HAL_TIMEOUT_MS));
            t1 = BENCH_now();
            CHECK("A1 End", SPI_End_DMA_RTOS(hspi, cs_bnk, cs_pin));
            spl_a[i] = t1 - t0;
        }
        add(G, rx_hal_names[k], rx_sizes[k], spl_a, N_SPL);
    }

    (void)osSemaphoreRelease(chip->sem_id);
}

/* ========================================================================
 * A2 – Primitives niveau 1 (appel direct depuis le thread du banc)
 * ======================================================================== */
static void bench_a2_primitives(void) {
    static const char G[] = "A2 Primitives niveau 1";
    uint32_t t0, t1;
    uint8_t  id[3];

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        (void)osSemaphoreAcquire(chip->sem_id, OP_TIMEOUT_MS);
        (void)osSemaphoreRelease(chip->sem_id);
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "Verrou puce pris + rendu", 0u, spl_a, N_SPL);

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        CHECK("A2 ReadStatus", W25Q_ReadStatus_RTOS(chip, 1u, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "ReadStatus SR1 (verrou)", 0u, spl_a, N_SPL);

    if (osSemaphoreAcquire(chip->sem_id, OP_TIMEOUT_MS) == osOK) {
        for (uint32_t i = 0; i < N_SPL; i++) {
            t0 = BENCH_now();
            CHECK("A2 ReadStatus NoLock", W25Q_ReadStatus_RTOS_NoLock(chip, 1u));
            t1 = BENCH_now();
            spl_a[i] = t1 - t0;
        }
        (void)osSemaphoreRelease(chip->sem_id);
        add(G, "ReadStatus SR1 (NoLock)", 0u, spl_a, N_SPL);
    } else {
        note_err("A2 verrou puce", W25Q_LOCK_TIMEOUT);
    }

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        CHECK("A2 ReadID", W25Q_ReadID_RTOS(chip, id, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "ReadID (JEDEC, 3 o)", 0u, spl_a, N_SPL);

    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        CHECK("A2 WaitForReady", W25Q_WaitForReady_RTOS(chip, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
    }
    add(G, "WaitForReady (puce libre)", 0u, spl_a, N_SPL);

    /* WREN et WRDI alternes, chacun mesure dans son buffer. */
    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        CHECK("A2 WREN", W25Q_SendCmd_RTOS(chip, W25Q_WRITE_ENABLE, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_a[i] = t1 - t0;
        t0 = BENCH_now();
        CHECK("A2 WRDI", W25Q_SendCmd_RTOS(chip, W25Q_WRITE_DISABLE, OP_TIMEOUT_MS));
        t1 = BENCH_now();
        spl_b[i] = t1 - t0;
    }
    add(G, "SendCmd WREN 06h", 0u, spl_a, N_SPL);
    add(G, "SendCmd WRDI 04h", 0u, spl_b, N_SPL);
}

/* ========================================================================
 * A3 – Surcout du scheduler
 * ======================================================================== */
static void bench_a3_scheduler(void) {
    static const char G[] = "A3 Scheduler (spawn + join)";
    static volatile uint32_t t_start, t_end;
    static task_ret_t        nop_ret;
    uint32_t t0, t2;

    /* Creation des threads hors mesure. */
    CHECK("A3 prewarm Nop",     task_prewarm(&TASK_W25QB_Nop));
    CHECK("A3 prewarm SendCmd", task_prewarm(&TASK_W25Q_SendCmd));

    /* Priorite haute : le corps preempte l'appelant des le spawn. */
    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        task_h_t h = W25QB_Nop_spawn(
            &(W25QB_Nop_args_t){ .t_start = &t_start, .t_end = &t_end },
            &(task_attr_t){ .priority = PRIO_JOB, .ret = &nop_ret, .join_bit = 1u });
        if (!task_h_valid(h) || task_join(h, JOIN_TIMEOUT_MS) != osOK) {
            note_err("A3 Nop prio+", RT_JOIN_TIMEOUT);
        }
        t2 = BENCH_now();
        spl_a[i] = t_start - t0;
        spl_b[i] = t2 - t_end;
        spl_c[i] = t2 - t0;
    }
    add(G, "Nop prio+ : spawn -> debut corps", 0u, spl_a, N_SPL);
    add(G, "Nop prio+ : fin corps -> join", 0u, spl_b, N_SPL);
    add(G, "Nop prio+ : aller-retour", 0u, spl_c, N_SPL);

    /* Priorite basse : le corps ne tourne que quand l'appelant bloque dans le join. */
    for (uint32_t i = 0; i < N_SPL; i++) {
        t0 = BENCH_now();
        task_h_t h = W25QB_Nop_spawn(
            &(W25QB_Nop_args_t){ .t_start = &t_start, .t_end = &t_end },
            &(task_attr_t){ .priority = PRIO_SPIN, .ret = &nop_ret, .join_bit = 1u });
        if (!task_h_valid(h) || task_join(h, JOIN_TIMEOUT_MS) != osOK) {
            note_err("A3 Nop prio-", RT_JOIN_TIMEOUT);
        }
        t2 = BENCH_now();
        spl_a[i] = t2 - t0;
    }
    add(G, "Nop prio- : aller-retour", 0u, spl_a, N_SPL);

    /* Meme operation que l'appel direct WRDI de A2 : l'ecart est le cout de la tache. */
    for (uint32_t i = 0; i < N_SPL; i++) {
        job_ret = RT_NOT_SET;
        t0 = BENCH_now();
        task_h_t h = W25Q_SendCmd_spawn(
            &(W25Q_SendCmd_args_t){ .chip = chip, .cmd = W25Q_WRITE_DISABLE, .timeout_ms = OP_TIMEOUT_MS },
            &(task_attr_t){ .priority = PRIO_JOB, .ret = &job_ret, .join_bit = 1u });
        if (!task_h_valid(h) || task_join(h, JOIN_TIMEOUT_MS) != osOK) {
            note_err("A3 SendCmd", RT_JOIN_TIMEOUT);
        }
        t2 = BENCH_now();
        CHECK("A3 SendCmd ret", job_ret);
        spl_a[i] = t2 - t0;
    }
    add(G, "Tache SendCmd WRDI : aller-retour", 0u, spl_a, N_SPL);
}

/* ========================================================================
 * A4 – Effacements (commande + attente BUSY)
 * ======================================================================== */
static void bench_a4_erase(void) {
    static const char G[] = "A4 Effacements (commande + attente BUSY)";
    uint32_t t0, t1, t2;
    uint32_t n = 0;

    /* 4 KB : on separe l'envoi de la commande (retour immediat) de l'attente. */
    for (uint32_t i = 0; i < 8u; i++) {
        t0 = BENCH_now();
        W25Q_STATE st = W25Q_SendCmdAddr_RTOS(chip, W25Q_SECTOR_ERASE_4B, Z_ERASE4K + i * SECTOR_BYTES, OP_TIMEOUT_MS);
        t1 = BENCH_now();
        if (st == W25Q_OK) st = W25Q_WaitForReady_RTOS(chip, OP_TIMEOUT_MS);
        t2 = BENCH_now();
        CHECK("A4 erase 4K", st);
        spl_a[n] = t1 - t0;
        spl_b[n] = t2 - t0;
        n++;
    }
    add(G, "Commande 21h seule (sans attente)", 0u, spl_a, n);
    add(G, "Secteur 4 KB", SECTOR_BYTES, spl_b, n);

    n = 0;
    for (uint32_t i = 0; i < 2u; i++) {
        t0 = BENCH_now();
        CHECK("A4 erase 32K", erase_wait(W25Q_32KB_BLOCK_ERASE, Z_ERASE32K + i * BLK32_BYTES));
        spl_a[n++] = BENCH_now() - t0;
    }
    add(G, "Bloc 32 KB", BLK32_BYTES, spl_a, n);

    /* Les zones PageProgram et Write sont preparees par ces effacements. */
    static const uint32_t blk64[] = { Z_PP, Z_WRITE, Z_SPARE };
    n = 0;
    for (uint32_t i = 0; i < 3u; i++) {
        t0 = BENCH_now();
        CHECK("A4 erase 64K", erase_wait(W25Q_64KB_BLOCK_ERASE_4B, blk64[i]));
        spl_a[n++] = BENCH_now() - t0;
    }
    add(G, "Bloc 64 KB", 0x10000UL, spl_a, n);
}

/* ========================================================================
 * A5 – PageProgram_RTOS direct (zone Z_PP vierge)
 * ======================================================================== */
static const uint16_t pp_sizes[] = { 1u, 16u, 64u, 128u, 256u };
static const char *const pp_names[] = {
    "PageProgram 1 o", "PageProgram 16 o", "PageProgram 64 o",
    "PageProgram 128 o", "PageProgram 256 o" };
#define N_PP_SIZES (sizeof(pp_sizes) / sizeof(pp_sizes[0]))

static void bench_a5_page_program(void) {
    static const char G[] = "A5 PageProgram_RTOS (appel direct)";
    uint32_t page = 0;
    uint32_t t0;

    for (uint32_t k = 0; k < N_PP_SIZES; k++) {
        for (uint32_t i = 0; i < N_PP; i++) {
            t0 = BENCH_now();
            CHECK("A5 PageProgram", W25Q_PageProgram_RTOS(chip, src_buf, Z_PP + page * W25Q_MEM_PAGE_SIZE,
                                                          pp_sizes[k], OP_TIMEOUT_MS));
            spl_a[i] = BENCH_now() - t0;
            page++;
        }
        add(G, pp_names[k], pp_sizes[k], spl_a, N_PP);
    }
}

/* ========================================================================
 * A6 – Taches Write / Read (zone Z_WRITE vierge)
 * ======================================================================== */
static void bench_a6_tasks(void) {
    static const char G[] = "A6 Taches Write / Read (spawn + join)";
    static const struct { uint32_t len; uint32_t n; const char *name; } wr[] = {
        {   256u, 8u, "Write 256 o"   },
        {  4096u, 4u, "Write 4 KB"    },
        { 16384u, 2u, "Write 16 KB"   },
    };
    static const struct { uint32_t len; uint32_t n; const char *name; } rd[] = {
        {    16u, 50u, "Read 16 o"    },
        {   256u, 50u, "Read 256 o"   },
        {  4096u, 20u, "Read 4 KB"    },
        { 16384u, 10u, "Read 16 KB"   },
    };
    uint32_t addr = Z_WRITE;
    uint32_t last = Z_WRITE;
    uint32_t t0;

    CHECK("A6 prewarm Write", task_prewarm(&TASK_W25Q_Write));
    CHECK("A6 prewarm Read",  task_prewarm(&TASK_W25Q_Read));

    for (uint32_t k = 0; k < sizeof(wr) / sizeof(wr[0]); k++) {
        for (uint32_t i = 0; i < wr[k].n; i++) {
            last = addr;
            t0 = BENCH_now();
            CHECK("A6 Write", run_write(addr, src_buf, wr[k].len));
            spl_a[i] = BENCH_now() - t0;
            addr += wr[k].len;
        }
        add(G, wr[k].name, wr[k].len, spl_a, wr[k].n);
    }

    /* Relecture du dernier Write 16 KB : des chiffres sur des donnees fausses ne valent rien. */
    memset(dst_buf, 0, sizeof(dst_buf));
    CHECK("A6 relecture", run_read(last, dst_buf, BIG_BYTES));
    if (memcmp(dst_buf, src_buf, BIG_BYTES) != 0) {
        note_err("A6 verification relecture", RT_VERIFY_FAIL);
    }

    for (uint32_t k = 0; k < sizeof(rd) / sizeof(rd[0]); k++) {
        for (uint32_t i = 0; i < rd[k].n; i++) {
            t0 = BENCH_now();
            CHECK("A6 Read", run_read(Z_WRITE, dst_buf, rd[k].len));
            spl_a[i] = BENCH_now() - t0;
        }
        add(G, rd[k].name, rd[k].len, spl_a, rd[k].n);
    }
}

/* ========================================================================
 * B – CPU laisse aux taches de priorite inferieure
 *
 * Un spinner a PRIO_SPIN compte tant qu'il a le CPU. Sa vitesse pendant un
 * osDelay du thread du banc sert de reference (100 %) ; pendant une
 * operation, le rapport donne la part de CPU que l'operation laisse.
 * ======================================================================== */
static volatile uint32_t spin_count;
static volatile bool     spin_stop;
static float             spin_ref;   /* comptes par cycle, CPU libre */

static void load_add(const char *name, uint32_t c0, uint32_t t0) {
    uint32_t dt = BENCH_now() - t0;
    uint32_t dc = spin_count - c0;
    if (n_load < MAX_LOAD && dt > 0u) {
        float rate = (float)dc / (float)dt;
        load_results[n_load++] = (load_result_t){
            .name         = name,
            .dur_ms       = BENCH_cyc_to_us(dt) / 1000.0f,
            .cpu_free_pct = (spin_ref > 0.0f) ? 100.0f * rate / spin_ref : 0.0f,
        };
        if (spin_ref <= 0.0f) spin_ref = rate;   /* premier appel : la reference */
    }
}

static void bench_b_cpu_load(void) {
    static task_ret_t spin_ret;
    uint32_t c0, t0;

    spin_count = 0u;
    spin_stop  = false;
    spin_ref   = 0.0f;

    task_h_t h = W25QB_Spin_spawn(
        &(W25QB_Spin_args_t){ .count = &spin_count, .stop = &spin_stop },
        &(task_attr_t){ .priority = PRIO_SPIN, .ret = &spin_ret, .join_bit = 2u });
    if (!task_h_valid(h)) {
        note_err("B spawn spinner", RT_SPAWN_FAIL);
        return;
    }

    c0 = spin_count; t0 = BENCH_now();
    osDelay(LOAD_REF_MS);
    load_add("Reference : osDelay(100)", c0, t0);
    if (n_load > 0u) load_results[0].cpu_free_pct = 100.0f;

    c0 = spin_count; t0 = BENCH_now();
    CHECK("B erase 4K", erase_wait(W25Q_SECTOR_ERASE_4B, Z_ERASE4K + 8u * SECTOR_BYTES));
    load_add("Effacement 4 KB + attente", c0, t0);

    c0 = spin_count; t0 = BENCH_now();
    CHECK("B erase 64K", erase_wait(W25Q_64KB_BLOCK_ERASE_4B, Z_SPARE));
    load_add("Effacement 64 KB + attente", c0, t0);

    c0 = spin_count; t0 = BENCH_now();
    CHECK("B Write 16K", run_write(Z_SPARE, src_buf, BIG_BYTES));
    load_add("Tache Write 16 KB", c0, t0);

    c0 = spin_count; t0 = BENCH_now();
    CHECK("B Read 16K", run_read(Z_SPARE, dst_buf, BIG_BYTES));
    load_add("Tache Read 16 KB", c0, t0);

    spin_stop = true;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) {
        note_err("B join spinner", RT_JOIN_TIMEOUT);
    }
}

/* ========================================================================
 * API
 * ======================================================================== */
W25Q_STATE W25Q_rtos_bench_init_chip(W25Q_t *w25q, const W25Q_config_t *config, uint32_t timeout_ms) {
    static task_ret_t ret;
    ret = RT_NOT_SET;
    task_h_t h = W25Q_Init_spawn(&(W25Q_Init_args_t){ .chip = w25q, .config = *config, .timeout_ms = timeout_ms },
                                 &(task_attr_t){ .priority = osPriorityNormal, .ret = &ret, .join_bit = 1u });
    if (!task_h_valid(h)) return W25Q_SEM_ERR;
    if (task_join(h, JOIN_TIMEOUT_MS) != osOK) return W25Q_BUSY_TIMEOUT;
    return W25Q_RET(ret);
}

void W25Q_rtos_bench_run(W25Q_t *w25q) {
    chip      = w25q;
    n_results = 0u;
    n_load    = 0u;
    memset(&bench_err, 0, sizeof(bench_err));

    BENCH_init();

    /* Motif non trivial pour toutes les ecritures. */
    for (uint32_t i = 0; i < BIG_BYTES; i++) {
        src_buf[i] = (uint8_t)((i * 7u) ^ (i >> 8));
    }

    bench_a1_spi();
    bench_a2_primitives();
    bench_a3_scheduler();
    bench_a4_erase();
    bench_a5_page_program();
    bench_a6_tasks();
    bench_b_cpu_load();
}

void W25Q_rtos_bench_print(void (*print)(const char *), W25Q_STATE init_st) {
    char line[160];
    uint32_t sck_khz = 0u;

    if (chip != NULL) {
        const SPI_HandleTypeDef *hspi = chip->config.bus.hspi;
        /* SPI1 est le seul SPI sur APB2 de ce composant. */
        uint32_t pclk  = (hspi->Instance == SPI1) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
        uint32_t presc = 2u << (hspi->Init.BaudRatePrescaler >> SPI_CR1_BR_Pos);
        sck_khz = pclk / presc / 1000u;
    }

    print("W25Q RTOS - banc de performance\r\n");
#ifdef DEBUG
    print("Build : Debug (-O0)\r\n");
#else
    print("Build : Release (-Os)\r\n");
#endif
    snprintf(line, sizeof(line), "SYSCLK %lu MHz, SCK %lu kHz, init puce : %s\r\n",
             (unsigned long)(SystemCoreClock / 1000000u), (unsigned long)sck_khz,
             (init_st == W25Q_OK) ? "OK" : "ECHEC");
    print(line);
    if (bench_err.count == 0u) {
        print("Erreurs : aucune\r\n");
    } else {
        snprintf(line, sizeof(line), "Erreurs : %lu, premiere : %s (code %ld)\r\n",
                 (unsigned long)bench_err.count, bench_err.where, (long)bench_err.code);
        print(line);
    }

    BENCH_print_table(results, n_results, print);

    snprintf(line, sizeof(line), "\r\n== B CPU laisse a une tache de priorite inferieure\r\n%-34s %10s %10s\r\n",
             "Operation", "duree ms", "CPU libre");
    print(line);
    for (size_t i = 0; i < n_load; i++) {
        snprintf(line, sizeof(line), "%-34s %10.2f %9.1f%%\r\n",
                 load_results[i].name, load_results[i].dur_ms, load_results[i].cpu_free_pct);
        print(line);
    }

    snprintf(line, sizeof(line), "\r\n== C Marge de pile (instance 0)\r\n%-34s %10s %10s %10s\r\n",
             "Pool", "pile o", "libre o", "utilise o");
    print(line);
    for (size_t i = 0; i < N_POOLS; i++) {
        const task_desc_t *d = bench_pools[i];
        uint32_t free_b = task_stack_headroom(d, 0u);
        if (free_b == 0u) {
            snprintf(line, sizeof(line), "%-34s %10lu %10s %10s\r\n",
                     d->name, (unsigned long)d->stack_bytes, "-", "non cree");
        } else {
            snprintf(line, sizeof(line), "%-34s %10lu %10lu %10lu\r\n",
                     d->name, (unsigned long)d->stack_bytes, (unsigned long)free_b,
                     (unsigned long)(d->stack_bytes - free_b));
        }
        print(line);
    }

    BENCH_print_csv(results, n_results, print);
}

#endif /* APEX_CFG_SCHED_RTOS && APEX_CFG_PROFILE_TEST && APEX_ENABLE_W25Q512 && APEX_CFG_W25Q_BENCH */
