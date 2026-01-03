/*********************** BMI088 Driver — APEX monofichier *************************
 * Niveaux d'API: 
 *  - Niveau 0 : SPI (sync + RTOS)
 *  - Niveau 1 : Primitives registres (acc/gyr) + burst data
 *  - Niveau 2 : Logique capteur (init, set cfg, read, self-test, offset, readAll)
 *  - Section RTOS : wrappers non-bloquants & sémaphore
 *  - Section Tasks : (optionnel) producteurs de données
 **********************************************************************************/

#ifndef BMI088_IMU_H
#define BMI088_IMU_H


#include <stdbool.h>
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "peripherals/spi.h"

#include "utils/data_topic.h"
#include "utils/types.h"
#include "utils/scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ Header public (minimal) =========================== */

/* --- Unités de sortie (modifiable ici) --- */
#define BMI_ACCEL_UNIT_MS2   1   /* 1: m/s^2 ; 0: g */
#define BMI_GYRO_UNIT_DPS    1   /* 1: deg/s ; 0: rad/s */

#define BMI_READ_MASK         0b10000000

/* ========================================================================== */
/*                BMI088 — Accelerometer + Temperature Registers               */
/*                Address Order : 0x00 → 0x7E (Datasheet Rev.1.9)              */
/* ========================================================================== */

/**
 * @defgroup BMI088_Registers_Accelerometer BMI088 Accelerometer Registers
 * @brief Register map for the accelerometer and internal temperature sensor
 * @{
 */

/* Identification and Status ------------------------------------------------ */
#define BMI_ACC_CHIP_ID                 0x00U  /**< Device identification (reset: 0x1E) */
#define BMI_ACC_RESERVED_01             0x01U  /**< Reserved */
#define BMI_ACC_ERR_REG                 0x02U  /**< Error flags register */
#define BMI_ACC_STATUS                  0x03U  /**< Data ready / status flags */

/* Reserved 0x04–0x11 ------------------------------------------------------- */

/* Acceleration data (16-bit signed, little-endian) ------------------------- */
#define BMI_ACC_X_LSB                   0x12U
#define BMI_ACC_X_MSB                   0x13U
#define BMI_ACC_Y_LSB                   0x14U
#define BMI_ACC_Y_MSB                   0x15U
#define BMI_ACC_Z_LSB                   0x16U
#define BMI_ACC_Z_MSB                   0x17U

/* Sensor time (24-bit free-running counter) -------------------------------- */
#define BMI_SENSORTIME_0                0x18U
#define BMI_SENSORTIME_1                0x19U
#define BMI_SENSORTIME_2                0x1AU

/* Reserved 0x1B–0x1C ------------------------------------------------------- */

/* Interrupt status --------------------------------------------------------- */
#define BMI_ACC_INT_STAT_1              0x1DU

/* Reserved 0x1E–0x21 ------------------------------------------------------- */

/* Temperature sensor (11-bit signed) --------------------------------------- */
#define BMI_TEMP_MSB                    0x22U
#define BMI_TEMP_LSB                    0x23U

/* FIFO length and data ------------------------------------------------------ */
#define BMI_FIFO_LENGTH_0               0x24U
#define BMI_FIFO_LENGTH_1               0x25U
#define BMI_FIFO_DATA                   0x26U

/* Reserved 0x27–0x3F -------------------------------------------------------- */

/* Configuration registers --------------------------------------------------- */
#define BMI_ACC_CONF                    0x40U
#define BMI_ACC_RANGE                   0x41U
/* Reserved 0x42–0x44 -------------------- */
#define BMI_FIFO_DOWNS                  0x45U
#define BMI_FIFO_WTM_0                  0x46U
#define BMI_FIFO_WTM_1                  0x47U
#define BMI_FIFO_CONFIG_0               0x48U
#define BMI_FIFO_CONFIG_1               0x49U

/* Reserved 0x4A–0x52 ------------------------------------------------------- */

/* Interrupt configuration --------------------------------------------------- */
#define BMI_INT1_IO_CONF                0x53U
#define BMI_INT2_IO_CONF                0x54U
/* Reserved 0x55–0x57 -------------------- */
#define BMI_INT1_INT2_MAP_DATA          0x58U

/* Reserved 0x59–0x6C ------------------------------------------------------- */
#define BMI_ACC_RESERVED_6C             0x6CU

/* Self test and power management ------------------------------------------- */
#define BMI_ACC_SELF_TEST               0x6DU
/* Reserved 0x6E–0x7B -------------------- */
#define BMI_ACC_PWR_CONF                0x7CU
#define BMI_ACC_PWR_CTRL                0x7DU
#define BMI_ACC_SOFTRESET               0x7EU

/** @} */ /* End of Accelerometer register map */

/* ========================================================================== */
/*                    BMI088 — Gyroscope Registers Map                         */
/*                    Address Order : 0x00 → 0x3F (Datasheet Rev.1.9)          */
/* ========================================================================== */

/**
 * @defgroup BMI088_Registers_Gyroscope BMI088 Gyroscope Registers
 * @brief Register map for the gyroscope interface
 * @{
 */

/* Identification and Status ------------------------------------------------- */
#define BMI_GYR_CHIP_ID                 0x00U  /**< Device identification (reset: 0x0F) */
#define BMI_GYR_RATE_X_LSB              0x02U  /**< Angular rate X-axis LSB */
#define BMI_GYR_RATE_X_MSB              0x03U  /**< Angular rate X-axis MSB */
#define BMI_GYR_RATE_Y_LSB              0x04U  /**< Angular rate Y-axis LSB */
#define BMI_GYR_RATE_Y_MSB              0x05U  /**< Angular rate Y-axis MSB */
#define BMI_GYR_RATE_Z_LSB              0x06U  /**< Angular rate Z-axis LSB */
#define BMI_GYR_RATE_Z_MSB              0x07U  /**< Angular rate Z-axis MSB */
#define BMI_GYR_INT_STATUS_0            0x09U  /**< Interrupt status register 0 */
#define BMI_GYR_INT_STATUS_1            0x0AU  /**< Interrupt status register 1 */
#define BMI_GYR_INT_STATUS_2            0x0BU  /**< Interrupt status register 2 */
#define BMI_GYR_INT_STATUS_3            0x0CU  /**< Interrupt status register 3 */
#define BMI_GYR_FIFO_STATUS             0x0EU  /**< FIFO status flags */

/* Configuration registers --------------------------------------------------- */
#define BMI_GYR_RANGE                   0x0FU  /**< ±dps full-scale range (bits[2:0]) */
#define BMI_GYR_BANDWIDTH               0x10U  /**< Bandwidth / Output data rate */
#define BMI_GYR_LPM1                    0x11U  /**< Power / suspend control register */
#define BMI_GYR_SOFTRESET               0x14U  /**< Software reset command (write 0xB6) */

/* Interrupt control and I/O ------------------------------------------------- */
#define BMI_GYR_INT_CTRL                0x15U  /**< Interrupt control */
#define BMI_GYR_INT3_IO_CONF            0x16U  /**< Interrupt pin 3 configuration */
#define BMI_GYR_INT4_IO_CONF            0x17U  /**< Interrupt pin 4 configuration */
#define BMI_GYR_INT3_INT4_IO_MAP        0x18U  /**< Interrupt source mapping */

/* Offset and Non-volatile configuration ------------------------------------- */
#define BMI_GYR_AUTO_OFFSET             0x1AU  /**< Auto offset compensation control */
#define BMI_GYR_NV_CONF                 0x1BU  /**< Non-volatile memory configuration */

/* Reserved 0x1C–0x3B -------------------------------------------------------- */

/* Self-test and trimming ---------------------------------------------------- */
#define BMI_GYR_SELF_TEST               0x3CU  /**< Built-in self-test control */
#define BMI_GYR_TRIM_NVM_CTRL           0x3EU  /**< NVM programming / trimming control */

/* End of address space ------------------------------------------------------ */

/** @} */ /* End of Gyroscope register map */

/* ========================================================================== */
/*           BMI088 — Accelerometer + Temperature: Enums per Register         */
/*           Datasheet: Rev. 1.9 (01/2024), Accelerometer map 0x00–0x7E       */
/* ========================================================================== */
/**
 * @defgroup BMI088_Enums_Accelerometer BMI088 Accelerometer Enumerations
 * @brief Enumerations grouped per accelerometer register field
 * @{
 */

/* -------------------------------------------------------------------------- */
/* 0x00 — ACC_CHIP_ID : Device ID (RO, reset = 0x1E)                          */
/* -------------------------------------------------------------------------- */
/* Pas de champs configurables ni de flags documentés (valeur constante). */   /* :contentReference[oaicite:2]{index=2} */
#define BMI_ACC_CHIP_ID_VALUE  0x1EU

/* -------------------------------------------------------------------------- */
/* 0x02 — ACC_ERR_REG : Error flags (RO, reset = 0x00)                         */
/* -------------------------------------------------------------------------- */
/* [0] fatal_err — “chip not operational”; reset par POR ou soft-reset. */      /* :contentReference[oaicite:3]{index=3} */
typedef enum {
    BMI_ACC_ERR_REG_FATAL_ERR_NONE  = 0b00000000, /**< No fatal error */
    BMI_ACC_ERR_REG_FATAL_ERR_SET   = 0b00000001  /**< Fatal error occurred */
} bmi_acc_err_reg_fatal_err_t;
#define BMI_ACC_ERR_REG_FATAL_ERR_MASK   (0b00000001)

/* [4:2] error_code — erreurs persistantes liées à ACC_CONF.                   */
/* 0x00 : no error ; 0x01 : invalid data in ACC_CONF.                           */ /* :contentReference[oaicite:4]{index=4} */
typedef enum {
    BMI_ACC_ERR_REG_ERROR_CODE_NONE       = 0b00000000, /**< No error */
    BMI_ACC_ERR_REG_ERROR_CODE_CONF_ERROR = 0b00000100  /**< Invalid ACC_CONF */
} bmi_acc_err_reg_error_code_t;
#define BMI_ACC_ERR_REG_ERROR_CODE_MASK   (0b00011100)

/* -------------------------------------------------------------------------- */
/* 0x03 — ACC_STATUS : Status (RO, reset = 0x10)                               */
/* -------------------------------------------------------------------------- */
/* [7] acc_drdy — mis à 1 quand nouvelle donnée accel prête ; clear on read. */ /* :contentReference[oaicite:5]{index=5} */
typedef enum {
    BMI_ACC_STATUS_DRDY = 0b10000000  /**< New acceleration data ready */
} bmi_acc_status_drdy_t;
#define BMI_ACC_STATUS_DRDY_MASK (0b10000000)

/* -------------------------------------------------------------------------- */
/* 0x12–0x17 — ACC_X/Y/Z_LSB/MSB : 16-bit output (RO, reset = 0x00)           */
/* -------------------------------------------------------------------------- */
/* Données brutes 2’s complement, verrouillage MSB lors de lecture LSB.      */ /* :contentReference[oaicite:6]{index=6} */

/* -------------------------------------------------------------------------- */
/* 0x18–0x1A — SENSORTIME_0/1/2 : 24-bit counter (RO, reset = 0x00)           */
/* -------------------------------------------------------------------------- */
/* Incrément 39.0625 µs ; overflow ~655.36 s.                                 */ /* :contentReference[oaicite:7]{index=7} */

/* -------------------------------------------------------------------------- */
/* 0x1D — ACC_INT_STAT_1 : Interrupt status (RO, reset = 0x00)                */
/* -------------------------------------------------------------------------- */
/* [7] acc_drdy — statut d’interruption “data ready” (clear on read).         */ /* :contentReference[oaicite:8]{index=8} */
typedef enum {
    BMI_ACC_INT_STAT_1_DRDY = 0b10000000
} bmi_acc_int_stat_1_drdy_t;
#define BMI_ACC_INT_STAT_1_DRDY_MASK (0b10000000)

/* -------------------------------------------------------------------------- */
/* 0x22–0x23 — TEMP_MSB/LSB : 11-bit temperature data (RO, reset = 0x00)      */
/* -------------------------------------------------------------------------- */
/* Résolution 0.125°C/LSB ; formule de conversion fournie en DS §5.3.7.      */ /* :contentReference[oaicite:9]{index=9} */
#define BMI_TEMP_LSB_MASK  0b11100000  /**< bits [7:5] of LSB are valid */


/* -------------------------------------------------------------------------- */
/* 0x24–0x25 — FIFO_LENGTH_0/1 : 14-bit FIFO byte counter (RO, reset = 0x00)  */
/* -------------------------------------------------------------------------- */
/* Empty FIFO = 0x8000 ; MAJ sur frame complète lue/écrite.                   */ /* :contentReference[oaicite:10]{index=10} */

/* -------------------------------------------------------------------------- */
/* 0x26 — FIFO_DATA : FIFO burst read (RO, reset = 0x00)                      */
/* -------------------------------------------------------------------------- */
/* Lecture en burst ; frames répétées si partiellement lues.                  */ /* :contentReference[oaicite:11]{index=11} */

/* -------------------------------------------------------------------------- */
/* 0x40 — ACC_CONF : Bandwidth & ODR (RW, reset = 0xA8)                       */
/* -------------------------------------------------------------------------- */
/* [7:4] acc_bwp — filtre/bande passante (OSR / normal).                      */ /* :contentReference[oaicite:12]{index=12} */
typedef enum {
    BMI_ACC_CONF_BWP_OSR4   = 0b10000000, /**< OSR4 (narrowest BW)          */
    BMI_ACC_CONF_BWP_OSR2   = 0b10010000, /**< OSR2                         */
    BMI_ACC_CONF_BWP_NORMAL = 0b10100000  /**< Normal (recommended)         */
} bmi_acc_conf_bwp_t;
#define BMI_ACC_CONF_BWP_MASK (0b11110000)

/* [3:0] acc_odr — Output Data Rate.                                          */ /* :contentReference[oaicite:13]{index=13} */
typedef enum {
    BMI_ACC_CONF_ODR_12_5_HZ  = 0b00000101,
    BMI_ACC_CONF_ODR_25_HZ    = 0b00000110,
    BMI_ACC_CONF_ODR_50_HZ    = 0b00000111,
    BMI_ACC_CONF_ODR_100_HZ   = 0b00001000,
    BMI_ACC_CONF_ODR_200_HZ   = 0b00001001,
    BMI_ACC_CONF_ODR_400_HZ   = 0b00001010,
    BMI_ACC_CONF_ODR_800_HZ   = 0b00001011,
    BMI_ACC_CONF_ODR_1600_HZ  = 0b00001100
} bmi_acc_conf_odr_t;
#define BMI_ACC_CONF_ODR_MASK (0b00001111)

/* -------------------------------------------------------------------------- */
/* 0x41 — ACC_RANGE : ±g range (RW, reset = 0x01)                             */
/* -------------------------------------------------------------------------- */
/* [1:0] acc_range — Full-scale range sélectionné.                           */ /* :contentReference[oaicite:14]{index=14} */
typedef enum {
    BMI_ACC_RANGE_3G   = 0b00000000, /**< ±3 g  */
    BMI_ACC_RANGE_6G   = 0b00000001, /**< ±6 g  */
    BMI_ACC_RANGE_12G  = 0b00000010, /**< ±12 g */
    BMI_ACC_RANGE_24G  = 0b00000011  /**< ±24 g */
} bmi_acc_range_t;
#define BMI_ACC_RANGE_MASK (0b00000011) /* bits [1:0] */                          

/* -------------------------------------------------------------------------- */
/* 0x45 — FIFO_DOWNS : Down-sampling (RW/RO mix, reset = 0x80)                */
/* -------------------------------------------------------------------------- */
/* [7] must_be_1 — “This bit must always be ‘1’.”                             */ /* :contentReference[oaicite:15]{index=15} */
typedef enum {
    BMI_ACC_FIFO_DOWNS_MUST_BE_1 = 0b10000000
} bmi_acc_fifo_downs_must1_t;
#define BMI_ACC_FIFO_DOWNS_MUST_BE_1_MASK (0b10000000)

/* [6:4] fifo_downs (RO) — facteur 2**k appliqué au flux FIFO.                */ /* :contentReference[oaicite:16]{index=16} */
typedef enum {
    BMI_ACC_FIFO_DOWNS_FACTOR_1   = 0b00000000, /**< no downsampling (2^0) */
    BMI_ACC_FIFO_DOWNS_FACTOR_2   = 0b00010000, /**< 2^1                   */
    BMI_ACC_FIFO_DOWNS_FACTOR_4   = 0b00100000, /**< 2^2                   */
    BMI_ACC_FIFO_DOWNS_FACTOR_8   = 0b00110000, /**< 2^3                   */
    BMI_ACC_FIFO_DOWNS_FACTOR_16  = 0b01000000, /**< 2^4                   */
    BMI_ACC_FIFO_DOWNS_FACTOR_32  = 0b01010000, /**< 2^5                   */
    BMI_ACC_FIFO_DOWNS_FACTOR_64  = 0b01100000, /**< 2^6                   */
    BMI_ACC_FIFO_DOWNS_FACTOR_128 = 0b01110000  /**< 2^7                   */
} bmi_acc_fifo_downs_factor_t;
#define BMI_ACC_FIFO_DOWNS_FACTOR_MASK (0b01110000)

/* -------------------------------------------------------------------------- */
/* 0x46–0x47 — FIFO_WTM_0/1 : 13-bit watermark (RW, reset = 0x0002)           */
/* -------------------------------------------------------------------------- */
/* FIFO watermark threshold en octets (13 bits).                              */ /* :contentReference[oaicite:17]{index=17} */

/* -------------------------------------------------------------------------- */
/* 0x48 — FIFO_CONFIG_0 : Mode (RW, reset = 0x02)                              */
/* -------------------------------------------------------------------------- */
/* [1] must_be_1 — “This bit must always be ‘1’.”                              */ /* :contentReference[oaicite:18]{index=18} */
typedef enum {
    BMI_ACC_FIFO_CONFIG_0_MUST_BE_1 = 0b00000010
} bmi_acc_fifo_config_0_const_t;
#define BMI_ACC_FIFO_CONFIG_0_MUST_BE_1_MASK (0b00000010)

/* [0] mode — STREAM(0) / FIFO(1).                                            */ /* :contentReference[oaicite:19]{index=19} */
typedef enum {
    BMI_ACC_FIFO_CONFIG_0_MODE_STREAM = 0b00000000,
    BMI_ACC_FIFO_CONFIG_0_MODE_FIFO   = 0b00000001
} bmi_acc_fifo_config_0_mode_t;
#define BMI_ACC_FIFO_CONFIG_0_MODE_MASK (0b00000001)

/* -------------------------------------------------------------------------- */
/* 0x49 — FIFO_CONFIG_1 : Source select (RW, reset = 0x10)                     */
/* -------------------------------------------------------------------------- */
/* [6] acc_en — inclure données accel dans FIFO.                              */ /* :contentReference[oaicite:20]{index=20} */
typedef enum {
    BMI_ACC_FIFO_CONFIG_1_ACC_EN_DISABLE = 0b00000000,
    BMI_ACC_FIFO_CONFIG_1_ACC_EN_ENABLE  = 0b01000000
} bmi_acc_fifo_config_1_acc_en_t;
#define BMI_ACC_FIFO_CONFIG_1_ACC_EN_MASK (0b01000000)

/* [4] must_be_1 — “This bit must always be ‘1’.”                              */ /* :contentReference[oaicite:21]{index=21} */
typedef enum {
    BMI_ACC_FIFO_CONFIG_1_MUST_BE_1 = 0b00010000
} bmi_acc_fifo_config_1_const_t;
#define BMI_ACC_FIFO_CONFIG_1_MUST_BE_1_MASK (0b00010000)

/* [3] int1_en — tag INT1 dans FIFO si INT1 en entrée.                         */ /* :contentReference[oaicite:22]{index=22} */
typedef enum {
    BMI_ACC_FIFO_CONFIG_1_INT1_EN_DISABLE = 0b00000000,
    BMI_ACC_FIFO_CONFIG_1_INT1_EN_ENABLE  = 0b00001000
} bmi_acc_fifo_config_1_int1_en_t;
#define BMI_ACC_FIFO_CONFIG_1_INT1_EN_MASK (0b00001000)

/* [2] int2_en — tag INT2 dans FIFO si INT2 en entrée.                         */ /* :contentReference[oaicite:23]{index=23} */
typedef enum {
    BMI_ACC_FIFO_CONFIG_1_INT2_EN_DISABLE = 0b00000000,
    BMI_ACC_FIFO_CONFIG_1_INT2_EN_ENABLE  = 0b00000100
} bmi_acc_fifo_config_1_int2_en_t;
#define BMI_ACC_FIFO_CONFIG_1_INT2_EN_MASK (0b00000100)

/* -------------------------------------------------------------------------- */
/* 0x53 — INT1_IO_CONF : INT1 I/O config (RW, reset = 0x00)                    */
/* -------------------------------------------------------------------------- */
/* [4] int1_in — activer INT1 en entrée.                                      */ /* :contentReference[oaicite:24]{index=24} */
typedef enum {
    BMI_ACC_INT1_IO_CONF_IN_DISABLE = 0b00000000,
    BMI_ACC_INT1_IO_CONF_IN_ENABLE  = 0b00010000
} bmi_acc_int1_io_conf_in_t;
#define BMI_ACC_INT1_IO_CONF_IN_MASK (0b00010000)

/* [3] int1_out — activer INT1 en sortie.                                     */ /* :contentReference[oaicite:25]{index=25} */
typedef enum {
    BMI_ACC_INT1_IO_CONF_OUT_DISABLE = 0b00000000,
    BMI_ACC_INT1_IO_CONF_OUT_ENABLE  = 0b00001000
} bmi_acc_int1_io_conf_out_t;
#define BMI_ACC_INT1_IO_CONF_OUT_MASK (0b00001000)

/* [2] int1_od — 0: push-pull ; 1: open-drain.                                */ /* :contentReference[oaicite:26]{index=26} */
typedef enum {
    BMI_ACC_INT1_IO_CONF_OD_PUSHPULL  = 0b00000000,
    BMI_ACC_INT1_IO_CONF_OD_OPENDRAIN = 0b00000100
} bmi_acc_int1_io_conf_od_t;
#define BMI_ACC_INT1_IO_CONF_OD_MASK (0b00000100)

/* [1] int1_lvl — 0: active low ; 1: active high.                              */ /* :contentReference[oaicite:27]{index=27} */
typedef enum {
    BMI_ACC_INT1_IO_CONF_LVL_ACTIVE_LOW  = 0b00000000,
    BMI_ACC_INT1_IO_CONF_LVL_ACTIVE_HIGH = 0b00000010
} bmi_acc_int1_io_conf_lvl_t;
#define BMI_ACC_INT1_IO_CONF_LVL_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x54 — INT2_IO_CONF : INT2 I/O config (RW, reset = 0x00)                    */
/* -------------------------------------------------------------------------- */
/* [4] int2_in — activer INT2 en entrée.                                      */ /* :contentReference[oaicite:28]{index=28} */
typedef enum {
    BMI_ACC_INT2_IO_CONF_IN_DISABLE = 0b00000000,
    BMI_ACC_INT2_IO_CONF_IN_ENABLE  = 0b00010000
} bmi_acc_int2_io_conf_in_t;
#define BMI_ACC_INT2_IO_CONF_IN_MASK (0b00010000)

/* [3] int2_out — activer INT2 en sortie.                                     */ /* :contentReference[oaicite:29]{index=29} */
typedef enum {
    BMI_ACC_INT2_IO_CONF_OUT_DISABLE = 0b00000000,
    BMI_ACC_INT2_IO_CONF_OUT_ENABLE  = 0b00001000
} bmi_acc_int2_io_conf_out_t;
#define BMI_ACC_INT2_IO_CONF_OUT_MASK (0b00001000)

/* [2] int2_od — 0: push-pull ; 1: open-drain.                                */ /* :contentReference[oaicite:30]{index=30} */
typedef enum {
    BMI_ACC_INT2_IO_CONF_OD_PUSHPULL  = 0b00000000,
    BMI_ACC_INT2_IO_CONF_OD_OPENDRAIN = 0b00000100
} bmi_acc_int2_io_conf_od_t;
#define BMI_ACC_INT2_IO_CONF_OD_MASK (0b00000100)

/* [1] int2_lvl — 0: active low ; 1: active high.                              */ /* :contentReference[oaicite:31]{index=31} */
typedef enum {
    BMI_ACC_INT2_IO_CONF_LVL_ACTIVE_LOW  = 0b00000000,
    BMI_ACC_INT2_IO_CONF_LVL_ACTIVE_HIGH = 0b00000010
} bmi_acc_int2_io_conf_lvl_t;
#define BMI_ACC_INT2_IO_CONF_LVL_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x58 — INT1_INT2_MAP_DATA : IRQ routing (RW, reset = 0x00)                  */
/* -------------------------------------------------------------------------- */
/* [6] int2_drdy ; [5] int2_fwm ; [4] int2_ffull ; [2] int1_drdy ;            */
/* [1] int1_fwm ; [0] int1_ffull.                                             */ /* :contentReference[oaicite:32]{index=32} */
typedef enum {
    BMI_ACC_INT1_INT2_MAP_DATA_INT1_FFULL = 0b00000001,
    BMI_ACC_INT1_INT2_MAP_DATA_INT1_FWM   = 0b00000010,
    BMI_ACC_INT1_INT2_MAP_DATA_INT1_DRDY  = 0b00000100,
    BMI_ACC_INT1_INT2_MAP_DATA_INT2_FFULL = 0b00010000,
    BMI_ACC_INT1_INT2_MAP_DATA_INT2_FWM   = 0b00100000,
    BMI_ACC_INT1_INT2_MAP_DATA_INT2_DRDY  = 0b01000000
} bmi_acc_int1_int2_map_data_t;
#define BMI_ACC_INT1_INT2_MAP_DATA_MASK (0b01110111)

/* -------------------------------------------------------------------------- */
/* 0x6D — ACC_SELF_TEST : Self-test (RW, reset = 0x00)                          */
/* -------------------------------------------------------------------------- */
/* Valeurs explicites : OFF=0x00 ; POS=0x0D ; NEG=0x09.                        */ /* :contentReference[oaicite:33]{index=33} */
typedef enum {
    BMI_ACC_SELF_TEST_OFF = 0b00000000,
    BMI_ACC_SELF_TEST_POS = 0b00001101,
    BMI_ACC_SELF_TEST_NEG = 0b00001001
} bmi_acc_self_test_t;
#define BMI_ACC_SELF_TEST_MASK (0b00001111)

/* -------------------------------------------------------------------------- */
/* 0x7C — ACC_PWR_CONF : Power save / mode (RW, reset = 0x03)                  */
/* -------------------------------------------------------------------------- */
/* acc_pwr_save : 0x03 = Suspend ; 0x00 = Active.                             */ /* :contentReference[oaicite:34]{index=34} */
typedef enum {
    BMI_ACC_PWR_CONF_ACTIVE  = 0b00000000,
    BMI_ACC_PWR_CONF_SUSPEND = 0b00000011
} bmi_acc_pwr_conf_t;
#define BMI_ACC_PWR_CONF_MASK (0b00000011)

/* -------------------------------------------------------------------------- */
/* 0x7D — ACC_PWR_CTRL : Sensor enable (RW, reset = 0x00)                       */
/* -------------------------------------------------------------------------- */
/* acc_enable : 0x00 = Off ; 0x04 = On.                                       */ /* :contentReference[oaicite:35]{index=35} */
typedef enum {
    BMI_ACC_PWR_CTRL_DISABLE = 0b00000000,
    BMI_ACC_PWR_CTRL_ENABLE  = 0b00000100
} bmi_acc_pwr_ctrl_t;
#define BMI_ACC_PWR_CTRL_MASK (0b00000100)

/* -------------------------------------------------------------------------- */
/* 0x7E — ACC_SOFTRESET : Software reset (WO, reset = N/A)                      */
/* -------------------------------------------------------------------------- */
/* Ecrire 0xB6 → reset (toutes configs reviennent aux valeurs reset).         */ /* :contentReference[oaicite:36]{index=36} */
typedef enum {
    BMI_ACC_SOFTRESET_CMD = 0b10110110
} bmi_acc_softreset_cmd_t;
#define BMI_ACC_SOFTRESET_MASK (0b11111111)

/** @} */ /* end of group BMI088_Enums_Accelerometer */

/* ========================================================================== */
/*                    BMI088 — Gyroscope Register Enumerations                */
/*                    Datasheet Rev. 1.9 (01/2024), 0x00–0x3F                 */
/* ========================================================================== */
/**
 * @defgroup BMI088_Enums_Gyroscope BMI088 Gyroscope Enumerations
 * @brief Enumerations grouped per gyroscope register field
 * @{
 */

/* -------------------------------------------------------------------------- */
/* 0x00 — GYR_CHIP_ID : Device ID (RO, reset = 0x0F)                          */
/* -------------------------------------------------------------------------- */
/* Valeur constante d’identification du gyroscope BMI088. */ /* :contentReference[oaicite:0]{index=0} */
#define BMI_GYR_CHIP_ID_VALUE  0x0FU

/* -------------------------------------------------------------------------- */
/* 0x09–0x0C — GYR_INT_STATUS_0..3 : Interrupt / status flags (RO, reset = 0) */
/* -------------------------------------------------------------------------- */
/* [7:0] bits de statut individuels selon configuration INT3/INT4. */ /* :contentReference[oaicite:1]{index=1} */

/* [0] drdy — Data ready flag. */ /* :contentReference[oaicite:2]{index=2} */
typedef enum {
    BMI_GYR_INT_STATUS_DRDY = 0b00000001 /**< New gyroscope data available */
} bmi_gyr_int_status_drdy_t;
#define BMI_GYR_INT_STATUS_DRDY_MASK (0b00000001)

/* -------------------------------------------------------------------------- */
/* 0x0F — GYR_RANGE : ±°/s range (RW, reset = 0x00)                            */
/* -------------------------------------------------------------------------- */
/* [2:0] gyr_range_sel — selection du plein échelle. */ /* :contentReference[oaicite:3]{index=3} */
typedef enum {
    BMI_GYR_RANGE_2000 = 0b00000000, /**< ±2000 °/s */
    BMI_GYR_RANGE_1000 = 0b00000001, /**< ±1000 °/s */
    BMI_GYR_RANGE_500  = 0b00000010, /**< ±500 °/s  */
    BMI_GYR_RANGE_250  = 0b00000011, /**< ±250 °/s  */
    BMI_GYR_RANGE_125  = 0b00000100  /**< ±125 °/s  */
} bmi_gyr_range_t;
#define BMI_GYR_RANGE_MASK (0b00000111)

/* -------------------------------------------------------------------------- */
/* 0x10 — GYR_BANDWIDTH : Bandwidth / ODR (RW, reset = 0x07)                  */
/* -------------------------------------------------------------------------- */
/* [2:0] gyr_bw_sel — fréquence de coupure / taux d’échantillonnage. */ /* :contentReference[oaicite:4]{index=4} */
typedef enum {
    BMI_GYR_BANDWIDTH_BW_532_HZ = 0b00000000, /**< 2000 Hz ODR / 532 Hz BW */
    BMI_GYR_BANDWIDTH_BW_230_HZ = 0b00000001, /**< 1000 Hz ODR / 230 Hz BW */
    BMI_GYR_BANDWIDTH_BW_116_HZ = 0b00000010, /**< 400 Hz ODR / 116 Hz BW  */
    BMI_GYR_BANDWIDTH_BW_47_HZ  = 0b00000011, /**< 200 Hz ODR / 47 Hz BW   */
    BMI_GYR_BANDWIDTH_BW_23_HZ  = 0b00000100, /**< 100 Hz ODR / 23 Hz BW   */
    BMI_GYR_BANDWIDTH_BW_12_HZ  = 0b00000101, /**< 50 Hz ODR / 12 Hz BW    */
    BMI_GYR_BANDWIDTH_BW_64_HZ  = 0b00000110, /**< 200 Hz ODR / 64 Hz BW   */
    BMI_GYR_BANDWIDTH_BW_32_HZ  = 0b00000111  /**< 100 Hz ODR / 32 Hz BW   */
} bmi_gyr_bandwidth_bw_t;
#define BMI_GYR_BANDWIDTH_BW_MASK (0b00000111)

/* -------------------------------------------------------------------------- */
/* 0x11 — GYR_LPM1 : Power mode control (RW, reset = 0x00)                     */
/* -------------------------------------------------------------------------- */
/* [7:5] reserved ; [2:0] power_mode. */ /* :contentReference[oaicite:5]{index=5} */
typedef enum {
    BMI_GYR_LPM1_MODE_NORMAL  = 0b00000000, /**< Normal operation */
    BMI_GYR_LPM1_MODE_DEEPSUSPEND = 0b00000111 /**< Deep suspend mode */
} bmi_gyr_lpm1_mode_t;
#define BMI_GYR_LPM1_MODE_MASK (0b00000111)

/* -------------------------------------------------------------------------- */
/* 0x14 — GYR_SOFTRESET : Software reset (WO, reset = N/A)                     */
/* -------------------------------------------------------------------------- */
/* Ecrire 0xB6 → soft reset du bloc gyro uniquement. */ /* :contentReference[oaicite:6]{index=6} */
typedef enum {
    BMI_GYR_SOFTRESET_CMD = 0b10110110
} bmi_gyr_softreset_cmd_t;
#define BMI_GYR_SOFTRESET_MASK (0b11111111)

/* -------------------------------------------------------------------------- */
/* 0x15 — GYR_INT_CTRL : Interrupt enable bits (RW, reset = 0x00)              */
/* -------------------------------------------------------------------------- */
/* [3] int3_en ; [2] int4_en. */ /* :contentReference[oaicite:7]{index=7} */
typedef enum {
    BMI_GYR_INT_CTRL_INT3_DISABLE = 0b00000000,
    BMI_GYR_INT_CTRL_INT3_ENABLE  = 0b00001000
} bmi_gyr_int_ctrl_int3_t;
#define BMI_GYR_INT_CTRL_INT3_MASK (0b00001000)

typedef enum {
    BMI_GYR_INT_CTRL_INT4_DISABLE = 0b00000000,
    BMI_GYR_INT_CTRL_INT4_ENABLE  = 0b00000100
} bmi_gyr_int_ctrl_int4_t;
#define BMI_GYR_INT_CTRL_INT4_MASK (0b00000100)

/* -------------------------------------------------------------------------- */
/* 0x16 — GYR_INT3_IO_CONF : INT3 pin config (RW, reset = 0x00)                */
/* -------------------------------------------------------------------------- */
/* [4] in_en ; [3] out_en ; [2] od ; [1] lvl. */ /* :contentReference[oaicite:8]{index=8} */
typedef enum {
    BMI_GYR_INT3_IO_CONF_IN_DISABLE = 0b00000000,
    BMI_GYR_INT3_IO_CONF_IN_ENABLE  = 0b00010000
} bmi_gyr_int3_io_conf_in_t;
#define BMI_GYR_INT3_IO_CONF_IN_MASK (0b00010000)

typedef enum {
    BMI_GYR_INT3_IO_CONF_OUT_DISABLE = 0b00000000,
    BMI_GYR_INT3_IO_CONF_OUT_ENABLE  = 0b00001000
} bmi_gyr_int3_io_conf_out_t;
#define BMI_GYR_INT3_IO_CONF_OUT_MASK (0b00001000)

typedef enum {
    BMI_GYR_INT3_IO_CONF_OD_PUSHPULL  = 0b00000000,
    BMI_GYR_INT3_IO_CONF_OD_OPENDRAIN = 0b00000100
} bmi_gyr_int3_io_conf_od_t;
#define BMI_GYR_INT3_IO_CONF_OD_MASK (0b00000100)

typedef enum {
    BMI_GYR_INT3_IO_CONF_LVL_ACTIVE_LOW  = 0b00000000,
    BMI_GYR_INT3_IO_CONF_LVL_ACTIVE_HIGH = 0b00000010
} bmi_gyr_int3_io_conf_lvl_t;
#define BMI_GYR_INT3_IO_CONF_LVL_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x17 — GYR_INT4_IO_CONF : INT4 pin config (RW, reset = 0x00)                */
/* -------------------------------------------------------------------------- */
typedef enum {
    BMI_GYR_INT4_IO_CONF_IN_DISABLE = 0b00000000,
    BMI_GYR_INT4_IO_CONF_IN_ENABLE  = 0b00010000
} bmi_gyr_int4_io_conf_in_t;
#define BMI_GYR_INT4_IO_CONF_IN_MASK (0b00010000)

typedef enum {
    BMI_GYR_INT4_IO_CONF_OUT_DISABLE = 0b00000000,
    BMI_GYR_INT4_IO_CONF_OUT_ENABLE  = 0b00001000
} bmi_gyr_int4_io_conf_out_t;
#define BMI_GYR_INT4_IO_CONF_OUT_MASK (0b00001000)

typedef enum {
    BMI_GYR_INT4_IO_CONF_OD_PUSHPULL  = 0b00000000,
    BMI_GYR_INT4_IO_CONF_OD_OPENDRAIN = 0b00000100
} bmi_gyr_int4_io_conf_od_t;
#define BMI_GYR_INT4_IO_CONF_OD_MASK (0b00000100)

typedef enum {
    BMI_GYR_INT4_IO_CONF_LVL_ACTIVE_LOW  = 0b00000000,
    BMI_GYR_INT4_IO_CONF_LVL_ACTIVE_HIGH = 0b00000010
} bmi_gyr_int4_io_conf_lvl_t;
#define BMI_GYR_INT4_IO_CONF_LVL_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x18 — GYR_INT3_INT4_IO_MAP : Interrupt mapping (RW, reset = 0x00)          */
/* -------------------------------------------------------------------------- */
/* [0] map_drdy_to_int3 ; [1] map_drdy_to_int4. */ /* :contentReference[oaicite:9]{index=9} */
typedef enum {
    BMI_GYR_INT3_INT4_IO_MAP_INT3_DRDY_DISABLE = 0b00000000,
    BMI_GYR_INT3_INT4_IO_MAP_INT3_DRDY_ENABLE  = 0b00000001
} bmi_gyr_int3_int4_io_map_int3_t;
#define BMI_GYR_INT3_INT4_IO_MAP_INT3_MASK (0b00000001)

typedef enum {
    BMI_GYR_INT3_INT4_IO_MAP_INT4_DRDY_DISABLE = 0b00000000,
    BMI_GYR_INT3_INT4_IO_MAP_INT4_DRDY_ENABLE  = 0b00000010
} bmi_gyr_int3_int4_io_map_int4_t;
#define BMI_GYR_INT3_INT4_IO_MAP_INT4_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x1A — GYR_AUTO_OFFSET : Auto offset control (RW, reset = 0x00)             */
/* -------------------------------------------------------------------------- */
/* [0] en — activer auto-offset ; [1] ready flag (RO). */ /* :contentReference[oaicite:10]{index=10} */
typedef enum {
    BMI_GYR_AUTO_OFFSET_ENABLE_DISABLE = 0b00000000,
    BMI_GYR_AUTO_OFFSET_ENABLE_ENABLE  = 0b00000001
} bmi_gyr_auto_offset_enable_t;
#define BMI_GYR_AUTO_OFFSET_ENABLE_MASK (0b00000001)

/* [1] ready flag (RO). */ /* :contentReference[oaicite:11]{index=11} */
typedef enum {
    BMI_GYR_AUTO_OFFSET_READY_NOTREADY = 0b00000000,
    BMI_GYR_AUTO_OFFSET_READY_DONE     = 0b00000010
} bmi_gyr_auto_offset_ready_t;
#define BMI_GYR_AUTO_OFFSET_READY_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x1B — GYR_NV_CONF : NVM configuration (RW, reset = 0x00)                   */
/* -------------------------------------------------------------------------- */
/* [0] nvm_prog_en — activer programmation des trims NVM. */ /* :contentReference[oaicite:12]{index=12} */
typedef enum {
    BMI_GYR_NV_CONF_PROG_DISABLE = 0b00000000,
    BMI_GYR_NV_CONF_PROG_ENABLE  = 0b00000001
} bmi_gyr_nv_conf_prog_t;
#define BMI_GYR_NV_CONF_PROG_MASK (0b00000001)

/* -------------------------------------------------------------------------- */
/* 0x3C — GYR_SELF_TEST : Built-in self-test (RW, reset = 0x00)                */
/* -------------------------------------------------------------------------- */
/* [0] enable ; [1] sign. */ /* :contentReference[oaicite:13]{index=13} */
typedef enum {
    BMI_GYR_SELF_TEST_DISABLE = 0b00000000,
    BMI_GYR_SELF_TEST_ENABLE  = 0b00000001
} bmi_gyr_self_test_enable_t;
#define BMI_GYR_SELF_TEST_ENABLE_MASK (0b00000001)

typedef enum {
    BMI_GYR_SELF_TEST_SIGN_NEGATIVE = 0b00000000,
    BMI_GYR_SELF_TEST_SIGN_POSITIVE = 0b00000010
} bmi_gyr_self_test_sign_t;
#define BMI_GYR_SELF_TEST_SIGN_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x3E — GYR_TRIM_NVM_CTRL : Trim / NVM control (RW, reset = 0x00)            */
/* -------------------------------------------------------------------------- */
/* [4] nvm_load_en ; [3] nvm_prog_trig. */ /* :contentReference[oaicite:14]{index=14} */
typedef enum {
    BMI_GYR_TRIM_NVM_CTRL_LOAD_DISABLE = 0b00000000,
    BMI_GYR_TRIM_NVM_CTRL_LOAD_ENABLE  = 0b00010000
} bmi_gyr_trim_nvm_ctrl_load_t;
#define BMI_GYR_TRIM_NVM_CTRL_LOAD_MASK (0b00010000)

typedef enum {
    BMI_GYR_TRIM_NVM_CTRL_PROG_TRIG_DISABLE = 0b00000000,
    BMI_GYR_TRIM_NVM_CTRL_PROG_TRIG_ENABLE  = 0b00001000
} bmi_gyr_trim_nvm_ctrl_prog_t;
#define BMI_GYR_TRIM_NVM_CTRL_PROG_MASK (0b00001000)

/** @} */ /* end of group BMI088_Enums_Gyroscope */

/**
 * @struct bmi_config_t
 * @brief Aggregated configuration for both accelerometer and gyroscope.
 * @details
 * This structure can be used for initialization, reconfiguration, and state
 * saving. It mirrors the BMI088 register configuration fields.
 */
typedef struct {
    /* Accelerometer */
    bmi_acc_range_t         acc_range;   /**< ±g full-scale range */
    bmi_acc_conf_bwp_t      acc_bwp;     /**< Bandwidth / oversampling */
    bmi_acc_conf_odr_t      acc_odr;     /**< Output Data Rate */
    bmi_acc_pwr_conf_t      acc_pwr;     /**< Power configuration */
    bmi_acc_pwr_ctrl_t      acc_ctrl;    /**< Power control enable/disable */

    /* Gyroscope */
    bmi_gyr_range_t         gyr_range;   /**< ±°/s full-scale range */
    bmi_gyr_bandwidth_bw_t  gyr_bw;      /**< Bandwidth / filter setting */
    bmi_gyr_lpm1_mode_t     gyr_mode;    /**< Power mode selection */

    // Additional configuration fields can be added here as needed
    // e.g., interrupt settings, FIFO configuration, etc.
} bmi_config_t;

/** @} */ /* end of group BMI088_Config */

typedef enum {
    BMI_OK = 0,
    BMI_SPI_ERR,
    BMI_INVALID_ARG,
    BMI_BUSY,
    BMI_TIMEOUT,
    BMI_UNKNOWN_ERR,
    BMI_SEM_ERR
} BMI_STATE;

typedef struct bmi088_t {

	/* SPI */
	SPI_HandleTypeDef	*spi;

	/* Accelerometer */
	GPIO_TypeDef		*cs_acc_bank;
	uint16_t			 cs_acc_pin;
	float3_t			 acc_offset;
    float                acc_conv;

	/* Gyroscope */
	GPIO_TypeDef		*cs_gyr_bank;
	uint16_t 		  	 cs_gyr_pin;
	float3_t			 gyr_offset;
    float                gyr_conv;

    bmi_config_t         config;

    StaticSemaphore_t    sem;
    osSemaphoreId_t      sem_id;
} bmi088_t;

/* -------------------------------------------------------------------------- */
/*                       Niveau 1 : Primitives capteur                        */
/* -------------------------------------------------------------------------- */

BMI_STATE BMI088_ReadRegister(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t *value);
BMI_STATE BMI088_WriteRegister(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t value);
BMI_STATE BMI088_ReadMultiple(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t *data, uint16_t len);
BMI_STATE BMI088_ReadID(bmi088_t *imu, uint8_t *acc_id, uint8_t *gyr_id);
BMI_STATE BMI088_SoftReset(bmi088_t *imu, bool is_gyr);

/* -------------------------------------------------------------------------- */
/*                      Niveau 2 : Logique périphérique                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise le capteur BMI088 (acc + gyr).
 */
BMI_STATE BMI088_Init(bmi088_t *imu, SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_acc_bank, uint16_t cs_acc_pin,
                      GPIO_TypeDef *cs_gyr_bank, uint16_t cs_gyr_pin,
                      const bmi_config_t *cfg);

/**
 * @brief Applique une configuration complète (acc + gyr).
 */
BMI_STATE BMI088_ApplyConfig(bmi088_t *imu, const bmi_config_t *cfg);

/**
 * @brief Lecture des données d'accélération (en m/s²).
 */
BMI_STATE BMI088_ReadAcc(bmi088_t *imu, float3_t *accel);

/**
 * @brief Lecture des données de gyroscope (en °/s).
 */
BMI_STATE BMI088_ReadGyr(bmi088_t *imu, float3_t *gyro);

/**
 * @brief Lecture de la température interne (en °C).
 */
BMI_STATE BMI088_ReadTemp(bmi088_t *imu, float *temp_c);



/* -------------------------------------------------------------------------- */
/*                       Niveau 1 : Primitives capteur RTOS                   */
/* -------------------------------------------------------------------------- */

BMI_STATE BMI088_ReadRegister_RTOS_base(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t *value, bool lock_sem);
BMI_STATE BMI088_WriteRegister_RTOS_base(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t value, bool lock_sem);
BMI_STATE BMI088_ReadMultiple_RTOS_base(bmi088_t *imu, bool is_gyr, uint8_t reg, uint8_t *data, uint16_t len, bool lock_sem);
BMI_STATE BMI088_ReadID_RTOS_base(bmi088_t *imu, uint8_t *acc_id, uint8_t *gyr_id, bool lock_sem);
BMI_STATE BMI088_SoftReset_RTOS_base(bmi088_t *imu, bool is_gyr, bool lock_sem);

#define BMI088_ReadRegister_RTOS_NoLock(imu, is_gyr, reg, value)        BMI088_ReadRegister_RTOS_base(imu, is_gyr, reg, value, false)
#define BMI088_WriteRegister_RTOS_NoLock(imu, is_gyr, reg, value)       BMI088_WriteRegister_RTOS_base(imu, is_gyr, reg, value, false)
#define BMI088_ReadMultiple_RTOS_NoLock(imu, is_gyr, reg, data, len)    BMI088_ReadMultiple_RTOS_base(imu, is_gyr, reg, data, len, false)
#define BMI088_ReadID_RTOS_NoLock(imu, acc_id, gyr_id)                  BMI088_ReadID_RTOS_base(imu, acc_id, gyr_id, false)
#define BMI088_SoftReset_RTOS_NoLock(imu, is_gyr)                       BMI088_SoftReset_RTOS_base(imu, is_gyr, false)

#define BMI088_ReadRegister_RTOS(imu, is_gyr, reg, value)               BMI088_ReadRegister_RTOS_base(imu, is_gyr, reg, value, true)
#define BMI088_WriteRegister_RTOS(imu, is_gyr, reg, value)              BMI088_WriteRegister_RTOS_base(imu, is_gyr, reg, value, true)
#define BMI088_ReadMultiple_RTOS(imu, is_gyr, reg, data, len)           BMI088_ReadMultiple_RTOS_base(imu, is_gyr, reg, data, len, true)
#define BMI088_ReadID_RTOS(imu, acc_id, gyr_id)                         BMI088_ReadID_RTOS_base(imu, acc_id, gyr_id, true)
#define BMI088_SoftReset_RTOS(imu, is_gyr)                              BMI088_SoftReset_RTOS(imu, is_gyr, true)



/* -------------------------------------------------------------------------- */
/*                      Niveau 2 : Logique périphérique RTOS                  */
/* -------------------------------------------------------------------------- */

typedef struct TASK_BMI088_Init_ARGS {
    bmi088_t                *imu;
    SPI_HandleTypeDef       *hspi;
    GPIO_TypeDef            *cs_acc_bank;
    uint16_t                 cs_acc_pin;
    GPIO_TypeDef            *cs_gyr_bank;
    uint16_t                 cs_gyr_pin;
    const bmi_config_t      *cfg;
    BMI_STATE               *return_state;
	osEventFlagsId_t         done_flags;
} TASK_BMI088_Init_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI088_Init, 1, 512);
void TASK_BMI088_Init(void *arguments);

BMI_STATE BMI088_ApplyConfig_RTOS(bmi088_t *imu, const bmi_config_t *cfg);

typedef struct TASK_BMI088_ReadAcc_ARGS {
    bmi088_t        *imu;
    data_topic_t   **dt;
    BMI_STATE       *return_state;
} TASK_BMI088_ReadAcc_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI088_ReadAcc, 1, 1024);
void TASK_BMI088_ReadAcc(void *arguments);

typedef struct TASK_BMI088_ReadGyr_ARGS {
    bmi088_t        *imu;
    data_topic_t   **dt;
    BMI_STATE       *return_state;
} TASK_BMI088_ReadGyr_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI088_ReadGyr, 1, 1024);
void TASK_BMI088_ReadGyr(void *arguments);

typedef struct TASK_BMI088_ReadTemp_ARGS {
    bmi088_t        *imu;
    data_topic_t   **dt;
    BMI_STATE       *return_state;
} TASK_BMI088_ReadTemp_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI088_ReadTemp, 1, 1024);
void TASK_BMI088_ReadTemp(void *arguments);


#ifdef __cplusplus
}
#endif


#endif /* BMI088_REGISTERS_H */