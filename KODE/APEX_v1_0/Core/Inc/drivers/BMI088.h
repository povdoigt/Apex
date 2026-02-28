/**
 * @file BMI088.h
 * @brief Driver for the Bosch BMI088 dual IMU (accelerometer + gyroscope).
 * @author Alexis Paillard
 * @version 1.0
 * @date 2025
 *
 * @note Targets Bosch BMI088 datasheet Rev. 1.9 (01/2024).
 *       SPI communication only (Mode 0 / Mode 3 supported by device).
 *       Two independent SPI chip-selects are required (ACC and GYR).
 *
 * @details
 * API layers:
 *  - **Level 0** – raw SPI primitives (blocking HAL / DMA+RTOS)
 *  - **Level 1** – single and burst register read/write helpers
 *  - **Level 2** – sensor logic: init, configuration, data read, self-test
 *  - **RTOS section** – semaphore-protected wrappers + FreeRTOS tasks
 */

/**
 * @defgroup BMI088_Driver BMI088 Driver
 * @brief Driver for the BMI088 IMU, providing functions for initialization, configuration, and data reading.
 * @details This module implements a driver for the Bosch BMI088 dual IMU, which includes a 3-axis accelerometer
 * and a 3-axis gyroscope. The driver provides functions for initializing the sensor, configuring its settings,
 * and reading acceleration and angular rate data. The driver is designed to be used in embedded systems with
 * an SPI interface and can be integrated into a FreeRTOS-based application.
 * @{
 */

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
 * Adress range: 0x00–0x7E
 * @{
 */

/* Identification and Status ------------------------------------------------ */
#define BMI_ACC_CHIP_ID                 0x00U  /**< Device identification (reset: 0x1E) */
// Reserved 0x01
#define BMI_ACC_ERR_REG                 0x02U  /**< Error flags register */
#define BMI_ACC_STATUS                  0x03U  /**< Data ready / status flags */

/* Reserved 0x04–0x11 ------------------------------------------------------- */

/* Acceleration data (16-bit signed, little-endian) ------------------------- */
#define BMI_ACC_X_LSB                   0x12U  /**< X-axis acceleration LSB (latched on LSB read) */
#define BMI_ACC_X_MSB                   0x13U  /**< X-axis acceleration MSB */
#define BMI_ACC_Y_LSB                   0x14U  /**< Y-axis acceleration LSB */
#define BMI_ACC_Y_MSB                   0x15U  /**< Y-axis acceleration MSB */
#define BMI_ACC_Z_LSB                   0x16U  /**< Z-axis acceleration LSB */
#define BMI_ACC_Z_MSB                   0x17U  /**< Z-axis acceleration MSB */

/* Sensor time (24-bit free-running counter) -------------------------------- */
#define BMI_SENSORTIME_0                0x18U  /**< Sensor time byte 0 (LSB), resolution 39.0625 µs */
#define BMI_SENSORTIME_1                0x19U  /**< Sensor time byte 1 */
#define BMI_SENSORTIME_2                0x1AU  /**< Sensor time byte 2 (MSB), overflow ~655 s */

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
#define BMI_ACC_CONF                    0x40U  /**< Bandwidth and ODR configuration (RW, reset=0xA8) */
#define BMI_ACC_RANGE                   0x41U  /**< Full-scale range selection (RW, reset=0x01) */
/* Reserved 0x42–0x44 -------------------- */
#define BMI_FIFO_DOWNS                  0x45U  /**< FIFO down-sampling factor (RW, reset=0x80) */
#define BMI_FIFO_WTM_0                  0x46U  /**< FIFO watermark threshold LSB (RW) */
#define BMI_FIFO_WTM_1                  0x47U  /**< FIFO watermark threshold MSB (RW) */
#define BMI_FIFO_CONFIG_0               0x48U  /**< FIFO mode: STREAM or FIFO (RW, reset=0x02) */
#define BMI_FIFO_CONFIG_1               0x49U  /**< FIFO data source selection (RW, reset=0x10) */

/* Reserved 0x4A–0x52 ------------------------------------------------------- */

/* Interrupt configuration --------------------------------------------------- */
#define BMI_INT1_IO_CONF                0x53U  /**< INT1 pin direction and electrical configuration (RW) */
#define BMI_INT2_IO_CONF                0x54U  /**< INT2 pin direction and electrical configuration (RW) */
/* Reserved 0x55–0x57 -------------------- */
#define BMI_INT1_INT2_MAP_DATA          0x58U  /**< Interrupt source mapping to INT1/INT2 pins (RW) */

/* Reserved 0x59–0x6C ------------------------------------------------------- */
#define BMI_ACC_RESERVED_6C             0x6CU  /**< Reserved (do not use) */

/* Self test and power management ------------------------------------------- */
#define BMI_ACC_SELF_TEST               0x6DU  /**< Self-test mode selection — OFF/POS/NEG (RW) */
/* Reserved 0x6E–0x7B -------------------- */
#define BMI_ACC_PWR_CONF                0x7CU  /**< Active / suspend power mode (RW, reset=0x03) */
#define BMI_ACC_PWR_CTRL                0x7DU  /**< Accelerometer sensor enable (RW, reset=0x00) */
#define BMI_ACC_SOFTRESET               0x7EU  /**< Software reset trigger — write 0xB6 (WO) */

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
// Reserved 0x08–0x09
#define BMI_GYR_INT_STATUS_1            0x0AU  /**< Interrupt status register 1 */
// Reserved 0x0B–0x0D
#define BMI_GYR_FIFO_STATUS             0x0EU  /**< FIFO status flags */

/* Configuration registers --------------------------------------------------- */
#define BMI_GYR_RANGE                   0x0FU  /**< +-dps full-scale range (bits[2:0]) */
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
 * @brief Enumerations grouped per accelerometer register field (0x00-0x7E)
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
    BMI_ACC_CONF_ODR_12_5_HZ  = 0b00000101,  /**< 12.5 Hz  */
    BMI_ACC_CONF_ODR_25_HZ    = 0b00000110,  /**< 25 Hz    */
    BMI_ACC_CONF_ODR_50_HZ    = 0b00000111,  /**< 50 Hz    */
    BMI_ACC_CONF_ODR_100_HZ   = 0b00001000,  /**< 100 Hz   */
    BMI_ACC_CONF_ODR_200_HZ   = 0b00001001,  /**< 200 Hz   */
    BMI_ACC_CONF_ODR_400_HZ   = 0b00001010,  /**< 400 Hz   */
    BMI_ACC_CONF_ODR_800_HZ   = 0b00001011,  /**< 800 Hz   */
    BMI_ACC_CONF_ODR_1600_HZ  = 0b00001100   /**< 1600 Hz  */
} bmi_acc_conf_odr_t;
#define BMI_ACC_CONF_ODR_MASK (0b00001111)

/* -------------------------------------------------------------------------- */
/* 0x41 — ACC_RANGE : ±g range (RW, reset = 0x01)                             */
/* -------------------------------------------------------------------------- */
/* [1:0] acc_range — Full-scale range sélectionné.                           */ /* :contentReference[oaicite:14]{index=14} */
typedef enum {
    BMI_ACC_RANGE_3G   = 0b00000000, /**< +-3 g  (highest resolution) */
    BMI_ACC_RANGE_6G   = 0b00000001, /**< +-6 g  (default)            */
    BMI_ACC_RANGE_12G  = 0b00000010, /**< +-12 g                       */
    BMI_ACC_RANGE_24G  = 0b00000011  /**< +-24 g (lowest resolution)  */
} bmi_acc_range_t;
#define BMI_ACC_RANGE_MASK (0b00000011) /* bits [1:0] */                          

/* -------------------------------------------------------------------------- */
/* 0x45 — FIFO_DOWNS : Down-sampling (RW/RO mix, reset = 0x80)                */
/* -------------------------------------------------------------------------- */
/* [7] must_be_1 — “This bit must always be ‘1’.”                             */ /* :contentReference[oaicite:15]{index=15} */
typedef enum {
    BMI_ACC_FIFO_DOWNS_MUST_BE_1 = 0b10000000  /**< Reserved — must always be 1 */
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
    BMI_ACC_FIFO_CONFIG_0_MUST_BE_1 = 0b00000010  /**< Reserved — must always be 1 */
} bmi_acc_fifo_config_0_const_t;
#define BMI_ACC_FIFO_CONFIG_0_MUST_BE_1_MASK (0b00000010)

/* [0] mode — STREAM(0) / FIFO(1).                                            */ /* :contentReference[oaicite:19]{index=19} */
typedef enum {
    BMI_ACC_FIFO_CONFIG_0_MODE_STREAM = 0b00000000,  /**< Stream mode: oldest data overwritten when full */
    BMI_ACC_FIFO_CONFIG_0_MODE_FIFO   = 0b00000001   /**< FIFO mode: new data discarded when full        */
} bmi_acc_fifo_config_0_mode_t;
#define BMI_ACC_FIFO_CONFIG_0_MODE_MASK (0b00000001)

/* -------------------------------------------------------------------------- */
/* 0x49 — FIFO_CONFIG_1 : Source select (RW, reset = 0x10)                     */
/* -------------------------------------------------------------------------- */
/* [6] acc_en — inclure données accel dans FIFO.                              */ /* :contentReference[oaicite:20]{index=20} */
typedef enum {
    BMI_ACC_FIFO_CONFIG_1_ACC_EN_DISABLE = 0b00000000,  /**< Accelerometer data excluded from FIFO */
    BMI_ACC_FIFO_CONFIG_1_ACC_EN_ENABLE  = 0b01000000   /**< Accelerometer data included in FIFO   */
} bmi_acc_fifo_config_1_acc_en_t;
#define BMI_ACC_FIFO_CONFIG_1_ACC_EN_MASK (0b01000000)

/* [4] must_be_1 — “This bit must always be ‘1’.”                              */ /* :contentReference[oaicite:21]{index=21} */
typedef enum {
    BMI_ACC_FIFO_CONFIG_1_MUST_BE_1 = 0b00010000  /**< Reserved — must always be 1 */
} bmi_acc_fifo_config_1_const_t;
#define BMI_ACC_FIFO_CONFIG_1_MUST_BE_1_MASK (0b00010000)

/* [3] int1_en — tag INT1 dans FIFO si INT1 en entrée.                         */ /* :contentReference[oaicite:22]{index=22} */
typedef enum {
    BMI_ACC_FIFO_CONFIG_1_INT1_EN_DISABLE = 0b00000000,  /**< INT1 tagging disabled */
    BMI_ACC_FIFO_CONFIG_1_INT1_EN_ENABLE  = 0b00001000   /**< INT1 tagging enabled  */
} bmi_acc_fifo_config_1_int1_en_t;
#define BMI_ACC_FIFO_CONFIG_1_INT1_EN_MASK (0b00001000)

/* [2] int2_en — tag INT2 dans FIFO si INT2 en entrée.                         */ /* :contentReference[oaicite:23]{index=23} */
typedef enum {
    BMI_ACC_FIFO_CONFIG_1_INT2_EN_DISABLE = 0b00000000,  /**< INT2 tagging disabled */
    BMI_ACC_FIFO_CONFIG_1_INT2_EN_ENABLE  = 0b00000100   /**< INT2 tagging enabled  */
} bmi_acc_fifo_config_1_int2_en_t;
#define BMI_ACC_FIFO_CONFIG_1_INT2_EN_MASK (0b00000100)

/* -------------------------------------------------------------------------- */
/* 0x53 — INT1_IO_CONF : INT1 I/O config (RW, reset = 0x00)                    */
/* -------------------------------------------------------------------------- */
/* [4] int1_in — activer INT1 en entrée.                                      */ /* :contentReference[oaicite:24]{index=24} */
typedef enum {
    BMI_ACC_INT1_IO_CONF_IN_DISABLE = 0b00000000,  /**< INT1 input disabled */
    BMI_ACC_INT1_IO_CONF_IN_ENABLE  = 0b00010000   /**< INT1 input enabled  */
} bmi_acc_int1_io_conf_in_t;
#define BMI_ACC_INT1_IO_CONF_IN_MASK (0b00010000)

/* [3] int1_out — activer INT1 en sortie.                                     */ /* :contentReference[oaicite:25]{index=25} */
typedef enum {
    BMI_ACC_INT1_IO_CONF_OUT_DISABLE = 0b00000000,  /**< INT1 output disabled */
    BMI_ACC_INT1_IO_CONF_OUT_ENABLE  = 0b00001000   /**< INT1 output enabled  */
} bmi_acc_int1_io_conf_out_t;
#define BMI_ACC_INT1_IO_CONF_OUT_MASK (0b00001000)

/* [2] int1_od — 0: push-pull ; 1: open-drain.                                */ /* :contentReference[oaicite:26]{index=26} */
typedef enum {
    BMI_ACC_INT1_IO_CONF_OD_PUSHPULL  = 0b00000000,  /**< Push-pull output  */
    BMI_ACC_INT1_IO_CONF_OD_OPENDRAIN = 0b00000100   /**< Open-drain output */
} bmi_acc_int1_io_conf_od_t;
#define BMI_ACC_INT1_IO_CONF_OD_MASK (0b00000100)

/* [1] int1_lvl — 0: active low ; 1: active high.                              */ /* :contentReference[oaicite:27]{index=27} */
typedef enum {
    BMI_ACC_INT1_IO_CONF_LVL_ACTIVE_LOW  = 0b00000000,  /**< Active low  */
    BMI_ACC_INT1_IO_CONF_LVL_ACTIVE_HIGH = 0b00000010   /**< Active high */
} bmi_acc_int1_io_conf_lvl_t;
#define BMI_ACC_INT1_IO_CONF_LVL_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x54 — INT2_IO_CONF : INT2 I/O config (RW, reset = 0x00)                    */
/* -------------------------------------------------------------------------- */
/* [4] int2_in — activer INT2 en entrée.                                      */ /* :contentReference[oaicite:28]{index=28} */
typedef enum {
    BMI_ACC_INT2_IO_CONF_IN_DISABLE = 0b00000000,  /**< INT2 input disabled */
    BMI_ACC_INT2_IO_CONF_IN_ENABLE  = 0b00010000   /**< INT2 input enabled  */
} bmi_acc_int2_io_conf_in_t;
#define BMI_ACC_INT2_IO_CONF_IN_MASK (0b00010000)

/* [3] int2_out — activer INT2 en sortie.                                     */ /* :contentReference[oaicite:29]{index=29} */
typedef enum {
    BMI_ACC_INT2_IO_CONF_OUT_DISABLE = 0b00000000,  /**< INT2 output disabled */
    BMI_ACC_INT2_IO_CONF_OUT_ENABLE  = 0b00001000   /**< INT2 output enabled  */
} bmi_acc_int2_io_conf_out_t;
#define BMI_ACC_INT2_IO_CONF_OUT_MASK (0b00001000)

/* [2] int2_od — 0: push-pull ; 1: open-drain.                                */ /* :contentReference[oaicite:30]{index=30} */
typedef enum {
    BMI_ACC_INT2_IO_CONF_OD_PUSHPULL  = 0b00000000,  /**< Push-pull output  */
    BMI_ACC_INT2_IO_CONF_OD_OPENDRAIN = 0b00000100   /**< Open-drain output */
} bmi_acc_int2_io_conf_od_t;
#define BMI_ACC_INT2_IO_CONF_OD_MASK (0b00000100)

/* [1] int2_lvl — 0: active low ; 1: active high.                              */ /* :contentReference[oaicite:31]{index=31} */
typedef enum {
    BMI_ACC_INT2_IO_CONF_LVL_ACTIVE_LOW  = 0b00000000,  /**< Active low  */
    BMI_ACC_INT2_IO_CONF_LVL_ACTIVE_HIGH = 0b00000010   /**< Active high */
} bmi_acc_int2_io_conf_lvl_t;
#define BMI_ACC_INT2_IO_CONF_LVL_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x58 — INT1_INT2_MAP_DATA : IRQ routing (RW, reset = 0x00)                  */
/* -------------------------------------------------------------------------- */
/* [6] int2_drdy ; [5] int2_fwm ; [4] int2_ffull ; [2] int1_drdy ;            */
/* [1] int1_fwm ; [0] int1_ffull.                                             */ /* :contentReference[oaicite:32]{index=32} */
typedef enum {
    BMI_ACC_INT1_INT2_MAP_DATA_INT1_FFULL = 0b00000001,  /**< FIFO full   → INT1 */
    BMI_ACC_INT1_INT2_MAP_DATA_INT1_FWM   = 0b00000010,  /**< FIFO wmark  → INT1 */
    BMI_ACC_INT1_INT2_MAP_DATA_INT1_DRDY  = 0b00000100,  /**< Data ready  → INT1 */
    BMI_ACC_INT1_INT2_MAP_DATA_INT2_FFULL = 0b00010000,  /**< FIFO full   → INT2 */
    BMI_ACC_INT1_INT2_MAP_DATA_INT2_FWM   = 0b00100000,  /**< FIFO wmark  → INT2 */
    BMI_ACC_INT1_INT2_MAP_DATA_INT2_DRDY  = 0b01000000   /**< Data ready  → INT2 */
} bmi_acc_int1_int2_map_data_t;
#define BMI_ACC_INT1_INT2_MAP_DATA_MASK (0b01110111)

/* -------------------------------------------------------------------------- */
/* 0x6D — ACC_SELF_TEST : Self-test (RW, reset = 0x00)                          */
/* -------------------------------------------------------------------------- */
/* Valeurs explicites : OFF=0x00 ; POS=0x0D ; NEG=0x09.                        */ /* :contentReference[oaicite:33]{index=33} */
typedef enum {
    BMI_ACC_SELF_TEST_OFF = 0b00000000,  /**< Self-test disabled       */
    BMI_ACC_SELF_TEST_POS = 0b00001101,  /**< Positive deflection test */
    BMI_ACC_SELF_TEST_NEG = 0b00001001   /**< Negative deflection test */
} bmi_acc_self_test_t;
#define BMI_ACC_SELF_TEST_MASK (0b00001111)

/* -------------------------------------------------------------------------- */
/* 0x7C — ACC_PWR_CONF : Power save / mode (RW, reset = 0x03)                  */
/* -------------------------------------------------------------------------- */
/* acc_pwr_save : 0x03 = Suspend ; 0x00 = Active.                             */ /* :contentReference[oaicite:34]{index=34} */
typedef enum {
    BMI_ACC_PWR_CONF_ACTIVE  = 0b00000000,  /**< Active mode  */
    BMI_ACC_PWR_CONF_SUSPEND = 0b00000011   /**< Suspend mode (default after reset) */
} bmi_acc_pwr_conf_t;
#define BMI_ACC_PWR_CONF_MASK (0b00000011)

/* -------------------------------------------------------------------------- */
/* 0x7D — ACC_PWR_CTRL : Sensor enable (RW, reset = 0x00)                       */
/* -------------------------------------------------------------------------- */
/* acc_enable : 0x00 = Off ; 0x04 = On.                                       */ /* :contentReference[oaicite:35]{index=35} */
typedef enum {
    BMI_ACC_PWR_CTRL_DISABLE = 0b00000000,  /**< Accelerometer off */
    BMI_ACC_PWR_CTRL_ENABLE  = 0b00000100   /**< Accelerometer on  */
} bmi_acc_pwr_ctrl_t;
#define BMI_ACC_PWR_CTRL_MASK (0b00000100)

/* -------------------------------------------------------------------------- */
/* 0x7E — ACC_SOFTRESET : Software reset (WO, reset = N/A)                      */
/* -------------------------------------------------------------------------- */
/* Ecrire 0xB6 → reset (toutes configs reviennent aux valeurs reset).         */ /* :contentReference[oaicite:36]{index=36} */
typedef enum {
    BMI_ACC_SOFTRESET_CMD = 0b10110110  /**< Write this value to trigger a soft reset */
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
    BMI_GYR_INT_STATUS_DRDY = 0b00000001  /**< New gyroscope data available */
} bmi_gyr_int_status_drdy_t;
#define BMI_GYR_INT_STATUS_DRDY_MASK (0b00000001)

/* -------------------------------------------------------------------------- */
/* 0x0F — GYR_RANGE : ±°/s range (RW, reset = 0x00)                            */
/* -------------------------------------------------------------------------- */
/* [2:0] gyr_range_sel — selection du plein échelle. */ /* :contentReference[oaicite:3]{index=3} */
typedef enum {
    BMI_GYR_RANGE_2000 = 0b00000000, /**< +-2000 °/s (lowest resolution)  */
    BMI_GYR_RANGE_1000 = 0b00000001, /**< +-1000 °/s                        */
    BMI_GYR_RANGE_500  = 0b00000010, /**< +-500 °/s                         */
    BMI_GYR_RANGE_250  = 0b00000011, /**< +-250 °/s                         */
    BMI_GYR_RANGE_125  = 0b00000100  /**< +-125 °/s (highest resolution)    */
} bmi_gyr_range_t;
#define BMI_GYR_RANGE_MASK (0b00000111)

/* -------------------------------------------------------------------------- */
/* 0x10 — GYR_BANDWIDTH : Bandwidth / ODR (RW, reset = 0x07)                  */
/* -------------------------------------------------------------------------- */
/* [2:0] gyr_bw_sel — fréquence de coupure / taux d’échantillonnage. */ /* :contentReference[oaicite:4]{index=4} */
typedef enum {
    BMI_GYR_BANDWIDTH_BW_532_HZ = 0b00000000, /**< 2000 Hz ODR / 532 Hz filter BW  */
    BMI_GYR_BANDWIDTH_BW_230_HZ = 0b00000001, /**< 1000 Hz ODR / 230 Hz filter BW  */
    BMI_GYR_BANDWIDTH_BW_116_HZ = 0b00000010, /**< 400 Hz ODR  / 116 Hz filter BW  */
    BMI_GYR_BANDWIDTH_BW_47_HZ  = 0b00000011, /**< 200 Hz ODR  / 47 Hz filter BW   */
    BMI_GYR_BANDWIDTH_BW_23_HZ  = 0b00000100, /**< 100 Hz ODR  / 23 Hz filter BW   */
    BMI_GYR_BANDWIDTH_BW_12_HZ  = 0b00000101, /**< 50 Hz ODR   / 12 Hz filter BW   */
    BMI_GYR_BANDWIDTH_BW_64_HZ  = 0b00000110, /**< 200 Hz ODR  / 64 Hz filter BW   */
    BMI_GYR_BANDWIDTH_BW_32_HZ  = 0b00000111  /**< 100 Hz ODR  / 32 Hz filter BW (default) */
} bmi_gyr_bandwidth_bw_t;
#define BMI_GYR_BANDWIDTH_BW_MASK (0b00000111)

/* -------------------------------------------------------------------------- */
/* 0x11 — GYR_LPM1 : Power mode control (RW, reset = 0x00)                     */
/* -------------------------------------------------------------------------- */
/* [7:5] reserved ; [2:0] power_mode. */ /* :contentReference[oaicite:5]{index=5} */
typedef enum {
    BMI_GYR_LPM1_MODE_NORMAL      = 0b00000000, /**< Normal (full operation) mode */
    BMI_GYR_LPM1_MODE_DEEPSUSPEND = 0b00000111  /**< Deep suspend (lowest power)  */
} bmi_gyr_lpm1_mode_t;
#define BMI_GYR_LPM1_MODE_MASK (0b00000111)

/* -------------------------------------------------------------------------- */
/* 0x14 — GYR_SOFTRESET : Software reset (WO, reset = N/A)                     */
/* -------------------------------------------------------------------------- */
/* Ecrire 0xB6 → soft reset du bloc gyro uniquement. */ /* :contentReference[oaicite:6]{index=6} */
typedef enum {
    BMI_GYR_SOFTRESET_CMD = 0b10110110  /**< Write this value to trigger a gyroscope soft reset */
} bmi_gyr_softreset_cmd_t;
#define BMI_GYR_SOFTRESET_MASK (0b11111111)

/* -------------------------------------------------------------------------- */
/* 0x15 — GYR_INT_CTRL : Interrupt enable bits (RW, reset = 0x00)              */
/* -------------------------------------------------------------------------- */
/* [3] int3_en ; [2] int4_en. */ /* :contentReference[oaicite:7]{index=7} */
typedef enum {
    BMI_GYR_INT_CTRL_INT3_DISABLE = 0b00000000,  /**< INT3 interrupt disabled */
    BMI_GYR_INT_CTRL_INT3_ENABLE  = 0b00001000   /**< INT3 interrupt enabled  */
} bmi_gyr_int_ctrl_int3_t;
#define BMI_GYR_INT_CTRL_INT3_MASK (0b00001000)

typedef enum {
    BMI_GYR_INT_CTRL_INT4_DISABLE = 0b00000000,  /**< INT4 interrupt disabled */
    BMI_GYR_INT_CTRL_INT4_ENABLE  = 0b00000100   /**< INT4 interrupt enabled  */
} bmi_gyr_int_ctrl_int4_t;
#define BMI_GYR_INT_CTRL_INT4_MASK (0b00000100)

/* -------------------------------------------------------------------------- */
/* 0x16 — GYR_INT3_IO_CONF : INT3 pin config (RW, reset = 0x00)                */
/* -------------------------------------------------------------------------- */
/* [4] in_en ; [3] out_en ; [2] od ; [1] lvl. */ /* :contentReference[oaicite:8]{index=8} */
typedef enum {
    BMI_GYR_INT3_IO_CONF_IN_DISABLE = 0b00000000,  /**< INT3 input disabled */
    BMI_GYR_INT3_IO_CONF_IN_ENABLE  = 0b00010000   /**< INT3 input enabled  */
} bmi_gyr_int3_io_conf_in_t;
#define BMI_GYR_INT3_IO_CONF_IN_MASK (0b00010000)

typedef enum {
    BMI_GYR_INT3_IO_CONF_OUT_DISABLE = 0b00000000,  /**< INT3 output disabled */
    BMI_GYR_INT3_IO_CONF_OUT_ENABLE  = 0b00001000   /**< INT3 output enabled  */
} bmi_gyr_int3_io_conf_out_t;
#define BMI_GYR_INT3_IO_CONF_OUT_MASK (0b00001000)

typedef enum {
    BMI_GYR_INT3_IO_CONF_OD_PUSHPULL  = 0b00000000,  /**< Push-pull output  */
    BMI_GYR_INT3_IO_CONF_OD_OPENDRAIN = 0b00000100   /**< Open-drain output */
} bmi_gyr_int3_io_conf_od_t;
#define BMI_GYR_INT3_IO_CONF_OD_MASK (0b00000100)

typedef enum {
    BMI_GYR_INT3_IO_CONF_LVL_ACTIVE_LOW  = 0b00000000,  /**< Active low  */
    BMI_GYR_INT3_IO_CONF_LVL_ACTIVE_HIGH = 0b00000010   /**< Active high */
} bmi_gyr_int3_io_conf_lvl_t;
#define BMI_GYR_INT3_IO_CONF_LVL_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x17 — GYR_INT4_IO_CONF : INT4 pin config (RW, reset = 0x00)                */
/* -------------------------------------------------------------------------- */
typedef enum {
    BMI_GYR_INT4_IO_CONF_IN_DISABLE = 0b00000000,  /**< INT4 input disabled */
    BMI_GYR_INT4_IO_CONF_IN_ENABLE  = 0b00010000   /**< INT4 input enabled  */
} bmi_gyr_int4_io_conf_in_t;
#define BMI_GYR_INT4_IO_CONF_IN_MASK (0b00010000)

typedef enum {
    BMI_GYR_INT4_IO_CONF_OUT_DISABLE = 0b00000000,  /**< INT4 output disabled */
    BMI_GYR_INT4_IO_CONF_OUT_ENABLE  = 0b00001000   /**< INT4 output enabled  */
} bmi_gyr_int4_io_conf_out_t;
#define BMI_GYR_INT4_IO_CONF_OUT_MASK (0b00001000)

typedef enum {
    BMI_GYR_INT4_IO_CONF_OD_PUSHPULL  = 0b00000000,  /**< Push-pull output  */
    BMI_GYR_INT4_IO_CONF_OD_OPENDRAIN = 0b00000100   /**< Open-drain output */
} bmi_gyr_int4_io_conf_od_t;
#define BMI_GYR_INT4_IO_CONF_OD_MASK (0b00000100)

typedef enum {
    BMI_GYR_INT4_IO_CONF_LVL_ACTIVE_LOW  = 0b00000000,  /**< Active low  */
    BMI_GYR_INT4_IO_CONF_LVL_ACTIVE_HIGH = 0b00000010   /**< Active high */
} bmi_gyr_int4_io_conf_lvl_t;
#define BMI_GYR_INT4_IO_CONF_LVL_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x18 — GYR_INT3_INT4_IO_MAP : Interrupt mapping (RW, reset = 0x00)          */
/* -------------------------------------------------------------------------- */
/* [0] map_drdy_to_int3 ; [1] map_drdy_to_int4. */ /* :contentReference[oaicite:9]{index=9} */
typedef enum {
    BMI_GYR_INT3_INT4_IO_MAP_INT3_DRDY_DISABLE = 0b00000000,  /**< Data-ready not routed to INT3 */
    BMI_GYR_INT3_INT4_IO_MAP_INT3_DRDY_ENABLE  = 0b00000001   /**< Data-ready routed to INT3     */
} bmi_gyr_int3_int4_io_map_int3_t;
#define BMI_GYR_INT3_INT4_IO_MAP_INT3_MASK (0b00000001)

typedef enum {
    BMI_GYR_INT3_INT4_IO_MAP_INT4_DRDY_DISABLE = 0b00000000,  /**< Data-ready not routed to INT4 */
    BMI_GYR_INT3_INT4_IO_MAP_INT4_DRDY_ENABLE  = 0b00000010   /**< Data-ready routed to INT4     */
} bmi_gyr_int3_int4_io_map_int4_t;
#define BMI_GYR_INT3_INT4_IO_MAP_INT4_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x1A — GYR_AUTO_OFFSET : Auto offset control (RW, reset = 0x00)             */
/* -------------------------------------------------------------------------- */
/* [0] en — activer auto-offset ; [1] ready flag (RO). */ /* :contentReference[oaicite:10]{index=10} */
typedef enum {
    BMI_GYR_AUTO_OFFSET_ENABLE_DISABLE = 0b00000000,  /**< Auto-offset disabled */
    BMI_GYR_AUTO_OFFSET_ENABLE_ENABLE  = 0b00000001   /**< Auto-offset enabled  */
} bmi_gyr_auto_offset_enable_t;
#define BMI_GYR_AUTO_OFFSET_ENABLE_MASK (0b00000001)

/* [1] ready flag (RO). */ /* :contentReference[oaicite:11]{index=11} */
typedef enum {
    BMI_GYR_AUTO_OFFSET_READY_NOTREADY = 0b00000000,  /**< Calibration not yet complete */
    BMI_GYR_AUTO_OFFSET_READY_DONE     = 0b00000010   /**< Calibration complete          */
} bmi_gyr_auto_offset_ready_t;
#define BMI_GYR_AUTO_OFFSET_READY_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x1B — GYR_NV_CONF : NVM configuration (RW, reset = 0x00)                   */
/* -------------------------------------------------------------------------- */
/* [0] nvm_prog_en — activer programmation des trims NVM. */ /* :contentReference[oaicite:12]{index=12} */
typedef enum {
    BMI_GYR_NV_CONF_PROG_DISABLE = 0b00000000,  /**< NVM programming disabled */
    BMI_GYR_NV_CONF_PROG_ENABLE  = 0b00000001   /**< NVM programming enabled  */
} bmi_gyr_nv_conf_prog_t;
#define BMI_GYR_NV_CONF_PROG_MASK (0b00000001)

/* -------------------------------------------------------------------------- */
/* 0x3C — GYR_SELF_TEST : Built-in self-test (RW, reset = 0x00)                */
/* -------------------------------------------------------------------------- */
/* [0] enable ; [1] sign. */ /* :contentReference[oaicite:13]{index=13} */
typedef enum {
    BMI_GYR_SELF_TEST_DISABLE = 0b00000000,  /**< Self-test disabled */
    BMI_GYR_SELF_TEST_ENABLE  = 0b00000001   /**< Self-test enabled  */
} bmi_gyr_self_test_enable_t;
#define BMI_GYR_SELF_TEST_ENABLE_MASK (0b00000001)

typedef enum {
    BMI_GYR_SELF_TEST_SIGN_NEGATIVE = 0b00000000,  /**< Negative deflection */
    BMI_GYR_SELF_TEST_SIGN_POSITIVE = 0b00000010   /**< Positive deflection */
} bmi_gyr_self_test_sign_t;
#define BMI_GYR_SELF_TEST_SIGN_MASK (0b00000010)

/* -------------------------------------------------------------------------- */
/* 0x3E — GYR_TRIM_NVM_CTRL : Trim / NVM control (RW, reset = 0x00)            */
/* -------------------------------------------------------------------------- */
/* [4] nvm_load_en ; [3] nvm_prog_trig. */ /* :contentReference[oaicite:14]{index=14} */
typedef enum {
    BMI_GYR_TRIM_NVM_CTRL_LOAD_DISABLE = 0b00000000,  /**< NVM load disabled */
    BMI_GYR_TRIM_NVM_CTRL_LOAD_ENABLE  = 0b00010000   /**< NVM load enabled  */
} bmi_gyr_trim_nvm_ctrl_load_t;
#define BMI_GYR_TRIM_NVM_CTRL_LOAD_MASK (0b00010000)

typedef enum {
    BMI_GYR_TRIM_NVM_CTRL_PROG_TRIG_DISABLE = 0b00000000,  /**< No NVM programming trigger */
    BMI_GYR_TRIM_NVM_CTRL_PROG_TRIG_ENABLE  = 0b00001000   /**< Trigger NVM programming    */
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
    bmi_acc_range_t         acc_range;   /**< +-g full-scale range */
    bmi_acc_conf_bwp_t      acc_bwp;     /**< Bandwidth / oversampling */
    bmi_acc_conf_odr_t      acc_odr;     /**< Output Data Rate */
    bmi_acc_pwr_conf_t      acc_pwr;     /**< Power configuration */
    bmi_acc_pwr_ctrl_t      acc_ctrl;    /**< Power control enable/disable */

    /* Gyroscope */
    bmi_gyr_range_t         gyr_range;   /**< +-°/s full-scale range */
    bmi_gyr_bandwidth_bw_t  gyr_bw;      /**< Bandwidth / filter setting */
    bmi_gyr_lpm1_mode_t     gyr_mode;    /**< Power mode selection */

    // Additional configuration fields can be added here as needed
    // e.g., interrupt settings, FIFO configuration, etc.
} bmi_config_t;

/** @} */ /* end of group BMI088_Config */

typedef enum {
    BMI_OK          = 0,  /**< Operation successful */
    BMI_SPI_ERR,          /**< SPI communication error (HAL level) */
    BMI_INVALID_ARG,      /**< NULL pointer or illegal argument */
    BMI_BUSY,             /**< Bus or peripheral is busy */
    BMI_TIMEOUT,          /**< Operation timed out */
    BMI_UNKNOWN_ERR,      /**< Unexpected error (e.g. wrong chip ID) */
    BMI_SEM_ERR           /**< FreeRTOS semaphore acquire/release error */
} BMI_STATE;

/**
 * @brief BMI088 device handle.
 *
 * @details Holds all runtime state for a single BMI088 instance:
 *  SPI handle, GPIO chip-selects, calibration offsets, physical-unit
 *  conversion factors, active configuration, and the FreeRTOS semaphore
 *  used by RTOS-aware variants.
 *
 *  Initialise with @ref BMI088_Init (bare-metal) or @ref TASK_BMI088_Init (RTOS).
 */
typedef struct bmi088_t {

    /* SPI */
    SPI_HandleTypeDef   *spi;           /**< HAL SPI handle shared by ACC and GYR */

    /* Accelerometer */
    GPIO_TypeDef        *cs_acc_bank;   /**< GPIO port of the accelerometer chip-select */
    uint16_t             cs_acc_pin;    /**< GPIO pin  of the accelerometer chip-select */
    float3_t             acc_offset;    /**< Static offset subtracted from raw ACC data */
    float                acc_conv;      /**< LSB → m/s² (or g) conversion factor, computed at init */

    /* Gyroscope */
    GPIO_TypeDef        *cs_gyr_bank;   /**< GPIO port of the gyroscope chip-select */
    uint16_t             cs_gyr_pin;    /**< GPIO pin  of the gyroscope chip-select */
    float3_t             gyr_offset;    /**< Static offset subtracted from raw GYR data */
    float                gyr_conv;      /**< LSB → °/s (or rad/s) conversion factor, computed at init */

    bmi_config_t         config;        /**< Active register configuration snapshot */

    StaticSemaphore_t    sem;           /**< Static semaphore control block (FreeRTOS) */
    osSemaphoreId_t      sem_id;        /**< CMSIS-RTOS semaphore handle (binary, init=1) */
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
#define BMI088_SoftReset_RTOS(imu, is_gyr)                              BMI088_SoftReset_RTOS_base(imu, is_gyr, true)



/* -------------------------------------------------------------------------- */
/*                      Niveau 2 : Logique peripherique RTOS                  */
/* -------------------------------------------------------------------------- */

typedef struct TASK_BMI088_Init_ARGS {
    bmi088_t                *imu;           /**< Handle to initialise */
    SPI_HandleTypeDef       *hspi;          /**< HAL SPI handle */
    GPIO_TypeDef            *cs_acc_bank;   /**< ACC chip-select GPIO port */
    uint16_t                 cs_acc_pin;    /**< ACC chip-select GPIO pin */
    GPIO_TypeDef            *cs_gyr_bank;   /**< GYR chip-select GPIO port */
    uint16_t                 cs_gyr_pin;    /**< GYR chip-select GPIO pin */
    const bmi_config_t      *cfg;           /**< Configuration to apply */
    BMI_STATE               *return_state;  /**< Written with the final status code */
    osEventFlagsId_t         done_flags;    /**< Optional event flag set on completion (bit 0) */
} TASK_BMI088_Init_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI088_Init, 1, 512);
void TASK_BMI088_Init(void *arguments);

BMI_STATE BMI088_ApplyConfig_RTOS(bmi088_t *imu, const bmi_config_t *cfg);

typedef struct TASK_BMI088_ReadAcc_ARGS {
    bmi088_t        *imu;           /**< Initialised BMI088 handle */
    data_topic_t   **dt;            /**< Set to the internal data topic on first iteration */
    BMI_STATE       *return_state;  /**< Updated each iteration with the latest status code */
} TASK_BMI088_ReadAcc_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI088_ReadAcc, 1, 1024);
void TASK_BMI088_ReadAcc(void *arguments);

typedef struct TASK_BMI088_ReadGyr_ARGS {
    bmi088_t        *imu;           /**< Initialised BMI088 handle */
    data_topic_t   **dt;            /**< Set to the internal data topic on first iteration */
    BMI_STATE       *return_state;  /**< Updated each iteration with the latest status code */
} TASK_BMI088_ReadGyr_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI088_ReadGyr, 1, 1024);
void TASK_BMI088_ReadGyr(void *arguments);

typedef struct TASK_BMI088_ReadTemp_ARGS {
    bmi088_t        *imu;           /**< Initialised BMI088 handle */
    data_topic_t   **dt;            /**< Set to the internal data topic on first iteration */
    BMI_STATE       *return_state;  /**< Updated each iteration with the latest status code */
} TASK_BMI088_ReadTemp_ARGS;
TASK_POOL_CONFIGURE(TASK_BMI088_ReadTemp, 1, 1024);
void TASK_BMI088_ReadTemp(void *arguments);


#ifdef __cplusplus
}
#endif


#endif /* BMI088_H */

/** @} */ /* end of group BMI088_Driver */