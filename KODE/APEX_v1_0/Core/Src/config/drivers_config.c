#include "config/drivers_config.h"

// =======================================================================
// BMI088 configuration
// =======================================================================
#if (APEX_ENABLE_BMI088 == 1)

#include "drivers/BMI088.h"

const bmi_config_t BMI088_CONFIG = {
    .acc_range  = BMI_ACC_RANGE_24G,
    .acc_bwp    = BMI_ACC_CONF_BWP_NORMAL,
    .acc_odr    = BMI_ACC_CONF_ODR_100_HZ,
    .acc_pwr    = BMI_ACC_PWR_CONF_ACTIVE,
    .acc_ctrl   = BMI_ACC_PWR_CTRL_ENABLE,

    .gyr_range  = BMI_GYR_RANGE_2000,
    .gyr_bw     = BMI_GYR_BANDWIDTH_BW_23_HZ,
    .gyr_mode   = BMI_GYR_LPM1_MODE_NORMAL,
};
bmi088_t BMI088;

// ... add any additional configuration or definitions for the BMI088 here

#endif