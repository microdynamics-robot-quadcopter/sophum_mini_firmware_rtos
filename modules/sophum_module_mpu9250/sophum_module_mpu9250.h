#ifndef __SOPHUM_MODULE_MPU9250_H__
#define __SOPHUM_MODULE_MPU9250_H__


/* according to the MPU-9250 Register Map and Descriptions v1.6 define below variable */
#define MPU9250_ADDR                                   0xD0 /* address AD0=0 0xD0 AD0=1 0xD2 */

/* MPU9250 register address */
#define MPU9250_SELF_TEST_X_GYRO                       0x00
#define MPU9250_SELF_TEST_Y_GYRO                       0x01
#define MPU9250_SELF_TEST_Z_GYRO                       0x02
#define MPU9250_SELF_TEST_X_ACCEL                      0x0D
#define MPU9250_SELF_TEST_Y_ACCEL                      0x0E
#define MPU9250_SELF_TEST_Z_ACCEL                      0x0F
#define MPU9250_XG_OFFSET_H                            0x13
#define MPU9250_XG_OFFSET_L                            0x14
#define MPU9250_YG_OFFSET_H                            0x15
#define MPU9250_YG_OFFSET_L                            0x16
#define MPU9250_ZG_OFFSET_H                            0x17
#define MPU9250_ZG_OFFSET_L                            0x18
#define MPU9250_SMPLRT_DIV                             0x19 /* gyro sample rate(typical value is 0x07) 1000/(1+7)=125HZ */
#define MPU9250_CONFIG                                 0x1A /* LPF typical value is 0x06(5HZ) */
#define MPU9250_GYRO_CONFIG                            0x1B /* gyro measurement range is 0x18(+/-2000 deg) */
#define MPU9250_ACCEL_CONFIG                           0x1C /* acce measurement range is 0x18(+/-16 g) */
#define MPU9250_ACCEL_CONFIG2                          0x1D /* acce LPF value is 0x06(5HZ) */
#define MPU9250_LP_ACCEL_ODR                           0x1E
#define MPU9250_WOM_THR                                0x1F
#define MPU9250_FIFO_EN                                0x23
#define MPU9250_I2C_MST_CTRL                           0x24
#define MPU9250_I2C_SLV0_ADDR                          0x25
#define MPU9250_I2C_SLV0_REG                           0x26
#define MPU9250_I2C_SLV0_CTRL                          0x27
#define MPU9250_I2C_SLV1_ADDR                          0x28
#define MPU9250_I2C_SLV1_REG                           0x29
#define MPU9250_I2C_SLV1_CTRL                          0x2A
#define MPU9250_I2C_SLV2_ADDR                          0x2B
#define MPU9250_I2C_SLV2_REG                           0x2C
#define MPU9250_I2C_SLV2_CTRL                          0x2D
#define MPU9250_I2C_SLV3_ADDR                          0x2E
#define MPU9250_I2C_SLV3_REG                           0x2F
#define MPU9250_I2C_SLV3_CTRL                          0x30
#define MPU9250_I2C_SLV4_ADDR                          0x31
#define MPU9250_I2C_SLV4_REG                           0x32
#define MPU9250_I2C_SLV4_DO                            0x33
#define MPU9250_I2C_SLV4_CTRL                          0x34
#define MPU9250_I2C_SLV4_DI                            0x35
#define MPU9250_I2C_MST_STATUS                         0x36
#define MPU9250_INT_PIN_CFG                            0x37
#define MPU9250_INT_ENABLE                             0x38
#define MPU9250_INT_STATUS                             0x3A
#define MPU9250_ACCEL_XOUT_H                           0x3B /* acce ouput data */
#define MPU9250_ACCEL_XOUT_L                           0x3C
#define MPU9250_ACCEL_YOUT_H                           0x3D
#define MPU9250_ACCEL_YOUT_L                           0x3E
#define MPU9250_ACCEL_ZOUT_H                           0x3F
#define MPU9250_ACCEL_ZOUT_L                           0x40
#define MPU9250_TEMP_OUT_H                             0x41 /* temp output data */
#define MPU9250_TEMP_OUT_L                             0x42
#define MPU9250_GYRO_XOUT_H                            0x43 /* gyro output data */
#define MPU9250_GYRO_XOUT_L                            0x44
#define MPU9250_GYRO_YOUT_H                            0x45
#define MPU9250_GYRO_YOUT_L                            0x46
#define MPU9250_GYRO_ZOUT_H                            0x47
#define MPU9250_GYRO_ZOUT_L                            0x48
#define MPU9250_EXT_SENS_DATA_00                       0x49
#define MPU9250_EXT_SENS_DATA_01                       0x4A
#define MPU9250_EXT_SENS_DATA_02                       0x4B
#define MPU9250_EXT_SENS_DATA_03                       0x4C
#define MPU9250_EXT_SENS_DATA_04                       0x4D
#define MPU9250_EXT_SENS_DATA_05                       0x4E
#define MPU9250_EXT_SENS_DATA_06                       0x4F
#define MPU9250_EXT_SENS_DATA_07                       0x50
#define MPU9250_EXT_SENS_DATA_08                       0x51
#define MPU9250_EXT_SENS_DATA_09                       0x52
#define MPU9250_EXT_SENS_DATA_10                       0x53
#define MPU9250_EXT_SENS_DATA_11                       0x54
#define MPU9250_EXT_SENS_DATA_12                       0x55
#define MPU9250_EXT_SENS_DATA_13                       0x56
#define MPU9250_EXT_SENS_DATA_14                       0x57
#define MPU9250_EXT_SENS_DATA_15                       0x58
#define MPU9250_EXT_SENS_DATA_16                       0x59
#define MPU9250_EXT_SENS_DATA_17                       0x5A
#define MPU9250_EXT_SENS_DATA_18                       0x5B
#define MPU9250_EXT_SENS_DATA_19                       0x5C
#define MPU9250_EXT_SENS_DATA_20                       0x5D
#define MPU9250_EXT_SENS_DATA_21                       0x5E
#define MPU9250_EXT_SENS_DATA_22                       0x5F
#define MPU9250_EXT_SENS_DATA_23                       0x60
#define MPU9250_I2C_SLV0_DO                            0x63
#define MPU9250_I2C_SLV1_DO                            0x64
#define MPU9250_I2C_SLV2_DO                            0x65
#define MPU9250_I2C_SLV3_DO                            0x66
#define MPU9250_I2C_MST_DELAY_CTRL                     0x67
#define MPU9250_SIGNAL_PATH_RESET                      0x68
#define MPU9250_MOT_DETECT_CTRL                        0x69
#define MPU9250_USER_CTRL                              0X6A /* when value is 0x10(SPI mode) */
#define MPU9250_PWR_MGMT_1                             0X6B /* typical value is 0x00 */
#define MPU9250_PWR_MGMT_2                             0X6C /* typical vlaue is 0x00 */
#define MPU9250_FIFO_COUNTH                            0x72
#define MPU9250_FIFO_COUNTL                            0x73
#define MPU9250_FIFO_R_W                               0x74
#define MPU9250_WHO_AM_I                               0X75 /* device ID(default value is 0x71) */
#define MPU9250_XA_OFFSET_H                            0x77
#define MPU9250_XA_OFFSET_L                            0x78
#define MPU9250_YA_OFFSET_H                            0x7A
#define MPU9250_YA_OFFSET_L                            0x7B
#define MPU9250_ZA_OFFSET_H                            0x7D
#define MPU9250_ZA_OFFSET_L                            0x7E

/* MPU9250 register bit */

/* CONFIG register(0x1A) bit */
#define MPU9250_CONFIG_FIFO_MODE_BIT                   6
#define MPU9250_CONFIG_EXT_SYNC_SET_BIT                5
#define MPU9250_CONFIG_EXT_SYNC_SET_LEN                3
#define MPU9250_CONFIG_DLPF_CFG_BIT                    2
#define MPU9250_CONFIG_DLPF_CFG_LEN                    3

/* CONFIG register(0x1A) bit[6] option value */
#define MPU9250_CONFIG_FIFO_MODE_OVERWRITE_ENABLE      0x0
#define MPU9250_CONFIG_FIFO_MODE_OVERWRITE_DISABLE     0x1

/* CONFIG register(0x1A) bit[5:3] option value */
#define MPU9250_CONFIG_EXT_SYNC_SET_DISABLE            0x0
#define MPU9250_CONFIG_EXT_SYNC_SET_TEMP_OUT_L         0x1
#define MPU9250_CONFIG_EXT_SYNC_SET_GYRO_XOUT_L        0x2
#define MPU9250_CONFIG_EXT_SYNC_SET_GYRO_YOUT_L        0x3
#define MPU9250_CONFIG_EXT_SYNC_SET_GYRO_ZOUT_L        0x4
#define MPU9250_CONFIG_EXT_SYNC_SET_ACCEL_XOUT_L       0x5
#define MPU9250_CONFIG_EXT_SYNC_SET_ACCEL_YOUT_L       0x6
#define MPU9250_CONFIG_EXT_SYNC_SET_ACCEL_ZOUT_L       0x7

/* CONFIG register(0x1A) bit[2:0] option value */
#define MPU9250_CONFIG_DLPF_CFG_256HZ                  0x0
#define MPU9250_CONFIG_DLPF_CFG_188HZ                  0x1
#define MPU9250_CONFIG_DLPF_CFG_98HZ                   0x2
#define MPU9250_CONFIG_DLPF_CFG_42HZ                   0x3
#define MPU9250_CONFIG_DLPF_CFG_20HZ                   0x4
#define MPU9250_CONFIG_DLPF_CFG_10HZ                   0x5
#define MPU9250_CONFIG_DLPF_CFG_5HZ                    0x6

/* GYRO_CONFIG register(0x1B) bit */
#define MPU9250_GYRO_CONFIG_XGYRO_CTEN_BIT             7
#define MPU9250_GYRO_CONFIG_YGYRO_CTEN_BIT             6
#define MPU9250_GYRO_CONFIG_ZGYRO_CTEN_BIT             5
#define MPU9250_GYRO_CONFIG_GYRO_FS_SEL_BIT            4
#define MPU9250_GYRO_CONFIG_GYRO_FS_SEL_LEN            2
#define MPU9250_GYRO_CONFIG_FCHOICE_B_BIT              1
#define MPU9250_GYRO_CONFIG_FCHOICE_B_LEN              2

/* GYRO_CONFIG register(0x1B) bit[4:3] option value */
#define MPU9250_GYRO_CONFIG_GYRO_FS_SEL_250DPS         0x0
#define MPU9250_GYRO_CONFIG_GYRO_FS_SEL_500DPS         0x1
#define MPU9250_GYRO_CONFIG_GYRO_FS_SEL_1000DPS        0x2
#define MPU9250_GYRO_CONFIG_GYRO_FS_SEL_2000DPS        0x3

/* ACCEL_CONFIG register(0x1C) bit */
#define MPU9250_ACCEL_CONFIG_AX_ST_EN_BIT              7
#define MPU9250_ACCEL_CONFIG_AY_ST_EN_BIT              6
#define MPU9250_ACCEL_CONFIG_AZ_ST_EN_BIT              5
#define MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_BIT          4
#define MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_LEN          2

/* ACCEL_CONFIG register(0x1C) bit[4:3] option value */
#define MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_2G           0x0
#define MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_4G           0x1
#define MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_8G           0x2
#define MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_16G          0x3

/* ACCEL_CONFIG2 register(0x1D) bit */
#define MPU9250_ACCEL_CONFIG2_ACCEL_FCHOICE_B_BIT      3
#define MPU9250_ACCEL_CONFIG2_A_DLPFCFG_BIT            2
#define MPU9250_ACCEL_CONFIG2_A_DLPFCFG_LEN            3

/* ACCEL_CONFIG2 register(0x1D) bit[2:0] option value */
#define MPU9250_ACCEL_CONFIG2_A_DLPFCFG_460HZ          0x0
#define MPU9250_ACCEL_CONFIG2_A_DLPFCFG_184HZ          0x1
#define MPU9250_ACCEL_CONFIG2_A_DLPFCFG_92HZ           0x2
#define MPU9250_ACCEL_CONFIG2_A_DLPFCFG_41HZ           0x3
#define MPU9250_ACCEL_CONFIG2_A_DLPFCFG_20HZ           0x4
#define MPU9250_ACCEL_CONFIG2_A_DLPFCFG_10HZ           0x5
#define MPU9250_ACCEL_CONFIG2_A_DLPFCFG_5HZ            0x6

/* LP_ACCEL_ODR register(0x1E) bit */
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_BIT           3
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_LEN           4

/* LP_ACCEL_ODR register(0x1E) bit[3:0] option value */
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_0P24HZ        0x0
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_0P49HZ        0x1
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_0P98HZ        0x2
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_1P95HZ        0x3
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_3P91HZ        0x4
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_7P81HZ        0x5
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_15P63HZ       0x6
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_31P25HZ       0x7
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_62P50HZ       0x8
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_125HZ         0x9
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_250HZ         0xA
#define MPU9250_LP_ACCE_ODR_LPOSC_CLKSEL_500HZ         0xB

/* FIFO_EN register(0x23) bit */
#define MPU9250_FIFO_EN_TEMP_OUT_BIT                   7
#define MPU9250_FIFO_EN_GYRO_XOUT_BIT                  6
#define MPU9250_FIFO_EN_GYRO_YOUT_BIT                  5
#define MPU9250_FIFO_EN_GYRO_ZOUT_BIT                  4
#define MPU9250_FIFO_EN_ACCEL_BIT                      3
#define MPU9250_FIFO_EN_SLV_2_BIT                      2
#define MPU9250_FIFO_EN_SLV_1_BIT                      1
#define MPU9250_FIFO_EN_SLV_0_BIT                      0

/* I2C_MST_CTRL register(0x24) bit */
#define MPU9250_I2C_MST_CTRL_MULT_MST_EN_BIT           7
#define MPU9250_I2C_MST_CTRL_WAIT_FOR_ES_BIT           6
#define MPU9250_I2C_MST_CTRL_SLV_3_FIFO_EN_BIT         5
#define MPU9250_I2C_MST_CTRL_I2C_MST_P_NSR_BIT         4
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_BIT           3
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_LEN           4

/* I2C_MST_CTRL register(0x24) bit[3:0] option value */
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_348KHZ        0x0
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_333KHZ        0x1
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_320KHZ        0x2
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_308KHZ        0x3
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_296KHZ        0x4
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_286KHZ        0x5
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_276KHZ        0x6
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_267KHZ        0x7
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_258KHZ        0x8
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_500KHZ        0x9
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_471KHZ        0xA
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_444KHZ        0xB
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_421KHZ        0xC
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_400KHZ        0xD
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_381KHZ        0xE
#define MPU9250_I2C_MST_CTRL_I2C_MST_CLK_364KHZ        0xF

/* I2C_SLVx(1,2,3)_ADDR register bit */
#define MPU9250_I2C_SLV_ADDR_I2C_SLV_RNW_BIT           7
#define MPU9250_I2C_SLV_ADDR_I2C_ID_BIT                6
#define MPU9250_I2C_SLV_ADDR_I2C_ID_LEN                7

/* I2C_SLVx(1,2,3)_CTRL register bit */
#define MPU9250_I2C_SLV_CTRL_I2C_SLV_EN_BIT            7
#define MPU9250_I2C_SLV_CTRL_I2C_SLV_BYTE_SW_BIT       6
#define MPU9250_I2C_SLV_CTRL_I2C_SLV_REG_DIS_BIT       5
#define MPU9250_I2C_SLV_CTRL_I2C_SLV_GRP_BIT           4
#define MPU9250_I2C_SLV_CTRL_I2C_SLV_LENG_BIT          3
#define MPU9250_I2C_SLV_CTRL_I2C_SLV_LENG_LEN          4

/* I2C_SLV4_ADDR register(0x31) bit */
#define MPU9250_I2C_SLV4_ADDR_I2C_SLV4_RNW_BIT         7
#define MPU9250_I2C_SLV4_ADDR_I2C_ID_BIT               6
#define MPU9250_I2C_SLV4_ADDR_I2C_ID_LEN               7

/* I2C_SLV4_CTRL register(0x34) bit */
#define MPU9250_I2C_SLV4_CTRL_I2C_SLV4_EN_BIT          7
#define MPU9250_I2C_SLV4_CTRL_I2C_SLV4_DONE_INT_EN_BIT 6
#define MPU9250_I2C_SLV4_CTRL_I2C_SLV4_REG_DIS_BIT     5
#define MPU9250_I2C_SLV4_CTRL_I2C_SLV4_MST_DLY_BIT     4
#define MPU9250_I2C_SLV4_CTRL_I2C_SLV4_MST_DLY_LEN     5


/* I2C_MST_STATUS register(0x36) bit */
#define MPU9250_I2C_MST_STATUS_PASS_THROUGH_BIT        7
#define MPU9250_I2C_MST_STATUS_I2C_SLV4_DONE_BIT       6
#define MPU9250_I2C_MST_STATUS_I2C_LOST_ARB_BIT        5
#define MPU9250_I2C_MST_STATUS_I2C_SLV4_NACK_BIT       4
#define MPU9250_I2C_MST_STATUS_I2C_SLV3_NACK_BIT       3
#define MPU9250_I2C_MST_STATUS_I2C_SLV2_NACK_BIT       2
#define MPU9250_I2C_MST_STATUS_I2C_SLV1_NACK_BIT       1
#define MPU9250_I2C_MST_STATUS_I2C_SLV0_NACK_BIT       0

/* INT_PIN_CFG register(0x37) bit */
#define MPU9250_INT_PIN_CFG_ACTL_BIT                   7
#define MPU9250_INT_PIN_CFG_OPEN_BIT                   6
#define MPU9250_INT_PIN_CFG_LATCH_INT_EN_BIT           5
#define MPU9250_INT_PIN_CFG_INT_ANYRD_2CLEAR_BIT       4
#define MPU9250_INT_PIN_CFG_ACTL_FSYNC_BIT             3
#define MPU9250_INT_PIN_CFG_FSYNC_INT_MODE_EN_BIT      2
#define MPU9250_INT_PIN_CFG_BYPASS_EN_BIT              1

/* INT_PIN_CFG register(0x37) bit[7:4] option value */
#define MPU9250_INT_PIN_CFG_ACTL_ACTIVEHIGH            0x0
#define MPU9250_INT_PIN_CFG_ACTL_ACTIVELOW             0x1

#define MPU9250_INT_PIN_CFG_OPEN_PUSHPULL              0x0
#define MPU9250_INT_PIN_CFG_OPEN_OPENDRAIN             0x1

#define MPU9250_INT_PIN_CFG_LATCH_50USPULSE            0x0
#define MPU9250_INT_PIN_CFG_LATCH_WAITCLEAR            0x1

#define MPU9250_INT_PIN_CFG_2CLEAR_STATUSREAD          0x0
#define MPU9250_INT_PIN_CFG_2CLEAR_ANYREAD             0x1


/* INT_ENABLE register(0x38) bit */
#define MPU9250_INT_ENABLE_WOM_EN_BIT                  6
#define MPU9250_INT_ENABLE_FIFO_OVERFLOW_EN_BIT        4
#define MPU9250_INT_ENABLE_FSYNC_INT_EN_BIT            3
#define MPU9250_INT_ENABLE_RAW_RDY_EN_BIT              0

/* INT_STATUS register(0x3A) bit */
#define MPU9250_INT_STATUS_WOM_INT_BIT                 6
#define MPU9250_INT_STATUS_FIFO_OVERFLOW_INT_BIT       4
#define MPU9250_INT_STATUS_FSYNC_INT_BIT               3
#define MPU9250_INT_STATUS_RAW_RDY_INT_BIT             0

/* I2C_MST_DELAY_CTRL register(0x67) bit */
#define MPU9250_I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW_BIT 7
#define MPU9250_I2C_MST_DELAY_CTRL_I2C_SLV4_DLY_EN_BIT 4
#define MPU9250_I2C_MST_DELAY_CTRL_I2C_SLV3_DLY_EN_BIT 3
#define MPU9250_I2C_MST_DELAY_CTRL_I2C_SLV2_DLY_EN_BIT 2
#define MPU9250_I2C_MST_DELAY_CTRL_I2C_SLV1_DLY_EN_BIT 1
#define MPU9250_I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN_BIT 0

/* SIGNAL_PATH_RESET register(0x68) bit */
#define MPU9250_SIGNAL_PATH_RESET_GYRO_RST_BIT         2
#define MPU9250_SIGNAL_PATH_RESET_ACCE_RST_BIT         1
#define MPU9250_SIGNAL_PATH_RESET_TEMP_RST_BIT         0

/* MOT_DETECT_CTRL register(0x69) bit */
#define MPU9250_MOT_DETECT_CTRL_ACCEL_INTEL_EN_BIT     7
#define MPU9250_MOT_DETECT_CTRL_ACCEL_INTEL_MODE_BIT   6

/* USER_CTRL register(0x6A) bit */
#define MPU9250_USER_CTRL_FIFO_EN_BIT                  6
#define MPU9250_USER_CTRL_I2C_MST_EN_BIT               5
#define MPU9250_USER_CTRL_I2C_IF_DIS_BIT               4
#define MPU9250_USER_CTRL_FIFO_RST_BIT                 2
#define MPU9250_USER_CTRL_I2C_MST_RST_BIT              1
#define MPU9250_USER_CTRL_SIG_COND_RST_BIT             0

/* PWR_MGMT_1 register(0x6B) bit */
#define MPU9250_PWR_MGMT_1_H_RESET_BIT                 7
#define MPU9250_PWR_MGMT_1_SLEEP_BIT                   6
#define MPU9250_PWR_MGMT_1_CYCLE_BIT                   5
#define MPU9250_PWR_MGMT_1_GYRO_STANDBY_BIT            4
#define MPU9250_PWR_MGMT_1_PD_PTAT_BIT                 3
#define MPU9250_PWR_MGMT_1_CLKSEL_BIT                  2
#define MPU9250_PWR_MGMT_1_CLKSEL_LEN                  3

/* PWR_MGMT_1 register(0x6B) bit[2:0] option value */
#define MPU9250_PWR_MGMT_1_CLKSEL_INTERNAL20M1         0x0
#define MPU9250_PWR_MGMT_1_CLKSEL_PLL_AUTO1            0x1
#define MPU9250_PWR_MGMT_1_CLKSEL_PLL_AUTO2            0x2
#define MPU9250_PWR_MGMT_1_CLKSEL_PLL_AUTO3            0x3
#define MPU9250_PWR_MGMT_1_CLKSEL_PLL_AUTO4            0x4
#define MPU9250_PWR_MGMT_1_CLKSEL_PLL_AUTO5            0x5
#define MPU9250_PWR_MGMT_1_CLKSEL_INTERNAL20M2         0x6
#define MPU9250_PWR_MGMT_1_CLKSEL_KEEP_RESET           0x7

/* PWR_MGMT_2 register(0x6C) bit */
#define MPU9250_PWR_MGMT_2_DISABLE_XA_BIT              5
#define MPU9250_PWR_MGMT_2_DISABLE_YA_BIT              4
#define MPU9250_PWR_MGMT_2_DISABLE_ZA_BIT              3
#define MPU9250_PWR_MGMT_2_DISABLE_XG_BIT              2
#define MPU9250_PWR_MGMT_2_DISABLE_YG_BIT              1
#define MPU9250_PWR_MGMT_2_DISABLE_ZG_BIT              0

/* FIFO_COUNTH register(0x72) bit */
#define MPU9250_FIFO_COUNTH_FIFO_CNT_BIT               4
#define MPU9250_FIFO_COUNTH_FIFO_CNT_LEN               5


#define MPU9250_DEG_PER_LSB_250DPS   (float)((2 * 250.0F) / 65536.0F)
#define MPU9250_DEG_PER_LSB_500DPS   (float)((2 * 500.0F) / 65536.0F)
#define MPU9250_DEG_PER_LSB_1000DPS  (float)((2 * 1000.0F) / 65536.0F)
#define MPU9250_DEG_PER_LSB_2000DPS  (float)((2 * 2000.0F) / 65536.0F)

#define MPU9250_G_PER_LSB_2G         (float)((2 * 2) / 65536.0F)
#define MPU9250_G_PER_LSB_4G         (float)((2 * 4) / 65536.0F)
#define MPU9250_G_PER_LSB_8G         (float)((2 * 8) / 65536.0F)
#define MPU9250_G_PER_LSB_16G        (float)((2 * 16) / 65536.0F)

/* self test limits value(percent) */
#define MPU9250_SELF_TEST_GYRO_LOW   (-14.0F)
#define MPU9250_SELF_TEST_GYRO_HIGH  (14.0F)
#define MPU9250_SELF_TEST_ACCE_LOW   (-14.0F)
#define MPU9250_SELF_TEST_ACCE_HIGH  (14.0F)


#define AK8963_ADDR                  0x0C

/* AK8963 register address */
#define AK8963_WIA                   0x00 /* should return 0x48 */
#define AK8963_INFO                  0x01
#define AK8963_ST1                   0x02 /* data ready status bit[0] */
#define AK8963_HXL                   0x03
#define AK8963_HXH                   0x04
#define AK8963_HYL                   0x05
#define AK8963_HYH                   0x06
#define AK8963_HZL                   0x07
#define AK8963_HZH                   0x08
#define AK8963_ST2                   0x09 /* data overflow bit 3 and data read error status bit 2 */
#define AK8963_CNTL1                 0x0A /* power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0 */
#define AK8963_CNTL2                 0x0B
#define AK8963_ASTC                  0x0C /* self test control */
#define AK8963_ASAX                  0x10 /* fuse ROM x-axis sensitivity adjustment value */
#define AK8963_ASAY                  0x11 /* fuse ROM y-axis sensitivity adjustment value */
#define AK8963_ASAZ                  0x12 /* fuse ROM z-axis sensitivity adjustment value */

/* AK8963 register bit */


/*public variable */

/* public operation */
extern void MPU9250_Init(void);

#endif


