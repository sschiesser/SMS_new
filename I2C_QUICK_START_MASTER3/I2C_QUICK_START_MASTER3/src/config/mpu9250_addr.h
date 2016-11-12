/*
 * mpu9250.h
 *
 * Created: 09.11.2016 14:38:59
 *  Author: Sébastien Schiesser
 */ 


#ifndef MPU9250_ADDR_H_
#define MPU9250_ADDR_H_

/* MPU-9250 registers */
#define MPU9250_SELF_TEST_X_GYRO                (0x00) // R/W
#define MPU9250_SELF_TEST_Y_GYRO                (0x01) // R/W
#define MPU9250_SELF_TEST_Z_GYRO                (0x02) // R/W
#define MPU9250_SELF_TEST_X_ACCEL               (0x0D) // R/W
#define MPU9250_SELF_TEST_Y_ACCEL               (0x0E) // R/W
#define MPU9250_SELF_TEST_Z_ACCEL               (0x0F) // R/W
#define MPU9250_XG_OFFSET_H                     (0x13) // R/W
#define MPU9250_XG_OFFSET_L                     (0x14) // R/W
#define MPU9250_YG_OFFSET_H                     (0x15) // R/W
#define MPU9250_YG_OFFSET_L                     (0x16) // R/W
#define MPU9250_ZG_OFFSET_H                     (0x17) // R/W
#define MPU9250_ZG_OFFSET_L                     (0x18) // R/W
#define MPU9250_SMPLRT_DIV                      (0x19) // R/W
#define MPU9250_CONFIG                          (0X1A) // R/W
#define MPU9250_GYRO_CONFIG                     (0x1B) // R/W
#define MPU9250_ACCEL_CONFIG                    (0x1C) // R/W
#define MPU9250_ACCEL_CONFIG_2                  (0x1D) // R/W
#define MPU9250_LP_ACCEL_ODR                    (0x1E) // R/W
#define MPU9250_WOM_THR                         (0x1F) // R/W
#define MPU9250_FIFO_EN                         (0x23) // R/W
#define MPU9250_I2C_MST_CTRL                    (0x24) // R/W
#define MPU9250_I2C_SLV0_ADDR                   (0x25) // R/W
#define MPU9250_I2C_SLV0_REG                    (0x26) // R/W
#define MPU9250_I2C_SLV0_CTRL                   (0x27) // R/W
#define MPU9250_I2C_SLV1_ADDR                   (0x28) // R/W
#define MPU9250_I2C_SLV1_REG                    (0x29) // R/W
#define MPU9250_I2C_SLV1_CTRL                   (0x2A) // R/W
#define MPU9250_I2C_SLV2_ADDR                   (0x2B) // R/W
#define MPU9250_I2C_SLV2_REG                    (0x2C) // R/W
#define MPU9250_I2C_SLV2_CTRL                   (0x2D) // R/W
#define MPU9250_I2C_SLV3_ADDR                   (0x2E) // R/W
#define MPU9250_I2C_SLV3_REG                    (0x2F) // R/W
#define MPU9250_I2C_SLV3_CTRL                   (0x30) // R/W
#define MPU9250_I2C_SLV4_ADDR                   (0x31) // R/W
#define MPU9250_I2C_SLV4_REG                    (0x32) // R/W
#define MPU9250_I2C_SLV4_DO                     (0x33) // R/W
#define MPU9250_I2C_SLV4_CTRL                   (0x34) // R/W
#define MPU9250_I2C_SLV4_DI                     (0x35) // R
#define MPU9250_I2C_MST_STATUS                  (0x36) // R
#define MPU9250_INT_PIN_CFG                     (0x37) // R/W
#define MPU9250_INT_ENABLE                      (0x38) // R/W
#define MPU9250_INT_STATUS                      (0x3A) // R
#define MPU9250_ACCEL_XOUT_H                    (0x3B) // R
#define MPU9250_ACCEL_XOUT_L                    (0x3C) // R
#define MPU9250_ACCEL_YOUT_H                    (0x3D) // R
#define MPU9250_ACCEL_YOUT_L                    (0x3E) // R
#define MPU9250_ACCEL_ZOUT_H                    (0x3F) // R
#define MPU9250_ACCEL_ZOUT_L                    (0x40) // R
#define MPU9250_TEMP_OUT_H                      (0x41) // R
#define MPU9250_TEMP_OUT_L                      (0x42) // R
#define MPU9250_GYRO_XOUT_H                     (0x43) // R
#define MPU9250_GYRO_XOUT_L                     (0x44) // R
#define MPU9250_GYRO_YOUT_H                     (0x45) // R
#define MPU9250_GYRO_YOUT_L                     (0x46) // R
#define MPU9250_GYRO_ZOUT_H                     (0x47) // R
#define MPU9250_GYRO_ZOUT_L                     (0x48) // R
#define MPU9250_EXT_SENS_DATA_00                (0x49) // R
#define MPU9250_EXT_SENS_DATA_01                (0x4A) // R
#define MPU9250_EXT_SENS_DATA_02                (0x4B) // R
#define MPU9250_EXT_SENS_DATA_03                (0x4C) // R
#define MPU9250_EXT_SENS_DATA_04                (0x4D) // R
#define MPU9250_EXT_SENS_DATA_05                (0x4E) // R
#define MPU9250_EXT_SENS_DATA_06                (0x4F) // R
#define MPU9250_EXT_SENS_DATA_07                (0x50) // R
#define MPU9250_EXT_SENS_DATA_08                (0x51) // R
#define MPU9250_EXT_SENS_DATA_09                (0x52) // R
#define MPU9250_EXT_SENS_DATA_10                (0x53) // R
#define MPU9250_EXT_SENS_DATA_11                (0x54) // R
#define MPU9250_EXT_SENS_DATA_12                (0x55) // R
#define MPU9250_EXT_SENS_DATA_13                (0x56) // R
#define MPU9250_EXT_SENS_DATA_14                (0x57) // R
#define MPU9250_EXT_SENS_DATA_15                (0x58) // R
#define MPU9250_EXT_SENS_DATA_16                (0x59) // R
#define MPU9250_EXT_SENS_DATA_17                (0x5A) // R
#define MPU9250_EXT_SENS_DATA_18                (0x5B) // R
#define MPU9250_EXT_SENS_DATA_19                (0x5C) // R
#define MPU9250_EXT_SENS_DATA_20                (0x5D) // R
#define MPU9250_EXT_SENS_DATA_21                (0x5E) // R
#define MPU9250_EXT_SENS_DATA_22                (0x5F) // R
#define MPU9250_EXT_SENS_DATA_23                (0x60) // R
#define MPU9250_I2C_SLV0_DO                     (0x63) // R/W
#define MPU9250_I2C_SLV1_DO                     (0x64) // R/W
#define MPU9250_I2C_SLV2_DO                     (0x65) // R/W
#define MPU9250_I2C_SLV3_DO                     (0x66) // R/W
#define MPU9250_I2C_MST_DELAY_CTRL              (0x67) // R/W
#define MPU9250_SIGNAL_PATH_RESET               (0x68) // R/W
#define MPU9250_MOT_DETECT_CTRL                 (0x69) // R/W
#define MPU9250_USER_CTRL                       (0x6A) // R/W
#define MPU9250_PWR_MGMT_1                      (0x6B) // R/W
#define MPU9250_PWR_MGMT_2                      (0x6C) // R/W
#define MPU9250_FIFO_COUNTH                     (0x72) // R/W
#define MPU9250_FIFO_COUNTL                     (0x73) // R/W
#define MPU9250_FIFO_R_W                        (0x74) // R/W
#define MPU9250_WHO_AM_I                        (0x75) // R
#define MPU9250_XA_OFFSET_H                     (0x77) // R/W
#define MPU9250_XA_OFFSET_L                     (0x78) // R/W
#define MPU9250_YA_OFFSET_H                     (0x7A) // R/W
#define MPU9250_YA_OFFSET_L                     (0x7B) // R/W
#define MPU9250_ZA_OFFSET_H                     (0x7D) // R/W
#define MPU9250_ZA_OFFSET_L                     (0x7E) // R/W

#define MPU9250_SPI_READ                        (1 << 7)
#define MPU9250_SPI_WRITE                       (0)


/* MPU9250_CONFIG (0x1A) */
#define CONFIG_FIFO_MODE                        (1 << 6)
#define CONFIG_EXT_SYNC_SET                     (1 << 3)
#define CONFIG_DLPF_CFG_256HZ_NOLPF2            (0x00)
#define CONFIG_DLPF_CFG_188HZ                   (0x01)
#define CONFIG_DLPF_CFG_98HZ                    (0x02)
#define CONFIG_DLPF_CFG_42HZ                    (0x03)
#define CONFIG_DLPF_CFG_20HZ                    (0x04)
#define CONFIG_DLPF_CFG_10HZ                    (0x05)
#define CONFIG_DLPF_CFG_5HZ                     (0x06)
#define CONFIG_DLPF_CFG_2100HZ_NOLPF            (0x07)
/* MPU9250_GYRO_CONFIG (0x1B) */
#define GYRO_CONFIG_XGYRO_CTEN                  (1 << 7)
#define GYRO_CONFIG_YGYRO_CTEN                  (1 << 6)
#define GYRO_CONFIG_ZGYRO_CTEN                  (1 << 5)
#define GYRO_CONFIG_GYRO_FS_SEL_250DPS          (0x00 << 3)
#define GYRO_CONFIG_GYRO_FS_SEL_500DPS          (0x01 << 3)
#define GYRO_CONFIG_GYRO_FS_SEL_1000DPS         (0x02 << 3)
#define GYRO_CONFIG_GYRO_FS_SEL_2000DPS         (0x03 << 3)
#define GYRO_CONFIG_FCHOICE                     (1 << 0)
/* MPU9250_ACCEL_CONFIG (0x1C) */
#define ACCEL_CONFIG_AX_ST_EN                   (1 << 7)
#define ACCEL_CONFIG_AY_ST_EN                   (1 << 6)
#define ACCEL_CONFIG_AZ_ST_EN                   (1 << 5)
#define ACCEL_CONFIG_ACCEL_FS_SEL_2G            (0x00 << 3)
#define ACCEL_CONFIG_ACCEL_FS_SEL_4G            (0x01 << 3)
#define ACCEL_CONFIG_ACCEL_FS_SEL_8G            (0x02 << 3)
#define ACCEL_CONFIG_ACCEL_FS_SEL_16G           (0x03 << 3)
/* MPU9250_ACCEL_CONFIG_2 (0x1D) */
#define ACCEL_CONFIG_2_ACCEL_FCHOICE_B          (1 << 2)
#define ACCEL_CONFIG_2_A_DLPF_CFG               (1 << 0)
/* MPU9250_LP_ACCEL_ODR (0x1E) */
#define LP_ACCEL_ODR_LPOSC_CLKSEL               (1 << 0)
/* MPU9250_WOM_THR (0x1F) */

/* MPU9250_FIFO_EN (0x23) */
#define FIFO_EN_TEMP_FIFO_EN                    (1 << 7)
#define FIFO_EN_GYRO_XOUT                       (1 << 6)
#define FIFO_EN_GYRO_YOUT                       (1 << 5)
#define FIFO_EN_GYRO_ZOUT                       (1 << 4)
#define FIFO_EN_ACCEL                           (1 << 3)
#define FIFO_EN_SLV2                            (1 << 2)
#define FIFO_EN_SLV1                            (1 << 1)
#define FIFO_EN_SLV0                            (1 << 0)
/* MPU9250_I2C_MST_CTRL (0x24) */
#define I2C_MST_CTRL_MULT_MST_EN                (1 << 7)
#define I2C_MST_CTRL_WAIT_FOR_ES                (1 << 6)
#define I2C_MST_CTRL_SLV_3_FIFO_EN              (1 << 5)
#define I2C_MST_CTRL_I2C_MST_P_NSR              (1 << 4)
#define I2C_MST_CTRL_I2C_MST_CLK                (1 << 0)
/* MPU9250_I2C_SLV0_ADDR (0x25) */
#define I2C_SLV0_ADDR_I2C_SLV0_RNW              (1 << 7)
#define I2C_SLV0_ADDR_I2C_ID_0                  (1 << 0)
/* MPU9250_I2C_SLV0_REG (0x26) */

/* MPU9250_I2C_SLV0_CTRL (0x27) */
#define I2C_SLV0_CTRL_I2C_SLV0_EN               (1 << 7)
#define I2C_SLV0_CTRL_I2C_SLV0_BYTE_SW          (1 << 6)
#define I2C_SLV0_CTRL_I2C_SLV0_REG_DIS          (1 << 5)
#define I2C_SLV0_CTRL_I2C_SLV0_GRP              (1 << 4)
#define I2C_SLV0_CTRL_I2C_SLV0_LENG             (1 << 0)
/* MPU9250_I2C_SLV1_ADDR (0x28) */
#define I2C_SLV1_ADDR_I2C_SLV1_RNW              (1 << 7)
#define I2C_SLV1_ADDR_I2C_ID_1                  (1 << 0)
/* MPU9250_I2C_SLV1_REG (0x29) */

/* MPU9250_I2C_SLV1_CTRL (0x2A) */
#define I2C_SLV1_CTRL_I2C_SLV1_EN               (1 << 7)
#define I2C_SLV1_CTRL_I2C_SLV1_BYTE_SW          (1 << 6)
#define I2C_SLV1_CTRL_I2C_SLV1_REG_DIS          (1 << 5)
#define I2C_SLV1_CTRL_I2C_SLV1_GRP              (1 << 4)
#define I2C_SLV1_CTRL_I2C_SLV1_LENG             (1 << 0)
/* MPU9250_I2C_SLV2_ADDR (0x2B) */
#define I2C_SLV2_ADDR_I2C_SLV2_RNW              (1 << 7)
#define I2C_SLV2_ADDR_I2C_ID_2                  (1 << 0)
/* MPU9250_I2C_SLV2_REG (0x2C) */

/* MPU9250_I2C_SLV2_CTRL (0x2D) */
#define I2C_SLV2_CTRL_I2C_SLV2_EN               (1 << 7)
#define I2C_SLV2_CTRL_I2C_SLV2_BYTE_SW          (1 << 6)
#define I2C_SLV2_CTRL_I2C_SLV2_REG_DIS          (1 << 5)
#define I2C_SLV2_CTRL_I2C_SLV2_GRP              (1 << 4)
#define I2C_SLV2_CTRL_I2C_SLV2_LENG             (1 << 0)
/* MPU9250_I2C_SLV3_ADDR (0x2E) */
#define I2C_SLV3_ADDR_I2C_SLV3_RNW              (1 << 7)
#define I2C_SLV3_ADDR_I2C_ID_3                  (1 << 0)
/* MPU9250_I2C_SLV3_REG (0x2F) */

/* MPU9250_I2C_SLV3_CTRL (0x30) */
#define I2C_SLV3_CTRL_I2C_SLV3_EN               (1 << 7)
#define I2C_SLV3_CTRL_I2C_SLV3_BYTE_SW          (1 << 6)
#define I2C_SLV3_CTRL_I2C_SLV3_REG_DIS          (1 << 5)
#define I2C_SLV3_CTRL_I2C_SLV3_GRP              (1 << 4)
#define I2C_SLV3_CTRL_I2C_SLV3_LENG             (1 << 0)
/* MPU9250_I2C_SLV4_ADDR (0x31) */
#define I2C_SLV4_ADDR_I2C_SLV4_RNW              (1 << 7)
#define I2C_SLV4_ADDR_I2C_ID_4                  (1 << 0)
/* MPU9250_I2C_SLV0_REG (0x32) */

/* MPU9250_I2C_SLV4_DO (0x33) */

/* MPU9250_I2C_SLV4_CTRL (0x34) */
#define I2C_SLV4_CTRL_I2C_SLV4_EN               (1 << 7)
#define I2C_SLV4_CTRL_I2C_SLV4_DONE_INT_EN      (1 << 6)
#define I2C_SLV4_CTRL_I2C_SLV4_REG_DIS          (1 << 5)
#define I2C_SLV4_CTRL_I2C_MST_DLY               (1 << 0)

/* MPU9250_I2C_SLV4_DI (0x35) */

/* MPU9250_I2C_MST_STATUS (0x36) */
#define I2C_MST_STATUS_PASS_THROUGH             (1 << 7)
#define I2C_MST_STATUS_I2C_SLV4_DONE            (1 << 6)
#define I2C_MST_STATUS_I2C_LOST_ARB             (1 << 5)
#define I2C_MST_STATUS_I2C_SLV4_NACK            (1 << 4)
#define I2C_MST_STATUS_I2C_SLV3_NACK            (1 << 3)
#define I2C_MST_STATUS_I2C_SLV2_NACK            (1 << 2)
#define I2C_MST_STATUS_I2C_SLV1_NACK            (1 << 1)
#define I2C_MST_STATUS_I2C_SLV0_NACK            (1 << 0)
/* MPU9250_INT_PIN_CFG (0x37) */
#define INT_PIN_CFG_ACTL                        (1 << 7)
#define INT_PIN_CFG_OPEN                        (1 << 6)
#define INT_PIN_CFG_LATCH_INT_EN                (1 << 5)
#define INT_PIN_CFG_INT_ANYRD_2CLEAR            (1 << 4)
#define INT_PIN_CFG_ACTL_FSYNC                  (1 << 3)
#define INT_PIN_CFG_FSYNC_INT_MODE_EN           (1 << 2)
#define INT_PIN_CFG_BYPASS_EN                   (1 << 1)
/* MPU9250_INT_ENABLE (0x38) */
#define INT_ENABLE_WOM_EN                       (1 << 6)
#define INT_ENABLE_FIFO_OFLOW_EN                (1 << 4)
#define INT_ENABLE_FSYNC_INT_EN                 (1 << 3)
#define INT_ENABLE_RAW_RDY_EN                    (1 << 0)
/* MPU9250_INT_STATUS (0x3A) */
#define INT_STATUS_WOM_INT                      (1 << 6)
#define INT_STATUS_FIFO_OFLOW_INT               (1 << 4)
#define INT_STATUS_FSYNC_INT                    (1 << 3)
#define INT_STATUS_RAW_DATA_RDY_INT             (1 << 0)
/* MPU9250_ACCEL_XOUT_H (0x3B) - MPU9250_EXT_SENS_DATA_23 (0x60) */

/* MPU9250_I2C_SLV0_DO (0x63) - MPU9250_I2C_SLV3_DO (0x66) */

/* MPU9250_I2C_MST_DELAY_CTRL (0x67) */
#define I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW      (1 << 7)
#define I2C_MST_DELAY_CTRL_I2C_SLV4_DLY_EN      (1 << 4)
#define I2C_MST_DELAY_CTRL_I2C_SLV3_DLY_EN      (1 << 3)
#define I2C_MST_DELAY_CTRL_I2C_SLV2_DLY_EN      (1 << 2)
#define I2C_MST_DELAY_CTRL_I2C_SLV1_DLY_EN      (1 << 1)
#define I2C_MST_DELAY_CTRL_I2C_SLV0_DLY_EN      (1 << 0)
/* MPU9250_SIGNAL_PATH_RESET (0x68) */
#define SIGNAL_PATH_RESET_GYRO_RST              (1 << 2)
#define SIGNAL_PATH_RESET_ACCEL_RST             (1 << 1)
#define SIGNAL_PATH_RESET_TEMP_RST              (1 << 0)
/* MPU9250_MOT_DETECT_CTRL (0x69) */
#define MOT_DETECT_CTRL_ACCEL_INTEL_EN          (1 << 7)
#define MOT_DETECT_CTRL_ACCEL_INTEL_MODE        (1 << 6)
/* MPU9250_USER_CTRL (0x6A) */
#define USER_CTRL_FIFO_EN                       (1 << 6)
#define USER_CTRL_I2C_MST_EN                    (1 << 5)
#define USER_CTRL_I2C_IF_DIS                    (1 << 4)
#define USER_CTRL_FIFO_RST                      (1 << 2)
#define USER_CTRL_I2C_MST_RST                   (1 << 1)
#define USER_CTRL_SIG_COND_RST                  (1 << 0)
/* MPU9250_PWR_MGMT_1 (0x6B) */
#define PWR_MGMT_1_H_RESET                      (1 << 7)
#define PWR_MGMT_1_SLEEP                        (1 << 6)
#define PWR_MGMT_1_CYCLE                        (1 << 5)
#define PWR_MGMT_1_GYRO_STANDBY                 (1 << 4)
#define PWR_MGMT_1_PD_PTAT                      (1 << 3)
#define PWR_MGMT_1_CLKSEL_INT20MHZ              (0x00)
#define PWR_MGMT_1_CLKSEL_AUTO                  (0x01)
#define PWR_MGMT_1_CLKSEL_STOP                  (0x07)
/* MPU9250_PWR_MGMT_2 (0x6C) */
#define PWR_MGMT_2_DIS_XA                       (1 << 5)
#define PWR_MGMT_2_DIS_YA                       (1 << 4)
#define PWR_MGMT_2_DIS_ZA                       (1 << 3)
#define PWR_MGMT_2_DIS_XG                       (1 << 2)
#define PWR_MGMT_2_DIS_YG                       (1 << 1)
#define PWR_MGMT_2_DIS_ZG                       (1 << 0)
/* MPU9250_FIFO_COUNTH (0x72) - MPU9250_ZA_OFFSET_L (0x7E) */

#define MPU9250_ACCEL_FS_DIVIDER_2G             (16384)
#define MPU9250_ACCEL_FS_DIVIDER_4G             (8192)
#define MPU9250_ACCEL_FS_DIVIDER_8G             (4096)
#define MPU9250_ACCEL_FS_DIVIDER_16G            (2048)
#define MPU9250_ACCEL_FS_DIVIDER_ERR            (0)

#define MPU9250_GYRO_FS_DIVIDER_250DPSX10       (1310)
#define MPU9250_GYRO_FS_DIVIDER_500DPSX10       (655)
#define MPU9250_GYRO_FS_DIVIDER_1000DPSX10      (328)
#define MPU9250_GYRO_FS_DIVIDER_2000DPSX10      (164)
#define MPU9250_GYRO_FS_DIVIDER_ERR             (0)


#endif /* MPU9250_ADDR_H_ */