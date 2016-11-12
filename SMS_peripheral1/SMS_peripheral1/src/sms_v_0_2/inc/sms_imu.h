/*
 * sms_imu.h
 *
 * Created: 23.09.2016 08:52:00
 *  Author: Sébastien Schiesser
 */ 


#ifndef SMS_IMU_H_
#define SMS_IMU_H_

/* -------
 * INCLUDE
 * ------- */
#include "sms_peripheral1.h"
//#include "mpu9250.h"


/* ------
 * MACROS
 * ------ */
#define SMS_IMU_DRDY_PIN    PIN_AO_GPIO_1
#define SMS_IMU_VCC_PIN     PIN_LP_GPIO_6

#define SMS_IMU_I2C_SLAVE_ADDR      0x69

//#define SMS_IMU_INTERRUPT_PIN PIN_AO_GPIO_2
//#define SMS_IMU_INIT_DATA_LEN 8
//#define SMS_IMU_READ_DATA_LEN 12
//#define SMS_IMU_ACCEL_Z_SENSITIVITY 16384
//#define SMS_IMU_GYRO_SENSITIVITY 131


/* ---------
 * VARIABLES
 * --------- */
///* GATT service handler */
//gatt_service_handler_t sms_imu_service_handler;
///* Button characteristic */
//uint8_t sms_imu_char_values[12];
//uint8_t sms_imu_char_init_values[12];
//
//extern const uint8_t sms_imu_init_data[][2];
//
//enum accel_fs_values {
    //ACCEL_FS_2G,
    //ACCEL_FS_4G,
    //ACCEL_FS_8G,
    //ACCEL_FS_16G,
    //ACCEL_FS_ERR
//};
//
//enum gyro_fs_values {
    //GYRO_FS_250DPS,
    //GYRO_FS_500DPS,
    //GYRO_FS_1000DPS,
    //GYRO_FS_2000DPS,
    //GYRO_FS_ERR
//};
//
//typedef struct sms_imu_device
//{
    //bool init_ok;
    //volatile bool comm_error;
    ////uint16_t accel_divider;
    //uint8_t accel_scale;
    //uint8_t accel_data[3];
    ////uint16_t gyro_divider;
    //uint16_t gyro_scale;
    //uint8_t gyro_data[3];
//} sms_imu_device_t;
//
//sms_imu_device_t mpu9250_device;
//
//struct rx_s {
    //unsigned char header[3];
    //unsigned char cmd;
//};
//struct hal_s {
    //unsigned char lp_accel_mode;
    //unsigned char sensors;
    //unsigned char dmp_on;
    //unsigned char wait_for_tap;
    //volatile unsigned char new_gyro;
    //unsigned char motion_int_mode;
    //unsigned long no_dmp_hz;
    //unsigned long next_pedo_ms;
    //unsigned long next_temp_ms;
    //unsigned long next_compass_ms;
    //unsigned int report;
    //unsigned short dmp_features;
    //struct rx_s rx;
//};
//
//
//extern struct chip_cfg_s chip_cfg;
//extern struct gyro_state_s st;


/* ------------
 * DECLARATIONS
 * ------------ */
void sms_imu_configure_gpio(void);
void sms_imu_register_callbacks(void);
void sms_imu_unregister_callbacks(void);
void sms_imu_interrupt_callback(void);

//void sms_imu_interrupt_callback(void);
//void sms_imu_startup(void);
//int sms_imu_initialize(void);
//int sms_imu_configure(void);
//int sms_imu_poll_data(void);
//void sms_imu_service_init(gatt_service_handler_t *sms_imu_serv, uint8_t *sms_imu_value);
//at_ble_status_t sms_imu_primary_service_define(gatt_service_handler_t *sms_service);


#endif /* SMS_IMU_H_ */