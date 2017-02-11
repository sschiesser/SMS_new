/*
 * sms_imu.h
 *
 * Created: 23.09.2016 08:52:00
 *  Author: Sébastien Schiesser
 */ 


#ifndef SMS_MPU_H_
#define SMS_MPU_H_

/* -------
 * INCLUDE
 * ------- */
#include "sms_peripheral1.h"
#include "mpu9250.h"


/* ------
 * MACROS
 * ------ */
#define SMS_MPU_SERVICE_UUID_1              (0x1C575ABE)
#define SMS_MPU_SERVICE_UUID_2              (0x53501111)
#define SMS_MPU_SERVICE_UUID_3              (0x00000000)
#define SMS_MPU_SERVICE_UUID_4              (0x00000000)

#define SMS_MPU_BLE_CHAR_LEN_G_A            (12)
#define SMS_MPU_BLE_CHAR_LEN_G_A_C          (18)
#define SMS_MPU_BLE_CHAR_LEN_G_A_C_T        (20)

#define SMS_MPU_DRDY_PIN                    (PIN_AO_GPIO_1) // XPLAINED PRO --> EXT3:9
#define SMS_MPU_VCC_PIN                     (PIN_LP_GPIO_6) // XPLAINED PRO --> EXT3:13

#define SMS_MPU_SAMPLE_RATE_HZ              (50)
#define SMS_MPU_TEMP_MULTIPLIER             (1)
#define SMS_MPU_COMPASS_MULTIPLIER          (1)
#define SMS_MPU_COMPASS_RATE_HZ             (SMS_MPU_SAMPLE_RATE_HZ / SMS_MPU_COMPASS_MULTIPLIER)

#define SMS_MPU_ACCEL_ON                    (0x01)
#define SMS_MPU_GYRO_ON                     (0x02)
#define SMS_MPU_COMPASS_ON                  (0x04)

#define SMS_MPU_GYRO_FS						(2000)
#define SMS_MPU_ACCEL_FS					(2) // 4

#define SMS_MPU_MOTION                      (0)
#define SMS_MPU_NO_MOTION                   (1)

/*
 * These are the free parameters in the Mahony filter and fusion scheme, 
 * Kp for proportional feedback, Ki for integral
 */
#define Kp									(2.0f * 5.0f) 
#define Ki									(0.0f)

//#define SMS_IMU_INTERRUPT_PIN PIN_AO_GPIO_2
//#define SMS_IMU_INIT_DATA_LEN 8
//#define SMS_IMU_READ_DATA_LEN 12
//#define SMS_IMU_ACCEL_Z_SENSITIVITY 16384
//#define SMS_IMU_GYRO_SENSITIVITY 131

/* ---------
 * VARIABLES
 * --------- */
enum sms_mpu_state {
	MPU_STATE_OFF = 0,
	MPU_STATE_STDBY,
	MPU_STATE_ON
};
struct sms_mpu_struct_s {
    struct mpu9250_config_s config; // config struct
	struct mpu9250_output_s output; // data output (fifo)
	struct mpu9250_interrupt_s interrupt; // interrupt states & flags
    enum sms_mpu_state state;
    gatt_service_handler_t service_handler;
    uint8_t char_values[12];
};
struct sms_mpu_struct_s mpu_device;


float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4]; // vector to hold quaternion
float eInt[3]; // vector to hold integral error for Mahony method
float beta, deltat;

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
//extern struct chip_cfg_s chip_cfg;
//extern struct gyro_state_s st;


/* ------------
 * DECLARATIONS
 * ------------ */
void sms_mpu_configure_gpio(void);
void sms_mpu_enable_callback(void);
void sms_mpu_disable_callback(void);
void sms_mpu_register_callbacks(void);
void sms_mpu_unregister_callbacks(void);
void sms_mpu_interrupt_callback(void);
int sms_mpu_check(void);
int sms_mpu_comp_check(void);
void sms_mpu_calibrate(float *dest1, float *dest2);
void sms_mpu_initialize(void);
void sms_mpu_comp_initialize(float *destination);
int sms_mpu_poll_data(void);
void sms_mpu_define_services(void);

void sms_mpu_selftest(float *destination);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

//void sms_imu_interrupt_callback(void);
//void sms_imu_startup(void);
//int sms_imu_initialize(void);
//int sms_imu_configure(void);
//int sms_imu_poll_data(void);
//void sms_imu_service_init(gatt_service_handler_t *sms_imu_serv, uint8_t *sms_imu_value);
//at_ble_status_t sms_imu_primary_service_define(gatt_service_handler_t *sms_service);


#endif /* SMS_MPU_H_ */