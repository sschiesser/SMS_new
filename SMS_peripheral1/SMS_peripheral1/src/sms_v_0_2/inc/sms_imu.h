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

#define SMS_MPU_BLE_CHAR_LEN_QUAT           (16) // 4 quaternions of 32 bit each = 16 uint8
#define SMS_MPU_BLE_CHAR_LEN_AHRS			(12) // yaw/pitch/roll of 32 bit each = 12 uint8

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

#define PI									(3.1415926535898)

//#define SMS_IMU_INTERRUPT_PIN PIN_AO_GPIO_2
//#define SMS_IMU_INIT_DATA_LEN 8
//#define SMS_IMU_READ_DATA_LEN 12
//#define SMS_IMU_ACCEL_Z_SENSITIVITY 16384
//#define SMS_IMU_GYRO_SENSITIVITY 131

/* ---------
 * VARIABLES
 * --------- */
enum sms_imu_state {
	IMU_STATE_OFF = 0,
	IMU_STATE_STDBY,
	IMU_STATE_ON
};
struct sms_imu_struct_s {
    struct mpu9250_config_s config; // config struct
	struct mpu9250_output_s output; // data output (fifo)
	struct mpu9250_interrupt_s interrupt; // interrupt states & flags
    enum sms_imu_state state;
    gatt_service_handler_t service_handler;
    uint8_t char_values[12];
};
struct sms_imu_struct_s imu_device;


/* ------------
 * DECLARATIONS
 * ------------ */
void sms_imu_configure_gpio(void);
void sms_imu_enable_callback(void);
void sms_imu_disable_callback(void);
void sms_imu_register_callbacks(void);
void sms_imu_unregister_callbacks(void);
void sms_imu_interrupt_callback(void);
int sms_imu_mpu_check(void);
int sms_imu_comp_check(void);
void sms_imu_mpu_calibrate(float *dest1, float *dest2);
void sms_imu_mpu_initialize(void);
void sms_imu_comp_initialize(float *destination);
int sms_imu_poll_data(void);
void sms_imu_define_services(void);
int sms_imu_startup(void);
void sms_imu_selftest(float *destination);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

void read_accel_data(int16_t *destination);
void read_gyro_data(int16_t *destination);
void read_comp_data(int16_t *destination);
int16_t read_temp_data(void);
float get_Mres(uint8_t m_scale);
float get_Gres(uint8_t g_scale);
float get_Ares(uint8_t a_scale);

void madgwick_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void mahony_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void ahrs_calculation(float *q);
//void sms_imu_interrupt_callback(void);
//void sms_imu_startup(void);
//int sms_imu_initialize(void);
//int sms_imu_configure(void);
//int sms_imu_poll_data(void);
//void sms_imu_service_init(gatt_service_handler_t *sms_imu_serv, uint8_t *sms_imu_value);
//at_ble_status_t sms_imu_primary_service_define(gatt_service_handler_t *sms_service);


#endif /* SMS_MPU_H_ */