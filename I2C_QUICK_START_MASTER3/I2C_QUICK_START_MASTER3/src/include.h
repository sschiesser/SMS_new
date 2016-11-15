/*
 * include.h
 *
 * Created: 09.11.2016 18:32:31
 *  Author: Sébastien Schiesser
 */ 


#ifndef INCLUDE_H_
#define INCLUDE_H_

/* -------
 * INCLUDE
 * ------- */
#include <asf.h>
#include "platform.h"
#include "console_serial.h"
#include "ble_manager.h"
#include "delay.h"
// Invensense's MotionDriver 6.12
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"

/* ------
 * MACROS
 * ------ */
/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (20)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)



/* ---------
 * VARIABLES
 * --------- */
struct i2c_master_module i2c_master_instance;
struct i2c_master_packet i2c_wpacket;
struct i2c_master_packet i2c_rpacket;


struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};



/* ------------
 * DECLARATIONS
 * ------------ */
void interrupt_cb(void);
void init_dualtimer(void);
void configure_i2c_master(void);
void imu_poll_data(void);




#endif /* INCLUDE_H_ */