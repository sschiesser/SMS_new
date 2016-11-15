/*
 * include.h
 *
 * Created: 09.11.2016 18:32:31
 *  Author: Sébastien Schiesser
 */ 


#ifndef INCLUDE_H_
#define INCLUDE_H_

#include <asf.h>
#include "platform.h"
#include "console_serial.h"
#include "ble_manager.h"
#include "delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

extern struct gyro_state_s st;

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


struct i2c_master_packet i2c_wpacket;
struct i2c_master_packet i2c_rpacket;

void interrupt_cb(void);
void init_dualtimer(void);
void configure_i2c_master(void);
void imu_poll_data(void);




#endif /* INCLUDE_H_ */