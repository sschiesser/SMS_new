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

extern struct gyro_state_s st;

struct i2c_master_packet i2c_wpacket;
struct i2c_master_packet i2c_rpacket;

void interrupt_cb(void);
void init_dualtimer(void);
void configure_i2c_master(void);
void imu_poll_data(void);




#endif /* INCLUDE_H_ */