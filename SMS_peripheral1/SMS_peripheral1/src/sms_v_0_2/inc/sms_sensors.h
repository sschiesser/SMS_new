/*
 * sms_sensors.h
 *
 * Created: 22.11.2016 08:22:49
 *  Author: Sébastien Schiesser
 */ 


#ifndef SMS_SENSORS_H_
#define SMS_SENSORS_H_
/* -------
 * INCLUDE
 * ------- */
#include "sms_peripheral1.h"


/* ------
 * MACROS
 * ------ */


/* ---------
 * VARIABLES
 * --------- */


/* ------------
 * DECLARATIONS
 * ------------ */
/* Sensors-related functions */
void sms_sensors_interrupt_enable(bool int_int, bool press_int);
void sms_sensors_switch(bool imu_en, bool press_en);




#endif /* SMS_SENSORS_H_ */