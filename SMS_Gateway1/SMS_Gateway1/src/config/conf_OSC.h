/*
 * conf_OSC.h
 *
 * Created: 29.11.2016 09:40:11
 *  Author: Sébastien Schiesser
 */ 


#ifndef CONF_OSC_H_
#define CONF_OSC_H_

/* -------
 * INCLUDE
 * ------- */


/* ------
 * MACROS
 * ------ */
/* Helper to enable string concatenation in macros */
#define STR_HELPER(x)                   #x
#define STR(x)                          STR_HELPER(x)
/* OSC addresses, with device instance numbers */
#define SMS_OSC_ADDR_ACCEL(x)           "/sms/" STR(x) "/mpu/accel"
#define SMS_OSC_ADDR_GYRO(x)            "/sms/" STR(x) "/mpu/gyro"
#define SMS_OSC_ADDR_JOYSTICK(x)        "/sms/" STR(x) "/mpu/joystick"
#define SMS_OSC_ADDR_AHRS(x)            "/sms/" STR(x) "/mpu/ahrs"
#define SMS_OSC_ADDR_PRESS(x)           "/sms/" STR(x) "/pressure"
#define SMS_OSC_ADDR_BUTTON(x)          "/sms/" STR(x) "/button"
#define SMS_OSC_ADDR_TEMP(x)            "/sms/" STR(x) "/temperature"
#define SMS_OSC_ADDR_BATT(x)            "/sms/" STR(x) "/battery"
#define SMS_OSC_ADDR_SYSTIME(x)         "/sms/" STR(x) "/systime"

/* ---------
 * VARIABLES
 * --------- */


/* ------------
 * DECLARATIONS
 * ------------ */




#endif /* CONF_OSC_H_ */