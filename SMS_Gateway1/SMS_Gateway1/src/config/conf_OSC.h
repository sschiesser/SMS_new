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
///* Helper to enable string concatenation in macros */
//#define STR_HELPER(x)                   #x
//#define STR(x)                          STR_HELPER(x)
///* OSC addresses, with device instance numbers */
//#define SMS_OSC_ADDR_ACCEL(x)           "/sms/pressure\0,iiii"
////#define SMS_OSC_ADDR_ACCEL(x)           "/sms/" STR(x) "/mpu/accel\0,iiii" //x, y, z, sum
//#define SMS_OSC_ADDR_GYRO(x)            "/sms/" STR(x) "/mpu/gyro\0,iiii" //x, y, z, sum
//#define SMS_OSC_ADDR_JOYSTICK(x)        "/sms/" STR(x) "/mpu/joystick\0,ii" //heading, pitch
//#define SMS_OSC_ADDR_AHRS(x)            "/sms/" STR(x) "/mpu/ahrs\0,iiii" //4 quaternions
//#define SMS_OSC_ADDR_PRESS(x)           "/sms/" STR(x) "/pressure\0,i" //pressure
//#define SMS_OSC_ADDR_BUTTON(x)          "/sms/" STR(x) "/button\0,ii" //b0, b1
//#define SMS_OSC_ADDR_TEMP(x)            "/sms/" STR(x) "/temperature\0,ii" //Tpress, Tgyro
//#define SMS_OSC_ADDR_BATT(x)            "/sms/" STR(x) "/battery\0,i" //battery
//#define SMS_OSC_ADDR_DELTA(x)           "/sms/" STR(x) "/delta\0,iii" //Dpacket, Dmpu, Dpress
//#define SMS_OSC_TERMINATION             'Q'

#define SMS_OSC_MSG_MAX_LEN             100

/* ---------
 * VARIABLES
 * --------- */
enum sensor_types {
	SMS_SENS_ACCEL,
	SMS_SENS_GYRO,
	SMS_SENS_BUTTON,
	SMS_SENS_PRESS,
	SMS_SENS_QUAT
};

/* ------------
 * DECLARATIONS
 * ------------ */


#endif /* CONF_OSC_H_ */