/*
 * sms_common.h
 *
 * Created: 14.11.2016 12:47:53
 *  Author: Sébastien Schiesser
 */ 


#ifndef SMS_COMMON_H_
#define SMS_COMMON_H_


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
enum sms_sensor_state {
    SENSOR_STATE_OFF,
    SENSOR_STATE_STDBY,
    SENSOR_STATE_ON
};

typedef enum sms_extint_toggle {
    SMS_EXTINT_ENABLE,
    SMS_EXTINT_DISABLE
}sms_extint_toggle_t;


/* ------------
 * DECLARATIONS
 * ------------ */
/* General functions */
void sms_monitor_configure_gpio(void);
void sms_monitor_get_states(const char *label);

/* Sensors-related functions */
void sms_sensors_interrupt_toggle(bool imu_int, bool press_int);
void sms_sensors_switch(bool imu_en, bool press_en);

#endif /* SMS_COMMON_H_ */