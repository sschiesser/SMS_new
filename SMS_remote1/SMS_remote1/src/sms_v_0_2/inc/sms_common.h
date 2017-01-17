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
#include "sms_remote1.h"


/* ------
 * MACROS
 * ------ */


/* ---------
 * VARIABLES
 * --------- */
//enum sms_sensor_state {
    //PRESSURE_STATE_OFF,
    //PRESSURE_STATE_STDBY,
    //PRESSURE_STATE_ON
//};

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

#endif /* SMS_COMMON_H_ */