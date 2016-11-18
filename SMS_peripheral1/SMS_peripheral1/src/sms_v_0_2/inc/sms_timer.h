/*
 * sms_timer.h
 *
 * Created: 08.06.2016 12:01:20
 *  Author: Sébastien Schiesser
 */ 


#ifndef SMS_TIMER_H_
#define SMS_TIMER_H_

/* -------
 * INCLUDE
 * ------- */
#include "sms_peripheral1.h"

/* ------
 * MACROS
 * ------ */
#define SMS_TIMER_AON_FREQ              32768
#define SMS_TIMER_AON_COUNT_10H         (36000 * SMS_TIMER_AON_FREQ)
#define SMS_TIMER_AON_COUNT_10S         (10 * SMS_TIMER_AON_FREQ)
#define SMS_TIMER_AON_COUNT_1S          SMS_TIMER_AON_FREQ
#define SMS_TIMER_AON_COUNT_100MS       (SMS_TIMER_AON_FREQ / 10)
#define SMS_TIMER_AON_COUNT_10MS        (SMS_TIMER_AON_FREQ / 100)
#define SMS_TIMER_AON_COUNT_1MS         (SMS_TIMER_AON_FREQ / 1000)

#define SMS_DUALTIMER_LOAD_S            26000000
#define SMS_DUALTIMER_LOAD_MS           26000
#define SMS_DUALTIMER_LOAD_US           26
#define SMS_DUALTIMER_T1_LOAD           SMS_DUALTIMER_LOAD_MS
#define SMS_DUALTIMER_T2_LOAD           SMS_DUALTIMER_LOAD_MS
#define SMS_DUALTIMER_DLY_MAX_S         165
#define SMS_DUALTIMER_DLY_MAX_MS        165191
#define SMS_DUALTIMER_DLY_MAX_US        165191049

#define PWR_WAKEUP_DOMAIN_ARM           (1)
#define DUALTIMER_TIMER1_CALLBACK       0x40
#define DUALTIMER_TIMER2_CALLBACK       0x41
#define AON_SLEEP_TIMER_EXPIRY_CALLBACK 0x42

/* A value of 13'000 gives a (theoretical) resolution of 0.5 ms */
//#define SMS_TIMER_LOAD_PER_US       26
//#define SMS_TIMER_LOAD_PER_MS       26000
//#define SMS_TIMER_MS_TO_LOAD(x)     (x * SMS_TIMER_LOAD_PER_MS)
//#define SMS_TIMER_US_TO_LOAD(x)     (x * SMS_TIMER_LOAD_PER_US)
//
//#define SMS_TIMER_MAX_DELAY_VAL     0xFFFFFFFF

/* ---------
 * VARIABLES
 * --------- */
typedef enum timer1_modes {
    TIMER1_MODE_NONE = 0x00,
    TIMER1_MODE_STARTUP,
    TIMER1_MODE_MS58_RESET,
    TIMER1_MODE_SHUTDOWN
}timer1_modes_t;
volatile timer1_modes_t timer1_current_mode;

typedef enum timer2_modes {
    TIMER2_MODE_NONE,
    TIMER2_MODE_INDICATION_TOUT,
    TIMER2_MODE_LED_STARTUP,
    TIMER2_MODE_LED_SHUTDOWN,
    TIMER2_MODE_LED_ADVERTISING,
    TIMER2_MODE_LED_CONNECTION_LOST
}timer2_modes_t;
volatile timer2_modes_t timer2_current_mode;

/** Enum for the possible callback types for the timer module. */
enum timer_callback_type {
    /** Callback for timer expiry*/
    TIMER_EXPIRED_CALLBACK_TYPE_DETECT = 1,
    COUNTER_OVERFLOWED_CALLBACK_CONTINUE,
    AON_TIMER_EXPIRED
};

typedef enum timer_unit_type {
    TIMER_UNIT_US,
    TIMER_UNIT_MS,
    TIMER_UNIT_S
}timer_unit_type_t;

typedef void (*sms_dualtimer_callback_t)(void);


/* ------------
 * DECLARATIONS
 * ------------ */
/* Initialize the AON timer with corresponding counting value & mode,
 * AND START IT AUTOMATICALLY! */
void sms_timer_aon_init(uint32_t cnt, enum aon_sleep_timer_mode cnt_mode);
/* Disable the AON timer (stop counting) */
void sms_timer_aon_disable(void);
/* Register the callback corresponding to a AON timer interrupt,
 * AND ENABLE THE INTERRUPT VECTOR! */
void sms_timer_aon_register_callback(void);
/* Unregister the AON timer callback and disable the interrupt vector */
void sms_timer_aon_unregister_callback(void);
/* Callback function for the AON timer --> send a platform interruption with
 * corresponding parameters */
void sms_timer_aon_callback(void);
void sms_timer_aon_get_ms(uint32_t *count);


void sms_dualtimer_init(void);
void sms_dualtimer_register_callback(enum dualtimer_timer tmr, sms_dualtimer_callback_t cb_ptr);
void sms_dualtimer_start(timer_unit_type_t unit, uint32_t delay, enum dualtimer_timer tmr);
void sms_dualtimer_stop(enum dualtimer_timer tmr);
void sms_dualtimer1_cb(void);
void sms_dualtimer2_cb(void);
void sms_dualtimer1_fn(void);
void sms_dualtimer2_fn(void);

#endif /* SMS_TIMER_H_ */