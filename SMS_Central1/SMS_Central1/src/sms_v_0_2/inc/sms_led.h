/*
 * sms_led.h
 *
 * Created: 19.09.2016 08:47:12
 *  Author: Sébastien Schiesser
 */ 


#ifndef SMS_LED_H_
#define SMS_LED_H_

/* -------
 * INCLUDE
 * ------- */
#include "sms_central1.h"

/* ------
 * MACROS
 * ------ */
#define SMS_LED_ACTIVE false
#define SMS_LED_INACTIVE true

#define SMS_BLINK_STARTUP_MS 100
#define SMS_BLINK_STARTUP_CNT 8
#define SMS_BLINK_SHTDWN_MS 400
#define SMS_BLINK_SHTDWN_CNT 4

/* ---------
 * VARIABLES
 * --------- */
enum sms_leds {
    SMS_LED_0_PIN = PIN_LP_GPIO_22
};

//typedef enum sms_led_blink_modes {
    //BLINK_MODE_STARTUP,
    //BLINK_MODE_SHUTTING_DOWN,
    //BLINK_MODE_ADVERTISING,
    //BLINK_MODE_CONNECTION_LOST
//}sms_led_blink_modes_t;
//sms_led_blink_modes_t led_current_mode;

volatile uint8_t sms_led_blink_cnt;

/* ------------
 * DECLARATIONS
 * ------------ */
void sms_led_gpio_init(void);
void sms_led_switch_on(enum sms_leds led);
void sms_led_switch_off(enum sms_leds led);
void sms_led_toggle(enum sms_leds led);
bool sms_led_get_state(enum sms_leds led);
void sms_led_blink_start(enum sms_leds led);
void sms_led_blink_fast(enum sms_leds led);
void sms_led_blink_slow(enum sms_leds led);



#endif /* SMS_LED_H_ */