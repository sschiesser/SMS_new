/*
 * sms_led.c
 *
 * Created: 19.09.2016 08:46:43
 *  Author: Sébastien Schiesser
 */ 

#include "sms_led.h"

void sms_led_gpio_init(void)
{
    struct gpio_config config_gpio_pin;

    /* LED0 @ GPIO_LP_GPIO_22 */
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_OUTPUT;
    if(gpio_pin_set_config(SMS_LED_0_PIN, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG_DEV("[sms_led_gpio_init]\tproblem while setting up led0");
    }
    sms_led_switch_off(SMS_LED_0_PIN);
}


void sms_led_switch_on(enum sms_leds led)
{
    gpio_pin_set_output_level(led, SMS_LED_ACTIVE);
}


void sms_led_switch_off(enum sms_leds led)
{
    gpio_pin_set_output_level(led, SMS_LED_INACTIVE);
}


void sms_led_toggle(enum sms_leds led)
{
    bool state = sms_led_get_state(led);
    if(state == SMS_LED_ACTIVE) sms_led_switch_off(led);
    else sms_led_switch_on(led);
}


bool sms_led_get_state(enum sms_leds led)
{
    return gpio_pin_get_output_level(led);
}


void sms_led_blink_start(enum sms_leds led)
{
    sms_led_switch_on(led);
    sms_led_blink_cnt = 0;
    uint32_t delay = 0;
    switch(timer2_current_mode) {
        case TIMER2_MODE_LED_STARTUP:
        delay = SMS_BLINK_STARTUP_MS;
        break;
        
        case TIMER2_MODE_LED_SHUTDOWN:
        delay = SMS_BLINK_SHTDWN_MS;
        break;
        
        case TIMER2_MODE_LED_ADVERTISING:
        break;
        
        case TIMER2_MODE_LED_CONNECTION_LOST:
        break;
        
        default:
        break;
    }

    sms_dualtimer_start(TIMER_UNIT_MS, delay, DUALTIMER_TIMER2);
}
    
    
//void sms_led_blink_fast(enum sms_leds led)
//{
    //uint8_t cnt;
    //for(cnt = 0; cnt < LED_BLINK_FAST_CNT; cnt++) {
        //sms_led_switch_on(led);
        ////delay_ms(LED_BLINK_FAST_MS);
        //sms_led_switch_off(led);
        ////delay_ms(LED_BLINK_FAST_MS);
    //}
//}
//
//
//void sms_led_blink_slow(enum sms_leds led)
//{
    //uint8_t cnt;
    //for(cnt = 0; cnt < LED_BLINK_SLOW_CNT; cnt++) {
        //sms_led_switch_on(led);
        ////delay_ms(LED_BLINK_SLOW_MS);
        //sms_led_switch_off(led);
        ////delay_ms(LED_BLINK_SLOW_MS);
    //}
//}