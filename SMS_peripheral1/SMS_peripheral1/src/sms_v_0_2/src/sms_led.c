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
	if(gpio_pin_set_config(SMS_LED_0, &config_gpio_pin) != STATUS_OK) {
		DBG_LOG_DEV("[sms_led_gpio_init]\tproblem while setting up led0");
	}
	sms_led_switch_off(SMS_LED_0);
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

void sms_led_blink_start(enum sms_leds led, enum sms_timer_modes mode)
{
	sms_led_switch_on(led);
	uint32_t delay = 0;
	timer2_instance.current_mode = mode;
	switch(mode) {
		case TIMER_MODE_ADVERTISING:
		delay = SMS_BLINK_ADV_MS;
		break;
		
		default:
		break;
	}

	sms_dualtimer_register_callback(timer2_instance.id, sms_dualtimer2_cb);
	sms_dualtimer_start(TIMER_UNIT_MS, delay, timer2_instance.id);
}

void sms_led_blink_stop(enum sms_leds led)
{
	/* Switch LED off and unregister timer2 callback
	* to use it later as a blocking delay */
	sms_led_switch_off(led);
	timer2_instance.current_mode = TIMER_MODE_NONE;
	sms_dualtimer_unregister_callback(timer2_instance.id);
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