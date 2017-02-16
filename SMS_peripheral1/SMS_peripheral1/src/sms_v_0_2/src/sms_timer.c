/*
* sms_timer.c
*
* Created: 08.06.2016 12:02:35
*  Author: Sébastien Schiesser
*/

#include "sms_timer.h"

/* AON SLEEP TIMER */
void sms_timer_aon_callback(void)
{
    //if(pressure_device.int_enabled) {
		pressure_device.interrupt.new_value = true;
	    send_plf_int_msg_ind(AON_SLEEP_TIMER_EXPIRY_CALLBACK, AON_TIMER_EXPIRED, NULL, 0);
	//}
}

void sms_timer_aon_init(uint32_t cnt_ms, enum aon_sleep_timer_mode cnt_mode)
{
    struct aon_sleep_timer_config config_aon_sleep_timer;
    aon_sleep_timer_get_config_defaults(&config_aon_sleep_timer);
    config_aon_sleep_timer.mode = cnt_mode;
    config_aon_sleep_timer.counter = cnt_ms * SMS_TIMER_AON_LOAD_1MS;
    aon_sleep_timer_init(&config_aon_sleep_timer);
}

void sms_timer_aon_disable(void)
{
    aon_sleep_timer_disable();
}

void sms_timer_aon_register_callback(void)
{
    aon_sleep_timer_register_callback(sms_timer_aon_callback);
    NVIC_EnableIRQ(AON_SLEEP_TIMER0_IRQn);
}

void sms_timer_aon_unregister_callback(void)
{
    aon_sleep_timer_unregister_callback();
    NVIC_DisableIRQ(AON_SLEEP_TIMER0_IRQn);
}


/* DUALTIMER */
void sms_dualtimer_init(void)
{
	timer1_instance.id = DUALTIMER_TIMER1;
	timer1_instance.current_mode = TIMER_MODE_NONE;
	timer1_instance.int_enabled = false;
	timer2_instance.id = DUALTIMER_TIMER2;
	timer2_instance.current_mode = TIMER_MODE_NONE;
	timer2_instance.int_enabled = false;
	
    struct dualtimer_config config_dualtimer;
    dualtimer_get_config_defaults(&config_dualtimer);

	/* Dualtimer1 used for background us counting without interrupts
	 * (similar to the millis() or micros() functions in Arduino) */
	config_dualtimer.timer1.load_value = 0xFFFFFFFF;
    config_dualtimer.timer1.counter_mode = DUALTIMER_FREE_RUNNING_MODE;
	config_dualtimer.timer1.interrup_enable = false;
	/* Dualtimer2 used first as a ms delay... 
	 * blocking (without cb) or not (with cb) */
    config_dualtimer.timer2.load_value = SMS_DUALTIMER_LOAD_MS;
    config_dualtimer.timer2.counter_mode = DUALTIMER_ONE_SHOT_MODE;
	config_dualtimer.timer2.interrup_enable = true;

    dualtimer_init(&config_dualtimer);
    dualtimer_disable(timer1_instance.id);
    dualtimer_disable(timer2_instance.id);
}

void sms_dualtimer_register_callback(enum dualtimer_timer tmr, sms_dualtimer_callback_t cb_handler)
{
    dualtimer_register_callback(tmr, cb_handler);
    sms_dualtimer_stop(tmr);
}

void sms_dualtimer_unregister_callback(enum dualtimer_timer tmr)
{
	sms_dualtimer_stop(tmr);
	dualtimer_unregister_callback(tmr);
}

void sms_dualtimer_start(timer_unit_type_t unit, uint32_t delay, enum dualtimer_timer tmr)
{
    uint32_t timer_load = 1;
    switch(unit) {
        case TIMER_UNIT_US:
        timer_load = SMS_DUALTIMER_LOAD_US;
        break;
        
        case TIMER_UNIT_MS:
        timer_load = SMS_DUALTIMER_LOAD_MS;
        break;
        
        case TIMER_UNIT_S:
        timer_load = SMS_DUALTIMER_LOAD_S;
        break;
        
        default:
        break;
    }
    
    if(delay <= 0) {
        //DBG_LOG("[sms_dualtimer_start]\tWarning! Delay value < 0... setting to 1");
        delay = 1;
    }
    
    //ulp_ready = false;
    DBG_LOG_DEV("[sms_dualtimer_start]\t\tStarting timer%d... load: %ld, delay: %ld", (tmr+1), timer_load, delay);
    dualtimer_set_counter(tmr, DUALTIMER_SET_CURRUNT_REG, timer_load * delay);
    dualtimer_enable(tmr);
    NVIC_EnableIRQ(DUALTIMER0_IRQn);
}

void sms_dualtimer_stop(enum dualtimer_timer tmr)
{
    dualtimer_disable(tmr);
}

void sms_dualtimer1_cb(void)
{
    timer1_instance.new_int = true;
    send_plf_int_msg_ind(DUALTIMER_TIMER1_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
}

void sms_dualtimer2_cb(void)
{
    timer2_instance.new_int = true;
    send_plf_int_msg_ind(DUALTIMER_TIMER2_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
}

void sms_dualtimer1_fn(void)
{
	
}

void sms_dualtimer2_fn(void)
{
	switch(timer2_instance.current_mode) {
		case TIMER_MODE_ADVERTISING:
		sms_led_toggle(SMS_LED_0);
		sms_dualtimer_start(TIMER_UNIT_MS, SMS_BLINK_ADV_MS, timer2_instance.id);
		break;
		
		default:
		break;
	}
}

void delay_ms(uint32_t delay) {
	uint32_t last = 0xffffffff;
	volatile uint32_t now;
	volatile bool quit = false;
	sms_dualtimer_start(TIMER_UNIT_MS, delay, timer2_instance.id);
	while(!quit) {
		now = dualtimer_get_value(timer2_instance.id);
		DBG_LOG_DEV("now: %lu", now);
		if((now == 0) || (now > last)) quit = true;
		else last = now;
	}
	sms_dualtimer_stop(timer2_instance.id);
}