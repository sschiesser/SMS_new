/*
* sms_timer.c
*
* Created: 08.06.2016 12:02:35
*  Author: Sébastien Schiesser
*/

#include "sms_timer.h"

/* DUALTIMER */
void sms_dualtimer_init(void)
{
    struct dualtimer_config config_dualtimer;
    dualtimer_get_config_defaults(&config_dualtimer);

    config_dualtimer.timer1.load_value = SMS_DUALTIMER_LOAD_S;
    config_dualtimer.timer1.counter_mode = DUALTIMER_ONE_SHOT_MODE;
    config_dualtimer.timer2.load_value = SMS_DUALTIMER_LOAD_S;
    config_dualtimer.timer2.counter_mode = DUALTIMER_ONE_SHOT_MODE;

    dualtimer_init(&config_dualtimer);
    dualtimer_disable(DUALTIMER_TIMER1);
    dualtimer_disable(DUALTIMER_TIMER2);
}

void sms_dualtimer_register_callback(enum dualtimer_timer tmr, sms_dualtimer_callback_t cb_handler)
{
    dualtimer_register_callback(tmr, cb_handler);
    sms_dualtimer_stop(tmr);
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
        //DBG_LOG("[sms_dualtimer_start]\tWarning! Delay value < 0... setting to 1000");
        delay = 1000;
    }
    
    //DBG_LOG_DEV("[sms_dualtimer_start]\t\tStarting timer%d... load: %ld, delay: %ld", (tmr+1), timer_load, delay);
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
    //sms_current_interrupt.source = INT_DUALTIMER1;
    send_plf_int_msg_ind(DUALTIMER_TIMER1_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
}

void sms_dualtimer2_cb(void)
{
    //sms_current_interrupt.source = INT_DUALTIMER2;
    send_plf_int_msg_ind(DUALTIMER_TIMER2_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
}


void sms_dualtimer1_fn(void)
{
}

void sms_dualtimer2_fn(void)
{
}