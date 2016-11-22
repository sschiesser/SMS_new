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
    sms_current_interrupt.source = INT_AON_TIMER;
    send_plf_int_msg_ind(AON_SLEEP_TIMER_EXPIRY_CALLBACK, AON_TIMER_EXPIRED, NULL, 0);
}

void sms_timer_aon_init(uint32_t cnt, enum aon_sleep_timer_mode cnt_mode)
{
    struct aon_sleep_timer_config config_aon_sleep_timer;
    aon_sleep_timer_get_config_defaults(&config_aon_sleep_timer);
    config_aon_sleep_timer.mode = cnt_mode;
    config_aon_sleep_timer.counter = cnt;
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

void sms_timer_aon_get_ms(uint32_t *count)
{
    count = (uint32_t *)0;
}

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
    
    ulp_ready = false;
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
    sms_current_interrupt.source = INT_DUALTIMER1;
    send_plf_int_msg_ind(DUALTIMER_TIMER1_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
}

void sms_dualtimer2_cb(void)
{
    sms_current_interrupt.source = INT_DUALTIMER2;
    send_plf_int_msg_ind(DUALTIMER_TIMER2_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
}


void sms_dualtimer1_fn(void)
{
    //sms_button_toggle_interrupt(SMS_BTN_INT_DISABLE, SMS_BTN_INT_DISABLE);
    switch(timer1_current_mode) {
        /* Timer1 mode = STARTUP */
        case TIMER1_MODE_STARTUP:
        {
            timer1_current_mode = TIMER1_MODE_NONE;
            button_instance.previous_state = button_instance.current_state;
            button_instance.current_state = sms_button_get_state();
            sms_monitor_get_states("[sms_dualtimer1_fn]");
            
            switch(button_instance.previous_state) {
                // --- Timer1 mode = STARTUP: switch prev_state ---
                case BUTTON_STATE_B0:
                switch(button_instance.current_state) {
                    // --- prev_state = b0: switch current_state ---
                    case BUTTON_STATE_B0:
                    if(ble_current_state == BLE_STATE_POWEROFF) {
                        sms_btn_cnt++;
                        if(sms_btn_cnt >= SMS_BTN_STARTUP_CNT) {
                            timer1_current_mode = TIMER1_MODE_NONE;
                            sms_ble_startup();
                        }
                        else {
                            //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                            timer1_current_mode = TIMER1_MODE_STARTUP;
                            //ulp_ready = false;
                            sms_dualtimer_start(TIMER_UNIT_MS, SMS_BTN_STARTUP_MS, DUALTIMER_TIMER1);
                        }
                    }
                    else {
                        //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                        if(timer2_current_mode == TIMER2_MODE_NONE) {
                            //release_sleep_lock();
                            ulp_ready = true;
                        }                            
                    }
                    break;
                    
                    // --- prev_state = b0: switch current_state ---
                    case BUTTON_STATE_B1:
                    if(ble_current_state == BLE_STATE_POWEROFF) {
                        sms_btn_cnt = 0;
                        //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                        timer1_current_mode = TIMER1_MODE_STARTUP;
                        //ulp_ready = false;
                        sms_dualtimer_start(TIMER_UNIT_MS, SMS_BLINK_STARTUP_MS, DUALTIMER_TIMER1);
                    }
                    else {
                        //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                        if(timer2_current_mode == TIMER2_MODE_NONE) {
                            //release_sleep_lock();
                            ulp_ready = true;
                        }                            
                    }
                    break;
                    
                    // --- prev_state = b0: switch current_state ---
                    case BUTTON_STATE_BOTH:
                    case BUTTON_STATE_NONE:
                    default:
                    //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                    if(timer2_current_mode == TIMER2_MODE_NONE) {
                        //release_sleep_lock();
                        ulp_ready = true;
                    }                        
                    break;
                }
                break;
                
                // --- Timer1 mode = STARTUP: switch prev_state ---
                case BUTTON_STATE_B1:
                switch(button_instance.current_state) {
                    // --- prev_state = b1: switch current_state ---
                    case BUTTON_STATE_B1:
                    if(ble_current_state == BLE_STATE_POWEROFF) {
                        sms_btn_cnt++;
                        if(sms_btn_cnt >= SMS_BTN_STARTUP_CNT) {
                            timer1_current_mode = TIMER1_MODE_NONE;
                            sms_ble_startup();
                        }
                        else {
                            //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                            timer1_current_mode = TIMER1_MODE_STARTUP;
                            //ulp_ready = false;
                            sms_dualtimer_start(TIMER_UNIT_MS, SMS_BTN_STARTUP_MS, DUALTIMER_TIMER1);
                        }
                    }
                    else {
                        //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                        if(timer2_current_mode == TIMER2_MODE_NONE) {
                            //release_sleep_lock();
                            ulp_ready = true;
                        }                            
                    }
                    break;
                    
                    // --- prev_state = b1: switch current_state ---
                    case BUTTON_STATE_B0:
                    if(ble_current_state == BLE_STATE_POWEROFF) {
                        sms_btn_cnt = 0;
                        //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                        timer1_current_mode = TIMER1_MODE_STARTUP;
                        //ulp_ready = false;
                        sms_dualtimer_start(TIMER_UNIT_MS, SMS_BLINK_STARTUP_MS, DUALTIMER_TIMER1);
                    }
                    else {
                        //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                        if(timer2_current_mode == TIMER2_MODE_NONE) {
                            //release_sleep_lock();
                            ulp_ready = true;
                        }                            
                    }
                    break;
                    
                    // --- prev_state = b1: switch current_state ---
                    case BUTTON_STATE_NONE:
                    case BUTTON_STATE_BOTH:
                    default:
                    //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                    if(timer2_current_mode == TIMER2_MODE_NONE) {
                        //release_sleep_lock();
                        ulp_ready = true;
                    }                        
                    break;
                }
                break;
                
                // --- Timer1 mode = STARTUP: switch prev_state ---
                case BUTTON_STATE_NONE:
                case BUTTON_STATE_BOTH:
                default:
                {
                    //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                    if(timer2_current_mode == TIMER2_MODE_NONE) {
                        //release_sleep_lock();
                        ulp_ready = true;
                    }                        
                    break;
                }
            }
            break; // Timer1 mode = STARTUP
        }
        
        /* Timer1 mode = SHUTDOWN */
        case TIMER1_MODE_SHUTDOWN:
        {
            //timer1_current_mode = TIMER1_MODE_NONE;
            button_instance.previous_state = button_instance.current_state;
            button_instance.current_state = sms_button_get_state();
            sms_monitor_get_states("[sms_dualtimer1_fn]");
            
            if((button_instance.previous_state == BUTTON_STATE_BOTH) && (button_instance.current_state == BUTTON_STATE_BOTH)) {
                sms_btn_cnt++;
                if(sms_btn_cnt >= SMS_BTN_SHTDWN_CNT) {
                    sms_ble_power_down();
                }
                else {
                    //sms_sensors_toggle_interrupt(SMS_EXTINT_DISABLE);
                    timer1_current_mode = TIMER1_MODE_SHUTDOWN;
                    sms_dualtimer_start(TIMER_UNIT_MS, SMS_BTN_SHTDWN_MS, DUALTIMER_TIMER1);
                }
            }
            else {
                if(pressure_device.state == PRESSURE_STATE_STDBY) {
                    DBG_LOG_DEV("[sms_dualtimer1_fn]\t\tStarting sensors (shutting down)...");
                    sms_sensors_interrupt_toggle(false, true);
                }                    
                timer1_current_mode = TIMER1_MODE_NONE;
                if(timer2_current_mode == TIMER2_MODE_NONE) {
                    ulp_ready = true;
                }                    
            }
            break; // Timer1 mode = SHUTDOWN
        }
        
        case TIMER1_MODE_NONE:
        default:
        //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
        if(timer2_current_mode == TIMER2_MODE_NONE) {
            //release_sleep_lock();
            ulp_ready = true;
        }            
        break;
    } // switch(timer1_current_mode)
}

void sms_dualtimer2_fn(void)
{
    //sms_button_toggle_interrupt(SMS_BTN_INT_DISABLE, SMS_BTN_INT_DISABLE);
    sms_monitor_get_states("[sms_dualtimer2_fn]");
    switch(timer2_current_mode) {
        case TIMER2_MODE_INDICATION_TOUT:
        timer2_current_mode = TIMER2_MODE_NONE;
        if(ble_current_state == BLE_STATE_PAIRED) {
            //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
            if(timer1_current_mode == TIMER1_MODE_NONE) {
                //release_sleep_lock();
                ulp_ready = true;
            }                
        }
        else {
            sms_ble_ind_retry++;
            if(sms_ble_ind_retry >= BLE_INDICATION_RETRY_MAX) {
                DBG_LOG_CONT_DEV(" ...giving up!");
                //DBG_LOG_DEV("[sms_dualtimer2_fn]\tTimer1 mode: %d", timer1_current_mode);
                timer2_current_mode = TIMER2_MODE_NONE;
                ble_current_state = BLE_STATE_PAIRED;
                //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
                //DBG_LOG_DEV("[sms_dualtimer2_fn]\t\tStarting sensors...");
                //sms_sensors_toggle_interrupt(SMS_EXTINT_ENABLE);
                //if(timer1_current_mode == TIMER1_MODE_NONE) release_sleep_lock();
            }
            else {
                DBG_LOG_CONT_DEV(" ...waiting... counter: %d", sms_ble_ind_retry);
                //sms_ble_send_characteristic(BLE_CHAR_PRESSURE);
                timer2_current_mode = TIMER2_MODE_INDICATION_TOUT;
                //ulp_ready = false;
                sms_dualtimer_start(TIMER_UNIT_MS, BLE_INDICATION_TOUT_MS, DUALTIMER_TIMER2);
                //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
            }
        }
        break;
        
        case TIMER2_MODE_LED_STARTUP:
        DBG_LOG_DEV("[sms_dualtimer2_fn]\t\tBlinking startup...");
        timer2_current_mode = TIMER2_MODE_NONE;
        sms_led_blink_cnt++;
        if(sms_led_blink_cnt >= SMS_BLINK_STARTUP_CNT) {
            sms_led_switch_off(SMS_LED_0_PIN);
            //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
            /* Blinking done... starting ble advertisement */
            //DBG_LOG_DEV("[sms_dualtimer2_fn]\tSMS awake... starting advertisement");
            sms_ble_advertise();
            ulp_ready = true;
        }
        else {
            //DBG_LOG_DEV("[sms_dualtimer2_fn]\tBlinking up... cnt = %d", sms_led_blink_cnt);
            sms_led_toggle(SMS_LED_0_PIN);
            timer2_current_mode = TIMER2_MODE_LED_STARTUP;
            //ulp_ready = false;
            sms_dualtimer_start(TIMER_UNIT_MS, SMS_BLINK_STARTUP_MS, DUALTIMER_TIMER2);
        }
        break;
        
        case TIMER2_MODE_LED_SHUTDOWN:
        DBG_LOG_DEV("[sms_dualtimer2_fn]\t\tBlinking shutdown...");
        timer2_current_mode = TIMER2_MODE_NONE;
        sms_led_blink_cnt++;
        if(sms_led_blink_cnt >= SMS_BLINK_SHTDWN_CNT) {
            sms_led_switch_off(SMS_LED_0_PIN);
            DBG_LOG_DEV("[sms_dualtimer2_fn]\t\tPowering off...");
            //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
            sms_init_variables();
            ulp_ready = true;
            //release_sleep_lock();
        }
        else {
            //DBG_LOG_DEV("[sms_dualtimer2_fn]\tBLinking off... cnt = %d", sms_led_blink_cnt);
            sms_led_toggle(SMS_LED_0_PIN);
            timer2_current_mode = TIMER2_MODE_LED_SHUTDOWN;
            //ulp_ready = false;
            sms_dualtimer_start(TIMER_UNIT_MS, SMS_BLINK_SHTDWN_MS, DUALTIMER_TIMER2);
        }
        break;
        
        case TIMER2_MODE_LED_ADVERTISING:
        case TIMER2_MODE_LED_CONNECTION_LOST:
        case TIMER2_MODE_NONE:
        //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE)
        break;
    }
}