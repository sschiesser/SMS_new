/*
* sms_button.c
*
* Created: 03.05.2016 15:02:24
*  Author: Sébastien Schiesser
*/

#include "sms_peripheral1.h"

/************************************************************************/
/* Callback functions --> doing things                                  */
/************************************************************************/
/* BUTTON_0 */
int sms_button_fn(enum sms_btn_ids btn)
{
    button_previous_state = button_current_state;
    button_current_state = sms_button_get_state();
    
    if(btn == SMS_BTN_0) sms_monitor_states("[sms_button_fn]-0");
    else if(btn == SMS_BTN_1) sms_monitor_states("[sms_button_fn]-1");
    else return -1;
    
    switch(button_current_state) {
        // --- current state ---
        case BUTTON_STATE_B0:
        switch(ble_current_state) {
            case BLE_STATE_POWEROFF:
            timer1_current_mode = TIMER1_MODE_STARTUP;
            timer2_current_mode = TIMER2_MODE_NONE;
            sms_btn_cnt = 0;
            //ulp_ready = false;
            sms_dualtimer_start(TIMER_UNIT_MS, SMS_BTN_STARTUP_MS, DUALTIMER_TIMER1);
            break;
            
            case BLE_STATE_PAIRED:
            timer1_current_mode = TIMER1_MODE_NONE;
            timer2_current_mode = TIMER2_MODE_NONE;
            //sms_ble_ind_retry = 0;
            if(btn == btn0_instance.id) sms_ble_send_characteristic(BLE_CHAR_BUTTON0);
            else if(btn == btn1_instance.id) sms_ble_send_characteristic(BLE_CHAR_BUTTON1);
            else return -1;
            break;
            
            case BLE_STATE_INDICATING:
            DBG_LOG_DEV("[sms_button_fn]\tStill indicating...");
            return -1;
            break;
            
            case BLE_STATE_DISCONNECTED:
            case BLE_STATE_ADVERTISING:
            case BLE_STATE_CONNECTED:
            default:
            DBG_LOG_DEV("[sms_button_fn]\t\t\tNot used states...");
            return -1;
            break;
        }
        //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
        break;
        
        // --- current state ---
        case BUTTON_STATE_B1:
        switch(ble_current_state) {
            case BLE_STATE_POWEROFF:
            timer1_current_mode = TIMER1_MODE_STARTUP;
            timer2_current_mode = TIMER2_MODE_NONE;
            sms_btn_cnt = 0;
            //ulp_ready = false;
            sms_dualtimer_start(TIMER_UNIT_MS, SMS_BTN_STARTUP_MS, DUALTIMER_TIMER1);
            break;
            
            case BLE_STATE_PAIRED:
            timer1_current_mode = TIMER1_MODE_NONE;
            timer2_current_mode = TIMER2_MODE_NONE;
            //sms_ble_ind_retry = 0;
            if(btn == btn0_instance.id) sms_ble_send_characteristic(BLE_CHAR_BUTTON0);
            else if(btn == btn1_instance.id) sms_ble_send_characteristic(BLE_CHAR_BUTTON1);
            else return -1;
            break;
            
            case BLE_STATE_INDICATING:
            DBG_LOG_DEV("[sms_button_fn]\tStill indicating...");
            return -1;
            break;
            
            case BLE_STATE_DISCONNECTED:
            case BLE_STATE_ADVERTISING:
            case BLE_STATE_CONNECTED:
            default:
            return -1;
            break;
        }
        //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
        break;
        
        // --- current state ---
        case BUTTON_STATE_BOTH:
        if(ble_current_state == BLE_STATE_POWEROFF) {
            timer1_current_mode = TIMER1_MODE_NONE;
            timer2_current_mode = TIMER2_MODE_NONE;
            ulp_ready = true;
            //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
            //release_sleep_lock();
        }
        else {
            sms_sensors_toggle_interrupt(SMS_EXT_INT_DISABLE);
            timer1_current_mode = TIMER1_MODE_SHUTDOWN;
            timer2_current_mode = TIMER2_MODE_NONE;
            sms_btn_cnt = 0;
            //ulp_ready = false;
            sms_dualtimer_start(TIMER_UNIT_MS, SMS_BTN_SHTDWN_MS, DUALTIMER_TIMER1);
            //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
        }
        break;
        
        // --- current state ---
        case BUTTON_STATE_NONE:
        ulp_ready = true;
        //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
        //if((timer1_current_mode == TIMER1_MODE_NONE) && (timer2_current_mode == TIMER2_MODE_NONE)) release_sleep_lock();
        break;
    }
    return 0;
}
/* BUTTON_1 */
//void sms_button_bt1_fn(void)
//{
    //button_previous_state = button_current_state;
    //button_current_state = sms_button_get_state();
    //sms_monitor_states("[sms_button_bt1_fn]");
//
    //switch(button_current_state) {
        //// --- current state ---
        //case BUTTON_STATE_B1:
        //switch(ble_current_state) {
            //case BLE_STATE_POWEROFF:
            //timer1_current_mode = TIMER1_MODE_STARTUP;
            //timer2_current_mode = TIMER2_MODE_NONE;
            //sms_btn_cnt = 0;
            //sms_dualtimer_start(TIMER_UNIT_MS, SMS_BTN_STARTUP_MS, DUALTIMER_TIMER1);
            //break;
            //
            //case BLE_STATE_PAIRED:
            //timer1_current_mode = TIMER1_MODE_NONE;
            //timer2_current_mode = TIMER2_MODE_NONE;
            ////sms_ble_ind_retry = 0;
            //sms_ble_send_characteristic(BLE_CHAR_BUTTON1);
            //break;
            //
            //case BLE_STATE_INDICATING:
            //DBG_LOG_DEV("[sms_button_bt0_fn]\tStill indicating...");
            //break;
            //
            //case BLE_STATE_DISCONNECTED:
            //case BLE_STATE_ADVERTISING:
            //case BLE_STATE_CONNECTED:
            //default:
            //break;
        //}
        //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
        //break;
        //
        //// --- current state ---
        //case BUTTON_STATE_B0:
        //switch(ble_current_state) {
            //case BLE_STATE_POWEROFF:
            //timer1_current_mode = TIMER1_MODE_STARTUP;
            //timer2_current_mode = TIMER2_MODE_NONE;
            //sms_btn_cnt = 0;
            //sms_dualtimer_start(TIMER_UNIT_MS, SMS_BTN_STARTUP_MS, DUALTIMER_TIMER1);
            //break;
            //
            //case BLE_STATE_PAIRED:
            //timer1_current_mode = TIMER1_MODE_NONE;
            //timer2_current_mode = TIMER2_MODE_NONE;
            ////sms_ble_ind_retry = 0;
            //sms_ble_send_characteristic(BLE_CHAR_BUTTON0);
            //break;
            //
            //case BLE_STATE_INDICATING:
            //DBG_LOG_DEV("[sms_button_bt0_fn]\tStill indicating...");
            //break;
            //
            //case BLE_STATE_DISCONNECTED:
            //case BLE_STATE_ADVERTISING:
            //case BLE_STATE_CONNECTED:
            //default:
            //break;
        //}
        //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
        //break;
        //
        //// --- current state ---
        //case BUTTON_STATE_BOTH:
        //if(ble_current_state == BLE_STATE_POWEROFF) {
            //timer1_current_mode = TIMER1_MODE_NONE;
            //timer2_current_mode = TIMER2_MODE_NONE;
            //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
            //release_sleep_lock();
        //}
        //else {
            //sms_sensors_toggle_interrupt(SMS_EXT_INT_DISABLE);
            //timer1_current_mode = TIMER1_MODE_SHUTDOWN;
            //timer2_current_mode = TIMER2_MODE_NONE;
            //sms_btn_cnt = 0;
            //sms_dualtimer_start(TIMER_UNIT_MS, SMS_BTN_SHTDWN_MS, DUALTIMER_TIMER1);
            //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
        //}
        //break;
//
        //// --- current state ---
        //case BUTTON_STATE_NONE:
        //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
        //if((timer1_current_mode == TIMER1_MODE_NONE) && (timer2_current_mode == TIMER2_MODE_NONE)) release_sleep_lock();
        //break;
    //}
//}
/************************************************************************/
/* Callback functions --> doing things                                  */
/************************************************************************/

/* Initialize gpio for button inputs */
void sms_button_configure_gpio(void)
{
    struct gpio_config config_gpio_pin;

    /* Button0 @ PIN_AO_GPIO_0 */
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_INPUT;
    config_gpio_pin.input_pull = GPIO_PIN_PULL_DOWN;
    config_gpio_pin.aon_wakeup = true;
    if(gpio_pin_set_config(btn0_instance.gpio_pin, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG_DEV("[sms_button_configure]\tproblem while setting up button0");
    }
    
    /* Button1 @ PIN_AO_GPIO_2 */
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_INPUT;
    config_gpio_pin.input_pull = GPIO_PIN_PULL_DOWN;
    config_gpio_pin.aon_wakeup = true;
    if(gpio_pin_set_config(btn1_instance.gpio_pin, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG("[sms_button_configure]\tProblem while setting up button1");
    }

    /* Button 0 on SAMB11 XPLAINED */
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_INPUT;
    config_gpio_pin.input_pull = GPIO_PIN_PULL_NONE;
    if(gpio_pin_set_config(BUTTON_0_PIN, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG("[sms_button_configure]\tProblem while setting up XPLAINED user button");
    }
}

/* Register button input callbacks */
void sms_button_register_callbacks(void)
{
    /* Button0 callback */
    gpio_register_callback(btn0_instance.gpio_pin, sms_button_bt0_callback, GPIO_CALLBACK_RISING);
    //gpio_enable_callback(SMS_BTN_0_PIN);
    
    /* Button1 callback */
    gpio_register_callback(btn1_instance.gpio_pin, sms_button_bt1_callback, GPIO_CALLBACK_RISING);
    //gpio_enable_callback(SMS_BTN_1_PIN);
    
    /* User button callback */
}

/* Disable button input callbacks */
void sms_button_disable_callbacks(void)
{
    /* Button0 callback */
    gpio_disable_callback(SMS_BTN_0_PIN);
    /* Button1 callback */
    gpio_disable_callback(SMS_BTN_1_PIN);
}

/* Get current buttons state */
sms_button_state_t sms_button_get_state(void)
{
    bool b0 = gpio_pin_get_input_level(SMS_BTN_0_PIN);
    bool b1 = gpio_pin_get_input_level(SMS_BTN_1_PIN);
    DBG_LOG("[sms_button_get_state]\t\tButton state: %d %d", b1, b0);
    if(b0 && b1) return BUTTON_STATE_BOTH;
    else if(b0 && !b1) return BUTTON_STATE_B0;
    else if(!b0 && b1) return BUTTON_STATE_B1;
    else return BUTTON_STATE_NONE;
}

/* En- or disable button interrupts */
void sms_button_toggle_interrupt(enum sms_btn_int_toggle en0, enum sms_btn_int_toggle en1)
{
    if(en0 == BTN_INT_ENABLE) {
        gpio_enable_callback(btn0_instance.gpio_pin);
    }
    else if(en0 == BTN_INT_DISABLE) {
        gpio_disable_callback(btn0_instance.gpio_pin);
    }
    
    if(en1 == BTN_INT_ENABLE) {
        gpio_enable_callback(btn1_instance.gpio_pin);
    }
    else if(en1 == BTN_INT_DISABLE) {
        gpio_disable_callback(btn1_instance.gpio_pin);
    }
}
/* Callbacks --> sending interrupt message to platform */
void sms_button_bt0_callback(void)
{
    //gpio_disable_callback(SMS_BTN_0_PIN);
    sms_current_interrupt.source = INT_BTN0;
    send_plf_int_msg_ind(btn0_instance.gpio_pin, GPIO_CALLBACK_RISING, NULL, 0);
}
void sms_button_bt1_callback(void)
{
    //gpio_disable_callback(SMS_BTN_1_PIN);
    sms_current_interrupt.source = INT_BTN1;
    send_plf_int_msg_ind(btn1_instance.gpio_pin, GPIO_CALLBACK_RISING, NULL, 0);
}

/* Define BLE service for buttons */
void sms_button_define_services(void)
{
    at_ble_status_t status;
    uint16_t init_value = 0;
    sms_button_service_init(&sms_button_service_handler, &init_value);
    if((status = sms_ble_primary_service_define(&sms_button_service_handler)) != AT_BLE_SUCCESS) {
        DBG_LOG("[sms_button_define_services]\tServices defining failed, reason 0x%x", status);
    }
    else {
        DBG_LOG_DEV("[sms_button_define_services]\tServices defined, SMS button handle: %d", sms_button_service_handler.serv_handle);
    }
}

/* Initialize BLE service for buttons */
void sms_button_service_init(gatt_service_handler_t *sms_button_serv, uint16_t *sms_button_value)
{
    uint8_t init_value = 0;
    //DBG_LOG_DEV("[sms_button_service_init]\tInitializing SMS button service");
    //SMS button service characteristic
    sms_button_serv->serv_handle = 0;
    sms_button_serv->serv_uuid.type = AT_BLE_UUID_128;
    sms_button_serv->serv_uuid.uuid[0] = (uint8_t) ((SMS_BTN_SERVICE_UUID_1) & 0xFF);
    sms_button_serv->serv_uuid.uuid[1] = (uint8_t) ((SMS_BTN_SERVICE_UUID_1 >> 8) & 0xFF);
    sms_button_serv->serv_uuid.uuid[2] = (uint8_t) ((SMS_BTN_SERVICE_UUID_1 >> 16) & 0xFF);
    sms_button_serv->serv_uuid.uuid[3] = (uint8_t) ((SMS_BTN_SERVICE_UUID_1 >> 24) & 0xFF);
    sms_button_serv->serv_uuid.uuid[4] = (uint8_t) ((SMS_BTN_SERVICE_UUID_2) & 0xFF);
    sms_button_serv->serv_uuid.uuid[5] = (uint8_t) ((SMS_BTN_SERVICE_UUID_2 >> 8) & 0xFF);
    sms_button_serv->serv_uuid.uuid[6] = (uint8_t) ((SMS_BTN_SERVICE_UUID_2 >> 16) & 0xFF);
    sms_button_serv->serv_uuid.uuid[7] = (uint8_t) ((SMS_BTN_SERVICE_UUID_2 >> 24) & 0xFF);
    sms_button_serv->serv_uuid.uuid[8] = (uint8_t) ((SMS_BTN_SERVICE_UUID_3) & 0xFF);
    sms_button_serv->serv_uuid.uuid[9] = (uint8_t) ((SMS_BTN_SERVICE_UUID_3 >> 8) & 0xFF);
    sms_button_serv->serv_uuid.uuid[10] = (uint8_t) ((SMS_BTN_SERVICE_UUID_3 >> 16) & 0xFF);
    sms_button_serv->serv_uuid.uuid[11] = (uint8_t) ((SMS_BTN_SERVICE_UUID_3 >> 24) & 0xFF);
    sms_button_serv->serv_uuid.uuid[12] = (uint8_t) ((SMS_BTN_SERVICE_UUID_4) & 0xFF);
    sms_button_serv->serv_uuid.uuid[13] = (uint8_t) ((SMS_BTN_SERVICE_UUID_4 >> 8) & 0xFF);
    sms_button_serv->serv_uuid.uuid[14] = (uint8_t) ((SMS_BTN_SERVICE_UUID_4 >> 16) & 0xFF);
    sms_button_serv->serv_uuid.uuid[15] = (uint8_t) ((SMS_BTN_SERVICE_UUID_4 >> 24) & 0xFF);
    
#   if SMS_SENDING_WITH_ACK == true
    sms_button_serv->serv_chars.properties = (AT_BLE_CHAR_READ | AT_BLE_CHAR_INDICATE); /* Properties */
#   else
    sms_button_serv->serv_chars.properties = (AT_BLE_CHAR_READ | AT_BLE_CHAR_NOTIFY); /* Properties */
#   endif
    sms_button_serv->serv_chars.init_value = &init_value;             /* value */
    sms_button_serv->serv_chars.value_init_len = (sizeof(uint8_t));
    sms_button_serv->serv_chars.value_max_len = (sizeof(uint8_t));
    sms_button_serv->serv_chars.value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);   /* permissions */
    sms_button_serv->serv_chars.user_desc = NULL;           /* user defined name */
    sms_button_serv->serv_chars.user_desc_len = 0;
    sms_button_serv->serv_chars.user_desc_max_len = 0;
    sms_button_serv->serv_chars.user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;             /*user description permissions*/
    sms_button_serv->serv_chars.client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*client config permissions*/
    sms_button_serv->serv_chars.server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*server config permissions*/
    sms_button_serv->serv_chars.user_desc_handle = 0;             /*user description handles*/
    sms_button_serv->serv_chars.client_config_handle = 0;         /*client config handles*/
    sms_button_serv->serv_chars.server_config_handle = 0;         /*server config handles*/
    
    sms_button_serv->serv_chars.presentation_format = NULL;       /* presentation format */
}