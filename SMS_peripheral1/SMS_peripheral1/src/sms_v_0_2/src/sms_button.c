/*
* sms_button.c
*
* Created: 03.05.2016 15:02:24
*  Author: Sébastien Schiesser
*/

#include "sms_button.h"

/************************************************************************/
/* Callback functions --> doing things                                  */
/************************************************************************/
int sms_button_fn(enum sms_btn_ids btn)
{
	bool wait_success = true;
    button_instance.previous_state = button_instance.current_state;
    button_instance.current_state = sms_button_get_state();
    
#if defined(DEBUG)
    if(btn == SMS_BTN_0) sms_monitor_get_states("[sms_button_fn]-0");
    else if(btn == SMS_BTN_1) sms_monitor_get_states("[sms_button_fn]-1");
    else return -1;
#endif
    
    switch(button_instance.current_state) {
        // --- current state ---
        case BUTTON_STATE_B0:
		case BUTTON_STATE_B1:
        switch(ble_instance.current_state) {
            case BLE_STATE_POWEROFF: // start-up command?
			DBG_LOG_DEV("[sms_button_fn]\t\t\tWaking up ");
			sms_disable_irq(SMS_BTN_0_IRQ | SMS_BTN_1_IRQ);
			for(uint8_t i = 0; i < 50; i++) {
				//gpio_pin_set_output_level(DBG_PIN_1, true);
				delay_ms(50);
				if(sms_button_get_state() != button_instance.current_state) {
					//gpio_pin_set_output_level(DBG_PIN_2, true);
					wait_success = false;
					break;
				}
				//gpio_pin_set_output_level(DBG_PIN_1, false);
				DBG_LOG_CONT_DEV(".");
			}
			sms_enable_irq(SMS_BTN_0_IRQ | SMS_BTN_1_IRQ);
			//gpio_pin_set_output_level(DBG_PIN_2, false);
			//gpio_pin_set_output_level(DBG_PIN_1, false);
			if(wait_success) {
				if(sms_ble_startup()) return -1;
			}
            break;
            
            case BLE_STATE_PAIRED:
            case BLE_STATE_INDICATING:
			sms_ble_send_characteristic(BLE_CHAR_BTN);
            break;
            
            case BLE_STATE_DISCONNECTED:
            case BLE_STATE_ADVERTISING:
            case BLE_STATE_CONNECTED:
            default:
            DBG_LOG_DEV("[sms_button_fn]\t\t\tNot used states...");
            return -1;
            break;
        }
        break;
        
        // --- current state ---
        case BUTTON_STATE_BOTH:
        if(ble_instance.current_state == BLE_STATE_POWEROFF) {
			DBG_LOG_DEV("[sms_button_fn]\t\t\tNot used state...");
            //ulp_ready = true;
        }
        else {
			DBG_LOG_DEV("[sms_button_fn]\t\t\tShutting down ");
			sms_disable_irq(SMS_BTN_0_IRQ | SMS_BTN_1_IRQ);
			for(uint8_t i = 0; i < 50; i++) {
				delay_ms(50);
				if(sms_button_get_state() != button_instance.current_state) {
					wait_success = false;
					break;
				}
				DBG_LOG_CONT_DEV(".");
			}
			sms_enable_irq(SMS_BTN_0_IRQ | SMS_BTN_1_IRQ);
			if(wait_success) {
				sms_ble_power_down();
			}
            //if((ble_current_state == BLE_STATE_PAIRED) || (ble_current_state == BLE_STATE_INDICATING)) {
                //pressure_device.state = PRESSURE_STATE_STDBY;
            //}
            //else {
                //pressure_device.state = PRESSURE_STATE_OFF;
            //}
            //sms_sensors_interrupt_toggle(false, false);
            //sms_btn_cnt = 0;
            //ulp_ready = false;
        }
        break;
        
        // --- current state ---
        case BUTTON_STATE_NONE:
		DBG_LOG_DEV("[sms_button_fn]\t\t\tNone");
        //ulp_ready = true;
        //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE);
        //if((timer1_current_mode == TIMER1_MODE_NONE) && (timer2_current_mode == TIMER2_MODE_NONE)) release_sleep_lock();
        break;
    }
    return 0;
}

/* Initialize all button-related variables */
void sms_button_init_variables(void)
{
	button_instance.current_state = BUTTON_STATE_NONE;
	button_instance.btn0.id = SMS_BTN_0;
	button_instance.btn0.gpio_pin = SMS_BTN_0_PIN;
	button_instance.btn0.int_enabled = true;
	button_instance.btn0.new_int = false;
	button_instance.btn0.char_value = 0;
	button_instance.btn1.id = SMS_BTN_1;
	button_instance.btn1.gpio_pin = SMS_BTN_1_PIN;
	button_instance.btn1.int_enabled = true;
	button_instance.btn1.new_int = false;
	button_instance.btn1.char_value = 0;
}

/* Initialize gpio for button inputs */
void sms_button_gpio_init(void)
{
    struct gpio_config config_gpio_pin;

    /* Button0 @ PIN_AO_GPIO_0 */
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_INPUT;
    config_gpio_pin.input_pull = GPIO_PIN_PULL_DOWN;
    config_gpio_pin.aon_wakeup = true;
    if(gpio_pin_set_config(button_instance.btn0.gpio_pin, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG_DEV("[sms_button_configure]\tproblem while setting up button0");
    }
    
    /* Button1 @ PIN_AO_GPIO_2 */
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_INPUT;
    config_gpio_pin.input_pull = GPIO_PIN_PULL_DOWN;
    config_gpio_pin.aon_wakeup = true;
    if(gpio_pin_set_config(button_instance.btn1.gpio_pin, &config_gpio_pin) != STATUS_OK) {
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
    gpio_register_callback(button_instance.btn0.gpio_pin, sms_button_bt0_callback, GPIO_CALLBACK_RISING);
    //gpio_enable_callback(SMS_BTN_0_PIN);
    
    /* Button1 callback */
    gpio_register_callback(button_instance.btn1.gpio_pin, sms_button_bt1_callback, GPIO_CALLBACK_RISING);
    //gpio_enable_callback(SMS_BTN_1_PIN);
    
    /* User button callback */
}

/* Get current buttons state */
enum sms_button_state sms_button_get_state(void)
{
    bool b0 = gpio_pin_get_input_level(SMS_BTN_0_PIN);
    bool b1 = gpio_pin_get_input_level(SMS_BTN_1_PIN);
    //DBG_LOG_DEV("[sms_button_get_state]\t\tButton state: %d %d", b1, b0);
    if(b0 && b1) return BUTTON_STATE_BOTH;
    else if(b0 && !b1) return BUTTON_STATE_B0;
    else if(!b0 && b1) return BUTTON_STATE_B1;
    else return BUTTON_STATE_NONE;
}

/* En- or disable button interrupts */
void sms_button_toggle_callback(enum sms_btn_int_tog tog0, enum sms_btn_int_tog tog1)
{
    if(tog0 == SMS_BTN_INT_ENABLE) {
        gpio_enable_callback(button_instance.btn0.gpio_pin);
    }
    else if(tog0 == SMS_BTN_INT_DISABLE) {
        gpio_disable_callback(button_instance.btn0.gpio_pin);
    }
    
    if(tog1 == SMS_BTN_INT_ENABLE) {
        gpio_enable_callback(button_instance.btn1.gpio_pin);
    }
    else if(tog1 == SMS_BTN_INT_DISABLE) {
        gpio_disable_callback(button_instance.btn1.gpio_pin);
    }
}
/* Callbacks --> sending interrupt message to platform */
void sms_button_bt0_callback(void)
{
    button_instance.btn0.new_int = true;
    send_plf_int_msg_ind(button_instance.btn0.gpio_pin, GPIO_CALLBACK_RISING, NULL, 0);
}
void sms_button_bt1_callback(void)
{
	button_instance.btn1.new_int = true;
    send_plf_int_msg_ind(button_instance.btn1.gpio_pin, GPIO_CALLBACK_RISING, NULL, 0);
}

/* Define BLE service for buttons */
void sms_button_define_services(void)
{
    at_ble_status_t status;
    uint8_t init_value = 0;
    sms_ble_service_init(BLE_SERV_BUTTON, &button_instance.service_handler, &init_value);
    if((status = sms_ble_primary_service_define(&button_instance.service_handler)) != AT_BLE_SUCCESS) {
        DBG_LOG("[sms_button_define_services]\tServices defining failed, reason 0x%x", status);
    }
    else {
        DBG_LOG_DEV("[sms_button_define_services]\tServices defined, SMS button handle: %d", button_instance.service_handler.serv_handle);
    }
}