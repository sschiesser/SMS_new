/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * This is a bare minimum user application template.
 *
 * For documentation of the board, go \ref group_common_boards "here" for a link
 * to the board-specific documentation.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# Basic usage of on-board LED and button
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */


/**
 * \mainpage
 * \section preface Preface
 * This is the reference manual for the Startup Template
 */

/*- Includes ---------------------------------------------------------------*/
#include "sms_remote1.h"

void sms_init_variables(void)
{
    timer1_current_mode = TIMER1_MODE_NONE;
    timer2_current_mode = TIMER2_MODE_NONE;
    sms_working_mode = SMS_MODE_BUTTON_SOLO;
    ulp_ready = false;

	// BLE
    ble_current_state = BLE_STATE_POWEROFF;
    sms_ble_send_cnt = 0;

	// button
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


static void resume_cb(void)
{
    init_port_list(); // re-initialize all ports
    serial_console_init(); // GPIO (UART) for the console
    sms_dualtimer_init();
    //delay_init();
    sms_button_configure_gpio(); // GPIO (AO_0 & AO_1) for the buttons
    //sms_led_gpio_init();
    //sms_monitor_configure_gpio();
    //gpio_pin_set_output_level(SMS_PRESSURE_VCC_PIN, true);
}

//static void sms_plf_event_cb(void)
//{
	//sms_current_interrupt.int_on = true;
//}


int main(void)
{
    /* Define current BLE state
     * ------------------------ */
    ble_current_state = BLE_STATE_STARTING;
    
    /* Initialize platform
     * ------------------- */
	platform_driver_init();
	gpio_init(); // GPIO
	serial_console_init(); // serial console for debugging
    
    /* Disable ULP
     * ----------- */
	//acquire_sleep_lock();
    

    /* Initialize SMS flags
     * -------------------- */
    sms_init_variables();
    
    
    /* Initialize hardware components
     * ------------------------------ */
    // Dualtimer
    sms_dualtimer_init();
	
    // Blocking delay (hacked from other SAM platforms)
    //delay_init();

	// Buttons
    sms_button_configure_gpio();
    
    // LED
    //sms_led_gpio_init();
    
	// monitoring...
    //sms_monitor_configure_gpio();
    
    /* Initialize the BLE module
     * ------------------------- */
	ble_device_init(NULL); // initialize the BLE chip and set the device address 
	
    
    /* Define BLE services
     * ------------------- */
    sms_button_define_services();
    
    /* Register callbacks
     * ------------------ */
    // Recovering from ULP
    register_resume_callback(resume_cb); // register resume callback

    // Dualtimer (AON timer enables on registration... so do it later)    
    sms_dualtimer_register_callback(DUALTIMER_TIMER1, sms_dualtimer1_cb); // button pressing timer
    sms_dualtimer_register_callback(DUALTIMER_TIMER2, sms_dualtimer2_cb); // LED blinking timer

    // Buttons
    sms_button_register_callbacks();
    
    // BLE
    ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GAP_EVENT_TYPE, sms_ble_gap_cb);
    ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GATT_SERVER_EVENT_TYPE, sms_ble_gatt_server_cb);
    //register_ble_user_event_cb(sms_plf_event_cb);

    /* Enable buttons interrupts
     * ------------------------- */
    sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE);
    
	
    /* Goto sleep
     * ---------- */
    //sms_ble_power_down();
	ulp_ready = true;
	release_sleep_lock();
	
    //ble_set_ulp_mode(BLE_ULP_MODE_SET);
    
	while(true)
	{
		/* BLE Event task */
		ble_event_task(BLE_EVENT_TIMEOUT);
		
		/* Sensor interrupt region */
		if(button_instance.btn0.new_int) {
			acquire_sleep_lock();
			DBG_LOG_DEV("Waking up... Btn0 int");
			//if(sms_button_fn(SMS_BTN_0) < 0) {
			if(sms_button_fn() < 0) {
				DBG_LOG_DEV("Error in sms_button_fn()");
			}
			// here
			button_instance.btn0.new_int = false;
		}
		if(button_instance.btn1.new_int) {
			acquire_sleep_lock();
			DBG_LOG_DEV("Waking up... Btn1 int");
			//if(sms_button_fn(SMS_BTN_1) < 0) {
			if(sms_button_fn() < 0) {
				DBG_LOG_DEV("Error in sms_button_fn()");
			}
			// here
			button_instance.btn1.new_int = false;
		}

		/* Timer interrupt region */
		if(timer1_instance.new_int) {
			DBG_LOG_DEV("Timer1 int... ");
			sms_dualtimer_stop(DUALTIMER_TIMER1);
			sms_dualtimer1_fn();
			timer1_instance.new_int = false;
		}
		if(timer2_instance.new_int) {
			DBG_LOG_DEV("Timer2 int... ");
			sms_dualtimer_stop(DUALTIMER_TIMER2);
			sms_dualtimer2_fn();
			timer2_instance.new_int = false;
		}
		
		/* ULP management */
		if(ulp_ready) {
			DBG_LOG_DEV("Going to sleep...");
			release_sleep_lock();
		}
		else {
			DBG_LOG_DEV("NOT tired!");
			acquire_sleep_lock();
		}
    }
}

