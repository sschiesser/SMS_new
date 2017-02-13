/**
* \file
*
* \brief BLE Startup Template
*
* Copyright (c) 2016 Atmel Corporation. All rights reserved.
*
* \asf_license_start
*
* \page License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. The name of Atmel may not be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an
*    Atmel microcontroller product.
*
* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \asf_license_stop
*
*/

/*
* Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel
* Support</a>
*/

/**
* \mainpage
* \section preface Preface
* This is the reference manual for the Startup Template
*/
/*- Includes ---------------------------------------------------------------*/
#include "sms_peripheral1.h"

void sms_init_variables(void)
{
	timer1_current_mode = TIMER1_MODE_NONE;
	timer2_current_mode = TIMER2_MODE_NONE;
	sms_working_mode = SMS_MODE_BUTTON_SOLO;
	ulp_ready = false;
	sms_ble_timeout = BLE_APP_TIMEOUT_OFF;
	
	sms_ble_init_variables();
	sms_button_init_variables();
	sms_imu_init_variables();
	sms_pressure_init_variables();
}

static void resume_cb(void)
{
	init_port_list(); // re-initialize all ports
	serial_console_init(); // GPIO (UART) for the console
	sms_dualtimer_init();
	sms_button_gpio_init(); // GPIO (AO_0 & AO_1) for the buttons
	sms_led_gpio_init();
	sms_spi_master_configure();
	sms_i2c_master_configure();
	sms_imu_configure_gpio();
	sms_monitor_configure_gpio();
	//gpio_pin_set_output_level(SMS_PRESSURE_VCC_PIN, true);
}

int main(void)
{
	/* Define current BLE state
	* ------------------------ */
	ble_instance.current_state = BLE_STATE_STARTING;
	
	/* Initialize platform
	* ------------------- */
	platform_driver_init();
	gpio_init(); // GPIO
	serial_console_init(); // serial console for debugging
	
	/* Disable ULP
	* ----------- */
	acquire_sleep_lock();
	

	/* Initialize SMS flags
	* -------------------- */
	sms_init_variables();
	
	
	/* Initialize hardware components
	* ------------------------------ */
	// Dualtimer
	sms_dualtimer_init();
	
	// Buttons
	sms_button_gpio_init();
	
	// LED
	sms_led_gpio_init();
	
	// I2C
	sms_i2c_master_configure();
	
	// SPI
	sms_spi_master_configure();
	
	// MPU
	sms_imu_configure_gpio();
	
	// monitoring...
	sms_monitor_configure_gpio();
	
	/* Initialize the BLE module
	* ------------------------- */
	ble_device_init(NULL); // initialize the BLE chip and set the device address
	
	
	/* Define BLE services
	* ------------------- */
	sms_button_define_services();
	sms_pressure_define_services();
	sms_imu_define_services();
	
	
	/* Register callbacks
	* ------------------ */
	// Recovering from ULP
	register_resume_callback(resume_cb); // register resume callback

	// Dualtimer (AON timer enables on registration... so do it later)
	//sms_dualtimer_register_callback(DUALTIMER_TIMER1, sms_dualtimer1_cb); // button pressing timer
	// DUALTIMER_TIMER2 used for blocking delay!! So don't register callback!!

	// Buttons
	sms_button_register_callbacks();
	
	// MPU
	sms_imu_register_callbacks();

	// BLE
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GAP_EVENT_TYPE, sms_ble_gap_cb);
	ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GATT_SERVER_EVENT_TYPE, sms_ble_gatt_server_cb);


	/* Enable buttons interrupts
	* ------------------------- */
	sms_button_toggle_callback(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE);

	//sms_button_toggle_callback(SMS_BTN_INT_DISABLE, SMS_BTN_INT_DISABLE);
	
	/* Goto sleep
	* ---------- */
	//sms_ble_power_down();

	//sms_imu_startup();
	//sms_sensors_interrupt_toggle(true, false);
	//sms_timer_aon_init(23, AON_SLEEP_TIMER_RELOAD_MODE);
	//sms_timer_aon_register_callback();
	
	at_ble_status_t ble_status;
	static uint32_t cnt = 0;
	while(true)
	{
		/* BLE Event task */
		ble_status = ble_event_task(sms_ble_timeout);
		
		if(ble_status == AT_BLE_SUCCESS) {
			/* Sensor interrupt region */
			if(button_instance.btn0.new_int) {
				button_instance.btn0.new_int = false;
				DBG_LOG("Btn0 int... ");
				if(sms_button_fn(SMS_BTN_0) < 0) {
					DBG_LOG_DEV("Error in sms_button_fn()");
				}
			}
			if(button_instance.btn1.new_int) {
				button_instance.btn1.new_int = false;
				DBG_LOG("Btn1 int... ");
				if(sms_button_fn(SMS_BTN_1) < 0) {
					DBG_LOG_DEV("Error in sms_button_fn()");
				}
			}
			if(imu_device.interrupt.new_gyro) {
				//gpio_pin_set_output_level(DBG_PIN_1, DBG_PIN_HIGH);
				//DBG_LOG("MPU int (%ld)... ", cnt++);
				//sms_imu_poll_data();
				//static uint32_t past = 0;
				//const uint32_t cnt_max = 23 * SMS_TIMER_AON_LOAD_1MS / SMS_TIMER_AON_LOAD_100US;
				//uint32_t now = aon_sleep_timer_get_current_value()/SMS_TIMER_AON_LOAD_100US;
				//uint32_t delta = ((now < past) ? (past - now) : (cnt_max - now + past));
				//DBG_LOG("past: %lu, now: %lu, delta: %lu", past, now, delta);
				//past = now;
				imu_device.interrupt.new_gyro = false;
				//imu_device.interrupt.rts = true;
				//gpio_pin_set_output_level(DBG_PIN_1, DBG_PIN_LOW);
				//DBG_LOG_CONT_DEV("done");
			}
			if(pressure_device.interrupt.new_value) {
				//DBG_LOG("Press int (%d)... ", ble_instance.sending_queue);
				gpio_pin_set_output_level(DBG_PIN_2, DBG_PIN_HIGH);
				//sms_pressure_poll_data();
				pressure_device.interrupt.new_value = false;
				//pressure_device.rts = true;
				gpio_pin_set_output_level(DBG_PIN_2, DBG_PIN_LOW);
				//DBG_LOG_CONT_DEV("done");
			}
			
			///* Timer interrupt region */
			//if(timer1_instance.new_int) {
			//DBG_LOG("Timer1 int... ");
			//sms_dualtimer_stop(DUALTIMER_TIMER1);
			//sms_dualtimer1_fn();
			//timer1_instance.new_int = false;
			//}
			//if(timer2_instance.new_int) {
			//DBG_LOG("Timer2 int... ");
			//sms_dualtimer_stop(DUALTIMER_TIMER2);
			//sms_dualtimer2_fn();
			//timer2_instance.new_int = false;
			//}
			
			/* Sending region */
			if(imu_device.interrupt.rts) {
				//DBG_LOG("MPU sending (%d/%d)... ", pressure_device.new_int, ble_instance.sending_queue);
				gpio_pin_set_output_level(DBG_PIN_2, DBG_PIN_HIGH);
				if(ble_instance.sending_queue == 0) {
					sms_ble_send_characteristic(BLE_CHAR_MPU);
				}
				else {
					DBG_LOG_CONT("flushing!");
				}
				imu_device.interrupt.rts = false;
				gpio_pin_set_output_level(DBG_PIN_2, DBG_PIN_LOW);
			}
			if(pressure_device.interrupt.rts) {
				DBG_LOG("Press sending (%d/%d)... ", imu_device.interrupt.new_gyro, ble_instance.sending_queue);
				gpio_pin_set_output_level(DBG_PIN_2, DBG_PIN_HIGH);
				if(ble_instance.sending_queue == 0) {
					sms_ble_send_characteristic(BLE_CHAR_PRESS);
				}
				else {
					DBG_LOG_CONT("flushing!");
				}
				pressure_device.interrupt.rts = false;
				gpio_pin_set_output_level(DBG_PIN_2, DBG_PIN_LOW);
			}
		}
		else if(ble_status == AT_BLE_GAP_TIMEOUT) {
			DBG_LOG("GAP timeout");
		}
		else if(ble_status == AT_BLE_TIMEOUT) {
			DBG_LOG("Event get timeout");
		}
		else {
			DBG_LOG("BLE error occurred");
		}
	}
}

