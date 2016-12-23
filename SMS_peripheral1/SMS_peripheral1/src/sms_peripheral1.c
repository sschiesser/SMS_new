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
    // states
    ble_current_state = BLE_STATE_POWEROFF;
    button_instance.current_state = BUTTON_STATE_NONE;
    pressure_device.hal.current_state = MS58_STATE_NONE;
    timer1_current_mode = TIMER1_MODE_NONE;
    timer2_current_mode = TIMER2_MODE_NONE;
    sms_working_mode = SMS_MODE_BUTTON_SOLO;

    // button
    btn0_instance.id = SMS_BTN_0;
    btn0_instance.gpio_pin = SMS_BTN_0_PIN;
    btn0_instance.int_enabled = true;
    btn0_instance.char_value = 0;
    
    pressure_device.state = PRESSURE_STATE_OFF;
    ulp_ready = false;
    
    
    btn1_instance.id = SMS_BTN_1;
    btn1_instance.gpio_pin = SMS_BTN_1_PIN;
    btn1_instance.int_enabled = true;
    btn1_instance.char_value = 0;
    
	for(uint8_t i = 0; i < 3; i++) {
		ready_to_send[i] = false;
	}
    sms_ble_send_cnt = 0;
}


static void resume_cb(void)
{
    init_port_list(); // re-initialize all ports
    serial_console_init(); // GPIO (UART) for the console
    sms_dualtimer_init();
    delay_init();
    sms_button_configure_gpio(); // GPIO (AO_0 & AO_1) for the buttons
    sms_led_gpio_init();
    sms_spi_master_configure();
    sms_i2c_master_configure();
    //sms_mpu_configure_gpio();
    sms_monitor_configure_gpio();
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
	acquire_sleep_lock();
    

    /* Initialize SMS flags
     * -------------------- */
    sms_init_variables();
    
    
    /* Initialize hardware components
     * ------------------------------ */
    // Dualtimer
    sms_dualtimer_init();
	
    // Blocking delay (hacked from other SAM platforms)
    delay_init();

	// Buttons
    sms_button_configure_gpio();
    
    // LED
    sms_led_gpio_init();
    
    // I2C
    sms_i2c_master_configure();
    
    // SPI
    sms_spi_master_configure();
    
    // MPU
    sms_mpu_configure_gpio();
    
	// monitoring...
    sms_monitor_configure_gpio();
    
    /* Initialize the BLE module
     * ------------------------- */
	ble_device_init(NULL); // initialize the BLE chip and set the device address 
	
    
    /* Define BLE services
     * ------------------- */
    sms_button_define_services();
    sms_pressure_define_services();
    sms_mpu_define_services();
    
    
    /* Register callbacks
     * ------------------ */
    // Recovering from ULP
    register_resume_callback(resume_cb); // register resume callback

    // Dualtimer (AON timer enables on registration... so do it later)    
    sms_dualtimer_register_callback(DUALTIMER_TIMER1, sms_dualtimer1_cb); // button pressing timer
    sms_dualtimer_register_callback(DUALTIMER_TIMER2, sms_dualtimer2_cb); // LED blinking timer

    // Buttons
    sms_button_register_callbacks();
    
    // MPU
    sms_mpu_register_callbacks();

    // BLE
    ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GAP_EVENT_TYPE, sms_ble_gap_cb);
    ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GATT_SERVER_EVENT_TYPE, sms_ble_gatt_server_cb);
    //register_ble_user_event_cb(sms_plf_event_cb);

    //ble_set_ulp_mode(BLE_ULP_MODE_SET);
    
    /* Enable buttons interrupts
     * ------------------------- */
    sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE);
    
    //gpio_pin_set_output_level(SMS_PRESSURE_VCC_PIN, true);
    
    //register int n21 asm("sp");
    //register int n22 asm("lr");
    //register int n23 asm("r15");
    //DBG_LOG("at post-init: sp 0x%x, lr 0x%x", n21, n22);
    
    //int res;
    //res = sms_mpu_initialize();
    //if(res) {
        //DBG_LOG("Could not initialize MPU!");
        //while(1) {}
    //}
    //sms_sensors_interrupt_toggle(true, false);
    //while(1) {}
    
    /* Goto sleep
     * ---------- */
    sms_ble_power_down();

    
	while(true)
	{
		/* BLE Event task */
		ble_event_task(BLE_EVENT_TIMEOUT);
		
		/* Write application task */
        if(sms_current_interrupt.int_on)
        {
            if(ulp_active) {
                //DBG_LOG_DEV("[main]\t\t\t\tWaking up...");
                acquire_sleep_lock();
                //DBG_LOG_CONT_DEV(" done!");
            }                
            //ulp_ready = false;
            //DBG_LOG_DEV("[main]\t\t\t\tDisabling button int...");
            sms_button_toggle_interrupt(SMS_BTN_INT_DISABLE, SMS_BTN_INT_DISABLE);
            //DBG_LOG_CONT_DEV(" done!");
            switch(sms_current_interrupt.source)
            {
                case INT_NONE:
                //sms_monitor_states("NONE");
                DBG_LOG_DEV("...NO SOURCE!!");
                //sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE);
                //if(ulp_ready) {
                    //DBG_LOG_DEV("[main]\t\t\tULP...");
                    //release_sleep_lock();
                //}
                //if((timer1_current_mode == TIMER1_MODE_NONE) && (timer2_current_mode == TIMER2_MODE_NONE)) release_sleep_lock();
                break;
                
                case INT_BTN0:
                //sms_monitor_states("INT_BTN0");
                DBG_LOG_DEV("...BTN0");
                if((sms_working_mode == SMS_MODE_BUTTON_MPU) || (sms_working_mode == SMS_MODE_COMPLETE) || (sms_working_mode == SMS_MODE_BUTTON_SOLO) || (sms_working_mode == SMS_MODE_BUTTON_PRESSURE)) {
                    //if(ble_current_state == BLE_STATE_PAIRED) {
                    //if(sensors_active) {
                        //DBG_LOG_DEV("[main]\t\t\t\tDisabling sensor int...");
                        //sms_sensors_toggle_interrupt(SMS_EXTINT_DISABLE);
                        //DBG_LOG_CONT_DEV(" done!");
                    //}                        
                    //sms_button_toggle_interrupt(SMS_BTN_INT_DISABLE, SMS_BTN_INT_DISABLE);
                    if(sms_button_fn(SMS_BTN_0) < 0) {
                        DBG_LOG("[main]\t\t\t\tError in sms_button_fn()!");
                    }
                }                    
                break;
                
                case INT_BTN1:
                //sms_monitor_states("INT_BTN1");
                DBG_LOG_DEV("...BTN1");
                if((sms_working_mode == SMS_MODE_BUTTON_MPU) || (sms_working_mode == SMS_MODE_COMPLETE) || (sms_working_mode == SMS_MODE_BUTTON_SOLO) || (sms_working_mode == SMS_MODE_BUTTON_PRESSURE)) {
                    //if(ble_current_state == BLE_STATE_PAIRED)
                    //if(sensors_active) {
                        //DBG_LOG_DEV("[main]\t\t\t\tDisabling sensor int...");
                        //sms_sensors_toggle_interrupt(SMS_EXTINT_DISABLE);
                        //DBG_LOG_CONT_DEV(" done!");
                    //}                        
                    //sms_button_toggle_interrupt(SMS_BTN_INT_DISABLE, SMS_BTN_INT_DISABLE);
                    if(sms_button_fn(SMS_BTN_1) < 0) {
                        DBG_LOG("[main]\t\t\t\tError in sms_button_fn()!");
                    }
                }                    
                break;
                
                case INT_MPU_DRDY:
                //sms_monitor_states("INT_IMU_DRDY");
                //DBG_LOG_DEV("...MPU_DRDY");
                if((sms_working_mode == SMS_MODE_BUTTON_MPU) || (sms_working_mode == SMS_MODE_COMPLETE) || (sms_working_mode == SMS_MODE_MPU_SOLO) || (sms_working_mode == SMS_MODE_MPU_PRESSURE)) {
					//if(ble_current_state == BLE_STATE_PAIRED) {
						gpio_pin_set_output_level(DBG_PIN_1, DBG_PIN_HIGH);
						//sms_sensors_interrupt_toggle(false, false);
		                sms_mpu_poll_data();
						//sms_sensors_interrupt_toggle(true, true);
						gpio_pin_set_output_level(DBG_PIN_2, DBG_PIN_LOW);
					//}
                }              
                break;
                
                case INT_AON_TIMER:
                //DBG_LOG_DEV("...AON_TIMER");
                if((sms_working_mode == SMS_MODE_BUTTON_PRESSURE) || (sms_working_mode == SMS_MODE_COMPLETE) || (sms_working_mode == SMS_MODE_PRESSURE_SOLO) || (sms_working_mode == SMS_MODE_MPU_PRESSURE)) {
                    if(ble_current_state == BLE_STATE_PAIRED) {
						gpio_pin_set_output_level(DBG_PIN_2, DBG_PIN_HIGH);
						//sms_sensors_interrupt_toggle(false, false);
                        sms_pressure_poll_data();
						//sms_sensors_interrupt_toggle(true, true);
						gpio_pin_set_output_level(DBG_PIN_2, DBG_PIN_LOW);
                    }
                    else if(ble_current_state == BLE_STATE_INDICATING) {
                        //DBG_LOG_DEV("[main]\t\t\t\tAON timer ready while indicating... skipping");
                    }
                    else {
                        //sms_timer_aon_disable();
                        //sms_ble_power_down();
                    }                        
                }                    
                break;
                
                case INT_DUALTIMER1:
                //sms_monitor_states("INT_DUALTIMER1");
                DBG_LOG_DEV("...DUALTIMER1");
                sms_dualtimer_stop(DUALTIMER_TIMER1);
                sms_dualtimer1_fn();
                break;
                
                case INT_DUALTIMER2:
                //sms_monitor_states("INT_DUALTIMER2");
                DBG_LOG_DEV("...DUALTIMER2");
                sms_dualtimer_stop(DUALTIMER_TIMER2);
                sms_dualtimer2_fn();
                break;
                
                default:
                DBG_LOG_DEV("...??");
                //sms_monitor_states("ERROR!!");
                break;
            }
            
            //DBG_LOG_DEV("[main]\t\t\t\tEnabling button int...");
            sms_button_toggle_interrupt(SMS_BTN_INT_ENABLE, SMS_BTN_INT_ENABLE);
            //DBG_LOG_CONT_DEV(" done!");
            sms_current_interrupt.int_on = false;
            sms_current_interrupt.source = INT_NONE;
        }
		
		if(ready_to_send[RTS_BUTTON_POS]) {
			DBG_LOG_DEV("[main]\t\t\t\tRTS button...");
			ready_to_send[RTS_BUTTON_POS] = false;
		}
		if(ready_to_send[RTS_PRESSURE_POS]) {
			//DBG_LOG_DEV("[main]\t\t\t\tRTS pressure...");
			//sms_ble_send_characteristic(BLE_CHAR_PRESS);
			ready_to_send[RTS_PRESSURE_POS] = false;
		}
		if(ready_to_send[RTS_MPU_POS]) {
			//DBG_LOG_DEV("[main]\t\t\t\tRTS mpu...");
			sms_ble_send_characteristic(BLE_CHAR_MPU);
			ready_to_send[RTS_MPU_POS] = false;
		}
        
        
        if(ulp_ready) {
            //DBG_LOG_DEV("[main]\t\t\t\tULP...");
            ulp_active = true;
            //release_sleep_lock();
            //DBG_LOG_CONT_DEV(" zzzz");
            //DBG_LOG_CONT_DEV(" !!");
        }            
        else {
            ulp_active = false;
        }
    }
}

