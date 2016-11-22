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
#include "sms_central1.h"

int main(void)
{
    /* Initialize platform
     * ------------------- */
	platform_driver_init();
	gpio_init();
	serial_console_init();

    /* Disable ULP
     * ----------- */
	acquire_sleep_lock();

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

    // SPI
    sms_spi_master_configure();

    /* Initialize the BLE module
     * ------------------------- */
	DBG_LOG_DEV("Initializing BLE Application");	
	ble_device_init(NULL);
	
    /* Register callbacks
     * ------------------ */
    // Dualtimer
    sms_dualtimer_register_callback(DUALTIMER_TIMER1, sms_dualtimer1_cb);
    sms_dualtimer_register_callback(DUALTIMER_TIMER2, sms_dualtimer2_cb);

    // Buttons
    sms_button_register_callbacks();
    
    // BLE
    ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GAP_EVENT_TYPE, sms_ble_gap_cb);
    ble_mgr_events_callback_handler(REGISTER_CALL_BACK, BLE_GATT_CLIENT_EVENT_TYPE, sms_ble_gatt_client_cb);
    //register_ble_user_event_cb(sms_plf_event_cb);

    //pxp_monitor_init(NULL);
    
    
	//gap_dev_scan();
	
	while(true)
	{
		/* BLE Event task */
		//ble_event_task(BLE_EVENT_TIMEOUT);
		
		/* Write application task */
        for(uint8_t i = 0; i < SPI_DATA_LENGTH; i++) {
            spi_wdata[i] = (uint8_t)rand();
            DBG_LOG_DEV("rand? %d %d", spi_wdata[i], spi_rdata[i]);
        }
        sms_spi_master_transceive(&spi_master_instance, &spi_slave_instance, spi_wdata, spi_rdata, SPI_DATA_LENGTH);
        
        delay_ms(1000);         
	}

}

