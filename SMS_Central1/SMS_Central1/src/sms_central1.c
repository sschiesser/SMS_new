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

/* timer callback function */
static void timer_callback_fn(void)
{
    DBG_LOG_DEV("[timer_callback_fn]");
	/* Add timer callback functionality here */
}

static void button_cb(void)
{
    DBG_LOG_DEV("[button_cb]");
	/* Add button callback functionality here */
}

/* AT_BLE_SCAN_REPORT (#2) */
static at_ble_status_t sms_central_scan_report_fn(void *params)
{
    DBG_LOG_DEV("[sms_central_scan_report_fn]");
	return AT_BLE_SUCCESS;
}

/* AT_BLE_CONNECTED (#5) */
static at_ble_status_t sms_central_connected_fn(void *params)
{
    DBG_LOG_DEV("[sms_central_connected_fn]");
    return AT_BLE_SUCCESS;
}

/* AT_BLE_DISCONNECTED (#7) */
static at_ble_status_t sms_central_disconnected_fn(void *params)
{
    DBG_LOG_DEV("[sms_central_disconnected_fn]");
    return AT_BLE_SUCCESS;
}

/* AT_BLE_PAIR_DONE (#9) */
static at_ble_status_t sms_central_pair_done_fn(void *params)
{
    DBG_LOG_DEV("[sms_central_pair_done_fn]");
    return AT_BLE_SUCCESS;
}

/* AT_BLE_ENCRYPTION_STATUS_CHANGED (#14) */
static at_ble_status_t sms_central_encryption_status_changed_fn(void *params)
{
    DBG_LOG_DEV("[sms_central_encryption_status_changed_fn]");
    return AT_BLE_SUCCESS;
}

static const ble_event_callback_t sms_central_gap_cb[] = {
    NULL, // AT_BLE_UNDEFINED_EVENT
    NULL, // AT_BLE_SCAN_INFO
    sms_central_scan_report_fn,
    NULL, // AT_BLE_ADV_REPORT
    NULL, // AT_BLE_RAND_ADDR_CHANGED
    sms_central_connected_fn, // AT_BLE_CONNECTED
    sms_central_disconnected_fn, // AT_BLE_DISCONNECTED
    NULL, // AT_BLE_CONN_PARAM_UPDATE_DONE
    NULL, // AT_BLE_CONN_PARAM_UPDATE_REQUEST
    sms_central_pair_done_fn, // AT_BLE_PAIR_DONE
    NULL, // AT_BLE_PAIR_REQUEST
    NULL, // AT_BLE_SLAVE_SEC_REQUEST
    NULL, // AT_BLE_PAIR_KEY_REQUEST
    NULL, // AT_BLE_ENCRYPTION_REQUEST
    sms_central_encryption_status_changed_fn, // AT_BLE_ENCRYPTION_STATUS_CHANGED
    NULL, // AT_BLE_RESOLV_RAND_ADDR_STATUS
    NULL, // AT_BLE_SIGN_COUNTERS_IND
    NULL, // AT_BLE_PEER_ATT_INFO_IND
    NULL // AT_BLE_CON_CHANNEL_MAP_IND
};

int main(void)
{
	platform_driver_init();
	acquire_sleep_lock();

	/* Initialize serial console */
	serial_console_init();
	
	/* Hardware timer */
	hw_timer_init();
	
	/* button initialization */
	gpio_init();
	button_init();
	button_register_callback(button_cb);
	
	hw_timer_register_callback(timer_callback_fn);

	DBG_LOG("Initializing BLE Application");
	
	/* initialize the BLE chip  and Set the Device Address */
	ble_device_init(NULL);
	
    pxp_monitor_init(NULL);
    
    register_hw_timer_start_func_cb((hw_timer_start_func_cb_t)hw_timer_start);
    register_hw_timer_start_func_cb(hw_timer_stop);
    
	gap_dev_scan();
	
	while(true)
	{
		/* BLE Event task */
		ble_event_task(BLE_EVENT_TIMEOUT);
		
		/* Write application task */
	}

}

