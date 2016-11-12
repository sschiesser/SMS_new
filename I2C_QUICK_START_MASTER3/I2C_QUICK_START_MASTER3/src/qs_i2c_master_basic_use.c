/**
 * \file
 *
 * \brief I2C Master Quick Start Guide for SAMB
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
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
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include "platform.h"
#include "console_serial.h"
#include "ble_manager.h"
#include "delay.h"
#include "inv_mpu.h"
#include "include.h"

/* Init software module. */
//! [dev_inst]
struct i2c_master_module i2c_master_instance;
volatile bool imu_interrupt = false;
//! [dev_inst]

void configure_i2c_master(void)
{
	/* Initialize config structure and software module. */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
    config_i2c_master.clock_source = I2C_CLK_INPUT_0;
    config_i2c_master.clock_divider = 65; // 26 MHz / 65 = 400000
    config_i2c_master.pin_number_pad0 = PIN_LP_GPIO_8;
    config_i2c_master.pin_number_pad1 = PIN_LP_GPIO_9;
    config_i2c_master.pinmux_sel_pad0 = MUX_LP_GPIO_8_I2C0_SDA;
    config_i2c_master.pinmux_sel_pad1 = MUX_LP_GPIO_9_I2C0_SCL;
	/* Initialize and enable device with config, and enable i2c. */
	while(i2c_master_init(&i2c_master_instance, I2C0, &config_i2c_master) != STATUS_OK);
	
	i2c_enable(i2c_master_instance.hw);
}

void init_dualtimer(void)
{
    struct dualtimer_config config_dualtimer;
    dualtimer_get_config_defaults(&config_dualtimer);
    
    config_dualtimer.timer1.load_value = 26000;
    config_dualtimer.timer2.load_value = 26000;
    
    dualtimer_init(&config_dualtimer);
    dualtimer_disable(DUALTIMER_TIMER1);
    dualtimer_disable(DUALTIMER_TIMER2);
}
void interrupt_cb(void)
{
    gpio_disable_callback(PIN_AO_GPIO_2);
    imu_interrupt = true;
    send_plf_int_msg_ind(PIN_AO_GPIO_2, GPIO_CALLBACK_RISING, NULL, 0);
}

static void configure_imu_gpio(void)
{
    struct gpio_config config_gpio_pin;
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_INPUT;
    config_gpio_pin.input_pull = GPIO_PIN_PULL_DOWN;
    config_gpio_pin.aon_wakeup = true;
    gpio_pin_set_config(PIN_AO_GPIO_2, &config_gpio_pin);
}
static void init_imu(void)
{
    struct int_param_s int_param;
    int_param.cb = interrupt_cb;
    int_param.pin = PIN_AO_GPIO_0;
    mpu_init(&int_param);
}

int main(void)
{
	//system_clock_config(CLOCK_RESOURCE_XO_26_MHZ, CLOCK_FREQ_26_MHZ);
 	//! [init]
    platform_driver_init();
    gpio_init();
    acquire_sleep_lock();
    serial_console_init();
    ble_device_init(NULL);

    init_dualtimer();
    delay_init();
	//! [init]
	//! [config]
    configure_imu_gpio();
	configure_i2c_master();
	//! [config]
    
    init_imu();
    while(1){}


	//! [main_loop]
	while (true) {
		ble_event_task(BLE_EVENT_TIMEOUT);
        if(imu_interrupt) {
            DBG_LOG("IMU INTERRUPT!");
            imu_interrupt = false;
        }            
	}
	//! [main_loop]
}
