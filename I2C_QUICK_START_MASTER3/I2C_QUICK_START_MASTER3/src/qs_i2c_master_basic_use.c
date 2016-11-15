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

#include "include.h"

#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)
#define DMP_FEATURE_SEND_ANY_GYRO   (DMP_FEATURE_SEND_RAW_GYRO | \
DMP_FEATURE_SEND_CAL_GYRO)


/* Init software module. */
volatile bool imu_interrupt = false;

static struct hal_s hal = {0};
unsigned char *mpl_key = (unsigned char *)"eMPL 5.1";
/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};    
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
        1, 0, 0,
    0, 0,-1}
};
#define COMPASS_ENABLED 1

//struct dmp_s {
    //void (*tap_cb)(unsigned char count, unsigned char direction);
    //void (*android_orient_cb)(unsigned char orientation);
    //unsigned short orient;
    //unsigned short feature_mask;
    //unsigned short fifo_rate;
    //unsigned char packet_length;
//};

//static struct dmp_s dmp = {
    //.tap_cb = NULL,
    //.android_orient_cb = NULL,
    //.orient = 0,
    //.feature_mask = 0,
    //.fifo_rate = 0,
    //.packet_length = 0
//};

//! [dev_inst]

void configure_i2c_master(void)
{
    i2c_wpacket.data = malloc(16 * sizeof(uint8_t));
    i2c_rpacket.data = malloc(16 * sizeof(uint8_t));
    
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
    config_dualtimer.timer1.timer_enable = false;
    config_dualtimer.timer2.timer_enable = false;
    
    dualtimer_init(&config_dualtimer);
    //dualtimer_disable(DUALTIMER_TIMER1);
    //dualtimer_disable(DUALTIMER_TIMER2);
}
void interrupt_cb(void)
{
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
    int_param.pin = PIN_AO_GPIO_2;
    mpu_init(&int_param);
}
static void configure_imu(void)
{
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(10);
    
    uint16_t gyro_rate, gyro_fsr;
    uint8_t accel_fsr;
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    
    //inv_set_gyro_sample_rate
    
    hal.dmp_features = (DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO);
    dmp_enable_feature(hal.dmp_features);
}

void imu_poll_data(void)
{
    //st.chip_cfg.dmp_on = 1;
    //short gyro[3], accel_short[3], sensors;
    //unsigned char more;
    //long accel[3], quaternion[4];
    //unsigned long *timestamp;
    //int res;
    //res = dmp_read_fifo(gyro, accel_short, quaternion, &timestamp, &sensors, &more);
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
    configure_imu();
    gpio_register_callback(PIN_AO_GPIO_2, interrupt_cb, GPIO_CALLBACK_RISING);
    gpio_enable_callback(PIN_AO_GPIO_2);
    //while(1){}


	//! [main_loop]
	while (true) {
		ble_event_task(BLE_EVENT_TIMEOUT);
        if(imu_interrupt) {
            DBG_LOG("IMU INTERRUPT!");
            imu_poll_data();
            imu_interrupt = false;
        }            
	}
	//! [main_loop]
}
