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

static inline void get_ms(uint32_t *count)
{
    static uint32_t old_val = 0;
    volatile uint32_t val = (uint32_t)((0xffffffff - dualtimer_get_value(DUALTIMER_TIMER2)) / (uint32_t)26000);
    volatile uint32_t delta;
    if(val >= old_val) delta = val - old_val;
    else delta = (uint32_t)(0xffffffff/(uint32_t)26000) - old_val + val;
    //DBG_LOG("old_val %lu, val %lu, delta %lu", old_val, val, delta);
    old_val = val;
    count[0] = delta;
}

void configure_i2c_master(void)
{
    i2c_wpacket.data = malloc(16 * sizeof(uint8_t));
    i2c_rpacket.data = malloc(16 * sizeof(uint8_t));
    
    /* Initialize config structure and software module. */
    struct i2c_master_config config_i2c_master;
    i2c_master_get_config_defaults(&config_i2c_master);
    /* 26 MHz / 65 = 400 kHz */
    config_i2c_master.clock_source = I2C_CLK_INPUT_0;
    config_i2c_master.clock_divider = 65;
    /* 3 MHz / 30 = 100 kHz */
    //config_i2c_master.clock_source = I2C_CLK_INPUT_3;
    //config_i2c_master.clock_divider = 30;
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
    config_dualtimer.timer2.load_value = 0xffffffff;
    config_dualtimer.timer2.interrup_enable = false;
    
    dualtimer_init(&config_dualtimer);
    dualtimer_disable(DUALTIMER_TIMER1);
    dualtimer_disable(DUALTIMER_TIMER2);
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

void imu_poll_data(void)
{
    static unsigned long sensor_timestamp;
    short gyro[3], accel_short[3], sensors;
    unsigned char more;
    //long accel[3];
    long quat[4];
    //long temperature;
    dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
    DBG_LOG("FIFO: %d %d %d, %d %d %d, %ld %ld %ld %ld", gyro[0], gyro[1], gyro[2], accel_short[0], accel_short[1], accel_short[2], quat[0], quat[1], quat[2], quat[3]);
}

int main(void)
{
    inv_error_t result;
    unsigned char accel_fsr = 0;
    unsigned char new_temp = 0;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp = 0;
    unsigned char new_compass = 0;
    unsigned short compass_fsr;
    
    platform_driver_init();
    gpio_init();
    acquire_sleep_lock();
    serial_console_init();
    ble_device_init(NULL);

    init_dualtimer();
    delay_init();

    configure_imu_gpio();
    configure_i2c_master();
    
    struct int_param_s int_param;
    int_param.cb = (void*)interrupt_cb;
    int_param.pin = PIN_AO_GPIO_2;
    result = mpu_init(&int_param);
    if(result) {
        DBG_LOG_DEV("Could not initialize MPU!");
        system_global_reset();
    }
    
    /* If you're not using an MPU9150 AND you're not using DMP features, this
    * function will place all slaves on the primary bus.
    * mpu_set_bypass(1);
    */
    result = inv_init_mpl();
    if(result) {
        DBG_LOG_DEV("Could not initialize MPL.");
        system_global_reset();
    }
    
    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    
    /* The MPL expects compass data at a constant rate (matching the rate
    * passed to inv_set_compass_sample_rate). If this is an issue for your
    * application, call this function, and the MPL will depend on the
    * timestamps passed to inv_build_compass instead.
    *
    * inv_9x_fusion_use_timestamps(1);
    */

    /* Update gyro biases when not in motion.
    * WARNING: These algorithms are mutually exclusive.
    */
    inv_enable_fast_nomot();
    /* inv_enable_motion_no_motion(); */
    /* inv_set_no_motion_time(1000); */

    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();

    /* This algorithm updates the accel biases when in motion. A more accurate
    * bias measurement can be made when running the self-test (see case 't' in
    * handle_input), but this algorithm can be enabled if the self-test can't
    * be executed in your application.
    *
    * inv_enable_in_use_auto_calibration();
    */
    /* Compass calibration algorithms. */
    //inv_enable_vector_compass_cal();
    //inv_enable_magnetic_disturbance();

    /* If you need to estimate your heading before the compass is calibrated,
    * enable this algorithm. It becomes useless after a good figure-eight is
    * detected, so we'll just leave it out to save memory.
    * inv_enable_heading_from_gyro();
    */

    /* Allows use of the MPL APIs in read_from_mpl. */
    //inv_enable_eMPL_outputs();

    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED) {
        while (1) {
            DBG_LOG("Not authorized.\n");
            delay_ms(5000);
        }
    }
    if (result) {
        MPL_LOGE("Could not start the MPL.\n");
        system_global_reset();
    }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
    * Use this function for proper power management.
    */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);

    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    mpu_get_compass_fsr(&compass_fsr);

    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
    /* The compass rate is independent of the gyro and accel rates. As long as
    * inv_set_compass_sample_rate is called with the correct value, the 9-axis
    * fusion algorithm's compass correction gain will work properly.
    */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);

    /* Set chip-to-body orientation matrix.
    * Set hardware units to dps/g's/degrees scaling factor.
    */
    inv_set_gyro_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation), (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation), (long)accel_fsr<<15);
    inv_set_compass_orientation_and_scale(inv_orientation_matrix_to_scalar(compass_pdata.orientation), (long)compass_fsr<<15);

    /* Initialize HAL state variables. */
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    /* Compass reads are handled by scheduler. */
    dualtimer_enable(DUALTIMER_TIMER2);
    get_ms(&timestamp);
    
    if (dmp_load_motion_driver_firmware()) {
        MPL_LOGE("Could not download DMP.\n");
        system_global_reset();
    }
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    inv_set_quat_sample_rate(1000000L / DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;

    //gpio_register_callback(PIN_AO_GPIO_2, interrupt_cb, GPIO_CALLBACK_RISING);
    gpio_enable_callback(PIN_AO_GPIO_2);
    //while(1){}


    uint8_t compass_cnt = 0;
    while (true) {
        ble_event_task(BLE_EVENT_TIMEOUT);

        get_ms(&timestamp);
        DBG_LOG("Timestamp: %ld", timestamp);
        
        if(imu_interrupt) {
            //imu_poll_data();
            compass_cnt++;
            if(compass_cnt > 2) {
                compass_cnt = 0;
                short compass_short[3];
                long compass[3];
                if(!mpu_get_compass_reg(compass_short, NULL)) {
                    compass[0] = (long)compass_short[0];
                    compass[1] = (long)compass_short[1];
                    compass[2] = (long)compass_short[2];
                    DBG_LOG("Compass: %ld %ld %ld", compass[0], compass[1], compass[2]);
                    inv_build_compass(compass, 0, (inv_time_t)NULL);
                    inv_execute_on_data();
                    //long data[9];
                    //int8_t accuracy;
                    //inv_get_sensor_type_quat(data, &accuracy, NULL);
                    //DBG_LOG("Quat: %ld %ld %ld %ld", data[0], data[1], data[2], data[3]);
                }
            }
            imu_interrupt = false;
        }
    }
    //! [main_loop]
}
