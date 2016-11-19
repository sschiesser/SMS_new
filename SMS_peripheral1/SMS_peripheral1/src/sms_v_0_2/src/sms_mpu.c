/*
* sms_imu.c
*
* Created: 14.06.2016 13:27:35
*  Author: Sébastien Schiesser
*/

#include "sms_peripheral1.h"

//static struct hal_s hal = {0};
    
void sms_mpu_configure_gpio(void)
{
    struct gpio_config config_gpio_pin;

    /* Data ready interrupt from IMU */
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_INPUT;
    config_gpio_pin.input_pull = GPIO_PIN_PULL_DOWN;
    config_gpio_pin.aon_wakeup = true;
    if(gpio_pin_set_config(SMS_MPU_DRDY_PIN, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG("[sms_imu_configure_gpio]\tProblem while setting up IMU DRDY pin");
    }

    /* Pin output to supply IMU */
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_OUTPUT;
    if(!gpio_pin_set_config(SMS_MPU_VCC_PIN, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG("[sms_imu_configure_gpio]\tProblem while setting up IMU VCC pin");
    }
    gpio_pin_set_output_level(SMS_MPU_VCC_PIN, true);
}

/* Register GPIO interrupt callback */
void sms_mpu_register_callbacks(void)
{
    /* MPU-9250 interrupt callback */
    gpio_register_callback(SMS_MPU_DRDY_PIN, sms_mpu_interrupt_callback, GPIO_CALLBACK_RISING);
}

/* Unregister GPIO interrupt callback */
void sms_mpu_unregister_callbacks(void)
{
    gpio_unregister_callback(SMS_MPU_DRDY_PIN, GPIO_CALLBACK_RISING);
}

/* Callback --> send interrupt message to platform */
void sms_mpu_interrupt_callback(void)
{
    sms_current_interrupt.source = INT_MPU_DRDY;
    send_plf_int_msg_ind(SMS_MPU_DRDY_PIN, GPIO_CALLBACK_RISING, NULL, 0);
}

int sms_mpu_initialize(void) {
    int res;
    //unsigned char accel_fsr = 0;
    //unsigned short gyro_rate, gyro_fsr, compass_fsr;
    
    /* Initialize MPU-9250 without interrupt parameter since this has to be set independently */
    DBG_LOG_DEV("Initializing MPU...");
    res = mpu_init(NULL);
    if(res) {
        DBG_LOG_CONT_DEV(" failed!");
        return -1;
    }
    DBG_LOG_DEV("Setting up MPU...");
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(SMS_MPU_SAMPLE_RATE_HZ);
    mpu_set_compass_sample_rate(SMS_MPU_COMPASS_RATE_HZ);
    mpu_get_sample_rate(&mpu_device.hal.sample_rate);
    mpu_get_accel_fsr(&mpu_device.hal.accel_fsr);
    mpu_get_compass_fsr(&mpu_device.hal.compass_fsr);
    
    mpu_device.hal.sensors = (SMS_MPU_ACCEL_ON | SMS_MPU_GYRO_ON | SMS_MPU_COMPASS_ON);
    mpu_device.hal.dmp_features = (DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_ANY_GYRO);
    dmp_enable_feature(mpu_device.hal.dmp_features);
    dmp_set_fifo_rate(SMS_MPU_SAMPLE_RATE_HZ);
    mpu_set_dmp_state(1);
    mpu_device.hal.dmp_on = 1;
    mpu_device.temp_cnt = 0;
    mpu_device.compass_cnt = 0;
    mpu_device.new_compass = false;
    mpu_device.new_temp = false;
    
    return 0;
}

/* Extract available IMU data */
int sms_mpu_poll_data(void)
{
    //DBG_LOG_DEV("[sms_imu_receive_data]\n\r  reading...");
    //st.chip_cfg.dmp_on = 1;
    short sensors;
    unsigned char more;
    unsigned long sensor_timestamp;
    int res;
    
    mpu_device.hal.new_data = 0;
    
    mpu_read_fifo(mpu_device.hal.gyro, mpu_device.hal.accel, &sensor_timestamp, &sensors, &more);
    
    //if(more) {
        //mpu_device.hal.new_data = 1;
    //}
    
    if(mpu_device.temp_cnt++ > SMS_MPU_TEMP_MULTIPLIER) {
        mpu_device.temp_cnt = 0;
        mpu_get_temperature(&mpu_device.hal.temperature, &sensor_timestamp);
        mpu_device.new_temp = true;
    }
    
    if(mpu_device.compass_cnt++ > SMS_MPU_COMPASS_MULTIPLIER) {
        mpu_device.compass_cnt = 0;
        mpu_get_compass_reg(&mpu_device.hal.compass, &sensor_timestamp);
        mpu_device.new_compass = true;   
    }
    sms_ble_send_characteristic(BLE_CHAR_MPU);    

    return 0;
}

void sms_mpu_define_services(void)
{
    at_ble_status_t status;
    uint8_t init_value = 0;
    sms_ble_service_init(BLE_SERV_MPU, &mpu_device.service_handler, &init_value);
    if((status = sms_ble_primary_service_define(&mpu_device.service_handler)) != AT_BLE_SUCCESS) {
        DBG_LOG("[sms_mpu_define_services]\tServices defining failed, reason 0x%x", status);
    }
    else {
        DBG_LOG_DEV("[sms_mpu_define_services]\tServices defined, SMS MPU handle: %d", mpu_device.service_handler.serv_handle);
    }
}