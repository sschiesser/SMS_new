/*
 * sms_sensors.c
 *
 * Created: 22.11.2016 08:22:39
 *  Author: Sébastien Schiesser
 */ 

#include "sms_peripheral1.h"

/* Sensors-related functions */
void sms_sensors_interrupt_toggle(bool mpu_int, bool press_int) {
    /* IMU --> IMU_DRDY */
    if(mpu_int) {
        mpu_device.state = MPU_STATE_ON;
        sms_mpu_enable_callback();
    }
    else {
        mpu_device.state = MPU_STATE_OFF;
        sms_mpu_disable_callback();
    }
    
    /* Pressure --> AON_SLEEP_TIMER
     * Note: Since there is no direct mechanism to simply enable and disable
     *       the AON sleep timer interruption, we have to initialize it each
     *       time (and it starts running) and register the corresponding
     *       callback (and it enables the interrupt)
     */
    if(press_int) {
        pressure_device.hal.current_state = MS58_STATE_CONV_PRESSURE;
        pressure_device.state = PRESSURE_STATE_ON;
        sms_timer_aon_init(SMS_PRESSURE_CONVERT_MS, AON_SLEEP_TIMER_RELOAD_MODE);
        sms_timer_aon_register_callback();
        sensors_active = true;
    }
    else {
        pressure_device.hal.current_state = MS58_STATE_READY;
        sms_timer_aon_disable();
        sms_timer_aon_unregister_callback();
        sensors_active = false;
    }
}
    

void sms_sensors_switch(bool mpu_en, bool press_en)
{
    /* IMU */
    if(mpu_en) {
        //if(sms_mpu_test()) {
            //DBG_LOG_DEV("[sms_sensors_switch]\t\t\tCouldn't initialize MPU");
            //gpio_pin_set_output_level(SMS_MPU_VCC_PIN, false);
        //}
        //else {
            //mpu_device.hal.init_ok = true;
            //sms_sensors_interrupt_toggle(true, false);
        //}
    }
    else {
        gpio_pin_set_output_level(SMS_MPU_VCC_PIN, false);
    }
    
    /* Pressure */
    if(press_en) {                
        pressure_device.hal.current_state = MS58_STATE_RESETTING;
        //pressure_device.hal.reset_done = false;
        //pressure_device.hal.init_ok = false;
        sms_pressure_startup();
    }
    else {
        gpio_pin_set_output_level(SMS_PRESSURE_VCC_PIN, false);
    }
}
