/*
 * sms_common.c
 *
 * Created: 14.11.2016 12:50:33
 *  Author: Sébastien Schiesser
 */ 

#include "sms_common.h"

/* General functions */
void sms_monitor_configure_gpio(void)
{
    struct gpio_config config_gpio_pin;
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction  = GPIO_PIN_DIR_OUTPUT;
    if(gpio_pin_set_config(DBG_PIN_1, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG("Problem while setting gpio pin");
    }
    gpio_pin_set_output_level(DBG_PIN_1, DBG_PIN_LOW);
}

void sms_monitor_get_states(const char *label)
{
    DBG_LOG_DEV("%s...\t\tB-prev %d, B-cur %d, BLE 0x%02x, T1 %d, T2 %d, SMS %d", label, button_instance.previous_state, button_instance.current_state, ble_current_state, timer1_current_mode, timer2_current_mode, sms_working_mode);
}



/* Sensors-related functions */
void sms_sensors_interrupt_toggle(bool mpu_int, bool press_int) {
    /* IMU --> IMU_DRDY */
    if(mpu_int) {
        gpio_enable_callback(SMS_MPU_DRDY_PIN);
    }
    else {
        gpio_disable_callback(SMS_MPU_DRDY_PIN);
    }
    
    /* Pressure --> AON_SLEEP_TIMER
     * Note: Since there is no direct mechanism to simply enable and disable
     *       the AON sleep timer interruption, we have to initialize it each
     *       time (and it starts running) and register the corresponding
     *       callback (and it enables the interrupt)
     */
    if(press_int) {
        pressure_device.ms58_device.current_state = MS58_STATE_CONV_PRESSURE;
        pressure_device.state = PRESSURE_STATE_ON;
        sms_timer_aon_init(SMS_PRESSURE_CONVERT_MS, AON_SLEEP_TIMER_RELOAD_MODE);
        sms_timer_aon_register_callback();
        sensors_active = true;
    }
    else {
        pressure_device.ms58_device.current_state = MS58_STATE_READY;
        sms_timer_aon_disable();
        sms_timer_aon_unregister_callback();
        sensors_active = false;
    }
}
    

void sms_sensors_switch(bool mpu_en, bool press_en)
{
    /* IMU */
    if(mpu_en) {
        sms_mpu_startup();
    }
    else {
        //gpio_pin_set_output_level(SMS_IMU_VCC_PIN, false);
    }
    
    /* Pressure */
    if(press_en) {                
        pressure_device.ms58_device.current_state = MS58_STATE_RESETTING;
        pressure_device.ms58_device.reset_done = false;
        pressure_device.ms58_device.init_ok = false;
        sms_pressure_startup();
    }
    else {
        gpio_pin_set_output_level(SMS_PRESSURE_VCC_PIN, false);
    }
}


