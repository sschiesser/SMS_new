/*
 * sms_sensors.c
 *
 * Created: 22.11.2016 08:22:39
 *  Author: Sébastien Schiesser
 */ 

#include "sms_peripheral1.h"

/* Sensors-related functions */
void sms_sensors_enable_callback(bool imu_cb, bool press_cb) {
    /* IMU --> IMU_DRDY */
    if(imu_cb) {
        sms_imu_enable_callback();
    }
    else {
        sms_imu_disable_callback();
    }
    
    /* Pressure --> AON_SLEEP_TIMER
     * Note: Since there is no direct mechanism to simply enable and disable
     *       the AON sleep timer interruption, we have to initialize it each
     *       time (and it starts running) and register the corresponding
     *       callback (and it enables the interrupt)
     */
    if(press_cb) {
        sms_timer_aon_register_callback();
    }
    else {
        sms_timer_aon_unregister_callback();
    }
}
    

void sms_sensors_switch(bool mpu_en, bool press_en)
{
    /* IMU */
    if(mpu_en) {
        if(sms_imu_startup()) {
	        DBG_LOG("[sms_sensors_switch]\t\tCouldn't start IMU");
			dualtimer_disable(timer1_instance.id);
			imu_device.state = IMU_STATE_OFF;
			imu_device.config.init_ok = false;
        }
        else {
			/* */
			struct dualtimer_config config;
			config.timer1.load_value = 0xffffffff;
			config.timer1.interrup_enable = false;
			config.timer1.counter_mode = DUALTIMER_FREE_RUNNING_MODE;
			dualtimer_init(&config);
			dualtimer_disable(timer2_instance.id);
			dualtimer_enable(timer1_instance.id);
			imu_device.state = IMU_STATE_ON;
	        imu_device.config.init_ok = true;
        }
    }
    else {
		dualtimer_disable(timer1_instance.id);
        imu_device.state = IMU_STATE_OFF;
		imu_device.config.init_ok = false;
		// switch off VCC pin to save current...
    }
    
    /* Pressure */
    if(press_en) {                
        if(sms_pressure_startup()) {
			DBG_LOG("[sms_sensors_switch]\t\t\tCouldn't start pressure sensor");
			sms_timer_aon_disable();
			pressure_device.state = PRESSURE_STATE_OFF;
			pressure_device.config.init_ok = false;
		}
		else {
			sms_timer_aon_init(SMS_PRESSURE_CONVERT_MS, AON_SLEEP_TIMER_RELOAD_MODE);
			pressure_device.state = PRESSURE_STATE_ON;
			pressure_device.config.init_ok = true;
		}
    }
    else {
		sms_timer_aon_disable();
		pressure_device.state = PRESSURE_STATE_OFF;
		pressure_device.config.init_ok = false;
		// switch off VCC pin to save current...
    }
	
	/* Set up SMS working mode & callbacks */
	if((imu_device.state == IMU_STATE_ON) && (pressure_device.state == PRESSURE_STATE_ON)) {
		sms_working_mode = SMS_MODE_COMPLETE;
		sms_sensors_enable_callback(true, true);
	}
	else if(imu_device.state == IMU_STATE_ON) {
		sms_working_mode = SMS_MODE_BUTTON_IMU;
		sms_sensors_enable_callback(true, false);
	}
	else if(pressure_device.state == PRESSURE_STATE_ON) {
		sms_working_mode = SMS_MODE_BUTTON_PRESSURE;
		sms_sensors_enable_callback(false, true);
	}
	else {
		sms_working_mode = SMS_MODE_BUTTON_SOLO;
		sms_sensors_enable_callback(false, false);
	}
	
	DBG_LOG_DEV("[sms_sensors_switch]\t\tSMS working mode: %d", sms_working_mode);
}
