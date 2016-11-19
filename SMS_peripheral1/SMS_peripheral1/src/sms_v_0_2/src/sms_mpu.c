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
        //DBG_LOG_DEV("ERROR! returned: %d", res);
        ///* -1 returned in case of a.o. i2c communication error */
        //if(res == -1) {
            //DBG_LOG_DEV("[sms_imu_poll_data]  comm_error = true");
        //}
        ///* -2 returned in case of fifo overflow */
        //if(res == -2) {
            //mpu_device.hal.dmp_features = (DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO);
            //if(dmp_enable_feature(mpu_device.hal.dmp_features) != 0) return -1;
            //if(mpu_set_dmp_state(1) != 0) return -1;
        //}
    //}
    //else {
        //sms_imu_char_values[0] = (uint8_t)(accel_short[0] & 0xff);
        //sms_imu_char_values[1] = (uint8_t)((accel_short[0] >> 8) & 0xff);
        //sms_imu_char_values[2] = (uint8_t)(accel_short[1] & 0xff);
        //sms_imu_char_values[3] = (uint8_t)((accel_short[1] >> 8) & 0xff);
        //sms_imu_char_values[4] = (uint8_t)(accel_short[2] & 0xff);
        //sms_imu_char_values[5] = (uint8_t)((accel_short[2] >> 8) & 0xff);
        //sms_imu_char_values[6] = (uint8_t)(gyro[0] & 0xff);
        //sms_imu_char_values[7] = (uint8_t)((gyro[0] >> 8) & 0xff);
        //sms_imu_char_values[8] = (uint8_t)(gyro[1] & 0xff);
        //sms_imu_char_values[9] = (uint8_t)((gyro[1] >> 8) & 0xff);
        //sms_imu_char_values[10] = (uint8_t)(gyro[2] & 0xff);
        //sms_imu_char_values[11] = (uint8_t)((gyro[2] >> 8) & 0xff);
        //for(uint8_t i = 0; i < 12; i++) {
        //DBG_LOG_DEV("[sms_imu_receive_data]  value[%d]: 0x%02x", i, sms_imu_char_values[i]);
        //}
        //at_ble_status_t status = at_ble_characteristic_value_set(sms_imu_service_handler.serv_chars.char_val_handle, &sms_imu_char_values, (12 * sizeof(uint8_t)));
        //if(status != AT_BLE_SUCCESS) {
            //DBG_LOG_DEV("[sms_imu_receive_data]  updating the characteristic failed, reason %d", status);
        //}
        //else {
            //status = at_ble_notification_send(sms_connection_handle, sms_imu_service_handler.serv_chars.char_val_handle);
            //if(status != AT_BLE_SUCCESS) {
                //DBG_LOG_DEV("[sms_imu_receive_data]  sending notification failed");
            //}
            //else {
                //DBG_LOG_DEV("[sms_imu_receive_data]  sending notification...");
            //}
        //}
    //}
    //gpio_enable_callback(SMS_IMU_INTERRUPT_PIN);


    return 0;
}

//void sms_imu_service_init(gatt_service_handler_t *sms_imu_serv, uint8_t *sms_imu_value)
//{
    ////SMS button service characteristic
    //sms_imu_serv->serv_handle = 0;
    //sms_imu_serv->serv_uuid.type = AT_BLE_UUID_128;
    //sms_imu_serv->serv_uuid.uuid[0] = (uint8_t) ((SMS_IMU_SERVICE_UUID_1) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[1] = (uint8_t) ((SMS_IMU_SERVICE_UUID_1 >> 8) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[2] = (uint8_t) ((SMS_IMU_SERVICE_UUID_1 >> 16) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[3] = (uint8_t) ((SMS_IMU_SERVICE_UUID_1 >> 24) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[4] = (uint8_t) ((SMS_IMU_SERVICE_UUID_2) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[5] = (uint8_t) ((SMS_IMU_SERVICE_UUID_2 >> 8) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[6] = (uint8_t) ((SMS_IMU_SERVICE_UUID_2 >> 16) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[7] = (uint8_t) ((SMS_IMU_SERVICE_UUID_2 >> 24) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[8] = (uint8_t) ((SMS_IMU_SERVICE_UUID_3) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[9] = (uint8_t) ((SMS_IMU_SERVICE_UUID_3 >> 8) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[10] = (uint8_t) ((SMS_IMU_SERVICE_UUID_3 >> 16) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[11] = (uint8_t) ((SMS_IMU_SERVICE_UUID_3 >> 24) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[12] = (uint8_t) ((SMS_IMU_SERVICE_UUID_4) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[13] = (uint8_t) ((SMS_IMU_SERVICE_UUID_4 >> 8) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[14] = (uint8_t) ((SMS_IMU_SERVICE_UUID_4 >> 16) & 0xFF);
    //sms_imu_serv->serv_uuid.uuid[15] = (uint8_t) ((SMS_IMU_SERVICE_UUID_4 >> 24) & 0xFF);
    //
    //sms_imu_serv->serv_chars.properties = (AT_BLE_CHAR_READ | AT_BLE_CHAR_NOTIFY); /* Properties */
    //sms_imu_serv->serv_chars.init_value = sms_imu_value;             /* value */
    //sms_imu_serv->serv_chars.value_init_len = 12 * sizeof(uint8_t);
    //sms_imu_serv->serv_chars.value_max_len = 12 * sizeof(uint8_t);
    //sms_imu_serv->serv_chars.value_permissions = (AT_BLE_ATTR_READABLE_NO_AUTHN_NO_AUTHR | AT_BLE_ATTR_WRITABLE_NO_AUTHN_NO_AUTHR);   /* permissions */
    //sms_imu_serv->serv_chars.user_desc = NULL;           /* user defined name */
    //sms_imu_serv->serv_chars.user_desc_len = 0;
    //sms_imu_serv->serv_chars.user_desc_max_len = 0;
    //sms_imu_serv->serv_chars.user_desc_permissions = AT_BLE_ATTR_NO_PERMISSIONS;             /*user description permissions*/
    //sms_imu_serv->serv_chars.client_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*client config permissions*/
    //sms_imu_serv->serv_chars.server_config_permissions = AT_BLE_ATTR_NO_PERMISSIONS;         /*server config permissions*/
    //sms_imu_serv->serv_chars.user_desc_handle = 0;             /*user description handles*/
    //sms_imu_serv->serv_chars.client_config_handle = 0;         /*client config handles*/
    //sms_imu_serv->serv_chars.server_config_handle = 0;         /*server config handles*/
    //
    ////presentation_format.format = AT_BLE_PRES_FORMAT_UINT8;
    ////presentation_format.exponent = SMS_BUTTON_PRESENTATION_FORMAT_EXPONENT;
    ////presentation_format.unit = (uint8_t) SMS_BUTTON_PRESENTATION_FORMAT_UNIT;
    ////presentation_format.unit = (uint8_t) (SMS_BUTTON_PRESENTATION_FORMAT_UNIT >> 8);
    ////presentation_format.name_space = SMS_BUTTON_PRESENTATION_FORMAT_NAMESPACE;
    ////presentation_format.description = (uint8_t) SMS_BUTTON_PRESENTATION_FORMAT_DESCRIPTOR;
    ////presentation_format.description = (uint8_t) (SMS_BUTTON_PRESENTATION_FORMAT_DESCRIPTOR >> 8);
    //
    //sms_imu_serv->serv_chars.presentation_format = NULL;       /* presentation format */
//
    ////DBG_LOG("[sms_imu_service_init]\n\r  setting primary service\r\n- uuid: 0x%02x\r\n- handle: 0x%02x\r\n- char uuid: 0x%02x%02x\r\n- char init value: %d", (unsigned int)sms_imu_serv->serv_uuid.uuid, sms_imu_serv->serv_handle, sms_imu_serv->serv_chars.uuid.uuid[1], sms_imu_serv->serv_chars.uuid.uuid[0], sms_imu_serv->serv_chars.value_init_len);
//}
//
//at_ble_status_t sms_imu_primary_service_define(gatt_service_handler_t *sms_service)
//{
    ////DBG_LOG("[sms_imu_primary_service_define]\n\r  defining primary service\r\n- uuid: 0x%02x\r\n- handle: 0x%02x\r\n- char uuid: 0x%02x%02x\r\n- char init value: %d",
    ////(unsigned int)sms_service->serv_uuid.uuid,
    ////sms_service->serv_handle,
    ////sms_service->serv_chars.uuid.uuid[1], sms_service->serv_chars.uuid.uuid[0],
    ////sms_service->serv_chars.value_init_len);
    //return(at_ble_primary_service_define(&sms_service->serv_uuid, &sms_service->serv_handle, NULL, 0, &sms_service->serv_chars, 1));
//}
