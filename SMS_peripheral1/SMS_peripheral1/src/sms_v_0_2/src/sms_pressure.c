﻿/*
* sms_pressure.c
*
* Created: 07.06.2016 16:34:47
*  Author: Sébastien Schiesser
*/

#include "sms_pressure.h"

void sms_pressure_configure_gpio(void)
{
    struct gpio_config config_gpio_pin;
    gpio_get_config_defaults(&config_gpio_pin);
    config_gpio_pin.direction = GPIO_PIN_DIR_OUTPUT;
    if(gpio_pin_set_config(SMS_PRESSURE_VCC_PIN, &config_gpio_pin) != STATUS_OK) {
        DBG_LOG_DEV("[sms_pressure_configure_gpio]\tproblem while setting up Vcc pin");
    }
    /* Disable power supply by default */
    gpio_pin_set_output_level(SMS_PRESSURE_VCC_PIN, false);
}


void sms_pressure_startup(void)
{
    DBG_LOG_DEV("[sms_pressure_startup]\t\tStarting pressure sensor");
    //gpio_pin_set_output_level(SMS_PRESSURE_VCC_PIN, true); // switch on MS58 pressure sensor
    /* Disable buttons for reset time (~3 ms) to avoid conflict with dualtimer1 */
    sms_button_toggle_interrupt(SMS_BTN_INT_DISABLE, SMS_BTN_INT_DISABLE);
    pressure_device.ms58_device.current_state = MS58_STATE_RESETTING;
    pressure_device.ms58_device.reset_done = false;
    /* Write the reset command to MS58 */
    sms_pressure_ms58_reset();
    timer1_current_mode = TIMER1_MODE_MS58_RESET;
    //ulp_ready = false;
    sms_dualtimer_start(TIMER_UNIT_MS, MS58_RESET_WAIT_MS, DUALTIMER_TIMER1);
}

enum status_code sms_pressure_init(void)
{
    /* Read the PROM values */
    if(sms_pressure_ms58_read_prom() == STATUS_OK) {
        pressure_device.ms58_device.init_ok = true;
        pressure_device.ms58_device.current_state = MS58_STATE_READY;
        /* Enable buttons again */
        //sms_button_toggle_interrupts(SMS_EXTINT_ENABLE);
        return STATUS_OK;
    }
    return STATUS_ERR_IO;
}

void sms_pressure_ms58_reset(void)
{
    //DBG_LOG_DEV("[sms_pressure_ms58_reset]\twriting reset command");
    spi_wdata[0] = MS58_RESET;
    spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 1);
}

enum status_code sms_pressure_ms58_read_prom(void)
{
    //DBG_LOG_DEV("[sms_pressure_ms58_read_prom] reading bytes... ");
    spi_wdata[0] = MS58_PROM_READ_1;
    spi_wdata[1] = 0x00;
    spi_wdata[2] = 0x00;
    spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 3);
    //DBG_LOG_DEV("[sms_pressure_ms58_read_prom] wdata[0]: 0x%02x, rdata[0]: 0x%02x\n\r wdata[1]: 0x%02x, rdata[1]: 0x%02x\n\r wdata[2]: 0x%02x, rdata[2]: 0x%02x", spi_wdata[0], spi_rdata[0], spi_wdata[1], spi_rdata[1], spi_wdata[2], spi_rdata[2]);
    pressure_device.ms58_device.prom_values[1] = (spi_rdata[1] << 8) | (spi_rdata[2]);

    spi_wdata[0] = MS58_PROM_READ_2;
    spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 3);
    //DBG_LOG_DEV("[sms_pressure_ms58_read_prom] wdata[0]: 0x%02x, rdata[0]: 0x%02x\n\r  wdata[1]: 0x%02x, rdata[1]: 0x%02x\n\r  wdata[2]: 0x%02x, rdata[2]: 0x%02x", spi_wdata[0], spi_rdata[0], spi_wdata[1], spi_rdata[1], spi_wdata[2], spi_rdata[2]);
    pressure_device.ms58_device.prom_values[2] = (spi_rdata[1] << 8) | (spi_rdata[2]);

    spi_wdata[0] = MS58_PROM_READ_3;
    spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 3);
    //DBG_LOG_DEV("[sms_pressure_ms58_read_prom] wdata[0]: 0x%02x, rdata[0]: 0x%02x\n\r  wdata[1]: 0x%02x, rdata[1]: 0x%02x\n\r  wdata[2]: 0x%02x, rdata[2]: 0x%02x", spi_wdata[0], spi_rdata[0], spi_wdata[1], spi_rdata[1], spi_wdata[2], spi_rdata[2]);
    pressure_device.ms58_device.prom_values[3] = (spi_rdata[1] << 8) | (spi_rdata[2]);

    spi_wdata[0] = MS58_PROM_READ_4;
    spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 3);
    //DBG_LOG_DEV("[sms_pressure_ms58_read_prom] wdata[0]: 0x%02x, rdata[0]: 0x%02x\n\r  wdata[1]: 0x%02x, rdata[1]: 0x%02x\n\r  wdata[2]: 0x%02x, rdata[2]: 0x%02x", spi_wdata[0], spi_rdata[0], spi_wdata[1], spi_rdata[1], spi_wdata[2], spi_rdata[2]);
    pressure_device.ms58_device.prom_values[4] = (spi_rdata[1] << 8) | (spi_rdata[2]);

    spi_wdata[0] = MS58_PROM_READ_5;
    spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 3);
    //DBG_LOG_DEV("[sms_pressure_ms58_read_prom] wdata[0]: 0x%02x, rdata[0]: 0x%02x\n\r  wdata[1]: 0x%02x, rdata[1]: 0x%02x\n\r  wdata[2]: 0x%02x, rdata[2]: 0x%02x", spi_wdata[0], spi_rdata[0], spi_wdata[1], spi_rdata[1], spi_wdata[2], spi_rdata[2]);
    pressure_device.ms58_device.prom_values[5] = (spi_rdata[1] << 8) | (spi_rdata[2]);

    spi_wdata[0] = MS58_PROM_READ_6;
    spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 3);
    //DBG_LOG_DEV("[sms_pressure_ms58_read_prom] wdata[0]: 0x%02x, rdata[0]: 0x%02x\n\r  wdata[1]: 0x%02x, rdata[1]: 0x%02x\n\r  wdata[2]: 0x%02x, rdata[2]: 0x%02x", spi_wdata[0], spi_rdata[0], spi_wdata[1], spi_rdata[1], spi_wdata[2], spi_rdata[2]);
    pressure_device.ms58_device.prom_values[6] = (spi_rdata[1] << 8) | (spi_rdata[2]);

    spi_wdata[0] = MS58_PROM_READ_7;
    spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 3);
    //DBG_LOG_DEV("[sms_pressure_ms58_read_prom] wdata[0]: 0x%02x, rdata[0]: 0x%02x\n\r  wdata[1]: 0x%02x, rdata[1]: 0x%02x\n\r  wdata[2]: 0x%02x, rdata[2]: 0x%02x", spi_wdata[0], spi_rdata[0], spi_wdata[1], spi_rdata[1], spi_wdata[2], spi_rdata[2]);
    pressure_device.ms58_device.prom_values[7] = (spi_rdata[1] << 8) | (spi_rdata[2]);

    //DBG_LOG_CONT_DEV("done! Results:");
    //for(uint8_t i = 1; i < MS58_PROM_VALUES_MAX; i++) {
        //DBG_LOG_DEV("  C%d -> %d", (i+1), ms58_device.prom_values[i]);
    //}

    return STATUS_OK;
}

void sms_pressure_poll_data(void)
{
    static uint32_t temp = 0x12345678;
    static uint32_t press = 0xfedcba98;
    if(ble_current_state == BLE_STATE_PAIRED) {
        DBG_LOG_DEV("[sms_pressure_poll_data]\tStarting data polling");
        if(sms_pressure_ms58_read_data() != STATUS_OK) {
            DBG_LOG_DEV("[sms_pressure_ms58_poll_data] problem reading ms58 data");
        }
        else {
            ////DBG_LOG_DEV("[sms_pressure_poll_data]\tDone");
            if(pressure_device.ms58_device.data_complete) {
                pressure_device.ms58_device.data_complete = false;
                sms_pressure_ms58_calculate();
                //ms58_device.temperature = ( (ms58_device.temperature >= (int32_t)0xffffffff) ? (ms58_device.temperature = 0) : (ms58_device.temperature + 1) );
                //ms58_device.pressure = ( (ms58_device.pressure < 0) ? (ms58_device.pressure = 0xffffffff) : (ms58_device.pressure - 1) );
                sms_ble_send_characteristic(BLE_CHAR_PRESS);
        }
    }
        //if((timer1_current_mode == TIMER1_MODE_NONE) && (timer2_current_mode == TIMER2_MODE_NONE)) release_sleep_lock();
    }        
}

enum status_code sms_pressure_ms58_read_data(void)
{
    switch(pressure_device.ms58_device.current_state) {
        case MS58_STATE_CONV_PRESSURE:
        //DBG_LOG_DEV("[sms_pressure_ms58_read_data] reading ADC pressure values...");
        spi_wdata[0] = MS58_ADC_READ;
        spi_wdata[1] = MS58_ADC_READ;
        spi_wdata[2] = MS58_ADC_READ;
        spi_wdata[3] = MS58_ADC_READ;
        spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 4);
        pressure_device.ms58_device.adc_values[MS58_TYPE_PRESS] = ((spi_rdata[1] << 16) | (spi_rdata[2] << 8) | (spi_rdata[3]));
        //DBG_LOG_DEV("[sms_pressure_ms58_read_data] D1 -> %ld", ms58_device.adc_values[MS58_TYPE_PRESS]);
        
        //DBG_LOG_DEV("[sms_pressure_ms58_read_data] starting D2 conversion");
        spi_wdata[0] = MS58_CONV_D2_512;
        spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 1);
        pressure_device.ms58_device.current_state = MS58_STATE_CONV_TEMPERATURE;
        break;
        
        case MS58_STATE_CONV_TEMPERATURE:
        //DBG_LOG_DEV("[sms_pressure_ms58_read_data] reading ADC temperature values...");
        spi_wdata[0] = MS58_ADC_READ;
        spi_wdata[1] = MS58_ADC_READ;
        spi_wdata[2] = MS58_ADC_READ;
        spi_wdata[3] = MS58_ADC_READ;
        spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 4);
        pressure_device.ms58_device.adc_values[MS58_TYPE_TEMP] = ((spi_rdata[1] << 16) | (spi_rdata[2] << 8) | (spi_rdata[3]));
        //DBG_LOG_DEV("[sms_pressure_ms58_read_data] D2 -> %ld", ms58_device.adc_values[MS58_TYPE_TEMP]);
        
        //DBG_LOG_DEV("[sms_pressure_ms58_read_data] starting D1 conversion");
        spi_wdata[0] = MS58_CONV_D1_512;
        spi_master_transceive(&spi_master_ms58_instance, &spi_slave_ms58_instance, spi_wdata, spi_rdata, 1);
        pressure_device.ms58_device.current_state = MS58_STATE_CONV_PRESSURE;
        pressure_device.ms58_device.data_complete = true;
        break;
        
        case MS58_STATE_RESETTING:
        case MS58_STATE_READY:
        case MS58_STATE_NONE:
        default:
        break;
    }
    return STATUS_OK;
}

void sms_pressure_ms58_calculate(void)
{
    /***************************************************************************
    * Calculated values...
    * Note: - ms58_prom_values[] are uint16_t
    *       - ms58_adc_values[] are uint32_t
    * Typical values given on the MS5003-01BA datasheets (March 25, 2013) are:
    * -------------------------------------------------------------------------
    * ms58_prom_values[]          |   ms58_adc_values[]
    * - C1 = 40'127 (SENSt1)      |   - D1 = 9'085'466 (Digital pressure)
    * - C2 = 36'924 (OFFt1)       |   - D2 = 8'569'150 (Digital temperature)
    * - C3 = 23'317 (TCS)         |
    * - C4 = 23'282 (TCO)         |
    * - C5 = 33'464 (Tref)        |
    * - C6 = 28'312 (TEMPSENS)    |
    **************************************************************************/
    int32_t deltaT;
    int64_t offset, sensitivity, tv1, tv2, tv3;

    /***************************
    * Temperature calculation *
    ***************************/
    /* dT = D2 - Tref = D2 - C5*2^8 */
    /* tv1: 33464 * 2^8 = 8566784 */
    tv1 = ((int64_t)(pressure_device.ms58_device.prom_values[5]) << 8);
    /* deltaT: 8569150 - 8566784 = 2366 */
    deltaT = (int32_t)((int64_t)pressure_device.ms58_device.adc_values[MS58_TYPE_TEMP] - tv1);

    /* TEMP = 20°C + dT*TEMPSENS = 2000 + dT * C6/2^23 */
    /* tv1: 28312 * 2366 = 66986192 */
    tv1 = ((int64_t)pressure_device.ms58_device.prom_values[6] * (int64_t)deltaT);
    /* tv2: 66986192 / 2^23 = 7(.985376358) */
    tv2 = (tv1 >> 23);
    /* temp: 7 + 2000 = 2007 */
    pressure_device.ms58_device.temperature = (int32_t)(tv2 + 2000);

    /************************
    * Pressure calculation *
    ************************/
    /* OFF = OFFt1 + TCO*dT = C2*2^16 + (C4*dT)/2^7 */
    /* tv1: 36924 * 2^16 = 2419851264 */
    tv1 = ((int64_t)(pressure_device.ms58_device.prom_values[2]) << 16);
    /* tv2: 23282 * 2366 = 55085212 */
    tv2 = ((int64_t)pressure_device.ms58_device.prom_values[4] * (int64_t)deltaT);
    /* tv3: 55085212 / 2^7 = 430353(.21875) */
    tv3 = (tv2 >> 7);
    /* offset: 2419851264 + 430353 = 2420281617 */
    offset = (tv1 + tv3);

    /* SENS = SENSt1 + TCS*dT = C1*2^15 + (C3*dT)/2^8 */
    /* tv1: 40127 * 2^15 = 1314881536 */
    tv1 = ((int64_t)(pressure_device.ms58_device.prom_values[1]) << 15);
    /* tv2: 23317 * 2366 = 55168022 */
    tv2 = ((int64_t)pressure_device.ms58_device.prom_values[3] * (int64_t)deltaT);
    /* tv3: 55168022 / 2^8 = 215500(.0859375) */
    tv3 = (tv2 >> 8);
    /* sensitivity: 1314881536 + 215500 = 1315097036 */
    sensitivity = (tv1 + tv3);

    /* P = D1*SENS - OFF = (D1*SENS/2^21 - OFF)/2^15 */
    /* tv1: (9085466 * 1315097036) / 2^21 = 5697378829(.612148284) */
    tv1 = (((int64_t)pressure_device.ms58_device.adc_values[MS58_TYPE_PRESS] * sensitivity) >> 21);
    /* tv2: 5697378829 - 2420281617 = 3277097212 */
    tv2 = tv1 - offset;
    /* press: 3277097212 / 2^15 = 100009(.070190) */
    pressure_device.ms58_device.pressure = (int32_t)(tv2 >> 15);

    DBG_LOG_DEV("[sms_pressure_ms58_calculate] temperature = %ld  pressure = %ld", pressure_device.ms58_device.temperature, pressure_device.ms58_device.pressure);
}


void sms_pressure_define_services(void)
{
    at_ble_status_t status;
    uint8_t init_value = 0;
    sms_ble_service_init(BLE_SERV_PRESSURE, &pressure_device.service_handler, &init_value);
    if((status = sms_ble_primary_service_define(&pressure_device.service_handler)) != AT_BLE_SUCCESS) {
        DBG_LOG("[sms_pressure_define_services]\tServices defining failed, reason 0x%x", status);
    }
    else {
        DBG_LOG_DEV("[sms_pressure_define_services]\tServices defined, SMS pressure handle: %d", pressure_device.service_handler.serv_handle);
    }
}