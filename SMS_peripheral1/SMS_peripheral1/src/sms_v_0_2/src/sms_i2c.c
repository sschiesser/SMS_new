/*
 * sms_i2c.c
 *
 * Created: 05.10.2016 17:22:36
 *  Author: Sébastien Schiesser
 */ 

#include <stdlib.h>
#include "sms_i2c.h"


void sms_i2c_master_configure(void)
{
    i2c_wpacket.data = malloc(I2C_DATA_LENGTH * sizeof(uint8_t));
    i2c_rpacket.data = malloc(I2C_DATA_LENGTH * sizeof(uint8_t));

    struct i2c_master_config config_i2c_master;
    i2c_master_get_config_defaults(&config_i2c_master);
    config_i2c_master.clock_source = SMS_I2C_MASTER_CLK_SRC;
    config_i2c_master.clock_divider = SMS_I2C_MASTER_CLK_DIV;
    config_i2c_master.pin_number_pad0 = SMS_I2C_MASTER_PIN_SDA;
    config_i2c_master.pin_number_pad1 = SMS_I2C_MASTER_PIN_SCL;
    config_i2c_master.pinmux_sel_pad0 = SMS_I2C_MASTER_MUX_SDA;
    config_i2c_master.pinmux_sel_pad1 = SMS_I2C_MASTER_MUX_SCL;
    while(i2c_master_init(&i2c_master_instance, SMS_I2C_MASTER_PORT, &config_i2c_master) != STATUS_OK);

    i2c_enable(i2c_master_instance.hw);
}

int sms_i2c_master_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t data_len, uint8_t const *data)
{
    //DBG_LOG_DEV("i2c writing to 0x%02x at 0x%02x... data: ", slave_addr, reg_addr);
    uint16_t timeout = 0;
    i2c_wpacket.address = (uint8_t)slave_addr;
    i2c_wpacket.data_length = (uint8_t)(data_len + 1);
    i2c_wpacket.data[0] = (uint8_t)reg_addr;
    for(uint8_t i = 0; i < data_len; i++) {
        i2c_wpacket.data[i+1] = (uint8_t)data[i];
        //DBG_LOG_CONT_DEV("0x%02x ", i2c_wpacket.data[i+1]);
    }
    while (i2c_master_write_packet_wait(&i2c_master_instance, &i2c_wpacket) != STATUS_OK) {
        /* Increment timeout counter and check if timed out. */
        if (timeout++ >= I2C_TIMEOUT) {
            return -1;
        }
    }
    return 0;
}

int sms_i2c_master_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t data_len, uint8_t *data)
{
    //DBG_LOG("i2c reading from 0x%02x at 0x%02x... data: ", slave_addr, reg_addr);
    uint16_t timeout;
    i2c_wpacket.address = (uint8_t)slave_addr;
    i2c_wpacket.data_length = 1;
    i2c_wpacket.data[0] = (uint8_t)reg_addr;
    i2c_rpacket.address = (uint8_t)slave_addr;
    i2c_rpacket.data_length = (uint8_t)data_len;
    
    timeout = 0;
    while(i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &i2c_wpacket) != STATUS_OK) {
		//DBG_LOG_DEV("t/o %d", timeout);
        if(timeout++ >= I2C_TIMEOUT) {
            return -1;
        }
    }
    
    timeout = 0;
    while(i2c_master_read_packet_wait(&i2c_master_instance, &i2c_rpacket) != STATUS_OK) {
		//DBG_LOG_DEV("t/o %d", timeout);
        if(timeout++ >= I2C_TIMEOUT) {
            return -1;
        }
    }
    for(uint8_t i = 0; i < data_len; i++) {
        data[i] = i2c_rpacket.data[i];
        //DBG_LOG_CONT(" 0x%02x ", data[i]);
    }
    return 0;
}