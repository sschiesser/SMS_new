/*
 * sms_i2c.c
 *
 * Created: 05.10.2016 17:22:36
 *  Author: Sébastien Schiesser
 */ 

#include <stdlib.h>
#include "sms_i2c.h"


void i2c_master_configure(void)
{
    DBG_LOG_DEV("[i2c_master_configure]  configuring I2C master...");

    i2c_wpacket.data = malloc(I2C_DATA_LENGTH * sizeof(uint8_t));
    i2c_rpacket.data = malloc(I2C_DATA_LENGTH * sizeof(uint8_t));
    for(uint8_t i = 0; i < I2C_DATA_LENGTH; i++) {
        i2c_wpacket.data[i] = 0;
        i2c_rpacket.data[i] = 0;
    }

    struct i2c_master_config config_i2c_master;
    i2c_master_get_config_defaults(&config_i2c_master);
    config_i2c_master.clock_source = I2C_MASTER_MPU9250_CLK_SRC;
    config_i2c_master.clock_divider = I2C_MASTER_MPU9250_CLK_DIV;
    config_i2c_master.pin_number_pad0 = I2C_MASTER_MPU9250_PIN_SDA;
    config_i2c_master.pin_number_pad1 = I2C_MASTER_MPU9250_PIN_SCL;
    config_i2c_master.pinmux_sel_pad0 = I2C_MASTER_MPU9250_MUX_SDA;
    config_i2c_master.pinmux_sel_pad1 = I2C_MASTER_MPU9250_MUX_SCL;
    while(i2c_master_init(&i2c_master_mpu9250_instance,I2C_MASTER_MPU9250_PORT, &config_i2c_master) != STATUS_OK);

    i2c_enable(i2c_master_mpu9250_instance.hw);
}
