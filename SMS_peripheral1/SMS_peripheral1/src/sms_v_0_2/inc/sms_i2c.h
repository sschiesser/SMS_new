/*
 * sms_i2c.h
 *
 * Created: 05.10.2016 17:22:56
 *  Author: Sébastien Schiesser
 */ 


#ifndef SMS_I2C_H_
#define SMS_I2C_H_

/* -------
 * INCLUDE
 * ------- */
#include "sms_peripheral1.h"

/* ------
 * MACROS
 * ------ */
/* I2C master settings for MPU9250 IMU */
#define I2C_MASTER_MPU9250_PORT     I2C0
#define I2C_MASTER_MPU9250_CLK_SRC  I2C_CLK_INPUT_3
#define I2C_MASTER_MPU9250_CLK_DIV  0x08
/* Pins */
#define I2C_MASTER_MPU9250_PIN_SDA  PIN_LP_GPIO_8
#define I2C_MASTER_MPU9250_PIN_SCL  PIN_LP_GPIO_9
/* Mux */
#define I2C_MASTER_MPU9250_MUX_SDA  MUX_LP_GPIO_8_I2C0_SDA
#define I2C_MASTER_MPU9250_MUX_SCL  MUX_LP_GPIO_9_I2C0_SCL

#define I2C_DATA_LENGTH                 (16)
#define I2C_TIMEOUT_CNT                 (1000)

/* ---------
 * VARIABLES
 * --------- */
//uint8_t i2c_wdata[I2C_DATA_LENGTH];
//uint8_t i2c_rdata[I2C_DATA_LENGTH];

struct i2c_master_module i2c_master_mpu9250_instance;
struct i2c_master_packet i2c_wpacket;
struct i2c_master_packet i2c_rpacket;

/* ------------
 * DECLARATIONS
 * ------------ */
void i2c_master_configure(void);

#endif /* SMS_I2C_H_ */