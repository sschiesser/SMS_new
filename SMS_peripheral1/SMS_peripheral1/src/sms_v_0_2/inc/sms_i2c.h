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
#define SMS_I2C_MASTER_PORT             (I2C0)
// 26 MHz clock & div = 65 --> 400000
//#define SMS_I2C_MASTER_CLK_SRC          (I2C_CLK_INPUT_0)
//#define SMS_I2C_MASTER_CLK_DIV          (65)
// 3 MHz clock & div = 30 --> 100000
#define SMS_I2C_MASTER_CLK_SRC			(I2C_CLK_INPUT_3)
#define SMS_I2C_MASTER_CLK_DIV			(30)
/* Pins */
#define SMS_I2C_MASTER_PIN_SDA          (PIN_LP_GPIO_8)
#define SMS_I2C_MASTER_PIN_SCL          (PIN_LP_GPIO_9)
/* Mux */
#define SMS_I2C_MASTER_MUX_SDA          (MUX_LP_GPIO_8_I2C0_SDA)
#define SMS_I2C_MASTER_MUX_SCL          (MUX_LP_GPIO_9_I2C0_SCL)

#define I2C_DATA_LENGTH                 (16)
#define I2C_TIMEOUT                     (1000)

/* ---------
 * VARIABLES
 * --------- */
//uint8_t i2c_wdata[I2C_DATA_LENGTH];
//uint8_t i2c_rdata[I2C_DATA_LENGTH];

struct i2c_master_module i2c_master_instance;
struct i2c_master_packet i2c_wpacket;
struct i2c_master_packet i2c_rpacket;

/* ------------
 * DECLARATIONS
 * ------------ */
void sms_i2c_master_configure(void);
int sms_i2c_master_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t data_len, uint8_t const *data);
int sms_i2c_master_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t data_len, uint8_t *data);

#endif /* SMS_I2C_H_ */