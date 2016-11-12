/*
 * sms_spi.h
 *
 * Created: 14.06.2016 13:35:24
 *  Author: Sébastien Schiesser
 */ 


#ifndef SMS_SPI_H_
#define SMS_SPI_H_

/* -------
 * INCLUDE
 * ------- */
#include "sms_peripheral1.h"

/* ------
 * MACROS
 * ------ */
/* SPI master settings for MS58 pressure sensor */
//#define SPI_MASTER_MS58_ENABLE      true
#define SPI_MASTER_MS58_PORT        SPI1
#define SPI_MASTER_MS58_MODE        SPI_TRANSFER_MODE_0
#define SPI_MASTER_MS58_CLOCK_DIV   104
/* Pins */
#define SPI_MASTER_MS58_PIN_SCK     PIN_LP_GPIO_17
#define SPI_MASTER_MS58_PIN_MOSI    PIN_LP_GPIO_19
#define SPI_MASTER_MS58_PIN_SSN     PINMUX_UNUSED
#define SPI_MASTER_MS58_MISO        PIN_LP_GPIO_18
/* Mux */
#define SPI_MASTER_MS58_MUX_SCK     MUX_LP_GPIO_17_SPI1_SCK
#define SPI_MASTER_MS58_MUX_MOSI    MUX_LP_GPIO_19_SPI1_MOSI
#define SPI_MASTER_MS58_MUX_SSN     PINMUX_UNUSED
#define SPI_MASTER_MS58_MUX_MISO    MUX_LP_GPIO_18_SPI1_MISO

///* SPI master settings for MPU-9250 IMU */
//#define SPI_SLAVE_MPU9250_SS_PIN    PIN_LP_GPIO_5
//#define SPI_MASTER_MPU9250_ENABLE   true
//#define SPI_MASTER_MPU9250_PORT     SPI0
//#define SPI_MASTER_MPU9250_MODE     SPI_TRANSFER_MODE_3
//#define SPI_MASTER_MPU9250_CLOCK_DIV    25 // MPU-9250 maximum SPI clock: 1 MHz (-> div = 25)
///* Pins */
//#define SPI_MASTER_MPU9250_PIN_SCK  PIN_LP_GPIO_10
//#define SPI_MASTER_MPU9250_PIN_MOSI PIN_LP_GPIO_11
//#define SPI_MASTER_MPU9250_PIN_SSN  PINMUX_UNUSED
//#define SPI_MASTER_MPU9250_PIN_MISO PIN_LP_GPIO_13
///* Mux */
//#define SPI_MASTER_MPU9250_MUX_SCK  MUX_LP_GPIO_10_SPI0_SCK
//#define SPI_MASTER_MPU9250_MUX_MOSI MUX_LP_GPIO_11_SPI0_MOSI
//#define SPI_MASTER_MPU9250_MUX_SSN  PINMUX_UNUSED
//#define SPI_MASTER_MPU9250_MUX_MISO MUX_LP_GPIO_13_SPI0_MISO

#define SPI_DATA_LENGTH 16

/* ---------
 * VARIABLES
 * --------- */
 /* SPI device instances */
struct spi_module spi_master_ms58_instance;
struct spi_slave_inst spi_slave_ms58_instance;

/* SPI data buffers */
uint8_t spi_wdata[SPI_DATA_LENGTH];
uint8_t spi_rdata[SPI_DATA_LENGTH];

/* ------------
 * DECLARATIONS
 * ------------ */
void spi_master_configure(void);
enum status_code spi_master_transceive(struct spi_module *const module, struct spi_slave_inst *const slave, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

#endif