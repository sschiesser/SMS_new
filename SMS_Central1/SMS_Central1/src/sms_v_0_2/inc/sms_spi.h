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
#include "sms_central1.h"

/* ------
 * MACROS
 * ------ */
/* SPI master settings */
#define SPI_MASTER_PORT         SPI1
#define SPI_MASTER_MODE         SPI_TRANSFER_MODE_0
#define SPI_MASTER_CLOCK_DIV    104
/* Pins */
#define SPI_MASTER_PIN_SCK      PIN_LP_GPIO_17
#define SPI_MASTER_PIN_MOSI     PIN_LP_GPIO_19
#define SPI_MASTER_PIN_SSN      PINMUX_UNUSED
#define SPI_MASTER_MISO         PIN_LP_GPIO_18
/* Mux */
#define SPI_MASTER_MUX_SCK      MUX_LP_GPIO_17_SPI1_SCK
#define SPI_MASTER_MUX_MOSI     MUX_LP_GPIO_19_SPI1_MOSI
#define SPI_MASTER_MUX_SSN      PINMUX_UNUSED
#define SPI_MASTER_MUX_MISO     MUX_LP_GPIO_18_SPI1_MISO

#define SPI_DATA_LENGTH         16
#define SPI_SLAVE_SSN_PIN       PIN_LP_GPIO_12
/* ---------
 * VARIABLES
 * --------- */
 /* SPI device instances */
struct spi_module spi_master_instance;
struct spi_slave_inst spi_slave_instance;

/* SPI data buffers */
uint8_t spi_wdata[SPI_DATA_LENGTH];
uint8_t spi_rdata[SPI_DATA_LENGTH];

/* ------------
 * DECLARATIONS
 * ------------ */
void sms_spi_master_configure(void);
enum status_code sms_spi_master_transceive(struct spi_module *const module, struct spi_slave_inst *const slave, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

#endif