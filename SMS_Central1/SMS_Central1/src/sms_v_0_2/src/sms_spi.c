/*
* sms_spi.c
*
* Created: 14.06.2016 13:34:05
*  Author: Sébastien Schiesser
*/

#include "sms_spi.h"

/* Configure SPI master:
 * - allocate memory for the write & read buffers
 * - set-up two different communication modes:
 *   + mode 0 on SPI1 @ 250 kHz for MS58 pressure sensor
 *   + mode 3 on SPI0 @ 1 MHz for MPU-9250 IMU
 * - assign SS pin for each slave device
 * - enable SPI
 */ 
void sms_spi_master_configure(void)
{
    //DBG_LOG_DEV("[spi_master_configure]\tconfiguring SPI masters and attaching slaves...");

    /* Initialize ms58 spi slave device */
    struct spi_config spi_master_config;
    struct spi_slave_inst_config spi_slave_config;
    spi_slave_inst_get_config_defaults(&spi_slave_config);
    spi_slave_config.ss_pin = SPI_SLAVE_SSN_PIN;
    spi_attach_slave(&spi_slave_instance, &spi_slave_config);
    

    /* Initialize SPI1 master for ms58 */
    spi_get_config_defaults(&spi_master_config);
    spi_master_config.clock_divider = SPI_MASTER_CLOCK_DIV;
    spi_master_config.transfer_mode = SPI_MASTER_MODE;
    spi_master_config.pin_number_pad[0] = SPI_MASTER_PIN_SCK;
    spi_master_config.pinmux_sel_pad[0] = SPI_MASTER_MUX_SCK;
    spi_master_config.pin_number_pad[1] = SPI_MASTER_PIN_MOSI;
    spi_master_config.pinmux_sel_pad[1] = SPI_MASTER_MUX_MOSI;
    spi_master_config.pin_number_pad[2] = SPI_MASTER_PIN_SSN;
    spi_master_config.pinmux_sel_pad[2] = SPI_MASTER_MUX_SSN;
    spi_master_config.pin_number_pad[3] = SPI_MASTER_MISO;
    spi_master_config.pinmux_sel_pad[3] = SPI_MASTER_MUX_MISO;
    spi_init(&spi_master_instance, SPI_MASTER_PORT, &spi_master_config);
    spi_enable(&spi_master_instance);
}


/* SPI master transceive callback:
 * - set spi_transceived_done flag
 */
//void spi_master_transceived_callback(struct spi_module *const module)
//{
    //spi_transceived_done = true;
//}

/* Configure SPI master callbacks:
 * - register buffer transceive callback for each spi master instance
 * - enable callbacks
 */
//void spi_master_configure_callbacks(void)
//{
    //DBG_LOG_DEV("[spi_master_configure_callbacks]  configuring SPI callbacks...");
    //spi_transceived_done = false;
//
    //spi_register_callback(&spi_master_ms58_instance, spi_master_transceived_callback, SPI_CALLBACK_BUFFER_TRANSCEIVED);
    //spi_register_callback(&spi_master_mpu9250_instance, spi_master_transceived_callback, SPI_CALLBACK_BUFFER_TRANSCEIVED);
//
    //spi_enable_callback(&spi_master_ms58_instance, SPI_CALLBACK_BUFFER_TRANSCEIVED);
    //spi_enable_callback(&spi_master_mpu9250_instance, SPI_CALLBACK_BUFFER_TRANSCEIVED);
//}

/* SPI transceive function:
 * - enable selected slave
 * - start transceive blocking job
 * - when job returned, disable selected slave
 */
enum status_code sms_spi_master_transceive(struct spi_module *const module, struct spi_slave_inst *const slave, uint8_t *tx_data, uint8_t *rx_data, uint16_t len) {
    enum status_code retVal;
    /* Enable slave */
    //DBG_LOG_CONT_DEV(" selecting slave...");
    spi_select_slave(module, slave, true);
    /* Write SPI data */
    //DBG_LOG_CONT_DEV(" writing data...");
    retVal = spi_transceive_buffer_wait(module, tx_data, rx_data, len);
    /* Disable slave */
    //DBG_LOG_CONT_DEV(" de-selecting slave...");
    spi_select_slave(module, slave, false);
    //DBG_LOG_CONT_DEV(" done!");
    return retVal;
}
