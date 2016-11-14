/*
 * sms_pressure.h
 *
 * Created: 07.06.2016 16:35:03
 *  Author: Sébastien Schiesser
 */ 


#ifndef SMS_PRESSURE_H_
#define SMS_PRESSURE_H_

/* -------
 * INCLUDE
 * ------- */
#include "sms_peripheral1.h"
#include "ms58.h"

/* ------
 * MACROS
 * ------ */
#define SMS_PRESSURE_SERVICE_UUID_1             (0x1C570000)
#define SMS_PRESSURE_SERVICE_UUID_2             (0x5ABE0000)
#define SMS_PRESSURE_SERVICE_UUID_3             (0x50300000)
#define SMS_PRESSURE_SERVICE_UUID_4             (0xEEEE0000)

#define SMS_PRESSURE_VCC_PIN                    (PIN_LP_GPIO_7)
#define SMS_PRESSURE_SPI_SS_PIN                 (PIN_LP_GPIO_16)

#define SMS_PRESSURE_RESET_MS                   (20 * SMS_TIMER_AON_COUNT_1MS)
#define SMS_PRESSURE_CONVERT1_MS                (SMS_TIMER_AON_COUNT_10MS)
#define SMS_PRESSURE_CONVERT2_MS                (SMS_TIMER_AON_COUNT_100MS)
#define SMS_PRESSURE_CONVERT3_MS                (SMS_TIMER_AON_COUNT_1S)
#define SMS_PRESSURE_CONVERT_MS                 (10 * SMS_PRESSURE_CONVERT1_MS)

/* ---------
 * VARIABLES
 * --------- */
/* GATT service handler */
gatt_service_handler_t sms_pressure_service_handler;
/* Pressure characteristic */
uint8_t sms_pressure_char_values[8];
//uint8_t sms_pressure_char_init_values[8];
//
//volatile sms_ms58_state_t ms58_current_state;
//volatile bool ms58_reset_done;

volatile sms_ms58_instance_t ms58_device;
enum sms_sensor_state sms_pressure_state;

//struct sms_pressure_ms58_inst {
    //bool device_init_ok;
    //uint8_t osr;
    //uint16_t prom_values[MS58_PROM_VALUES_MAX];
    //uint32_t adc_values[MS58_ADC_VALUES_MAX];
    //int32_t pressure;
    //int32_t temperature;
    //bool conv_d1_done;
//};
//
//struct sms_pressure_ms58_inst ms58_device;

/* ------------
 * DECLARATIONS
 * ------------ */
void sms_pressure_configure_gpio(void);
void sms_pressure_startup(void);
enum status_code sms_pressure_init(void);
void sms_pressure_ms58_reset(void);
void sms_pressure_poll_data(void);
enum status_code sms_pressure_ms58_read_prom(void);
enum status_code sms_pressure_ms58_read_data(void);
void sms_pressure_ms58_calculate(void);

void sms_pressure_define_services(void);
void sms_pressure_service_init(gatt_service_handler_t *sms_pressure_serv, uint8_t *sms_pressure_value);

#endif /* SMS_PRESSURE_H_ */