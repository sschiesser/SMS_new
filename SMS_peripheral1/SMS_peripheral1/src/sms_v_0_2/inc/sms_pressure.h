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
#define SMS_PRESSURE_SERVICE_UUID_1             (0x1C575ABE)
#define SMS_PRESSURE_SERVICE_UUID_2             (0x5350EEEE)
#define SMS_PRESSURE_SERVICE_UUID_3             (0x00000000)
#define SMS_PRESSURE_SERVICE_UUID_4             (0x00000000)

#define SMS_PRESSURE_BLE_CHAR_LEN               (8)

#define SMS_PRESSURE_VCC_PIN                    (PIN_LP_GPIO_7)
#define SMS_PRESSURE_SPI_SS_PIN                 (PIN_LP_GPIO_16)

#define SMS_PRESSURE_RESET_MS                   (20 * SMS_TIMER_AON_COUNT_1MS)
#define SMS_PRESSURE_CONVERT1_MS                (SMS_TIMER_AON_COUNT_10MS)
#define SMS_PRESSURE_CONVERT2_MS                (SMS_TIMER_AON_COUNT_100MS)
#define SMS_PRESSURE_CONVERT3_MS                (SMS_TIMER_AON_COUNT_1S)
#define SMS_PRESSURE_CONVERT_MS                 (3*SMS_PRESSURE_CONVERT1_MS)

/* ---------
 * VARIABLES
 * --------- */
enum sms_pressure_state {
    PRESSURE_STATE_OFF,
    PRESSURE_STATE_STDBY,
    PRESSURE_STATE_ON
};
typedef struct sms_pressure_struct {
    ms58_instance_t hal;
	bool int_enabled;
	volatile bool new_int;
	volatile bool rts;
    enum sms_pressure_state state;
    gatt_service_handler_t service_handler;
    uint8_t char_values[8];
}sms_pressure_struct_t;
sms_pressure_struct_t pressure_device;


/* ------------
 * DECLARATIONS
 * ------------ */
void sms_pressure_init_variables(void);
void sms_pressure_configure_gpio(void);
void sms_pressure_startup(void);
enum status_code sms_pressure_init(void);
void sms_pressure_ms58_reset(void);
void sms_pressure_poll_data(void);
enum status_code sms_pressure_ms58_read_prom(void);
enum status_code sms_pressure_ms58_read_data(void);
void sms_pressure_ms58_calculate(void);
void sms_pressure_define_services(void);

#endif /* SMS_PRESSURE_H_ */