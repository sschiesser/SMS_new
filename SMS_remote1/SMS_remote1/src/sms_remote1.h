/*
 * sms_remote1.h
 *
 * Created: 17.01.2017 11:51:29
 *  Author: sschies1
 */ 

#ifndef __SMS_REMOTE_H__
#define __SMS_REMOTE_H__

/* ------------------------------------------------
 * INCLUDE
 * ------------------------------------------------ */
#include <asf.h>
#include "platform.h"
#include "at_ble_api.h"
#include "console_serial.h"
#include "timer_hw.h"
#include "ble_manager.h"
#include "ble_utils.h"
#include "delay.h"
/* SMS include */
#include "sms_common.h"
#include "sms_button.h"
#include "sms_timer.h"
#include "sms_led.h"
#include "sms_ble.h"


/* ------------------------------------------------
 * MACROS
 * ------------------------------------------------ */
#define SMS_SENDING_WITH_ACK                (false)
/** @brief APP_FAST_ADV between 0x0020 and 0x4000 in 0.625 ms units (20ms to 10.24s). */
#define APP_FAST_ADV						(1600)

/** @brief APP_ADV_TIMEOUT Advertising time-out between 0x0001 and 0x028F in seconds, 0x0000 disables time-out.*/
#define APP_ADV_TIMEOUT						(655)

#define DBG_PIN_1                           (PIN_LP_GPIO_14)
#define DBG_PIN_2							(PIN_LP_GPIO_15)
#define DBG_PIN_HIGH                        (true)
#define DBG_PIN_LOW                         (false)


/* ------------------------------------------------
 * VARIABLES
 * ------------------------------------------------ */
typedef enum sms_mode {
    SMS_MODE_NONE,
    SMS_MODE_BUTTON_SOLO,
    SMS_MODE_MPU_SOLO,
    SMS_MODE_PRESSURE_SOLO,
    SMS_MODE_BUTTON_MPU,
    SMS_MODE_BUTTON_PRESSURE,
    SMS_MODE_MPU_PRESSURE,
    SMS_MODE_COMPLETE
}sms_mode_t;
volatile sms_mode_t sms_working_mode;

//typedef enum sms_plf_int_src {
    //INT_NONE,
    //INT_BTN0,
    //INT_BTN1,
    //INT_MPU_DRDY,
    //INT_AON_TIMER,
    //INT_DUALTIMER1,
    //INT_DUALTIMER2
//}sms_plf_int_src_t;
//typedef struct sms_plf_int {
    //sms_plf_int_src_t source;
    //volatile bool int_on;
//}sms_plf_int_t;
//volatile sms_plf_int_t sms_current_interrupt;

uint8_t sms_btn_cnt;
at_ble_handle_t sms_connection_handle;

volatile bool ulp_ready;
volatile bool ulp_active;
volatile bool sensors_active;

volatile uint32_t psp;
volatile uint32_t msp;

/* ------------------------------------------------
 * DECLARATIONS
 * ------------------------------------------------ */
void sms_init_variables(void);


#endif /* SMS_REMOTE_H_ */