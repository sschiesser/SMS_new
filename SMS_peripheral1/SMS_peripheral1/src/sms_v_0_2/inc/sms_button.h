﻿/*
* sms_button.h
*
* Created: 03.05.2016 15:01:55
*  Author: Sébastien Schiesser
*/
#ifndef SMS_BUTTON_H_
#define SMS_BUTTON_H_


/* -------
 * INCLUDE
 * ------- */
#include "sms_peripheral1.h"
#include "sms_common.h"
#include "at_ble_api.h"
#include "ble_manager.h"


/* ------
 * MACROS
 * ------ */

#define SMS_BUTTON_SERVICE_UUID_1           (0x1C575ABE)
#define SMS_BUTTON_SERVICE_UUID_2	        (0x5350BBBB)
#define SMS_BUTTON_SERVICE_UUID_3	        (0x000000000)
#define SMS_BUTTON_SERVICE_UUID_4	        (0x00000000)

#define SMS_BUTTON_BLE_CHAR_LEN             (1)

///** characteristic presentation format value */
//#define SMS_BTN_PRESENTATION_FORMAT_VALUE 0x04
///** @brief Characteristic presentation format exponent */
//#define SMS_BTN_PRESENTATION_FORMAT_EXPONENT 0x00
///** @brief Characteristic presentation format unit */
//#define SMS_BTN_PRESENTATION_FORMAT_UNIT SMS_BUTTON_SERVICE_UUID
///** @brief Characteristic presentation format namespace */
//#define SMS_BTN_PRESENTATION_FORMAT_NAMESPACE 0x01
///**  @brief Characteristic presentation format descriptor */
//#define SMS_BTN_PRESENTATION_FORMAT_DESCRIPTOR 0x1000

#define SMS_BTN_0_PIN                       (PIN_AO_GPIO_0) // XPLAINED PRO --> EXT1:9
#define SMS_BTN_1_PIN                       (PIN_AO_GPIO_2) // XPLAINED PRO --> EXT1:6

#define SMS_BTN_NUMBER                      (2)

#define SMS_BTN_SHTDWN_MS                   (250)
#define SMS_BTN_SHTDWN_CNT                  (10)
#define SMS_BTN_STARTUP_MS                  (250)
#define SMS_BTN_STARTUP_CNT                 (16)


/* ---------
 * VARIABLES
 * --------- */
/* sms_btn_... -> variables assigned to each single button instance */
enum sms_btn_ids {
    SMS_BTN_0 = 0,
    SMS_BTN_1 = 1
};
enum sms_btn_int_tog {
    SMS_BTN_INT_ENABLE,
    SMS_BTN_INT_DISABLE
};
struct sms_btn_struct {
    enum sms_btn_ids id;
    uint8_t gpio_pin;
    enum sms_btn_int_tog int_enabled;
	bool new_int;
	bool new_char;
    uint8_t char_value;
};

/* sms_button_... -> variable assigned to the global button service */
enum sms_button_state {
    BUTTON_STATE_NONE,
    BUTTON_STATE_B0,
    BUTTON_STATE_B1,
    BUTTON_STATE_BOTH
};

typedef struct sms_button_struct {
	struct sms_btn_struct btn0;
	struct sms_btn_struct btn1;
    enum sms_button_state previous_state;
    enum sms_button_state current_state;
    gatt_service_handler_t service_handler;
}sms_button_struct_t;
sms_button_struct_t button_instance;


/* ------------
 * DECLARATIONS
 * ------------ */
int sms_button_fn(enum sms_btn_ids btn);
void sms_button_init_variables(void);
void sms_button_gpio_init(void);
void sms_button_register_callbacks(void);
enum sms_button_state sms_button_get_state(void);
void sms_button_toggle_callback(enum sms_btn_int_tog tog1, enum sms_btn_int_tog tog2);
void sms_button_bt0_callback(void);
void sms_button_bt1_callback(void);
void sms_button_define_services(void);

#endif /* SMS_BUTTON_H_ */