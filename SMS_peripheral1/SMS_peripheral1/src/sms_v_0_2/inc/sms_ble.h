/*
* sms_ble.h
*
* Created: 19.09.2016 14:02:31
*  Author: Sébastien Schiesser
*/


#ifndef SMS_BLE_H_
#define SMS_BLE_H_

/* -------
* INCLUDE
* ------- */
#include "sms_peripheral1.h"
#include "ble_manager.h"
#include "at_ble_api.h"

/* ------
* MACROS
* ------ */
#define BLE_DEVICE_NAME                                 "SABRe-SMS"
#define BLE_AUTHENTICATION_LEVEL                        (AT_BLE_NO_SEC)
#define BLE_IO_CAPABALITIES                             (AT_BLE_IO_CAP_NO_INPUT_NO_OUTPUT)
#define BLE_MITM_REQ                                    (false)
#define BLE_BOND_REQ                                    (false)
#define BLE_GAP_ADV_SERVICE_16BIT_UUID_ENABLE           (true)
#define BLE_GAP_ADV_MANUFACTURER_SPECIFIC_DATA_ENABLE   (true)
#define BLE_GAP_ADV_COMPLETE_LOCAL_NAME_ENABLE          (true)
#define BLE_GAP_ADV_SHORTENED_LOCAL_NAME_ENABLE         (false)

#define BLE_GAP_ADV_DATA_MANUFACTURER_SPECIFIC_DATA		"\x1C\x57\x2d\x5A\xBE\x2d" //\x53\x50"

#define BLE_INDICATION_TOUT_MS                          (100)
#define BLE_INDICATION_RETRY_MAX                        (1)

#define BLE_CHAR_SIZE_BUTTON                            (SMS_BUTTON_BLE_CHAR_LEN)
#define BLE_CHAR_SIZE_PRESSURE                          (SMS_PRESSURE_BLE_CHAR_LEN)
#define BLE_CHAR_SIZE_MPU								(SMS_MPU_BLE_CHAR_LEN_QUAT)
#define BLE_CHAR_SIZE_MAX                               (BLE_CHAR_SIZE_MPU)

#define RTS_BUTTON_POS									0
#define RTS_PRESSURE_POS								1
#define RTS_MPU_POS										2

#define BLE_SEND_TIMEOUT								10

#define BLE_ADV_TIMEOUT									(60) // in seconds...
#define BLE_ADV_INTERVAL								(1600) // in 0.625 ms units (range: 0x0020 - 0x4000)

#define BLE_APP_TIMEOUT_OFF								(0xFFFFFFFF) // no timeout
#define BLE_APP_TIMEOUT_PAIR							(500) // 500 @ 10 ms resolution --> 5000 ms
#define BLE_APP_TIMEOUT_NOTIFY							(2) // 2 @ 10 ms resolution --> 20 ms

/* ---------
* VARIABLES
* --------- */
enum sms_ble_serv_type {
	BLE_SERV_BUTTON,
	BLE_SERV_PRESSURE,
	BLE_SERV_MPU
};

enum sms_ble_char_type {
	BLE_CHAR_BTN,
	BLE_CHAR_PRESS,
	BLE_CHAR_MPU
};

typedef enum sms_ble_state {
    BLE_STATE_POWEROFF = 0x00,
    BLE_STATE_STARTING = 0x10,
    BLE_STATE_DISCONNECTED,
    BLE_STATE_ADVERTISING = 0x20,
    BLE_STATE_CONNECTED,
    BLE_STATE_PAIRED,
    BLE_STATE_INDICATING,
    BLE_STATE_SHUTTINGDOWN = 0xA0
}sms_ble_state_t;
//volatile sms_ble_state_t ble_current_state;


uint32_t sms_ble_timeout;

struct ble_device_s {
	volatile sms_ble_state_t current_state;
	volatile uint8_t sending_queue;
	at_ble_handle_t conn_handle;
	uint8_t ind_retries;
	uint16_t send_cnt;
};
struct ble_device_s ble_instance;



/* ------------
* DECLARATIONS
* ------------ */
/* AT_BLE_ functions */
at_ble_status_t sms_ble_adv_report_fn(void *params);
at_ble_status_t sms_ble_connected_fn(void *params);
at_ble_status_t sms_ble_disconnected_fn(void *params);
at_ble_status_t sms_ble_paired_fn(void *params);
at_ble_status_t sms_ble_pair_request_fn(void *params);
at_ble_status_t sms_ble_notification_confirmed_fn(void *params);
at_ble_status_t sms_ble_indication_confirmed_fn(void *params);

/* BLE GAP functions list */
const ble_event_callback_t sms_ble_gap_cb[GAP_HANDLE_FUNC_MAX];
/* BLE GATT SERVER functions list */
const ble_event_callback_t sms_ble_gatt_server_cb[GATT_SERVER_HANDLER_FUNC_MAX];

/* Own functions */
void sms_ble_init_variables(void);
int sms_ble_startup(void);
void sms_ble_power_down(void);
at_ble_status_t sms_ble_advertise(void);
at_ble_status_t sms_ble_primary_service_define(gatt_service_handler_t *service);
void sms_ble_service_init(enum sms_ble_serv_type type, gatt_service_handler_t *service, uint8_t *value);
at_ble_status_t sms_ble_send_characteristic(enum sms_ble_char_type ch);

#endif /* SMS_BLE_H_ */