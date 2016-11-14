/*
 * sms_ble.c
 *
 * Created: 19.09.2016 14:02:18
 *  Author: Sébastien Schiesser
 */ 

#include <stdio.h>
//#include <stdlib.h>
#include "sms_peripheral1.h"

void sms_ble_startup(void)
{
    //sms_button_toggle_interrupt(BTN_INT_DISABLE, BTN_INT_DISABLE);
    timer2_current_mode = TIMER2_MODE_LED_STARTUP;
    sms_led_blink_start(SMS_LED_0_PIN);
}

void sms_ble_power_down(void)
{
    sms_monitor_states("[sms_ble_power_down]");
    if(ble_current_state == BLE_STATE_POWEROFF) {
        /* If already power off state, then go back sleeping */
        //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
        ulp_ready = true;
        release_sleep_lock();
    }
    else {
        /* Disable button interrupts */
        //sms_button_toggle_interrupt(BTN_INT_DISABLE, BTN_INT_DISABLE);        
        /* Disconnect if necessary from BLE network */
        switch(ble_current_state) {
            case BLE_STATE_ADVERTISING:
            DBG_LOG_DEV("[sms_ble_power_down]\tStopping command received during advertisement. Stopping... ");
            if(at_ble_adv_stop() != AT_BLE_SUCCESS) {
                DBG_LOG_CONT_DEV("failed!!!");
                //#pragma TBD: manage adv_stop failure
            }
            else {
                DBG_LOG_CONT_DEV("done!");
                ble_current_state = BLE_STATE_DISCONNECTED;
            }
            break;
            
            case BLE_STATE_PAIRED:
            DBG_LOG_DEV("[sms_ble_power_down]\t\tDevice paired... disabling interrupts & switching down sensors");
            
            case BLE_STATE_INDICATING:
            DBG_LOG_DEV("[sms_ble_power_down]\t\tCurrently indicating");
            sms_sensors_toggle_interrupt(SMS_EXT_INT_DISABLE);
            //#pragma TBD: switch-off sensors to save current
            //sms_sensors_switch(false);
            
            case BLE_STATE_CONNECTED:
            DBG_LOG_DEV("[sms_ble_power_down]\t\tDevice connected... disconnecting");
            at_ble_disconnect(sms_connection_handle, AT_BLE_TERMINATED_BY_USER);
            break;
            
            default:
            break;
        }
        
        ble_current_state = BLE_STATE_DISCONNECTED;
        timer2_current_mode = TIMER2_MODE_LED_SHUTDOWN;
        sms_led_blink_start(SMS_LED_0_PIN);
    }
}

at_ble_status_t sms_ble_advertise(void)
{
    at_ble_status_t status = AT_BLE_FAILURE;
    ble_current_state = BLE_STATE_ADVERTISING;

    /* Set the advertisement data */
    if((status = ble_advertisement_data_set()) != AT_BLE_SUCCESS) {
        DBG_LOG("[sms_ble_advertise]\tAdvertisement data set failed!");
        return status;
    }

    /* Start of advertisement */
    if((status = at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED, AT_BLE_ADV_GEN_DISCOVERABLE, NULL, AT_BLE_ADV_FP_ANY, APP_FAST_ADV, APP_ADV_TIMEOUT, 0)) == AT_BLE_SUCCESS)
    {
        DBG_LOG_DEV("[sms_ble_advertise]\t\tBLE Started Advertisement");
        return AT_BLE_SUCCESS;
    } 
    else {
        DBG_LOG("[sms_service_advertise]\tBLE Advertisement start failed: reason 0x%x", status);
    }
    return AT_BLE_FAILURE;
}

/* AT_BLE_ADV_REPORT (#3) */
at_ble_status_t sms_ble_adv_report_fn(void *params)
{
    at_ble_adv_report_t *adv_report = (at_ble_adv_report_t *)params;
    ble_current_state = BLE_STATE_DISCONNECTED;
    DBG_LOG_DEV("[sms_ble_adv_report_fn]\tAdvertisement timeout...");
    //DBG_LOG_DEV("- status: 0x%02x", adv_report->status);
    sms_ble_power_down();
    return AT_BLE_SUCCESS;
}

/* AT_BLE_CONNECTED (#5) */
at_ble_status_t sms_ble_connected_fn(void *params)
{
    if(ble_current_state == BLE_STATE_ADVERTISING) {
        at_ble_connected_t *connected = (at_ble_connected_t *)params;
        sms_ble_conn_handle = connected->handle;
        ble_current_state = BLE_STATE_CONNECTED;
        DBG_LOG_DEV("[sms_ble_connected_fn]\t\tDevices connected...");
        //DBG_LOG_DEV("- conn handle: 0x%04x\r\n- conn interval: %d\r\n- conn latency: %d\r\n- supervision timeout: %d\r\n- peer address: 0x", connected->handle, connected->conn_params.con_interval, connected->conn_params.con_latency, connected->conn_params.sup_to);
        //for(uint8_t i = 0; i < AT_BLE_ADDR_LEN; i++) {
            //DBG_LOG_CONT_DEV("%02x",connected->peer_addr.addr[AT_BLE_ADDR_LEN - (i+1)]);
        //}            
    }
    else {
        sms_ble_power_down();
    }    
    return AT_BLE_SUCCESS;
}

/* AT_BLE_DISCONNECTED (#6) */
at_ble_status_t sms_ble_disconnected_fn(void *params)
{
    at_ble_disconnected_t *disconnect = (at_ble_disconnected_t *)params;
    if(ble_current_state == BLE_STATE_PAIRED) {
        sms_sensors_toggle_interrupt(SMS_EXT_INT_DISABLE);
        sms_sensors_switch(false);
    }
    ble_current_state = BLE_STATE_DISCONNECTED;
    DBG_LOG_DEV("[sms_ble_disconnected_fn]\tPeer disconnected... Bnew %d, BLE 0x%02x, T1 %d, T2 %d", button_current_state, ble_current_state, timer1_current_mode, timer2_current_mode);
    //DBG_LOG_DEV("- conn handle: 0x%04x\r\n- reason: 0x%02x", disconnect->handle, disconnect->reason);
    switch(disconnect->reason) {
        case AT_BLE_AUTH_FAILURE: //0x05
        case AT_BLE_SUPERVISION_TIMEOUT: //0x08
        case AT_BLE_UNSUPPORTED_REMOTE_FEATURE: // 0x1A
        case AT_BLE_PAIRING_WITH_UNIT_KEY_NOT_SUP: // 0x29
        case AT_BLE_UNACCEPTABLE_INTERVAL: // 0x3B
        sms_ble_advertise();
        break;
        
        case AT_BLE_TERMINATED_BY_USER: // 0x13
        case AT_BLE_REMOTE_DEV_TERM_LOW_RESOURCES: //0x14
        case AT_BLE_REMOTE_DEV_POWER_OFF: //0x15
        case AT_BLE_CON_TERM_BY_LOCAL_HOST: //0x16
        default:
        sms_ble_power_down();
        break;
    }
    
    return AT_BLE_SUCCESS;
}

/* AT_BLE_PAIR_DONE (#9) */
at_ble_status_t sms_ble_paired_fn(void *params)
{
    if(ble_current_state == BLE_STATE_CONNECTED) {
        ble_current_state = BLE_STATE_PAIRED;
        at_ble_pair_done_t *pair_status = (at_ble_pair_done_t *)params;
        sms_monitor_states("[sms_ble_paired_fn]");
        //DBG_LOG_DEV("[sms_ble_paired_fn]\t\tDevices paired... Bnew %d, BLE 0x%02x, T1 %d, T2 %d", button_current_state, ble_current_state, timer1_current_mode, timer2_current_mode);
        //DBG_LOG_DEV("- conn handle: 0x%04x\r\n- authorization: 0x%02x\r\n- status: 0x%02x", pair_status->handle, pair_status->auth, pair_status->status);
        sms_sensors_switch(true); // ! Release sleep lock & enable buttons interrupt after reset done!
        //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
    }
    else {
        sms_ble_power_down();
    }        
    return AT_BLE_SUCCESS;
}

/* AT_BLE_PAIR_REQUEST (#10) */
at_ble_status_t sms_ble_pair_request_fn(void *params)
{
    at_ble_pair_request_t *request = (at_ble_pair_request_t *)params;
    DBG_LOG_DEV("[sms_ble_pair_request_fn]\tPairing request... Bnew %d, BLE 0x%02x, T1 %d, T2 %d", button_current_state, ble_current_state, timer1_current_mode, timer2_current_mode);
    //DBG_LOG_DEV("- conn handle: 0x%04x\r\n- peer features: 0x%02x", request->handle, request->peer_features);
    return AT_BLE_SUCCESS;
}

/* AT_BLE_NOTIFICATION_CONFIRMED (#29) */
at_ble_status_t sms_ble_notification_confirmed_fn(void *params)
{
    //gpio_pin_set_output_level(dbg_pin, DBG_PIN_HIGH);
    
    at_ble_cmd_complete_event_t *notification_status = (at_ble_cmd_complete_event_t *)params;
    //button_current_state = sms_button_get_state();
    //DBG_LOG_DEV("[sms_ble_notification_confirmed_fn]\tNotification sent... Bnew %d, BLE 0x%02x, T1 %d, T2 %d", button_current_state, ble_current_state, timer1_current_mode, timer2_current_mode);
    //DBG_LOG_DEV("- conn handle: 0x%04x\r\n- operation: 0x%02x\r\n- status: 0x%02x", notification_status->conn_handle, notification_status->operation, notification_status->status);
    sms_dualtimer_stop(DUALTIMER_TIMER2);
    timer2_current_mode = TIMER2_MODE_NONE;
    ble_current_state = BLE_STATE_PAIRED;
    //DBG_LOG_DEV("[sms_ble_notification_confirmed_fn]\tEnabling button int...");
    //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
    //DBG_LOG_CONT_DEV(" done!");
    //DBG_LOG_DEV("[sms_ble_notification_confirmed_fn]\tEnabling sensor int...");
    //sms_sensors_toggle_interrupt(SMS_EXT_INT_ENABLE);
    //DBG_LOG_CONT_DEV(" done!");

    //gpio_pin_set_output_level(dbg_pin, DBG_PIN_LOW);
    
    DBG_LOG_DEV("Timer1 current mode: %d", timer1_current_mode);
    if(timer1_current_mode == TIMER1_MODE_NONE) {
        ulp_ready = true;
    }
    return AT_BLE_SUCCESS;
}

/* AT_BLE_INDICATION_CONFIRMED (#30) */
at_ble_status_t sms_ble_indication_confirmed_fn(void *params)
{
    //gpio_pin_set_output_level(dbg_pin, DBG_PIN_HIGH);
    
    at_ble_indication_confirmed_t *indication_status = (at_ble_indication_confirmed_t *)params;
    //button_current_state = sms_button_get_state();
    //DBG_LOG_DEV("[sms_ble_indication_confirmed]\tIndication confirmed... Bnew %d, BLE 0x%02x, T1 %d, T2 %d", button_current_state, ble_current_state, timer1_current_mode, timer2_current_mode);
    //DBG_LOG_DEV("- conn handle: 0x%04x\r\n- char handle: 0x%04x\r\n- status: 0x%02x", indication_status->conn_handle, indication_status->char_handle, indication_status->status);
    sms_dualtimer_stop(DUALTIMER_TIMER2);
    timer2_current_mode = TIMER2_MODE_NONE;
    ble_current_state = BLE_STATE_PAIRED;
    //sms_button_toggle_interrupt(BTN_INT_ENABLE, BTN_INT_ENABLE);
    //sms_sensors_toggle_interrupt(SMS_EXT_INT_ENABLE);
    
    //gpio_pin_set_output_level(dbg_pin, DBG_PIN_LOW);
    
    if(timer1_current_mode == TIMER1_MODE_NONE) {
        ulp_ready = true;
    }        
    return AT_BLE_SUCCESS;
}

const ble_event_callback_t sms_ble_gap_cb[] = {
    NULL,
    NULL,
    NULL,
    sms_ble_adv_report_fn,
    NULL,
    sms_ble_connected_fn,
    sms_ble_disconnected_fn,
    NULL,
    NULL,
    sms_ble_paired_fn,
    sms_ble_pair_request_fn,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};

const ble_event_callback_t sms_ble_gatt_server_cb[] = {
    sms_ble_notification_confirmed_fn,
    sms_ble_indication_confirmed_fn,
    NULL, //ble_characteristic_changed_handler
    NULL, //ble_characteristic_config_changed_handler
    NULL, //ble_service_changed_indication_sent_handler
    NULL, //ble_write_authorize_request_handler
    NULL, //ble_mtu_changed_indication
    NULL, //ble_mtu_changed_cmd_complete
    NULL, //ble_characteristic_write_cmd_cmp
    NULL //ble_read_authorize_request_handler
};

at_ble_status_t sms_ble_send_characteristic(enum sms_ble_char_type ch)
{
    at_ble_status_t status = AT_BLE_SUCCESS;
    at_ble_handle_t val_handle = 0;
    uint8_t length = 0;
    uint8_t char_size = 0;
    uint8_t send_val[BLE_CHAR_SIZE_MAX];
    ble_current_state = BLE_STATE_INDICATING;

    
    sms_ble_send_cnt++;
    
    switch(ch) {
        case BLE_CHAR_BUTTON0:
        sms_button_char_value[btn0_instance.id] = ((sms_button_char_value[btn0_instance.id] >= 0x7f) ? 0 : (sms_button_char_value[btn0_instance.id] + 1));
        send_val[0] = sms_button_char_value[btn0_instance.id];
        val_handle = sms_button_service_handler.serv_chars.char_val_handle;
        length = 1;
        break;
        
        case BLE_CHAR_BUTTON1:
        sms_button_char_value[btn1_instance.id] = ((sms_button_char_value[btn1_instance.id] >= 0xff) ? 0 : (sms_button_char_value[btn1_instance.id] + 1));
        send_val[0] = sms_button_char_value[btn1_instance.id] + 0x80;
        val_handle = sms_button_service_handler.serv_chars.char_val_handle;
        length = 1;
        break;
        
        case BLE_CHAR_PRESSURE:
        send_val[0] = (uint8_t)(ms58_device.temperature & 0xff);
        send_val[1] = (uint8_t)((ms58_device.temperature >> 8) & 0xff);
        send_val[2] = (uint8_t)((ms58_device.temperature >> 16) & 0xff);
        send_val[3] = (uint8_t)((ms58_device.temperature >> 24) & 0xff);
        send_val[4] = (uint8_t)(ms58_device.pressure & 0xff);
        send_val[5] = (uint8_t)((ms58_device.pressure >> 8) & 0xff);
        send_val[6] = (uint8_t)((ms58_device.pressure >> 16) & 0xff);
        send_val[7] = (uint8_t)((ms58_device.pressure >> 24) & 0xff);
        val_handle = sms_pressure_service_handler.serv_chars.char_val_handle;
        length = 8;
        break;
        
        case BLE_CHAR_IMU:
        break;
    }
    
    status = at_ble_characteristic_value_set(val_handle, send_val, (length * sizeof(uint8_t)));
    if(status == AT_BLE_SUCCESS) {
        //DBG_LOG_DEV("[sms_ble_send_characteristic]\tSending char value... BLE 0x%02x, T1 %d, T2 %d, cnt = %d", ble_current_state, timer1_current_mode, timer2_current_mode, sms_ble_send_cnt);
        //DBG_LOG_DEV("- conn handle: 0x%04x\r\n- service handle: 0x%04x\r\n- service uuid: 0x", sms_connection_handle, sms_button_service_handler.serv_handle);
        //for(uint8_t i = 0; i < AT_BLE_UUID_128_LEN; i++) {
        //DBG_LOG_CONT_DEV("%02x", sms_button_service_handler.serv_uuid.uuid[(i)]);
        //}
        //DBG_LOG_DEV("- char value handle: 0x%04x\r\n- char value: 0x%02x", sms_button_service_handler.serv_chars.char_val_handle, send_val);
        
        printf("\r\ncnt: %d", sms_ble_send_cnt);
        gpio_pin_set_output_level(DBG_PIN_1, DBG_PIN_HIGH);
#   if SMS_SENDING_WITH_ACK == true
        sms_ble_ind_retry = 0;
        status = at_ble_indication_send(sms_connection_handle, val_handle);
#   else
        status = at_ble_notification_send(sms_connection_handle, val_handle);
#   endif
        gpio_pin_set_output_level(DBG_PIN_1, DBG_PIN_LOW);
        //psp = __get_PSP();
        //msp = __get_MSP();
        //printf("\r\n\@ sending: psp 0x%lx, msp 0x%lx", psp, msp);
        //register int n41 asm("sp");
        //register int n42 asm("lr");
        //register int n43 asm("r15");
        //printf("\r\n\@ sending: sp 0x%x, lr 0x%x", n41, n42);
        
        if(status == AT_BLE_SUCCESS) {
            timer2_current_mode = TIMER2_MODE_INDICATION_TOUT;
            sms_dualtimer_start(TIMER_UNIT_MS, BLE_INDICATION_TOUT_MS, DUALTIMER_TIMER2);
            //ulp_ready = false;
        }
        else {
            //#pragma TBD: handle sending error...
        }
    }
    return status;
}


at_ble_status_t sms_ble_primary_service_define(gatt_service_handler_t *service)
{
    //DBG_LOG_DEV("[sms_ble_primary_service_define]\n\r  defining primary service\r\n- uuid: 0x%02x\r\n- handle: 0x%02x\r\n- char uuid: 0x%02x%02x\r\n- char init value: %d", (unsigned int)service->serv_uuid.uuid, service->serv_handle, service->serv_chars.uuid.uuid[1], service->serv_chars.uuid.uuid[0], service->serv_chars.value_init_len);
    return(at_ble_primary_service_define(&service->serv_uuid, &service->serv_handle, NULL, 0, &service->serv_chars, 1));
}