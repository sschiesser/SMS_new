/*
 * sms_ble.c
 *
 * Created: 19.09.2016 14:02:18
 *  Author: Sébastien Schiesser
 */ 

#include <stdio.h>
#include "sms_central1.h"

void sms_ble_startup(void)
{
}

/* AT_BLE_SCAN_REPORT (#2) */
at_ble_status_t sms_ble_scan_report_fn(void *params)
{
    return AT_BLE_SUCCESS;
}

/* AT_BLE_CONNECTED (#5) */
at_ble_status_t sms_ble_connected_fn(void *params)
{
    return AT_BLE_SUCCESS;
}

/* AT_BLE_DISCONNECTED (#6) */
at_ble_status_t sms_ble_disconnected_fn(void *params)
{
    return AT_BLE_SUCCESS;
}

/* AT_BLE_PAIR_DONE (#9) */
at_ble_status_t sms_ble_paired_fn(void *params)
{ 
    return AT_BLE_SUCCESS;
}

/* AT_BLE_PRIMARY_SERVICE_FOUND (#19) */
at_ble_status_t sms_ble_primary_service_found_fn(void *params)
{
    return AT_BLE_SUCCESS;
}

/* AT_BLE_CHARACTERISTIC_FOUND (#21) */
at_ble_status_t sms_ble_characteristic_found_fn(void *params)
{
    return AT_BLE_SUCCESS;
}

/* AT_BLE_DISCOVERY_COMPLETE (#23) */
at_ble_status_t sms_ble_discovery_complete_fn(void *params)
{
    return AT_BLE_SUCCESS;
}

/* AT_BLE_CHARACTERISTIC_READ_BY_UUID_RESPONSE (#24) */
at_ble_status_t sms_ble_characteristic_read_fn(void *params)
{
    return AT_BLE_SUCCESS;
}

/* AT_BLE_NOTIFICATION_RECEIVED (#27) */
at_ble_status_t sms_ble_notification_received_fn(void *params)
{
    return AT_BLE_SUCCESS;
}

/* AT_BLE_INDICATION_RECEIVED (#28) */
at_ble_status_t sms_ble_indication_received_fn(void *params)
{
    return AT_BLE_SUCCESS;
}

const ble_event_callback_t sms_ble_gap_cb[] = {
    NULL, //ble_undefined_event
    NULL, //ble_scan_info
    sms_ble_scan_report_fn,
    NULL, //ble_adv_report
    NULL, //ble_rand_addr_changed
    sms_ble_connected_fn,
    sms_ble_disconnected_fn,
    NULL, //ble_conn_param_update_done
    NULL, //ble_conn_param_update_request
    sms_ble_paired_fn,
    NULL, //ble_pair_request
    NULL, //ble_slave_sec_request
    NULL, //ble_pair_key_request
    NULL, //ble_encryption_request
    NULL, //ble_encyrption_status_changed
    NULL, //ble_resolv_rand_addr_status
    NULL, //ble_sign_counters_ind
    NULL, //ble_peer_att_info_ind
    NULL //ble_con_channel_map_ind
};

const ble_event_callback_t sms_ble_gatt_client_cb[] = {
    sms_ble_primary_service_found_fn,
    NULL, //ble_included_service_found
    sms_ble_characteristic_found_fn,
    NULL, //ble_descriptor_found
    sms_ble_discovery_complete_fn,
    sms_ble_characteristic_read_fn,
    NULL, //ble_characteristic_read_multiple_response
    NULL, //ble_characteristic_write_response
    sms_ble_notification_received_fn,
    sms_ble_indication_received_fn
};