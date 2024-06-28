/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *
 *  OFU (OTA FW Upgrade) Over SPP implementation. Server side only.
 */
#ifdef OTA_FW_UPGRADE
#include "wiced.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "app_ofu.h"
#include "app_ofu_ble.h"
#include "app_ofu_srv.h"
#include "app_trace.h"

/*
 * Definitions
 */
typedef struct
{
    app_ofu_callback_t *p_callback;
    wiced_bt_device_address_t bdaddr;
    uint16_t conn_id;
    uint16_t config_descriptor;
    uint8_t connected;
    uint8_t ofu_started;
} app_ofu_ble_cb_t;

/*
 * Local functions
 */
static void app_ofu_ble_app_callback(app_ofu_event_t event);
static wiced_result_t app_ofu_ble_send_status(uint8_t status);

/*
 * Global variables
 */
static app_ofu_ble_cb_t app_ofu_ble_cb;

/*
 * app_ofu_ble_init
 */
wiced_result_t app_ofu_ble_init(app_ofu_callback_t *p_callback)
{
    wiced_result_t status;

    APP_OFU_TRACE_DBG("\n");

    memset(&app_ofu_ble_cb, 0, sizeof(app_ofu_ble_cb));

    app_ofu_ble_cb.p_callback = p_callback;

    return WICED_BT_SUCCESS;
}

/*
 * app_ofu_ble_app_callback
 * This function is called to send OFU Events (Started/Completed/Aborted) to the application
 */
static void app_ofu_ble_app_callback(app_ofu_event_t event)
{
    app_ofu_event_data_t event_data;

    if (app_ofu_ble_cb.p_callback == NULL)
    {
        APP_TRACE_ERR("No Callback\n");
        return;
    }

    switch(event)
    {
    case APP_OFU_EVENT_STARTED:
        app_ofu_ble_cb.ofu_started = WICED_TRUE;
        event_data.started.transport = APP_OFU_TRANSPORT_BLE_SERVER;
        /* Change the LE Connection interval to increase throughput (reduce Upgrade duration) */
        wiced_bt_l2cap_update_ble_conn_params(app_ofu_ble_cb.bdaddr, 6, 6, 0, 200);
        break;

    case APP_OFU_EVENT_COMPLETED:
        app_ofu_ble_cb.ofu_started = WICED_FALSE;
        event_data.completed.transport = APP_OFU_TRANSPORT_BLE_SERVER;
        /* Change the LE Connection interval to default (reduce power consumed) */
        wiced_bt_l2cap_update_ble_conn_params(app_ofu_ble_cb.bdaddr, 40, 60, 0, 500);
        break;

    case APP_OFU_EVENT_ABORTED:
        app_ofu_ble_cb.ofu_started = WICED_FALSE;
        event_data.aborted.transport = APP_OFU_TRANSPORT_BLE_SERVER;
        /* Change the LE Connection interval to default (reduce power consumed) */
        wiced_bt_l2cap_update_ble_conn_params(app_ofu_ble_cb.bdaddr, 40, 60, 0, 500);
        break;

    default:
        APP_TRACE_ERR("Unknown event:%d\n", event);
        return;
    }

    /* Send OFU Event to application */
    app_ofu_ble_cb.p_callback(event, &event_data);
}

/*
 * app_ofu_ble_connection_up
 */
void app_ofu_ble_connection_up(uint16_t conn_id, uint8_t *bdaddr)
{
    APP_OFU_TRACE_DBG("conn_id:%d address:%B\n", conn_id, bdaddr);
    app_ofu_ble_cb.connected = WICED_TRUE;
    app_ofu_ble_cb.conn_id = conn_id;
    memcpy(app_ofu_ble_cb.bdaddr, bdaddr, BD_ADDR_LEN);
}

/*
 * app_ofu_ble_connection_down
 */
void app_ofu_ble_connection_down(uint16_t conn_id)
{
    APP_OFU_TRACE_DBG("conn_id:%d\n", conn_id);

    memset(app_ofu_ble_cb.bdaddr, 0, BD_ADDR_LEN);
    app_ofu_ble_cb.connected = WICED_FALSE;
    app_ofu_ble_cb.conn_id = 0;
    app_ofu_ble_cb.config_descriptor = 0;

    /* If LE disconnected during OFU, send an Abort message to the application */
    if (app_ofu_ble_cb.ofu_started)
    {
        app_ofu_ble_cb.ofu_started = WICED_FALSE;
        app_ofu_ble_app_callback(APP_OFU_EVENT_ABORTED);
    }
}

/*
 * app_ofu_ble_read_handler
 * Process GATT Read request
 */
wiced_bt_gatt_status_t app_ofu_ble_read_handler(uint16_t conn_id,
        wiced_bt_gatt_read_t *p_read_data)
{
    switch (p_read_data->handle)
    {
    case HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR:
        if (p_read_data->offset >= 2)
            return WICED_BT_GATT_INVALID_OFFSET;

        if (*p_read_data->p_val_len < 2)
            return WICED_BT_GATT_INVALID_ATTR_LEN;

        if (p_read_data->offset == 1)
        {
            p_read_data->p_val[0] = app_ofu_ble_cb.config_descriptor >> 8;
            *p_read_data->p_val_len = 1;
        }
        else
        {
            p_read_data->p_val[0] = app_ofu_ble_cb.config_descriptor & 0xff;
            p_read_data->p_val[1] = app_ofu_ble_cb.config_descriptor >> 8;
            *p_read_data->p_val_len = 2;
        }
        return WICED_BT_GATT_SUCCESS;
    }
    return WICED_BT_GATT_INVALID_HANDLE;
}

/*
 * app_ofu_ble_write_handler
 * Process GATT Write request
 */
wiced_bt_gatt_status_t app_ofu_ble_write_handler(uint16_t conn_id,
        wiced_bt_gatt_write_t *p_write_data)
{
    uint8_t srv_status;
    uint8_t header;
    uint8_t *p;
    uint16_t length;

    p = p_write_data->p_val;
    length = p_write_data->val_len;

    switch (p_write_data->handle)
    {
    case HANDLE_OTA_FW_UPGRADE_CONTROL_POINT:
        if (length < 1)
        {
            APP_TRACE_ERR("wrong len:%d\n", length);
            return WICED_BT_GATT_ERROR;
        }
        /* Extract Packet Header (the Command) */
        STREAM_TO_UINT8(header, p);
        length--;
        /* Handle the OFU Command */
        srv_status = app_ofu_srv_command_handler(header, p, length, app_ofu_ble_app_callback);
        /* Sends Status */
        app_ofu_ble_send_status(srv_status);
        if (srv_status != WICED_OTA_UPGRADE_STATUS_OK)
        {
            return WICED_BT_GATT_ERROR;
        }
        break;

    case HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR:
        if (length != 2)
        {
            APP_TRACE_ERR("wrong client config len:%d\n", length);
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        STREAM_TO_UINT16(app_ofu_ble_cb.config_descriptor, p);
        APP_OFU_TRACE_DBG("CCC:0x%x\n", app_ofu_ble_cb.config_descriptor);
        break;

    case HANDLE_OTA_FW_UPGRADE_DATA:
        srv_status = app_ofu_srv_data_handler(p, length, app_ofu_ble_app_callback);
        if ((srv_status != WICED_OTA_UPGRADE_STATUS_OK) &&
            (srv_status != WICED_OTA_UPGRADE_STATUS_CONTINUE))
        {
            return WICED_BT_GATT_ERROR;
        }
        break;

    default:
        APP_TRACE_ERR("invalid handle:0x%x\n", p_write_data->handle);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * app_ofu_ble_send_status
 * Send OFU Status (to OFU client)
 */
static wiced_result_t app_ofu_ble_send_status(uint8_t status)
{
    if (app_ofu_ble_cb.config_descriptor & GATT_CLIENT_CONFIG_INDICATION)
    {
        return wiced_bt_gatt_send_indication(app_ofu_ble_cb.conn_id,
                HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &status);
    }
    else if (app_ofu_ble_cb.config_descriptor & GATT_CLIENT_CONFIG_NOTIFICATION)
    {
        return wiced_bt_gatt_send_notification(app_ofu_ble_cb.conn_id,
                HANDLE_OTA_FW_UPGRADE_CONTROL_POINT, 1, &status);
    }
    else
    {
        APP_TRACE_ERR("Neither Ind nor Notif enabled\n");
    }
    return WICED_BT_GATT_ERROR;
}
#endif
