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
#include "wiced_bt_rfcomm.h"
#define BD_ADDR_LEN 6
typedef uint8_t BD_ADDR[BD_ADDR_LEN];
#include "wiced_bt_spp.h"
#include "app_ofu.h"
#include "app_ofu_spp.h"
#include "app_ofu_srv.h"
#include "app_trace.h"
#include <wiced_bt_ota_firmware_upgrade.h>

/*
 * Definitions
 */
/* Max TX packet to be sent over SPP */
#define APP_OFU_SPP_MTU             1021

typedef struct
{
    uint8_t connected;
    uint16_t handle;
    uint8_t ofu_started;
    app_ofu_callback_t *p_callback;
} app_ofu_spp_cb_t;

/*
 * Local functions
 */
static void app_ofu_spp_app_callback(app_ofu_event_t event);
static void app_ofu_spp_connection_up_callback(uint16_t handle, uint8_t *bda);
static void app_ofu_spp_connection_down_callback(uint16_t handle);
static wiced_bool_t app_ofu_spp_rx_data_callback(uint16_t handle, uint8_t *p_data, uint32_t length);
static wiced_result_t app_ofu_spp_send_status(uint16_t handle, uint8_t evt_status);

/*
 * Global variables
 */
static app_ofu_spp_cb_t app_ofu_spp_cb;
static const wiced_bt_spp_reg_t app_ofu_spp_reg =
{
    APP_OFU_SPP_RFCOMM_SCN,                 /* RFCOMM service channel number for SPP connection */
    APP_OFU_SPP_MTU,                        /* RFCOMM MTU for SPP connection */
    app_ofu_spp_connection_up_callback,     /* SPP connection established */
    NULL,                                   /* SPP connection establishment failed, not used */
                                            /* because this app never initiates connection */
    NULL,                                   /* SPP service not found, not used because this app */
                                            /* never initiates connection */
    app_ofu_spp_connection_down_callback,   /* SPP connection disconnected */
    app_ofu_spp_rx_data_callback,           /* Data packet received */
};

/*
 * app_ofu_init
 */
wiced_result_t app_ofu_spp_init(app_ofu_callback_t *p_callback)
{
    wiced_result_t status;

    APP_OFU_TRACE_DBG("\n");

    memset(&app_ofu_spp_cb, 0, sizeof(app_ofu_spp_cb));

    app_ofu_spp_cb.p_callback = p_callback;

    /* Initialize SPP library */
    wiced_bt_spp_startup((wiced_bt_spp_reg_t *)&app_ofu_spp_reg);

    return WICED_BT_SUCCESS;
}

/*
 * app_ofu_spp_ready_to_switch
 */
wiced_bool_t app_ofu_spp_ready_to_switch(void)
{
    if (app_ofu_spp_cb.connected)
    {
        APP_TRACE_ERR("OFU SPP Connected. PS-Switch not allowed\n");
        return WICED_FALSE;
    }

   return WICED_TRUE;
}

/*
 * app_ofu_spp_app_callback
 * This function is called to send OFU Events (Started/Completed/Aborted) to application
 */
static void app_ofu_spp_app_callback(app_ofu_event_t event)
{
    app_ofu_event_data_t event_data;

    if (app_ofu_spp_cb.p_callback == NULL)
    {
        APP_TRACE_ERR("No Callback\n");
        return;
    }

    switch(event)
    {
    case APP_OFU_EVENT_STARTED:
        app_ofu_spp_cb.ofu_started = WICED_TRUE;
        event_data.started.transport = APP_OFU_TRANSPORT_SPP_SERVER;
        break;

    case APP_OFU_EVENT_COMPLETED:
        app_ofu_spp_cb.ofu_started = WICED_FALSE;
        event_data.completed.transport = APP_OFU_TRANSPORT_SPP_SERVER;
        break;

    case APP_OFU_EVENT_ABORTED:
        app_ofu_spp_cb.ofu_started = WICED_FALSE;
        event_data.aborted.transport = APP_OFU_TRANSPORT_SPP_SERVER;
        break;

    default:
        APP_TRACE_ERR("Unknown event:%d\n", event);
        return;
    }

    /* Send OFU Event to application */
    app_ofu_spp_cb.p_callback(event, &event_data);
}

/*
 * app_ofu_spp_connection_up_callback
 */
static void app_ofu_spp_connection_up_callback(uint16_t handle, uint8_t* bda)
{
    APP_OFU_TRACE_DBG("handle:%d address:%B\n", handle, bda);
    app_ofu_spp_cb.connected = WICED_TRUE;
    app_ofu_spp_cb.handle = handle;
}

/*
 * app_ofu_spp_connection_down_callback
 */
static void app_ofu_spp_connection_down_callback(uint16_t handle)
{
    APP_OFU_TRACE_DBG("handle:%d\n", handle);
    app_ofu_spp_cb.connected = WICED_FALSE;

    /* If SPP disconnected during OFU, send an Abort message to the app */
    if (app_ofu_spp_cb.ofu_started)
    {
        app_ofu_spp_cb.ofu_started = WICED_FALSE;
        app_ofu_spp_app_callback(APP_OFU_EVENT_ABORTED);
    }
}

/*
 * app_ofu_spp_rx_data_callback
 * Process data received from SPP session.
 */
static wiced_bool_t app_ofu_spp_rx_data_callback(uint16_t handle, uint8_t *p_data, uint32_t length)
{
    uint8_t srv_status;
    uint8_t header;
    uint16_t payload_len;

    /* Extract Packet Header */
    STREAM_TO_UINT8(header, p_data);
    length--;

    /* Extract Payload Length */
    STREAM_TO_UINT16(payload_len, p_data);
    length -= 2;
    if (payload_len != length)
    {
        APP_TRACE_ERR("wrong payload_len:%d length:%d\n", payload_len, length);

        /* Send response to SPP Client */
        app_ofu_spp_send_status(handle, WICED_OTA_UPGRADE_STATUS_BAD_PARAM);

        return WICED_TRUE;
    }

    APP_OFU_TRACE_DBG("handle:%d header:0x%x payload_len:%d data:[%02x-%02x]\n",
            handle, header, payload_len, p_data[0], p_data[payload_len - 1]);

    switch(APP_OFU_HDR_TYPE_GET(header))
    {
    case APP_OFU_CONTROL_COMMAND:
        srv_status = app_ofu_srv_command_handler(header, p_data, length, app_ofu_spp_app_callback);
        break;

    case APP_OFU_DATA:
        srv_status = app_ofu_srv_data_handler(p_data, length, app_ofu_spp_app_callback);
        break;

    default:
        APP_TRACE_ERR("Unexpected type:0x%x\n", APP_OFU_HDR_TYPE_GET(header));
        srv_status = WICED_OTA_UPGRADE_STATUS_UNSUPPORTED_COMMAND;
    }

    /* Send response to SPP Client */
    app_ofu_spp_send_status(handle, srv_status);


    return WICED_TRUE;
}

/*
 * app_ofu_spp_send_status
 */
static wiced_result_t app_ofu_spp_send_status(uint16_t handle, uint8_t evt_status)
{
    uint8_t tx_data[3];
    uint8_t *p;
    wiced_bool_t rv;

    if (app_ofu_spp_cb.connected == WICED_FALSE)
    {
        APP_TRACE_ERR("SPP Not connected\n");
        return WICED_BT_ERROR;
    }

    APP_OFU_TRACE_DBG("status:%d\n", evt_status);

    p = &tx_data[0];

    /* Write the Event Header */
    UINT8_TO_STREAM(p, APP_OFU_HDR_SET(APP_OFU_EVENT, evt_status));

    /* Write the Event Payload Length */
    UINT16_TO_STREAM(p, 0);

    /* Send the Event message over SPP */
    rv = wiced_bt_spp_send_session_data(handle, tx_data, p - tx_data);
    if (rv == WICED_FALSE)
    {
        APP_TRACE_ERR("wiced_bt_spp_send_session_data failed\n");
        return WICED_BT_ERROR;
    }

    return WICED_SUCCESS;
}
#endif
