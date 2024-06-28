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
 *  OFU (OTA FW Upgrade) Over LRAC/WASS implementation
 *
 */

#ifdef OTA_FW_UPGRADE
#include "app_ofu.h"
#include "app_ofu_lrac.h"
#include "app_ofu_srv.h"
#include "app_ofu_clt.h"
#include "app_lrac.h"
#include "app_trace.h"
#include <wiced_bt_ota_firmware_upgrade.h>

/*
 * Definitions
 */
typedef struct
{
    app_ofu_callback_t *p_callback;
} app_ofu_lrac_cb_t;

/*
 * Global variables
 */
static app_ofu_lrac_cb_t app_ofu_lrac_cb;

#ifdef APP_OFU_DEBUG
static const char *app_ofu_type[4] =
{
        "Unknown",
        "CMD",
        "DATA",
        "EVENT",
};

static const char *app_ofu_type_cmd[8] =
{
        "Unknown",
        "Prepare",
        "Download",
        "Verify",
        "Finish(Unused)",
        "GetSts(Unused)",
        "ClrSts(Unused)",
        "Abort",
};
#endif /* APP_OFU_DEBUG */
/*
 * Local Functions
 */
static wiced_result_t app_ofu_lrac_send(uint8_t header, uint8_t *p_data, uint16_t length);
static void app_ofu_lrac_server_callback(app_ofu_event_t event);
static void app_ofu_lrac_client_callback(app_ofu_event_t event);
static void app_ofu_lrac_app_callback(app_ofu_event_t event, app_ofu_transport_t transport);
#ifdef APP_OFU_DEBUG
static void app_ofu_lrac_dbg_trace(uint8_t header, uint8_t *p_data, uint16_t length);
#endif

/*
 * app_ofu_lrac_init
 */
wiced_result_t app_ofu_lrac_init(app_ofu_callback_t *p_callback)
{
    APP_OFU_TRACE_DBG("\n");

    memset(&app_ofu_lrac_cb, 0, sizeof(app_ofu_lrac_cb));

    app_ofu_lrac_cb.p_callback = p_callback;

    return WICED_BT_SUCCESS;
}

/*
 * app_ofu_lrac_rx_handler
 * This function handles data received from the peer LRAC device
 */
void app_ofu_lrac_rx_handler(uint8_t *p_data, uint16_t length)
{
    wiced_result_t status;
    uint8_t header;
    uint8_t type;
    uint16_t payload_len;
    uint8_t server_status;

    if (length == 0)
    {
        APP_TRACE_ERR("no data\n");
        return;
    }

    /* Extract Packet Header */
    STREAM_TO_UINT8(header, p_data);
    length--;

#ifdef APP_OFU_DEBUG
    app_ofu_lrac_dbg_trace(header, p_data, length);
#endif

    /* Read Packet type from Header */
    type = APP_OFU_HDR_TYPE_GET(header);

    /* If this is either a Command or Data */
    if ((type == APP_OFU_CONTROL_COMMAND) ||
        (type == APP_OFU_DATA))
    {
        /* Extract Payload Length */
        STREAM_TO_UINT16(payload_len, p_data);
        length -= 2;
        if (payload_len != length)
        {
            APP_TRACE_ERR("wrong payload_len:%d length:%d\n", payload_len, length);
            return;
        }

        switch(type)
        {
        case APP_OFU_CONTROL_COMMAND:
            server_status = app_ofu_srv_command_handler(header, p_data, length,
                    app_ofu_lrac_server_callback);
            break;

        case APP_OFU_DATA:
            server_status = app_ofu_srv_data_handler(p_data, length, app_ofu_lrac_server_callback);
            break;
        }

        /* Send Response to Peer device in every case */
        app_ofu_lrac_send(APP_OFU_HDR_SET(APP_OFU_EVENT, server_status), NULL, 0);
    }
    else if (type == APP_OFU_EVENT)
    {
        app_ofu_clt_rx_handler(header, app_ofu_lrac_client_callback, app_ofu_lrac_send);
    }
    else
    {
        APP_TRACE_ERR("Unknown type:%d )header:0x%x\n", type, header);
    }
}

/*
 * app_ofu_lrac_server_callback
 * This function is called to sends OFU Server events to application
 */
static void app_ofu_lrac_server_callback(app_ofu_event_t event)
{
    app_ofu_lrac_app_callback(event, APP_OFU_TRANSPORT_LRAC_SERVER);
}

/*
 * app_ofu_lrac_client_callback
 * This function is called to sends OFU Client events to application
 */
static void app_ofu_lrac_client_callback(app_ofu_event_t event)
{
    app_ofu_lrac_app_callback(event, APP_OFU_TRANSPORT_LRAC_CLIENT);
}

/*
 * app_ofu_lrac_app_callback
 * This function is called to sends OFU Client or Server events to application
 */
static void app_ofu_lrac_app_callback(app_ofu_event_t event, app_ofu_transport_t transport)
{
    app_ofu_event_data_t event_data;

    if (app_ofu_lrac_cb.p_callback == NULL)
    {
        APP_TRACE_ERR("No Callback\n");
        return;
    }

    switch(event)
    {
    case APP_OFU_EVENT_STARTED:
        event_data.started.transport = transport;
        break;

    case APP_OFU_EVENT_COMPLETED:
        event_data.completed.transport = transport;
        break;

    case APP_OFU_EVENT_ABORTED:
        event_data.aborted.transport = transport;
        break;

    default:
        APP_TRACE_ERR("Unknown event:%d\n", event);
        return;
    }

    /* Send OFU Event to application */
    app_ofu_lrac_cb.p_callback(event, &event_data);
}

/*
 * app_lrac_ofu_send
 * This function is called to sends OFU packets to the peer LRAC Device
 */
static wiced_result_t app_ofu_lrac_send(uint8_t header, uint8_t *p_data, uint16_t length)
{
    wiced_result_t status = WICED_BT_SUCCESS;
    uint8_t type;
    uint8_t tx_data[1021];
    uint8_t *p = &tx_data[1];   /* Skip first byte */

    /* Get Packet type from Header */
    type = APP_OFU_HDR_TYPE_GET(header);

#ifdef APP_OFU_DEBUG
    app_ofu_lrac_dbg_trace(header, p_data, length);
#endif

    /* Write the Header */
    UINT8_TO_STREAM(p, header);

    switch(type)
    {
    case APP_OFU_CONTROL_COMMAND:
    case APP_OFU_DATA:
        /* Write the Payload length */
        UINT16_TO_STREAM(p, length);
        if (length && p_data)
        {
            memcpy(p, p_data, length);
            p += length;
        }
        break;

    case APP_OFU_EVENT:
        /* No Payload length for Event packets */
        break;

    default:
        APP_TRACE_ERR("Unknown type:%d\n", type);
        return WICED_BT_ERROR;
    }

    /* Send the OFU Packet to the peer LRAC device */
    status = app_lrac_send_ofu(tx_data, p - tx_data);

    return status;
}

/*
 * app_ofu_lrac_start
 * This function is called to start OFU to the peer LRAC Device (as Client)
 */
wiced_result_t app_ofu_lrac_start(uint16_t mtu)
{
    APP_OFU_TRACE_DBG("mtu:%d\n", mtu);

    return app_ofu_clt_start(mtu, app_ofu_lrac_client_callback, app_ofu_lrac_send);
}

#ifdef APP_OFU_DEBUG
/*
 * app_ofu_lrac_dbg_trace
 */
static void app_ofu_lrac_dbg_trace(uint8_t header, uint8_t *p_data, uint16_t length)
{
    char const *p_type;
    char const *p_param;
    uint8_t type;
    uint8_t param;

    /* Get Packet type from Header */
    type = APP_OFU_HDR_TYPE_GET(header);

    if (type > APP_OFU_EVENT)
        p_type = app_ofu_type[0];
    else
        p_type = app_ofu_type[type];

    if (type == APP_OFU_CONTROL_COMMAND)
    {
        param = APP_OFU_HDR_CMD_GET(header);
        if (param > WICED_OTA_UPGRADE_COMMAND_ABORT)
            p_param = app_ofu_type_cmd[0];
        else
            p_param = app_ofu_type_cmd[param];
        APP_OFU_TRACE_DBG("%s/%s len:%d\n", p_type, p_param, length);
    }
    else
    {
        APP_OFU_TRACE_DBG("%s len:%d\n", p_type, length);
    }
}
#endif /* APP_OFU_DEBUG */
#endif
