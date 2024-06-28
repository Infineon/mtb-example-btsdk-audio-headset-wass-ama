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
 *  OFU (OTA FW Upgrade) Client implementation
 */
#ifdef OTA_FW_UPGRADE
#include "app_ofu.h"
#include "app_ofu_clt.h"
#include "app_lrac.h"
#include "app_trace.h"
#include <wiced_bt_ota_firmware_upgrade.h>
#include <wiced_firmware_upgrade.h>
#include <wiced_timer.h>

/*
 * Definitions
 */
#define APP_OFU_CLT_REQ_TIMER_DURATION      5   /* OFU Request Timeout */

typedef enum
{
    APP_OFU_CLT_STATE_IDLE = 0,
    APP_OFU_CLT_STATE_PREPARING,
    APP_OFU_CLT_STATE_CONFIGURING,
    APP_OFU_CLT_STATE_DATA_TRANSFER,
    APP_OFU_CLT_STATE_VERIFYING,
    APP_OFU_CLT_STATE_ABORTING,
} app_ota_clt_state_t;

typedef struct
{
    app_ota_clt_state_t state;
    uint32_t active_ds_length;
    uint32_t current_offset;
    uint16_t mtu;
    uint32_t crc32;
#ifdef APP_OFU_DEBUG
    int nb_tx_data_packet;
#endif
    wiced_timer_t req_timer;
    app_ofu_clt_app_callback_t *p_app_callback;
    app_ofu_clt_send_callback_t *p_send_callback;
} app_ofu_clt_cb_t;

/*
 * Global variables
 */
static app_ofu_clt_cb_t app_ofu_clt_cb;
static const uint8_t ds_image_prefix[8] = {'B', 'R', 'C', 'M', 'c', 'f', 'g', 'D'};

/*
 * Local functions
 */
static void app_ofu_clt_configure(app_ofu_clt_app_callback_t *p_app_callback,
        app_ofu_clt_send_callback_t *p_send_callback);
static void app_ofu_clt_download(app_ofu_clt_app_callback_t *p_app_callback,
        app_ofu_clt_send_callback_t *p_send_callback);
static void app_ofu_clt_abort(app_ofu_clt_app_callback_t *p_app_callback,
        app_ofu_clt_send_callback_t *p_send_callback);
static wiced_result_t app_ofu_clt_active_partition_info_get(void);
static uint32_t app_ofu_clt_crc32_update(uint32_t crc32, uint8_t *p_data, uint16_t length);
static void app_ofu_clt_req_timer_start(app_ofu_clt_app_callback_t *p_app_callback,
        app_ofu_clt_send_callback_t *p_send_callback);
static void app_ofu_clt_req_timer_stop(void);
static void app_ofu_clt_req_timeout(uint32_t param);

/*
 * External functions
 */
uint32_t crc32_Update(uint32_t crc, uint8_t *buf, uint16_t len);
extern uint32_t wiced_firmware_upgrade_retrieve_from_active_ds(uint32_t offset, uint8_t *data,
        uint32_t length);

/*
 * app_ofu_clt_init
 */
wiced_result_t app_ofu_clt_init(void)
{
    memset(&app_ofu_clt_cb, 0, sizeof(app_ofu_clt_cb));

    /* Initialize the Request Timer (in case peer device does not reply to a Request) */
    wiced_init_timer(&app_ofu_clt_cb.req_timer, app_ofu_clt_req_timeout,
            0, WICED_SECONDS_TIMER);

    return WICED_BT_SUCCESS;
}

/*
 * app_ofu_clt_start
 * This function is used to Start OFU (as Client)
 */
wiced_result_t app_ofu_clt_start(uint16_t mtu, app_ofu_clt_app_callback_t *p_app_callback,
        app_ofu_clt_send_callback_t *p_send_callback)
{
    wiced_result_t status;
    uint8_t header;

    APP_OFU_TRACE_DBG("mtu:%d\n", mtu);

    if (app_ofu_clt_cb.state == APP_OFU_CLT_STATE_ABORTING)
    {
        /* Handle case where the previous OFU was stuck in Aborting state */
        APP_TRACE_ERR("OFU was aborting\n");
    }

    if (app_ofu_clt_cb.state != APP_OFU_CLT_STATE_IDLE)
    {
        APP_TRACE_ERR("Wrong state:%d\n", app_ofu_clt_cb.state);
        return WICED_BT_BUSY;
    }

    /* 4 header Bytes will be added (LRAC/OFU/Len) to Data sent */
    app_ofu_clt_cb.mtu = mtu - 4;
    app_ofu_clt_cb.mtu &= ~0x3;

    header = APP_OFU_HDR_SET(APP_OFU_CONTROL_COMMAND, WICED_OTA_UPGRADE_COMMAND_PREPARE_DOWNLOAD);
    status = p_send_callback(header, NULL, 0);
    if (status == WICED_BT_SUCCESS)
    {
        app_ofu_clt_cb.state = APP_OFU_CLT_STATE_PREPARING;

        /* Tell the application OFU is Started (as LRAC Client) */
        p_app_callback(APP_OFU_EVENT_STARTED);

        /* Start a Request timer */
        app_ofu_clt_req_timer_start(p_app_callback, p_send_callback);
    }
    else
    {
        app_ofu_clt_cb.state = APP_OFU_CLT_STATE_IDLE;
    }

    return status;
}

/*
 * app_ofu_clt_rx_handler
 * This function is used handle Rx data (as OFU Client)
 */
void app_ofu_clt_rx_handler(uint8_t header, app_ofu_clt_app_callback_t *p_app_callback,
        app_ofu_clt_send_callback_t *p_send_callback)
{
    uint8_t rcv_status;

    rcv_status = APP_OFU_HDR_STS_GET(header);
    APP_OFU_TRACE_DBG("rcv_status:%d state:%d\n", rcv_status, app_ofu_clt_cb.state);

    switch(app_ofu_clt_cb.state)
    {
    case APP_OFU_CLT_STATE_IDLE:
        APP_TRACE_ERR("Unexpected data:0x%x received in Idle\n", header);
        break;

    case APP_OFU_CLT_STATE_PREPARING:
        if (rcv_status != WICED_OTA_UPGRADE_STATUS_OK)
        {
            APP_TRACE_ERR("Peer OFU LRAC Server rejected OFU Prepare Cmd (status:%d)\n",
                    rcv_status);
            app_ofu_clt_abort(p_app_callback, p_send_callback);
            return;
        }
        app_ofu_clt_configure(p_app_callback, p_send_callback);
        break;

    case APP_OFU_CLT_STATE_CONFIGURING:
        if (rcv_status != WICED_OTA_UPGRADE_STATUS_OK)
        {
            APP_TRACE_ERR("Peer OFU LRAC Server rejected OFU Configure Cmd (status:%d)\n",
                    rcv_status);
            app_ofu_clt_abort(p_app_callback, p_send_callback);
            return;
        }
        app_ofu_clt_download(p_app_callback, p_send_callback);
        break;

    case APP_OFU_CLT_STATE_DATA_TRANSFER:
        if ((rcv_status != WICED_OTA_UPGRADE_STATUS_OK) &&
            (rcv_status != WICED_OTA_UPGRADE_STATUS_CONTINUE))
        {
            APP_TRACE_ERR("Peer OFU LRAC Server rejected OFU Data (status:%d)\n", rcv_status);
            app_ofu_clt_abort(p_app_callback, p_send_callback);
            return;
        }
        app_ofu_clt_download(p_app_callback, p_send_callback);
        break;

    case APP_OFU_CLT_STATE_VERIFYING:
        /* Stop the Request timer */
        app_ofu_clt_req_timer_stop();

        if (rcv_status != WICED_OTA_UPGRADE_STATUS_OK)
        {
            APP_TRACE_ERR("Peer OFU LRAC Server rejected OFU Verify (CRC) Cmd (status:%d)\n",
                    rcv_status);
            app_ofu_clt_abort(p_app_callback, p_send_callback);
            return;
        }
        APP_OFU_TRACE_DBG("OFU Verified. Peer device will reboot\n");
        app_ofu_clt_cb.state = APP_OFU_CLT_STATE_IDLE;
        /* Tell the application OFU is Completed */
        p_app_callback(APP_OFU_EVENT_COMPLETED);
        break;

    case APP_OFU_CLT_STATE_ABORTING:
        /* Stop the Request timer */
        app_ofu_clt_req_timer_stop();

        app_ofu_clt_cb.state = APP_OFU_CLT_STATE_IDLE;
        break;

    default:
        APP_TRACE_ERR("Wrong state:%d\n", app_ofu_clt_cb.state);
        app_ofu_clt_abort(p_app_callback, p_send_callback);
        break;
    }
}

/*
 * app_ofu_clt_send_configure_send
 */
static void app_ofu_clt_configure(app_ofu_clt_app_callback_t *p_app_callback,
        app_ofu_clt_send_callback_t *p_send_callback)
{
    wiced_result_t status;
    uint8_t header;
    uint8_t tx_data[sizeof(uint32_t)];
    uint8_t *p;

    status = app_ofu_clt_active_partition_info_get();
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("app_ofu_clt_active_partition_info_get failed\n");
        app_ofu_clt_abort(p_app_callback, p_send_callback);
        return;
    }

    app_ofu_clt_cb.state = APP_OFU_CLT_STATE_CONFIGURING;
    p = tx_data;
    UINT32_TO_STREAM(p, app_ofu_clt_cb.active_ds_length);

    /* Send Configure Command to peer LRAC device */
    header = APP_OFU_HDR_SET(APP_OFU_CONTROL_COMMAND, WICED_OTA_UPGRADE_COMMAND_DOWNLOAD);
    status = p_send_callback(header, tx_data, p - tx_data);
    if (status != WICED_BT_SUCCESS)
    {
        app_ofu_clt_abort(p_app_callback, p_send_callback);
        return;
    }

    /* Start a Request timer */
    app_ofu_clt_req_timer_start(p_app_callback, p_send_callback);

    app_ofu_clt_cb.current_offset = 0;
    app_ofu_clt_cb.crc32 = 0xFFFFFFFF;
}

/*
 * app_ofu_clt_download
 */
static void app_ofu_clt_download(app_ofu_clt_app_callback_t *p_app_callback,
        app_ofu_clt_send_callback_t *p_send_callback)
{
    wiced_result_t status;
    uint8_t header;
    uint8_t tx_data[1021];
    uint8_t *p;
    uint32_t bytes_to_send;
    uint32_t read_length;
    uint32_t bytes_to_read;

    bytes_to_send = app_ofu_clt_cb.active_ds_length - app_ofu_clt_cb.current_offset;
    if (bytes_to_send > app_ofu_clt_cb.mtu)
    {
        bytes_to_send = app_ofu_clt_cb.mtu;
    }

    if (bytes_to_send > 0)
    {
        app_ofu_clt_cb.state = APP_OFU_CLT_STATE_DATA_TRANSFER;

        /* The NVRAM Read must be a multiple of 4 bytes */
        bytes_to_read = bytes_to_send + 3;
        bytes_to_read &= 0xFFFC;

        read_length = wiced_firmware_upgrade_retrieve_from_active_ds(
                app_ofu_clt_cb.current_offset, tx_data, bytes_to_read);
        if ((read_length == 0) ||
            (read_length != bytes_to_read))
        {
            APP_TRACE_ERR("wiced_firmware_upgrade_retrieve_from_active_ds failed\n");
            app_ofu_clt_abort(p_app_callback, p_send_callback);
            return;
        }
        app_ofu_clt_cb.current_offset += bytes_to_send;

        /* Update CRC on the fly */
        app_ofu_clt_cb.crc32 = app_ofu_clt_crc32_update(app_ofu_clt_cb.crc32, tx_data,
                bytes_to_send);

        APP_OFU_TRACE_DBG("progress:%d/%d\n", app_ofu_clt_cb.current_offset,
                    app_ofu_clt_cb.active_ds_length);

        /* Send Data to peer LRAC device */
        header = APP_OFU_HDR_SET(APP_OFU_DATA, 0);
        status = p_send_callback(header, tx_data, bytes_to_send);
        if (status == WICED_BT_SUCCESS)
        {
            /* Start a Request timer */
            app_ofu_clt_req_timer_start(p_app_callback, p_send_callback);
        }
        else
        {
            app_ofu_clt_abort(p_app_callback, p_send_callback);
        }
    }
    else
    {
        app_ofu_clt_cb.state = APP_OFU_CLT_STATE_VERIFYING;

        app_ofu_clt_cb.crc32 ^= 0xFFFFFFFF;
        p = tx_data;
        UINT32_TO_STREAM(p, app_ofu_clt_cb.crc32);

        APP_OFU_TRACE_DBG("Send Verify cmd CRC32:0x%X\n", app_ofu_clt_cb.crc32);

        /* Send Data to peer LRAC device */
        header = APP_OFU_HDR_SET(APP_OFU_CONTROL_COMMAND, WICED_OTA_UPGRADE_COMMAND_VERIFY);
        status = p_send_callback(header, tx_data, p - tx_data);
        if (status == WICED_BT_SUCCESS)
        {
            /* Start a Request timer */
            app_ofu_clt_req_timer_start(p_app_callback, p_send_callback);
        }
        else
        {
            app_ofu_clt_abort(p_app_callback, p_send_callback);
        }
    }
}

/*
 * app_ofu_clt_abort
 */
static void app_ofu_clt_abort(app_ofu_clt_app_callback_t *p_app_callback,
        app_ofu_clt_send_callback_t *p_send_callback)
{
    wiced_result_t status;
    uint8_t header;

    app_ofu_clt_cb.state = APP_OFU_CLT_STATE_ABORTING;

    /* Send the Abort command to peer device */
    header = APP_OFU_HDR_SET(APP_OFU_CONTROL_COMMAND, WICED_OTA_UPGRADE_COMMAND_ABORT);
    status = p_send_callback(header, NULL, 0);
    if (status == WICED_BT_SUCCESS)
    {
        /* Start a Request timer */
        app_ofu_clt_req_timer_start(p_app_callback, p_send_callback);
    }
    else
    {
        app_ofu_clt_cb.state = APP_OFU_CLT_STATE_IDLE;
    }

    /* Tell the application OFU is Aborted (as LRAC Client) */
    p_app_callback(APP_OFU_EVENT_ABORTED);
}

/*
 * app_ofu_clt_active_partition_info_get
 */
static wiced_result_t app_ofu_clt_active_partition_info_get(void)
{
    uint32_t active_ds_address;
    uint32_t active_ds_length;
    uint8_t *p;
    uint32_t u32;
    uint8_t data_chunk[4 * sizeof(uint32_t)];
    uint32_t read_length;

    if (!wiced_firmware_upgrade_init_nv_locations())
    {
        APP_TRACE_ERR("failed init nv locations\n");
        return WICED_BT_ERROR;
    }

    /* Read the the First 16 bytes (Header) of the Active Data Partition */
    read_length = wiced_firmware_upgrade_retrieve_from_active_ds(0, data_chunk, sizeof(data_chunk));
    if (read_length != sizeof(data_chunk))
    {
        APP_TRACE_ERR("wiced_firmware_upgrade_retrieve_from_active_ds failed\n");
        return WICED_BT_ERROR;
    }
    p = data_chunk;

    /* Check if the Data Section contains the Image Prefix */
    if (memcmp(p, ds_image_prefix, sizeof(ds_image_prefix)) != 0)
    {
        APP_TRACE_ERR("\n");
        return WICED_BT_ERROR;
    }
    p += sizeof(ds_image_prefix);

    /* Extract the CRC (which should be 0 because it's unused) */
    STREAM_TO_UINT32(u32, p);
    if (u32 != 0)
    {
        APP_TRACE_ERR("CRC:0x%x not 0\n", u32);
        /* Do not return (just a warning) */
    }

    /* Read Real DS Section size */
    STREAM_TO_UINT32(u32, p);
    /* FW image cannot be bigger than half of the Flash's size */
    if (u32 > (512 * 1024))
    {
        APP_TRACE_ERR("Wrong size:%d\n", u32);
        return WICED_BT_ERROR;
    }

    /* Save the DS Length */
    app_ofu_clt_cb.active_ds_length = u32 + (4 * sizeof(uint32_t));
    APP_OFU_TRACE_DBG("Active DS Length:%d\n", app_ofu_clt_cb.active_ds_length);

    return WICED_BT_SUCCESS;
}

/*
 * app_ofu_clt_crc32_update
 */
static uint32_t app_ofu_clt_crc32_update(uint32_t crc32, uint8_t *p_data, uint16_t length)
{
    return crc32_Update(crc32, p_data, length);
}

/*
 * app_ofu_clt_req_timer_start
 */
static void app_ofu_clt_req_timer_start(app_ofu_clt_app_callback_t *p_app_callback,
        app_ofu_clt_send_callback_t *p_send_callback)
{
    /* Stop the Request timer (in case it was already running */
    wiced_stop_timer(&app_ofu_clt_cb.req_timer);

    /* Save the Callback functions */
    app_ofu_clt_cb.p_app_callback = p_app_callback;
    app_ofu_clt_cb.p_send_callback = p_send_callback;

    wiced_start_timer(&app_ofu_clt_cb.req_timer, APP_OFU_CLT_REQ_TIMER_DURATION);
}

/*
 * app_ofu_clt_req_timer_stop
 */
static void app_ofu_clt_req_timer_stop(void)
{
    /* Stop the Request timer (in case it was already running */
    wiced_stop_timer(&app_ofu_clt_cb.req_timer);
}

/*
 * app_ofu_clt_req_timeout
 */
static void app_ofu_clt_req_timeout(uint32_t param)
{
    uint8_t header;

    APP_TRACE_ERR("\n");

    if (app_ofu_clt_cb.p_app_callback)
    {
        /* Tell the application OFU is Aborted */
        app_ofu_clt_cb.p_app_callback(APP_OFU_EVENT_ABORTED);
    }

    if (app_ofu_clt_cb.p_send_callback)
    {
        app_ofu_clt_cb.state = APP_OFU_CLT_STATE_IDLE;

        /* Send an Abort command. Do not use  app_ofu_clt_abort() to prevent forever loop */
        header = APP_OFU_HDR_SET(APP_OFU_CONTROL_COMMAND, WICED_OTA_UPGRADE_COMMAND_ABORT);
        app_ofu_clt_cb.p_send_callback(header, NULL, 0);
    }
}
#endif
