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
 *  OFU (OTA FW Upgrade) Server implementation.
 */
#ifdef OTA_FW_UPGRADE
#include <string.h>
#include "app_ofu.h"
#include "app_ofu_srv.h"
#include "p_256_ecc_pp.h"
#include "sha256.h"
#include "app_trace.h"
#include <ota_fw_upgrade.h>
#include <wiced_firmware_upgrade.h>
#include <wiced_timer.h>

/*
 * Definitions
 */
typedef enum
{
    APP_OFU_SRV_STATE_IDLE = 0,
    APP_OFU_SRV_STATE_READY_FOR_DOWNLOAD,
    APP_OFU_SRV_STATE_DATA_TRANSFER,
    APP_OFU_SRV_STATE_VERIFIED,
    APP_OFU_SRV_STATE_ABORTED,
} app_ota_srv_state_t;

typedef struct
{
    app_ota_srv_state_t state;
    uint16_t        current_offset;          /* Offset in the image to store the data */
    int32_t         current_block_offset;
    int32_t         total_offset;
#ifdef APP_OFU_DEBUG
    uint32_t        recv_crc32;
    int             nb_rx_data_packet;
#endif
    wiced_timer_t   reset_timer;
    uint8_t         read_buffer[OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT];
} app_ofu_srv_cb_t;

/*
 * Global variables
 */
static app_ofu_srv_cb_t app_ofu_srv_cb;

/*
 * External definitions
 */
#ifdef APP_OFU_DEBUG
uint32_t crc32_Update(uint32_t crc, uint8_t *p_buf, uint16_t len);
#endif

/*
 * Local functions
 */
static int32_t app_ofu_srv_verify_secure(int32_t total_len);
static void app_ofu_srv_abort(app_ofu_srv_app_callback_t *p_callback);
static void app_ofu_srv_reset_timeout(uint32_t param);
#ifdef APP_OFU_DEBUG
static uint32_t app_ofu_srv_crc32_update(uint32_t crc32, uint8_t *p_data, uint16_t length);
#endif
/*
 * app_ofu_srv_init
 */
wiced_result_t app_ofu_srv_init(void)
{
    APP_OFU_TRACE_DBG("\n");

    memset(&app_ofu_srv_cb, 0, sizeof(app_ofu_srv_cb));

    if (!wiced_ota_fw_upgrade_init(NULL, NULL, NULL))
    {
        APP_TRACE_ERR("wiced_ota_fw_upgrade_init failed\n");
        return WICED_BT_ERROR;
    }

    /* Initialize the Reset Timer (used after OFU Complete) */
    wiced_init_timer(&app_ofu_srv_cb.reset_timer, app_ofu_srv_reset_timeout,
            0, WICED_SECONDS_TIMER);

    return WICED_BT_SUCCESS;
}

/*
 * app_ofu_srv_command_handler
 */
uint8_t app_ofu_srv_command_handler(uint8_t header, uint8_t *p_data, uint16_t length,
        app_ofu_srv_app_callback_t *p_app_callback)
{
    uint8_t value = WICED_OTA_UPGRADE_STATUS_OK;
    int32_t verified = WICED_FALSE;
    uint8_t command;
    uint16_t payload_len;

    /* Extract Command from Header */
    command = APP_OFU_HDR_CMD_GET(header);

    APP_OFU_TRACE_DBG("cmd:%d len:%d state:%d\n", command, length, app_ofu_srv_cb.state);

    switch(command)
    {
    case WICED_OTA_UPGRADE_COMMAND_PREPARE_DOWNLOAD:
        APP_OFU_TRACE_DBG("Cmd: PrepareDownload\n");
#ifdef APP_OFU_DEBUG
        app_ofu_srv_cb.nb_rx_data_packet = 0;
#endif
        app_ofu_srv_cb.state = APP_OFU_SRV_STATE_READY_FOR_DOWNLOAD;
        /* Tell the app that OFU is Started */
        p_app_callback(APP_OFU_EVENT_STARTED);
        return WICED_OTA_UPGRADE_STATUS_OK;

    case WICED_OTA_UPGRADE_COMMAND_ABORT:
        APP_OFU_TRACE_DBG("Cmd: Abort\n");
        app_ofu_srv_abort(p_app_callback);
        return WICED_OTA_UPGRADE_STATUS_OK;
    }

    switch (app_ofu_srv_cb.state)
    {
    case APP_OFU_SRV_STATE_IDLE:
        return WICED_OTA_UPGRADE_STATUS_OK;

    case APP_OFU_SRV_STATE_READY_FOR_DOWNLOAD:
        if (command == WICED_OTA_UPGRADE_COMMAND_DOWNLOAD)
        {
            APP_OFU_TRACE_DBG("Cmd: Download\n");
            /* command to start upgrade should be accompanied by 4 bytes with the image size */
            if (length < 4)
            {
                APP_TRACE_ERR("Bad Download len:%d\n", length);
                app_ofu_srv_abort(p_app_callback);
                return WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE;
            }

            /* Extract Download file size */
            STREAM_TO_UINT32(ota_fw_upgrade_state.total_len, p_data);
            APP_OFU_TRACE_DBG("Download len:%d\n", ota_fw_upgrade_state.total_len);

            if (!wiced_firmware_upgrade_init_nv_locations())
            {
                APP_TRACE_ERR("failed init nv locations\n");
                app_ofu_srv_abort(p_app_callback);
                return WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE;
            }

            app_ofu_srv_cb.state                = APP_OFU_SRV_STATE_DATA_TRANSFER;
            app_ofu_srv_cb.current_offset       = 0;
            app_ofu_srv_cb.current_block_offset = 0;
            app_ofu_srv_cb.total_offset         = 0;

#ifdef APP_OFU_DEBUG
            app_ofu_srv_cb.recv_crc32           = 0xFFFFFFFF;
#endif

#if ( defined(CYW20719B0) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20721B2) )
            /*
             * if we are using Secure version the total length comes in the beginning of the image,
             * do not use the one from the downloader.
             */
            if (p_ecdsa_public_key != NULL)
            {
                ota_fw_upgrade_state.total_len = 0;
            }
#endif
            return WICED_OTA_UPGRADE_STATUS_OK;
        }
        else
        {
            APP_TRACE_ERR("Unexpected command%d state:%d\n", command, app_ofu_srv_cb.state);
            app_ofu_srv_abort(p_app_callback);
            return WICED_OTA_UPGRADE_STATUS_ILLEGAL_STATE;
        }
        break;

    case APP_OFU_SRV_STATE_DATA_TRANSFER:
        if (command == WICED_OTA_UPGRADE_COMMAND_VERIFY)
        {
            /* command to perform verification */
            if (ota_fw_upgrade_state.total_len != app_ofu_srv_cb.total_offset)
            {
                APP_TRACE_ERR("Verify failed received:%d out of %d\n",
                        app_ofu_srv_cb.total_offset, ota_fw_upgrade_state.total_len);
                app_ofu_srv_abort(p_app_callback);
                return WICED_OTA_UPGRADE_STATUS_VERIFICATION_FAILED;
            }

            /* For none-secure case the command should have 4 bytes CRC32 */
            if (p_ecdsa_public_key == NULL)
            {
                /* command to start upgrade should be accompanied by 4 bytes with the image size */
                if (length < 4)
                {
                    app_ofu_srv_abort(p_app_callback);
                    return WICED_OTA_UPGRADE_STATUS_BAD_PARAM;
                }
                STREAM_TO_UINT32(ota_fw_upgrade_state.crc32, p_data);
                verified = ota_fw_upgrade_verify();
            }
            else
            {
                APP_OFU_TRACE_DBG("app_ofu_srv_verify_secure() \n");
                verified = ota_sec_fw_upgrade_verify();
            }

            if (!verified)
            {
                APP_TRACE_ERR("Verify failed\n");
                app_ofu_srv_abort(p_app_callback);
                return WICED_OTA_UPGRADE_STATUS_VERIFICATION_FAILED;
            }

            APP_OFU_TRACE_DBG("Verify success\n");
            app_ofu_srv_cb.state = APP_OFU_SRV_STATE_VERIFIED;

            APP_OFU_TRACE_DBG("Starting Reset timer\n");
            wiced_start_timer(&app_ofu_srv_cb.reset_timer, 1);

            /* Tell the app that OFU is Complete */
            p_app_callback(APP_OFU_EVENT_COMPLETED);
            app_ofu_srv_cb.state = APP_OFU_SRV_STATE_IDLE;
            return WICED_OTA_UPGRADE_STATUS_OK;
        }
        else
        {
            APP_TRACE_ERR("Unexpected command%d state:%d\n", command, app_ofu_srv_cb.state);
            return WICED_OTA_UPGRADE_STATUS_ILLEGAL_STATE;
        }
        break;

    case APP_OFU_SRV_STATE_ABORTED:
    default:
        break;
    }

    app_ofu_srv_abort(p_app_callback);
    return WICED_OTA_UPGRADE_STATUS_ILLEGAL_STATE;
}

/*
 * app_ofu_srv_data_handler
 */
uint8_t app_ofu_srv_data_handler(uint8_t *p_data, uint16_t length,
        app_ofu_srv_app_callback_t *p_app_callback)
{
    uint8_t *p;
    int bytes_to_copy;
    uint16_t payload_len;
    uint32_t nb_wrote;

#ifndef CYW20706A2
    uint16_t image_product_id;
    uint8_t  image_major, image_minor;
#endif

#ifdef APP_OFU_DEBUG
    app_ofu_srv_cb.nb_rx_data_packet++;
    APP_OFU_TRACE_DBG("length:%d data:[%02X..%02X] nb_rx_data_packet:%d\n",
            length, p_data[0], p_data[length - 1], app_ofu_srv_cb.nb_rx_data_packet);
#endif

    p = p_data;

    if (app_ofu_srv_cb.state != APP_OFU_SRV_STATE_DATA_TRANSFER)
    {
        app_ofu_srv_abort(p_app_callback);
        return WICED_OTA_UPGRADE_STATUS_ILLEGAL_STATE;
    }

/* Image prefixes are not supported on 20706 */
#ifndef CYW20706A2
    /* For the Secure upgrade, verify the Product info */
    if (p_ecdsa_public_key != NULL)
    {
        /*
         * If this is the first chunk of the image, we need to verify the header and extract
         * the length. Following check is for the FW2 */
        if (ota_fw_upgrade_state.total_len == 0)
        {
            if (memcmp(p, ds_image_prefix, sizeof(ds_image_prefix)) != 0)
            {
                APP_TRACE_ERR("Bad data start\n");
                app_ofu_srv_abort(p_app_callback);
                return WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE;
            }
            p += sizeof(ds_image_prefix);
            STREAM_TO_UINT16(image_product_id, p);
            UNUSED_VARIABLE(image_product_id);
            STREAM_TO_UINT8(image_major, p);
            UNUSED_VARIABLE(image_major);
            STREAM_TO_UINT8(image_minor, p);
            UNUSED_VARIABLE(image_minor);

            /* length store in the image does not include size of ds_image_prefix */
            STREAM_TO_UINT32(ota_fw_upgrade_state.total_len, p);
            ota_fw_upgrade_state.total_len += DS_IMAGE_PREFIX_LEN + SIGNATURE_LEN;

            /*
             * ToDo validate flash size
             * ToDo validate that product is the same as stored and major is >= the one that stored
             */
            APP_OFU_TRACE_DBG("Image for Product 0x%x %d.%d len:%d\n",
                    image_product_id, image_major, image_minor, ota_fw_upgrade_state.total_len);
        }
    }
    else
#endif
    {
#ifdef APP_OFU_DEBUG
        /* For testing calculate received CRC32 of the received data */
        app_ofu_srv_cb.recv_crc32 = app_ofu_srv_crc32_update(app_ofu_srv_cb.recv_crc32, p_data,
                length);
#endif
    }

    p = p_data;

    while (length)
    {
        if ((app_ofu_srv_cb.current_block_offset + length) < OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT)
        {
            bytes_to_copy = length;
        }
        else
        {
            bytes_to_copy = OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT;
            bytes_to_copy -= app_ofu_srv_cb.current_block_offset;
        }

        if ((app_ofu_srv_cb.total_offset + app_ofu_srv_cb.current_block_offset + bytes_to_copy >
                ota_fw_upgrade_state.total_len))
        {
            APP_TRACE_ERR("Too much data. size of the image %d offset %d, block offset %d len rcvd %d\n",
                    ota_fw_upgrade_state.total_len, app_ofu_srv_cb.total_offset,
                    app_ofu_srv_cb.current_block_offset, length);
            return WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE_SIZE;
        }

        memcpy(&(app_ofu_srv_cb.read_buffer[app_ofu_srv_cb.current_block_offset]), p, bytes_to_copy);
        app_ofu_srv_cb.current_block_offset += bytes_to_copy;

        if ((app_ofu_srv_cb.current_block_offset == OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT) ||
            (app_ofu_srv_cb.total_offset + app_ofu_srv_cb.current_block_offset == ota_fw_upgrade_state.total_len))
        {
            APP_OFU_TRACE_DBG("write offset:0x%x\n", app_ofu_srv_cb.total_offset);

            /* write should be on the word boundary and in full words, we may write a bit more */
            nb_wrote = wiced_firmware_upgrade_store_to_nv(app_ofu_srv_cb.total_offset,
                    app_ofu_srv_cb.read_buffer,
                    (app_ofu_srv_cb.current_block_offset + 3) & 0xFFFFFFFC);
            if (nb_wrote != ((app_ofu_srv_cb.current_block_offset + 3) & 0xFFFFFFFC))
            {
                APP_TRACE_ERR("wiced_firmware_upgrade_store_to_nv failed\n");
                app_ofu_srv_abort(p_app_callback);
                return WICED_OTA_UPGRADE_STATUS_ILLEGAL_STATE;
            }
            app_ofu_srv_cb.total_offset += app_ofu_srv_cb.current_block_offset;
            app_ofu_srv_cb.current_block_offset = 0;

#ifdef APP_OFU_DEBUG
            APP_OFU_TRACE_DBG("progress:%d/%d\n", app_ofu_srv_cb.total_offset,
                    ota_fw_upgrade_state.total_len);
#endif

#ifdef APP_OFU_DEBUG
            if (app_ofu_srv_cb.total_offset == ota_fw_upgrade_state.total_len)
            {
                app_ofu_srv_cb.recv_crc32 = app_ofu_srv_cb.recv_crc32 ^ 0xFFFFFFFF;
                APP_OFU_TRACE_DBG("last OFU chunk received. Calculated CRC:%X\n",
                        app_ofu_srv_cb.recv_crc32);
            }
#endif
        }

        length -= bytes_to_copy;
        p += bytes_to_copy;

        /* APP_OFU_TRACE_DBG("remaining len: %d \n", length); */
    }
    return WICED_OTA_UPGRADE_STATUS_CONTINUE;
}

/*
 * app_ofu_srv_abort
 */
static void app_ofu_srv_abort(app_ofu_srv_app_callback_t *p_app_callback)
{
    app_ofu_srv_cb.state = APP_OFU_SRV_STATE_ABORTED;

    /* Tell the app that OFU is Aborted */
    p_app_callback(APP_OFU_EVENT_ABORTED);
}

/*
 * Process timeout started after the last notification to perform restart
 */
static void app_ofu_srv_reset_timeout(uint32_t param)
{
    APP_TRACE_DBG("Configure the Flash to boot on the new FW and Reboot\n");
    wiced_firmware_upgrade_finish();
}

#ifdef APP_OFU_DEBUG
/*
 * app_ofu_srv_crc32_update
 */
static uint32_t app_ofu_srv_crc32_update(uint32_t crc32, uint8_t *p_data, uint16_t length)
{
    return crc32_Update(crc32, p_data, length);
}
#endif
#endif
