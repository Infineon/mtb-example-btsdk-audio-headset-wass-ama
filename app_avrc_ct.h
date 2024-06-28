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
 */

#pragma once

#include "bt_types.h"
#include "wiced_bt_sdp.h"
#include "wiced_result.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_avrc_ct.h"
#include "wiced_bt_avrc_tg.h"

typedef enum
{
    APP_AVRC_CT_EVT_CONNECTED = 1,
    APP_AVRC_CT_EVT_DISCONNECTED,
} app_avrc_ct_event_t;

typedef struct
{
    wiced_result_t status;
    wiced_bt_device_address_t bdaddr;
} app_avrc_ct_connected_t;

typedef union
{
    app_avrc_ct_connected_t connected;
} app_avrc_ct_event_data_t;

typedef void (app_avrc_ct_callback_t)(app_avrc_ct_event_t event, app_avrc_ct_event_data_t *p_data);

wiced_result_t app_avrc_ct_init(app_avrc_ct_callback_t *p_callback);

void app_avrc_ct_connection_state_callback(uint8_t handle,
        wiced_bt_device_address_t remote_addr, wiced_result_t status,
        wiced_bt_avrc_ct_connection_state_t connection_state, uint32_t peer_features);

/**
 *
 * Function         app_avrc_ct_connection_state_callback_pre_handler
 *
 *                  Callback invoked by the AVRCP api when a connection status change has occurred.
 *
 * @param[in]       remote_addr      : Peer address
 * @param[in]       status           : result of the connection status update attempted
 * @param[in]       connection_state : Connection state set by update
 * @param[in]       peer_features    : If new connection, this is a map of the peer's capabilities
 *
 * @return          Nothing
 */
wiced_bool_t app_avrc_ct_connection_state_callback_pre_handler(uint8_t handle,
        wiced_bt_device_address_t remote_addr, wiced_result_t status,
        wiced_bt_avrc_ct_connection_state_t connection_state, uint32_t peer_features);
