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

#include <stdint.h>
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2dp_sink.h"

#define APP_A2DP_SINK_JITTER_BUFFER_TARGET_MIN      50
#define APP_A2DP_SINK_JITTER_BUFFER_TARGET_MAX      60

typedef enum
{
    APP_A2DP_SINK_EVT_STREAM_CONNECTED = 1,
    APP_A2DP_SINK_EVT_STREAM_DISCONNECTED,
    APP_A2DP_SINK_EVT_STREAM_START_REQ,
    APP_A2DP_SINK_EVT_STREAM_STARTED,
    APP_A2DP_SINK_EVT_STREAM_STOPPED
} app_a2dp_sink_event_t;

typedef struct
{
    wiced_bt_device_address_t bdaddr;
    wiced_result_t status;
    uint16_t handle;                    /* A2DP Connection Handle */
} app_a2dp_sink_event_data_connected_t;

typedef struct
{
    uint16_t handle;                    /* A2DP Connection Handle */
} app_a2dp_sink_event_data_disconnected_t;

typedef struct
{
    wiced_bt_device_address_t bdaddr;   /* Source device's BT address */
} app_a2dp_sink_event_data_start_req_t;

typedef union
{
    app_a2dp_sink_event_data_connected_t connected;
    app_a2dp_sink_event_data_disconnected_t disconnected;
    app_a2dp_sink_event_data_start_req_t start_req;
} app_a2dp_sink_event_data_t;

typedef void (app_a2dp_sink_callback_t)(app_a2dp_sink_event_t event,
        app_a2dp_sink_event_data_t *p_data);

wiced_result_t app_a2dp_sink_init (app_a2dp_sink_callback_t *p_callback);

void app_a2dp_sink_cback(wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t* p_data);
wiced_bool_t app_a2dp_sink_cback_pre_handler(wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t *p_data);
/*
 * app_a2dp_sink_start_rsp
 */
wiced_result_t app_a2dp_sink_start_rsp(void);

/*
 * app_a2dp_sink_underrun
 * This function is called when the FW reports a Jitter Buffer UnderRun error
 */
void app_a2dp_sink_underrun(void);

/*
 * app_a2dp_sink_jitter_buffer_target_set
 * This function can be used to change the Jitter Buffer target depth.
 * It is called on Secondary when the Primary changes this value.
 * This function is needed in case a PS-Switch occurs after Jitter Buffer target depth changes.
 */
wiced_result_t app_a2dp_sink_jitter_buffer_target_set(uint8_t jitter_buffer_target);

/*
 * app_a2dp_sink_start_req_pending_resume
 *
 * Resume the pending AVDT START IND. request
 */
void app_a2dp_sink_start_req_pending_resume(void);

/*
 * app_a2dp_sink_start_req_reset
 *
 * Reset the AVDT Start Ind. request pending information.
 */
void app_a2dp_sink_start_req_pending_reset(uint16_t handle);
