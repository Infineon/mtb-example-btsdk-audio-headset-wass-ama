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

/** @file
 *
 * This file provides the private interface definitions for handsfree
 *
 */
#pragma once

#include "bt_types.h"
#include "wiced_bt_hfp_hf_int.h"
#include "wiced_bt_hfp_hf.h"

/** HF Supported features. */
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
#define WICED_HANDSFREE_SUPPORTED_FEATURES  (WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                             WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                             WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                             WICED_BT_HFP_HF_FEATURE_HF_INDICATORS | \
                                             WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION | \
                                             WICED_BT_HFP_HF_FEATURE_ESCO_S4_SETTINGS_SUPPORT)
#else
#define WICED_HANDSFREE_SUPPORTED_FEATURES  (WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                             WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                             WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                             WICED_BT_HFP_HF_FEATURE_HF_INDICATORS | \
                                             WICED_BT_HFP_HF_FEATURE_ESCO_S4_SETTINGS_SUPPORT)

#endif

/** HF supported features for SDP. */
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
#define APP_HANDFREE_SDP_FEATURE            (WICED_BT_HFP_HF_SDP_FEATURE_3WAY_CALLING | \
                                             WICED_BT_HFP_HF_SDP_FEATURE_CLIP | \
                                             WICED_BT_HFP_HF_SDP_FEATURE_REMOTE_VOL_CTRL | \
                                             WICED_BT_HFP_HF_SDP_FEATURE_WIDEBAND_SPEECH)
#else
#define APP_HANDFREE_SDP_FEATURE            (WICED_BT_HFP_HF_SDP_FEATURE_3WAY_CALLING | \
                                             WICED_BT_HFP_HF_SDP_FEATURE_CLIP | \
                                             WICED_BT_HFP_HF_SDP_FEATURE_REMOTE_VOL_CTRL)
#endif

typedef enum
{
    APP_HANDSFREE_EVENT_CONNECTED = 1,
    APP_HANDSFREE_EVENT_DISCONNECTED,
    APP_HANDSFREE_EVENT_AUDIO_CONNECT_REQ,
    APP_HANDSFREE_EVENT_AUDIO_CONNECTED,
    APP_HANDSFREE_EVENT_AUDIO_DISCONNECTED,
} app_handsfree_event_t;

typedef struct
{
    wiced_result_t status;
    wiced_bt_device_address_t bdaddr;
} app_handsfree_event_data_connected_t;

typedef struct
{
    wiced_bt_device_address_t bdaddr;
} app_handsfree_event_data_audio_connect_req_t;

typedef struct
{
    uint16_t sco_index;                  /**< SCO index */
} app_handsfree_event_data_audio_disconnected_t;

typedef union
{
    app_handsfree_event_data_connected_t connected;
    app_handsfree_event_data_audio_connect_req_t audio_connect_req;
    app_handsfree_event_data_audio_disconnected_t audio_disconnected;
} app_handsfree_event_data_t;

/*
 * app_handsfree_callback_t
 */
typedef void (app_handsfree_callback_t)(app_handsfree_event_t event,
        app_handsfree_event_data_t *p_data);

/*
 * app_handsfree_init
 */
wiced_result_t app_handsfree_init(app_handsfree_callback_t *p_callback);

/*
 * app_handsfree_event_callback
 */
void app_handsfree_event_callback(wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data);

/* ****************************************************************************
 * Function: app_handsfree_event_callback_pre_handler
 *
 * Parameters:
 *          event - control event code
 *          p_data - event data
 *
 * Description:
 *          Control callback pre-handler supplied by the HFP profile code.
 * ***************************************************************************/
wiced_bool_t app_handsfree_event_callback_pre_handler(wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data);

/*
 * app_handsfree_sco_management_callback_handler
 */
void app_handsfree_sco_management_callback_handler(wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data);

/*
 * app_handsfree_sco_start_req_pending_resume
 *
 * Resume the pending SCO connection request
 */
void app_handsfree_sco_start_req_pending_resume(void);

/*
 * app_handsfree_sco_start_req_pending_reset
 *
 * Reset the SCO connection request pending information.
 */
void app_handsfree_sco_start_req_pending_reset(uint16_t sco_index, wiced_bt_device_address_t bdaddr);
