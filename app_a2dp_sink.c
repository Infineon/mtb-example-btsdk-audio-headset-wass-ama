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
 * This file implements audio application controlled over UART.
 *
 */
#include "wiced.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_trace.h"
#include "hci_control_api.h"
#include "app_lrac.h"
#include "app_a2dp_sink.h"
#include "app_trace.h"
#include "app_nvram.h"
#include "app_bt.h"
#include "app_audio_insert.h"
#ifdef VOICE_PROMPT
#include "app_voice_prompt.h"
#endif // VOICE_PROMPT
#include "bt_hs_spk_control.h"
#include "bt_hs_spk_audio.h"
#include "bt_hs_spk_handsfree.h"

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/

typedef struct
{
    wiced_bool_t                pending;
    wiced_bt_a2dp_sink_start_t  data;
} app_a2dp_sink_start_req_info_t;

typedef struct
{
    app_a2dp_sink_callback_t        *p_callback;
    app_a2dp_sink_start_req_info_t  start_request;
} app_a2dp_sink_cb_t;

/*
 * Global variables
 */
/* A2DP module control block */
static app_a2dp_sink_cb_t app_a2dp_sink_cb = {0};
extern wiced_bt_a2dp_config_data_t bt_audio_config;

/*
 * External functions
 */
wiced_result_t wiced_audio_sink_set_jitter_buffer_target(uint8_t target);

/*
 * Local functions
 */

/*******************************************************************************
 * A2DP Application HCI Control handlers
 *******************************************************************************/

/* ****************************************************************************
 * Function: app_a2dp_sink_cback_pre_handler
 *
 * Parameters:
 *          event - control event code
 *          p_data - event data
 *
 * Description:
 *          Control callback pre-handler supplied by the a2dp sink profile code.
 * ***************************************************************************/
wiced_bool_t app_a2dp_sink_cback_pre_handler(wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t *p_data)
{
#ifdef DISABLE_SNIFF_MODE_DURING_A2DP
    wiced_bt_device_address_t bdaddr;
    wiced_bool_t bdaddr_valid;

    bdaddr_valid = bt_hs_spk_audio_current_streaming_addr_get(bdaddr);
#endif /* DISABLE_SNIFF_MODE_DURING_A2DP */

    switch (event)
    {
    case WICED_BT_A2DP_SINK_DISCONNECT_EVT: /* Disconnected event, received on disconnection from a peer device */
        /* Keep track of Connected Profiles */
        if (app_lrac_phone_profile_connection_down(p_data->disconnect.bd_addr, APP_NVRAM_PROFILES_A2DP_SINK_MASK))
        {
#ifdef VOICE_PROMPT
            app_audio_insert_start_req(APP_VOICE_PROMPT_INDEX_BT_DISCONNECTED);
#endif // VOICE_PROMPT
        }

#ifdef DISABLE_SNIFF_MODE_DURING_A2DP
        /* enable SNIFF mode if the streaming one is disconnected */
        if (bdaddr_valid &&
                (memcmp(bdaddr,
                        p_data->disconnect.bd_addr,
                        sizeof(wiced_bt_device_address_t)) == 0))
        {
            bt_hs_spk_control_acl_link_policy_sniff_mode_set(NULL, WICED_TRUE);
        }
#endif /* DISABLE_SNIFF_MODE_DURING_A2DP */
        break;
    case WICED_BT_A2DP_SINK_START_IND_EVT:  /* Start stream indication event, received when start req is received */
#ifdef DISABLE_SNIFF_MODE_DURING_A2DP
        /* disable sniff mode */
        bt_hs_spk_control_acl_link_policy_sniff_mode_set(NULL, WICED_FALSE);
#endif /* DISABLE_SNIFF_MODE_DURING_A2DP */

        /* If Audio Insertion ongoing, Stop it first */
        if (app_audio_insert_is_started())
        {
            /* Stop the audio insertion. */
            if (app_audio_insert_stop_req(APP_AUDIO_INSERT_STOP_REQ_SUSPEND) == WICED_BT_SUCCESS)
            {
                /* Check if the request is already pending. */
                if (app_a2dp_sink_cb.start_request.pending == WICED_FALSE)
                {
                    /* Store information. */
                    app_a2dp_sink_cb.start_request.pending = WICED_TRUE;
                    memcpy((void *) &app_a2dp_sink_cb.start_request.data,
                           (void *) &p_data->start_ind,
                           sizeof(wiced_bt_a2dp_sink_start_t));
                }

                return WICED_FALSE;
            }
        }
        break;
    case WICED_BT_A2DP_SINK_START_CFM_EVT:
#ifdef DISABLE_SNIFF_MODE_DURING_A2DP
        /* disable sniff mode */
        bt_hs_spk_control_acl_link_policy_sniff_mode_set(NULL, WICED_FALSE);
#endif /* DISABLE_SNIFF_MODE_DURING_A2DP */
        break;
    case WICED_BT_A2DP_SINK_SUSPEND_EVT:
#ifdef DISABLE_SNIFF_MODE_DURING_A2DP
        /* enable sniff mode if the streaming one is suspend */
        if (bdaddr_valid &&
                (memcmp(bdaddr,
                        p_data->suspend.bd_addr,
                        sizeof(wiced_bt_device_address_t)) == 0) &&
            !bt_hs_spk_handsfree_sco_connection_check(NULL) )
        {
            bt_hs_spk_control_acl_link_policy_sniff_mode_set(NULL, WICED_TRUE);
        }
#endif /* DISABLE_SNIFF_MODE_DURING_A2DP */
        break;
    default:
        break;
    }

    return WICED_TRUE;
}


/* ****************************************************************************
 * Function: app_a2dp_sink_cback
 *
 * Parameters:
 *          event - control event called back
 *          p_data - event data
 *
 * Description:
 *          Control callback supplied by  the a2dp sink profile code.
 * ***************************************************************************/
void app_a2dp_sink_cback(wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t *p_data)
{
    app_a2dp_sink_event_data_t event_data;
    wiced_result_t status;

    if (app_a2dp_sink_cb.p_callback == NULL)
    {
        APP_TRACE_ERR("No Callback\n");
        return;
    }

    switch(event)
    {
        case WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT: /**< Codec config event, received when codec config for a streaming session is updated */
            break;

        case WICED_BT_A2DP_SINK_CONNECT_EVT:      /**< Connected event, received on establishing connection to a peer device. Ready to stream. */
            if (p_data->connect.result == WICED_SUCCESS)
            {
                /* Change the Link Policy */
                bt_hs_spk_control_acl_link_policy_set(p_data->connect.bd_addr,
                                                      HCI_ENABLE_ROLE_SWITCH | HCI_ENABLE_SNIFF_MODE);
            }

            /* Notify APP of connection state change */
            event_data.connected.status = p_data->connect.result;
            event_data.connected.handle = p_data->connect.handle;
            memcpy(event_data.connected.bdaddr, p_data->connect.bd_addr,
                    sizeof(wiced_bt_device_address_t));

            app_a2dp_sink_cb.p_callback(APP_A2DP_SINK_EVT_STREAM_CONNECTED, &event_data);
            break;

        case WICED_BT_A2DP_SINK_DISCONNECT_EVT:   /**< Disconnected event, received on disconnection from a peer device */
            event_data.disconnected.handle = p_data->disconnect.handle;
            app_a2dp_sink_cb.p_callback(APP_A2DP_SINK_EVT_STREAM_DISCONNECTED, &event_data);

            break;

        case WICED_BT_A2DP_SINK_START_IND_EVT:        /**< Start stream event, received when audio streaming is about to start */
            memcpy((void *) event_data.start_req.bdaddr,
                   (void *) p_data->start_ind.bdaddr,
                   sizeof(wiced_bt_device_address_t));

            app_a2dp_sink_cb.p_callback(APP_A2DP_SINK_EVT_STREAM_START_REQ, &event_data);
            break;

        case WICED_BT_A2DP_SINK_START_CFM_EVT:        /**< Start stream event, received when audio streaming is about to start */
            /* Send the Delay Report to the A2DP Source */
            bt_hs_spk_audio_a2dp_delay_update();

            /* Sent the Jitter Buffer Target depth to the Secondary */
            status = app_lrac_jitter_buffer_target_send(bt_audio_config.p_param.target_buf_depth);
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_lrac_jitter_buffer_target_send failed status:%d\n", status);

            app_a2dp_sink_cb.p_callback(APP_A2DP_SINK_EVT_STREAM_STARTED, NULL);
            break;

        case WICED_BT_A2DP_SINK_SUSPEND_EVT:      /**< Suspend stream event, received when audio streaming is suspended */
            app_a2dp_sink_start_req_pending_reset(p_data->suspend.handle);

            /* Process this suspend event only when there is no existent audio streaming. */
            if (bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED)
            {
                /* Set back the Jitter Buffer Target depth to the Minimum. */
                app_a2dp_sink_jitter_buffer_target_set(APP_A2DP_SINK_JITTER_BUFFER_TARGET_MIN);

                app_a2dp_sink_cb.p_callback(APP_A2DP_SINK_EVT_STREAM_STOPPED, NULL);
            }
            break;

        default:
            break;
    }
}

/*
 * This api initialize A2DP Sink
 */
wiced_result_t app_a2dp_sink_init (app_a2dp_sink_callback_t *p_callback)
{
    if (p_callback == NULL)
    {
        APP_TRACE_ERR("No callback\n");
        return WICED_BT_BADARG;
    }

    app_a2dp_sink_cb.p_callback = p_callback;

    return WICED_SUCCESS;
}

/*
 * app_a2dp_sink_start_rsp
 */
wiced_result_t app_a2dp_sink_start_rsp(void)
{
    wiced_result_t status = WICED_BT_BADARG;

    /* Send the Delay Report */
    bt_hs_spk_audio_a2dp_delay_update();

    /* Sent the Jitter Buffer Target depth to the Secondary */
    status = app_lrac_jitter_buffer_target_send(bt_audio_config.p_param.target_buf_depth);
    if (status != WICED_BT_SUCCESS)
        APP_TRACE_ERR("app_lrac_jitter_buffer_target_send failed status:%d\n", status);

    app_a2dp_sink_cb.p_callback(APP_A2DP_SINK_EVT_STREAM_STARTED, NULL);

    return status;
}

/*
 * app_a2dp_sink_underrun
 * This function is called when the FW reports a Jitter Buffer, UnderRun error
 */
void app_a2dp_sink_underrun(void)
{
    wiced_result_t status;

    /*
     * To solve the UnderRun issue, we will try to Increase the Jitter Buffer Target depth
     */
    /* If the Jitter Buffer Target depth is already Maximum */
    if (bt_audio_config.p_param.target_buf_depth >= APP_A2DP_SINK_JITTER_BUFFER_TARGET_MAX)
    {
        /* There is nothing we can do. Return. */
        return;
    }

    /* Change the Jitter Buffer Target depth */
    status = app_a2dp_sink_jitter_buffer_target_set(APP_A2DP_SINK_JITTER_BUFFER_TARGET_MAX);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("app_a2dp_sink_jitter_buffer_target_set failed status:%d\n", status);
        return;
    }

    /*
     * Calculate the New A2DP Delay introduced by this Jitter Buffer Target depth and send it
     * sent it to the A2DP Source
     */
    bt_hs_spk_audio_a2dp_delay_update();

    /* Sent the Jitter Buffer Target depth to the Secondary */
    status = app_lrac_jitter_buffer_target_send(bt_audio_config.p_param.target_buf_depth);
    if (status != WICED_BT_SUCCESS)
        APP_TRACE_ERR("app_lrac_jitter_buffer_target_send failed status:%d\n", status);
}

/*
 * app_a2dp_sink_jitter_buffer_target_set
 * This function can be used to change the Jitter Buffer target depth.
 * It is called on Secondary when the Primary changes this value.
 * This function is needed in case a PS-Switch occurs after Jitter Buffer target depth changes.
 */
wiced_result_t app_a2dp_sink_jitter_buffer_target_set(uint8_t jitter_buffer_target)
{
    wiced_result_t status;

    APP_TRACE_DBG("target:%d\n", jitter_buffer_target);

    /* Change the Jitter Buffer Target depth */
    status = wiced_audio_sink_set_jitter_buffer_target(jitter_buffer_target);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_audio_sink_set_jitter_buffer_target failed status:%d\n", status);
        return status;
    }
    bt_audio_config.p_param.start_buf_depth = jitter_buffer_target;
    bt_audio_config.p_param.target_buf_depth = jitter_buffer_target;

    return status;
}

/*
 * app_a2dp_sink_start_req_pending_resume
 *
 * Resume the pending AVDT START IND. request
 */
void app_a2dp_sink_start_req_pending_resume(void)
{
    wiced_bt_a2dp_sink_event_data_t event_data = {0};

    if (app_a2dp_sink_cb.start_request.pending == WICED_FALSE)
    {
        return;
    }

    memcpy((void *) &event_data.start_ind,
           (void *) &app_a2dp_sink_cb.start_request.data,
           sizeof(wiced_bt_a2dp_sink_start_t));

    memset((void *) &app_a2dp_sink_cb.start_request,
           0,
           sizeof(app_a2dp_sink_start_req_info_t));

    bt_hs_spk_audio_a2dp_sink_event_emulator(WICED_BT_A2DP_SINK_START_IND_EVT, &event_data);
}

/*
 * app_a2dp_sink_start_req_reset
 *
 * Reset the AVDT Start Ind. request pending information.
 */
void app_a2dp_sink_start_req_pending_reset(uint16_t handle)
{
    if (app_a2dp_sink_cb.start_request.pending == WICED_FALSE)
    {
        return;
    }

    if (app_a2dp_sink_cb.start_request.data.handle != handle)
    {
        return;
    }

    memset((void *) &app_a2dp_sink_cb.start_request,
           0,
           sizeof(app_a2dp_sink_start_req_info_t));
}
