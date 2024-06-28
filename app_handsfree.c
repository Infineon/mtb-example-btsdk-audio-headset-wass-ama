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
 *  This file implements the handsfree functions
 *
 */

#include "app_handsfree.h"

#ifdef AMA_ENABLED
#include <ama.h>
#endif
#include "wiced_bt_sco.h"
#include "wiced_timer.h"
#include "wiced_bt_dev.h"
#include "app_trace.h"
#include "app_bt.h"
#include "app_lrac.h"
#include "app_nvram.h"
#include "app_audio_insert.h"
#ifdef VOICE_PROMPT
#include "app_voice_prompt.h"
#endif // VOICE_PROMPT
#include "bt_hs_spk_handsfree.h"
#include "bt_hs_spk_audio.h"
#include "wiced_bt_lrac.h"

/*
 * Structures
 */
typedef struct
{
    wiced_bool_t                        pending;
    wiced_bt_sco_connection_request_t   data;
} app_handsfree_sco_start_req_info_t;

typedef struct
{
    app_handsfree_callback_t            *p_callback;
    app_handsfree_sco_start_req_info_t  sco_start_request;
} app_handsfree_cb_t;

/* HFP event handler. */
typedef void (*APP_HANDFREE_EVENT_HANDLER)(wiced_bt_hfp_hf_event_data_t *p_data);

/*
 * Local functions
 */
static void app_handsfree_event_callback_connection_state(wiced_bt_hfp_hf_event_data_t* p_data);

/*
 * External functions
 */
extern void app_main_a2dp_stop(void);

/*
 * Globlale variables
 */
app_handsfree_cb_t app_handsfree_cb = {0};

static const APP_HANDFREE_EVENT_HANDLER app_handsfree_event_handler[] =
{
    &app_handsfree_event_callback_connection_state,     /* WICED_BT_HFP_HF_CONNECTION_STATE_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_SERVICE_STATE_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_SERVICE_TYPE_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_CALL_SETUP_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_RING_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_INBAND_RING_STATE_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_RSSI_IND_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_VOLUME_CHANGE_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_CLIP_IND_EVT */
    NULL,                                               /* WICED_BT_HFP_HFP_CODEC_SET_EVT */
    NULL,                                               /* WICED_BT_HFP_HFP_ACTIVE_CALL_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_OK_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_ERROR_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_CME_ERROR_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_CNUM_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_BINP_EVT */
    NULL,                                               /* WICED_BT_HFP_HF_VOICE_RECOGNITION_EVT */
    NULL                                                /* WICED_BT_HFP_HF_BIND_EVT */
};

/*
 * Initialize HFP
 */
wiced_result_t app_handsfree_init(app_handsfree_callback_t *p_callback)
{
    app_handsfree_cb.p_callback = p_callback;

    return WICED_BT_SUCCESS;
}

/*
 * Process SCO management callback
 */
void app_handsfree_sco_management_callback( wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data )
{
    app_handsfree_event_data_t event_data;

    switch (event)
    {
        case BTM_SCO_CONNECTED_EVT:             /**< SCO connected event. Event data: #wiced_bt_sco_connected_t */
            app_handsfree_cb.p_callback(APP_HANDSFREE_EVENT_AUDIO_CONNECTED, NULL);
            break;

        case BTM_SCO_DISCONNECTED_EVT:          /**< SCO disconnected event. Event data: #wiced_bt_sco_disconnected_t */
            event_data.audio_disconnected.sco_index = p_event_data->sco_disconnected.sco_index;
            app_handsfree_cb.p_callback(APP_HANDSFREE_EVENT_AUDIO_DISCONNECTED, &event_data);
            /* Set phone as non-busy state for PS-link power mgmt feature */
            wiced_bt_lrac_sniff_power_mgmt_set_phone_busy_state(WICED_FALSE);
            break;

        case BTM_SCO_CONNECTION_REQUEST_EVT:    /**< SCO connection request event. Event data: #wiced_bt_sco_connection_request_t */
            /* SCO Connection Request. Ask the main app to accept it (to handle conflict with Audio Insert) */
            memcpy((void *) event_data.audio_connect_req.bdaddr,
                   (void *) p_event_data->sco_connection_request.bd_addr,
                   sizeof(wiced_bt_device_address_t));
            /* Set phone as busy state for PS-link power mgmt feature */
            wiced_bt_lrac_sniff_power_mgmt_set_phone_busy_state(WICED_TRUE);
            app_handsfree_cb.p_callback(APP_HANDSFREE_EVENT_AUDIO_CONNECT_REQ, &event_data);
            break;

        case BTM_SCO_CONNECTION_CHANGE_EVT:     /**< SCO connection change event. Event data: #wiced_bt_sco_connection_change_t */
            break;
    }
}

/*
 * app_handsfree_sco_management_callback_pre_handler
 *
 */
wiced_bool_t app_handsfree_sco_management_callback_pre_handler(wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data)
{
    switch (event)
    {
    case BTM_SCO_CONNECTION_REQUEST_EVT:    /**< SCO connection request event. Event data: #wiced_bt_sco_connection_request_t */
        /* Handle the Audio Connect Request */
        /* For some Andriod systems, the AVDTP_SUSPEND command will be issued after
         * LMP_eSCO_link_req/LMP_SCO_link_req command. We need to check the A2DP play state and
         * stop the A2DP media connection before accepting the incoming     SCO/eSCO connection request
         * due to hardware constraint (memory issue).
         * To achieve this, we have to
         * 1. stop codec
         * 2. stop litehost
         * 3. stop A2DP
         * 4. Stop LARC A2DP eavesdropping
         * 5. enable codec for SCO/eSCO
         * 6. enable litehost */
        APP_TRACE_DBG("SCO Connect Req BdAddr:%B\n", p_event_data->sco_connection_request.bd_addr);
        if (bt_hs_spk_audio_streaming_check(NULL) == WICED_ALREADY_CONNECTED)
        {
            app_main_a2dp_stop();
        }

        /* If Audio Insertion ongoing, Stop it first */
        if (app_audio_insert_is_started())
        {
            /* Stop the audio insertion. */
            if (app_audio_insert_stop_req(APP_AUDIO_INSERT_STOP_REQ_SUSPEND) == WICED_BT_SUCCESS)
            {
                /* Check if the request is already pending. */
                if (app_handsfree_cb.sco_start_request.pending == WICED_FALSE)
                {
                    /* Store information. */
                    app_handsfree_cb.sco_start_request.pending = WICED_TRUE;
                    memcpy((void *) &app_handsfree_cb.sco_start_request.data,
                           (void *) &p_event_data->sco_connection_request,
                           sizeof(wiced_bt_sco_connection_request_t));
                }
                return WICED_FALSE;
            }
        }
        break;

    case BTM_SCO_CONNECTED_EVT:
        APP_TRACE_DBG("SCO Connected idx:%d\n", p_event_data->sco_connected.sco_index);
        break;

    case BTM_SCO_DISCONNECTED_EVT:
        APP_TRACE_DBG("SCO Disconnected idx:%d\n", p_event_data->sco_disconnected.sco_index);
        break;

    case BTM_SCO_CONNECTION_CHANGE_EVT:
        APP_TRACE_DBG("SCO Connection Change BdAddr:%B\n",
                p_event_data->sco_connection_change.bd_addr);
        break;

    default:
        break;
    }

    return WICED_TRUE;
}

/*
 * app_handsfree_sco_management_callback_handler
 */
void app_handsfree_sco_management_callback_handler(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    if (app_handsfree_sco_management_callback_pre_handler(event, p_event_data) == WICED_FALSE)
    {
        return;
    }

    hf_sco_management_callback(event, p_event_data);
    app_handsfree_sco_management_callback(event, p_event_data);
}

/*
 * This function will handle HF connection events
 */
static void app_handsfree_event_callback_connection_state(wiced_bt_hfp_hf_event_data_t* p_data)
{
    app_handsfree_event_data_t event_data;

    /* Process the event by state. */
    switch (p_data->conn_data.conn_state)
    {
    /* HF control connection is closed */
    case WICED_BT_HFP_HF_STATE_DISCONNECTED:
        /* Send an event to application main handler. */
        app_handsfree_cb.p_callback(APP_HANDSFREE_EVENT_DISCONNECTED, NULL);
        break;

    /* HF control connection established */
    case WICED_BT_HFP_HF_STATE_CONNECTED:
        /*
         * Be sure the Audio Gateway is Central. We do it here to make it happen as soon as
         * possible. If we let the Main module doing it once SLC is fully connected it may be
         * too late because the SCO link may have been already established (Role Switch is not
         * allowed if a SCO link is connected)
         */
        if (BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS == 1)
        {
            bt_hs_spk_control_bt_role_set(p_data->conn_data.remote_address, HCI_ROLE_PERIPHERAL);
        }
        break;

    /* HF synchronized with AG's state, ready to send/receive commands/notifications */
    case WICED_BT_HFP_HF_STATE_SLC_CONNECTED:
        memcpy(event_data.connected.bdaddr, p_data->conn_data.remote_address,
                sizeof(wiced_bt_device_address_t));
        event_data.connected.status = WICED_BT_SUCCESS;
        app_handsfree_cb.p_callback(APP_HANDSFREE_EVENT_CONNECTED, &event_data);
        break;

    /* Unknown state */
    default:
        return;
    }
}

/*
 * Control callback supplied by the handsfree profile code.
 */
void app_handsfree_event_callback( wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data)
{
    /* Check if the application callback is set. */
    if (app_handsfree_cb.p_callback == NULL)
    {
        APP_TRACE_ERR("No Callback\n");
        return;
    }

    /* Process the incoming HFP event. */
    if (event > WICED_BT_HFP_HF_BIND_EVT)
    {
        APP_TRACE_ERR("Unknown HFP event\n");
    }
    else
    {
        if (app_handsfree_event_handler[event])
        {
            (*app_handsfree_event_handler[event])(p_data);
        }
    }
}

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
wiced_bool_t app_handsfree_event_callback_pre_handler(wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data)
{
    switch (event)
    {
    case WICED_BT_HFP_HF_CONNECTION_STATE_EVT:
        switch (p_data->conn_data.conn_state)
        {
        /* HF control connection is closed */
        case WICED_BT_HFP_HF_STATE_DISCONNECTED:
            /* Keep track of Connected Profiles */
            if (app_lrac_phone_profile_connection_down(p_data->conn_data.remote_address, APP_NVRAM_PROFILES_HFP_HS_MASK))
            {
#ifdef VOICE_PROMPT
                app_audio_insert_start_req(APP_VOICE_PROMPT_INDEX_BT_DISCONNECTED);
#endif
            }
            break;
        default:
            break;
        }
        break;

    default:
        break;
    }

#ifdef AMA_ENABLED
    return ama_hfp_pre_handler(event, p_data);
#else
    return WICED_TRUE;
#endif
}

/*
 * app_handsfree_sco_start_req_pending_resume
 *
 * Resume the pending SCO connection request
 */
void app_handsfree_sco_start_req_pending_resume(void)
{
    wiced_bt_management_evt_data_t event_data = {0};

    if (app_handsfree_cb.sco_start_request.pending == WICED_FALSE)
    {
        WICED_BT_TRACE("app_handsfree_sco_start_req_pending_resume pending false\n");
        return;
    }

    memcpy((void *) &event_data.sco_connection_request,
           (void *) &app_handsfree_cb.sco_start_request.data,
           sizeof(wiced_bt_sco_connection_request_t));

    memset((void *) &app_handsfree_cb.sco_start_request,
           0,
           sizeof(app_handsfree_sco_start_req_info_t));

    app_handsfree_sco_management_callback_handler(BTM_SCO_CONNECTION_REQUEST_EVT, &event_data);
}

/*
 * app_handsfree_sco_start_req_pending_reset
 *
 * Reset the SCO connection request pending information.
 */
void app_handsfree_sco_start_req_pending_reset(uint16_t sco_index, wiced_bt_device_address_t bdaddr)
{
    if (app_handsfree_cb.sco_start_request.pending == WICED_FALSE)
    {
        return;
    }

    if (bdaddr)
    {
        if (memcmp((void *) bdaddr,
                   (void *) app_handsfree_cb.sco_start_request.data.bd_addr,
                   sizeof(wiced_bt_device_address_t)) != 0)
        {
            return;
        }
    }
    else
    {
        if (app_handsfree_cb.sco_start_request.data.sco_index != sco_index)
        {
            return;
        }
    }

    memset((void *) &app_handsfree_cb.sco_start_request,
           0,
           sizeof(app_handsfree_sco_start_req_info_t));
}
