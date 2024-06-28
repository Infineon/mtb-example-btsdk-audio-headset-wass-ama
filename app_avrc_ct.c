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
#include "app_avrc_ct.h"
#include "app_trace.h"
#include "app_lrac.h"
#include "app_nvram.h"
#include "app_audio_insert.h"
#ifdef VOICE_PROMPT
#include "app_voice_prompt.h"
#endif // VOICE_PROMPT

/******************************************************************************
 *                          Constants
 ******************************************************************************/

/******************************************************************************
 *                         Variable Definitions
 ******************************************************************************/
typedef struct
{
    app_avrc_ct_callback_t *p_callback;
} app_avrc_ct_cb_t;


static app_avrc_ct_cb_t app_avrc_ct_cb;

uint8_t app_avrc_ct_supported_events[] =
{
    FALSE,
    TRUE,            /* AVRC_EVT_PLAY_STATUS_CHANGE             0x01    Playback Status Changed */
    TRUE,            /* AVRC_EVT_TRACK_CHANGE                   0x02    Track Changed */
    FALSE,           /* AVRC_EVT_TRACK_REACHED_END              0x03    Track End Reached */
    FALSE,           /* AVRC_EVT_TRACK_REACHED_START            0x04    Track Reached Start */
    FALSE,           /* AVRC_EVT_PLAY_POS_CHANGED               0x05    Playback position changed */
    FALSE,           /* AVRC_EVT_BATTERY_STATUS_CHANGE          0x06    Battery status changed */
    FALSE,           /* AVRC_EVT_SYSTEM_STATUS_CHANGE           0x07    System status changed */
    FALSE,           /* AVRC_EVT_APP_SETTING_CHANGE             0x08    Player application settings changed */
    FALSE,           /* AVRC_EVT_NOW_PLAYING_CHANGE             0x09    Now Playing Content Changed (AVRCP 1.4) */
    FALSE,           /* AVRC_EVT_AVAL_PLAYERS_CHANGE            0x0a    Available Players Changed Notification (AVRCP 1.4) */
    FALSE,           /* AVRC_EVT_ADDR_PLAYER_CHANGE             0x0b    Addressed Player Changed Notification (AVRCP 1.4) */
    FALSE,           /* AVRC_EVT_UIDS_CHANGE                    0x0c    UIDs Changed Notification (AVRCP 1.4) */
    TRUE             /* AVRC_EVT_VOLUME_CHANGE                  0x0d    Notify Volume Change (AVRCP 1.4) */
};

/*
 * Local functions
 */


/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 * app_avrc_ct_init
 */
wiced_result_t app_avrc_ct_init(app_avrc_ct_callback_t *p_callback)
{
    if (p_callback == NULL)
    {
        APP_TRACE_ERR("no callback\n");
        return WICED_BT_BADARG;
    }

    app_avrc_ct_cb.p_callback = p_callback;

    return WICED_BT_SUCCESS;
}

/**
 *
 * Function         app_avrc_ct_connection_state_callback
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
void app_avrc_ct_connection_state_callback(uint8_t handle,
        wiced_bt_device_address_t remote_addr, wiced_result_t status,
        wiced_bt_avrc_ct_connection_state_t connection_state, uint32_t peer_features)
{
    app_avrc_ct_event_data_t event_data;

    /* Service the connection state change. */
    switch( connection_state )
    {
        case REMOTE_CONTROL_DISCONNECTED:
            /* Inform the APP of the disconnect */
            app_avrc_ct_cb.p_callback(APP_AVRC_CT_EVT_DISCONNECTED, NULL);
            break;

        case REMOTE_CONTROL_CONNECTED:
            /* Inform the APP of the connection */
            event_data.connected.status = WICED_BT_SUCCESS;
            memcpy(event_data.connected.bdaddr, remote_addr, BD_ADDR_LEN);
            app_avrc_ct_cb.p_callback(APP_AVRC_CT_EVT_CONNECTED, &event_data);
            break;

        case REMOTE_CONTROL_INITIALIZED:
            break;
    }
}

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
        wiced_bt_avrc_ct_connection_state_t connection_state, uint32_t peer_features)
{
    /* Service the connection state change. */
    switch (connection_state)
    {
    case REMOTE_CONTROL_DISCONNECTED:
        /* Keep track of Connected Profiles */
        if (app_lrac_phone_profile_connection_down(remote_addr, APP_NVRAM_PROFILES_AVRC_CT_MASK))
        {
#ifdef VOICE_PROMPT
            app_audio_insert_start_req(APP_VOICE_PROMPT_INDEX_BT_DISCONNECTED);
#endif
        }
        break;

    default:
        break;
    }

    return WICED_TRUE;
}
