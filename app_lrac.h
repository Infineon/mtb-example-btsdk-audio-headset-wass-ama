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
 * This file implements the DHI Functions
 */

#pragma once

#include "wiced.h"
#include "wiced_bt_lrac.h"

typedef enum
{
    APP_LRAC_CONNECTED = 1,
    APP_LRAC_DISCONNECTED,
    APP_LRAC_A2DP_STARTED,
    APP_LRAC_A2DP_STOPPED,
    APP_LRAC_HFP_STARTED,
    APP_LRAC_HFP_STOPPED,
    APP_LRAC_BUTTON,
    APP_LRAC_VOLUME,
    APP_LRAC_SWITCH_COMPLETED,
    APP_LRAC_AUDIO_GLITCH,
    APP_LRAC_LOCAL_JITTER_BUFFER,
    APP_LRAC_USER_DATA,
} app_lrac_event_t;

typedef struct
{
    wiced_result_t status;
    wiced_bt_device_address_t bdaddr;
    wiced_bt_lrac_role_t lrac_role;
    wiced_bt_lrac_audio_side_t audio_side;
} app_lrac_event_data_connected_t;

typedef struct
{
    uint8_t dummy;
} app_lrac_event_data_disconnected_t;

typedef struct
{
    wiced_result_t status;
    wiced_bt_a2dp_codec_info_t codec_info;
    wiced_bool_t sync;
} app_lrac_event_data_a2dp_started_t;

typedef struct
{
    wiced_result_t status;
} app_lrac_event_data_a2dp_stopped_t;

typedef struct
{
    wiced_result_t status;
    wiced_bool_t wide_band;
} app_lrac_event_data_hfp_started_t;

typedef struct
{
    wiced_result_t status;
} app_lrac_event_data_hfp_stopped_t;

typedef struct
{
    uint8_t button_id;
    uint32_t repeat_counter;
} app_lrac_event_data_button_t;

typedef struct
{
    int32_t am_vol_level;
    uint8_t am_vol_effect;
} app_lrac_event_data_volume_t;

typedef struct
{
    wiced_result_t status;
    wiced_bt_lrac_role_t new_role;
    wiced_bool_t fatal_error;       /* Unrecoverable error. Reboot recommended */
} app_lrac_event_switch_completed_t;

typedef union
{
    app_lrac_event_data_connected_t connected;
    app_lrac_event_data_disconnected_t disconnected;
    app_lrac_event_data_a2dp_started_t a2dp_started;
    app_lrac_event_data_a2dp_stopped_t a2dp_stopped;
    app_lrac_event_data_hfp_started_t hfp_started;
    app_lrac_event_data_hfp_stopped_t hfp_stopped;
    app_lrac_event_data_button_t button;
    app_lrac_event_data_volume_t volume;
    app_lrac_event_switch_completed_t switch_completed;
} app_lrac_event_data_t;

typedef void (app_lrac_callback_t)(app_lrac_event_t event, app_lrac_event_data_t *p_data);

/*
 * lrac_init
 * Initializes the LRAC System (app and library)
 */
wiced_result_t app_lrac_init(app_lrac_callback_t *p_callback);

/*
 * lrac_connect
 * This function is used to Connect a connection to a peer LRAC device.
 */
void app_lrac_connect(void);

/*
 * app_lrac_disconnect
 * This function is used to Disconnect a connection to a peer LRAC device.
 */
void app_lrac_disconnect(void);

/*
 * app_lrac_is_connected
 */
wiced_bool_t app_lrac_is_connected(void);

/*
 * app_lrac_connecting
 */
void app_lrac_connection_status_handler(uint8_t *p_features, wiced_bool_t is_connected,
        uint16_t handle, uint8_t reason);

/*
 * app_lrac_config_role_get
 */
wiced_bt_lrac_role_t app_lrac_config_role_get(void);

/*
 * app_lrac_config_peer_addr_get
 *
 * Get the LRAC peer device's Bluetooth address
 */
void app_lrac_config_peer_addr_get(wiced_bt_device_address_t peer_addr);

/*
 * app_lrac_a2dp_start_req
 */
wiced_result_t app_lrac_a2dp_start_req(wiced_bool_t sync);

/*
 * app_lrac_a2dp_stop_req
 */
wiced_result_t app_lrac_a2dp_stop_req(void);

/*
 * app_lrac_hfp_start_req
 */
wiced_result_t app_lrac_hfp_start_req(void);

/*
 * app_lrac_hfp_stop_req
 */
wiced_result_t app_lrac_hfp_stop_req(void);


/*
 * app_lrac_button_send
 */
wiced_result_t app_lrac_button_send(uint8_t button_id,uint32_t repeat_counter);

/*
 * app_lrac_volume_send
 *
 * @param[in] am_vol_level - local volume level used for Audio Manager
 * @param[in] am_vol_effect - volume effect used for Audio Manager, Pri can use it to control peer volume
 */
wiced_result_t app_lrac_volume_send(int32_t am_vol_level, uint8_t am_vol_effect);

/*
 * app_lrac_send_ofu
 */
wiced_result_t app_lrac_send_ofu(uint8_t *p_data, uint16_t length);

/*
 * app_lrac_tx_data
 */
wiced_result_t app_lrac_tx_data(uint8_t *p_data, uint16_t length);

/*
 * app_lrac_switch_req
 */
wiced_result_t app_lrac_switch_req(wiced_bool_t prevent_glitch);

/*
 * app_lrac_switch_is_in_progress
 */
wiced_bool_t app_lrac_switch_is_in_progress(void);

/*
 * app_lrac_ready_to_switch
 * This function will be called before LRAC Switch will be performed
 */
wiced_bool_t app_lrac_ready_to_switch(void);

/*
 * app_lrac_switch_get
 */
wiced_result_t app_lrac_switch_get(void *p_opaque, uint16_t *p_sync_data_len);

/*
 * app_lrac_switch_set
 */
wiced_result_t app_lrac_switch_set(void *p_opaque, uint16_t sync_data_len);

/*
 * app_lrac_role_get_desc
 */
char *app_lrac_role_get_desc(wiced_bt_lrac_role_t role);

/*
 * app_lrac_audio_side_get_desc
 */
char *app_lrac_audio_side_get_desc(wiced_bt_lrac_audio_side_t audio_side);

/*
 * app_lrac_send_quality
 */
wiced_result_t app_lrac_send_quality(void *p_data, uint16_t length);

/*
 * app_lrac_phone_profile_connection_up
 *
 * Indicate the profile for peer device is connected
 *
 * @param[in]   bdaddr: peer device's Bluetooth address
 * @param[in]   profile: refer to app_nvram_profiles_t
 *
 * @return      WICED_TRUE: the profile is the first connected profile with peer device
 *
 */
wiced_bool_t app_lrac_phone_profile_connection_up(wiced_bt_device_address_t bdaddr, uint8_t profile);

/*
 * app_lrac_phone_profile_connection_down
 *
 * Indicate the profile for peer device is disconnected.
 *
 * @param[in]   bdaddr: peer device's Bluetooth address
 * @param[in]   profile: refer to app_nvram_profiles_t
 *
 * @return      WICED_TRUE: the profile is the last connected profile with peer device
 *
 */
wiced_bool_t app_lrac_phone_profile_connection_down(wiced_bt_device_address_t bdaddr, uint8_t profile);

/*
 * app_lrac_jitter_buffer_target_send
 * Send the A2DP Jitter Buffer Target depth to peer device
 */
wiced_result_t app_lrac_jitter_buffer_target_send(uint8_t jitter_buffer_target);

/*
 * app_lrac_link_key_get
 */
wiced_bt_device_sec_keys_t *app_lrac_link_key_get(void);

/*
 * app_lrac_link_key_update
 */
void app_lrac_link_key_update(wiced_bt_device_sec_keys_t *p_key_data);

/*
 * app_lrac_link_key_reset
 */
void app_lrac_link_key_reset(void);

/*
 * app_lrac_nvram_update_req
 */
void app_lrac_nvram_update_req(void);
