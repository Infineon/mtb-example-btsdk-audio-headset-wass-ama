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

#include "wiced.h"
#include "wiced_bt_a2dp_defs.h"

/*
 * Maximum Bluetooth Device Name length
 *
 * Limit the name of the device to 20 bytes (including \0).
 * See Section 3.2.2.2 of Bluetooth Core Spec. v5.0 for details.
 */
#define APP_MAIN_BT_DEV_NAME_LEN            20

/* Hands-Free State */
typedef enum
{
    APP_MAIN_HANDSFREE_STATE_IDLE = 0,      /* Nothing Pending */
    APP_MAIN_HANDSFREE_STATE_DIALING,       /* Phone is Dialing (Outgoing call) */
    APP_MAIN_HANDSFREE_STATE_RINGING,       /* Phone is Ringing (Incoming call) */
    APP_MAIN_HANDSFREE_STATE_CALLING        /* Phone call in progress */
} app_main_handsfree_state_t;

/* AVRC Controller state (reflects the AVRC Target (Phone) Player's state (phone icon)) */
typedef enum
{
    APP_MAIN_AVRC_CT_PLAYSTATE_STOPPED = 0, /* Stopped */
    APP_MAIN_AVRC_CT_PLAYSTATE_PLAYING,     /* Playing */
    APP_MAIN_AVRC_CT_PLAYSTATE_PAUSED       /* Paused */
} app_main_avrc_ct_playstate_t;

/*
 * app_main_post_init
 */
wiced_result_t app_main_post_init(void);

/*
 * app_main_a2dp_sink_codec_get
 */
wiced_bt_a2dp_codec_info_t *app_main_a2dp_sink_codec_get(void);

/*
 * app_main_switch_is_ready
 */
wiced_bool_t app_main_switch_is_ready(void);

/*
 * app_main_switch_get
 */
wiced_result_t app_main_switch_get(void *p_opaque, uint16_t *p_sync_data_len);

/*
 * app_main_switch_set
 */
wiced_result_t app_main_switch_set(void *p_opaque, uint16_t p_sync_data_len);

/*
 * app_main_free_memory_check
 */
void app_main_free_memory_check(void);

/*
 * app_main_lrac_switch_backup_bt_visibility
 */
void app_main_lrac_switch_backup_bt_visibility(void);

/*
 * app_main_lrac_eavesdropping_state_recover
 */
wiced_bool_t app_main_lrac_eavesdropping_state_recover(void);
