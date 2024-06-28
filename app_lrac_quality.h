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

#include "wiced_bt_lrac.h"

typedef enum
{
    APP_LRAC_QUALITY_UNDERRUN,      /* A2DP Jitter Buffer Underrun */
    /* TODO: Add "PS-Switch Recommended" event */
} app_lrac_quality_event_t;

typedef union
{

} app_lrac_quality_event_data_t;

typedef void (app_lrac_quality_callback_t)(app_lrac_quality_event_t event,
        app_lrac_quality_event_data_t *p_data);

/*
 * app_lrac_quality_init
 */
wiced_result_t app_lrac_quality_init(app_lrac_quality_callback_t *p_callback);

/*
 * app_lrac_quality_timer_start
 */
wiced_result_t app_lrac_quality_timer_start(void);

/*
 * app_lrac_quality_timer_stop
 */
wiced_result_t app_lrac_quality_timer_stop(void);

/*
 * app_lrac_quality_rssi_handler
 */
void app_lrac_quality_rssi_handler(wiced_bt_lrac_rssi_t *p_data);

/*
 * app_lrac_quality_fw_statistics_handler
 *
 * LRAC FW Statistics handler
 */
void app_lrac_quality_fw_statistics_handler(wiced_bt_lrac_fw_statistics_t *p_data);

/*
 * app_lrac_quality_audio_gitch_handler
 *
 * LRAC Audio Glitch handler
 */
void app_lrac_quality_audio_gitch_handler(wiced_bt_lrac_audio_glitch_t *p_data);

/*
 * app_lrac_quality_fw_statistics_handler
 */
void app_lrac_quality_fw_statistics_handler(wiced_bt_lrac_fw_statistics_t *p_data);

/*
 * app_lrac_quality_jitter_buffer_handler
 *
 * LRAC Jitter Buffer handler
 */
void app_lrac_quality_jitter_buffer_handler(wiced_bt_lrac_jitter_buffer_t *p_data);

/*
 * app_lrac_quality_peer_handler
 *
 * LRAC Peer Quality handler
 */
void app_lrac_quality_peer_handler(uint8_t *p_data, uint16_t length);
