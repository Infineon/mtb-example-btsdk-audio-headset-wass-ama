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

#define VOLUME_RAMP_UP_TIME_INIT_MUTE 30        /* volume effect timer will be activated 30ms later  (init mute) */
#define VOLUME_RAMP_UP_TIME_JOIN_LATE 750       /* volume effect timer will be activated 750ms later (join late) */
#define VOLUME_RAMP_UP_TIME_UNDERRUN  750       /* volume effect timer will be activated 750ms later (underrun) */
#define VOLUME_RAMP_UP_TIME_INIT_HFP_MUTE 1500  /* volume effect timer will be activated 1500ms later (init mute) */

/* NOTE: The init mute time is based on the wave file analysis with audio pop sound,
 *       The underrun and join late time are based on the max playing time of jitter buffer + WASS storage.
 *       Ttoal packets will be 20+12 AAC packets, 32 * 23.2(one AAC packet play 23.2ms) = 742.4ms
 */

typedef enum
{
    APP_VOLUME_SELECT_VOICE,
    APP_VOLUME_SELECT_STREAM,
    APP_VOLUME_SELECT_NONE,     /* only store am_vol_level */
} app_volume_select_t;

/*
 * app_volume_init
 */
wiced_result_t app_volume_init(void);

/*
 * app_volume_get
 *
 * Get current volume level for Audio Manager
 */
int32_t app_volume_get(void);

/*
 * app_volume_set
 *
 * @param[in] volume_select
 * @param[in] am_vol_level
 */
void app_volume_set(app_volume_select_t volume_select, int32_t am_vol_level, uint8_t am_vol_effect);

#ifdef VOLUME_EFFECT
void app_volume_effect_start_timer(uint32_t time_ms, uint8_t am_vol_effect_event);

void app_volume_effect_stop_timer(void);

wiced_bool_t app_volume_effect_ready_to_switch(void);
#endif
