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

#include "app_volume.h"
#include "app_trace.h"
#include "bt_hs_spk_handsfree.h"
#include "bt_hs_spk_handsfree_utils.h"
#include "bt_hs_spk_audio.h"
#include "wiced_timer.h"

typedef struct
{
    int32_t am_vol_level;
#ifdef VOLUME_EFFECT
    uint8_t am_vol_effect;              /* Store local volume effect state */
    wiced_timer_t volume_effect_timer;
#endif
} app_volume_cb_t;

/*
 * Global variables
 */
static app_volume_cb_t app_volume_cb = {0};

#ifdef VOLUME_EFFECT
static void app_volume_effect_timer_callback(uint32_t param)
{
    wiced_result_t status;
    int32_t am_vol_target;

    if (bt_hs_spk_handsfree_sco_connection_check(NULL))
    {
        /* HFP */
        if ( bt_hs_spk_handsfree_volume_get() != 0)
        {
            am_vol_target = bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(bt_hs_spk_handsfree_volume_get());
        }
        else
        {
            /* use Primay's am volume */
            am_vol_target = app_volume_get();
        }
        APP_TRACE_DBG("app_volume_effect_timer_callback(HFP) am_vol_target=%d\n", am_vol_target);

        app_volume_cb.am_vol_effect = VOLUME_EFFECT_RAMP_UP;
        bt_hs_spk_handsfree_audio_manager_stream_volume_set(am_vol_target, VOLUME_EFFECT_RAMP_UP);

        app_volume_cb.am_vol_effect = VOLUME_EFFECT_NONE;
    }
    else
    {
        /* A2DP */
        if (bt_hs_spk_audio_volume_get() != 0)
        {
            am_vol_target = bt_hs_spk_audio_utils_abs_volume_to_am_volume(bt_hs_spk_audio_volume_get());
        }
        else
        {
            /* use Primay's am volume */
            am_vol_target = app_volume_get();
        }

        APP_TRACE_DBG("app_volume_effect_timer_callback(A2DP) am_vol_target=%d\n", am_vol_target);

        app_volume_cb.am_vol_effect = VOLUME_EFFECT_RAMP_UP;
        bt_hs_spk_audio_audio_manager_stream_volume_set(am_vol_target, VOLUME_EFFECT_RAMP_UP);

        app_volume_cb.am_vol_effect = VOLUME_EFFECT_NONE;
    }
}
#endif

/*
 * app_volume_init
 */
wiced_result_t app_volume_init(void)
{
    app_volume_cb.am_vol_level = AM_VOL_LEVEL_HIGH / 2;

#ifdef VOLUME_EFFECT
    wiced_init_timer(&app_volume_cb.volume_effect_timer,
        app_volume_effect_timer_callback, 0, WICED_MILLI_SECONDS_TIMER);
#endif

    return WICED_BT_SUCCESS;
}

/*
 * app_volume_get
 *
 * Get current volume level for Audio Manager
 */
int32_t app_volume_get(void)
{
    return app_volume_cb.am_vol_level;
}

/*
 * app_volume_set
 *
 * @param[in] volume_select
 * @param[in] am_vol_level
 */
void app_volume_set(app_volume_select_t volume_select, int32_t am_vol_level, uint8_t am_vol_effect_event)
{
    switch (volume_select)
    {
    case APP_VOLUME_SELECT_VOICE:
        bt_hs_spk_handsfree_audio_manager_stream_volume_set(am_vol_level, am_vol_effect_event);
        break;
    case APP_VOLUME_SELECT_STREAM:
        bt_hs_spk_audio_audio_manager_stream_volume_set(am_vol_level, am_vol_effect_event);
        break;
    case APP_VOLUME_SELECT_NONE:
        /* only store am_vol_level */
        break;
    default:
        return;
    }

    app_volume_cb.am_vol_level = am_vol_level;
}

#ifdef VOLUME_EFFECT
void app_volume_effect_start_timer(uint32_t time_ms, uint8_t am_vol_effect_event)
{
    app_volume_cb.am_vol_effect = am_vol_effect_event;
    wiced_start_timer(&app_volume_cb.volume_effect_timer, time_ms);
}

void app_volume_effect_stop_timer(void)
{
    if (wiced_is_timer_in_use(&app_volume_cb.volume_effect_timer))
    {
        APP_TRACE_DBG("Stop app_volume_effect_timer\n");
        wiced_stop_timer(&app_volume_cb.volume_effect_timer);
    }
}

wiced_bool_t app_volume_effect_ready_to_switch(void)
{
    if (wiced_is_timer_in_use(&app_volume_cb.volume_effect_timer))
    {
        APP_TRACE_ERR("Audio Effect running. PS-Switch not allowed\n");
        return WICED_FALSE;
    }

    return WICED_TRUE;
}
#endif
