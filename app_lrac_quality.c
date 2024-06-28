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

/*
 * The purpose of this file is to receive all the LRAC Quality information (Statistics, RSSI, etc)
 * from the local and the peer device.
 * The decision to perform a Primary/Secondary Switch will be based on all these values.
 * Other informations may also be taken into account (e.g. Battery level of both devices, etc).
 */
#include "wiced.h"
#include "wiced_timer.h"
#include "wiced_bt_lrac.h"
#include "app_lrac.h"
#include "app_lrac_quality.h"
#include "app_trace.h"
#include <wiced_utilities.h>

/*
 * Definitions
 */
/* Uncomment the following line to exchange Quality values (RSSI, Statistics, etc) with peer */
/* #define APP_LRAC_QUALITY_PEER_DATA */

#define APP_LRAC_QUALITY_TIMER_DURATION         5   /* in seconds */

/* The eLNA Gain is board specific */
#ifndef APP_LRAC_QUALITY_ELNA_GAIN
#define APP_LRAC_QUALITY_ELNA_GAIN              0   /* The ePA on EVK1 has a null LNA gain */
#endif

typedef struct
{
    uint8_t num_phone;
    int8_t phone_avg_rssi[WICED_BT_LRAC_MAX_AUDIO_SRC_CONNECTIONS];     /* RSSI value of Phones */
    int8_t peer_avg_rssi;       /* RSSI value of the peer WASS device */
} app_lrac_quality_rssi_t;

typedef struct
{
    uint32_t underrun_count;
    uint32_t overrun_count;
    uint32_t system_underrun_count;
} app_lrac_jitter_buffer_t;

typedef struct
{
    uint32_t miss_packet_count;
    uint32_t out_of_sync_hw_cnt;
    uint32_t out_of_sync_sw_cnt;
    uint32_t corrupt_pkt_cnt;
    uint32_t overrun_cnt;
    uint32_t late_delivery_cnt;
} app_lrac_quality_lrac_audio_glitch_t;

typedef struct
{
    wiced_bt_lrac_fw_statistics_t fw_statistics;
    app_lrac_jitter_buffer_t jitter_buffer_counters;
    app_lrac_quality_lrac_audio_glitch_t audio_glitch_counters;
    app_lrac_quality_rssi_t rssi;
} app_lrac_quality_info_t;

typedef struct
{
    app_lrac_quality_callback_t *p_callback;
    wiced_timer_t timer;
    wiced_bt_lrac_fw_statistics_t acculumated_fw_statistics;
    app_lrac_quality_info_t local;      /* Local Quality information */
#ifdef APP_LRAC_QUALITY_PEER_DATA
    app_lrac_quality_info_t peer;       /* Peer Quality information */
#endif
} app_lrac_quality_cb_t;

/*
 * Global variables
 */
static app_lrac_quality_cb_t app_lrac_quality_cb;

/*
 * External functions
 */
uint16_t wiced_audio_sink_get_jitter_buffer_size(void);
uint8_t wiced_audio_sink_get_jitter_buffer_target(void);

/*
 * Local functions
 */
static void app_lrac_quality_timer_callback(uint32_t param);
static void app_lrac_quality_print(char *p_prefix, app_lrac_quality_info_t *p_quality_info);

/*
 * app_lrac_quality_rssi_handler
 */
wiced_result_t app_lrac_quality_init(app_lrac_quality_callback_t *p_callback)
{
    memset(&app_lrac_quality_cb, 0, sizeof(app_lrac_quality_cb));

    app_lrac_quality_cb.p_callback = p_callback;

    wiced_init_timer(&app_lrac_quality_cb.timer,
            app_lrac_quality_timer_callback, 0, WICED_SECONDS_PERIODIC_TIMER);

    return WICED_BT_SUCCESS;
}

/*
 * app_lrac_quality_timer_callback
 */
static void app_lrac_quality_timer_callback(uint32_t param)
{
#ifdef APP_LRAC_QUALITY_PEER_DATA
    app_lrac_send_quality(&app_lrac_quality_cb.local, sizeof(app_lrac_quality_cb.local));
#endif

    /* Print the Local Statistics */
    app_lrac_quality_print("Local", &app_lrac_quality_cb.local);

    /* Reset the Local Statistics */
    memset(&app_lrac_quality_cb.local, 0, sizeof(app_lrac_quality_cb.local));
}

/*
 * app_lrac_quality_timer_start
 */
wiced_result_t app_lrac_quality_timer_start(void)
{
    /* Check if the timer is in use now. */
    if (wiced_is_timer_in_use(&app_lrac_quality_cb.timer))
    {
        return WICED_BT_SUCCESS;
    }

    /* Reset the local Statistics */
    memset(&app_lrac_quality_cb.local, 0, sizeof(app_lrac_quality_cb.local));

    /* Reset the Accumulated FW Statistics */
    memset(&app_lrac_quality_cb.acculumated_fw_statistics, 0,
            sizeof(app_lrac_quality_cb.acculumated_fw_statistics));

#ifdef APP_LRAC_QUALITY_PEER_DATA
    /* Reset the Peer Statistics */
    memset(&app_lrac_quality_cb.peer, 0, sizeof(app_lrac_quality_cb.peer));
#endif

    /* Start the Periodic timer */
    wiced_start_timer(&app_lrac_quality_cb.timer, APP_LRAC_QUALITY_TIMER_DURATION);

    return WICED_BT_SUCCESS;
}

/*
 * app_lrac_quality_timer_stop
 */
wiced_result_t app_lrac_quality_timer_stop(void)
{
    /* Print the Locally Accumulated FW Statistics */
    WICED_BT_TRACE("[Local Accumulated] FW_STATISTICS good:%d retx:%d missed:%d bad:%d\n",
            app_lrac_quality_cb.acculumated_fw_statistics.nb_good,
            app_lrac_quality_cb.acculumated_fw_statistics.nb_re_tx,
            app_lrac_quality_cb.acculumated_fw_statistics.nb_missed,
            app_lrac_quality_cb.acculumated_fw_statistics.nb_bad);

    /* Stop the Periodic timer */
    wiced_stop_timer(&app_lrac_quality_cb.timer);

    return WICED_BT_SUCCESS;
}

/*
 * app_lrac_quality_rssi_handler
 *
 * LRAC RSSI handler
 */
void app_lrac_quality_rssi_handler(wiced_bt_lrac_rssi_t *p_data)
{
    uint32_t w;

    /* Save the, locally measured, RSSI values for Phones (if valid) and add LNA Gain */
    app_lrac_quality_cb.local.rssi.num_phone = p_data->num_phone;
    for (w = 0; w < p_data->num_phone; w++)
    {
        app_lrac_quality_cb.local.rssi.phone_avg_rssi[w] = p_data->phone_link[w].avg_rssi;
        app_lrac_quality_cb.local.rssi.phone_avg_rssi[w] += APP_LRAC_QUALITY_ELNA_GAIN;
    }
    for ( ; w < _countof(app_lrac_quality_cb.local.rssi.phone_avg_rssi); w++)
    {
        app_lrac_quality_cb.local.rssi.phone_avg_rssi[w] = 0;
    }

    /* Save the, locally measured, RSSI value for the PS Link (if valid) and add LNA Gain */
    if (p_data->ps_link.conn_handle != 0xFFF)
    {
        app_lrac_quality_cb.local.rssi.peer_avg_rssi = p_data->ps_link.avg_rssi;
        app_lrac_quality_cb.local.rssi.peer_avg_rssi += APP_LRAC_QUALITY_ELNA_GAIN;
    }
    else
    {
        app_lrac_quality_cb.local.rssi.peer_avg_rssi = 0;
    }

    WICED_BT_TRACE("[Local] RSSI ");
    for (w = 0; w < app_lrac_quality_cb.local.rssi.num_phone; w++)
    {
        WICED_BT_TRACE("Phone #%d:%d ",
                w, (int)app_lrac_quality_cb.local.rssi.phone_avg_rssi[w]);
    }
    WICED_BT_TRACE("PS-Link:%d\n", (int)app_lrac_quality_cb.local.rssi.peer_avg_rssi);
}

/*
 * app_lrac_quality_fw_statistics_handler
 *
 * LRAC FW Statistics handler
 */
void app_lrac_quality_fw_statistics_handler(wiced_bt_lrac_fw_statistics_t *p_data)
{
    /* Accumulate the, locally measured, FW Statistics. Will be periodically reset */
    app_lrac_quality_cb.local.fw_statistics.nb_good += p_data->nb_good;
    app_lrac_quality_cb.local.fw_statistics.nb_re_tx += p_data->nb_re_tx;
    app_lrac_quality_cb.local.fw_statistics.nb_missed += p_data->nb_missed;
    app_lrac_quality_cb.local.fw_statistics.nb_bad += p_data->nb_bad;

    /* Accumulate the, locally measured, FW Statistics in another structure (not periodically reset) */
    app_lrac_quality_cb.acculumated_fw_statistics.nb_good += p_data->nb_good;
    app_lrac_quality_cb.acculumated_fw_statistics.nb_re_tx += p_data->nb_re_tx;
    app_lrac_quality_cb.acculumated_fw_statistics.nb_missed += p_data->nb_missed;
    app_lrac_quality_cb.acculumated_fw_statistics.nb_bad += p_data->nb_bad;
}

/*
 * app_lrac_quality_audio_gitch_handler
 *
 * LRAC Audio Glitch handler
 */
void app_lrac_quality_audio_gitch_handler(wiced_bt_lrac_audio_glitch_t *p_data)
{
    uint32_t pkt_cnt;

    switch (p_data->type)
    {
    case WICED_BT_LRAC_AUDIO_GLITCH_TYPE_NONE:
        break;

    case WICED_BT_LRAC_AUDIO_GLITCH_TYPE_OUT_OF_SYNC_ADJ_HW:
        app_lrac_quality_cb.local.audio_glitch_counters.out_of_sync_hw_cnt++;
        break;

    case WICED_BT_LRAC_AUDIO_GLITCH_TYPE_OUT_OF_SYNC_ADJ_SW:
        app_lrac_quality_cb.local.audio_glitch_counters.out_of_sync_sw_cnt++;
        break;

    case WICED_BT_LRAC_AUDIO_GLITCH_TYPE_MISS_PACKET:
        pkt_cnt = (p_data->cur_seq - p_data->last_seq) & 0xFFFF;
        app_lrac_quality_cb.local.audio_glitch_counters.miss_packet_count += (pkt_cnt - 1);
        break;

    case WICED_BT_LRAC_AUDIO_GLITCH_TYPE_CORRUPT_PKT:
        app_lrac_quality_cb.local.audio_glitch_counters.corrupt_pkt_cnt++;
        break;

    case WICED_BT_LRAC_AUDIO_GLITCH_TYPE_OVERRUN:
        app_lrac_quality_cb.local.audio_glitch_counters.overrun_cnt++;
        break;

    case WICED_BT_LRAC_AUDIO_GLITCH_TYPE_LATE_DELIVERY:
        app_lrac_quality_cb.local.audio_glitch_counters.late_delivery_cnt++;
        break;

    default:
        break;
    }
}

/*
 * app_lrac_quality_jitter_buffer_handler
 *
 * LRAC Jitter Buffer handler
 */
void app_lrac_quality_jitter_buffer_handler(wiced_bt_lrac_jitter_buffer_t *p_data)
{
    switch (p_data->state)
    {
    case WICED_BT_LRAC_JITTER_BUFFER_STATE_UNDERRUN:
        app_lrac_quality_cb.local.jitter_buffer_counters.underrun_count++;

        /* A2DP UnderRun. Send this information to the main */
        if (app_lrac_quality_cb.p_callback)
            app_lrac_quality_cb.p_callback(APP_LRAC_QUALITY_UNDERRUN, NULL);
        break;

    case WICED_BT_LRAC_JITTER_BUFFER_STATE_OVERRUN:
        app_lrac_quality_cb.local.jitter_buffer_counters.overrun_count++;
        break;

    case WICED_BT_LRAC_JITTER_BUFFER_STATE_SYSTEM_UNDERRUN:
        app_lrac_quality_cb.local.jitter_buffer_counters.system_underrun_count++;

        /* A2DP UnderRun. Send this information to the main */
        if (app_lrac_quality_cb.p_callback)
            app_lrac_quality_cb.p_callback(APP_LRAC_QUALITY_UNDERRUN, NULL);
        break;

    default:
        break;
    }
}

/*
 * app_lrac_quality_peer_handler
 *
 * LRAC Peer Quality handler
 */
void app_lrac_quality_peer_handler(uint8_t *p_data, uint16_t length)
{
#ifdef APP_LRAC_QUALITY_PEER_DATA
    if (length != sizeof(app_lrac_quality_cb.peer))
    {
        APP_TRACE_ERR("Bad Length:%d\n", length);
        return;
    }

    /* Copy the Quality data received from Peer device */
    memcpy(&app_lrac_quality_cb.peer, p_data, length);

    /* Print the received Quality data */
    app_lrac_quality_print("Peer", &app_lrac_quality_cb.peer);

    /* Reset the Peer Statistics */
    memset(&app_lrac_quality_cb.peer, 0, sizeof(app_lrac_quality_cb.peer));
#endif
}

/*
 * app_lrac_quality_print
 */
static void app_lrac_quality_print(char *p_prefix, app_lrac_quality_info_t *p_quality_info)
{
    /* Print FW Statistics */
    if (p_quality_info->fw_statistics.nb_good != 0)
    {
        WICED_BT_TRACE("[%s] FW_STATISTICS good:%d retx:%d missed:%d bad:%d\n", p_prefix,
                p_quality_info->fw_statistics.nb_good,
                p_quality_info->fw_statistics.nb_re_tx,
                p_quality_info->fw_statistics.nb_missed,
                p_quality_info->fw_statistics.nb_bad);
    }

    /* Print Audio Glitch statistics */
    if ((p_quality_info->audio_glitch_counters.miss_packet_count != 0) ||
        (p_quality_info->audio_glitch_counters.out_of_sync_hw_cnt != 0) ||
        (p_quality_info->audio_glitch_counters.out_of_sync_sw_cnt != 0) ||
        (p_quality_info->audio_glitch_counters.corrupt_pkt_cnt != 0) ||
        (p_quality_info->audio_glitch_counters.overrun_cnt != 0) ||
        (p_quality_info->audio_glitch_counters.late_delivery_cnt != 0))
    {
        WICED_BT_TRACE("[%s] Audio Glitch: miss_pkt:%d sync_hw:%d sync_sw:%d corrupt:%d overrun:%d late_delivery:%d\n",
                p_prefix,
                p_quality_info->audio_glitch_counters.miss_packet_count,
                p_quality_info->audio_glitch_counters.out_of_sync_hw_cnt,
                p_quality_info->audio_glitch_counters.out_of_sync_sw_cnt,
                p_quality_info->audio_glitch_counters.corrupt_pkt_cnt,
                p_quality_info->audio_glitch_counters.overrun_cnt,
                p_quality_info->audio_glitch_counters.late_delivery_cnt);
    }

    /* Print local Jitter Buffer state statistics. */
    if ((p_quality_info->jitter_buffer_counters.underrun_count != 0) ||
        (p_quality_info->jitter_buffer_counters.overrun_count != 0)  ||
        (p_quality_info->jitter_buffer_counters.system_underrun_count != 0))
    {
        WICED_BT_TRACE("[%s] Jitter Buffer: underrun:%d overrun:%d system_underrun:%d\n",
                p_prefix,
                p_quality_info->jitter_buffer_counters.underrun_count,
                p_quality_info->jitter_buffer_counters.overrun_count,
                p_quality_info->jitter_buffer_counters.system_underrun_count);
    }

#if 0
    /* Test/Debug code. */
    WICED_BT_TRACE("Current Jitter Buffer Level:%d\n",
            wiced_bt_lrac_audio_jitter_buffer_level_get());
    WICED_BT_TRACE("Current Jitter Size:%d target:%d\n",
            wiced_audio_sink_get_jitter_buffer_size(),
            wiced_audio_sink_get_jitter_buffer_target());
#endif
}
