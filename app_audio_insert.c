/*
 *  Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
 *  an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * 
 *  This software, including source code, documentation and related
 *  materials ("Software") is owned by Cypress Semiconductor Corporation
 *  or one of its affiliates ("Cypress") and is protected by and subject to
 *  worldwide patent protection (United States and foreign),
 *  United States copyright laws and international treaty provisions.
 *  Therefore, you may use this Software only as provided in the license
 *  agreement accompanying the software package from which you
 *  obtained this Software ("EULA").
 *  If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 *  non-transferable license to copy, modify, and compile the Software
 *  source code solely for use in connection with Cypress's
 *  integrated circuit products.  Any reproduction, modification, translation,
 *  compilation, or representation of this Software except as specified
 *  above is prohibited without the express written permission of Cypress.
 * 
 *  Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 *  reserves the right to make changes to the Software without notice. Cypress
 *  does not assume any liability arising out of the application or use of the
 *  Software or any product or circuit described in the Software. Cypress does
 *  not authorize its products for use in any products where a malfunction or
 *  failure of the Cypress product may reasonably be expected to result in
 *  significant property damage, injury or death ("High Risk Product"). By
 *  including Cypress's product in a High Risk Product, the manufacturer
 *  of such system or application assumes all risk of such use and in doing
 *  so agrees to indemnify Cypress against all liability.
 */

#include <stdint.h>
#include "wiced.h"
#include "wiced_timer.h"
#include "wiced_bt_sco_hook.h"
#ifdef VOICE_PROMPT
#include "wiced_bt_voice_prompt.h"
#include "app_cpu_clock.h"
#endif // VOICE_PROMPT
#include "app_trace.h"
#include "app_audio_insert.h"
#include "app_main.h"
#include "app_volume.h"
#include "bt_hs_spk_handsfree.h"
#include "bt_hs_spk_audio.h"
#include "bt_hs_spk_audio_insert.h"
#include "wiced_bt_lrac.h"
#include "wiced_bt_event.h"
#include "wiced_bt_audio_insert.h"

/*
 * Definitions
 */
#define APP_AUDIO_INSERT_QUEUE_SIZE     3

#define APP_AUDIO_INSERT_SCO_DELAY      200     /* SCO Delay in milliseconds */

typedef enum
{
    APP_AUDIO_INSERT_STATE_IDLE = 0,
    APP_AUDIO_INSERT_STATE_STARTING,
    APP_AUDIO_INSERT_STATE_STARTED,
    APP_AUDIO_INSERT_STATE_STOPPING,
} app_audio_insert_state_t;

typedef struct
{
    app_audio_insert_callback_t *p_callback;

    app_audio_insert_state_t state;

    bt_hs_spk_audio_insert_config_t config;

    uint8_t file_index;
    uint32_t expected_sco_time_seq_num;
    uint8_t api_req;   /* Indicates if the current AudioInsert is from the API */

    uint8_t suspended;
    uint8_t nb_enqueued;    /* Number of Voice Prompt Messages enqueued */
    uint8_t queue[APP_AUDIO_INSERT_QUEUE_SIZE];

#ifdef VOICE_PROMPT
    pcm_s16_t voice_prompt_samples[WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_AUDIO * 2];
#endif

} app_audio_insert_cb_t;

/*
 * Local functions
 */
static wiced_result_t app_audio_insert_start(uint8_t audio_file_index);
static wiced_result_t app_audio_insert_start_req_handler(uint8_t audio_file_index);
#ifndef VOICE_PROMPT
static void app_audio_insert_timer_callback(void);
#endif // !VOICE_PROMPT
static wiced_result_t app_audio_insert_start_continue(void);
static void app_audio_insert_start_wait_callback(void);

static uint32_t app_audio_insert_sco_expected_time_sequence_number_calculate(
        uint32_t current_time_sequence_number, uint32_t delay_time);

static void app_audio_insert_state_set(app_audio_insert_state_t state);
static app_audio_insert_state_t app_audio_insert_state_get(void);

static wiced_result_t app_audio_insert_queue_insert(uint8_t audio_file_index);
static wiced_result_t app_audio_insert_queue_extract(uint8_t *p_audio_file_index);
static wiced_result_t app_audio_insert_queue_read(uint8_t *p_audio_file_index);
static void app_audio_insert_queue_remove(uint8_t file_index);
static wiced_bool_t app_audio_insert_queue_is_full(void);
static wiced_bool_t app_audio_insert_queue_is_empty(void);

static uint32_t app_audio_insert_sampling_rate_get(void);
static void app_audio_insert_config_data_set(wiced_bool_t multiple);

#ifdef VOICE_PROMPT
static void             app_audio_insert_source_data_exhausted_handler(void);
static void             app_audio_insert_source_data_fill(void);
static int              app_audio_insert_source_data_generate(void *p_data);
static wiced_result_t   app_audio_insert_source_data_prepare(void);
#endif // VOICE_PROMPT
/*
 * Global variables
 */
static app_audio_insert_cb_t app_audio_insert_cb;

/*
 * app_audio_insert_adv_audio_enable
 *
 * Enable the advanced audio insert.
 */
static void app_audio_insert_adv_audio_enable(void)
{
    if (app_lrac_config_role_get() == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        if (wiced_bt_lrac_audio_insert_enable(app_audio_insert_sampling_rate_get(),
                                              wiced_bt_lrac_audio_insert_handle_get()) == WICED_FALSE)
        {
            APP_TRACE_ERR("app_audio_insert_adv_audio_enable\n");
        }
    }
}

/*
 * app_audio_insert_adv_audio_disable
 *
 * Disable the advanced audio insert.
 */
static void app_audio_insert_adv_audio_disable(void)
{
    if (wiced_bt_lrac_audio_insert_disable() == WICED_FALSE)
    {
        APP_TRACE_ERR("app_audio_insert_adv_audio_disable\n");
    }
}

/*
 * app_audio_insert_init
 */
wiced_result_t app_audio_insert_init(app_audio_insert_callback_t *p_callback)
{
    wiced_bt_audio_insert_advanced_control_config_t adv_config = {0};

    memset(&app_audio_insert_cb, 0, sizeof(app_audio_insert_cb));

    /* Save the application's callback */
    app_audio_insert_cb.p_callback = p_callback;

    /* Install advanced audio insert feature. */
    adv_config.audio.p_enable = &app_audio_insert_adv_audio_enable;
    adv_config.audio.p_disable = &app_audio_insert_adv_audio_disable;

    wiced_bt_audio_insert_advanced_control_utility_install(&adv_config);

    return WICED_BT_SUCCESS;
}

/*
 * app_audio_insert_start_req
 * This function is called when the application requests Audio Insert
 */
wiced_result_t app_audio_insert_start_req(uint8_t audio_file_index)
{
    wiced_result_t status;

    APP_TRACE_DBG("AudioInsertStartReq file:%d\n", audio_file_index);

    app_audio_insert_cb.suspended = WICED_FALSE;

    /* Insert the Audio File Index in the Queue */
    status = app_audio_insert_queue_insert(audio_file_index);
    if (status != WICED_BT_SUCCESS)
    {
        return status;
    }

    /* If no Audio Insertion ongoing */
    if (app_audio_insert_state_get() == APP_AUDIO_INSERT_STATE_IDLE)
    {
        /* We can try to start Audio Insertion immediately */
        status = app_audio_insert_start(audio_file_index);
        /* If Audio Insert is in a bad state (e.g. Starting/Stopping Eavesdropping) */
        if (status == WICED_BT_BUSY)
        {
            /* The request in enqueued. It will be played later */
            status = WICED_BT_SUCCESS;
        }
    }

    return status;
}

/*
 * app_audio_insert_start
 */
static wiced_result_t app_audio_insert_start(uint8_t audio_file_index)
{
    wiced_result_t status;

    APP_TRACE_DBG("AudioInsertStart file:%d\n", audio_file_index);

    /* If state is not Idle (Already playing) */
    if (app_audio_insert_state_get() != APP_AUDIO_INSERT_STATE_IDLE)
    {
        APP_TRACE_ERR("AudioInsert already ongoing\n");
        return WICED_BT_ERROR;
    }

    app_audio_insert_cb.file_index = audio_file_index;
    app_audio_insert_cb.api_req = WICED_TRUE;
    app_audio_insert_state_set(APP_AUDIO_INSERT_STATE_STARTING);

    /* Exit sniff power_mgmt */
    if (app_lrac_config_role_get() == WICED_BT_LRAC_ROLE_PRIMARY &&
        app_lrac_is_connected())
    {
        if (wiced_bt_lrac_sniff_power_mgmt_exit(
                app_audio_insert_start_wait_callback) == WICED_BT_PENDING)
        {
            APP_TRACE_DBG("Waiting PS link to Exit Long Sniff\n");
            /* wait until power management paused */
            return WICED_BT_SUCCESS;
        }
    }

    /* No need to wait for Power Management to Exit Sniff */
    return app_audio_insert_start_continue();
}

/*
 * app_audio_insert_start_wait_callback
 * This, internal, function is called when Sniff mode is exited
 */
static void app_audio_insert_start_wait_callback(void)
{
    APP_TRACE_DBG("app_audio_insert_start_wait_callback\n");

    /* Continue Voice Prompt process */
    app_audio_insert_start_continue();
}

/*
 * app_audio_insert_start_continue
 */
static wiced_result_t app_audio_insert_start_continue(void)
{
    wiced_result_t status;
    uint8_t abs_vol;
    int32_t am_vol_level;
    wiced_bool_t local;

#ifdef VOICE_PROMPT
    /* Prepare the insertion data. */
    status = app_audio_insert_source_data_prepare();

    if (status != WICED_BT_SUCCESS)
    {
        app_audio_insert_state_set(APP_AUDIO_INSERT_STATE_IDLE);

        return status;
    }
#endif // VOICE_PROMPT

    /* Sync. the volume with Secondary in IDLE case.
     * The volume between Primary and Secondary are already synchronized
     * once the HFP audio connection is established or the audio streaming is
     * started. */
    if (app_lrac_is_connected() &&
        (app_lrac_config_role_get() == WICED_BT_LRAC_ROLE_PRIMARY) &&
        (bt_hs_spk_handsfree_sco_connection_check(NULL) == WICED_FALSE) &&
        (bt_hs_spk_audio_streaming_check(NULL) == WICED_NOT_FOUND))
    {
        abs_vol = bt_hs_spk_audio_volume_get();

        if (abs_vol)
        {
            am_vol_level = bt_hs_spk_audio_utils_abs_volume_to_am_volume((int32_t) abs_vol);
        }
        else
        {
            am_vol_level = app_volume_get();
        }

        app_lrac_volume_send(am_vol_level, VOLUME_EFFECT_NONE);
    }

    if (bt_hs_spk_handsfree_sco_connection_check(NULL))
    {
        /* Add a delay (counted in Slot) to synchronize both sides */
        app_audio_insert_cb.expected_sco_time_seq_num =
                app_audio_insert_sco_expected_time_sequence_number_calculate(
                        wiced_bt_audio_insert_sco_in_data_latest_time_sequence_number_get(),
                        APP_AUDIO_INSERT_SCO_DELAY);
    }
    else
    {
        app_audio_insert_cb.expected_sco_time_seq_num = 0;
    }

    /* By default, Primary will insert Audio on both sides */
    if (app_lrac_config_role_get() == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        if (app_lrac_is_connected())
        {
            local = WICED_FALSE;
        }
        else
        {
            local = WICED_TRUE;
        }
    }
    else
    {
        local = WICED_TRUE; /* Secondary can only Play locally */
    }

    /* Ask LRAC Library to Start audio Insertion. */
    status = wiced_bt_lrac_audio_insert_start_req(app_audio_insert_cb.file_index,
            local, app_audio_insert_cb.expected_sco_time_seq_num);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_bt_lrac_audio_insert_start_req failed:%d\n", status);
        app_audio_insert_state_set(APP_AUDIO_INSERT_STATE_IDLE);
#ifdef VOICE_PROMPT
        /* Close the Voice Prompt file */
        wiced_bt_voice_prompt_close();
#endif // VOICE_PROMPT
        return status;
    }

    return WICED_BT_SUCCESS;
}

/*
 * app_audio_insert_start_req_handler
 * Handle Audio Injection received from peer (Primary) device
 */
static wiced_result_t app_audio_insert_start_req_handler(uint8_t audio_file_index)
{
    wiced_result_t status = WICED_BT_SUCCESS;

    /* If state is not Idle (Already playing) */
    if (app_audio_insert_state_get() != APP_AUDIO_INSERT_STATE_IDLE)
    {
        APP_TRACE_ERR("AudioInsert already ongoing\n");
        /* Reply to the Audio Insert Request */
        wiced_bt_lrac_audio_insert_start_rsp(WICED_BT_ERROR);

        return WICED_BT_ERROR;
    }
    else
    {
        app_audio_insert_cb.file_index = audio_file_index;
#ifdef VOICE_PROMPT
        app_audio_insert_state_set(APP_AUDIO_INSERT_STATE_STARTING);

        /* Prepare the insertion data. */
        status = app_audio_insert_source_data_prepare();

        if (status != WICED_BT_SUCCESS)
        {
            app_audio_insert_state_set(APP_AUDIO_INSERT_STATE_IDLE);
        }
#endif
    }

    /* Reply to the Audio Insert Request */
    wiced_bt_lrac_audio_insert_start_rsp(status);

    return status;
}

/*
 * app_audio_insert_stop_req
 */
wiced_result_t app_audio_insert_stop_req(app_audio_insert_stop_req_t stop_request)
{
    wiced_result_t status;
    uint8_t audio_file_index;

    APP_TRACE_DBG("AudioInsertStopReq %d\n", stop_request);

    /* If state is not Idle (Already playing) */
    if (app_audio_insert_state_get() != APP_AUDIO_INSERT_STATE_STARTED)
    {
        APP_TRACE_ERR("AudioInsert Not Started\n");
        return WICED_BT_ERROR;
    }

    switch(stop_request)
    {
    case APP_AUDIO_INSERT_STOP_REQ_REGULAR:
        app_audio_insert_cb.suspended = WICED_FALSE;
        break;

    case APP_AUDIO_INSERT_STOP_REQ_SUSPEND:
        app_audio_insert_cb.suspended = WICED_TRUE;
        break;

    case APP_AUDIO_INSERT_STOP_REQ_FLUSH:
        app_audio_insert_cb.suspended = WICED_FALSE;
        /* If application requests to Stop & Flush */
        while (app_audio_insert_queue_is_empty() == WICED_FALSE)
        {
            app_audio_insert_queue_extract(&audio_file_index);
        }
        break;

    default:
        APP_TRACE_ERR("Wrong stop_request:%d\n", stop_request);
        return WICED_BT_BADARG;
    }

#ifdef VOICE_PROMPT
    /* Close the Voice Prompt file */
    status = wiced_bt_voice_prompt_close();
    if (status != WICED_BT_SUCCESS)
        APP_TRACE_ERR("wiced_bt_voice_prompt_close failed:%d\n", status);
#endif

    app_audio_insert_state_set(APP_AUDIO_INSERT_STATE_STOPPING);

    /* Ask LRAC Library to Stop Audio Insertion */
    status = wiced_bt_lrac_audio_insert_stop_req();
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_bt_lrac_audio_insert_stop_req failed:%d\n", status);
        app_audio_insert_state_set(APP_AUDIO_INSERT_STATE_STARTED);
        return status;
    }

    return status;
}

/*
 * app_audio_insert_resume
 * Resume (Restart) AudioInsert if needed
 */
void app_audio_insert_resume(void)
{
    wiced_result_t status;
    uint8_t audio_file_index;

    APP_TRACE_DBG("AudioInsertResume\n");

    /* If state is not Idle (Already playing) */
    if (app_audio_insert_state_get() != APP_AUDIO_INSERT_STATE_IDLE)
    {
        APP_TRACE_DBG("AudioInsertResume already playing\n");
        return;
    }

    if (app_audio_insert_cb.suspended)
    {
        /* If AudioInsert was suspended, do not try to play other enqueued message */
        APP_TRACE_DBG("AudioInsertResume suspended\n");
        return;
    }

    /* Audio Insert is Idle, check if there is something to play in the queue */
    if (app_audio_insert_queue_is_empty())
    {
        /* The queue is empty, nothing to play */
       return;
    }

    /* Read the pending Audio Insert Request from the Queue */
    status = app_audio_insert_queue_read(&audio_file_index);

    /* If the Audio Insert Queue is not empty, restart Audio Insert */
    if (status == WICED_BT_SUCCESS)
    {
        status = app_audio_insert_start(audio_file_index);
        if (status != WICED_BT_SUCCESS)
        {
            APP_TRACE_ERR("app_audio_insert_start failed:%d\n", status);
        }
    }
}

#ifndef VOICE_PROMPT
/*
 * app_audio_insert_timer_callback
 */
static void app_audio_insert_timer_callback(void)
{
    wiced_result_t result;

    APP_TRACE_DBG("AudioInsert Timer\n");

    /* Stop audio Insertion */
    result = app_audio_insert_stop_req(APP_AUDIO_INSERT_STOP_REQ_REGULAR);

    if (result != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("app_audio_insert_stop_req fail: %d\n", result);
    }

    (void) result;
}
#endif // !VOICE_PROMPT

/*
 * app_audio_insert_sco_expected_time_sequence_number_calculate
 *
 * Calculate the expected SCO data time sequence number after the specified delay time.
 *
 * @param[in]   current_time_sequence_number - current input SCO data time sequence number
 * @param[in]   delay_time - specified delay time in millisecond (maximum value is 1279 ms)
 *
 * @return      expected SCO data time sequence number
 */
static uint32_t app_audio_insert_sco_expected_time_sequence_number_calculate(
        uint32_t current_time_sequence_number, uint32_t delay_time)
{
    uint32_t expected_bt_clock_increment;
    uint32_t expected_time_sequence_number_increment;
    uint32_t expected_time_sequence_number;

    /*
     * The incoming SCO data time sequence number (wiced_bt_sco_hook_event_mic_samples_t)
     * is the Bluetooth clock shifted left with 4 bits and each increment in Bluetooth clock is (625 / 2) us.
     * Hence, we first calculate the expected Bluetooth clock increment for the specified delay time.
     *
     * Expected Bluetooth clock increment = delay_time / (0.625 / 2)
     *                                    = (delay_time * 2) / 0.625
     *                                    = (delay_time * 2) / (5 / 8)
     *                                    = (delay_time * 2 * 8 ) / 5
     *
     * To avoid the truncated error, we will round up the result. Therefore,
     * Expected Bluetooth clock increment = ((delay_time * 2 * 8) + 4) / 5
     */
    expected_bt_clock_increment = ((delay_time * 2 * 8) + 4) / 5;

    /* Left shit 4 bits to get the expected SCO data time sequence number increment. */
    expected_time_sequence_number_increment = expected_bt_clock_increment << 4;

    /* Calculate the expected SCO data time sequence number. */
    expected_time_sequence_number = current_time_sequence_number + expected_time_sequence_number_increment;

    if (expected_time_sequence_number > 0xFFFF)
    {
        expected_time_sequence_number -= 0xFFFF;
    }

    APP_TRACE_DBG("Audio Insert SCO delay:%d sco_seq_num:%d\n", delay_time,
            expected_time_sequence_number);

    return expected_time_sequence_number;
}

/*
 * app_audio_insert_handler
 * This function handles the LRAC Audio Insert Events
 */
void app_audio_insert_lrac_event_handler(wiced_bt_lrac_event_t event,
        wiced_bt_lrac_event_data_t *p_data)
{
    wiced_result_t status;
    app_audio_insert_event_data_t event_data;
    uint8_t audio_insert_index;

    switch(event)
    {
    case WICED_BT_LRAC_EVENT_AUDIO_INSERT_START_RSP:
        APP_TRACE_DBG("AUDIO_INSERT_START_RSP status:%d local:%d\n",
                p_data->audio_insert_start_rsp.status,
                p_data->audio_insert_start_rsp.local_audio_insert);
        if (p_data->audio_insert_start_rsp.status == WICED_BT_SUCCESS)
        {
            app_audio_insert_state_set(APP_AUDIO_INSERT_STATE_STARTED);

            /* This print is for SVT automation script */
            WICED_BT_TRACE("APP_AUDIO_INSERT_STARTED status:%d local:%d audio_file_index:%d sample_rate:%d\n",
            p_data->audio_insert_start_rsp.status,p_data->audio_insert_start_rsp.local_audio_insert,app_audio_insert_cb.file_index,app_audio_insert_sampling_rate_get());

            app_audio_insert_config_data_set(!p_data->audio_insert_start_rsp.local_audio_insert);
            bt_hs_spk_audio_insert_start(&app_audio_insert_cb.config);
        }
        else
        {
#ifdef VOICE_PROMPT
            wiced_bt_voice_prompt_close();
#endif // VOICE_PROMPT
        }
        break;

    case WICED_BT_LRAC_EVENT_AUDIO_INSERT_START_REQ:
        APP_TRACE_DBG("AUDIO_INSERT_START_REQ file:%d sco_seq_num:%d\n",
                p_data->audio_insert_start_req.audio_file_index,
                p_data->audio_insert_start_req.expected_sco_time_seq_num);

        /* Call the Audio Insert Start Request Handler */
        status = app_audio_insert_start_req_handler(p_data->audio_insert_start_req.audio_file_index);
        if (status == WICED_BT_SUCCESS)
        {
            /* We received an Audio Insertion Request from the peer device */
            /* So, this is not a Local Insert and not an API Request */
            app_audio_insert_cb.api_req = WICED_FALSE;
            app_audio_insert_state_set(APP_AUDIO_INSERT_STATE_STARTED);
            app_audio_insert_cb.expected_sco_time_seq_num =
                    p_data->audio_insert_start_req.expected_sco_time_seq_num;

            /* This print is for SVT automation script */
            WICED_BT_TRACE("APP_AUDIO_INSERT_STARTED status:%d local:%d audio_file_index:%d sample_rate:%d\n",
            status,WICED_FALSE,app_audio_insert_cb.file_index,app_audio_insert_sampling_rate_get());

            app_audio_insert_config_data_set(WICED_TRUE);
            /* Audio Insertion Request is from the peer device, don't enable timer */
            /* It will receive Stop Request from peer too */
            app_audio_insert_cb.config.duration = 0;

            bt_hs_spk_audio_insert_start(&app_audio_insert_cb.config);
        }
        break;

    case WICED_BT_LRAC_EVENT_AUDIO_INSERT_STOP_RSP:
        APP_TRACE_DBG("AUDIO_INSERT_STOP_RSP (%d, %d)\n",
                      app_audio_insert_state_get(),
                      p_data->audio_insert_stop_rsp.status);

        if (app_audio_insert_state_get() == APP_AUDIO_INSERT_STATE_IDLE)
        {
            break;
        }

        bt_hs_spk_audio_insert_stop();

        app_audio_insert_state_set(APP_AUDIO_INSERT_STATE_IDLE);

        /* Simulate an error to re-enter Sniff mode if No more Audio Insertion pending */
        status = WICED_BT_ERROR;

        if (app_audio_insert_cb.suspended)
        {
            /* If AudioInsert was suspended, do not try to play other enqueued message */
            APP_TRACE_DBG("AudioInsert suspended\n");
            app_audio_insert_cb.suspended = WICED_FALSE;
        }
        else
        {
            /* If it was an API Request (i.e. local request) */
            if (app_audio_insert_cb.api_req)
            {
                if (app_audio_insert_queue_is_empty() == WICED_FALSE)
                {
                    /* Extract the Audio Insert Index from the Queue */
                    app_audio_insert_queue_extract(&audio_insert_index);
                }
            }

            /* Check if there is a pending Audio Insert Request from the Queue */
            if (app_audio_insert_queue_is_empty() == WICED_FALSE)
            {
                status = app_audio_insert_queue_read(&audio_insert_index);
                if (status == WICED_BT_SUCCESS)
                {
                    /* If the Audio Insert Queue is not empty, restart Audio Insert */
                    status = app_audio_insert_start(audio_insert_index);
                    if (status != WICED_BT_SUCCESS)
                    {
                        APP_TRACE_ERR("app_audio_insert_start failed:%d\n", status);
                    }
                }
            }
        }

        /*
         * If the Audio Insert Queue is empty or if Audio Insert failed, tell the
         * application that Audio Insert is stopped
         */
        if (status != WICED_BT_SUCCESS)
        {
            if (app_lrac_config_role_get() == WICED_BT_LRAC_ROLE_PRIMARY)
            {
                if (app_lrac_is_connected())
                {
                    APP_TRACE_DBG("Power Save on PS-Link\n");

                    /* Re-enter sniff power_mgmt */
                    wiced_bt_lrac_sniff_power_mgmt_enter(NULL);
                }
            }
        }

        /* Send Audio Insert Stop event to APP */
        event_data.stopped.status = p_data->audio_insert_stop_rsp.status;
        app_audio_insert_cb.p_callback(APP_AUDIO_INSERT_STOPPED, &event_data);
        break;

    default:
        APP_TRACE_ERR("unknown event:%d\n", event);
        break;
    }
}

/*
 * app_audio_insert_is_started
 * This function is typically used to check if a PS-Switch is allowed
 */
wiced_bool_t app_audio_insert_is_started(void)
{
    /* If Audio Insert is Idle, return False */
    if (app_audio_insert_state_get() == APP_AUDIO_INSERT_STATE_IDLE)
    {
        return WICED_FALSE;
    }

    /* Return True is every other cases */
    return WICED_TRUE;
}

/*
 * app_audio_insert_state_set
 * Set AudioInsert state
 */
static void app_audio_insert_state_set(app_audio_insert_state_t state)
{
    app_audio_insert_cb.state = state;
}

/*
 * app_audio_insert_state_get
 * Get AudioInsert state
 */
static app_audio_insert_state_t app_audio_insert_state_get(void)
{
    return app_audio_insert_cb.state;
}

/*
 * app_audio_insert_queue_insert
 * Insert an element in the Queue
 */
static wiced_result_t app_audio_insert_queue_insert(uint8_t audio_file_index)
{
    /* Check if the Voice Prompt Queue is Full */
    if (app_audio_insert_queue_is_full())
    {
        APP_TRACE_ERR("VoicePrompt Queue Full\n");
        return WICED_QUEUE_FULL;
    }

    /* Enqueue the Audio file Index */
    app_audio_insert_cb.queue[app_audio_insert_cb.nb_enqueued] = audio_file_index;
    app_audio_insert_cb.nb_enqueued++;

    return WICED_BT_SUCCESS;
}

/*
 * app_audio_insert_queue_extract
 * Read & Extract an element from the Queue
 */
static wiced_result_t app_audio_insert_queue_extract(uint8_t *p_audio_file_index)
{
    uint8_t i;

    /* Check if the Voice Prompt Queue is Empty */
    if (app_audio_insert_queue_is_empty())
    {
        APP_TRACE_ERR("VoicePrompt Queue Empty\n");
        return WICED_QUEUE_EMPTY;
    }

    /* Read the head of the Queue */
    *p_audio_file_index = app_audio_insert_cb.queue[0];
    app_audio_insert_cb.nb_enqueued--;

    /* Shift the Queue elements. The First is always the Head */
    for ( i = 0 ; i < app_audio_insert_cb.nb_enqueued ; i++ )
    {
        app_audio_insert_cb.queue[i] = app_audio_insert_cb.queue[i + 1];
    }

    return WICED_BT_SUCCESS;
}

/*
 * app_audio_insert_queue_read
 * Read the First element of the Queue
 */
static wiced_result_t app_audio_insert_queue_read(uint8_t *p_audio_file_index)
{
    /* Check if the Voice Prompt Queue is Empty */
    if (app_audio_insert_queue_is_empty())
    {
        APP_TRACE_ERR("VoicePrompt Queue Empty\n");
        return WICED_QUEUE_EMPTY;
    }

    /* Read (do not extract the element) the head of the Queue */
    *p_audio_file_index = app_audio_insert_cb.queue[0];

    return WICED_BT_SUCCESS;
}

/*
 * app_audio_insert_queue_remove
 * Remove every reference to a file_index in the queue
 */
static void app_audio_insert_queue_remove(uint8_t file_index)
{
    uint8_t i, j;
    uint8_t enqueued = app_audio_insert_cb.nb_enqueued;

    /* Go through the queue */
    for ( i = 0 ; i < enqueued ; i++)
    {
        /* If this file_index must be suppressed */
        if (app_audio_insert_cb.queue[i] == file_index)
        {
            for ( j = i ; j < enqueued ; j++)
            {
                app_audio_insert_cb.queue[j] = app_audio_insert_cb.queue[j + 1];
            }
            app_audio_insert_cb.nb_enqueued--;
        }
    }
}

/*
 * app_audio_insert_queue_is_full
 * Check if the Queue is full
 */
static wiced_bool_t app_audio_insert_queue_is_full(void)
{
    if (app_audio_insert_cb.nb_enqueued >= APP_AUDIO_INSERT_QUEUE_SIZE)
        return WICED_TRUE;

    return WICED_FALSE;
}

/*
 * app_audio_insert_queue_is_empty
 * Check if the Queue is empty
 */
static wiced_bool_t app_audio_insert_queue_is_empty(void)
{
    if (app_audio_insert_cb.nb_enqueued == 0)
        return WICED_TRUE;

    return WICED_FALSE;
}

/*
 * app_audio_insert_sampling_rate_get
 *
 * Get current sampling rate used for audio insertion
 */
static uint32_t app_audio_insert_sampling_rate_get(void)
{
    if (bt_hs_spk_handsfree_audio_manager_stream_check())
    {
        return (uint32_t) bt_hs_spk_handsfree_audio_manager_sampling_rate_get();
    }
    else
    {
        if (bt_hs_spk_audio_audio_manager_stream_check())
        {
            return (uint32_t) bt_hs_spk_audio_audio_manager_sampling_rate_get();
        }
        else
        {
            return AM_PLAYBACK_SR_48K;
        }
    }
}

#ifdef VOICE_PROMPT
/*
 * app_audio_insert_source_data_prepare
 *
 * Prepare the insertion data.
 */
static wiced_result_t app_audio_insert_source_data_prepare(void)
{
    wiced_result_t status;
    uint32_t samples_nb_got;

    /* Increase CUP speed to reduce insertion data extraction, re-sampling, and filling. */
    app_cpu_clock_increase(APP_CPU_CLOCK_AUDIO_INSERT);

    /* Open the Voice Prompt file */
    status = wiced_bt_voice_prompt_open(app_audio_insert_cb.file_index);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_bt_voice_prompt_open failed: %d\n", status);

        /*
         * If this audio File cannot be opened, there is a problem with it (e.g. file not present
         * or wrong file format). Remove any reference to this file in the queue to prevent
         * forever loop
         */
        app_audio_insert_queue_remove(app_audio_insert_cb.file_index);
        app_cpu_clock_decrease(APP_CPU_CLOCK_AUDIO_INSERT);

        return status;
    }

    /* Set the frequency of target insertion data. */
    status = wiced_bt_voice_prompt_frequency_set((uint16_t) app_audio_insert_sampling_rate_get());

    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_bt_voice_prompt_frequency_set failed: %d\n", status);

        /* Close the Voice Prompt file */
        wiced_bt_voice_prompt_close();

        app_cpu_clock_decrease(APP_CPU_CLOCK_AUDIO_INSERT);

        return status;
    }

    app_cpu_clock_decrease(APP_CPU_CLOCK_AUDIO_INSERT);

    /* Prepare the first-time insertion data. */
    app_audio_insert_source_data_generate(NULL);
    app_audio_insert_source_data_fill();
    app_audio_insert_source_data_generate(NULL);

    return WICED_BT_SUCCESS;
}
/*
 * app_audio_insert_source_data_exhausted_handler
 *
 * Handle the case when the insert data is exhausted under audio insert process.
 */
static void app_audio_insert_source_data_exhausted_handler(void)
{
    app_audio_insert_source_data_fill();
    wiced_app_event_serialize(&app_audio_insert_source_data_generate, NULL);
}

static int app_audio_insert_source_data_generate(void *p_data)
{
    /* Check current state. */
    if ((app_audio_insert_state_get() != APP_AUDIO_INSERT_STATE_STARTING) &&
        (app_audio_insert_state_get() != APP_AUDIO_INSERT_STATE_STARTED))
    {
        return 1;
    }

    app_cpu_clock_increase(APP_CPU_CLOCK_AUDIO_INSERT);

    wiced_bt_voice_prompt_samples_generate();

    app_cpu_clock_decrease(APP_CPU_CLOCK_AUDIO_INSERT);

    return 0;
}

static void app_audio_insert_source_data_fill(void)
{
    uint32_t samples_nb_got;
    wiced_bool_t end_of_file;
    uint16_t samples_nb;
    wiced_bool_t stereo;

    /* Check current state. */
    if ((app_audio_insert_state_get() != APP_AUDIO_INSERT_STATE_STARTING) &&
        (app_audio_insert_state_get() != APP_AUDIO_INSERT_STATE_STARTED))
    {
        return;
    }

    /* Increase CUP speed to reduce insertion data extraction, re-sampling, and filling. */
    app_cpu_clock_increase(APP_CPU_CLOCK_AUDIO_INSERT);

    /* Get current sampling number and identify if mono or stereo shall be used. */
    if (bt_hs_spk_handsfree_audio_manager_stream_check())
    {
        samples_nb = WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_SCO;
        stereo = WICED_FALSE;
    }
    else
    {
        samples_nb = WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_AUDIO * 2;

        if (bt_hs_spk_audio_audio_manager_stream_check())
        {
            stereo = bt_hs_spk_audio_audio_manager_channel_number_get() > 1 ? WICED_TRUE : WICED_FALSE;
        }
        else
        {
            stereo = WICED_TRUE;
        }
    }

    samples_nb_got = wiced_bt_voice_prompt_samples_get(&app_audio_insert_cb.voice_prompt_samples[0],
                                                       samples_nb,
                                                       &end_of_file,
                                                       stereo);


    if ((samples_nb_got == 0) &&
        (end_of_file) &&
        (app_audio_insert_state_get() == APP_AUDIO_INSERT_STATE_STARTED))
    {
        bt_hs_spk_audio_insert_stop();
        app_audio_insert_stop_req(APP_AUDIO_INSERT_STOP_REQ_REGULAR);
    }

    app_cpu_clock_decrease(APP_CPU_CLOCK_AUDIO_INSERT);
}
#endif // VOICE_PROMPT

/*
 * app_audio_insert_config_data_set
 *
 * Set the configuration data used for audio insert
 */
static void app_audio_insert_config_data_set(wiced_bool_t multiple)
{
    wiced_bool_t in_sco;

    in_sco = bt_hs_spk_handsfree_audio_manager_stream_check();

#ifdef VOICE_PROMPT
    app_audio_insert_cb.config.sample_rate                          = app_audio_insert_sampling_rate_get();
    app_audio_insert_cb.config.duration                             = 0;
    app_audio_insert_cb.config.p_source                             = app_audio_insert_cb.voice_prompt_samples;
    app_audio_insert_cb.config.len                                  = in_sco ?
                                                                      WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_SCO * sizeof(int16_t):
                                                                      WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_AUDIO * 2 * sizeof(int16_t);
    app_audio_insert_cb.config.p_source_data_exhausted_callback     = &app_audio_insert_source_data_exhausted_handler;
    app_audio_insert_cb.config.stop_insertion_when_source_exhausted = WICED_FALSE;
    app_audio_insert_cb.config.multiple                             = multiple;
    app_audio_insert_cb.config.insert_data_after_target_seq_num     = multiple;
    app_audio_insert_cb.config.expected_sco_time_seq_num            = app_audio_insert_cb.expected_sco_time_seq_num;
    app_audio_insert_cb.config.p_timeout_callback                   = NULL;
    app_audio_insert_cb.config.stopped_when_state_is_changed        = WICED_FALSE;
#else
    app_audio_insert_cb.config.sample_rate                          = app_audio_insert_sampling_rate_get();
    app_audio_insert_cb.config.duration                             = ((uint32_t) (app_audio_insert_cb.file_index + 1)) * 1000;
    app_audio_insert_cb.config.p_source                             = in_sco ? sine_wave_mono : sine_wave_stereo;
    app_audio_insert_cb.config.len                                  = in_sco ? sizeof(sine_wave_mono) : sizeof(sine_wave_stereo);
    app_audio_insert_cb.config.p_source_data_exhausted_callback     = NULL;
    app_audio_insert_cb.config.stop_insertion_when_source_exhausted = WICED_FALSE;
    app_audio_insert_cb.config.multiple                             = multiple;
    app_audio_insert_cb.config.insert_data_after_target_seq_num     = multiple;
    app_audio_insert_cb.config.expected_sco_time_seq_num            = app_audio_insert_cb.expected_sco_time_seq_num;
    app_audio_insert_cb.config.p_timeout_callback                   = app_audio_insert_timer_callback;
    app_audio_insert_cb.config.stopped_when_state_is_changed        = WICED_FALSE;
#endif
}
