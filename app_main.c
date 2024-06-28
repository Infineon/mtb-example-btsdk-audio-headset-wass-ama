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
 * This file implements the entry point of the Wiced Application
 */
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_l2c.h"
#include "wiced_timer.h"
#include "wiced_memory.h"
#include "sparcommon.h"
#include "wiced_bt_audio.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_pcm.h"
#include "wiced_audio_manager.h"
#include "wiced_transport.h"
#ifdef PRE_INIT_LIB
#include "wiced_pre_init.h"
#endif
#include "hci_control_api.h"
#include "wiced_app_cfg.h"
#include "app_lrac.h"
#include "app_lrac_quality.h"
#include "app_nvram.h"
#include "app_trace.h"
#include "app_a2dp_sink.h"
#include "app_avrc_ct.h"
#include "app_handsfree.h"
#include "app_volume.h"
#ifdef AMA_ENABLED
#include <ama.h>
#include <ama_le.h>
#else
#include "app_ble.h"
#endif
#include "app_hci.h"
#include "app_bt.h"
#include "app_main.h"
#ifdef APP_OFU_SUPPORT
#include "ofu/app_ofu.h"
#endif
#include "wiced_platform.h"
#include "wiced_platform_audio_common.h"
#include "app_cpu_clock.h"
#include "wiced_bt_a2dp_sink_int.h"
#include "wiced_bt_a2dp_sink.h"
#ifdef APP_TPUT_SPP
#include "app_tput_spp.h"
#endif
#ifdef APPLICATION_WATCHDOG_ENABLED
#include "wiced_hal_wdog.h"
#endif
#include "app_audio_insert.h"
#ifdef VOICE_PROMPT
#include "app_voice_prompt.h"
#endif
#ifdef AUTO_ELNA_SWITCH
#include "cycfg_pins.h"
#include "wiced_hal_rfm.h"
#ifndef TX_PU
#define TX_PU   CTX
#endif
#ifndef RX_PU
#define RX_PU   CRX
#endif
#endif

#include "bt_hs_spk_control.h"
#include "bt_hs_spk_handsfree.h"
#include "bt_hs_spk_handsfree_utils.h"
#include "bt_hs_spk_audio.h"
#include "wiced_bt_event.h"

/*
 * Definitions
 */
#define APP_MAIN_PAIRING_TIMER_DURATION     60      /* In Seconds */
#define TRANSPORT_DETECT_SECONDS            3       //unit: second

/* Maximum number of HFP Eavesdropping Retry */
#define APP_MAIN_LRAC_HFP_RETRY_MAX         2

#ifndef VOICE_PROMPT
#define APP_VOICE_PROMPT_INDEX_VOLUME_MAX               0   /* 1 second audio prompt */
#endif

#define APP_MAIN_HFP_EAVESDROPPING_DELAY_TIME 1250  /* In milliseconds */

typedef struct
{
    wiced_bool_t connected;
    wiced_bt_lrac_role_t role;
    wiced_result_t connection_status;
    wiced_timer_t eavesdropping_recover_timer;
} app_main_lrac_cb_t;

typedef struct
{
    /* Regular A2DP Connection (to phone) */
    wiced_bt_a2dp_codec_info_t codec_config;

    /* LRAC A2DP Connection */
    wiced_bool_t lrac_started;

    /* LRAC A2DP Secondary start status (sync start or join late) */
    wiced_bool_t sec_sync_start;
} app_main_a2dp_sink_cb_t;

typedef struct
{
    /* LRAC HFP Connection */
    wiced_bool_t lrac_started;

    uint8_t lrac_start_retry;
} app_main_handsfree_hf_cb_t;

#ifdef APP_OFU_SUPPORT
typedef struct
{
    uint8_t ongoing;
} app_main_ofu_cb_t;
#endif

typedef struct
{
    wiced_bool_t dissoverable;
    wiced_bool_t connectable;
} app_main_bt_visibility_t;

typedef struct
{
    app_main_bt_visibility_t bt_visibility;
    uint16_t remain_discovery_timer;
} app_main_switch_t;

typedef struct
{
    app_main_lrac_cb_t lrac;
    app_main_a2dp_sink_cb_t a2dp_sink;
    app_main_handsfree_hf_cb_t handsfree;
#ifdef APP_OFU_SUPPORT
    app_main_ofu_cb_t ofu;
#endif
    app_main_bt_visibility_t bt_visibility;
    app_main_switch_t lrac_switch;
} app_main_cb_t;

/*
 * Global variables
 */
static app_main_cb_t app_main_cb;

extern wiced_bt_a2dp_config_data_t bt_audio_config;
extern uint8_t app_avrc_ct_supported_events[];

/*
 * External functions
 */
extern void coredump_read(void);
extern wiced_result_t BTM_SetDeviceClass(wiced_bt_dev_class_t dev_class);

/*
 * Local functions
 */
static void app_main_lrac_callback(app_lrac_event_t event, app_lrac_event_data_t *p_data);
static void app_main_a2dp_sink_callback(app_a2dp_sink_event_t event,
        app_a2dp_sink_event_data_t *p_data);
static void app_main_avrc_ct_callback(app_avrc_ct_event_t event, app_avrc_ct_event_data_t *p_data);
static void app_main_handsfree_callback(app_handsfree_event_t event,
        app_handsfree_event_data_t *p_data);
#ifndef AMA_ENABLED
static void app_main_ble_callback(app_ble_event_t event, app_ble_event_data_t *p_data);
#endif
static void app_main_audio_insert_callback(app_audio_insert_event_t event,
        app_audio_insert_event_data_t *p_data);
#ifdef APP_OFU_SUPPORT
static void app_main_ofu_callback(app_ofu_event_t event, app_ofu_event_data_t *p_data);
#endif

static wiced_bool_t app_main_button_callback(platform_button_id_t button_id,
        uint32_t repeat_counter);
static wiced_bool_t app_main_button_primary_handler(platform_button_id_t button_id,
        uint32_t repeat_counter);
static wiced_bool_t app_main_button_secondary_handler(platform_button_id_t button_id,
        uint32_t repeat_counter);
static void app_main_quality_callback(app_lrac_quality_event_t event,
        app_lrac_quality_event_data_t *p_data);
static void app_main_platform_charger_callback(platform_charger_event_t event);

static wiced_result_t app_main_update_dev(void);
static wiced_result_t app_main_update_dev_after_switch (void);
static void app_main_update_dev_dev_name(uint8_t *local_bdaddr);
static wiced_result_t app_main_connect(void);
void app_main_a2dp_stop(void);
static wiced_result_t app_main_codec_route_set(platform_codec_route_t codec_route);

#ifdef APPLICATION_WATCHDOG_ENABLED
static void app_wdog_callback(uint8_t* app_thread_cb, uint32_t app_thread_cb_size,
        uint8_t* app_thread_stack, uint32_t app_thread_stack_size);
#endif

static void app_main_a2dp_streaming_source_device_changed_handler(void);
static void app_main_bt_visibility_change_handler(wiced_bool_t discoverable, wiced_bool_t connectable);
static void app_main_local_volume_change_handler(int32_t am_vol_level, uint8_t am_vol_effect_event);
static void app_main_nvram_link_keys_update_callback(void);

#ifdef VOICE_PROMPT
static int app_main_audio_insert_bt_connected(void *p_data);
static int app_main_audio_insert_ready_to_pair(void *p_data);
static int app_main_audio_insert_volume_max(void *p_data);
#endif // VOICE_PROMPT

static void app_main_lrac_eavesdropping_recover_timer_callback(uint32_t param);

/*
 *  Application Start, i.e., entry point to the application.
 */
APPLICATION_START( )
{
    /* Initialize HCI and PUART */
    app_hci_init();

#ifdef CYW20721B2
    wiced_bt_dev_lrac_disable_secure_connection();
#endif

    WICED_BT_TRACE("******************************************\n");
    WICED_BT_TRACE("*  Headset WASS AMA Application Started  *\n");
    WICED_BT_TRACE("******************************************\n");

    /* Start The Bluetooth Stack */
    app_bt_init();

#ifdef AMA_ENABLED
    ama_init();
#endif
}

/*
 * app_main_post_init
 */
wiced_result_t app_main_post_init(void)
{
    wiced_result_t status = WICED_BT_ERROR;
    wiced_bool_t ret = WICED_FALSE;
    wiced_bt_device_address_t local_bdadr;
#ifdef VOICE_PROMPT
    wiced_bt_voice_prompt_config_t voice_prompt_config;
#endif
    bt_hs_spk_control_config_t config = {0};
    app_nvram_sleep_t sleep_config;

    memset(&app_main_cb, 0, sizeof(app_main_cb));

    /* Check coredump */
    coredump_read();

#ifdef PRE_INIT_LIB
    /*
     * Verify that the Pre Initialization (for RAM optimization) is done.
     * This function call MUST be present to ensure the PreInit library is loaded and executed
     */
    if (wiced_pre_init_is_done() == WICED_FALSE)
        APP_TRACE_ERR("wiced_pre_init_is_done failed\n");
#endif

    config.conn_status_change_cb                            = &app_bt_connection_status_callback;
    config.p_bt_visibility_chg_cb                           = &app_main_bt_visibility_change_handler;
    config.discoverable_timeout                             = APP_MAIN_PAIRING_TIMER_DURATION;
    config.acl3mbpsPacketSupport                            = WICED_FALSE;
    config.audio.a2dp.p_audio_config                        = &bt_audio_config;
    config.audio.a2dp.p_pre_handler                         = &app_a2dp_sink_cback_pre_handler;
    config.audio.a2dp.post_handler                          = app_a2dp_sink_cback;
    config.audio.a2dp.p_streaming_source_chg_cb             = &app_main_a2dp_streaming_source_device_changed_handler;
    config.audio.avrc_ct.p_supported_events                 = app_avrc_ct_supported_events;
    config.audio.avrc_ct.connection_state_cb.p_pre_handler  = &app_avrc_ct_connection_state_callback_pre_handler;
    config.audio.avrc_ct.connection_state_cb.post_handler   = app_avrc_ct_connection_state_callback;
    config.audio.avrc_ct.command_cb.pre_handler             = NULL;
    config.audio.avrc_ct.command_cb.post_handler            = NULL;
    config.audio.avrc_ct.rsp_cb.pre_handler                 = NULL;
    config.audio.avrc_ct.rsp_cb.post_handler                = NULL;
    config.audio.avrc_ct.passthrough_rsp_cb.pre_handler     = NULL;
    config.audio.avrc_ct.passthrough_rsp_cb.post_handler    = NULL;
    config.hfp.rfcomm.buffer_size                           = 100;
    config.hfp.rfcomm.buffer_count                          = 2;
    config.hfp.feature_mask                                 = WICED_HANDSFREE_SUPPORTED_FEATURES;
    config.hfp.p_pre_handler                                = &app_handsfree_event_callback_pre_handler;
    config.hfp.post_handler                                 = app_handsfree_event_callback;

    /* Read the Sleep Config entry in NVRAM */
    status = app_nvram_sleep_get(&sleep_config);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_DBG("PDS Sleep not present in NVRAM\n");
        config.sleep_config.enable = WICED_FALSE;
    }
    else
    {
        switch (sleep_config.sleep_mode)
        {
        case APP_NVRAM_SLEEP_MODE_DISABLED:
            config.sleep_config.enable  = WICED_FALSE;
            break;
        case APP_NVRAM_SLEEP_MODE_TRANSPORT: /* PDS With Transport */
            APP_TRACE_DBG("PDS Sleep Enabled with Transport\n");
            config.sleep_config.enable      = WICED_TRUE;
            config.sleep_config.sleep_mode  = WICED_SLEEP_MODE_TRANSPORT;
            break;

        case APP_NVRAM_SLEEP_MODE_NO_TRANSPORT: /* PDS Without Transport */
            APP_TRACE_DBG("PDS Sleep Enabled without Transport\n");
            config.sleep_config.enable      = WICED_TRUE;
            config.sleep_config.sleep_mode  = WICED_SLEEP_MODE_NO_TRANSPORT;
            break;

        default:
            APP_TRACE_ERR("Unknown Sleep mode:%d\n", sleep_config.sleep_mode);
            config.sleep_config.enable = WICED_FALSE;
            break;
        }
    }

    config.sleep_config.sleep_mode              = WICED_SLEEP_MODE_NO_TRANSPORT;
    config.sleep_config.host_wake_mode          = WICED_SLEEP_WAKE_ACTIVE_HIGH;
    config.sleep_config.device_wake_mode        = WICED_SLEEP_WAKE_ACTIVE_LOW;
    config.sleep_config.device_wake_source      = WICED_SLEEP_WAKE_SOURCE_GPIO;
    config.sleep_config.device_wake_gpio_num    = WICED_P02;

    config.nvram.link_key.id            = NVRAM_ID_LINK_KEYS;
    config.nvram.link_key.p_callback    = &app_main_nvram_link_keys_update_callback;
    config.p_local_vol_chg_cb           = &app_main_local_volume_change_handler;

    if (WICED_SUCCESS != bt_hs_spk_post_stack_init(&config))
    {
        APP_TRACE_ERR("bt_hs_spk_post_stack_init failed\n");
    }

    /*Set audio sink*/
    bt_hs_spk_set_audio_sink(AM_HEADPHONES);

    /* Platform Init */
    platform_init();

    /* Platform Button Init */
    wass_button_init(app_main_button_callback);

    /* platform_charger_init */
    platform_charger_init(app_main_platform_charger_callback);

    wiced_bt_l2cap_set_idle_timeout(0, 0xFFFF, WICED_TRUE);

    /* Configure Audio buffer */
    APP_TRACE_DBG("Allocating Audio buffer CodecSize:%d TxSize:%d\n",
            wiced_bt_audio_buf_config.audio_codec_buffer_size,
            wiced_bt_audio_buf_config.audio_tx_buffer_size);
    status = wiced_audio_buffer_initialize(wiced_bt_audio_buf_config);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_audio_buffer_initialize failed %d\n", status);
        return status;
    }

    /* Read the Local BdAddr from NVRAM and apply it */
    app_nvram_local_bdaddr_read(local_bdadr);
    wiced_bt_set_local_bdaddr(local_bdadr, BLE_ADDR_PUBLIC);
    APP_TRACE_DBG("Local BdAddr:%B\n", local_bdadr);

    /* Initialize A2DP */
    status = app_a2dp_sink_init(app_main_a2dp_sink_callback);
    if(status != WICED_BT_SUCCESS )
    {
        APP_TRACE_DBG("app_a2dp_sink_init failed %d\n", status);
        return status;
    }

    /* Initialize AVRC Controller */
    status = app_avrc_ct_init(app_main_avrc_ct_callback);
    if(status != WICED_BT_SUCCESS )
    {
        APP_TRACE_DBG("app_avrc_ct_init failed %d\n", status);
        return status;
    }

    /* Initialize HandsFree */
    status = app_handsfree_init(app_main_handsfree_callback);
    if(status != WICED_BT_SUCCESS )
    {
        APP_TRACE_DBG("app_handsfree_init failed %d\n", status);
        return status;
    }

#ifdef AMA_ENABLED
    status = ama_post_init(&wiced_bt_cfg_settings);
    if (WICED_BT_SUCCESS != status)
    {
        WICED_BT_TRACE("ERROR ama_post_init %u\n", status);
        return status;
    }
    ama_le_post_init(&wiced_bt_cfg_settings);
#else
    /* Enable LE */
    status = app_ble_init(app_main_ble_callback);
    if(status != WICED_BT_SUCCESS )
    {
        APP_TRACE_DBG("app_le_init failed %d\n", status);
        return status;
    }
#endif

    /* Initialize LRAC */
    status = app_lrac_init(app_main_lrac_callback);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_DBG("app_lrac_init failed %d\n", status);
        return status;
    }

#if defined (OTA_FW_UPGRADE) && (APP_OFU_SUPPORT)
    /* Initialize OFU */
    status = app_ofu_init(app_main_ofu_callback);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_DBG("app_ofu_init failed %d\n", status);
        /* No return on purpose because OFU may not work (e.g. XIP or OCF) */
    }
#endif

    /* Initialize CPU Clock (API used to Switch between 48 MHz and 96 MHz */
    status = app_cpu_clock_init();
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_DBG("app_cpu_clock_init failed %d\n", status);
        return status;
    }

    app_main_cb.lrac.role = app_lrac_config_role_get();
    if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_UNKNOWN)
    {
        APP_TRACE_ERR("Unknow role !!!!!!!\n");
        APP_TRACE_ERR("LRAC Role Unknown. Use the LRAC Config tool to configure the board.\n");
    }
    platform_led_set(PLATFORM_LED_POWER_ON, app_main_cb.lrac.role);

    /* Left/Right Audio side not yet known. Let's suppose we are the Left side */
    /* The actual Audio Side will be configured during LRAC link configuration */
    platform_audio_side_set(PLATFORM_AUDIO_SIDE_LEFT);

    /* Update Device's name */
    app_main_update_dev();

    /* Enable Pairing by default.
     * Note that to set the Primary device in Pairing mode, we will just Enable InquiryScan
     */
    wiced_bt_set_pairable_mode(1, 0 );

    /* Initialize and Configure default volumes */
    app_volume_init();

    /* Enable AFH Channel Assessment. The FW will, implicitly, disable it on Secondary */
    status = wiced_bt_dev_set_afh_channel_assessment(WICED_TRUE);
    if(status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_bt_dev_set_afh_channel_assessment failed %d\n", status);
        return status;
    }

    /* Initialize LRAC quality measurement */
    status = app_lrac_quality_init(app_main_quality_callback);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("app_lrac_quality_init failed status:%d\n", status);
    }

    /* Init LRAC eavesdropping recover timer */
    wiced_init_timer(&app_main_cb.lrac.eavesdropping_recover_timer,
        app_main_lrac_eavesdropping_recover_timer_callback, 0, WICED_MILLI_SECONDS_TIMER);

#ifdef APP_TPUT_SPP
    /* Initialized Throughput SPP */
    status = app_tput_spp_init();
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("app_tput_init failed status:%d\n", status);
    }
#endif

#ifdef APPLICATION_WATCHDOG_ENABLED
    status = wiced_hal_wdog_enable_application_wdog(app_wdog_callback);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_hal_wdog_enable_application_wdog failed status:%d\n", status);
    }
#endif

    /* Initialize Audio Insert */
    status = app_audio_insert_init(app_main_audio_insert_callback);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_DBG("app_audio_insert_init failed %d\n", status);
        return status;
    }

#ifdef VOICE_PROMPT
    /* Get Voice Prompt configuration from NVRAM */
    status = app_nvram_voice_prompt_config_get(&voice_prompt_config);
    if (status == WICED_BT_SUCCESS)
    {
        /* Initialize Voice Prompt Library with the configuration from NVRAM */
        status = wiced_bt_voice_prompt_init(&voice_prompt_config);
        if (status == WICED_BT_SUCCESS)
        {
            /* Play the PowerOn message */
            app_audio_insert_start_req(APP_VOICE_PROMPT_INDEX_POWER_ON);
        }
        else
        {
            APP_TRACE_ERR("wiced_bt_voice_prompt_init failed status:%d\n", status);
        }
    }
    else
    {
        APP_TRACE_ERR("app_nvram_voice_prompt_config_get failed\n");
    }
#endif

#ifdef AUTO_ELNA_SWITCH
    wiced_hal_rfm_auto_elna_enable(1, RX_PU);
#endif
#ifdef AUTO_EPA_SWITCH
    wiced_hal_rfm_auto_epa_enable(1, TX_PU);
#endif

    APP_TRACE_DBG("Platform is Off. Press the Multi-Purpose button to Power it On\n");

    return status;
}

/*
 * app_main_lrac_eavesdropping_state_recover
 *
 * Recover the Secondary devices' eavesdropping state
 */
wiced_bool_t app_main_lrac_eavesdropping_state_recover(void)
{
    wiced_result_t status;

    if (app_main_cb.lrac.role != WICED_BT_LRAC_ROLE_PRIMARY)
    {
        return WICED_FALSE;
    }

    if (wiced_is_timer_in_use(&app_main_cb.lrac.eavesdropping_recover_timer))
    {
        APP_TRACE_DBG("eavesdropping recovery timer was already active\n");
        return WICED_TRUE;
    }

    /* Check SCO state. */
    if (bt_hs_spk_handsfree_sco_connection_check(NULL))
    {   /* SCO is connected. */
        if (app_main_cb.handsfree.lrac_started == WICED_FALSE)
        {   /* Peer device does not start the SCO eavesdropping. */
            /* make sure AP-Link is in active mode because during HFP calling,
             * phone might request to get into sniff mode */
            status = bt_hs_spk_control_bt_power_mode_set(WICED_TRUE, NULL, NULL);
            if (status == WICED_BT_PENDING)
            {
                /* Recovery procedure will be triggered again when AP-Link gets
                 * into active mode */
                APP_TRACE_DBG("Request AP-Link to active mode before recovery\n");
                return TRUE;
            }

            /* Start HFP Eavesdropping on the Secondary */
            status = app_lrac_hfp_start_req();
            if (status != WICED_BT_SUCCESS)
            {
                APP_TRACE_ERR("app_lrac_hfp_start_req failed %d\n", status);

                return WICED_FALSE;
            }
        }

        return WICED_TRUE;
    }
    else
    {
        /* SCO is not connected*/
        if (app_main_cb.handsfree.lrac_started)
        {   /* Peer device is doing the SCO eavesdropping now. */
            /* Ask the Secondary to Stop Voice Eavesdropping */
            status = app_lrac_hfp_stop_req();

            if (status != WICED_BT_SUCCESS)
            {
                APP_TRACE_ERR("app_lrac_voice_stop_req failed %d\n", status);

                return WICED_FALSE;
            }
            else
            {
                return WICED_TRUE;
            }
        }
    }

    /* Check A2DP state. */
    if (bt_hs_spk_audio_is_a2dp_streaming_started())
    {   /* A2DP is started. */
        /* Check if the audio streaming is interrupted. In some cases, the audio streaming will
         * be interrupted by the expected behavior.
         * For example, the audio streaming will be interrupted by an expected HFP audio
         * connection. In this case, the a2dp is paused but not suspended yet. But the resource
         * shall be preserved for the upcoming HFP audio connection (SCO connection).
         * Therefore, we shall NOT try to recover this a2dp eavesdropping. */
        if (bt_hs_spk_audio_is_a2dp_streaming_interrupted(NULL) == WICED_FALSE)
        {
            if (app_main_cb.a2dp_sink.lrac_started == WICED_FALSE)
            {   /* Peer device does not start the stream eavesdropping. */
                /* Start A2DP Eavesdropping on the Secondary */
                status = app_lrac_a2dp_start_req(WICED_FALSE);

                if (status != WICED_BT_SUCCESS)
                {
                    APP_TRACE_ERR("app_lrac_a2dp_start_req failed %d\n", status);

                    return WICED_FALSE;
                }
            }
        }

        return WICED_TRUE;
    }
    else
    {   /* A2DP is not started. */
        if (app_main_cb.a2dp_sink.lrac_started)
        {   // Peer device is doing the stream eavesdropping now.
            /* Ask the secondary to stop A2DP eavesdropping */
            status = app_lrac_a2dp_stop_req();

            if (status != WICED_BT_SUCCESS)
            {
                APP_TRACE_ERR("app_lrac_a2dp_stop_req failed %d\n", status);

                return WICED_FALSE;
            }
            else
            {
                return WICED_TRUE;
            }
        }
    }

    return WICED_FALSE;
}

/*
 * app_main_lrac_eavesdropping_recover_timer_callback
 *
 * If eavesdropping_recover_timer was activated, the callback will be triggered
 */
static void app_main_lrac_eavesdropping_recover_timer_callback(uint32_t param)
{
    /* Start eavesdropping state recovery */
    app_main_lrac_eavesdropping_state_recover();
}

/*
 * app_main_lrac_callback
 */
static void app_main_lrac_callback(app_lrac_event_t event, app_lrac_event_data_t *p_data)
{
    wiced_result_t status;
    wiced_bt_device_address_t bdaddr;

    switch(event)
    {
    case APP_LRAC_CONNECTED:
        APP_TRACE_DBG("Connected bdaddr:%B status:%d LracRole:%s(%d) AudioSide:%s(%d)\n",
                p_data->connected.bdaddr, p_data->connected.status,
                app_lrac_role_get_desc(p_data->connected.lrac_role),
                p_data->connected.lrac_role,
                app_lrac_audio_side_get_desc(p_data->connected.audio_side),
                p_data->connected.audio_side);
        app_main_cb.lrac.connection_status = p_data->connected.status;
        if (p_data->connected.status == WICED_BT_SUCCESS)
        {
            app_main_cb.lrac.connected = WICED_TRUE;

            platform_led_set(PLATFORM_LED_LRAC_CONNECTION, 1);

            /* If the Role changed */
            if (app_main_cb.lrac.role != p_data->connected.lrac_role)
            {
                app_main_cb.lrac.role = p_data->connected.lrac_role;
                app_main_update_dev();
            }
            app_main_cb.lrac.role = p_data->connected.lrac_role;

            /* Configure the Audio codec to play either the Left or the right side */
            if (p_data->connected.audio_side == WICED_BT_LRAC_AUDIO_SIDE_LEFT)
            {
                platform_audio_side_set(PLATFORM_AUDIO_SIDE_LEFT);
            }
            else if (p_data->connected.audio_side == WICED_BT_LRAC_AUDIO_SIDE_RIGHT)
            {
                platform_audio_side_set(PLATFORM_AUDIO_SIDE_RIGHT);
            }
            else
            {
                APP_TRACE_ERR("Wrong Audio Side:%s(%d)\n",
                        app_lrac_audio_side_get_desc(p_data->connected.audio_side),
                        p_data->connected.audio_side);
            }

            if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
            {
                if (app_main_lrac_eavesdropping_state_recover() == WICED_FALSE)
                {
                    if ((bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED) &&
                        (bt_hs_spk_handsfree_sco_connection_check(NULL) == WICED_FALSE))
                    {
                        APP_TRACE_DBG("Neither HFP Call nor A2DP Started. Nothing to do.\n");

#ifdef VOICE_PROMPT
                        app_audio_insert_start_req(APP_VOICE_PROMPT_INDEX_STEREO_CONNECTED);
#endif
                    }
                }

                /* Set Bluetooth visibility. */
                if (bt_hs_spk_control_connection_status_check_be_edr(WICED_TRUE) == WICED_TRUE)
                {
                    bt_hs_spk_control_handle_set_visibility(app_main_cb.bt_visibility.dissoverable,
                                                            WICED_FALSE,
                                                            BT_TRANSPORT_BR_EDR);
                }
            }
            else
            {
                platform_state_set(GLOBAL_IDLE_STATE);

                /* Set Bluetooth visibility. */
                bt_hs_spk_control_handle_set_visibility(WICED_FALSE, WICED_FALSE, BT_TRANSPORT_BR_EDR);
            }
        }

        /* Check if something must be connected */
        app_main_connect();
        break;

    case APP_LRAC_DISCONNECTED:
        APP_TRACE_DBG("Disconnected\n");
        app_main_cb.lrac.connected = WICED_FALSE;
        app_main_cb.a2dp_sink.lrac_started = WICED_FALSE;

        /* Resume the A2DP Start Ind. request if it is pending. */
        app_a2dp_sink_start_req_pending_resume();

        app_main_cb.handsfree.lrac_started = WICED_FALSE;
        app_main_cb.handsfree.lrac_start_retry = 0;

        /* Resume the SCO Connection Request if it is pending. */
        app_handsfree_sco_start_req_pending_resume();

        bt_hs_spk_control_handle_set_visibility(app_main_cb.bt_visibility.dissoverable,
                                                WICED_TRUE,
                                                BT_TRANSPORT_BR_EDR);

        platform_led_set(PLATFORM_LED_LRAC_CONNECTION, 0);

        if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_SECONDARY)
            platform_state_set(GLOBAL_POWER_UP_STATE);

#ifdef VOICE_PROMPT
        app_audio_insert_start_req(APP_VOICE_PROMPT_INDEX_STEREO_DISCONNECTED);
#endif
        break;

    case APP_LRAC_A2DP_STARTED:
        APP_TRACE_DBG("A2DP Started status:%d sync_start:%d\n", p_data->a2dp_started.status, p_data->a2dp_started.sync);
        if (p_data->a2dp_started.status == WICED_BT_SUCCESS)
        {
            app_main_cb.a2dp_sink.lrac_started = WICED_TRUE;
            app_main_cb.a2dp_sink.sec_sync_start = p_data->a2dp_started.sync;
            if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_SECONDARY)
            {
                /* Save Codec information */
                memcpy(&app_main_cb.a2dp_sink.codec_config, &p_data->a2dp_started.codec_info,
                        sizeof(app_main_cb.a2dp_sink.codec_config));

                platform_led_set(PLATFORM_LED_A2DP_STREAM_STATE, 1);
            }
            else if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
            {
                app_main_lrac_eavesdropping_state_recover();
            }

            app_lrac_quality_timer_start();
        }
        else
        {
            app_main_cb.a2dp_sink.lrac_started = WICED_FALSE;
            if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_SECONDARY)
            {
                platform_led_set(PLATFORM_LED_A2DP_STREAM_STATE, 0);
            }
        }
        /* Check if Audio Insert must be resumed */
        app_audio_insert_resume();
        break;

    case APP_LRAC_A2DP_STOPPED:
        APP_TRACE_DBG("A2DP Stopped status:%d\n", p_data->a2dp_stopped.status);
        app_main_cb.a2dp_sink.lrac_started = WICED_FALSE;
        if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            app_main_lrac_eavesdropping_state_recover();

            if (app_audio_insert_is_started())
            {
                status = app_audio_insert_stop_req(APP_AUDIO_INSERT_STOP_REQ_SUSPEND);
                if (status != WICED_BT_SUCCESS)
                {
                    APP_TRACE_ERR("app_audio_insert_stop_req failed\n");
                }
            }

            if (bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED)
            {
                app_lrac_quality_timer_stop();
            }
        }
        else if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_SECONDARY)
        {
            app_main_codec_route_set(PLATFORM_CODEC_ROUTE_NONE);
            platform_led_set(PLATFORM_LED_A2DP_STREAM_STATE, 0);
            app_lrac_quality_timer_stop();
        }
        /* Check if Audio Insert must be resumed */
        app_audio_insert_resume();
        break;

    case APP_LRAC_HFP_STARTED:
        APP_TRACE_DBG("HFP Started status:%d wide_band:%d\n",
                p_data->hfp_started.status, p_data->hfp_started.wide_band);

#ifdef VOLUME_EFFECT
        if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            app_volume_set(APP_VOLUME_SELECT_VOICE,
                    bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(bt_hs_spk_handsfree_volume_get()),
                    VOLUME_EFFECT_RAMP_UP);
        }
#endif

        if (p_data->hfp_started.status == WICED_BT_SUCCESS)
        {
            app_main_cb.handsfree.lrac_started = WICED_TRUE;
            app_main_cb.handsfree.lrac_start_retry = 0;
            if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_SECONDARY)
            {
                /* Configuring Audio route depending on HFP Voice configuration */
                if (p_data->hfp_started.wide_band)
                    app_main_codec_route_set(PLATFORM_CODEC_ROUTE_MSBC);
                else
                    app_main_codec_route_set(PLATFORM_CODEC_ROUTE_CVSD);
                platform_led_set(PLATFORM_LED_HFP_VOICE_STATE, 1);
            }
            else if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
            {
                app_main_lrac_eavesdropping_state_recover();
            }

            app_lrac_quality_timer_start();
        }
        else
        {
            app_main_cb.handsfree.lrac_started = WICED_FALSE;

            if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
            {
                /* Update HFP/LRAC Start Retry counter */
                app_main_cb.handsfree.lrac_start_retry++;

                /* In case of error, try again */
                if (app_main_cb.handsfree.lrac_start_retry < APP_MAIN_LRAC_HFP_RETRY_MAX)
                {
                    app_main_lrac_eavesdropping_state_recover();
                }
            }
            else if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_SECONDARY)
            {
                app_main_codec_route_set(PLATFORM_CODEC_ROUTE_NONE);
                platform_led_set(PLATFORM_LED_HFP_VOICE_STATE, 0);
            }
        }
        /* Check if Audio Insert must be resumed */
        app_audio_insert_resume();
        break;

    case APP_LRAC_HFP_STOPPED:
        APP_TRACE_DBG("HFP Stopped status:%d\n", p_data->hfp_stopped.status);
        app_main_cb.handsfree.lrac_started = WICED_FALSE;
        app_main_cb.handsfree.lrac_start_retry = 0;
        if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            if (app_audio_insert_is_started())
            {
                status = app_audio_insert_stop_req(APP_AUDIO_INSERT_STOP_REQ_SUSPEND);
                if (status != WICED_BT_SUCCESS)
                {
                    APP_TRACE_ERR("app_audio_insert_stop_req failed\n");
                }
            }
            else
            {
                app_main_lrac_eavesdropping_state_recover();
            }
        }
        else if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_SECONDARY)
        {
            app_main_codec_route_set(PLATFORM_CODEC_ROUTE_NONE);
            platform_led_set(PLATFORM_LED_HFP_VOICE_STATE, 0);
        }
        app_lrac_quality_timer_stop();

        /* Check if Audio Insert must be resumed */
        app_audio_insert_resume();
        break;

    case APP_LRAC_BUTTON:
        if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            /* User pressed a button on Secondary. Handle it on Primary */
            platform_button_emulator(p_data->button.button_id, p_data->button.repeat_counter);
        }
        else
        {
            APP_TRACE_ERR("Unexpected APP_LRAC_BUTTON");
        }
        break;

    case APP_LRAC_VOLUME:
        if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_SECONDARY)
        {
            app_volume_set(app_main_cb.handsfree.lrac_started ? APP_VOLUME_SELECT_VOICE : APP_VOLUME_SELECT_STREAM,
                           p_data->volume.am_vol_level, p_data->volume.am_vol_effect);
        }
        break;

    case APP_LRAC_SWITCH_COMPLETED:
        if (p_data->switch_completed.status == WICED_BT_SUCCESS)
        {
            WICED_BT_TRACE("SWITCH_COMPLETED Success new_role:%s(%d)\n",
                    app_lrac_role_get_desc(p_data->switch_completed.new_role),
                    p_data->switch_completed.new_role);

            /* Update device after LRAC Switch */
            app_main_update_dev_after_switch();

            if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
            {
                app_main_lrac_eavesdropping_state_recover();
            }
        }
        else
        {
            APP_TRACE_ERR("SWITCH_COMPLETED status:%d new_role:%s(%d) fatal_error:%d\n",
                    p_data->switch_completed.status,
                    app_lrac_role_get_desc(p_data->switch_completed.new_role),
                    p_data->switch_completed.new_role,
                    p_data->switch_completed.fatal_error);
            if (p_data->switch_completed.fatal_error)
            {
                APP_TRACE_ERR("SWITCH FATAL ERROR. REBOOT HIGHLY RECOMMENDED\n");
                /* Generate a watchdog Reset */
                wdog_generate_hw_reset();
            }
            /* PS-SWITCH is force aborted. To do eavesdropping recovery procedure */
            if (p_data->switch_completed.status == WICED_UNFINISHED)
            {
                if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
                {
                    app_main_lrac_eavesdropping_state_recover();
                }
            }
        }

        /* Enable GFPS provider module. */
        if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
#ifdef AMA_ENABLED
            ama_resume();
#endif
        }
        break;

    default:
        APP_TRACE_ERR("Unknown event:%d\n", event);
        break;
    }
}

/*
 * app_main_a2dp_sink_callback
 */
static void app_main_a2dp_sink_callback(app_a2dp_sink_event_t event,
        app_a2dp_sink_event_data_t *p_data)
{
    wiced_result_t status;

    switch(event)
    {
    case APP_A2DP_SINK_EVT_STREAM_CONNECTED:
        if (p_data->connected.status == WICED_BT_SUCCESS)
        {
            app_a2dp_sink_start_req_pending_reset(p_data->connected.handle);

            platform_led_set(PLATFORM_LED_A2DP_CONNECTION, 1);
            platform_state_set(GLOBAL_IDLE_STATE);

            /* Be sure the A2DP Source is Central */
            if (BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS == 1)
            {
                bt_hs_spk_control_bt_role_set(p_data->connected.bdaddr, HCI_ROLE_PERIPHERAL);
            }

            /* Keep track of Connected Profiles */
            if (app_lrac_phone_profile_connection_up(p_data->connected.bdaddr, APP_NVRAM_PROFILES_A2DP_SINK_MASK))
            {
                /* Play Voice Prompt message if needed */
#ifdef VOICE_PROMPT
                wiced_app_event_serialize(&app_main_audio_insert_bt_connected, NULL);
#endif
            }
        }
        /* Check it another profile must be connected */
        app_main_connect();
        break;

    case APP_A2DP_SINK_EVT_STREAM_DISCONNECTED:
        app_a2dp_sink_start_req_pending_reset(p_data->disconnected.handle);

        if ((app_main_cb.lrac.connected) &&
            (app_main_cb.a2dp_sink.lrac_started))
        {
            if (bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED)
            {
                app_lrac_a2dp_stop_req();
            }
        }

        if ((bt_hs_spk_handsfree_hfp_connection_check(NULL, WICED_FALSE) == WICED_FALSE) &&
            (bt_hs_spk_audio_connection_check(NULL) == WICED_FALSE))
        {
            platform_state_set(GLOBAL_POWER_UP_STATE);
        }
        else
        {
            APP_TRACE_DBG("HFP or AVRC is connected.\n");
        }

        platform_led_set(PLATFORM_LED_A2DP_CONNECTION, 0);
        break;

    case APP_A2DP_SINK_EVT_STREAM_START_REQ:
        /* Phone request A2DP Start */
        /* Since this event is triggered by the A2DP sink event post-handler,
         * we have to check if the AVDTP START IND has been accepted by the bt_hs_spk_lib.
         * Otherwise, the existent A2DP eavesdropping will be stopped by this event. */
        if (bt_hs_spk_audio_streaming_check(p_data->start_req.bdaddr) == WICED_ALREADY_CONNECTED)
        {
            status = app_a2dp_sink_start_rsp();
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_a2dp_sink_start_rsp failed\n");
        }
        break;

    case APP_A2DP_SINK_EVT_STREAM_STARTED:
        platform_led_set(PLATFORM_LED_A2DP_STREAM_STATE, 1);
        /* Tell the Phone which volume we currently use */
        bt_hs_spk_audio_volume_sync();

        /* If the LRAC link is connected */
#if 1 // todo
        /*
         * todo. wait for lrac_lib to support the eavesdropping replacement (BTSDK-3041)
         *       After eavesdropping replacement is accomplished, the primary device can
         *       ask secondary to do a2dp eavesdropping without check current eavesdropping
         *       state.
         */
        if (app_main_cb.lrac.connected)
        {
            if (app_main_cb.a2dp_sink.lrac_started)
            {
                /* Stop the eavesdropping first.
                 * The eavesdropping recovery mechanism will start the a2dp
                 * eavesdropping after receiving the LRAC_A2DP_STOP_RSP event.*/
                app_lrac_a2dp_stop_req();
            }
            else
            {
                /* Ask the Secondary to start A2DP Eavesdropping */
                status = app_lrac_a2dp_start_req(WICED_TRUE);
                if (status != WICED_BT_SUCCESS)
                {
                    APP_TRACE_ERR("app_lrac_a2dp_start_req failed %d\n", status);
                }
            }
        }
#else
        if ((app_main_cb.lrac.connected) &&
            (app_main_cb.a2dp_sink.lrac_started == WICED_FALSE))
        {
            /* Ask the Secondary to start A2DP Eavesdropping */
            status = app_lrac_a2dp_start_req(WICED_TRUE);
            if (status != WICED_BT_SUCCESS)
            {
                APP_TRACE_ERR("app_lrac_a2dp_start_req failed %d\n", status);
            }
        }
        else
        {
            /* TODO: We may try to connect the Secondary here... */
        }
#endif
        app_lrac_quality_timer_start();
        break;

    case APP_A2DP_SINK_EVT_STREAM_STOPPED:
        /* If Audio Insertion ongoing, stop it */
        if (app_audio_insert_is_started())
        {
            APP_TRACE_DBG("Stop Audio Insert\n");
            status = app_audio_insert_stop_req(APP_AUDIO_INSERT_STOP_REQ_SUSPEND);
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_audio_insert_stop_req failed\n");
        }

        /* Stop eavesdropping. */
        app_main_a2dp_stop();

        app_lrac_quality_timer_stop();
        break;

    default:
        APP_TRACE_ERR("Unknown event:%d\n", event);
        break;
    }
}

/*
 * app_main_avrc_ct_callback
 */
void app_main_avrc_ct_callback (app_avrc_ct_event_t event, app_avrc_ct_event_data_t *p_data)
{
    wiced_result_t status;
    uint8_t volume;

    switch(event)
    {
    case APP_AVRC_CT_EVT_CONNECTED:
        if (p_data->connected.status == WICED_BT_SUCCESS)
        {
            platform_led_set(PLATFORM_LED_AVRC_CONNECTION, 1);
            platform_state_set(GLOBAL_IDLE_STATE);

            /* Keep track of Connected Profiles */
            if (app_lrac_phone_profile_connection_up(p_data->connected.bdaddr, APP_NVRAM_PROFILES_AVRC_CT_MASK))
            {
#ifdef VOICE_PROMPT
                wiced_app_event_serialize(&app_main_audio_insert_bt_connected, NULL);
#endif
            }
        }

        /* Check it another profile must be connected */
        app_main_connect();
        break;

    case APP_AVRC_CT_EVT_DISCONNECTED:
        platform_led_set(PLATFORM_LED_AVRC_CONNECTION, 0);

        if ((bt_hs_spk_handsfree_hfp_connection_check(NULL, WICED_FALSE) == WICED_FALSE) &&
            (bt_hs_spk_audio_connection_check(NULL) == WICED_FALSE))
        {
            platform_state_set(GLOBAL_POWER_UP_STATE);
        }
        else
        {
            APP_TRACE_DBG("HFP or A2DP is connected.\n");
        }

        break;

    default:
        break;
    }
}

/*
 * app_main_handsfree_callback
 */
static void app_main_handsfree_callback(app_handsfree_event_t event,
        app_handsfree_event_data_t *p_data)
{
    wiced_result_t status;
    wiced_bt_device_address_t bdaddr;

    switch(event)
    {
    case APP_HANDSFREE_EVENT_CONNECTED:
        if (p_data->connected.status == WICED_BT_SUCCESS)
        {
            app_handsfree_sco_start_req_pending_reset(0, p_data->connected.bdaddr);

            /* Send the Device's Battery level to the Phone (for test) */
            bt_hs_spk_handsfree_battery_level_tx(p_data->connected.bdaddr, 80);

            platform_state_set(GLOBAL_IDLE_STATE);

            /* Be sure the Audio Gateway is Central */
            if (BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS == 1)
            {
                bt_hs_spk_control_bt_role_set(p_data->connected.bdaddr, HCI_ROLE_PERIPHERAL);
            }

            /* For some reasons, for example, the user disconnect and connect the Bluetooth connection via
             * phone Bluetooth menu, the SCO/eSCO connection will be established before to the HFP Service
             * Level Connection process is finished. We need to handle this exception although the
             * HFP spec. requires the Service Level Connection to be established before Audio
             * Connection.
             * Otherwise, the secondary cannot have the eavesdropped voice stream. */
            if (bt_hs_spk_handsfree_sco_connection_check(p_data->connected.bdaddr))
            {
                if (app_main_cb.handsfree.lrac_started == WICED_FALSE)
                {
                    /* Ask the Secondary to start Voice Eavesdropping */
                    status = app_lrac_hfp_start_req();
                    if (status != WICED_BT_SUCCESS)
                        APP_TRACE_ERR("app_lrac_hfp_start_req failed %d\n", status);
                }
                else
                {
                    APP_TRACE_DBG("SCO already eavesdropping\n");
                }
            }

            /* Keep track of Connected Profiles */
            if (app_lrac_phone_profile_connection_up(p_data->connected.bdaddr, APP_NVRAM_PROFILES_HFP_HS_MASK))
            {
#ifdef VOICE_PROMPT
                wiced_app_event_serialize(&app_main_audio_insert_bt_connected, NULL);
#endif
            }
        }
        else
        {
            APP_TRACE_DBG("HFP connection failed\n");
            platform_led_set(PLATFORM_LED_HFP_CONNECTION, 0);
        }
        /* Check it another profile must be connected */
        app_main_connect();

        break;

    case APP_HANDSFREE_EVENT_DISCONNECTED:
        memset(&app_main_cb.handsfree, 0, sizeof(app_main_cb.handsfree));

        if ((bt_hs_spk_handsfree_hfp_connection_check(NULL, WICED_FALSE) == WICED_FALSE) &&
            (bt_hs_spk_audio_connection_check(NULL) == WICED_FALSE))
        {
            platform_state_set(GLOBAL_POWER_UP_STATE);
        }
        else
        {
            APP_TRACE_DBG("avrc_ct or A2DP is connected.\n");
        }
        break;

    case APP_HANDSFREE_EVENT_AUDIO_CONNECT_REQ:
        /*
         * If the device was really implementing Noise Reduction and Echo Cancellation we
         * should send an AT command to the Phone to disable it.
         * But, as this demo app does not implement it, we don't send it (for better audio
         * quality).
         */
        bt_hs_spk_handsfree_ag_nrec_disable(p_data->audio_connect_req.bdaddr);
        break;

    case APP_HANDSFREE_EVENT_AUDIO_CONNECTED:
        /* If the LRAC link is connected. */
        if ((app_main_cb.lrac.connected) &&
            (app_main_cb.handsfree.lrac_started == WICED_FALSE))
        {
            /* Ready to ask Secondary to start HFP eavesdropping.
             * Some Phones(Pixel series) will enter SNIFF mode even they want to start HFP,
             * and BTM_POWER_MANAGEMENT_STATUS_EVT with SNIFF mode may be handled later than
             * we want to start HFP eavesdropping. Delay the starting time for HFP eavesdropping
             * to avoid starting HFP eavesdropping during AP-link is in SNIFF mode.
             */
            APP_TRACE_DBG("Ready to do app_lrac_hfp_start_req\n");
            wiced_start_timer(&app_main_cb.lrac.eavesdropping_recover_timer,
                APP_MAIN_HFP_EAVESDROPPING_DELAY_TIME);
        }
        else
        {
            /* We may try to connect the Secondary here... */

            /* Check if Audio Insert must be resumed */
            app_audio_insert_resume();
        }
        break;

    case APP_HANDSFREE_EVENT_AUDIO_DISCONNECTED:
        app_handsfree_sco_start_req_pending_reset(p_data->audio_disconnected.sco_index, NULL);

        if ((bt_hs_spk_handsfree_hfp_connection_check(NULL, WICED_FALSE) == WICED_FALSE) &&
            (bt_hs_spk_audio_connection_check(NULL) == WICED_FALSE))
        {
            /* Platform shall be set to POWER UP STATE only when all the three profile (HFP,
             * A2DP, and AVRCP) are disconnected. */
            platform_state_set(GLOBAL_POWER_UP_STATE);
        }
        else
        {
            APP_TRACE_DBG("AVRC or A2DP is connected.\n");
        }

         /* If the LRAC link is connected */
         if (app_main_cb.lrac.connected)
         {
             /* Ask the Secondary to Stop Voice Eavesdropping */
             status = app_lrac_hfp_stop_req();
             if (status != WICED_BT_SUCCESS)
                 APP_TRACE_ERR("app_lrac_voice_stop_req failed %d\n", status);
         }
         else
         {
             /* Check if Audio Insert must be resumed */
             app_audio_insert_resume();
         }
         break;

    default:
        APP_TRACE_ERR("Unknown event:%d\n", event);
        break;
    }
}

#ifndef AMA_ENABLED
/*
 * app_main_le_callback
 */
static void app_main_ble_callback(app_ble_event_t event, app_ble_event_data_t *p_data)
{
    switch(event)
    {
    case APP_BLE_EVENT_ADV_CHANGED:
        APP_TRACE_DBG("BLE_ADV_CHANGED mode:%d\n", p_data->adv_changed.mode);
        break;

    case APP_BLE_EVENT_GATT_CONNECTED:
        APP_TRACE_DBG("BLE_GATT_CONNECTED BdAddr:%B ConnId:%d Status:%d\n",
                p_data->gatt_connected.bdaddr, p_data->gatt_connected.conn_id,
                p_data->gatt_connected.status);
        /*
         * LE Advertisement is automatically stopped during LE Connection.
         * We don't have to, explicitly, stop LE Advertisement
         */
        break;

    default:
        APP_TRACE_ERR("Unknown event:%d\n", event);
        break;
    }
}
#endif

/*
 * app_main_audio_insert_callback
 * This function handler Audio Insert Events
 */
static void app_main_audio_insert_callback(app_audio_insert_event_t event,
        app_audio_insert_event_data_t *p_data)
{
    wiced_result_t status;

    switch(event)
    {
    case APP_AUDIO_INSERT_STOPPED:
        APP_TRACE_DBG("APP_AUDIO_INSERT_STOPPED status:%d\n", p_data->stopped.status);
        /* Check if the Codec Route is already configured (either Streaming or SCO) */
        if ((bt_hs_spk_audio_streaming_check(NULL) == WICED_ALREADY_CONNECTED) ||
            (app_main_cb.a2dp_sink.lrac_started) ||
            (bt_hs_spk_handsfree_sco_connection_check(NULL)) ||
            (app_main_cb.handsfree.lrac_started))
        {
            APP_TRACE_DBG("AudioRoute already in use. Do not stop Stop it\n");
        }
        else
        {
            /* If a dedicated Audio Manager Stream (to control external codec) was used, close it */
            bt_hs_spk_audio_audio_manager_stream_stop();

            /* If Audio Insert stopped because A2DP Start was pending */
            /* Resume the A2DP Start Ind. request if it is pending. */
            app_a2dp_sink_start_req_pending_resume();

            /* If Audio Insert stopped because Audio (SCO) Request was pending */
            /* Resume the SCO Connection Request if it is pending. */
            app_handsfree_sco_start_req_pending_resume();
        }
        break;

    default:
        APP_TRACE_ERR("Unknown AudioInsert event:%d\n", event);
        break;
    }
}

#ifdef APP_OFU_SUPPORT
/*
 *
 */
static char *app_main_ofu_transport_get_desc(app_ofu_transport_t ofu_transport)
{
    switch(ofu_transport)
    {
    case APP_OFU_TRANSPORT_SPP_SERVER: return "SPP Server";
    case APP_OFU_TRANSPORT_BLE_SERVER: return "LE Server";
    case APP_OFU_TRANSPORT_LRAC_SERVER: return "LRAC Server";
    case APP_OFU_TRANSPORT_LRAC_CLIENT: return "LRAC Client";
    }
    return "Unknown";
};

/*
 * app_main_ofu_callback
 */
static void app_main_ofu_callback(app_ofu_event_t event, app_ofu_event_data_t *p_data)
{
    switch(event)
    {
    case APP_OFU_EVENT_STARTED:
        APP_TRACE_DBG("OFU Started transport:%s(%d)\n",
                app_main_ofu_transport_get_desc(p_data->started.transport),
                p_data->started.transport);
        if (app_main_cb.ofu.ongoing)
            APP_TRACE_ERR("OFU already ongoing\n");
        app_main_cb.ofu.ongoing = WICED_TRUE;
        break;

    case APP_OFU_EVENT_COMPLETED:
        APP_TRACE_DBG("OFU Completed transport:%s(%d)\n",
                app_main_ofu_transport_get_desc(p_data->completed.transport),
                p_data->completed.transport);
        if (app_main_cb.ofu.ongoing == WICED_FALSE)
            APP_TRACE_ERR("OFU was not ongoing\n");
        app_main_cb.ofu.ongoing = WICED_FALSE;
        break;

    case APP_OFU_EVENT_ABORTED:
        APP_TRACE_DBG("OFU Aborted transport:%s(%d)\n",
                app_main_ofu_transport_get_desc(p_data->aborted.transport),
                p_data->aborted.transport);
        if (app_main_cb.ofu.ongoing == WICED_FALSE)
            APP_TRACE_ERR("OFU was not ongoing\n");
        app_main_cb.ofu.ongoing = WICED_FALSE;
        break;

    default:
        APP_TRACE_ERR("Unknown event:%d\n", event);
        break;
    }
}
#endif

/*
 * app_main_button_secondary_handler
 */
static wiced_bool_t app_main_button_secondary_handler(platform_button_id_t button_id,uint32_t repeat_counter)
{
    wiced_result_t status;

    switch(platform_state_get())
    {
    case GLOBAL_POWER_UP_STATE:
        switch(button_id)
        {
        case PLATFORM_BUTTON_LRAC_BUTTON_1_SHORT:
            APP_TRACE_DBG("Power On platform\n");
            platform_led_set(PLATFORM_LED_POWER_ON, app_main_cb.lrac.role);

            /* If LRAC Link not yet established, try to connect to Primary */
            if (app_main_cb.lrac.connected == WICED_FALSE)
            {
                bt_hs_spk_control_handle_set_visibility(WICED_FALSE, WICED_TRUE, BT_TRANSPORT_BR_EDR);

                app_main_cb.lrac.connection_status = WICED_BT_SUCCESS;
                app_main_connect();
            }
            break;
        case PLATFORM_BUTTON_POWER_OFF:
            platform_led_set(PLATFORM_LED_POWER_OFF, 0);
            break;
        default: /* Already handled above */
            break;
        }
        break;
    case GLOBAL_IDLE_STATE:
        /* If LRAC Link established, try to send button event to Primary */
        if (app_main_cb.lrac.connected)
        {
            /* The Button Pressed information must be sent to the Primary */
            status = app_lrac_button_send(button_id,repeat_counter);

            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_lrac_button_send failed status:%d\n", status);
        }
        break;
    case GLOBAL_POWER_DOWN_STATE:
        break;
    default:
        APP_TRACE_ERR("Bad State:%d\n",platform_state_get());
        break;
    }

    return WICED_TRUE;
}

/*
 * app_main_button_primary_handler
 */
static wiced_bool_t app_main_button_primary_handler(platform_button_id_t button_id,uint32_t repeat_counter)
{
    wiced_bool_t ret = WICED_FALSE;

    /*
     * On Primary device, perform action to Phone.
     */
    switch(button_id)
    {
    case PLATFORM_BUTTON_POWER_ON:
        /* Upon PowerOn try to connect to the Secondary LRAC device and to phone */
        platform_led_set(PLATFORM_LED_POWER_ON, app_main_cb.lrac.role);

        bt_hs_spk_control_handle_set_visibility(app_main_cb.bt_visibility.dissoverable,
                                                WICED_TRUE,
                                                BT_TRANSPORT_BR_EDR);

        app_main_cb.lrac.connection_status = WICED_BT_SUCCESS;
        app_main_connect();
        return WICED_FALSE;
        //return WICED_TRUE;

    case PLATFORM_BUTTON_POWER_OFF:
        APP_TRACE_DBG("TODO: Check if PowerOff must be sent to Secondary\n");
        bt_hs_spk_control_disconnect(NULL);
        platform_led_set(PLATFORM_LED_POWER_OFF, 0);
        return WICED_TRUE;

    /* Common Buttons */
    case PLATFORM_BUTTON_PAIRING:
        APP_TRACE_DBG("Starting Pairing Timer\n");

        platform_led_set(PLATFORM_LED_PAIRING, 1);
#ifdef VOICE_PROMPT
        wiced_app_event_serialize(&app_main_audio_insert_ready_to_pair, NULL);
#endif
        return WICED_FALSE;

    case PLATFORM_BUTTON_VOLUME_UP:
        /* Check if call session exists. */
        if (bt_hs_spk_handsfree_call_session_check())
        {
            if (bt_hs_spk_handsfree_volume_get() == WICED_HANDSFREE_VOLUME_MAX)
            {
                /* Already maximum volume */
                /* Prompt audio to indicate the volume is already at maximum */
#ifdef VOICE_PROMPT
                wiced_app_event_serialize(&app_main_audio_insert_volume_max, NULL);
#else
                app_audio_insert_start_req(APP_VOICE_PROMPT_INDEX_VOLUME_MAX);
#endif
            }
            else
            {
                if (repeat_counter > 0)
                {
                    platform_button_emulator(PLATFORM_BUTTON_LRAC_BUTTON_2_SHORT, 0);

                    return WICED_TRUE;
                }
            }

            return WICED_FALSE;
        }

        /* Check if the audio streaming exists.  */
        if (bt_hs_spk_audio_streaming_check(NULL) == WICED_ALREADY_CONNECTED)
        {
            if (bt_hs_spk_audio_volume_get() == BT_HS_SPK_AUDIO_VOLUME_MAX)
            {
                /* Already maximum volume */
                /* Prompt audio to indicate the volume is already at maximum */
#ifdef VOICE_PROMPT
                wiced_app_event_serialize(&app_main_audio_insert_volume_max, NULL);
#else
                app_audio_insert_start_req(APP_VOICE_PROMPT_INDEX_VOLUME_MAX);
#endif
            }
            else
            {
                if (repeat_counter > 0)
                {
                    platform_button_emulator(PLATFORM_BUTTON_LRAC_BUTTON_2_SHORT, 0);

                    return WICED_TRUE;
                }
            }

            return WICED_FALSE;
        }
        break;

    case PLATFORM_BUTTON_VOLUME_DOWN:
        if (repeat_counter > 0)
        {
            platform_button_emulator(PLATFORM_BUTTON_LRAC_BUTTON_3_SHORT, 0);

            return WICED_TRUE;
        }
        break;

    case PLATFORM_BUTTON_LRAC_BUTTON_1_SHORT:
    case PLATFORM_BUTTON_LRAC_BUTTON_1_LONG:
    case PLATFORM_BUTTON_LRAC_BUTTON_1_REPEAT:
        APP_TRACE_DBG("LRAC Button id:%d\n",button_id);
        /* The button was neither handled by HFP nor AVRC, check the global state */
        switch(platform_state_get())
        {
        case GLOBAL_POWER_UP_STATE:
            if (button_id == PLATFORM_BUTTON_LRAC_BUTTON_1_SHORT)
            {
                return app_main_button_primary_handler(PLATFORM_BUTTON_POWER_ON,repeat_counter);
            }
            else if (button_id == PLATFORM_BUTTON_LRAC_BUTTON_1_LONG)
            {
                APP_TRACE_DBG("Pairing\n");
                return app_main_button_primary_handler(PLATFORM_BUTTON_PAIRING,repeat_counter);
            }

            return WICED_FALSE;

        case GLOBAL_IDLE_STATE:
            switch(button_id)
            {
            case PLATFORM_BUTTON_LRAC_BUTTON_1_SHORT:
                /* Reset Connection status upon button press to force reconnection attempts */
                app_main_cb.lrac.connection_status = WICED_BT_SUCCESS;
                app_main_connect();
                break;

            default: /* Already handled above */
                break;
            }
            break;

        case GLOBAL_POWER_DOWN_STATE:
            return WICED_TRUE;

        default:
            APP_TRACE_ERR("Bad State:%d\n", platform_state_get());
            return WICED_TRUE;
        }
        break;

    case PLATFORM_BUTTON_LRAC_BUTTON_2_SHORT:
    case PLATFORM_BUTTON_LRAC_BUTTON_2_LONG:
        APP_TRACE_DBG("LRAC Button id:%d\n",button_id);
        /* The volume up / next track operation shall only be handled with specific states. */
        switch (platform_state_get())
        {
        case GLOBAL_POWER_UP_STATE:
        case GLOBAL_POWER_DOWN_STATE:
            /* In PowerUp and PowerDown states, ignore this button */
            return WICED_TRUE;

        case GLOBAL_IDLE_STATE:
            break;

        default:
            APP_TRACE_ERR("Bad State:%d\n", platform_state_get());
            return WICED_TRUE;
        }

        if (button_id == PLATFORM_BUTTON_LRAC_BUTTON_2_SHORT)
        {
            return app_main_button_primary_handler(PLATFORM_BUTTON_VOLUME_UP, 0);
        }
        break;

    case PLATFORM_BUTTON_LRAC_BUTTON_3_SHORT:
    case PLATFORM_BUTTON_LRAC_BUTTON_3_LONG:
        APP_TRACE_DBG("LRAC Button id:%d\n",button_id);
        /* The volume up / next track operation shall only be handled with specific states. */
        switch (platform_state_get())
        {
        case GLOBAL_POWER_UP_STATE:
        case GLOBAL_POWER_DOWN_STATE:
            /* In PowerUp and PowerDown states, ignore this button */
            return WICED_TRUE;

        case GLOBAL_IDLE_STATE:
            break;

        default:
            APP_TRACE_ERR("Bad State:%d\n", platform_state_get());
            return WICED_TRUE;
        }

        if (button_id == PLATFORM_BUTTON_LRAC_BUTTON_3_SHORT)
        {
            return app_main_button_primary_handler(PLATFORM_BUTTON_VOLUME_DOWN, 0);
        }
        break;

    case PLATFORM_BUTTON_WICED_HCI_DETECT_ON:
        wiced_transport_set_detect_on(TRANSPORT_DETECT_SECONDS);
        break;
    default:
        APP_TRACE_ERR("Unknown ButtonId:%d\n", button_id);
        ret = WICED_FALSE;
        break;
    }
    return ret;
}

/*
 * app_main_button_callback
 */
static wiced_bool_t app_main_button_callback(platform_button_id_t button_id,uint32_t repeat_counter)
{
    switch (app_main_cb.lrac.role)
    {
    case WICED_BT_LRAC_ROLE_PRIMARY:
        return app_main_button_primary_handler(button_id,repeat_counter);

    case WICED_BT_LRAC_ROLE_SECONDARY:
        return app_main_button_secondary_handler(button_id,repeat_counter);

    default:
        APP_TRACE_ERR("Unknown role !!!!!!\n");
        APP_TRACE_ERR("LRAC Role Unknown. Use the LRAC Config tool to configure the board.\n");
        break;
    }

    return WICED_TRUE;
}

/*
 * app_main_quality_callback
 */
static void app_main_quality_callback(app_lrac_quality_event_t event,
       app_lrac_quality_event_data_t *p_data)
{
    switch (event)
    {
    case APP_LRAC_QUALITY_UNDERRUN:
        if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            /*
             * If we are Primary, tell it to A2DP Sink.
             * It will increase the Jitter Buffer Target depth (if possible) and send the new
             * A2DP Delay to the A2DP source
             */
            app_a2dp_sink_underrun();
        }
        else
        {
            /* As Secondary, we may want to tell the Primary we had an UnderRun... */
        }
        break;

    default:
        APP_TRACE_ERR("Unknown event:%d\n", event);
        break;
    }
}
/*
 * app_main_platform_charger_callback
 */
static void app_main_platform_charger_callback(platform_charger_event_t event)
{
    switch(event)
    {
    case PLATFORM_CHARGER_INSERTED:
        APP_TRACE_DBG("Charger Inserted\n");
        bt_hs_spk_control_disconnect(NULL);
        platform_led_set(PLATFORM_LED_CHARGER, event);

        APP_TRACE_DBG("Disconnecting LRAC\n");
        if (app_main_cb.lrac.connected)
            app_lrac_disconnect();

        APP_TRACE_DBG("Disabling Bluetooth visibility.\n");
        bt_hs_spk_control_handle_set_visibility(WICED_FALSE, WICED_FALSE, BT_TRANSPORT_BR_EDR);
        break;

    case PLATFORM_CHARGER_REMOVED:
        APP_TRACE_DBG("Charger Removed. Rebooting...\n");
        platform_led_set(PLATFORM_LED_CHARGER, event);
        wdog_generate_hw_reset();   /* Generate a watchdog Reset */
        break;

    case PLATFORM_CHARGER_CHARGE_COMPLETE:
        platform_led_set(PLATFORM_LED_CHARGER, event);
        break;

    default:
        APP_TRACE_ERR("Unknown event:%d\n", event);
        break;
    }
}

/*
 * Update local device's device name
 *
 * For demo purpose, add last byte of the BdAddr to the first two characters of device's name.
 */
static void app_main_update_dev_dev_name(uint8_t *local_bdaddr)
{
    char device_name[APP_MAIN_BT_DEV_NAME_LEN] = {'\0'};
    char lut[16] = "0123456789ABCDEF";
    uint16_t i = 0;
    uint16_t assigned_dev_name_len = strlen((char *)wiced_bt_cfg_settings.device_name);

    memset(device_name, 0, sizeof(device_name));
    device_name[i++] = lut[(local_bdaddr[BD_ADDR_LEN -1] >> 4) & 0x0F];
    device_name[i++] = lut[local_bdaddr[BD_ADDR_LEN -1] & 0x0F];
    device_name[i++] = ' ';

    if ((i + assigned_dev_name_len + 1) > APP_MAIN_BT_DEV_NAME_LEN)
    {
        /* At one byte shall be reserved for string ending, '\0'. */
        strncpy(&device_name[i],
                (char *)wiced_bt_cfg_settings.device_name,
                (APP_MAIN_BT_DEV_NAME_LEN - i - 1));
    }
    else
    {
        strncpy(&device_name[i],
                (char *)wiced_bt_cfg_settings.device_name,
                assigned_dev_name_len);
    }

    app_bt_eir_write(device_name, WICED_FALSE);
#ifndef AMA_ENABLED
    app_ble_name_set(device_name);
#endif
}

/*
 * app_main_update_dev
 */
static wiced_result_t app_main_update_dev (void)
{
    wiced_bool_t ret;
    uint8_t local_bdaddr[BD_ADDR_LEN];

    if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        WICED_BT_TRACE("Configure device as Primary\n");

        BTM_SetDeviceClass((uint8_t *)wiced_bt_cfg_settings.device_class);
        WICED_BT_TRACE("Device Class: 0x%02x%02x%02x\n",wiced_bt_cfg_settings.device_class[0],wiced_bt_cfg_settings.device_class[1],wiced_bt_cfg_settings.device_class[2]);

        /* Update device's name. */
        app_nvram_local_bdaddr_read(local_bdaddr);
        app_main_update_dev_dev_name(&local_bdaddr[0]);

        /* Update SDP database. */
        ret = wiced_bt_sdp_db_init((uint8_t *)headset_sdp_db, headset_sdp_db_size);
        if( ret != TRUE )
        {
            APP_TRACE_ERR("wiced_bt_sdp_db_init(Headset) failed\n");
            return WICED_BT_ERROR;
        }
    }
    else
    {
        WICED_BT_TRACE("Configure device as Secondary\n");
        BTM_SetDeviceClass((uint8_t *)wiced_bt_cfg_settings.device_class);
        WICED_BT_TRACE("Device Class: 0x%02x%02x%02x\n",wiced_bt_cfg_settings.device_class[0],wiced_bt_cfg_settings.device_class[1],wiced_bt_cfg_settings.device_class[2]);
        app_bt_eir_write("do not connect", WICED_TRUE);
#ifndef AMA_ENABLED
        app_ble_name_set("do not connect");
#endif
        ret = wiced_bt_sdp_db_init((uint8_t *)lrac_sdp_db, lrac_sdp_db_size);
        if( ret != TRUE )
        {
            APP_TRACE_ERR("wiced_bt_sdp_db_init(LRAC) failed\n");
            return WICED_BT_ERROR;
        }
    }
    return WICED_BT_SUCCESS;
}

/*
 * app_main_update_dev_after_switch
 */
static wiced_result_t app_main_update_dev_after_switch (void)
{
    wiced_bool_t ret;
    uint8_t local_bdaddr[BD_ADDR_LEN];

    if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        WICED_BT_TRACE("Configure device as Primary\n");

        /* Update device's name. */
        wiced_bt_dev_read_local_addr(local_bdaddr);
        app_main_update_dev_dev_name(&local_bdaddr[0]);

        /* Update SDP database. */
        ret = wiced_bt_sdp_db_init((uint8_t *)headset_sdp_db, headset_sdp_db_size);
        if (ret == WICED_FALSE)
        {
            APP_TRACE_ERR("wiced_bt_sdp_db_init(Headset) failed\n");
            return WICED_BT_ERROR;
        }
    }
    else
    {
        WICED_BT_TRACE("Configure device as Secondary\n");
        app_bt_eir_write("do not connect", WICED_TRUE);
#ifndef AMA_ENABLED
        app_ble_name_set("do not connect");
#endif
        ret = wiced_bt_sdp_db_init((uint8_t *)lrac_sdp_db, lrac_sdp_db_size);
        if (ret == WICED_FALSE)
        {
            APP_TRACE_ERR("wiced_bt_sdp_db_init(LRAC) failed\n");
            return WICED_BT_ERROR;
        }
    }
    return WICED_BT_SUCCESS;
}

/*
 * app_main_a2dp_sink_codec_get
 */
wiced_bt_a2dp_codec_info_t *app_main_a2dp_sink_codec_get(void)
{
    return &app_main_cb.a2dp_sink.codec_config;
}

/*
 * app_main_connect
 */
static wiced_result_t app_main_connect(void)
{
    if ((app_main_cb.lrac.connected == WICED_FALSE) &&
        (app_main_cb.lrac.connection_status == WICED_BT_SUCCESS))  /* Prevent forever loop */
    {
        /* Try to connect to the peer LRAC Device (Secondary) */
        app_lrac_connect();
        return WICED_BT_SUCCESS;
    }

    return WICED_BT_SUCCESS;
}

/*
 * app_main_switch_is_ready
 */
wiced_bool_t app_main_switch_is_ready(void)
{
    /*
     * This function will be called before LRAC Switch will be performed.
     * The main module may prevent the Switch to happen here.
     */

#ifdef APP_OFU_SUPPORT
    if (app_main_cb.ofu.ongoing)
    {
        APP_TRACE_ERR("OFU Ongoing. Switch not allowed\n");
        return WICED_FALSE;
    }
#endif

    /* Check if the system is under reconnection process. */
    if (bt_hs_spk_control_reconnect_state_get())
    {
        APP_TRACE_ERR("Reconnecting. Switch not allowed\n");
        return WICED_FALSE;
    }

    return WICED_TRUE;
}

/*
 * app_main_switch_get
 */
wiced_result_t app_main_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
{
    uint16_t expected_data_len;
    uint8_t *p;

    /* This function will, perhaps, have to read some NVRAM data */
    if (p_opaque == NULL)
    {
        APP_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    if (p_sync_data_len == NULL)
    {
        APP_TRACE_ERR("p_sync_data_len is NULL\n");
        return WICED_BT_BADARG;
    }

    expected_data_len = sizeof(app_main_cb) +
                        sizeof(bt_hs_spk_control_connection_info_t) +
                        sizeof(bt_hs_spk_handsfree_call_session_info_t) +
                        sizeof(bt_hs_spk_audio_context_info_t);

    if (*p_sync_data_len < expected_data_len)
    {
        APP_TRACE_ERR("buffer too small (%d/%d)\n", *p_sync_data_len, expected_data_len);
        return WICED_BT_BADARG;
    }

    p = (uint8_t *) p_opaque;

    memcpy(p, &app_main_cb, sizeof(app_main_cb));
    p += sizeof(app_main_cb);

    bt_hs_spk_control_connection_info_get((bt_hs_spk_control_connection_info_t *) p);
    p += sizeof(bt_hs_spk_control_connection_info_t);

    bt_hs_spk_handsfree_call_session_info_get((bt_hs_spk_handsfree_call_session_info_t *) p);
    p += sizeof(bt_hs_spk_handsfree_call_session_info_t);

    bt_hs_spk_audio_audio_context_info_get((bt_hs_spk_audio_context_info_t *) p);
    p += sizeof(bt_hs_spk_audio_context_info_t);

    *p_sync_data_len = expected_data_len;

    return WICED_BT_SUCCESS;
}

/*
 * app_main_switch_set
 */
wiced_result_t app_main_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    uint16_t expected_data_len;
    uint8_t *p;

    if (p_opaque == NULL)
    {
        APP_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    expected_data_len = sizeof(app_main_cb) +
                        sizeof(bt_hs_spk_control_connection_info_t) +
                        sizeof(bt_hs_spk_handsfree_call_session_info_t) +
                        sizeof(bt_hs_spk_audio_context_info_t);

    if (sync_data_len != expected_data_len)
    {
        APP_TRACE_ERR("bad buffer size (%d/%d)\n", sync_data_len, expected_data_len);
        return WICED_BT_BADARG;
    }

    p = (uint8_t *) p_opaque;

    memcpy(&app_main_cb, p_opaque, sizeof(app_main_cb));
    p += sizeof(app_main_cb);

    bt_hs_spk_control_connection_info_set((bt_hs_spk_control_connection_info_t *) p);
    p += sizeof(bt_hs_spk_control_connection_info_t);

    bt_hs_spk_handsfree_call_session_info_set((bt_hs_spk_handsfree_call_session_info_t *) p);
    p += sizeof(bt_hs_spk_handsfree_call_session_info_t);

    bt_hs_spk_audio_audio_context_info_set((bt_hs_spk_audio_context_info_t *) p);

    /* update discoverable and connectable */
    bt_hs_spk_button_lrac_switch_restore_visibility(app_main_cb.lrac_switch.bt_visibility.dissoverable,
            app_main_cb.lrac_switch.bt_visibility.connectable,
            app_main_cb.lrac_switch.remain_discovery_timer);

    return WICED_BT_SUCCESS;
}

/*
 * app_main_a2dp_stop
 */
void app_main_a2dp_stop(void)
{
    wiced_result_t status;

    if ((app_main_cb.lrac.connected) &&
        (app_main_cb.a2dp_sink.lrac_started))
    {
        /* Ask the secondary to stop A2DP eavesdropping */
        status = app_lrac_a2dp_stop_req();
        if (status != WICED_BT_SUCCESS)
        {
            APP_TRACE_ERR("app_lrac_a2dp_stop_req failed %d\n", status);
        }
    }

    platform_led_set(PLATFORM_LED_A2DP_STREAM_STATE, 0);
}

/*
 * app_main_codec_route_set
 */
static wiced_result_t app_main_codec_route_set(platform_codec_route_t codec_route)
{
    return platform_codec_route_set(codec_route);
}

/*
 * app_main_free_memory_check
 * Previous study said that if the Free Bytes After Init is lower than 1052 bytes,
 * WICED-HCI will not work and core-dump message shown. (See SWWICED-12265 comment)
 * Add a check function to prevent free memory insufficiency.
 */
void app_main_free_memory_check(void)
{
#define MIN_FREE_MEMORY_BYTE    1052
    if (wiced_memory_get_free_bytes() < MIN_FREE_MEMORY_BYTE)
    {
        WICED_BT_TRACE("ERR: Free Memory insufficient! Rebooting.\n");
        wiced_hal_wdog_reset_system();
    }
}

#ifdef APPLICATION_WATCHDOG_ENABLED
/*
 * app_wdog_callback
 */
static void app_wdog_callback(uint8_t* app_thread_cb, uint32_t app_thread_cb_size,
        uint8_t* app_thread_stack, uint32_t app_thread_stack_size)
{
    APP_TRACE_ERR("Application thread watchdog timeout!\n");

    APP_TRACE_ERR("app thread control block address: %x  size: %d\n", (UINT32)app_thread_cb, app_thread_cb_size);
    WICED_BT_TRACE_ARRAY(app_thread_cb, app_thread_cb_size, "app thread control block dump:\n");

    APP_TRACE_ERR("app thread stack address: %x  size: %d\n", (UINT32)app_thread_stack, app_thread_stack_size);
    WICED_BT_TRACE_ARRAY(app_thread_stack, app_thread_stack_size, "app thread stack dump:\n");
}
#endif

/*
 * app_main_nvram_link_keys_update_callback
 *
 * Callback when the NVRAM section for storing link keys has been updated.
 */
static void app_main_nvram_link_keys_update_callback(void)
{
    app_lrac_nvram_update_req();
}

/*
 * app_main_local_volume_change_handler
 *
 * Handle the case that local volume has been changed.
 *
 * @param[in]   am_vol_level  - Audio Manager volume level
 * @param[in]   am_vol_effect_event - Audio Manager volume effect event received from bt_hs_spk_lib
 */
static void app_main_local_volume_change_handler(int32_t am_vol_level, uint8_t am_vol_effect_event)
{
    APP_TRACE_DBG("app_main_local_volume_change_handler (vol=%d effect=%d)\n", am_vol_level, am_vol_effect_event);

    if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
#ifdef VOLUME_EFFECT
        if ((am_vol_effect_event == VOLUME_EFFECT_NONE) ||
                (am_vol_effect_event == VOLUME_EFFECT_RAMP_UP))
        {
            /* Stop timer if enabled */
            app_volume_effect_stop_timer();
            /* Store am_vol_level */
            app_volume_set(APP_VOLUME_SELECT_NONE, am_vol_level, am_vol_effect_event);
        }

        if (am_vol_effect_event == VOLUME_EFFECT_INIT_MUTE)
        {
            /* Initial mute when audio start, set the timer to ramp up volume */
            app_volume_effect_start_timer(VOLUME_RAMP_UP_TIME_INIT_MUTE, am_vol_effect_event);
        }
        else if (am_vol_effect_event == VOLUME_EFFECT_INIT_HFP_MUTE)
        {
            /* Initial mute when voice start, set the timer to ramp up volume */
            app_volume_effect_start_timer(VOLUME_RAMP_UP_TIME_INIT_HFP_MUTE, am_vol_effect_event);
        }

        if (app_main_cb.lrac.connected &&
            (am_vol_effect_event != VOLUME_EFFECT_INIT_MUTE) &&
            (am_vol_effect_event != VOLUME_EFFECT_INIT_HFP_MUTE))
        {
            /* Inform Secondary Device, for VOLUME_EFFECT_INIT_MUTE, PRI and SEC will handle by themselves */
            app_lrac_volume_send(am_vol_level, am_vol_effect_event);
        }
#else
        if (app_main_cb.lrac.connected)
        {
            /* Inform Secondary Device to update the volume. */
            app_lrac_volume_send(am_vol_level, am_vol_effect_event);
        }
#endif
    }
#ifdef VOLUME_EFFECT
    else if (app_main_cb.lrac.role == WICED_BT_LRAC_ROLE_SECONDARY)
    {
        if (am_vol_effect_event == VOLUME_EFFECT_INIT_MUTE)
        {
            if (app_main_cb.a2dp_sink.lrac_started)
            {
                if (app_main_cb.a2dp_sink.sec_sync_start)
                {
                    app_volume_effect_start_timer(VOLUME_RAMP_UP_TIME_INIT_MUTE, am_vol_effect_event);
                }
                else
                {
                    app_volume_effect_start_timer(VOLUME_RAMP_UP_TIME_JOIN_LATE, am_vol_effect_event);
                }
            }
            else
            {
                app_volume_effect_start_timer(VOLUME_RAMP_UP_TIME_INIT_MUTE, am_vol_effect_event);
            }
        }
        else if (am_vol_effect_event == VOLUME_EFFECT_INIT_HFP_MUTE)
        {
            /* Initial mute when voice start, set the timer to ramp up volume */
            app_volume_effect_start_timer(VOLUME_RAMP_UP_TIME_INIT_HFP_MUTE, am_vol_effect_event);
        }
        else if (am_vol_effect_event == VOLUME_EFFECT_UNDERRUN_MUTE)
        {
            /* Stop timer first if timer already set by the previous underrun or init mute */
            app_volume_effect_stop_timer();
            app_volume_effect_start_timer(VOLUME_RAMP_UP_TIME_UNDERRUN, am_vol_effect_event);
        }
        else if (am_vol_effect_event == VOLUME_EFFECT_RAMP_UP)
        {
            /* Stop timer because Primary already inform the bt_hs_spk_lib to ramp up volume immediately */
            app_volume_effect_stop_timer();
        }
    }
#endif
}

/*
 * app_main_a2dp_streaming_source_device_changed_handler
 *
 * Handle the case where the audio streaming source device has been changed.
 */
static void app_main_a2dp_streaming_source_device_changed_handler(void)
{
    APP_TRACE_DBG("app_main_a2dp_streaming_source_device_changed_handler\n");

    /* Stop existent audio insertion. */
    if (app_audio_insert_stop_req(APP_AUDIO_INSERT_STOP_REQ_SUSPEND) != WICED_BT_SUCCESS)
    {
        return;
    }

    /* Ask the Secondary to stop existent eavesdropping. */
    if ((app_main_cb.lrac.connected) &&
        (app_main_cb.a2dp_sink.lrac_started))
    {
        app_lrac_a2dp_stop_req();
    }
}

/*
 * app_main_bt_visibility_change_handler
 *
 * Handle the case when the BR/EDR visibility has been changed.
 */
static void app_main_bt_visibility_change_handler(wiced_bool_t discoverable, wiced_bool_t connectable)
{
    WICED_BT_TRACE("app_main_bt_visibility_change_handler (%d, %d, %d)\n",
                  discoverable,
                  connectable,
                  app_main_cb.lrac.connected);

    /* Update information. */
    app_main_cb.bt_visibility.dissoverable = discoverable;
    app_main_cb.bt_visibility.connectable = connectable;

    /* Handle only with assigned device role. */
    if ((app_main_cb.lrac.role != WICED_BT_LRAC_ROLE_PRIMARY) &&
        (app_main_cb.lrac.role != WICED_BT_LRAC_ROLE_SECONDARY))
    {
        return;
    }

    /* Only handle the case that the connectivity has been disabled. */
    if (connectable)
    {
        return;
    }

    /* Handle only when LRAC is not connected yet. */
    if (app_main_cb.lrac.connected)
    {
        return;
    }

    /* Enable the connectivity for LRAC. */
    bt_hs_spk_control_handle_set_visibility(discoverable, WICED_TRUE, BT_TRANSPORT_BR_EDR);
}

/*
 * app_main_lrac_switch_backup_bt_visibility
 *
 * backup Bluetooth visibility before lrac switch
 */
void app_main_lrac_switch_backup_bt_visibility(void)
{
    memcpy(&app_main_cb.lrac_switch.bt_visibility,
            &app_main_cb.bt_visibility,
            sizeof(app_main_cb.lrac_switch.bt_visibility));

    app_main_cb.lrac_switch.remain_discovery_timer = bt_hs_spk_button_get_remain_bt_service_timer();
}
#ifdef VOICE_PROMPT

/*
 * app_main_audio_insert_ready_to_pair
 *
 * Trigger to play the "ready to pair" voice prompt
 */
static int app_main_audio_insert_ready_to_pair(void *p_data)
{
    app_audio_insert_start_req(APP_VOICE_PROMPT_INDEX_READY_TO_PAIR);

    return 0;
}

/*
 * app_main_audio_insert_volume_max
 *
 * Trigger to play the "volume maximum" voice prompt
 */
static int app_main_audio_insert_volume_max(void *p_data)
{
    app_audio_insert_start_req(APP_VOICE_PROMPT_INDEX_VOLUME_MAX);

    return 0;
}

/*
 * app_main_audio_insert_bt_connected
 *
 * Trigger to play the "Bluetooth Connected" voice prompt
 */
static int app_main_audio_insert_bt_connected(void *p_data)
{
    app_audio_insert_start_req(APP_VOICE_PROMPT_INDEX_BT_CONNECTED);

    return 0;
}

#endif // VOICE_PROMPT
