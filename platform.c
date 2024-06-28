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
 * This file implements platform specific functions
 */
#include "app_lrac.h"
#include "app_trace.h"
#include "app_hci.h"
#ifdef AMA_ENABLED
#include <ama.h>
#else
#include "app_ble.h"
#endif
#include "app_main.h"
#include "app_volume.h"
#include "app_a2dp_sink.h"
#include "clock_timer.h"            /* clock_SystemTimeMicroseconds64() */
#include "wiced_audio_manager.h"
#ifndef PLATFORM_LED_DISABLED
#include "wiced_led_manager.h"
#endif // !PLATFORM_LED_DISABLED
#include "wiced_transport.h"
//#include "wiced_button_manager.h"
#include "wiced_platform.h"
#include "wiced_platform_audio_common.h"
#include "wiced_bt_dev.h"
#include "wiced_hal_eflash.h"
#include "app_audio_insert.h"
#include "bt_hs_spk_button.h"
#include "bt_hs_spk_handsfree_utils.h"
#include "bt_hs_spk_audio.h"
#include "bt_hs_spk_control.h"
#include "bt_hs_spk_handsfree.h"

/*
 * Definitions
 */
#define PLATFORM_CONNECTED_LRAC         0x01    /* LRAC Connected */
#define PLATFORM_CONNECTED_A2DP         0x02    /* A2DP Connected */
#define PLATFORM_CONNECTED_AVRC         0x04    /* AVRC Connected */
#define PLATFORM_CONNECTED_HFP          0x08    /* HFP Connected */

#define SYSTEM_UNDERRUN_STATE           0x14

#define PLATFORM_EVB1_LED_RED           PLATFORM_LED_1
#define PLATFORM_EVB1_LED_GREEN         PLATFORM_LED_2

#ifndef APP_BUTTON_MAX
#define APP_BUTTON_MAX WICED_PLATFORM_BUTTON_MAX_DEF
#endif

typedef enum
{
    PLATFORM_BTN_DRV_ACTION_PRESSED = 0,
    PLATFORM_BTN_DRV_ACTION_SHORT,
    PLATFORM_BTN_DRV_ACTION_LONG_HELD,
    PLATFORM_BTN_DRV_ACTION_LONG_RELEASE,
    PLATFORM_BTN_DRV_ACTION_REPEAT,
    PLATFORM_BTN_DRV_ACTION_RELEASED,
    PLATFORM_BTN_DRV_ACTION_INVALID,
} platform_btn_drv_action_t;

typedef enum
{
    AUDIO_DECODE_STEREO = 0,
    AUDIO_DECODE_MONO_LEFT,
    AUDIO_DECODE_MONO_RIGHT,
    AUDIO_DECODE_MONO_MIX_LR    /* MONO_MIX_LR only supported in SBC decoder */
} audio_decode_channel_t;

/*
 * This compile option is used to Disable LEDs. This allow these gpio to be used for other
 * debug purposes (e.g. FW debug)
 */
#ifdef PLATFORM_DEBUG
#define PLATFORM_LED_DISABLED
#endif

/*
 * Structures
 */
typedef struct
{
    platform_button_callback_t *p_app_button_callback;
    platform_global_state_t state;
    wiced_bt_lrac_role_t lrac_role;
} platform_cb_t;

typedef struct
{
    platform_btn_drv_action_t       action;
    button_manager_event_t          event;
    button_manager_button_state_t   state;
} plaform_button_action_mapping_t;

typedef struct
{
    platform_button_id_t        id;
    platform_button_t           button;
    platform_btn_drv_action_t   action;
} platform_button_id_mapping_t;

/*
 * Global variables
 */
platform_cb_t platform_cb;

/*
 * External functions
 */
wiced_result_t lite_host_lrac_enableI2SAudioSwapLR(uint8_t enable);
extern wiced_result_t wiced_audio_sink_set_decode_channel(uint8_t ch);

/*
 * Local functions
 */
static wiced_bool_t platform_button_drv_callback(platform_button_t button, button_manager_event_t event, button_manager_button_state_t state, uint32_t repeat);

static char *platform_state_get_descr(platform_global_state_t state);

static void platform_vsc_cmd_cplt_callback(
        wiced_bt_dev_vendor_specific_command_complete_params_t *p_cmd_cplt_param);

#ifdef CYW9BT_AUDIO
static wiced_bool_t platform_vse_callback (uint8_t len, uint8_t *p);
#endif

#ifndef PLATFORM_LED_DISABLED
static void platform_init_led( void );
#endif

/*
 * Static variable
 */
#ifndef PLATFORM_LED_DISABLED
/* LED configuration for APP. status indication */
static wiced_led_config_t platform_evb1_led_config[] =
{
    {
        .led = PLATFORM_LED_1,
        .bright = 50,
    },
    {
        .led = PLATFORM_LED_2,
        .bright = 50,
    }
};
#endif

// variables used for button manager
static button_manager_t platform_evb1_button_manager;

static wiced_button_manager_configuration_t platform_evb1_button_manager_configuration =
{
    .short_hold_duration        = 500,  /*msec*/
    .medium_hold_duration       = 700,
    .long_hold_duration         = 1500,
    .very_long_hold_duration    = 2500,
    .debounce_duration          = 50,   /* typically a click takes around ~150-200 ms */
    .continuous_hold_detect     = WICED_TRUE,
    .event_handler              = NULL,
};

static wiced_button_configuration_t platform_evb1_button_configurations[] =
{
#ifndef PLATFORM_DEBUG
    /*
     * SW15
     *      Short Click: Power On
     *                   A2DP Play
     *                   A2DP Pause
     *                   HFP Hang-up
     *                   HFP Answer
     *      Long Click: HFP Hang-up
     *      Holding: Pairing, HFP Hang-up
     */
    {
        .button             = PLATFORM_BUTTON_1,
        .button_event_mask  = BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_HOLDING_EVENT,
        .application_event  = 0
    },
    /*
     * SW17
     *      Short Click \ Holding: Volume up
     *      Long Click: Next Track
     */
    {
        .button             = PLATFORM_BUTTON_2,
        .button_event_mask  = BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_HOLDING_EVENT,
        .application_event  = 0
    },
    /*
     * SW18
     *      Short Click \ Holding: Volume down
     *      Long Click: Previous Track
     */
    {
        .button             = PLATFORM_BUTTON_3,
        .button_event_mask  = BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_HOLDING_EVENT,
        .application_event  = 0
    },
#if (APP_BUTTON_MAX >= 4)

    #ifdef APP_TRANSPORT_DETECT_ON
    /*
     * SW19
     *      Short Click: Enable Transport Detect
     *      Holding: Trigger AMA if AMA is connected.
     *                  Otherwise, trigger VOICE_REC
     */
    {
        .button             = PLATFORM_BUTTON_4,
        .button_event_mask  = BUTTON_CLICK_EVENT | BUTTON_HOLDING_EVENT,
        .application_event  = 0
    },
    #else   // !APP_TRANSPORT_DETECT_ON
    /*
     * SW19
     *      Holding: Trigger AMA if AMA is connected.
     *                  Otherwise, trigger VOICE_REC
     */
    {
        .button             = PLATFORM_BUTTON_4,
        .button_event_mask  = BUTTON_HOLDING_EVENT,
        .application_event  = 0
    },
    #endif  // APP_TRANSPORT_DETECT_ON
#endif // APP_BUTTON_MAX >= 4

#else   // PLATFORM_DEBUG

#if (APP_BUTTON_MAX >= 4)
    /*
     * SW19
     *      Short Click: Power On
     *                   A2DP Play
     *                   A2DP Pause
     *                   HFP Hang-up
     *                   HFP Answer
     *      Long Click: HFP Hang-up
     *      Holding: Pairing, HFP Hang-up
     */
    {
        .button             = PLATFORM_BUTTON_4,
        .button_event_mask  = BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_HOLDING_EVENT,
        .application_event  = 0
    },
#endif // APP_BUTTON_MAX >= 4

#endif  // PLATFORM_DEBUG
};

button_manager_button_t platform_evb1_buttons[] =
{
#ifndef PLATFORM_DEBUG
    {
        .configuration  = &platform_evb1_button_configurations[ PLATFORM_BUTTON_1 ]
    },
    {
        .configuration  = &platform_evb1_button_configurations[ PLATFORM_BUTTON_2 ]
    },
    {
        .configuration  = &platform_evb1_button_configurations[ PLATFORM_BUTTON_3 ]
    },
#if (APP_BUTTON_MAX >= 4)
    {
        .configuration  = &platform_evb1_button_configurations[ PLATFORM_BUTTON_4 ]
    },
#endif // APP_BUTTON_MAX >= 4
#else   // PLATFORM_DEBUG
    {
        .configuration  = &platform_evb1_button_configurations[ 0 ]
    },
#endif  // PLATFORM_DEBUG
};

static bt_hs_spk_button_action_t platform_evb1_button_action[] =
{
#ifndef PLATFORM_DEBUG
    /*
     * PLAY_PAUSE_BUTTON
     *      Short Click: Power On
     *                   A2DP Play
     *                   A2DP Pause
     *                   HFP Hang-up
     *                   HFP Answer
     *      Long Click: HFP Hang-up
     *      Holding: Pairing, HFP Hang-up
     */
    {
        .action = ACTION_PAUSE_PLAY,
        .button = PLAY_PAUSE_BUTTON,
        .event  = BUTTON_CLICK_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
    {
        .action = ACTION_BT_DISCOVERABLE,
        .button = PLAY_PAUSE_BUTTON,
        .event  = BUTTON_LONG_DURATION_EVENT,
        .state  = BUTTON_STATE_HELD,
    },
    /*
     * VOLUME_UP_NEXT_TRACK_BUTTON
     *      Short Click \ Holding: Volume up
     *      Long Click: Next Track
     */
    {
        .action = ACTION_VOLUME_UP,
        .button = VOLUME_UP_NEXT_TRACK_BUTTON,
        .event  = BUTTON_CLICK_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
    {
        .action = ACTION_FORWARD,
        .button = VOLUME_UP_NEXT_TRACK_BUTTON,
        .event  = BUTTON_LONG_DURATION_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
#if (APP_BUTTON_MAX < 4)
    {
        .action = ACTION_VOICE_RECOGNITION,
        .button = VOLUME_UP_NEXT_TRACK_BUTTON,
        .event  = BUTTON_HOLDING_EVENT,
        .state  = BUTTON_STATE_HELD,
        .repeat = 2,
    },
#endif

    /*
     * VOLUME_DOWN_PREVIOUS_TRACK_BUTTON
     *      Short Click \ Holding: Volume down
     *      Long Click: Previous Track
     */
    {
        .action = ACTION_VOLUME_DOWN,
        .button = VOLUME_DOWN_PREVIOUS_TRACK_BUTTON,
        .event  = BUTTON_CLICK_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
    {
        .action = ACTION_BACKWARD,
        .button = VOLUME_DOWN_PREVIOUS_TRACK_BUTTON,
        .event  = BUTTON_LONG_DURATION_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
#if (APP_BUTTON_MAX < 4)
    {
        .action = ACTION_TRANSPORT_DETECT_ON,
        .button = VOLUME_DOWN_PREVIOUS_TRACK_BUTTON,
        .event  = BUTTON_HOLDING_EVENT,
        .state  = BUTTON_STATE_HELD,
        .repeat = 2,
    },
#endif

#if (APP_BUTTON_MAX >= 4)
    /*
     * VOICE_REC_BUTTON
     *
     *      Holding: VOICE_REC
     */
	#ifdef APP_TRANSPORT_DETECT_ON
    {
        .action = ACTION_TRANSPORT_DETECT_ON,
        .button = VOICE_REC_BUTTON,
        .event  = BUTTON_CLICK_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
    {
        .action = ACTION_VOICE_RECOGNITION,
        .button = VOICE_REC_BUTTON,
        .event  = BUTTON_HOLDING_EVENT,
        .state  = BUTTON_STATE_HELD,
        .repeat = 2,
    },
	#else   // !APP_TRANSPORT_DETECT_ON
    {
        .action = ACTION_VOICE_RECOGNITION,
        .button = VOICE_REC_BUTTON,
        .event  = BUTTON_HOLDING_EVENT,
        .state  = BUTTON_STATE_HELD,
        .repeat = 2,
    },
    #endif  // APP_TRANSPORT_DETECT_ON
#endif  // (APP_BUTTON_MAX >= 4)

#else   // PLATFORM_DEBUG

#if (APP_BUTTON_MAX >= 4)
    /*
     * VOICE_REC_BUTTON
     *      Short Click: Power On
     *                   A2DP Play
     *                   A2DP Pause
     *                   HFP Hang-up
     *                   HFP Answer
     *      Long Click: HFP Hang-up
     *      Holding: Pairing, HFP Hang-up
     */
    {
        .action = ACTION_PAUSE_PLAY,
        .button = VOICE_REC_BUTTON,
        .event  = BUTTON_CLICK_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
    {
        .action = ACTION_BT_DISCOVERABLE,
        .button = VOICE_REC_BUTTON,
        .event  = BUTTON_LONG_DURATION_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
#endif  // (APP_BUTTON_MAX >= 4)
#endif  // PLATFORM_DEBUG
};

const plaform_button_action_mapping_t platform_button_action_mapping[] =
{
    {
        .action = PLATFORM_BTN_DRV_ACTION_SHORT,
        .event  = BUTTON_CLICK_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
    {
        .action = PLATFORM_BTN_DRV_ACTION_LONG_HELD,
        .event  = BUTTON_LONG_DURATION_EVENT,
        .state  = BUTTON_STATE_HELD,
    },
    {
        .action = PLATFORM_BTN_DRV_ACTION_LONG_RELEASE,
        .event  = BUTTON_LONG_DURATION_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
    {
        .action = PLATFORM_BTN_DRV_ACTION_REPEAT,
        .event  = BUTTON_HOLDING_EVENT,
        .state  = BUTTON_STATE_HELD,
    },
};

const platform_button_id_mapping_t platform_button_id_mapping[] =
{
#ifndef PLATFORM_DEBUG
    /* PLATFORM_BUTTON_1 */
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_1_SHORT,
        .button = PLATFORM_BUTTON_1,
        .action = PLATFORM_BTN_DRV_ACTION_SHORT,
    },
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_1_LONG,
        .button = PLATFORM_BUTTON_1,
        .action = PLATFORM_BTN_DRV_ACTION_LONG_HELD,
    },
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_1_REPEAT,
        .button = PLATFORM_BUTTON_1,
        .action = PLATFORM_BTN_DRV_ACTION_REPEAT,
    },
    /* PLATFORM_BUTTON_2 */
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_2_SHORT,
        .button = PLATFORM_BUTTON_2,
        .action = PLATFORM_BTN_DRV_ACTION_SHORT,
    },
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_2_LONG,
        .button = PLATFORM_BUTTON_2,
        .action = PLATFORM_BTN_DRV_ACTION_LONG_RELEASE,
    },
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_2_REPEAT,
        .button = PLATFORM_BUTTON_2,
        .action = PLATFORM_BTN_DRV_ACTION_REPEAT,
    },
    /* PLATFORM_BUTTON_3 */
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_3_SHORT,
        .button = PLATFORM_BUTTON_3,
        .action = PLATFORM_BTN_DRV_ACTION_SHORT,
    },
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_3_LONG,
        .button = PLATFORM_BUTTON_3,
        .action = PLATFORM_BTN_DRV_ACTION_LONG_RELEASE,
    },
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_3_REPEAT,
        .button = PLATFORM_BUTTON_3,
        .action = PLATFORM_BTN_DRV_ACTION_REPEAT,
    },
#if (APP_BUTTON_MAX >= 4)
    /* PLATFORM_BUTTON_4 */
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_4_LONG,
        .button = PLATFORM_BUTTON_4,
        .action = PLATFORM_BTN_DRV_ACTION_LONG_HELD,
    },
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_4_REPEAT,
        .button = PLATFORM_BUTTON_4,
        .action = PLATFORM_BTN_DRV_ACTION_REPEAT,
    },
    #ifdef APP_TRANSPORT_DETECT_ON
    {
        .id     = PLATFORM_BUTTON_WICED_HCI_DETECT_ON,
        .button = PLATFORM_BUTTON_4,
        .action = PLATFORM_BTN_DRV_ACTION_SHORT,
    },
    #endif  // APP_TRANSPORT_DETECT_ON
#endif  // (APP_BUTTON_MAX >= 4)

#else   // PLATFORM_DEBUG

#if (APP_BUTTON_MAX >= 4)
    /* PLATFORM_BUTTON_4 */
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_1_SHORT,
        .button = PLATFORM_BUTTON_4,
        .action = PLATFORM_BTN_DRV_ACTION_SHORT,
    },
    {
        .id     = PLATFORM_BUTTON_LRAC_BUTTON_1_LONG,
        .button = PLATFORM_BUTTON_4,
        .action = PLATFORM_BTN_DRV_ACTION_LONG_RELEASE,
    },
    {
        .id     = PLATFORM_BUTTON_WICED_HCI_DETECT_ON,
        .button = PLATFORM_BUTTON_4,
        .action = PLATFORM_BTN_DRV_ACTION_REPEAT,
    },
#endif  // (APP_BUTTON_MAX >= 4)

#endif   // PLATFORM_DEBUG
};

/*
 * platform_init
 */
wiced_result_t platform_init(void)
{
    wiced_result_t status = WICED_BT_SUCCESS;

    memset(&platform_cb, 0, sizeof(platform_cb));

    platform_cb.lrac_role = WICED_BT_LRAC_ROLE_UNKNOWN;

#ifdef PLATFORM_DEBUG
    APP_TRACE_DBG("PLATFORM_DEBUG defined. Call wiced_platform_debug_enable()\n");
    wiced_platform_debug_enable();
#endif

    /* Initiate LED. */
#ifndef PLATFORM_LED_DISABLED
    platform_init_led();
#endif

#ifdef CYW9BT_AUDIO
    /*
     * To prevent Audio 'glitch' when the Codec Chip is enabled, we will need to catch the
     * LiteHost VSE (Jitter Buffer events).
     */
    /* Register for VSE */
    bt_hs_spk_control_register_vse_callback(&platform_vse_callback);

#else
    APP_TRACE_ERR("CYW9BT_AUDIO not defined => No Audio Codec\n");
#endif
    return status;
}

/*
 * wass_button_init
 */
wiced_result_t wass_button_init(platform_button_callback_t *p_callback)
{
    bt_hs_spk_button_config_t config;

    platform_cb.p_app_button_callback = p_callback;

    config.p_manager                                = &platform_evb1_button_manager;
    config.p_configuration                          = &platform_evb1_button_manager_configuration;
    config.p_app_buttons                            = platform_evb1_buttons;
    config.number_of_buttons                        = ARRAY_SIZE(platform_evb1_buttons);
    config.p_pre_handler                            = &platform_button_drv_callback;
    config.button_action_config.p_action            = platform_evb1_button_action;
    config.button_action_config.number_of_actions   = ARRAY_SIZE(platform_evb1_button_action);

    return bt_hs_spk_init_button_interface(&config);
}

/*
 * platform_state_get_descr
 */
static char *platform_state_get_descr(platform_global_state_t state)
{
    switch(state)
    {
    case GLOBAL_POWER_UP_STATE: return "GLOBAL_POWER_UP_STATE";
    case GLOBAL_IDLE_STATE: return "GLOBAL_IDLE_STATE";
    case GLOBAL_POWER_DOWN_STATE: return "GLOBAL_POWER_DOWN_STATE";
    }
    return "Unknown";
}

/*
 * platform_state_set
 */
wiced_result_t platform_state_set(platform_global_state_t state)
{
    APP_TRACE_DBG("Platform Set State:%s\n", platform_state_get_descr(state));

    switch(state)
    {
    case GLOBAL_POWER_UP_STATE:
        platform_cb.state = GLOBAL_POWER_UP_STATE;
        break;

    case GLOBAL_IDLE_STATE:
        platform_cb.state = GLOBAL_IDLE_STATE;
        break;

    case GLOBAL_POWER_DOWN_STATE:
        //POWER DOWN Not Implemented
        break;

    default:
        APP_TRACE_ERR("Wrong state %d\n", state);
        break;
    }
    return WICED_BT_SUCCESS;
}

/*
 * platform_state_get
 */
platform_global_state_t platform_state_get(void)
{
    APP_TRACE_DBG("Platform Get State:%s\n", platform_state_get_descr(platform_cb.state));
    return platform_cb.state;
}

/*
 * platform_codec_route_set
 */
wiced_result_t platform_codec_route_set(platform_codec_route_t codec_route)
{
#ifdef CYW9BT_AUDIO
    audio_config_t audio_config = {0};

    switch(codec_route)
    {
    case PLATFORM_CODEC_ROUTE_NONE:
        /* Check if a2dp/audio_insert is opened. */
        bt_hs_spk_audio_audio_manager_stream_stop();

        /* Check if hfp is opened. */
        bt_hs_spk_handsfree_audio_manager_stream_stop();
        break;

    case PLATFORM_CODEC_ROUTE_CVSD:
    case PLATFORM_CODEC_ROUTE_MSBC:
        if (codec_route == PLATFORM_CODEC_ROUTE_CVSD)
            audio_config.sr = AM_PLAYBACK_SR_8K;
        else
            audio_config.sr = AM_PLAYBACK_SR_16K;

        audio_config.channels = 2;
        audio_config.bits_per_sample = DEFAULT_BITSPSAM;
        audio_config.volume = app_volume_get();
        audio_config.sink = bt_hs_spk_get_audio_sink();

        bt_hs_spk_handsfree_audio_manager_stream_start(&audio_config);

#ifdef NREC_ENABLE
        bt_hs_spk_handsfree_audio_manager_nrec_enable();
#endif // NREC_ENABLE
        break;

    default:
        APP_TRACE_ERR("Unknown codec_route:%d\n", codec_route);
        break;
    }
#else
    APP_TRACE_ERR("CYW9BT_AUDIO not defined => No Audio Codec\n");
#endif
    return WICED_BT_SUCCESS;
}

/*
 * platform_led_set
 */
void platform_led_set(platform_led_state_t led_state, uint8_t param)
{
    APP_TRACE_DBG("led_state %d, param %d\n", led_state, param);
#if 1   // todo
    switch (led_state)
    {
    case PLATFORM_LED_POWER_ON:      /* Parameter: Primary/Secondary/Unknown */
        platform_cb.lrac_role = (wiced_bt_lrac_role_t) param;
        break;

    default:
        break;
    }
#else
    switch(led_state)
    {
    case PLATFORM_LED_POWER_ON:      /* Parameter: Primary/Secondary/Unknown */
        switch((wiced_bt_lrac_role_t)param)
        {
            case WICED_BT_LRAC_ROLE_PRIMARY:
                /* Green Led blinks quickly */
#ifndef PLATFORM_LED_DISABLED
                wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
                wiced_led_manager_blink_led(PLATFORM_EVB1_LED_GREEN, 200, 200);
#endif

                platform_cb.lrac_role = WICED_BT_LRAC_ROLE_PRIMARY;
                break;

            case WICED_BT_LRAC_ROLE_SECONDARY:
                /* Red Led blinks quickly */
#ifndef PLATFORM_LED_DISABLED
                wiced_led_manager_disable_led(PLATFORM_EVB1_LED_GREEN);
                wiced_led_manager_blink_led(PLATFORM_EVB1_LED_RED, 200, 200);
#endif

                platform_cb.lrac_role = WICED_BT_LRAC_ROLE_SECONDARY;
                break;
            default:
                /* Both Leds blinks quickly */
#ifndef PLATFORM_LED_DISABLED
                wiced_led_manager_blink_led(PLATFORM_EVB1_LED_RED, 200, 200);
                wiced_led_manager_blink_led(PLATFORM_EVB1_LED_GREEN, 200, 200);
#endif
                break;
        }
        break;

    case PLATFORM_LED_POWER_OFF:         /* No Parameter */
#ifndef PLATFORM_LED_DISABLED
        wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
        wiced_led_manager_disable_led(PLATFORM_EVB1_LED_GREEN);
#endif
        break;

    case PLATFORM_LED_PAIRING:          /* Param: Pairing/NonPairing */
        if (param)
        {
            /* Both Leds blinks quickly */
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_blink_led(PLATFORM_EVB1_LED_RED, 500, 500);
            wiced_led_manager_blink_led(PLATFORM_EVB1_LED_GREEN, 500, 500);
#endif
        }
        else
        {
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_GREEN);
#endif
        }
        break;

    case PLATFORM_LED_LRAC_CONNECTION:   /* Parameter: Connected/Disconnected */
        if (param)
        {
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_GREEN);
            wiced_led_manager_enable_led(PLATFORM_EVB1_LED_RED);
#endif
        }
        else
        {
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_GREEN);
#endif
        }
        break;

    case PLATFORM_LED_A2DP_CONNECTION:   /* Parameter: Connected/Disconnected */
        if (param)
        {
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
            wiced_led_manager_enable_led(PLATFORM_EVB1_LED_GREEN);
#endif
        }
        else
        {
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_GREEN);
#endif
        }
        break;

    case PLATFORM_LED_AVRC_CONNECTION:   /* Parameter: Connected/Disconnected */
        if (param)
        {
        }
        break;

    case PLATFORM_LED_HFP_CONNECTION:    /* Parameter: Connected/Disconnected */
        if (param)
        {
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
            wiced_led_manager_enable_led(PLATFORM_EVB1_LED_GREEN);
#endif
        }
        else
        {
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_GREEN);
#endif
        }
        break;

    case PLATFORM_LED_HFP_VOICE_STATE:   /* Parameter: Voice/NoVoice */
        if (param)
        {
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
            wiced_led_manager_blink_led(PLATFORM_EVB1_LED_GREEN, 200, 800);
#endif
        }
        else
        {
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
            wiced_led_manager_enable_led(PLATFORM_EVB1_LED_GREEN);
#endif
        }
        break;

    case PLATFORM_LED_A2DP_STREAM_STATE: /* Parameter: Streaming/NotStreaming */
        if (param)
        {
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
            wiced_led_manager_blink_led(PLATFORM_EVB1_LED_GREEN, 200, 800);
#endif
        }
        else
        {
#ifndef PLATFORM_LED_DISABLED
            wiced_led_manager_disable_led(PLATFORM_EVB1_LED_RED);
            wiced_led_manager_enable_led(PLATFORM_EVB1_LED_GREEN);
#endif
        }
        break;

    case PLATFORM_LED_CHARGER:
        switch((platform_charger_event_t)param)
        {
        case PLATFORM_CHARGER_INSERTED: /* Charger Inserted */
        case PLATFORM_CHARGER_REMOVED: /* Charger Removed */
        case PLATFORM_CHARGER_CHARGE_COMPLETE:  /* Charge Complete */
            break;

        default:
            break;
        }
        return;

    default:
        APP_TRACE_ERR("Wrong led_state:%d\n", led_state);
        return;
    }
#endif
}

/*
 * platform_audio_side_set
 */
wiced_result_t platform_audio_side_set(platform_audio_side_t audio_side)
{
    switch(audio_side)
    {
    case PLATFORM_AUDIO_SIDE_LEFT:
        /* Do not Swap Left/Right audio. The Codec 'plays' the I2S Left Side */
        lite_host_lrac_enableI2SAudioSwapLR(WICED_FALSE);

        /* Set decoding channel for Left Side (Only support AAC & SBC ) */
        wiced_audio_sink_set_decode_channel(AUDIO_DECODE_MONO_LEFT);
        break;

    case PLATFORM_AUDIO_SIDE_RIGHT:
        /* Swap Left/Right audio. The Codec 'plays' the I2S Right Side */
        lite_host_lrac_enableI2SAudioSwapLR(WICED_TRUE);

        /* Set decoding channel for Right Side (Only support AAC & SBC ) */
        wiced_audio_sink_set_decode_channel(AUDIO_DECODE_MONO_RIGHT);
        break;

    default:
        return WICED_BT_ERROR;
    }
    return WICED_BT_SUCCESS;
}

/*
 * platform_charger_init
 */
wiced_result_t platform_charger_init(platform_charger_callback_t *p_callback)
{
    return WICED_BT_SUCCESS;
}

/*
 * platform_handle_hci_command
 */
void platform_handle_hci_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len)
{
    wiced_result_t status;
    uint8_t button_id;
    uint8_t wiced_hci_status = 1;
    uint8_t audio_file_index;
    uint8_t audio_insert_ext_cmd;
    uint8_t ble_adv_mode;
    uint8_t trace_level;
    uint8_t send_cmd_status = 1;
    uint16_t vsc_opcode;
    uint8_t jitter_buffer_target;
    int8_t elna_gain;
#ifdef VOICE_PROMPT
    uint32_t offset;
    uint32_t length;
#endif
    uint8_t prevent_glitch;

    APP_TRACE_DBG("Opcode:0x%04x Length:%d\n", cmd_opcode, data_len);

    switch(cmd_opcode)
    {
    case HCI_PLATFORM_COMMAND_BUTTON:
        STREAM_TO_UINT8(button_id, p_data);
        APP_TRACE_DBG("Simulate Button:%d\n", button_id);
        platform_cb.p_app_button_callback(button_id, 0);
        wiced_hci_status = 0;
        break;

    case HCI_PLATFORM_COMMAND_AUDIO_INSERT:
        STREAM_TO_UINT8(audio_file_index, p_data);
        if (audio_file_index != 0)
        {
            APP_TRACE_DBG("Simulate AudioInsertStart:%d\n", audio_file_index - 1);
            status = app_audio_insert_start_req(audio_file_index - 1);
        }
        else
        {
            APP_TRACE_DBG("Simulate AudioInsertStop:0\n");
            status = app_audio_insert_stop_req(APP_AUDIO_INSERT_STOP_REQ_REGULAR);
        }
        if (status == WICED_BT_SUCCESS)
            wiced_hci_status = 0;
        else
            wiced_hci_status = 1;
        break;

    case HCI_PLATFORM_COMMAND_AUDIO_INSERT_EXT:
        STREAM_TO_UINT8(audio_insert_ext_cmd, p_data);
        if (audio_insert_ext_cmd <= APP_AUDIO_INSERT_STOP_REQ_MAX)
        {
            APP_TRACE_DBG("Simulate AudioInsertStop:%d\n", audio_insert_ext_cmd);
            status = app_audio_insert_stop_req((app_audio_insert_stop_req_t)audio_insert_ext_cmd);
        }
        else
        {
            APP_TRACE_DBG("Simulate AudioInsertResume\n");
            app_audio_insert_resume();
            status = WICED_BT_SUCCESS;
        }
        if (status == WICED_BT_SUCCESS)
            wiced_hci_status = 0;
        else
            wiced_hci_status = 1;
        break;

#ifndef AMA_ENABLED
    case HCI_PLATFORM_COMMAND_BLE_ADV:
        STREAM_TO_UINT8(ble_adv_mode, p_data);
        APP_TRACE_DBG("Simulate ble_adv mode:%d\n", ble_adv_mode);
        status = app_ble_advert_enable(ble_adv_mode);
        if (status == WICED_BT_SUCCESS)
            wiced_hci_status = 0;
        else
            wiced_hci_status = 1;
        break;
#endif

    case HCI_PLATFORM_COMMAND_LRAC_SWITCH:
        STREAM_TO_UINT8(prevent_glitch, p_data);
        APP_TRACE_DBG("LRAC Switch prevent_glitch:%d\n", prevent_glitch);
        /* Perform Primary/Secondary Switch */
        status = app_lrac_switch_req(prevent_glitch != 0);
        if (status == WICED_BT_SUCCESS)
        {
            wiced_hci_status = 0;
            app_hci_lrac_switch_in_progress_set();
        }
        else
        {
            wiced_hci_status = status;
        }
        break;

    case HCI_PLATFORM_COMMAND_LRAC_TRACE_LEVEL:
        STREAM_TO_UINT8(trace_level, p_data);
        APP_TRACE_DBG("Set LRAC Trace Level:%d\n", trace_level);
        /* Set the new Trace Level */
        wiced_bt_lrac_trace_level_set((wiced_bt_lrac_trace_level_t)trace_level);
        wiced_hci_status = 0;
        break;

    case HCI_PLATFORM_COMMAND_VSC_WRAPPER:
        send_cmd_status = 0;
        STREAM_TO_UINT16(vsc_opcode, p_data);
        data_len -= 2;
        APP_TRACE_DBG("VSC Wrapper opcode:0x%04X length:%d\n", vsc_opcode, data_len);
        status = wiced_bt_dev_vendor_specific_command(vsc_opcode, data_len, p_data,
                platform_vsc_cmd_cplt_callback);
        if ((status != WICED_BT_SUCCESS) &&
            (status != WICED_BT_PENDING))
        {
            APP_TRACE_ERR("wiced_bt_dev_vendor_specific_command failed %d\n", status);
            wiced_hci_status = 1;
            wiced_transport_send_data(HCI_PLATFORM_EVENT_VSC_CMD_CPLT, &wiced_hci_status,
                    sizeof(wiced_hci_status));
        }
        break;

    case HCI_PLATFORM_COMMAND_AP_CONN_CHECK:
        /* 0: connected, 1: not connected */
        wiced_hci_status = !(bt_hs_spk_control_connection_status_check_be_edr(WICED_FALSE));
        break;

    case HCI_PLATFORM_COMMAND_JITTER_TARGET_SET:
        STREAM_TO_UINT8(jitter_buffer_target, p_data);
        APP_TRACE_DBG("Jitter Buffer Target:%d\n", jitter_buffer_target);
        status = app_a2dp_sink_jitter_buffer_target_set(jitter_buffer_target);
        if (status == WICED_BT_SUCCESS)
            wiced_hci_status = 0;
        else
            wiced_hci_status = 1;
        break;

    case HCI_PLATFORM_COMMAND_ELNA_GAIN_SET:
        STREAM_TO_UINT8(elna_gain, p_data);
        APP_TRACE_DBG("eLNA Gain:%d\n", (int)elna_gain);
        status = wiced_bt_lrac_elna_gain_set(elna_gain);
        if (status == WICED_BT_SUCCESS)
            wiced_hci_status = 0;
        else
            wiced_hci_status = 1;
        break;

#ifdef VOICE_PROMPT
    case HCI_PLATFORM_COMMAND_EF_ERASE:/* Embedded Flash Erase */
        STREAM_TO_UINT32(offset, p_data);
        STREAM_TO_UINT32(length, p_data);
        APP_TRACE_DBG("Embedded Flash Erase offset:0x%x length:0x%x\n", offset, length);
        status = wiced_hal_eflash_erase(offset, length);
        if (status == WICED_BT_SUCCESS)
            wiced_hci_status = 0;
        else
            wiced_hci_status = 1;
        break;

    case HCI_PLATFORM_COMMAND_EF_WRITE:/* Embedded Flash Write */
        STREAM_TO_UINT32(offset, p_data);
        data_len -= 4;
        length = data_len;
        APP_TRACE_DBG("Embedded Flash Write offset:0x%x length:0x%x\n", offset, length);
        status = wiced_hal_eflash_write(offset, p_data, length);
        if (status == WICED_BT_SUCCESS)
            wiced_hci_status = 0;
        else
            wiced_hci_status = 1;
        break;
#endif

    default:
        break;
    }

    if (send_cmd_status)
    {
        wiced_transport_send_data(HCI_PLATFORM_EVENT_COMMAND_STATUS,
                &wiced_hci_status, sizeof(wiced_hci_status));
    }
}

/*
 * platform_vsc_cmd_cplt_callback
 */
static void platform_vsc_cmd_cplt_callback(
        wiced_bt_dev_vendor_specific_command_complete_params_t *p_cmd_cplt_param)
{
    if (p_cmd_cplt_param == NULL)
        return;

    APP_TRACE_DBG("opcode:0x%04X hci_status:%d len:%d\n", p_cmd_cplt_param->opcode,
            p_cmd_cplt_param->p_param_buf[0], p_cmd_cplt_param->param_len);

    wiced_transport_send_data(HCI_PLATFORM_EVENT_VSC_CMD_CPLT,
            p_cmd_cplt_param->p_param_buf, p_cmd_cplt_param->param_len);
}

/*
 * platform_button_drv_callback
 */
static wiced_bool_t platform_button_drv_callback(platform_button_t button, button_manager_event_t event, button_manager_button_state_t state, uint32_t repeat)
{
    platform_btn_drv_action_t action = PLATFORM_BTN_DRV_ACTION_INVALID;
    platform_button_id_t button_id = PLATFORM_BUTTON_LAST;
    uint8_t i;

    APP_TRACE_DBG("button:%d event:0x%x state:%d repeat:%d\n",
                  button,
                  (unsigned int)event,
                  (unsigned int)state,
                  repeat);

    /* ignore button if PS-SWITCH is on-going */
    if (app_lrac_switch_is_in_progress())
    {
        APP_TRACE_DBG("PS-SWITCH is on-going, ignore btn\n");
        return WICED_FALSE;
    }

    /* Transfer button event to action. */
    for (i = 0 ; i < (sizeof(platform_button_action_mapping) / sizeof(platform_button_action_mapping[0])) ; i++)
    {
        if ((event == platform_button_action_mapping[i].event) &&
            (state == platform_button_action_mapping[i].state))
        {
            action = platform_button_action_mapping[i].action;
            break;
        }
    }

    if (action == PLATFORM_BTN_DRV_ACTION_INVALID)
    {
        return WICED_TRUE;
    }

    /* Transfer button action to button id.*/
    for (i = 0 ; i < (sizeof(platform_button_id_mapping) / sizeof(platform_button_id_mapping[0])) ; i++)
    {
        if ((button == platform_button_id_mapping[i].button) &&
            (action == platform_button_id_mapping[i].action))
        {
            button_id = platform_button_id_mapping[i].id;
            break;
        }
    }

    if (button_id == PLATFORM_BUTTON_LAST)
    {
        return WICED_TRUE;
    }

#ifdef AMA_ENABLED
    if (platform_cb.lrac_role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        if (!ama_button_pre_handler(button, event, state, repeat))
        {
            return WICED_FALSE;
        }
    }
#endif

    return !(platform_cb.p_app_button_callback(button_id, repeat));
}

#ifdef CYW9BT_AUDIO
/*
 * platform_vse_callback
 */
static wiced_bool_t platform_vse_callback (uint8_t len, uint8_t *p)
{
    uint8_t evt_code;
    uint16_t uipc_event;
    uint8_t opcode;
    uint8_t status;
    int32_t a2dp_stream_id;
    int32_t hfp_stream_id;
    wiced_bt_a2dp_codec_info_t *p_a2dp_codec;
    audio_config_t audio_config;

    /* Check device role. */
    /* NOTE: PRIMARY will return here, it means only SECONDARY handle jitter buffer event in platform_vse_callback */
    switch (platform_cb.lrac_role)
    {
    case WICED_BT_LRAC_ROLE_PRIMARY:
        return WICED_TRUE;

    case WICED_BT_LRAC_ROLE_SECONDARY:
        break;

    default:
        return WICED_FALSE;
    }

    STREAM_TO_UINT8(evt_code, p);     /* Extract VSE Event Code */
    len--;

    /* If it's not a Jitter Buffer VSE */
    if (evt_code != HCI_VSE_JITTER_BUFFER_EVENT)
    {
        return WICED_FALSE;
    }

    STREAM_TO_UINT16(uipc_event, p);/* Extract UIPC code */
    len -= 2;
    STREAM_TO_UINT8(opcode, p);     /* Extract OpCode */
    len--;
    STREAM_TO_UINT8(status, p);     /* Extract Status */
    len--;

    if ((uipc_event == BT_EVT_BTU_IPC_BTM_EVT) &&
        (opcode == AV_SINK_PLAY_STATUS_IND))
    {
        WICED_BT_TRACE("jitter_buffer_event_handler (status : 0x%02X)\n", status);

        if (status == JITTER_NORMAL_STATE)
        {
            /* The JitterBuffer is ready. Start A2DP Codec Stream */
            p_a2dp_codec = app_main_a2dp_sink_codec_get();
            bt_hs_spk_audio_a2dp_codec_info_to_audio_config(p_a2dp_codec, &audio_config);
            audio_config.volume = app_volume_get();
            audio_config.mic_gain = 0;
            audio_config.sink = bt_hs_spk_get_audio_sink();
            bt_hs_spk_audio_audio_manager_stream_start(&audio_config);
        }
        else if (status == JITTER_UNDERRUN_STATE)
        {
            extern uint16_t last_seq_num;
            WICED_BT_TRACE("UNDERRUN(0x%04X)\n", last_seq_num);
#ifdef VOLUME_EFFECT
            /* Mute if underrun, set volume to 0 and inform application UNDERRUN_MUTE event*/
            bt_hs_spk_audio_audio_manager_stream_volume_set(0, VOLUME_EFFECT_UNDERRUN_MUTE);
#endif
        }
    }

    return WICED_FALSE;
}
#endif /* CYW9BT_AUDIO */

/*
 * platform_switch_get
 */
wiced_result_t platform_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
{
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

    if (*p_sync_data_len < sizeof(platform_cb))
    {
        APP_TRACE_ERR("buffer too small (%d/%d)\n", *p_sync_data_len, sizeof(platform_cb));
        return WICED_BT_BADARG;
    }

    memcpy(p_opaque, &platform_cb, sizeof(platform_cb));

    *p_sync_data_len = sizeof(platform_cb);

    return WICED_BT_SUCCESS;
}

/*
 * platform_switch_set
 */
wiced_result_t platform_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    if (p_opaque == NULL)
    {
        APP_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    if (sync_data_len != sizeof(platform_cb))
    {
        APP_TRACE_ERR("bad buffer size (%d/%d)\n", sync_data_len, sizeof(platform_cb));
        return WICED_BT_BADARG;
    }

    memcpy(&platform_cb, p_opaque, sync_data_len);

    return WICED_BT_SUCCESS;
}

#ifndef PLATFORM_LED_DISABLED
/**************************************************************************************************
* Function:     platform_init_led
*
* Abstract:     Initiate LEDs.
*
* Input/Output: None
*
* Return:       None
*
* Notices:
**************************************************************************************************/
static void platform_init_led( void )
{
    uint8_t i;
    uint32_t j;

    for (i = 0 ; i < PLATFORM_LED_MAX ; i++)
    {
        if (wiced_led_manager_init(&platform_evb1_led_config[i]) != WICED_SUCCESS)
        {
            APP_TRACE_ERR("LED init. fail (%d).\n", i);
        }
        else
        {
            wiced_led_manager_disable_led(i);
        }
    }
}
#endif

/*
 * platform_button_emulator
 *
 * Emulate the button event.
 */
void platform_button_emulator(platform_button_id_t button_id, uint32_t repeat_counter)
{
    uint8_t i;
    platform_btn_drv_action_t action = PLATFORM_BTN_DRV_ACTION_INVALID;
    platform_button_t button;

    /* Transfer button id to button action. */
    for (i = 0 ; i < (sizeof(platform_button_id_mapping) / sizeof(platform_button_id_mapping[0])) ; i++)
    {
        if (button_id == platform_button_id_mapping[i].id)
        {
            action = platform_button_id_mapping[i].action;
            button = platform_button_id_mapping[i].button;

            break;
        }
    }

    if (action == PLATFORM_BTN_DRV_ACTION_INVALID)
    {
        return;
    }

    /* Transfer action to event and state. */
    for (i = 0 ; i < (sizeof(platform_button_action_mapping) / sizeof(platform_button_action_mapping[0])) ; i++)
    {
        if (action == platform_button_action_mapping[i].action)
        {
            break;
        }
    }

    if (i >= (sizeof(platform_button_action_mapping) / sizeof(platform_button_action_mapping[0])))
    {
        return;
    }

    bt_hs_spk_button_event_emulator(button,
                                    platform_button_action_mapping[i].event,
                                    platform_button_action_mapping[i].state,
                                    repeat_counter);
}
