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
 * This file implements the LRAC Functions
 */

#include "wiced.h"
#include "wiced_timer.h"
#include "wiced_memory.h"
#include "wiced_bt_lrac.h"
#ifdef AMA_ENABLED
#include <ama.h>
#endif
#include "app_lrac.h"
#include "app_lrac_quality.h"
#include "app_nvram.h"
#include "app_main.h"
#include "app_trace.h"
#include "app_bt.h"
#ifdef APP_OFU_SUPPORT
#include "ofu/app_ofu.h"
#include "ofu/app_ofu_lrac.h"
#endif
#include "app_lrac_switch.h"
#include "app_cpu_clock.h"
#include "app_a2dp_sink.h"
#include "app_hci.h"
#include "wiced_bt_a2dp_defs.h"
#include "app_audio_insert.h"
#include "bt_hs_spk_control.h"
#include "bt_hs_spk_audio.h"
#include "bt_hs_spk_handsfree.h"
#include "bt_hs_spk_handsfree_utils.h"

/*
 * Definitions
 */
#define APP_LRAC_SNIFF_POWER_MGMT_ENABLE            WICED_TRUE
#define APP_LRAC_SNIFF_POWER_MGMT_IDLE_DURATION     480
    /* 480 * 0.625 = 300 ms
     *
     * NOTE: We could adjust this value to have better power saving.
     * But it may cause jitter buffer overrun at the initial stage.
     *
     * The suggestion value of 300ms is from our jitter buffer size(13k).
     * 13000/595 = 21 SBC audio packets, 21 packets playing time = 21*14.5 = 304.5ms.
     * Set 300ms means audio source will buffer about 300ms audio data when we are in long sniff interval.
     * When audio source can start to send those packets, it will not exceed our jitter buffer.
     */

#define APP_LRAC_SWITCH_PREVENT_GLITCH_JITTER_BUFFER_SIZE   4000
    /*
     * Calculated in SBC 44.1 kbps,
     *      There are 640 samples within 1 audio packet (595 bytes).
     *      Time for 1 audio packet = 1 / 44.1 k * 640 = 14.5 ms
     *
     * For 4000 bytes, the preserved time will be
     *      4000 / 595 * 14.5 = 97.5 ms
     *
     * NOTE: The limitation shall meet the setting of JITTER_BUFFER_TARGET or it will be
     * rejected always.
     */

/*
 * LRAC OTA Data Operation Code (app_lrac_data_opcode_t)
 */
enum
{
    APP_LRAC_DATA_OPCODE_OFU = 1,
    APP_LRAC_DATA_OPCODE_BUTTON,
    APP_LRAC_DATA_OPCODE_VOLUME,
    APP_LRAC_DATA_OPCODE_NVRAM_WRITE_REQ,
    APP_LRAC_DATA_OPCODE_NVRAM_WRITE_RSP,
    APP_LRAC_DATA_OPCODE_QUALITY,
    APP_LRAC_DATA_OPCODE_JITTER_BUFFER_TARGET,
    /* Add other LRAC DATA OPCODE Here ... */
};
typedef uint8_t app_lrac_data_opcode_t;

/*
 * structures
 */
typedef struct
{
    app_lrac_callback_t *p_callback;
    wiced_bool_t connected;
    wiced_bool_t connecting;
    wiced_bt_device_address_t bdaddr;
    wiced_bt_device_sec_keys_t  key_data;
    wiced_bool_t initiator;
    wiced_bt_lrac_role_t role;
    wiced_bt_lrac_audio_side_t audio_side;

    /* Switch */
    wiced_bool_t switch_in_progress;
    wiced_bool_t switch_prevent_glitch;

    /* NVRAM */
    wiced_bool_t nvram_update_in_progress;

    /* Local device LRAC version info. */
    wiced_bt_lrac_version_rsp_t version;
} app_lrac_cb_t;

/*
 * External functions
 */

/*
 * Local functions
 */
static void app_lrac_cback(wiced_bt_lrac_event_t event, wiced_bt_lrac_event_data_t *p_data);

static void app_lrac_configuration_start(void);
static void app_lrac_rx_data_handler(uint8_t *p_data, uint16_t length);

static void app_lrac_switch_req_power_mode_callback(wiced_bt_device_address_t bdaddr,
        wiced_bt_dev_power_mgmt_status_t power_mode);
static void app_lrac_switch_rsp_power_mode_callback(wiced_bt_device_address_t bdaddr,
        wiced_bt_dev_power_mgmt_status_t power_mode);
static void app_lrac_switch_sniff_restore(void);

static int app_lrac_version_compare(const wiced_bt_lrac_version_rsp_t *p_peer_version);

/*
 * Global variables
 */
static app_lrac_cb_t app_lrac_cb;

/*
 * lrac_init
 */
wiced_result_t app_lrac_init(app_lrac_callback_t *p_callback)
{
    app_nvram_lrac_info_t lrac_info;
    wiced_result_t status;
    wiced_bt_lrac_lite_host_feature_t lite_host_lrac_feature;
    wiced_bt_lrac_lite_host_debug_mask_t lite_host_debug_mask;

    memset(&app_lrac_cb, 0, sizeof(app_lrac_cb));

    /* Initialize NVRAM */
    status = app_nvram_init();
    if(status != WICED_BT_SUCCESS )
    {
        APP_TRACE_DBG("app_nvram_init failed %d\n", status);
        return status;
    }

    /* Initialize LRAC Library */
    status = wiced_bt_lrac_init(app_lrac_cback);
    if (status != WICED_BT_SUCCESS)
        return status;

    /* Load LRAC library version. */
    app_lrac_cb.version.major   = WICED_BT_LRAC_VERSION_MAJOR;
    app_lrac_cb.version.minor   = WICED_BT_LRAC_VERSION_MINOR;
    app_lrac_cb.version.build   = WICED_BT_LRAC_VERSION_BUILD;

    APP_TRACE_DBG("LRAC Library version:%d.%d.%d\n",
                  app_lrac_cb.version.major,
                  app_lrac_cb.version.minor,
                  app_lrac_cb.version.build);

    /* Enable LRAC Library debug traces. This will have to be removed for the final application */
    wiced_bt_lrac_trace_level_set(WICED_BT_LRAC_TRACE_LEVEL_DEBUG);

    /* Check if LRAC Info exists in NVRAM */
    status = app_nvram_lrac_info_read(&lrac_info);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_DBG("Create default NVRAM LRAC\n");
        /* If it does not exist, create a dummy entry */
        memset(&lrac_info, 0, sizeof(lrac_info));
        /* Role and Audio Side will be negotiated later */
        lrac_info.role = WICED_BT_LRAC_ROLE_UNKNOWN;
        /* The Audio side may come from a GPIO */
        lrac_info.audio_side = WICED_BT_LRAC_AUDIO_SIDE_UNKNOWN;
        status = app_nvram_lrac_info_write(&lrac_info);
        if (status != WICED_BT_SUCCESS)
        {
            return status;
        }
    }
    APP_TRACE_DBG("Config Role:%s(%d) AudioSide:%s(%d)\n",
            app_lrac_role_get_desc(lrac_info.role), lrac_info.role,
            app_lrac_audio_side_get_desc(lrac_info.audio_side), lrac_info.audio_side);

    /* Check if peer LRAC Device is known */
    app_nvram_lrac_bdaddr_get(app_lrac_cb.bdaddr);

    APP_TRACE_DBG("Peer LRAC Device: %B\n", app_lrac_cb.bdaddr);

    /* Load key data. */
    if (bt_hs_spk_control_misc_data_content_check((uint8_t *) app_lrac_cb.bdaddr, sizeof(wiced_bt_device_address_t)))
    {
        app_nvram_pairing_info_read(&app_lrac_cb.key_data);
    }

    app_lrac_cb.p_callback = p_callback;
    app_lrac_cb.role = lrac_info.role;
    app_lrac_cb.audio_side = lrac_info.audio_side;

    /* Enable Sniff Power Management */
    if (app_lrac_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        wiced_bt_lrac_sniff_power_mgmt_enable(APP_LRAC_SNIFF_POWER_MGMT_ENABLE,
                APP_LRAC_SNIFF_POWER_MGMT_IDLE_DURATION, NULL);
    }

    /* Set LiteHost LRAC features. Notes:
     *    wiced_bt_lrac_lite_host_feature_get function gets the current features setting,
     *    wiced_bt_lrac_lite_host_feature_set function sets the features
     */
    lite_host_lrac_feature = wiced_bt_lrac_lite_host_feature_get();
#if 0
    lite_host_lrac_feature |= LRAC_RESTART_ON_UNDERRUN;
    /* Enable LiteHost if required only (not 100% threat safe) */
    lite_host_lrac_feature |= LRAC_ENABLE_DEBUG_TO_WICED;
#endif
    lite_host_lrac_feature = wiced_bt_lrac_lite_host_feature_set(lite_host_lrac_feature);
    APP_TRACE_DBG("Current Litehost LRAC Features Setting:0x%x\n", lite_host_lrac_feature);

    /* Enable LiteHost Packet Replacement debug statistics */
    lite_host_debug_mask = wiced_bt_lrac_lite_host_debug_mask_get();
    lite_host_debug_mask |= LRAC_DEBUG_REPLACE_STATS;
    wiced_bt_lrac_lite_host_debug_mask_set(lite_host_debug_mask);

    return WICED_BT_SUCCESS;
}

/*
 * app_lrac_cback
 * This function handles the LRAC Events from the LRAC library
 */
static void app_lrac_cback(wiced_bt_lrac_event_t event, wiced_bt_lrac_event_data_t *p_data)
{
    app_nvram_lrac_info_t lrac_info;
    wiced_result_t status = WICED_BT_SUCCESS;
    app_lrac_event_data_t event_data;
    int version_compare;

    switch(event)
    {
    /* Connection to the peer LRAC device is established */
    case WICED_BT_LRAC_EVENT_CONNECTED:
        APP_TRACE_DBG("LRAC Connected status:%d BdAddr:%B\n", p_data->connected.status,
                p_data->connected.bdaddr);
        if (p_data->connected.status == WICED_BT_SUCCESS)
        {
            if (app_lrac_cb.connected)
            {
                /* duplicate connected event */
                APP_TRACE_ERR("Duplicate LRAC_CONNECTED event:%B\n", p_data->connected.bdaddr);
            }
            else
            {
                app_lrac_cb.connected = WICED_TRUE;
                /* Save this Bdaddr in the Dedicated Pairing Info NVRAM */
                status = app_nvram_lrac_bdaddr_set(p_data->connected.bdaddr);
                if (status != WICED_BT_SUCCESS)
                    APP_TRACE_ERR("app_nvram_lrac_bdaddr_set failed %d\n", status);
                memcpy(app_lrac_cb.bdaddr, p_data->connected.bdaddr, sizeof(app_lrac_cb.bdaddr));

                /* Send the LRAC configuration Request
                 * NOTE: it will be triggered on PRI device instead of initiator */
                app_lrac_configuration_start();
            }
        }
        else
        {
            app_lrac_cb.connected = WICED_FALSE;
            app_lrac_cb.connecting = WICED_FALSE;
            app_lrac_cb.initiator = WICED_FALSE;
            event_data.connected.lrac_role = WICED_BT_LRAC_ROLE_UNKNOWN;
            event_data.connected.audio_side = WICED_BT_LRAC_AUDIO_SIDE_UNKNOWN;
            event_data.connected.status = p_data->connected.status;
            memcpy(event_data.connected.bdaddr, p_data->connected.bdaddr,
                    sizeof(event_data.connected.bdaddr));
            app_lrac_cb.p_callback(APP_LRAC_CONNECTED, &event_data);
        }
        break;

    /* Connection to the peer LRAC device is released */
    case WICED_BT_LRAC_EVENT_DISCONNECTED:
        APP_TRACE_DBG("LRAC Disconnected reason:%d\n", p_data->disconnected.reason);
        app_lrac_cb.connected = WICED_FALSE;
        app_lrac_cb.initiator = WICED_FALSE;
        app_lrac_cb.connecting = WICED_FALSE;
        app_lrac_cb.switch_in_progress = WICED_FALSE;
        app_lrac_cb.p_callback(APP_LRAC_DISCONNECTED, &event_data);
        break;

    case WICED_BT_LRAC_EVENT_CONFIG_REQ:
        APP_TRACE_DBG("LRAC ConfigReq Role:%s(%d) AudioSide:%s(%d)\n",
                app_lrac_role_get_desc(p_data->config_req.role), p_data->config_req.role,
                app_lrac_audio_side_get_desc(p_data->config_req.audio_side),
                p_data->config_req.audio_side);
        /* Check if LRAC Info exists in NVRAM */
        status = app_nvram_lrac_info_read(&lrac_info);
        if (status != WICED_BT_SUCCESS)
        {
            APP_TRACE_ERR("No LRAC NVRAM Info!!!\n");
        }
        lrac_info.role = p_data->config_req.role;
        APP_TRACE_DBG("TODO: Check if AudioSide is compatible with GPIO\n");
        lrac_info.audio_side = p_data->config_req.audio_side;
        status = app_nvram_lrac_info_write(&lrac_info);
        if (status != WICED_BT_SUCCESS)
        {
            APP_TRACE_ERR("app_nvram_lrac_info_write failed\n");
        }
        wiced_bt_lrac_configure_rsp(status);
        app_lrac_cb.role = lrac_info.role;

        /* Read the peer Version */
        status = wiced_bt_lrac_version_req();
        if (status != WICED_BT_SUCCESS)
            APP_TRACE_ERR("wiced_bt_lrac_version_req failed\n");
        break;

    case WICED_BT_LRAC_EVENT_CONFIG_RSP:
        APP_TRACE_DBG("LRAC ConfigRsp Status:%d\n", p_data->config_rsp.status);
        if (p_data->config_rsp.status == WICED_BT_SUCCESS)
        {
            /* Check if LRAC Info exists in NVRAM */
            status = app_nvram_lrac_info_read(&lrac_info);
            if (status != WICED_BT_SUCCESS)
            {
                APP_TRACE_ERR("No LRAC NVRAM Info!!!\n");
            }
            lrac_info.role = app_lrac_cb.role;
            lrac_info.audio_side = app_lrac_cb.audio_side;
            status = app_nvram_lrac_info_write(&lrac_info);
            if (status != WICED_BT_SUCCESS)
            {
                APP_TRACE_ERR("app_nvram_lrac_info_write failed\n");
            }

            /* Read the peer Version */
            status = wiced_bt_lrac_version_req();
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("wiced_bt_lrac_version_req failed\n");
        }
        break;

    /* The peer LRAC Device (Secondary) Started A2DP Eavesdropping */
    case WICED_BT_LRAC_EVENT_A2DP_START:
        APP_TRACE_DBG("LRAC A2DP Started Status:%d Sync:%d\n", p_data->a2dp_start.status,
                p_data->a2dp_start.sync);
        event_data.a2dp_started.status = p_data->a2dp_start.status;
        event_data.a2dp_started.sync = p_data->a2dp_start.sync;
        memcpy(&event_data.a2dp_started.codec_info,
                &p_data->a2dp_start.codec_info,
                sizeof(event_data.a2dp_started.codec_info));
        app_lrac_cb.p_callback(APP_LRAC_A2DP_STARTED, &event_data);
        break;

    /* The peer LRAC Device (Secondary) Stopped A2DP Eavesdropping */
    case WICED_BT_LRAC_EVENT_A2DP_STOP:
        APP_TRACE_DBG("LRAC A2DP Stopped Status:%d\n", p_data->a2dp_stop.status);
        event_data.a2dp_stopped.status = p_data->a2dp_stop.status;
        app_lrac_cb.p_callback(APP_LRAC_A2DP_STOPPED, &event_data);
        break;

    case WICED_BT_LRAC_EVENT_HFP_START:
        APP_TRACE_DBG("LRAC HFP Started Status:%d wide_band:%d\n", p_data->hfp_start.status,
                p_data->hfp_start.wide_band);
        event_data.hfp_started.status = p_data->hfp_start.status;
        event_data.hfp_started.wide_band = p_data->hfp_start.wide_band;
        app_lrac_cb.p_callback(APP_LRAC_HFP_STARTED, &event_data);
        break;

    case WICED_BT_LRAC_EVENT_HFP_STOP:
        APP_TRACE_DBG("LRAC HFP Stopped Status:%d\n", p_data->hfp_stop.status);
        event_data.hfp_stopped.status = p_data->hfp_stop.status;
        app_lrac_cb.p_callback(APP_LRAC_HFP_STOPPED, &event_data);
        break;

    case WICED_BT_LRAC_EVENT_VERSION_RSP:
        APP_TRACE_DBG("LRAC peer_version:%d.%d.%d local_version:%d.%d.%d\n",
                p_data->version_rsp.major, p_data->version_rsp.minor, p_data->version_rsp.build,
                app_lrac_cb.version.major,
                app_lrac_cb.version.minor,
                app_lrac_cb.version.build);
        /* Compare the Local and Peer Versions */
        version_compare = app_lrac_version_compare(&p_data->version_rsp);
        if (version_compare > 0)
        {
#ifdef APP_OFU_SUPPORT
#ifdef OTA_FW_UPGRADE
            APP_TRACE_DBG("Local Version is higher than the Peer Version. Start OFU\n");
            status = app_ofu_lrac_start(512);
            if (status != WICED_BT_SUCCESS)
#endif
#else
            APP_TRACE_ERR("OFU Not Supported. Disconnect LRAC Device\n");
#endif
            {
                APP_TRACE_ERR("app_ofu_lrac_start failed\n");
                wiced_bt_lrac_disconnect();
            }
        }
        else if (version_compare < 0)
        {
            APP_TRACE_DBG("Local Version is lower than the Peer Version. Peer will Start OFU\n");
        }
        else
        {
            APP_TRACE_DBG("Same Versions. No OFU needed\n");
            /* Check if NVRAM entries must be updated */
            app_lrac_nvram_update_req();
        }

        /* LRAC is connected */
        app_lrac_cb.connecting = WICED_FALSE;
        event_data.connected.status = WICED_BT_SUCCESS;
        memcpy(event_data.connected.bdaddr, app_lrac_cb.bdaddr,
                sizeof(event_data.connected.bdaddr));
        event_data.connected.lrac_role = app_lrac_cb.role;
        event_data.connected.audio_side = app_lrac_cb.audio_side;
        app_lrac_cb.p_callback(APP_LRAC_CONNECTED, &event_data);
        break;

    case WICED_BT_LRAC_EVENT_RX_DATA:
        /*
        APP_TRACE_DBG("RX_DATA length:%d Data:%02X %02X...\n", p_data->rx_data.length,
                p_data->rx_data.p_data[0], p_data->rx_data.p_data[1]);
        */
        app_lrac_rx_data_handler(p_data->rx_data.p_data, p_data->rx_data.length);
        break;

    case WICED_BT_LRAC_EVENT_AUDIO_INSERT_START_RSP:
    case WICED_BT_LRAC_EVENT_AUDIO_INSERT_START_REQ:
    case WICED_BT_LRAC_EVENT_AUDIO_INSERT_STOP_RSP:
        /* Audio Insert events are handled by a dedicated function/file */
        app_audio_insert_lrac_event_handler(event, p_data);
        break;

    case WICED_BT_LRAC_EVENT_SWITCH_REQ:
        APP_TRACE_DBG("SWITCH_REQ new_role:%s(%d) prevent_glitch:%d\n",
                app_lrac_role_get_desc(p_data->switch_req.new_role), p_data->switch_req.new_role,
                p_data->switch_req.prevent_glitch);
        APP_TRACE_DBG("Current Jitter Buffer Level:%d\n",
                wiced_bt_lrac_audio_jitter_buffer_level_get());

        /* set before calling app_lrac_switch_is_ready */
        app_lrac_cb.switch_prevent_glitch = p_data->switch_req.prevent_glitch;

        /* Check if Ready to Switch (no pending actions) */
        if (app_lrac_switch_is_ready() == WICED_FALSE)
        {
            APP_TRACE_ERR("Switch not allowed\n");
            status = wiced_bt_lrac_switch_rsp(WICED_NOT_AVAILABLE, WICED_FALSE);
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("wiced_bt_lrac_switch_rsp failed %d\n", status);
        }
        else
        {
            status = WICED_BT_SUCCESS;
            if (app_lrac_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
            {
                /* Ask the Main app if a Phone is connected */
                if (bt_hs_spk_control_connection_status_check_be_edr(WICED_FALSE))
                {   /* At least one AP link exists. */
                    /* Disable Sniff mode to avoid the AP link(s) becomes sniff during the switch process. */
                    bt_hs_spk_control_acl_link_policy_sniff_mode_set(NULL, WICED_FALSE);

                    /* Set the existent AP link(s) to Active Mode. */
                    status = bt_hs_spk_control_bt_power_mode_set(WICED_TRUE, NULL, app_lrac_switch_rsp_power_mode_callback);

                    switch (status)
                    {
                    /* AP link(s) is already in active mode. */
                    case WICED_BT_SUCCESS:
                        break;

                    /* AP link(s) is waiting for changing power mode to active mode. */
                    case WICED_BT_PENDING:
                        /* The callback will be called when the AP link(s) becomes Active */
                        app_lrac_cb.switch_in_progress = WICED_TRUE;
                        return;

                    default:
#ifdef DISABLE_SNIFF_MODE_DURING_A2DP
                        /* only enable sniff mode if no streaming */
                        if (bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED)
#endif
                        {
                            /* Enable Sniff mode */
                            bt_hs_spk_control_acl_link_policy_sniff_mode_set(NULL, WICED_TRUE);
                        }

                        app_lrac_cb.switch_in_progress = WICED_FALSE;
                        APP_TRACE_ERR("power_mode_set_active failed (%d)\n", status);
                        return;
                    }
                }
                else
                {   /* No AP link(s) */
                    /* Exit sniff power_mgmt - lower layer will wait for exit automatically  */
                    wiced_bt_lrac_sniff_power_mgmt_exit(NULL);
                }
            }
            if (status == WICED_BT_SUCCESS)
            {
                app_lrac_cb.switch_in_progress = WICED_TRUE;
            }

            /* Send Switch Response */
            status = wiced_bt_lrac_switch_rsp(status, app_lrac_cb.switch_prevent_glitch);
            if (status == WICED_BT_SUCCESS)
            {
                /* Disable discovery */
                app_main_lrac_switch_backup_bt_visibility();
                bt_hs_spk_button_set_discovery(WICED_FALSE);

#ifdef AMA_ENABLED
                ama_suspend(1);
#endif

                /* Increase the CPU Clock to reduce the PS-Switch time */
                app_cpu_clock_increase(APP_CPU_CLOCK_REQUESTER_PS_SWITCH);
            }
            else
            {
                /* Ask the Main app if a Phone is connected */
                if (bt_hs_spk_control_connection_status_check_be_edr(WICED_FALSE))
                {   /* At least one AP link exists. */
#ifdef DISABLE_SNIFF_MODE_DURING_A2DP
                    /* only enable sniff mode if no streaming */
                    if (bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED)
#endif
                    {
                        /* Enable Sniff mode */
                        bt_hs_spk_control_acl_link_policy_sniff_mode_set(NULL, WICED_TRUE);
                    }
                }
                APP_TRACE_ERR("wiced_bt_lrac_switch_rsp failed %d\n", status);
                app_lrac_cb.switch_in_progress = WICED_FALSE;
            }
        }
        break;

    case WICED_BT_LRAC_EVENT_SWITCH_DATA_REQ:
        WICED_BT_TRACE("SWITCH_DATA_REQ\n");
        status = app_lrac_switch_data_collect();
        if (status != WICED_BT_SUCCESS)
        {
            APP_TRACE_ERR("app_lrac_switch_data_collect failed %d\n", status);
            status = wiced_bt_lrac_switch_abort_req();
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("wiced_bt_lrac_switch_abort_req failed %d\n", status);
        }
        break;

    case WICED_BT_LRAC_EVENT_SWITCH_DATA_IND:
        /*
        APP_TRACE_DBG("SWITCH_DATA_IND tag:%d lenght:%d\n",
              p_data->switch_data_ind.data_tag, p_data->switch_data_ind.length);
        */
        status = app_lrac_switch_data_apply(p_data->switch_data_ind.data_tag,
                p_data->switch_data_ind.p_data, p_data->switch_data_ind.length);
        if (status != WICED_BT_SUCCESS)
        {
            APP_TRACE_ERR("app_lrac_switch_data_apply failed %d\n", status);
            status = wiced_bt_lrac_switch_abort_req();
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("wiced_bt_lrac_switch_abort_req failed %d\n", status);
        }
        break;

    case WICED_BT_LRAC_EVENT_SWITCH_RSP:
        WICED_BT_TRACE("SWITCH_RSP status:%d new_role:%s(%d)\n",
                p_data->switch_rsp.status,
                app_lrac_role_get_desc(p_data->switch_rsp.new_role),
                p_data->switch_rsp.new_role);
        app_lrac_cb.switch_in_progress = WICED_FALSE;
        /* Restore the CPU Clock */
        app_cpu_clock_decrease(APP_CPU_CLOCK_REQUESTER_PS_SWITCH);
        /* Re-enter sniff power_mgmt */
        if (app_lrac_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            wiced_bt_lrac_sniff_power_mgmt_enter(NULL);
        }
        app_lrac_switch_sniff_restore();
        app_hci_lrac_switch_result(p_data->switch_rsp.status, 0, 0);

        event_data.switch_completed.status = p_data->switch_rsp.status;
        event_data.switch_completed.new_role = p_data->switch_rsp.new_role;
        event_data.switch_completed.fatal_error = WICED_FALSE;
        app_lrac_cb.p_callback(APP_LRAC_SWITCH_COMPLETED, &event_data);
        break;

    case WICED_BT_LRAC_EVENT_SWITCH_ABORTED:
        APP_TRACE_ERR("SWITCH_ABORTED status:%d local_abort:%d fatal_error:%d\n",
                p_data->switch_aborted.status,
                p_data->switch_aborted.local_abort,
                p_data->switch_aborted.fatal_error);
        app_lrac_cb.switch_in_progress = WICED_FALSE;
        /* Restore the CPU Clock */
        app_cpu_clock_decrease(APP_CPU_CLOCK_REQUESTER_PS_SWITCH);
        /* Re-enter sniff power_mgmt */
        if (app_lrac_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
        {
            wiced_bt_lrac_sniff_power_mgmt_enter(NULL);
        }
        app_lrac_switch_sniff_restore();
        app_hci_lrac_switch_result(p_data->switch_aborted.status,
                p_data->switch_aborted.local_abort,
                p_data->switch_aborted.fatal_error);

        if (p_data->switch_aborted.status == WICED_BT_LRAC_SWITCH_FORCE_ABORT)
        {
            /* Set specific error status for force abort */
            event_data.switch_completed.status = WICED_UNFINISHED;
        }
        else
        {
            event_data.switch_completed.status = WICED_BT_ERROR;
        }
        event_data.switch_completed.new_role = app_lrac_cb.role;
        event_data.switch_completed.fatal_error = p_data->switch_aborted.fatal_error;
        app_lrac_cb.p_callback(APP_LRAC_SWITCH_COMPLETED, &event_data);
        break;

    case WICED_BT_LRAC_EVENT_I2S_STARTED:
        break;

    case WICED_BT_LRAC_EVENT_INIT_STATUS:
        APP_TRACE_DBG("LRAC INIT_STATUS %d\n", p_data->init_status.status);
        if (p_data->init_status.status != HCI_SUCCESS)
        {
            APP_TRACE_ERR("lrac_lib init event: status:%d\n", p_data->init_status.status);
        }
        break;

    case WICED_BT_LRAC_EVENT_AUDIO_GLITCH:
        app_lrac_quality_audio_gitch_handler(&p_data->audio_glitch);
        break;

    case WICED_BT_LRAC_EVENT_JITTER_BUFFER:
        app_lrac_quality_jitter_buffer_handler(&p_data->jitter_buffer);
        break;

    case WICED_BT_LRAC_EVENT_RSSI:
        app_lrac_quality_rssi_handler(&p_data->rssi);
        break;

    case WICED_BT_LRAC_EVENT_FW_STATISTICS:
        app_lrac_quality_fw_statistics_handler(&p_data->fw_statistics);
        break;

    default:
        APP_TRACE_ERR("unknown event:%d\n", event);
        break;
    }
}

/*
 * app_lrac_nvram_update_req
 */
void app_lrac_nvram_update_req(void)
{
    uint8_t tx_data[sizeof(uint8_t) + (sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT)];
    wiced_bt_device_link_keys_t *p_link_keys = (wiced_bt_device_link_keys_t *) &tx_data[sizeof(uint8_t)];
    wiced_result_t status;

    APP_TRACE_DBG("app_lrac_nvram_update_req (%d)\n", app_lrac_cb.role);

    /* Check LRAC role. */
    if (app_lrac_cb.role != WICED_BT_LRAC_ROLE_PRIMARY)
    {
        return;
    }

    /* NVRAM can be updated to the peer device if connected only */
    if (app_lrac_cb.connected == WICED_FALSE)
    {
        APP_TRACE_DBG("LRAC device not connected\n");
        return;
    }

    /* If NVRAM processing ongoing */
    if (app_lrac_cb.nvram_update_in_progress)
    {
        APP_TRACE_DBG("already processing\n");
        return;
    }

    /* Get the NVRAM data to be updated */
    memcpy((void *) p_link_keys,
           (void *) bt_hs_spk_control_link_keys_get(),
           sizeof(wiced_bt_device_link_keys_t) * BT_HS_SPK_CONTROL_LINK_KEY_COUNT);

    /* Write to peer device. */
    tx_data[0] = APP_LRAC_DATA_OPCODE_NVRAM_WRITE_REQ;

    status = wiced_bt_lrac_tx_data(tx_data, sizeof(tx_data));
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_bt_lrac_tx_data failed %d\n", status);
    }
    else
    {
        app_lrac_cb.nvram_update_in_progress = WICED_TRUE;
    }
}

/*
 * app_lrac_nvram_send_write_rsp
 */
#if 1
static void app_lrac_nvram_send_write_rsp(wiced_result_t rsp_status)
{
    uint8_t tx_data[sizeof(uint8_t) + sizeof(uint16_t)];
    uint8_t *p = &tx_data[0];
    wiced_result_t status;

    UINT8_TO_STREAM(p, APP_LRAC_DATA_OPCODE_NVRAM_WRITE_RSP);
    UINT16_TO_STREAM(p, rsp_status);

    status = wiced_bt_lrac_tx_data(tx_data, p - tx_data);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_bt_lrac_tx_data failed %d\n", status);
    }
}
#else
static void app_lrac_nvram_send_write_rsp(uint16_t nvram_id, wiced_result_t rsp_status)
{
    uint8_t tx_data[sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint16_t)];
    uint8_t *p = &tx_data[0];
    wiced_result_t status;

    UINT8_TO_STREAM(p, APP_LRAC_DATA_OPCODE_NVRAM_WRITE_RSP);
    UINT16_TO_STREAM(p, nvram_id);
    UINT16_TO_STREAM(p, rsp_status);

    status = wiced_bt_lrac_tx_data(tx_data, p - tx_data);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_bt_lrac_tx_data failed %d\n", status);
    }
}
#endif

/*
 * app_lrac_connect
 * This function is used to Connect/Disconnect a connection to a peer LRAC device.
 */
void app_lrac_connect(void)
{
    wiced_result_t status;

    APP_TRACE_DBG("app_lrac_connect (%B, %d, %d, %d)\n",
                  app_lrac_cb.bdaddr,
                  app_lrac_cb.connected,
                  app_lrac_cb.connecting,
                  app_lrac_cb.initiator);

    /* If the LRAC connection is established or the connection process is ongoing, release it */
    if ((app_lrac_cb.connected) ||
        (app_lrac_cb.connecting))
    {
        return;
    }

    /* Check if LRAC Info exists in NVRAM */
    if (bt_hs_spk_control_misc_data_content_check((uint8_t *) app_lrac_cb.bdaddr,
                                                  sizeof(wiced_bt_device_address_t)) == WICED_FALSE)
    {
        return;
    }

    /* Open the LRAC connection */
    status = wiced_bt_lrac_connect(app_lrac_cb.bdaddr);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_bt_lrac_connect failed %d\n", status);
        return;
    }

    app_lrac_cb.initiator = WICED_TRUE;
    app_lrac_cb.connecting = WICED_TRUE;
}

/*
 * app_lrac_disconnect
 * This function is used to Disconnect a connection to a peer LRAC device.
 */
void app_lrac_disconnect(void)
{
    wiced_result_t status;

    /* If the LRAC connection is established, release it */
    if (app_lrac_cb.connected)
    {
        status = wiced_bt_lrac_disconnect();
        if (status != WICED_BT_SUCCESS)
            APP_TRACE_ERR("wiced_bt_lrac_disconnect failed:%d\n", status);
    }
    else
    {
        APP_TRACE_ERR("not connected\n");
    }
}

/*
 * app_lrac_is_connected
 */
wiced_bool_t app_lrac_is_connected(void)
{
    return app_lrac_cb.connected;
}

/*
 * app_lrac_connecting
 */
void app_lrac_connection_status_handler(uint8_t *p_features, wiced_bool_t is_connected,
        uint16_t handle, uint8_t reason)
{
    APP_TRACE_DBG("LRAC_CONN_STATUS is_connected:%d handle:%d reason:%d\n", is_connected,
            handle, reason);

    /* baseband is connected */
    if (is_connected)
    {
        if (!app_lrac_cb.initiator)
        {
            /* set to TRUE to prevent the duplicate initiator if trigger again before LRAC_CONNECTED */
            APP_TRACE_DBG("LRAC CONNECTING\n");
            app_lrac_cb.connecting = WICED_TRUE;
        }
    }
    else
    {
        app_lrac_cb.connecting = WICED_FALSE;
        app_lrac_cb.connected = WICED_FALSE;
    }
}

/*
 * app_lrac_version_compare
 * Compare the LRAC peer Version with the local version
 * Returns:
 *   1: If the local version is higher than the peer version => We need to upgrade the peer device
 *   0: If the version are the same => No upgrade needed
 *  -1: If the local version is lower than the peer version => Peer device will upgrade us
 */
static int app_lrac_version_compare(const wiced_bt_lrac_version_rsp_t *p_peer_version)
{
    /* First, check Major Version */
    if (p_peer_version->major < app_lrac_cb.version.major)
        return 1;
    if (p_peer_version->major > app_lrac_cb.version.major)
        return -1;
    /* Secondly, check Minor Version */
    if (p_peer_version->minor < app_lrac_cb.version.minor)
        return 1;
    if (p_peer_version->minor > app_lrac_cb.version.minor)
        return -1;
    /* Lastly, check Build Version */
    if (p_peer_version->build < app_lrac_cb.version.build)
        return 1;
    if (p_peer_version->build > app_lrac_cb.version.build)
        return -1;

    /* Version are equal */
    return 0;
}

/*
 * app_lrac_a2dp_start_req
 */
wiced_result_t app_lrac_a2dp_start_req(wiced_bool_t sync)
{
    bt_hs_spk_audio_info_t a2dp_info;
    wiced_result_t status;

    APP_TRACE_DBG("app_lrac_a2dp_start_req(%d)\n", sync);

    /* If PS-SWITCH is in progress, to abort PS-SWITCH procedure
     * and do eavesdropping recover procedure when PS-SWITCH is
     * aborted */
    if (app_lrac_cb.switch_in_progress)
    {
        APP_TRACE_ERR("Force abort PS-SWITCH procedure\n");
        wiced_bt_lrac_switch_force_abort_req_before_start();
        return WICED_WAIT_ABORTED;
    }

    if (bt_hs_spk_audio_a2dp_info_get(&a2dp_info) == WICED_TRUE)
    {
        status = wiced_bt_lrac_a2dp_start_req(a2dp_info.bdaddr, a2dp_info.handle,
                &a2dp_info.codec, a2dp_info.cp_type, sync);

        if (status == WICED_BT_SUCCESS)
        {
            /* Send the local volume for A2DP stream to the Secondary. */
            app_lrac_volume_send(bt_hs_spk_audio_utils_abs_volume_to_am_volume(bt_hs_spk_audio_volume_get()), VOLUME_EFFECT_NONE);
        }

        return status;
    }

    return WICED_NOT_CONNECTED;
}

/*
 * app_lrac_a2dp_stop_req
 */
wiced_result_t app_lrac_a2dp_stop_req(void)
{
    APP_TRACE_DBG("app_lrac_a2dp_stop_req\n");

    /* If PS-SWITCH is in progress, to abort PS-SWITCH procedure
     * and do eavesdropping recover procedure when PS-SWITCH is
     * aborted */
    if (app_lrac_cb.switch_in_progress)
    {
        APP_TRACE_ERR("Force abort PS-SWITCH procedure\n");
        wiced_bt_lrac_switch_force_abort_req_before_start();
        return WICED_WAIT_ABORTED;
    }

    return wiced_bt_lrac_a2dp_stop_req();
}

/*
 * app_lrac_hfp_start_req
 */
wiced_result_t app_lrac_hfp_start_req(void)
{
    bt_hs_spk_handsfree_active_call_session_info_t call_info;
    wiced_result_t status;

    APP_TRACE_DBG("app_lrac_hfp_start_req\n");

    /* If PS-SWITCH is in progress, to abort PS-SWITCH procedure
     * and do eavesdropping recover procedure when PS-SWITCH is
     * aborted */
    if (app_lrac_cb.switch_in_progress)
    {
        APP_TRACE_ERR("Force abort PS-SWITCH procedure\n");
        wiced_bt_lrac_switch_force_abort_req_before_start();
        return WICED_WAIT_ABORTED;
    }

    if (bt_hs_spk_handsfree_active_call_session_info_get(&call_info) == WICED_TRUE)
    {
        /* For HFP eavesdropping, we shall send the voice volume to the Secondary first. */
        app_lrac_volume_send(bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(bt_hs_spk_handsfree_volume_get()), VOLUME_EFFECT_NONE);

        status = wiced_bt_lrac_hfp_start_req(call_info.bdaddr, call_info.sco_idx, call_info.wide_band);

        return status;
    }

    return WICED_NOT_CONNECTED;
}

/*
 * app_lrac_hfp_stop_req
 */
wiced_result_t app_lrac_hfp_stop_req(void)
{
    /* If PS-SWITCH is in progress, to abort PS-SWITCH procedure
     * and do eavesdropping recover procedure when PS-SWITCH is
     * aborted */
    if (app_lrac_cb.switch_in_progress)
    {
        APP_TRACE_ERR("Force abort PS-SWITCH procedure\n");
        wiced_bt_lrac_switch_force_abort_req_before_start();
        return WICED_WAIT_ABORTED;
    }

    return wiced_bt_lrac_hfp_stop_req();
}

/*
 * app_lrac_button_send
 */
wiced_result_t app_lrac_button_send(uint8_t button_id,uint32_t repeat_counter)
{
    uint8_t tx_data[1 + 1 + 4]; // opcode + button_id + repeat_counter
    uint8_t *p = tx_data;

    APP_TRACE_DBG("ButtonId:%d\n", button_id);

    UINT8_TO_STREAM(p, APP_LRAC_DATA_OPCODE_BUTTON);
    UINT8_TO_STREAM(p, button_id);
    UINT32_TO_STREAM(p, repeat_counter);

    return wiced_bt_lrac_tx_data(tx_data, p - tx_data);
}

/*
 * app_lrac_volume_send
 *
 * @param[in] am_vol_level  - local volume level used for Audio Manager
 * @param[in] am_vol_effect - volume effect used for Audio Manager, Pri can use it to control peer volume
 */
wiced_result_t app_lrac_volume_send(int32_t am_vol_level, uint8_t am_vol_effect)
{
    uint8_t tx_data[2 + sizeof(int32_t)];
    uint8_t *p = tx_data;

    APP_TRACE_DBG("app_lrac_volume_send (vol=%d effect=%d)\n", am_vol_level, am_vol_effect);

    UINT8_TO_STREAM(p, APP_LRAC_DATA_OPCODE_VOLUME);
    UINT32_TO_STREAM(p, am_vol_level);
    UINT8_TO_STREAM(p, am_vol_effect);

    return wiced_bt_lrac_tx_data(tx_data, p - tx_data);
}

/*
 * app_lrac_send_ofu
 */
wiced_result_t app_lrac_send_ofu(uint8_t *p_data, uint16_t length)
{
    uint8_t *p = p_data;

    APP_TRACE_DBG("length:%d\n", length);

    UINT8_TO_STREAM(p, APP_LRAC_DATA_OPCODE_OFU);

    return wiced_bt_lrac_tx_data(p_data, length);
}

/*
 * app_lrac_tx_data
 */
wiced_result_t app_lrac_tx_data(uint8_t *p_data, uint16_t length)
{
    APP_TRACE_DBG("length:%d\n", length);
    return wiced_bt_lrac_tx_data(p_data, length);
}

/*
 * app_lrac_configuration_start
 */
static void app_lrac_configuration_start(void)
{
    app_nvram_lrac_info_t lrac_info;
    wiced_result_t status;

    /* Check if LRAC Info exists in NVRAM */
    status = app_nvram_lrac_info_read(&lrac_info);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("No LRAC NVRAM Info!!!\n");
        return;
    }

    /* If the LRAC Role (Primary/Secondary) is not yet defined (in NVRAM) */
    if (lrac_info.role == WICED_BT_LRAC_ROLE_UNKNOWN)
    {
        /* Let's decide we are the Primary */
        lrac_info.role = WICED_BT_LRAC_ROLE_PRIMARY;
    }
    /* If the LRAC Audio side (Primary/Secondary) is not yet defined (in NVRAM) */
    if (lrac_info.audio_side == WICED_BT_LRAC_AUDIO_SIDE_UNKNOWN)
    {
        /* Let's decide we are the Left side */
        lrac_info.audio_side = WICED_BT_LRAC_AUDIO_SIDE_LEFT;
    }

    /* Do configuration on PRI side only */
    if (lrac_info.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        APP_TRACE_DBG("app_lrac_configuration_start\n");
        app_lrac_cb.role = lrac_info.role;
        app_lrac_cb.audio_side = lrac_info.audio_side;

        /* Configure LRAC Role and Audio Side */
        status = wiced_bt_lrac_configure_req(lrac_info.role, lrac_info.audio_side);
        if (status != WICED_BT_SUCCESS)
            APP_TRACE_ERR("wiced_bt_lrac_configure_req failed\n");
    }
}

/*
 * app_lrac_config_role_get
 */
wiced_bt_lrac_role_t app_lrac_config_role_get(void)
{
    return app_lrac_cb.role;
}

/*
 * app_lrac_config_peer_addr_get
 *
 * Get the LRAC peer device's Bluetooth address
 */
void app_lrac_config_peer_addr_get(wiced_bt_device_address_t peer_addr)
{
    memcpy((void *) peer_addr, (void *) app_lrac_cb.bdaddr, sizeof(wiced_bt_device_address_t));
}

/*
 * app_lrac_rx_data_handler
 */
static void app_lrac_rx_data_handler(uint8_t *p_data, uint16_t length)
{
    uint8_t lrac_data_opcode;
    uint8_t button_id;
    app_lrac_event_data_t event_data;
    uint16_t nvram_id;
    wiced_result_t status;
    uint8_t jitter_buffer_target;

    if (length < 1)
    {
        APP_TRACE_ERR("No data received");
        return;
    }

    /* Extract the OPCODE from the Rx Data Stream */
    STREAM_TO_UINT8(lrac_data_opcode, p_data);
    length--;

    /* APP_TRACE_DBG("LRAC Data OpCode:0x%X len:%d\n", lrac_data_opcode, length); */

    switch(lrac_data_opcode)
    {
    case APP_LRAC_DATA_OPCODE_OFU:
#ifdef APP_OFU_SUPPORT
#ifdef OTA_FW_UPGRADE
        app_ofu_lrac_rx_handler(p_data, length);
#endif
#else
        {
            APP_TRACE_DBG("OFU Not Supported. Reject OFU\n");
            uint8_t ofu_err_msg[2] = { 0x00, 0x3F};
            app_lrac_send_ofu(ofu_err_msg, sizeof(ofu_err_msg));
        }
#endif
        break;

    case APP_LRAC_DATA_OPCODE_BUTTON:
        STREAM_TO_UINT8(event_data.button.button_id, p_data);
        STREAM_TO_UINT32(event_data.button.repeat_counter, p_data);
        app_lrac_cb.p_callback(APP_LRAC_BUTTON, &event_data);
        break;

    case APP_LRAC_DATA_OPCODE_VOLUME:
        STREAM_TO_UINT32(event_data.volume.am_vol_level, p_data);
        STREAM_TO_UINT8(event_data.volume.am_vol_effect, p_data);
        app_lrac_cb.p_callback(APP_LRAC_VOLUME, &event_data);
        break;

    /* Peer device asks to update an NVRAM entry */
    case APP_LRAC_DATA_OPCODE_NVRAM_WRITE_REQ:
        /* Check LRAC role. */
        if (app_lrac_cb.role != WICED_BT_LRAC_ROLE_SECONDARY)
        {
            break;
        }

        /* Update the NVRAM with the received data */
        status = bt_hs_spk_control_link_keys_set((wiced_bt_device_link_keys_t *) p_data);

        /* Reply to peer */
        app_lrac_nvram_send_write_rsp(status);
        break;

    /* Peer device replies to an NVRAM entry update request */
    case APP_LRAC_DATA_OPCODE_NVRAM_WRITE_RSP:
        STREAM_TO_UINT16(status, p_data);
        APP_TRACE_DBG("LRAC Data OpCode:NVRAM_WRITE_RSP status:%d\n", status);
        app_lrac_cb.nvram_update_in_progress = WICED_FALSE;
        break;

    case APP_LRAC_DATA_OPCODE_QUALITY:
        app_lrac_quality_peer_handler(p_data, length);
        break;

    case APP_LRAC_DATA_OPCODE_JITTER_BUFFER_TARGET:
        STREAM_TO_UINT8(jitter_buffer_target, p_data);
        app_a2dp_sink_jitter_buffer_target_set(jitter_buffer_target);
        break;

    default:
        APP_TRACE_ERR("Unknown LRAC Rx Data OpCode:0x%x\n", lrac_data_opcode);
        break;
    }
}

/*
 * app_lrac_switch_req
 */
wiced_result_t app_lrac_switch_req(wiced_bool_t prevent_glitch)
{
    wiced_bt_lrac_role_t role;
    wiced_result_t status = WICED_BT_ERROR;

    APP_TRACE_DBG("app_lrac_switch_req prevent_glitch:%d\n", prevent_glitch);

    /* set before calling app_lrac_switch_is_ready */
    app_lrac_cb.switch_prevent_glitch = prevent_glitch;

    if (app_lrac_switch_is_ready() == WICED_FALSE)
    {
        return WICED_NOT_AVAILABLE;
    }

    if (app_lrac_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {
        role = WICED_BT_LRAC_ROLE_SECONDARY;
    }
    else if (app_lrac_cb.role == WICED_BT_LRAC_ROLE_SECONDARY)
    {
        role = WICED_BT_LRAC_ROLE_PRIMARY;
    }
    else
    {
        APP_TRACE_ERR("Role not yet configured\n");
        return WICED_ERROR;
    }

    APP_TRACE_DBG("Current Jitter Buffer Level:%d\n",
            wiced_bt_lrac_audio_jitter_buffer_level_get());

    app_lrac_cb.switch_in_progress = WICED_TRUE;

    if (app_lrac_cb.role == WICED_BT_LRAC_ROLE_PRIMARY)
    {   /* Primary */
        /* Ask the Main app if a Phone is connected */
        if (bt_hs_spk_control_connection_status_check_be_edr(WICED_FALSE))
        {   /* At least one AP link exists. */
            /* Disable Sniff mode to avoid the AP link(s) becomes sniff during the switch process. */
            bt_hs_spk_control_acl_link_policy_sniff_mode_set(NULL, WICED_FALSE);

            /* Set the existent AP link(s) to Active Mode. */
            status = bt_hs_spk_control_bt_power_mode_set(WICED_TRUE, NULL, app_lrac_switch_req_power_mode_callback);

            switch (status)
            {
            /* AP link(s) is already in active mode. */
            case WICED_BT_SUCCESS:
                break;

            /* AP link(s) is waiting for changing power mode to active mode. */
            case WICED_BT_PENDING:
                /* The callback will be called when the AP link(s) becomes Active */
                return WICED_BT_SUCCESS;

            default:
#ifdef DISABLE_SNIFF_MODE_DURING_A2DP
                /* only enable sniff mode if no streaming */
                if (bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED)
#endif
                {
                    /* Enable Sniff mode */
                    bt_hs_spk_control_acl_link_policy_sniff_mode_set(NULL, WICED_TRUE);
                }

                app_lrac_cb.switch_in_progress = WICED_FALSE;
                APP_TRACE_ERR("power_mode_set_active failed (%d)\n", status);
                return status;
            }
        }
        else
        {   /* No AP link(s) */
            /* Exit sniff power_mgmt - lower layer will wait for exit automatically  */
            wiced_bt_lrac_sniff_power_mgmt_exit(NULL);
        }
    }

    /* Ask LRAC Library to perform the Switch */
    status = wiced_bt_lrac_switch_req(role, prevent_glitch);
    APP_TRACE_DBG("wiced_bt_lrac_switch_req status: %d\n", status);
    if (status == WICED_BT_SUCCESS)
    {
        /* Disable discovery */
        app_main_lrac_switch_backup_bt_visibility();
        bt_hs_spk_button_set_discovery(WICED_FALSE);

#ifdef AMA_ENABLED
        ama_suspend(1);
#endif

        /* Increase the CPU Clock to reduce the PS-Switch time */
        app_cpu_clock_increase(APP_CPU_CLOCK_REQUESTER_PS_SWITCH);
    }
    else
    {
        app_lrac_switch_sniff_restore();
        app_lrac_cb.switch_in_progress = WICED_FALSE;
        APP_TRACE_ERR("wiced_bt_lrac_switch_req failed %d\n", status);
        return WICED_ERROR;
    }

    return status;
}

/*
 * app_lrac_switch_req_power_mode_callback
 */
static void app_lrac_switch_req_power_mode_callback(wiced_bt_device_address_t bdaddr,
        wiced_bt_dev_power_mgmt_status_t power_mode)
{
    wiced_result_t status;
    app_lrac_event_data_t event_data;

    APP_TRACE_DBG("app_lrac_switch_req_power_mode_callback (%B, %d, %d)\n",
                  bdaddr,
                  power_mode,
                  app_lrac_cb.switch_in_progress);

    /* Check if the device is still under the switch process. */
    if (app_lrac_cb.switch_in_progress == WICED_FALSE)
    {
        return;
    }

    /* Peer device (phone here) in in Active mode */
    if (power_mode == WICED_POWER_STATE_ACTIVE)
    {
        if (app_lrac_cb.switch_in_progress)
        {
            /* PS-Switch is already in progress (we requested it). Set it to FALSE to prevent error */
            app_lrac_cb.switch_in_progress = WICED_FALSE;
            status = app_lrac_switch_req(app_lrac_cb.switch_prevent_glitch);
        }
        else
        {
            status = WICED_BT_SUCCESS;
        }
    }
    else
    {
        status = WICED_BT_ERROR;
    }

    if (status != WICED_BT_SUCCESS)
    {
        /* report status */
        wiced_bt_lrac_switch_result_t report_status;
        if (status == WICED_NOT_AVAILABLE)
        {
            report_status = WICED_BT_LRAC_SWITCH_NOT_READY;
        }
        else
        {
            report_status = WICED_BT_LRAC_SWITCH_USER_ABORT;
        }
        app_hci_lrac_switch_result(report_status, 1, 0);

        app_lrac_cb.switch_in_progress = WICED_FALSE;
        event_data.switch_completed.status = WICED_BT_ERROR;
        event_data.switch_completed.new_role = app_lrac_cb.role;
        event_data.switch_completed.fatal_error = WICED_FALSE;
        app_lrac_cb.p_callback(APP_LRAC_SWITCH_COMPLETED, &event_data);
    }
}

/*
 * app_lrac_switch_rsp_power_mode_callback
 */
static void app_lrac_switch_rsp_power_mode_callback(wiced_bt_device_address_t bdaddr,
        wiced_bt_dev_power_mgmt_status_t power_mode)
{
    wiced_bt_lrac_event_data_t event;
    wiced_result_t status;

    APP_TRACE_DBG("app_lrac_switch_rsp_power_mode_callback (%B, %d, %d)\n",
                  bdaddr,
                  power_mode,
                  app_lrac_cb.switch_in_progress);

    /* Check if the device is still under the switch process. */
    if (app_lrac_cb.switch_in_progress == WICED_FALSE)
    {
        return;
    }

    app_lrac_cb.switch_in_progress = WICED_FALSE;

    /* Peer device (phone here) in in Active mode */
    if (power_mode == WICED_POWER_STATE_ACTIVE)
    {
        event.switch_req.new_role = app_lrac_cb.role == WICED_BT_LRAC_ROLE_PRIMARY ? WICED_BT_LRAC_ROLE_SECONDARY : WICED_BT_LRAC_ROLE_PRIMARY;
        event.switch_req.prevent_glitch = app_lrac_cb.switch_prevent_glitch;
        app_lrac_cback(WICED_BT_LRAC_EVENT_SWITCH_REQ, &event);
    }
    else
    {
        /* Send Switch Response */
        status = wiced_bt_lrac_switch_rsp(WICED_BT_ERROR, WICED_FALSE);
        if (status != WICED_BT_SUCCESS)
        {
            APP_TRACE_ERR("wiced_bt_lrac_switch_rsp failed %d\n", status);
        }
    }
}

/*
 * app_lrac_switch_is_in_progress
 */
wiced_bool_t app_lrac_switch_is_in_progress(void)
{
    if (app_lrac_cb.switch_in_progress)
        APP_TRACE_DBG("switch_in_progress:%d\n", app_lrac_cb.switch_in_progress);
    return app_lrac_cb.switch_in_progress;
}

/*
 * app_lrac_switch_sniff_restore
 * This function is called after PS-Switch to restore Sniff (to the Phone) if needed
 */
static void app_lrac_switch_sniff_restore(void)
{
    /* Nothing to do if we are not the Primary */
    if (app_lrac_cb.role != WICED_BT_LRAC_ROLE_PRIMARY)
    {
        return;
    }

    /* Ask the Main app if a Phone is connected */
    if (bt_hs_spk_control_connection_status_check_be_edr(WICED_FALSE))
    {   /* At least one AP link exists. */
#ifdef DISABLE_SNIFF_MODE_DURING_A2DP
        /* only enable sniff mode if no streaming */
        if (!bt_hs_spk_audio_is_a2dp_streaming_started())
#endif
        {
            /* (Re)Enable Sniff mode */
            bt_hs_spk_control_acl_link_policy_sniff_mode_set(NULL, WICED_TRUE);

            /* Ask the AP link(s) to enter sniff mode if possible. */
            bt_hs_spk_control_bt_power_mode_set(WICED_FALSE, NULL, NULL);
        }
    }
}

/*
 * app_lrac_ready_to_switch
 * This function will be called before LRAC Switch will be performed
 */
wiced_bool_t app_lrac_ready_to_switch(void)
{
    if (app_audio_insert_is_started())
    {
        APP_TRACE_ERR("Switch not allowed during Audio Insertion\n");
        return WICED_FALSE;
    }

    if (app_lrac_switch_is_in_progress())
    {
        APP_TRACE_ERR("Switch already in progress\n");
        return WICED_FALSE;
    }

    /* Condition to prevent glitch */
    if (app_lrac_cb.switch_prevent_glitch)
    {
        if (bt_hs_spk_audio_streaming_check(NULL) == WICED_ALREADY_CONNECTED)
        {
            uint16_t level = wiced_bt_lrac_audio_jitter_buffer_level_get();

            if (level < APP_LRAC_SWITCH_PREVENT_GLITCH_JITTER_BUFFER_SIZE)
            {
                APP_TRACE_ERR("Rejct switch to prevent glitch(%d/%d)\n", level,
                        APP_LRAC_SWITCH_PREVENT_GLITCH_JITTER_BUFFER_SIZE);
                return WICED_FALSE;
            }
        }
    }

    return WICED_TRUE;
}

/*
 * app_lrac_switch_get
 */
wiced_result_t app_lrac_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
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

    if (*p_sync_data_len < sizeof(app_lrac_cb))
    {
        APP_TRACE_ERR("buffer too small (%d/%d)\n", *p_sync_data_len, sizeof(app_lrac_cb));
        return WICED_BT_BADARG;
    }

    memcpy(p_opaque, &app_lrac_cb, sizeof(app_lrac_cb));

    *p_sync_data_len = sizeof(app_lrac_cb);

    return WICED_BT_SUCCESS;
}

/*
 * app_lrac_switch_set
 */
wiced_result_t app_lrac_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    wiced_bt_lrac_audio_side_t  audio_side;

    if (p_opaque == NULL)
    {
        APP_TRACE_ERR("p_opaque is NULL\n");
        return WICED_BT_BADARG;
    }

    if (sync_data_len != sizeof(app_lrac_cb))
    {
        APP_TRACE_ERR("bad buffer size (%d/%d)\n", sync_data_len, sizeof(app_lrac_cb));
        return WICED_BT_BADARG;
    }

    /* The Audio side is not supposed to change after LRAC Switch. Save current Audio side */
    audio_side = app_lrac_cb.audio_side;

    /* Copy the received data */
    memcpy(&app_lrac_cb, p_opaque, sync_data_len);

    /* Restore Audio side */
    app_lrac_cb.audio_side = audio_side;

    return WICED_BT_SUCCESS;
}

/*
 * app_lrac_role_get_desc
 */
char *app_lrac_role_get_desc(wiced_bt_lrac_role_t role)
{
    switch(role)
    {
    case WICED_BT_LRAC_ROLE_PRIMARY: return "Primary";
    case WICED_BT_LRAC_ROLE_SECONDARY: return "Secondary";
    default: return "Unknown Role";
    }
}

/*
 * app_lrac_audio_side_get_desc
 */
char *app_lrac_audio_side_get_desc(wiced_bt_lrac_audio_side_t audio_side)
{
    switch(audio_side)
    {
    case WICED_BT_LRAC_AUDIO_SIDE_LEFT: return "Left";
    case WICED_BT_LRAC_AUDIO_SIDE_RIGHT: return "Right";
    default: return "Unknown Audio Side";
    }
}

/*
 * app_lrac_send_quality
 * Send 'Quality' information (RSSI, FW Statistics, Under/OverRun, etc.) to peer device
 */
wiced_result_t app_lrac_send_quality(void *p_data, uint16_t length)
{
    uint8_t *p_buffer;
    uint8_t *p;
    wiced_result_t status;
    uint16_t total_length;

    /* Allocate memory for the data to send */
    p_buffer = wiced_bt_get_buffer(length + 1);
    if (p_buffer == NULL)
    {
        APP_TRACE_ERR("No memory\n");
        return WICED_BT_NO_RESOURCES;
    }

    p = p_buffer;

    /* Write opcode */
    UINT8_TO_STREAM(p, APP_LRAC_DATA_OPCODE_QUALITY);

    /* Write Quality data*/
    memcpy(p, p_data, length);

    /* Send LRAC LRAC User Data to peer device. */
    status = wiced_bt_lrac_tx_data(p_buffer, length + 1);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("Send LRAC User Data fail (%d).\n", status);
    }

    wiced_bt_free_buffer(p_buffer);

    return WICED_BT_SUCCESS;
}

/*
 * app_lrac_jitter_buffer_target_send
 * Send the A2DP Jitter Buffer Target depth to peer device
 */
wiced_result_t app_lrac_jitter_buffer_target_send(uint8_t jitter_buffer_target)
{
    uint8_t tx_data[1 + 1];
    uint8_t *p = tx_data;

    APP_TRACE_DBG("Target:%d\n", jitter_buffer_target);

    UINT8_TO_STREAM(p, APP_LRAC_DATA_OPCODE_JITTER_BUFFER_TARGET);
    UINT8_TO_STREAM(p, jitter_buffer_target);

    return wiced_bt_lrac_tx_data(tx_data, p - tx_data);
}

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
wiced_bool_t app_lrac_phone_profile_connection_up(wiced_bt_device_address_t bdaddr, uint8_t profile)
{
    wiced_bool_t connected = WICED_FALSE;

    switch (profile)
    {
    case APP_NVRAM_PROFILES_A2DP_SINK_MASK:
        /* Check if the HFP profile is already connected with the peer device. */
        if (bt_hs_spk_handsfree_hfp_connection_check(bdaddr, WICED_TRUE))
        {
            return WICED_FALSE;
        }

        /* Check if the AVRC profile is already connected with the peer device. */
        if (bt_hs_spk_audio_avrc_connection_check(bdaddr, &connected))
        {
            if (connected)
            {
                return WICED_FALSE;
            }
        }
        break;
    case APP_NVRAM_PROFILES_AVRC_CT_MASK:
        /* Check if the HFP profile is already connected with the peer device. */
        if (bt_hs_spk_handsfree_hfp_connection_check(bdaddr, WICED_TRUE))
        {
            return WICED_FALSE;
        }

        /* Check if the A2DP profile is already connected with the peer device. */
        if (bt_hs_spk_audio_a2dp_connection_check(bdaddr, &connected))
        {
            if (connected)
            {
                return WICED_FALSE;
            }
        }
        break;
    case APP_NVRAM_PROFILES_HFP_HS_MASK:
        /* Check if the A2DP or AVRC profile is already connected with the peer device. */
        if (bt_hs_spk_audio_connection_check(bdaddr))
        {
            return WICED_FALSE;
        }
        break;
    default:
        return WICED_FALSE;
    }

    /* Tell it to the LRAC/WASS library (to Start RSSI measurement) */
    APP_TRACE_DBG("app_lrac_phone_profile_connection_up:%B\n", bdaddr);
    wiced_bt_lrac_phone_connection_up(bdaddr);

    return WICED_TRUE;
}

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
wiced_bool_t app_lrac_phone_profile_connection_down(wiced_bt_device_address_t bdaddr, uint8_t profile)
{
    wiced_bool_t connected = WICED_FALSE;

    switch (profile)
    {
    case APP_NVRAM_PROFILES_A2DP_SINK_MASK:
        /* Sanity check */
        /* Check if the A2DP profile is already connected. */
        if (bt_hs_spk_audio_a2dp_connection_check(bdaddr, &connected) == WICED_FALSE)
        {
            return WICED_FALSE;
        }

        if (connected == WICED_FALSE)
        {
            return WICED_FALSE;
        }

        /* Check if the HFP profile is still connected with the peer device. */
        if (bt_hs_spk_handsfree_hfp_connection_check(bdaddr, WICED_FALSE))
        {
            return WICED_FALSE;
        }

        /* Check if the AVRC profile is still connected with the peer device. */
        if (bt_hs_spk_audio_avrc_connection_check(bdaddr, &connected))
        {
            if (connected)
            {
                return WICED_FALSE;
            }
        }
        break;
    case APP_NVRAM_PROFILES_AVRC_CT_MASK:
        /* Sanity check */
        /* Check if the AVRC profile is already connected. */
        if (bt_hs_spk_audio_avrc_connection_check(bdaddr, &connected) == WICED_FALSE)
        {
            return WICED_FALSE;
        }

        if (connected == WICED_FALSE)
        {
            return WICED_FALSE;
        }

        /* Check if the HFP profile is still connected with the peer device. */
        if (bt_hs_spk_handsfree_hfp_connection_check(bdaddr, WICED_FALSE))
        {
            return WICED_FALSE;
        }

        /* Check if the A2DP profile is still connected with the peer device. */
        if (bt_hs_spk_audio_a2dp_connection_check(bdaddr, &connected))
        {
            if (connected)
            {
                return WICED_FALSE;
            }
        }
        break;
    case APP_NVRAM_PROFILES_HFP_HS_MASK:
        /* Sanity check */
        /* Check if the HFP profile is already connected. */
        if (bt_hs_spk_handsfree_hfp_connection_check(bdaddr, WICED_TRUE) == WICED_FALSE)
        {
            return WICED_FALSE;
        }

        /* Check if the A2DP or AVRC profile is still connected with the peer device. */
        if (bt_hs_spk_audio_connection_check(bdaddr))
        {
            return WICED_FALSE;
        }
        break;
    default:
        return WICED_FALSE;
    }

    /* Tell it to the LRAC/WASS library (to stop RSSI measurement) */
    APP_TRACE_DBG("app_lrac_phone_profile_connection_down:%B\n", bdaddr);
    wiced_bt_lrac_phone_connection_down(bdaddr);

    return WICED_TRUE;
}

/*
 * app_lrac_link_key_get
 */
wiced_bt_device_sec_keys_t *app_lrac_link_key_get(void)
{
    return &app_lrac_cb.key_data;
}

/*
 * app_lrac_link_key_update
 */
void app_lrac_link_key_update(wiced_bt_device_sec_keys_t *p_key_data)
{
    /* Check parameter. */
    if (p_key_data == NULL)
    {
        return;
    }

    /* Compare the data content. */
    if (memcmp((void *) &app_lrac_cb.key_data,
               (void *) p_key_data,
               sizeof(wiced_bt_device_sec_keys_t)) == 0)
    {
        return;
    }

    /* Update the key data. */
    memcpy((void *) &app_lrac_cb.key_data,
           (void *) p_key_data,
           sizeof(wiced_bt_device_sec_keys_t));

    /* Update NVRAM. */
    app_nvram_pairing_info_write(&app_lrac_cb.key_data);
}

/*
 * app_lrac_link_key_reset
 */
void app_lrac_link_key_reset(void)
{
    if (bt_hs_spk_control_misc_data_content_check((uint8_t *) &app_lrac_cb.key_data,
                                                  sizeof(wiced_bt_device_sec_keys_t)) == WICED_FALSE)
    {
        return;
    }

    memset((void *) &app_lrac_cb.key_data, 0, sizeof(wiced_bt_device_sec_keys_t));

    app_nvram_pairing_info_delete();
}
