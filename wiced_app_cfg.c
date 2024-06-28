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
 * Runtime Bluetooth stack configuration parameters
 *
 */
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_audio.h"
#include "wiced_bt_lrac.h"
#include "app_a2dp_sink.h"
#include "app_handsfree.h"
#ifdef APP_OFU_SUPPORT
#include "ofu/app_ofu_spp.h"
#endif
#ifdef APP_TPUT_SPP
#include "app_tput_spp.h"
#endif
#include "bt_hs_spk_handsfree.h"

enum
{
#ifdef APP_OFU_SUPPORT
    OFU_SPP_RFCOMM_PORT_COUNT = 1,
#else
    OFU_SPP_RFCOMM_PORT_COUNT = 0,
#endif
};

#if (WICED_A2DP_EXT_CODEC == WICED_TRUE)
extern wiced_codec_interface_functions_t AAC_codec_function_table;
#endif

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
#define WICED_PIN_CODE_LEN                  4
const uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };

const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name                         = (uint8_t *)"WASS",                                          /**< Local device name (NULL terminated) */
    .device_class                        = {0x24, 0x04, 0x18},                                         /**< Local device class */
    .security_requirement_mask           = (  BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT ), /**< Security requirements mask (BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT (see #wiced_bt_sec_level_e)) */

    .max_simultaneous_links              = BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS + 1,               /**< Maximum number simultaneous links to different devices */

    .br_edr_scan_cfg =                                              /* BR/EDR scan config */
    {
        .inquiry_scan_type               = BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .inquiry_scan_interval           = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  (0 to use default) */
        .inquiry_scan_window             = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window (0 to use default) */

        .page_scan_type                  = BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .page_scan_interval              = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  (0 to use default) */
        .page_scan_window                = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window (0 to use default) */
    },

    .ble_scan_cfg =                                                 /* LE scan settings  */
    {
        .scan_mode                       = BTM_BLE_SCAN_MODE_ACTIVE,                                   /**< LE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

        /* Advertisement scan configuration */
        .high_duty_scan_interval         = 96,                                                         /**< High duty scan interval */
        .high_duty_scan_window           = 48,                                                         /**< High duty scan window */
        .high_duty_scan_duration         = 30,                                                         /**< High duty scan duration in seconds (0 for infinite) */

        .low_duty_scan_interval          = 2048,                                                       /**< Low duty scan interval  */
        .low_duty_scan_window            = 48,                                                         /**< Low duty scan window */
        .low_duty_scan_duration          = 30,                                                         /**< Low duty scan duration in seconds (0 for infinite) */

        /* Connection scan configuration */
        .high_duty_conn_scan_interval    = 96,                                                         /**< High duty cycle connection scan interval */
        .high_duty_conn_scan_window      = 48,                                                         /**< High duty cycle connection scan window */
        .high_duty_conn_duration         = 30,                                                         /**< High duty cycle connection duration in seconds (0 for infinite) */

        .low_duty_conn_scan_interval     = 2048,                                                       /**< Low duty cycle connection scan interval */
        .low_duty_conn_scan_window       = 48,                                                         /**< Low duty cycle connection scan window */
        .low_duty_conn_duration          = 30,                                                         /**< Low duty cycle connection duration in seconds (0 for infinite) */

        /* Connection configuration */
        .conn_min_interval               = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                     /**< Minimum connection interval */
        .conn_max_interval               = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                     /**< Maximum connection interval */
        .conn_latency                    = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection latency */
        .conn_supervision_timeout        = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection link supervision timeout */
    },

    .ble_advert_cfg =                                               /* LE advertisement settings */
    {
        .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */
                                           BTM_BLE_ADVERT_CHNL_38 |
                                           BTM_BLE_ADVERT_CHNL_39,

        .high_duty_min_interval          = 160,                                                        /**< High duty undirected connectable minimum advertising interval */
        .high_duty_max_interval          = 160,                                                        /**< High duty undirected connectable maximum advertising interval */
        .high_duty_duration              = 0,                                                          /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */

        .low_duty_min_interval           = 400,                                                        /**< Low duty undirected connectable minimum advertising interval */
        .low_duty_max_interval           = 400,                                                        /**< Low duty undirected connectable maximum advertising interval */
        .low_duty_duration               = 0,                                                          /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */

        .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High duty directed connectable minimum advertising interval */
        .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High duty directed connectable maximum advertising interval */

        .low_duty_directed_min_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low duty directed connectable minimum advertising interval */
        .low_duty_directed_max_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low duty directed connectable maximum advertising interval */
        .low_duty_directed_duration      = 30,                                                         /**< Low duty directed connectable advertising duration in seconds (0 for infinite) */

        .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        .high_duty_nonconn_duration      = 30,                                                         /**< High duty non-connectable advertising duration in seconds (0 for infinite) */

        .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        .low_duty_nonconn_duration       = 0                                                           /**< Low duty non-connectable advertising duration in seconds (0 for infinite) */
    },

    .gatt_cfg =                                                     /* GATT configuration */
    {
        .appearance                     = APPEARANCE_GENERIC_TAG,                                      /**< GATT appearance (see gatt_appearance_e) */
        .client_max_links               = 3,                                                           /**< Client config: maximum number of servers that local client can connect to  */
        .server_max_links               = 3,                                                           /**< Server config: maximum number of remote clients connections allowed by the local */
        .max_attr_len                   = 243 - 5,                                                     /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
#if !defined(CYW20706A2)
        .max_mtu_size                   = 243,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
#endif
    },

    .rfcomm_cfg =                                                   /* RFCOMM configuration */
    {
        .max_links                      = WICED_BT_HFP_HF_MAX_CONN,                                    /**< Maximum number of simultaneous connected remote devices*/
        .max_ports                      = WICED_BT_HFP_HF_MAX_CONN + OFU_SPP_RFCOMM_PORT_COUNT         /**< Maximum number of simultaneous RFCOMM ports */
    },

    .l2cap_application =                                            /* Application managed l2cap protocol configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        .max_psm                        = 0,                                                           /**< Maximum number of application-managed BR/EDR PSMs */
        .max_channels                   = 0,                                                           /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        .max_le_psm                     = 1,                                                           /**< Maximum number of application-managed LE PSMs */
        .max_le_channels                = 1,                                                           /**< Maximum number of application-managed LE channels */
#if !defined(CYW20706A2)
        /* LE L2cap fixed channel configuration */
        .max_le_l2cap_fixed_channels    = 0                                                            /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */
#endif
    },

    .avdt_cfg =
    /* Audio/Video Distribution configuration */
    {
        .max_links                      = WICED_BT_A2DP_SINK_MAX_NUM_CONN,                             /**< Maximum simultaneous audio/video links */
#if !defined(CYW20706A2)
        .max_seps                       = WICED_BT_A2DP_SINK_MAX_NUM_CONN * WICED_BT_A2DP_SINK_MAX_NUM_CODECS, /**< Maximum number of stream end points */
#endif
    },

    .avrc_cfg =                                                     /* Audio/Video Remote Control configuration */
    {
        .roles                          = 1,                                                           /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
        .max_links                      = MAX_CONNECTED_RCC_DEVICES                                    /**< Maximum simultaneous remote control links */
    },

    /* LE Address Resolution DB size  */
    .addr_resolution_db_size            = 5,                                                           /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/

#ifdef CYW20706A2
    .max_mtu_size                       = 365,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    .max_pwr_db_val                     = 12                                                           /**< Max. power level of the device */
#else
    /* Maximum number of buffer pools */
    .max_number_of_buffer_pools         = 10,                                                          /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */

    /* Interval of  random address refreshing */
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_CHANGE_TIMEOUT,          /**< Interval of  random address refreshing - secs */

    /* LE Filter Accept List size */
    .ble_filter_accept_list_size                = 0,                                                           /**< Maximum number of Filter Accept List devices allowed. Cannot be more than 128 */
#endif

#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1) || defined (CYW20820A1)
    .default_ble_power_level            = 12,                                                          /**< Default LE power level, Refer lm_TxPwrTable table for the power range */
#endif
};

/*****************************************************************************
 * SDP database for the hci_control application
 ****************************************************************************/
/* Macro to encode the header of a 128 bits UUID */
#define SDP_ATTR_UUID128      ((UUID_DESC_TYPE << 3) | SIZE_SIXTEEN_BYTES)

/* Macro to declare a Service Class based on 128 bits UUID */
#define SDP_ATTR_CLASS_ID128                                   \
    SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(17), \
    SDP_ATTR_UUID128

// SDP Record handle for AVDT Sink
#define HANDLE_AVDT_SINK                        0x10001
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_TARGET                      0x10002
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_CONTROLLER                  0x10003
// SDP Record Handle for HFP
#define HANDLE_HFP_HF                           0x10004
// SDP Record handle for SPP OFU
#define HANDLE_SPP_OFU                          0x10005
// SDP Record handle for LRAC
#define HANDLE_LRAC                             0x10006
// SDP Record handle for SPP Throughput
#define HANDLE_SPP_TPUT                         0x10007

/*
 * SDP Server database containing everything (including LRAC Record)
 */

const uint8_t headset_sdp_db[] =
{
    SDP_ATTR_SEQUENCE_2(
              77 + 2                // A2DP Sink       ==> 77 + 2
            + 56 + 2                // AVRC Target     ==> 56 + 2
            + 59 + 2                // AVRC Controller ==> 59 + 2
            + 75 + 2                // Handsfree       ==> 75 + 2
#ifdef APP_OFU_SUPPORT
            + 69 + 2                // SPP OFU         ==> 69 + 2
#endif
            + 81 + 2                // LRAC            ==> 79 + 2
#ifdef APP_TPUT_SPP
            + 74 + 2                // SPP TPUT        ==> 74 + 2
#endif
            ),

    // SDP Record for A2DP Sink
    SDP_ATTR_SEQUENCE_1(77),        // 2 bytes (Length of record)
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVDT_SINK),               // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SINK),           // 8 bytes
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST),                // 3 bytes
            SDP_ATTR_SEQUENCE_1(16),                            // 2 bytes
                SDP_ATTR_SEQUENCE_1(6),                             // 2 bytes
                    SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),               // 3 bytes
                    SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),                 // 3 bytes
                SDP_ATTR_SEQUENCE_1(6),                         // 2 bytes
                    SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),               // 3 bytes
                    SDP_ATTR_VALUE_UINT2(AVDT_VERSION_1_3),             // 3 bytes
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST),              // 3 bytes
            SDP_ATTR_SEQUENCE_1(8),                                 // 2 bytes
                SDP_ATTR_SEQUENCE_1(6),                             // 2 bytes
                    SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION), // 3 bytes
                    SDP_ATTR_VALUE_UINT2(AVDT_VERSION_1_3),                 // 3 bytes
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x000B),     // 6 bytes
        SDP_ATTR_SERVICE_NAME(16),                              // 5  + 16 bytes
            'W', 'I', 'C', 'E', 'D', ' ', 'A', 'u', 'd', 'i', 'o', ' ', 'S', 'i', 'n', 'k',

    // SDP Record for AVRC Target
    SDP_ATTR_SEQUENCE_1(56),        // 2 bytes (Length of record)
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_TARGET),             // 8 bytes
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),             // 3 bytes
            SDP_ATTR_SEQUENCE_1(3),                             // 2 bytes
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_TARGET), // 3 bytes
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST),                    // 3 bytes
            SDP_ATTR_SEQUENCE_1(16),                                // 2 bytes
                SDP_ATTR_SEQUENCE_1(6),                                 // 2 bytes
                    SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),                   // 3 bytes
                    SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),                     // 3 bytes
                SDP_ATTR_SEQUENCE_1(6),                                 // 2 bytes
                    SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),                   // 3 bytes
                    SDP_ATTR_VALUE_UINT2(0x0104),                       // 3 bytes
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST),                  // 3 bytes
            SDP_ATTR_SEQUENCE_1(8),                                     // 2 bytes
                SDP_ATTR_SEQUENCE_1(6),                                     // 2 bytes
                    SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),      // 3 bytes
                    SDP_ATTR_VALUE_UINT2(AVRC_REV_1_5),                     // 3 bytes
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_TG_CAT2), // 6 bytes

    // SDP Record for AVRC Controller
    SDP_ATTR_SEQUENCE_1(59),        // 2 bytes (Length of record)
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_CONTROLLER),         // 8 bytes
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),             // 3 bytes
            SDP_ATTR_SEQUENCE_1(6),                             // 2 bytes
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),      // 3 bytes
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_CONTROL),    // 3 bytes
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),   // 3 + 2 bytes
            SDP_ATTR_SEQUENCE_1(6),                             // 2 bytes
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),           // 3 bytes
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),             // 3 bytes
            SDP_ATTR_SEQUENCE_1(6),                             // 2 bytes
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),           // 3 bytes
                SDP_ATTR_VALUE_UINT2(0x104),                    // 3 bytes
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),  // 3 + 2 bytes
            SDP_ATTR_SEQUENCE_1(6),                             // 2 bytes
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),  // 3 bytes
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_3),             // 3 bytes
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_CT_CAT1),  // 6 bytes

    // SDP Record for Hands-Free Unit
    SDP_ATTR_SEQUENCE_1(75),        // 2 bytes (Length of record)
        SDP_ATTR_RECORD_HANDLE(HANDLE_HFP_HF),                  // 8 bytes
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(6), // 3 + 2 bytes
            SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),       // 3 bytes
            SDP_ATTR_UUID16(UUID_SERVCLASS_GENERIC_AUDIO),      // 3 bytes
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(WICED_HANDSFREE_SCN),    // 17 bytes
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),  // 3 + 2 bytes
            SDP_ATTR_SEQUENCE_1(6),                             // 2 bytes
                SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),   // 3 bytes
                SDP_ATTR_VALUE_UINT2(0x0106),                   // 3 bytes
        SDP_ATTR_SERVICE_NAME(15),                              // 5 + 15 bytes
            'W', 'I', 'C', 'E', 'D', ' ', 'H', 'F', ' ', 'D', 'E', 'V', 'I', 'C', 'E',
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, APP_HANDFREE_SDP_FEATURE),  // 6 bytes

#ifdef APP_OFU_SUPPORT
    SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes
        SDP_ATTR_RECORD_HANDLE(HANDLE_SPP_OFU),                             // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),                      // 8
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(APP_OFU_SPP_RFCOMM_SCN),         // 17 bytes
        SDP_ATTR_BROWSE_LIST,                                               // 8
        SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_SERIAL_PORT, 0x0102),     // 13 byte
        SDP_ATTR_SERVICE_NAME(10),                                          // 15
        'S', 'P', 'P', ' ', 'S', 'E', 'R', 'V', 'E', 'R',
#endif

    // LRAC Record
    SDP_ATTR_SEQUENCE_1(8 + 6 + 16 + 18 + 15),                  // 2 bytes (Length of record)
        SDP_ATTR_RECORD_HANDLE(HANDLE_LRAC),                    // 8 bytes
        SDP_ATTR_CLASS_ID128,                                   // 6 bytes
        WICED_BT_LRAC_UUID128,                                  // 16 bytes
        SDP_ATTR_PROTOCOL_DESC_LIST(WICED_BT_LRAC_PSM_CONTROL), // 18 bytes
        SDP_ATTR_TEXT_1(ATTR_ID_SERVICE_NAME, 10),              // 5 + 10
            'W', 'I', 'C', 'E', 'D', '-', 'L', 'R', 'A', 'C',

#ifdef APP_TPUT_SPP
    SDP_ATTR_SEQUENCE_1(74),                                                // 2 bytes
        SDP_ATTR_RECORD_HANDLE(HANDLE_SPP_TPUT),                            // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),                      // 8
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(APP_TPUT_SPP_RFCOMM_SCN),        // 17 bytes
        SDP_ATTR_BROWSE_LIST,                                               // 8
        SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_SERIAL_PORT, 0x0102),     // 13 byte
        SDP_ATTR_SERVICE_NAME(15),                                          // 5 + 15
        'S', 'P', 'P', ' ', 'T', 'P', 'U', 'T', ' ', 'S', 'E', 'R', 'V', 'E', 'R',
#endif
};

const uint16_t headset_sdp_db_size = sizeof(headset_sdp_db);

/*
 * SDP Server database containing LRAC Record only
 */
const uint8_t lrac_sdp_db[] =
{

    SDP_ATTR_SEQUENCE_2(
            + 79 + 2                // LRAC            ==> 79 + 2
            ),

    // LRAC Record
    SDP_ATTR_SEQUENCE_1(8 + 6 + 16 + 17 + 15),              // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE(HANDLE_LRAC),                // 8 bytes
        SDP_ATTR_CLASS_ID128,                               // 6 bytes
        WICED_BT_LRAC_UUID128,                               // 16 bytes
        SDP_ATTR_PROTOCOL_DESC_LIST(WICED_BT_LRAC_PSM_CONTROL),       // 17
        SDP_ATTR_TEXT_1(ATTR_ID_SERVICE_NAME, 10),           // 5 + 10
            'W', 'I', 'C', 'E', 'D', '-', 'L', 'R', 'A', 'C'
};

const uint16_t lrac_sdp_db_size = sizeof(lrac_sdp_db);

/*****************************************************************************
 * wiced_bt  buffer pool configuration
 *
 * Configure buffer pools used by the stack  according to application's requirement
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[] =
{
/*  { buf_size, buf_count } */
    { 64,       20  },      /* Small Buffer Pool */
    { 272,      6   },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
#ifdef APP_TPUT_SPP
    { 1056,     15   },     /* Large Buffer Pool  (used for HCI ACL messages) */
#else
    { 1056,     6   },      /* Large Buffer Pool  (used for HCI ACL messages) */
#endif
    { 1056,     1   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};

/*****************************************************************************
 *   codec and audio tuning configurations
 ****************************************************************************/
/*  Recommended max_bitpool for high quality audio */
#define BT_AUDIO_A2DP_SBC_MAX_BITPOOL   53

/* Array of decoder capabilities information. */
wiced_bt_a2dp_codec_info_t bt_audio_codec_capabilities[] =
{
    {
        .codec_id = WICED_BT_A2DP_CODEC_SBC,
        .cie =
        {
            .sbc =
            {
                (A2D_SBC_IE_SAMP_FREQ_44 | A2D_SBC_IE_SAMP_FREQ_48),    /* samp_freq */
                (A2D_SBC_IE_CH_MD_MONO   | A2D_SBC_IE_CH_MD_STEREO |
                 A2D_SBC_IE_CH_MD_JOINT  | A2D_SBC_IE_CH_MD_DUAL),      /* ch_mode */
                (A2D_SBC_IE_BLOCKS_16    | A2D_SBC_IE_BLOCKS_12 |
                 A2D_SBC_IE_BLOCKS_8     | A2D_SBC_IE_BLOCKS_4),        /* block_len */
                (A2D_SBC_IE_SUBBAND_4    | A2D_SBC_IE_SUBBAND_8),       /* num_subbands */
                (A2D_SBC_IE_ALLOC_MD_L   | A2D_SBC_IE_ALLOC_MD_S),      /* alloc_mthd */
                BT_AUDIO_A2DP_SBC_MAX_BITPOOL,                          /* max_bitpool for high quality audio */
                A2D_SBC_IE_MIN_BITPOOL                                  /* min_bitpool */
            }
        }
#ifdef A2DP_SINK_AAC_ENABLED
    },

    {
        .codec_id = WICED_BT_A2DP_CODEC_M24,
        .cie =
        {
            .m24 =
            {
                (A2D_M24_IE_OBJ_MSK),                                   /* obj_type */
                (A2D_M24_IE_SAMP_FREQ_44 | A2D_M24_IE_SAMP_FREQ_48),    /* samp_freq */
                (A2D_M24_IE_CHNL_MSK),                                  /* chnl */
                (A2D_M24_IE_VBR_MSK),                                   /* b7: VBR */
                (A2D_M24_IE_BITRATE_MSK)                                /* bitrate - b7-b0 of octect 3, all of octect4, 5*/
            }
        }
#endif
    }
};

/** A2DP sink configuration data */
wiced_bt_a2dp_config_data_t bt_audio_config =
{
    /* feature mask */
#ifdef A2DP_SINK_ENABLE_CONTENT_PROTECTION
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_DELAY_RPT |                 /* Delay Report Feature */
                    WICED_BT_A2DP_SINK_FEAT_PROTECT,                    /* Content Protection */
#else
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_DELAY_RPT,                  /* Delay Report Feature */
#endif
    .codec_capabilities =
    {
        .count = sizeof(bt_audio_codec_capabilities) / sizeof(bt_audio_codec_capabilities[0]),
        .info  = bt_audio_codec_capabilities,                   /* codec configuration */
    },
    .p_param =
    {
#ifdef A2DP_SINK_AAC_ENABLED
        .buf_depth_ms                   = 300,                                          /* in msec */
#else
        .buf_depth_ms                   = 400,                                          /* in msec */
#endif
        .start_buf_depth                = APP_A2DP_SINK_JITTER_BUFFER_TARGET_MIN,       /* start playback percentage of the buffer depth */
        .target_buf_depth               = APP_A2DP_SINK_JITTER_BUFFER_TARGET_MIN,       /* target level percentage of the buffer depth */
        .overrun_control                = WICED_BT_A2DP_SINK_OVERRUN_CONTROL_FLUSH_DATA,/* overrun flow control flag */
        .adj_ppm_max                    = +300,                                         /* Max PPM adjustment value */
        .adj_ppm_min                    = -300,                                         /* Min PPM adjustment value */
        .adj_ppb_per_msec               = 200,                                          /* PPM adjustment per milli second */
        .lvl_correction_threshold_high  = +2000,                                        /* Level correction threshold high value */
        .lvl_correction_threshold_low   = -2000,                                        /* Level correction threshold low value */
        .adj_proportional_gain          = 20,                                           /* Proportional component of total PPM adjustment */
        .adj_integral_gain              = 2,                                            /* Integral component of total PPM adjustment */
    },
    .ext_codec =
    {
#ifdef A2DP_SINK_AAC_ENABLED
        .codec_id        = WICED_AUDIO_CODEC_AAC_DEC,
        .codec_functions = &AAC_codec_function_table,
#else
        .codec_id        = WICED_AUDIO_CODEC_NONE,
        .codec_functions = NULL,
#endif
    }
};

/* It needs 14728 bytes for HFP(mSBC use mainly) and 14148 bytes for A2DP(jitter buffer use mainly) */
#ifdef A2DP_SINK_AAC_ENABLED
#define AUDIO_BUF_SIZE_MAIN                 10852
#else
#define AUDIO_BUF_SIZE_MAIN                 (15 * 1024)
#endif

#ifdef A2DP_SINK_AAC_ENABLED
/* For AAC, audio codec memory requires 21248 bytes and sample buffer required 2 * 4 1024 bytes */
#define AUDIO_BUF_SIZE_CODEC                (21248 + (2 * 4 * 1024))
#else
/* SBC audio codec memory requires 6908 bytes */
#define AUDIO_BUF_SIZE_CODEC                6908
#endif

#define AUDIO_CODEC_BUFFER_SIZE             (AUDIO_BUF_SIZE_MAIN + AUDIO_BUF_SIZE_CODEC)

/**  Audio buffer configuration configuration */
const wiced_bt_audio_config_buffer_t wiced_bt_audio_buf_config =
{
    .role                       =   WICED_AUDIO_SINK_ROLE | WICED_HF_ROLE,
    .audio_tx_buffer_size       =   0,
    .audio_codec_buffer_size    =   AUDIO_CODEC_BUFFER_SIZE,
};

/*
 * wiced_app_cfg_sdp_record_get_size
 */
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return (uint16_t)sizeof(headset_sdp_db);
}

/*
 * wiced_app_cfg_buf_pools_get_num
 */
const wiced_bt_cfg_settings_t *wiced_app_cfg_get_settings(void)
{
    return &wiced_bt_cfg_settings;
}


/*
 * wiced_app_cfg_buf_pools_get_num
 */
int wiced_app_cfg_buf_pools_get_num(void)
{
    return (int)sizeof(wiced_app_cfg_buf_pools)/sizeof(wiced_app_cfg_buf_pools[0]);
}

/*
 * wiced_app_cfg_buf_pools_get
 */
const wiced_bt_cfg_buf_pool_t *wiced_app_cfg_buf_pools_get(void)
{
    return wiced_app_cfg_buf_pools;
}
