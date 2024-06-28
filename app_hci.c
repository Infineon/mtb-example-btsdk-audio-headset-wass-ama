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

#include "wiced_bt_dev.h"
#include "wiced_transport.h"
#include "wiced_hal_puart.h"
#include "wiced_gki.h"
#include "wiced_memory.h"
#include "hcidefs.h"
#include "hci_control_api.h"
#include "app_hci.h"
#include "app_trace.h"
#include "app_nvram.h"
#include "wiced_platform.h"
#include "wiced_platform_audio_common.h"

/*
 * Definitions
 */
#define TRANS_UART_BUFFER_SIZE              1024
#define NB_ELEMENT(a)                       (sizeof(a)/ sizeof(a[0]))

#define HCI_VSC_LRAC_META_OPCODE            (HCI_GRP_VENDOR_SPECIFIC | 0x01CF)
#define HCI_VSE_LRAC_META_EVENT             0x86

/*
 * Local functions
 */
static uint32_t app_hci_proc_rx_cmd(uint8_t *p_data, uint32_t length);
static void app_hci_packet_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
static void app_hci_device_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len);
static void app_hci_handle_trace_enable(uint8_t *p_data);
static void app_hci_handle_read_buffer_stats(void);
static void app_hci_nvram_write(uint8_t *p_data, uint16_t length);
static void app_hci_transport_status(wiced_transport_type_t type);
static void app_hci_packet_decode(wiced_bt_hci_trace_type_t type, uint8_t *p_data, uint16_t length);
static void app_hci_packet_decode_event(uint8_t *p_data, uint16_t length);
static void app_hci_packet_decode_command(uint8_t *p_data, uint16_t length);
static void app_hci_packet_decode_acl( wiced_bt_hci_trace_type_t type, uint8_t *p_data,
        uint16_t length);
static char *app_hci_cmd_get_opcode_desc(uint16_t opcode);

/*
 * External functions
 */
typedef struct __attribute__((packed)) DYNAMIC_MEMORY_POOL_STATS
{
    uint16_t block_size;
    uint16_t block_count;
    uint16_t low_water_mark;
} DYNAMIC_MEMORY_POOL_STATS_t;

typedef struct DYNAMIC_MEMORY_POOL
{
    struct DYNAMIC_MEMORY_POOL* next;
    uint16_t block_size;
    uint16_t block_count;
    uint8_t *pool_base;
    uint8_t *first_free;
    uint8_t num_free;
    uint8_t die_reserve_count;
    void *notify_first;
    void *notify_last;
    uint16_t low_water_mark;
} DYNAMIC_MEMORY_POOL_t;

extern DYNAMIC_MEMORY_POOL_t g_dynamic_memory_GeneralUsePools[];
extern void dynamic_memory_GetPoolStatistics(DYNAMIC_MEMORY_POOL_t *p_pool,
        DYNAMIC_MEMORY_POOL_STATS_t *p_pool_stats);
extern wiced_debug_uart_types_t wiced_get_debug_uart(void);
extern uint8_t wiced_bt_get_number_of_buffer_pools(void);

/*
 * Global variables
 */
static const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg.uart_cfg =
    {
        .mode = WICED_TRANSPORT_UART_HCI_MODE,
        .baud_rate = HCI_UART_DEFAULT_BAUD,
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size = 0,
        .buffer_count = 0,
    },
    .p_status_handler = app_hci_transport_status,
    .p_data_handler = app_hci_proc_rx_cmd,
    .p_tx_complete_cback = NULL,
};

static wiced_bool_t app_hci_lrac_switch_in_progress = WICED_FALSE;

static const char *app_hci_events_desc[] =
{
        "?? Unknown event (0x00)",              /* 0x00 */
        "HCI_INQUIRY_COMPLETE",                 /* 0X01 */
        "HCI_INQUIRY_RESULT",                   /* 0X02 */
        "HCI_CONNECTION_COMPLETE",              /* 0X03 */
        "HCI_CONNECTION_REQUEST",               /* 0X04 */
        "HCI_DISCONNECTION_COMPLETE",           /* 0X05 */
        "HCI_AUTHENTICATION_COMPLETE",          /* 0X06 */
        "HCI_REMOTE_NAME_REQUEST_COMPLETE",     /* 0X07 */
        "HCI_ENCRYPTION_CHANGE",                /* 0X08 */
        "HCI_CHANGE_CONNECTION_LINK_KEY_COMPLETE", /* 0X09 */
        "HCI_TEMP_LINK_KEY_COMPLETE",           /* 0X0A (10D) */
        "HCI_READ_REMOTE_SUPPORTED_FEATURES_COMPLETE", /* 0XB */
        "HCI_READ_REMOTE_VERSION_INFORMATION_COMPLETE", /* 0XC */
        "HCI_QOS_SETUP_COMPLETE",               /* 0X0D */
        "HCI_COMMAND_COMPLETE",                 /* 0X0E */
        "HCI_COMMAND_STATUS",                   /* 0X0F */
        "HCI_HARDWARE_ERROR",                   /* 0X10 */
        "HCI_FLUSH_OCCURRED",                   /* 0X11 */
        "HCI_ROLE_CHANGE",                      /* 0X12 */
        "HCI_NUMBER_OF_COMPLETED_PACKETS",      /* 0X13 */
        "HCI_MODE_CHANGE",                      /* 0X14 (20D) */
        "HCI_RETURN_LINK_KEYS",                 /* 0X15 */
        "HCI_PIN_CODE_REQUEST",                 /* 0X16 */
        "HCI_LINK_KEY_REQUEST",                 /* 0X17 */
        "HCI_LINK_KEY_NOTIFICATION",            /* 0X18 */
        "HCI_LOOPBACK_COMMAND",                 /* 0X19 */
        "HCI_DATA_BUFFER_OVERFLOW",             /* 0X1A */
        "HCI_MAX_SLOTS_CHANGED",                /* 0X1B */
        "HCI_READ_CLOCK_OFFSET_COMPLETE",       /* 0X1C */
        "HCI_CONNECTION_PACKET_TYPE_CHANGED",   /* 0X1D */
        "HCI_QOS_VIOLATION",                    /* 0X1E (30) */
        "HCI_PAGE_SCAN_MODE_CHANGE",            /* 0X1F */
        "HCI_PAGE_SCAN_REPETITION_MODE_CHANGE", /* 0X20 */
        "HCI_FLOW_SPECIFICATION_COMPLETE",      /* 0X21 */
        "HCI_INQUIRY_RESULT_WITH_RSSI",         /* 0X22 */
        "HCI_READ_REMOTE_EXTENDED_FEATURES_COMPLETE", /* 0X23 */
        "Reserved (0x24)",
        "Reserved (0x25)",
        "Reserved (0x26)",
        "Reserved (0x27)",
        "Reserved (0x28)",                      /* 0x28 (40d) */
        "Reserved (0x29)",
        "Reserved (0x2A)",
        "Reserved (0x2B)",
        "HCI_SYNCHRONOUS_CONNECTION_COMPLETE",  /* 0X2C */
        "HCI_SYNCHRONOUS_CONNECTION_CHANGED",   /* 0X2D */
        "HCI_SNIFF_SUBRATING",                  /* 0X2E */
        "HCI_EXTENDED_INQUIRY_RESULT",          /* 0X2F */
        "HCI_ENCRYPTION_KEY_REFRESH_COMPLETE",  /* 0X30 */
        "HCI_IO_CAPABILITY_REQUEST",            /* 0X31 */
        "HCI_IO_CAPABILITY_RESPONSE",           /* 0X32 (50D) */
        "HCI_USER_CONFIRMATION_REQUEST",        /* 0X33 */
        "HCI_USER_PASSKEY_REQUEST",             /* 0X34 */
        "HCI_REMOTE_OOB_DATA_REQUEST",          /* 0X35 */
        "HCI_SIMPLE_PAIRING_COMPLETE",          /* 0X36 */
        "RESERVED (0X37)",
        "HCI_LINK_SUPERVISION_TIMEOUT_CHANGED", /* 0X38 */
        "HCI_ENHANCED_FLUSH_COMPLETE",          /* 0X39 */
        "RESERVED (0X3A)",
        "HCI_USER_PASSKEY_NOTIFICATION",        /* 0X3B */
        "HCI_KEYPRESS_NOTIFICATION",            /* 0X3C (60D) */
        "REMOTE_HOST_SUPPORTED_FEATURES_NOTIFICATION", /* 0X3D */
        "HCI_BLE_EVENT",                        /* 0X3E */
        "??",                                   /* 0X3F REMOVED FROM SPEC */
        "PHYSICAL_LINK_COMP_EVT",               /* 0X40 */
        "CHANNEL_SELECTED_EVT",                 /* 0X41 */
        "DISC_PHYSICAL_LINK_COMP_EVT",          /* 0X42 */
        "PHY_LINK_LOSS_EARLY_WARNING_EVT",      /* 0X43 */
        "PHY_LINK_RECOVERY_EVT",                /* 0X44 */
        "LOGICAL_LINK_COMP_EVT",                /* 0X45 */
        "DISC_LOGICAL_LINK_COMP_EVT",           /* 0X46 (70D) */
        "FLOW_SPEC_MODIFY_COMP_EVT",            /* 0X47 */
        "NUM_COMPL_DATA_BLOCKS_EVT",            /* 0X48 */
        "??",                                   /* 0X49 */
        "??",                                   /* 0X4A */
        "??",                                   /* 0X4B */
        "SHORT_RANGE_MODE_COMPLETE_EVT",        /* 0X4C */
        "AMP_STATUS_CHANGE_EVT",                /* 0X4D */
        "??",                                   /* 0X4E */
        "HCI_SYNC_TRAIN_CPLT_EVT",              /* 0X4F */
        "HCI_SYNC_TRAIN_RECEIVED_EVT",          /* 0X50 (80D) */
        "HCI_CLB_RX_DATA_EVT",                  /* 0X51 */
        "HCI_CLB_RX_TIMEOUT_EVT",               /* 0X52 */
        "HCI_TRUNC_PAGE_CPTL_EVT",              /* 0X53 */
        "HCI_PERIPHERAL_PAGE_RSP_TIMEOUTEVT",   /* 0X54 */
        "HCI_CLB_CHANNEL_CHANGE_EVT"            /* 0X55 */
};

static const char *app_hci_lrac_sub_opcode_desc[] =
{
    "GET_ACL_EAVESDROPPING_PARAMS",             // 0
    "SETANDENABLE_ACL_EAVESDROPPING",           // 1
    "PAUSE_LINK",                               // 2
    "ASSOCIATE_AP_PS",                          // 3
    "STOP_EAVESDROPPING",                       // 4
    "FW_STATISTICS",                            // 5
    "GET_SCO_EAVESDROPPING_PARAMS",             // 6
    "SETANDENABLE_SCO_EAVESDROPPING",           // 7
    "PS_SWITCH_START",                          // 8
    "PS_SWITCH_PARAM_GET",                      // 9
    "PS_SWITCH_EXECUTE",                        // 10
    "PS_SWITCH_FINALIZE",                       // 11
    "PS_SWITCH_ABORT",                          // 12
};
/*
 * app_hci_init
 */
wiced_result_t app_hci_init(void)
{
    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    /* Set the UART type as WICED_ROUTE_DEBUG_TO_PUART */
    wiced_hal_puart_init();
    // wiced_hal_puart_configuration(921600, PARITY_NONE, STOP_BIT_1);
    wiced_hal_puart_configuration(3000000, PARITY_NONE, STOP_BIT_2);
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#ifdef HCI_TRACE_OVER_TRANSPORT
    wiced_bt_dev_register_hci_trace(app_hci_packet_cback);
#endif
#endif
    return WICED_BT_SUCCESS;
}

/*
 * app_hci_lrac_switch_in_progress_set
 */
void app_hci_lrac_switch_in_progress_set(void)
{
    app_hci_lrac_switch_in_progress = WICED_TRUE;
}

/*
 * app_hci_lrac_switch_result
 */
void app_hci_lrac_switch_result(wiced_bt_lrac_switch_result_t status,
        uint8_t local_abort, uint8_t fatal_error)
{
    app_hci_lrac_switch_result_evt_t evt;

    if (!app_hci_lrac_switch_in_progress)
    {
        return;
    }

    WICED_BT_TRACE("lrac_switch status:%d local_abort:%d fatal_error:%d\n", status,
            local_abort, fatal_error);
    evt.status = status;
    evt.local_abort = local_abort;
    evt.fatal_error = fatal_error;
    wiced_transport_send_data(HCI_PLATFORM_EVENT_LRAC_SWITCH_RESULT, (uint8_t *)&evt,
            sizeof(evt));

    app_hci_lrac_switch_in_progress = WICED_FALSE;
}

/*
 *  Process all HCI packet received from the Bluetooth stack
 */
static void app_hci_packet_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    wiced_debug_uart_types_t debug_uart_route;

    debug_uart_route = wiced_get_debug_uart();

    if (debug_uart_route == WICED_ROUTE_DEBUG_TO_WICED_UART)
    {
        // send the trace over HCI
        wiced_transport_send_hci_trace( NULL, type, length, p_data  );
    }
    else if (debug_uart_route == WICED_ROUTE_DEBUG_TO_PUART)
    {
        app_hci_packet_decode(type, p_data, length);
    }
}

/*
 * Handle received command over UART. Please refer to the WICED Smart Ready
 * Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
 * HCI UART control protocol.
*/
static uint32_t app_hci_proc_rx_cmd( uint8_t *p_buffer, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint8_t  buffer_processed = WICED_TRUE;

    if ( !p_buffer )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if(( length < 4 ) || (p_data == NULL))
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer( p_buffer );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    APP_TRACE_DBG("OpCode:0x%04X\n", opcode);

    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        app_hci_device_handle_command(opcode, p_data, payload_len);
        break;

    case HCI_PLATFORM_GROUP:
        platform_handle_hci_command(opcode, p_data, payload_len);
        break;

#if 0
    case HCI_CONTROL_GROUP_LE:
    case HCI_CONTROL_GROUP_GATT:
        app_hci_le_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_AUDIO_SINK:
        app_hci_audio_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_AVRC_TARGET:
        //app_hci_avrc_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_AVRC_CONTROLLER:
        app_hci_avrc_handle_ctrlr_command(opcode, p_data, payload_len);
        break;

    case HCI_CONTROL_GROUP_HF:
        app_hci_hf_handle_command ( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_MISC:
        app_hci_misc_handle_command( opcode, p_data, payload_len );
        break;
#endif

    default:
        break;
    }
    if (buffer_processed)
    {
        // Freeing the buffer in which data is received
        wiced_transport_free_buffer( p_buffer );
    }

    return HCI_CONTROL_STATUS_SUCCESS;
}
/*
 * app_hci_device_handle_command
 */
static void app_hci_device_handle_command(uint16_t cmd_opcode, uint8_t* p_data,
        uint32_t data_len)
{
    uint8_t bytes_written;

    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        app_hci_handle_trace_enable(p_data);
        break;

    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        app_hci_nvram_write(p_data, data_len);
        break;

    case HCI_CONTROL_COMMAND_RESET:
        APP_TRACE_DBG("Reset!!!\n");
        {
            uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
            wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &status,
                    sizeof(status));
        }
        /* Generate a watchdog Reset */
        wdog_generate_hw_reset();
        break;

    case HCI_CONTROL_COMMAND_READ_BUFF_STATS:
        app_hci_handle_read_buffer_stats();
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
    case HCI_CONTROL_COMMAND_INQUIRY:
    case HCI_CONTROL_COMMAND_SET_VISIBILITY:
    case  HCI_CONTROL_COMMAND_SET_PAIRING_MODE:
        APP_TRACE_DBG("opcode %X unhandled\n");
        break;
    default:
        APP_TRACE_ERR("opcode %X unknown\n");
        break;
    }
}

/*
 * handle command from UART to configure traces
 */
static void app_hci_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t) *p_data;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;

    APP_TRACE_DBG("HCI Traces:%d DebugRoute:%d\n", hci_trace_enable, route_debug);

#ifdef HCI_TRACE_OVER_TRANSPORT
    if (hci_trace_enable)
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace(app_hci_packet_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace(NULL);
    }
#endif

    wiced_set_debug_uart(route_debug);

    wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &status, sizeof(status));
}

/*
 *  Handle read buffer statistics
 */
static void app_hci_handle_read_buffer_stats(void)
{
    wiced_bt_buffer_statistics_t buff_stats[wiced_bt_get_number_of_buffer_pools()];
    wiced_result_t result;
    uint8_t i;

    result = wiced_bt_get_buffer_usage(buff_stats, sizeof(buff_stats));

    if (result == WICED_BT_SUCCESS)
    {
        // Print out the stats to trace
        WICED_BT_TRACE( "Buffer usage statistics:\n");

        for(i = 0 ; i < wiced_bt_get_number_of_buffer_pools() ; i++)
        {
            WICED_BT_TRACE("pool_id:%d size:%d curr_cnt:%d max_cnt:%d total:%d\n",
                           buff_stats[i].pool_id, buff_stats[i].pool_size,
                           buff_stats[i].current_allocated_count,
                           buff_stats[i].max_allocated_count,
                           buff_stats[i].total_count);
        }

        /* Return the statistics via WICED-HCI */
        wiced_transport_send_data(HCI_CONTROL_EVENT_READ_BUFFER_STATS, (uint8_t *)&buff_stats,
                sizeof(buff_stats));
    }
    else
    {
        i = HCI_CONTROL_STATUS_FAILED;
        wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &i, 1);
    }
}

/*
 * app_hci_nvram_write
 */
static void app_hci_nvram_write(uint8_t *p_data, uint16_t length)
{
    uint16_t nvram_id;
    wiced_result_t status;
    uint8_t cmd_status;

    if (length < 2)
    {
        APP_TRACE_ERR("command to short");
        cmd_status = 1;
        wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &cmd_status, sizeof(cmd_status));
        return;
    }
    STREAM_TO_UINT16(nvram_id, p_data);

    APP_TRACE_DBG("nvram_id:0x%x length:%d\n", nvram_id, length - 2);

    status = app_nvram_write(nvram_id, p_data, length - 2);
    if (status != WICED_BT_SUCCESS)
        cmd_status = 1;
    else
        cmd_status = 0;

    wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &cmd_status, sizeof(cmd_status));
}

/*
 * app_hci_transport_status
 * This function is called when the MCU opens the HCI UART
 */
static void app_hci_transport_status( wiced_transport_type_t type )
{
    APP_TRACE_DBG("app_hci_transport_status %x \n", type);
    wiced_transport_send_data(HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0);
}

/*
 * app_hci_cmd_get_opcode_desc
 */
static char *app_hci_cmd_get_opcode_desc(uint16_t opcode)
{
    switch(opcode)
    {
    case HCI_INQUIRY: return "HCI_INQUIRY";
    case HCI_CREATE_CONNECTION: return"HCI_CREATE_CONNECTION";
    case HCI_ADD_SCO_CONNECTION: return "HCI_ADD_SCO_CONNECTION";
    case HCI_ACCEPT_CONNECTION_REQUEST: return "HCI_ACCEPT_CONNECTION_REQUEST";
    case HCI_REJECT_CONNECTION_REQUEST: return "HCI_REJECT_CONNECTION_REQUEST";
    case HCI_LINK_KEY_REQUEST_REPLY: return "HCI_LINK_KEY_REQUEST_REPLY";
    case HCI_LINK_KEY_REQUEST_NEG_REPLY: return "HCI_LINK_KEY_REQUEST_NEG_REPLY";
    case HCI_AUTHENTICATION_REQUESTED: return "HCI_AUTHENTICATION_REQUESTED";
    case HCI_SET_CONN_ENCRYPTION: return "HCI_SET_CONN_ENCRYPTION";
    case HCI_SETUP_ESCO_CONNECTION: return "HCI_SETUP_ESCO_CONNECTION";
    case HCI_ACCEPT_ESCO_CONNECTION: return "HCI_ACCEPT_ESCO_CONNECTION";
    case HCI_REJECT_ESCO_CONNECTION: return "HCI_REJECT_ESCO_CONNECTION";
    case HCI_ENH_SETUP_ESCO_CONNECTION: return "HCI_ENH_SETUP_ESCO_CONNECTION";
    case HCI_ENH_ACCEPT_ESCO_CONNECTION: return "HCI_ENH_ACCEPT_ESCO_CONNECTION";
    case HCI_SNIFF_MODE: return "HCI_SNIFF_MODE";
    case HCI_EXIT_SNIFF_MODE: return "HCI_EXIT_SNIFF_MODE";
    case HCI_SWITCH_ROLE: return "HCI_SWITCH_ROLE";
    case HCI_WRITE_POLICY_SETTINGS: return "HCI_WRITE_POLICY_SETTINGS";
    case HCI_RESET: return "HCI_RESET";
    case HCI_CHANGE_LOCAL_NAME: return "HCI_CHANGE_LOCAL_NAME";
    case HCI_WRITE_SCAN_ENABLE: return "HCI_WRITE_SCAN_ENABLE";
    case HCI_WRITE_VOICE_SETTINGS: return "HCI_WRITE_VOICE_SETTINGS";
    case (HCI_GRP_VENDOR_SPECIFIC | 0x0001): return "HCI_VSC_SET_LOCAL_BDADDR";
    case HCI_VSC_LRAC_META_OPCODE: return "HCI_VSC_LRAC_META_OPCODE";
    case (HCI_GRP_VENDOR_SPECIFIC | 0x011A): return "HCI_VSC_WRITE_A2DP_CONNECTION";
    case (HCI_GRP_VENDOR_SPECIFIC | 0x012F): return "HCI_VSC_SET_TX_POWER_RANGE";
    default: return "Undecoded";
    }
}

/*
 * app_hci_packet_decode
 */
static void app_hci_packet_decode(wiced_bt_hci_trace_type_t type, uint8_t *p_data, uint16_t length)
{
    switch(type)
    {
    case HCI_TRACE_EVENT: /**< HCI event data from controller to the host */
        app_hci_packet_decode_event(p_data, length);
        break;

    case HCI_TRACE_COMMAND: /**< HCI command data from host to controller */
        app_hci_packet_decode_command(p_data, length);
        break;

    case HCI_TRACE_INCOMING_ACL_DATA:/**< HCI incoming acl data */
    case HCI_TRACE_OUTGOING_ACL_DATA:/**< HCI outgoing acl data */
        app_hci_packet_decode_acl(type, p_data, length);
        break;

    default:
        APP_TRACE_ERR("Unknown type:%d\n", type);
        break;
    }
}

/*
 * app_hci_packet_decode_event
 */
static void app_hci_packet_decode_event(uint8_t *p_data, uint16_t length)
{
    uint8_t event;
    uint8_t hci_len;
    uint16_t opcode;
    uint8_t evt_code;
    uint8_t evt_sub_code;
    uint8_t hci_status;
    uint8_t unused;
    uint8_t bdaddr[BD_ADDR_LEN];
    uint8_t linkkey[LINK_KEY_LEN];
    uint8_t cod[DEV_CLASS_LEN];
    uint8_t role;
    uint8_t link_type;
    uint16_t connection_handle;
    uint8_t reason;

    STREAM_TO_UINT8(event, p_data);
    STREAM_TO_UINT8(hci_len, p_data);

    switch(event)
    {
    case HCI_CONNECTION_COMP_EVT:
        STREAM_TO_UINT8(hci_status, p_data);
        STREAM_TO_UINT16(connection_handle, p_data);
        STREAM_TO_BDADDR(bdaddr, p_data);
        STREAM_TO_UINT8(link_type, p_data);
        WICED_BT_TRACE("RCVT HCI Event:HCI_CONNECTION_COMP_EVT Status:%d ConHdl:0x%x BdAddr:%B LinkType:%d\n",
                hci_status, connection_handle, bdaddr, link_type);
        break;

    case HCI_CONNECTION_REQUEST_EVT:
        STREAM_TO_BDADDR(bdaddr, p_data);
        STREAM_TO_DEVCLASS(cod, p_data);
        STREAM_TO_UINT8(link_type, p_data);
        WICED_BT_TRACE("RCVT HCI Event:HCI_CONNECTION_REQUEST_EVT BdAddr:%B LinkType:%d\n",
                bdaddr, link_type);
        break;

    case HCI_DISCONNECTION_COMP_EVT:
        STREAM_TO_UINT8(hci_status, p_data);
        STREAM_TO_UINT16(connection_handle, p_data);
        STREAM_TO_UINT8(reason, p_data);
        WICED_BT_TRACE("RCVT HCI Event:HCI_DISCONNECTION_COMP_EVT status:0x%x ConHdl:0x%x Reason:0x%x\n",
                hci_status, connection_handle, reason);
        break;

    case HCI_COMMAND_COMPLETE_EVT:
        STREAM_TO_UINT8(unused, p_data);    /* Unused byte (Number Of HCI Command Packet) */
        STREAM_TO_UINT16(opcode, p_data);
        STREAM_TO_UINT8(hci_status, p_data);
        WICED_BT_TRACE("RCVT HCI Event:HCI_COMMAND_COMPLETE_EVT OpCode:%s (0x%04X) Status:%d Len:%d\n",
                app_hci_cmd_get_opcode_desc(opcode), opcode, hci_status, hci_len - 4);
        if (opcode == HCI_VSC_LRAC_META_OPCODE)
        {
            STREAM_TO_UINT8(opcode, p_data);
            if (opcode < NB_ELEMENT(app_hci_lrac_sub_opcode_desc))
                WICED_BT_TRACE(" LRAC SubOpcode:%s(%d)\n", app_hci_lrac_sub_opcode_desc[opcode], opcode);
            else
                WICED_BT_TRACE(" LRAC SubOpcode:%d unknown\n", opcode);
        }
        break;

    case HCI_COMMAND_STATUS_EVT:
        STREAM_TO_UINT8(hci_status, p_data);
        STREAM_TO_UINT8(unused, p_data);    /* Unused byte (Number Of HCI Command Packet) */
        STREAM_TO_UINT16(opcode, p_data);
        WICED_BT_TRACE("RCVT HCI Event:HCI_COMMAND_STATUS_EVT OpCode:%s (0x%04X) Status:%d\n",
                app_hci_cmd_get_opcode_desc(opcode), opcode, hci_status);
        break;

    case HCI_NUM_COMPL_DATA_PKTS_EVT:
        /* Do not Print Number Of Complete Packet events */
#if 1
        STREAM_TO_UINT8(unused, p_data);
        STREAM_TO_UINT16(connection_handle, p_data);
        STREAM_TO_UINT16(opcode, p_data);
        WICED_BT_TRACE("RCVT HCI Event:HCI_NUM_COMPL_DATA_PKTS_EVT nbh:%d cnh:0x%x nb:%d\n",
                unused, connection_handle, opcode);
#endif
        break;

    case HCI_VENDOR_SPECIFIC_EVT:
        STREAM_TO_UINT8(evt_code, p_data);
        /* LRAC VSE are already decoded by the library */
#if 0
        if ((evt_code != 0x86) && (evt_code != 0x1a))
#endif
        {
            STREAM_TO_UINT8(evt_sub_code, p_data);    /* Sub Event */
            WICED_BT_TRACE("RCVT HCI Event:VSE Code:0x%02X SubCode:0x%02X Len:%d\n",
                    evt_code, evt_sub_code, hci_len - 2);
        }
        break;

    case HCI_LINK_KEY_NOTIFICATION_EVT:
        STREAM_TO_BDADDR(bdaddr, p_data);
        REVERSE_STREAM_TO_ARRAY(linkkey, p_data, LINK_KEY_LEN);
        WICED_BT_TRACE("RCVT HCI LINK_KEY Notification BdAddr:%B\n", bdaddr);
        WICED_BT_TRACE(" LinkKey:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                linkkey[0], linkkey[1], linkkey[2], linkkey[3],
                linkkey[4], linkkey[5], linkkey[6], linkkey[7],
                linkkey[8], linkkey[9], linkkey[10], linkkey[11],
                linkkey[12], linkkey[13], linkkey[14], linkkey[15]);
        break;

    case HCI_ROLE_CHANGE_EVT:
        STREAM_TO_UINT8(hci_status, p_data);
        STREAM_TO_BDADDR(bdaddr, p_data);
        STREAM_TO_UINT8(role, p_data);
        WICED_BT_TRACE("RCVT HCI HCI_ROLE_CHANGE_EVT Status:%d BdAddr:%B role:%s(%d)\n",
                hci_status, bdaddr, role==0?"Central":"Peripheral", role);
        break;

    default:
        if (event < (sizeof(app_hci_events_desc) / sizeof(char *)))
        {
            WICED_BT_TRACE("RCVT HCI Event:%s (0x%02X) Len:%d\n",
                    app_hci_events_desc[event], event, hci_len);
        }
        else
        {
            WICED_BT_TRACE("RCVT HCI Event:??? (0x%02X) Len:%d\n", event, hci_len);
        }
        break;
    }
}

/*
 * app_hci_packet_decode_command
 */
static void app_hci_packet_decode_command(uint8_t *p_data, uint16_t length)
{
    uint16_t opcode;
    uint8_t hci_len;
    uint8_t bdaddr[BD_ADDR_LEN];
    uint8_t linkkey[LINK_KEY_LEN];
    uint8_t hci_status;
    uint8_t role;
    uint8_t scan_enable;
    uint16_t conn_handle;
    uint16_t sniff_max_interval;
    uint16_t sniff_min_interval;
    uint16_t sniff_attempt;
    uint16_t sniff_timeout;
    STREAM_TO_UINT16(opcode, p_data);
    STREAM_TO_UINT8(hci_len, p_data);

    WICED_BT_TRACE("SENT HCI Cmd OpCode:%s (0x%04X) Len:%d\n",
            app_hci_cmd_get_opcode_desc(opcode), opcode, hci_len);

    switch(opcode)
    {
    case HCI_CREATE_CONNECTION:
        STREAM_TO_BDADDR(bdaddr, p_data);
        WICED_BT_TRACE(" BdAddr:%B\n", bdaddr);
        break;

    case HCI_LINK_KEY_REQUEST_REPLY:
        STREAM_TO_BDADDR(bdaddr, p_data);
        REVERSE_STREAM_TO_ARRAY(linkkey, p_data, LINK_KEY_LEN);
        WICED_BT_TRACE(" LINK_KEY Reply BdAddr:%B\n", bdaddr);
        WICED_BT_TRACE(" LinkKey:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                linkkey[0], linkkey[1], linkkey[2], linkkey[3], linkkey[4], linkkey[5], linkkey[6], linkkey[7],
                linkkey[8], linkkey[9], linkkey[10], linkkey[11], linkkey[12], linkkey[13], linkkey[14], linkkey[15]);
        break;

    case HCI_SWITCH_ROLE:
        STREAM_TO_BDADDR(bdaddr, p_data);
        STREAM_TO_UINT8(role, p_data);
        WICED_BT_TRACE(" BdAddr:%B role:%s(%d)\n", bdaddr, role==0?"Central":"Peripheral", role);
        break;

    case HCI_WRITE_SCAN_ENABLE:
        STREAM_TO_UINT8(scan_enable, p_data);
        WICED_BT_TRACE(" scan_enable:%d\n", scan_enable);
        break;

    case HCI_SNIFF_MODE:
        STREAM_TO_UINT16(conn_handle, p_data);
        STREAM_TO_UINT16(sniff_max_interval, p_data);
        STREAM_TO_UINT16(sniff_min_interval, p_data);
        STREAM_TO_UINT16(sniff_attempt, p_data);
        STREAM_TO_UINT16(sniff_timeout, p_data);
        WICED_BT_TRACE(" ConHdl:0x%x Max:%d Min:%d Attempt:%d Timeout:%d\n", conn_handle,
                sniff_max_interval, sniff_min_interval, sniff_attempt, sniff_timeout);
        break;

    case HCI_VSC_LRAC_META_OPCODE:                     /* LRAC Meta Command */
        STREAM_TO_UINT8(opcode, p_data);        /* Extract LRAC Sub OpCode */
        if (opcode < NB_ELEMENT(app_hci_lrac_sub_opcode_desc))
            WICED_BT_TRACE(" LRAC SubOpcode:%s(%d)\n",
                    app_hci_lrac_sub_opcode_desc[opcode], opcode);
        else
            WICED_BT_TRACE(" LRAC SubOpcode:%d unknown\n", opcode);
        break;

    default:
        break;
    }
}

/*
 * app_hci_packet_decode_acl
 */
static void app_hci_packet_decode_acl( wiced_bt_hci_trace_type_t type, uint8_t *p_data,
        uint16_t length)
{
    uint16_t conn_handle;
    uint8_t packet_boundary;
    uint16_t hci_len;
    uint16_t l2c_len;
    uint16_t l2c_cid;

    STREAM_TO_UINT16(conn_handle, p_data);
    STREAM_TO_UINT16(hci_len, p_data);

    packet_boundary = (conn_handle >> 12) & 0x03;

    /* Continuing packet */
    if (packet_boundary == 1)
    {
        if (type == HCI_TRACE_INCOMING_ACL_DATA)
            WICED_BT_TRACE("RCVT HCI ACL ConHdl:0x%x Len:%d Continue\n", conn_handle, hci_len);
        else
            WICED_BT_TRACE("SENT HCI ACL ConHdl:0x%x Len:%d Continue\n", conn_handle, hci_len);
    }
    /* First Flushable or First Non Flushable or Complete Flushable packet */
    else
    {
        /* Extract L2CAP Header (Length & CID) */
        STREAM_TO_UINT16(l2c_len, p_data);
        STREAM_TO_UINT16(l2c_cid, p_data);
        if (type == HCI_TRACE_INCOMING_ACL_DATA)
            WICED_BT_TRACE("RCVT HCI ACL ConHdl:0x%x Len:%d Start(%d) L2C-CID:0x%x L2C-Len:%d\n",
                    conn_handle, hci_len, packet_boundary, l2c_cid, l2c_len);
        else
            WICED_BT_TRACE("SENT HCI ACL ConHdl:0x%x Len:%d Start(%d) L2C-CID:0x%x L2C-Len:%d\n",
                    conn_handle, hci_len, packet_boundary, l2c_cid, l2c_len);
    }
}
