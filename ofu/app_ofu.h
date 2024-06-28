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
 *
 *  OFU (OTA FW Upgrade) API Implementation.
 */

#pragma once

#include "wiced.h"
#include "app_trace.h"

/*
 * definitions
 */
#ifdef APP_OFU_DEBUG
#define APP_OFU_TRACE_DBG              APP_TRACE_DBG
#else
#define APP_OFU_TRACE_DBG(...)
#endif

/* SPP Over The Air Firmware Upgrade (OFU) Server channel */
#define APP_OFU_SPP_RFCOMM_SCN              2

#define APP_OFU_CONTROL_COMMAND             1
#define APP_OFU_DATA                        2
#define APP_OFU_EVENT                       3

#define APP_OFU_HDR_TYPE_GET(a)             (a >> 4)
#define APP_OFU_HDR_CMD_GET(a)              (a & 0x0F)
#define APP_OFU_HDR_STS_GET(a)              (a & 0x0F)
#define APP_OFU_HDR_SET(type, value)        ((type << 4) | (value & 0x0F))

typedef enum
{
    APP_OFU_EVENT_STARTED,
    APP_OFU_EVENT_ABORTED,
    APP_OFU_EVENT_COMPLETED,
} app_ofu_event_t;

typedef enum
{
    APP_OFU_TRANSPORT_SPP_SERVER = 0,
    APP_OFU_TRANSPORT_BLE_SERVER,
    APP_OFU_TRANSPORT_LRAC_SERVER,
    APP_OFU_TRANSPORT_LRAC_CLIENT
} app_ofu_transport_t;

typedef struct
{
    app_ofu_transport_t transport;
} app_ofu_event_data_started_t;

typedef app_ofu_event_data_started_t app_ofu_event_data_aborted_t;
typedef app_ofu_event_data_started_t app_ofu_event_data_completed_t;

typedef union
{
    app_ofu_event_data_started_t started;
    app_ofu_event_data_aborted_t aborted;
    app_ofu_event_data_completed_t completed;
} app_ofu_event_data_t;

typedef void (app_ofu_callback_t)(app_ofu_event_t event, app_ofu_event_data_t *p_data);

/*
 * app_ofu_init
 */
wiced_result_t app_ofu_init(app_ofu_callback_t *p_callback);

/*
 * app_ofu_ready_to_switch
 */
wiced_bool_t app_ofu_ready_to_switch(void);

/*
 * app_ofu_switch_get
 */
wiced_result_t app_ofu_switch_get(void *p_opaque, uint16_t *p_sync_data_len);

/*
 * app_ofu_switch_set
 */
wiced_result_t app_ofu_switch_set(void *p_opaque, uint16_t sync_data_len);
