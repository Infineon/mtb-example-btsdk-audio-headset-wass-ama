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

#include <stdint.h>
#include "wiced.h"
#include "wiced_bt_lrac.h"

/*
 * Group codes
 */
#define HCI_PLATFORM_GROUP                      0xD0

/*
 * Platform (Customer specific) Group Commands
 */
/* Button Simulation */
#define HCI_PLATFORM_COMMAND_BUTTON             ((HCI_PLATFORM_GROUP << 8) | 0x20)
/* Audio Insertion Simulation */
#define HCI_PLATFORM_COMMAND_AUDIO_INSERT       ((HCI_PLATFORM_GROUP << 8) | 0x21)
/* Audio Insertion Simulation */
#define HCI_PLATFORM_COMMAND_BLE_ADV            ((HCI_PLATFORM_GROUP << 8) | 0x22)
/* LRAC Switch Simulation */
#define HCI_PLATFORM_COMMAND_LRAC_SWITCH        ((HCI_PLATFORM_GROUP << 8) | 0x23)
/* LRAC Set Trace Level */
#define HCI_PLATFORM_COMMAND_LRAC_TRACE_LEVEL   ((HCI_PLATFORM_GROUP << 8) | 0x24)
/* HCI VSC Wrapper */
#define HCI_PLATFORM_COMMAND_VSC_WRAPPER        ((HCI_PLATFORM_GROUP << 8) | 0x25)
/* HCI Check AP Connection */
#define HCI_PLATFORM_COMMAND_AP_CONN_CHECK      ((HCI_PLATFORM_GROUP << 8) | 0x26)
/* Jitter Buffer Set */
#define HCI_PLATFORM_COMMAND_JITTER_TARGET_SET  ((HCI_PLATFORM_GROUP << 8) | 0x28)
/* eLNA Gain Set */
#define HCI_PLATFORM_COMMAND_ELNA_GAIN_SET      ((HCI_PLATFORM_GROUP << 8) | 0x29)
/* Embedded Flash Erase */
#define HCI_PLATFORM_COMMAND_EF_ERASE           ((HCI_PLATFORM_GROUP << 8) | 0x30)
/* Embedded Flash Write */
#define HCI_PLATFORM_COMMAND_EF_WRITE           ((HCI_PLATFORM_GROUP << 8) | 0x32)
/* Audio Insertion Extended Simulation */
#define HCI_PLATFORM_COMMAND_AUDIO_INSERT_EXT   ((HCI_PLATFORM_GROUP << 8) | 0x33)

/*
 * Platform (Customer specific) Group Events
 */
/* Command status event for the requested operation */
#define HCI_PLATFORM_EVENT_LRAC_SWITCH_RESULT   ((HCI_PLATFORM_GROUP << 8) | 0x23)
/* VSC Wrapper Command Complete event */
#define HCI_PLATFORM_EVENT_VSC_CMD_CPLT         ((HCI_PLATFORM_GROUP << 8) | 0x25)
/* Command status event for the requested operation */
#define HCI_PLATFORM_EVENT_COMMAND_STATUS       ((HCI_PLATFORM_GROUP << 8) | 0xFF)

/*
 * Event structure
 */
typedef struct __attribute__((packed)) app_hci_lrac_switch_result_evt
{
    wiced_bt_lrac_switch_result_t   status;
    uint8_t                         local_abort;
    uint8_t                         fatal_error;
} app_hci_lrac_switch_result_evt_t;

/*
 * app_hci_init
 */
wiced_result_t app_hci_init(void);

/*
 * app_hci_lrac_switch_in_progress_set
 */
void app_hci_lrac_switch_in_progress_set(void);

/*
 * app_hci_lrac_switch_result
 */
void app_hci_lrac_switch_result(wiced_bt_lrac_switch_result_t status,
        uint8_t local_abort, uint8_t fatal_error);
