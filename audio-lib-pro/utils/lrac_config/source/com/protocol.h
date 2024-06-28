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

typedef enum
{
    PROTOCOL_EVENT_DISCONNECT = 0,
    PROTOCOL_EVENT_RX_WICED_EVENT,
    PROTOCOL_EVENT_RX_HCI_EVENT,
    PROTOCOL_EVENT_RX_HCI_ACL,
    PROTOCOL_EVENT_RX_HCI_SCO
} protocol_event_t;

typedef enum
{
    PROTOCOL_TYPE_WICED = 0,
    PROTOCOL_TYPE_HCI_CMD,
    PROTOCOL_TYPE_HCI_ACL,
    PROTOCOL_TYPE_HCI_SCO,
    PROTOCOL_TYPE_HCI_DIAG
} protocol_type_t;


typedef void (protocol_callback_t)(protocol_event_t event,
        uint16_t id, uint8_t *p_data, int data_len);

int protocol_init(void);
int protocol_open(char *p_device, int baudrate, int flow_control, protocol_callback_t *p_callback);
int protocol_set_baudrate(int baudrate, int flow_control);
int protocol_close(void);
int protocol_send(protocol_type_t type, uint16_t id, uint8_t *p_data,
        int data_len);
