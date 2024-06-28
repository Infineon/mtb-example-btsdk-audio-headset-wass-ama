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

#include <stdio.h>
#include <string.h>
#include <pthread.h>

#include "protocol.h"

#include "hci.h"
#include "wiced.h"
#include "serial.h"
#include "utils.h"

/*
 * Definitions
 */
#define MIN(a,b)    (a > b ? b : a)

#define PROTOCOL_HCI_CMD_PKT                0x01
#define PROTOCOL_HCI_ACL_PKT                0x02
#define PROTOCOL_HCI_SCO_PKT                0x03
#define PROTOCOL_HCI_EVT_PKT                0x04
#define PROTOCOL_WICED_PKT                  0x19

#define PROTOCOL_HEADER_SIZE                (1 + 2 + 2)
#define PROTOCOL_PAYLOAD_SIZE               1024

#define PROTOCOL_CMD_MAX_LEN                (PROTOCOL_HEADER_SIZE + PROTOCOL_PAYLOAD_SIZE)
#define PROTOCOL_EVT_MAX_LEN                (PROTOCOL_HEADER_SIZE + PROTOCOL_PAYLOAD_SIZE)

typedef enum
{
    IDLE = 0,

    HCI_CMD_W4_OP_L,
    HCI_CMD_W4_OP_H,
    HCI_CMD_W4_LEN,
    HCI_CMD_W4_DATA,

    HCI_ACL_W4_CH_L,
    HCI_ACL_W4_CH_H,
    HCI_ACL_W4_LEN_L,
    HCI_ACL_W4_LEN_H,
    HCI_ACL_W4_DATA,

    HCI_SCO_W4_CH_L,
    HCI_SCO_W4_CH_H,
    HCI_SCO_W4_LEN,
    HCI_SCO_W4_DATA,

    HCI_EVT_W4_EVT,
    HCI_EVT_W4_LEN,
    HCI_EVT_W4_DATA,

    WICED_W4_CMD,
    WICED_W4_GROUP,
    WICED_W4_LEN_L,
    WICED_W4_LEN_H,
    WICED_W4_DATA,
} protocol_state_t;

typedef struct
{
    protocol_callback_t *p_callback;
    protocol_state_t rx_state;
    uint8_t rx_data[PROTOCOL_PAYLOAD_SIZE];
    uint16_t opcode;
    uint8_t event;
    uint16_t con_hdl;
    uint16_t length;
    uint16_t data_counter;
} protocol_cb_t;

/*
 * Global variables
 */
static protocol_cb_t protocol_cb;

/*
 * Local functions
 */
static void protocol_cback (protocol_event_t event, uint16_t id, uint8_t *p_data,
        int data_len);
static void protocol_serial_cback(serial_event_t event, uint8_t *p_data, int data_len);

/*
 * protocol_init
 */
int protocol_init(void)
{
    memset(&protocol_cb, 0, sizeof(protocol_cb));

    return serial_init();
}

/*
 * protocol_open
 */
int protocol_open(char *p_device, int baudrate, int flow_control, protocol_callback_t *p_callback)
{
    int status;

    if (protocol_cb.p_callback != NULL)
    {
        TRACE_ERR("already opened");
        return -1;
    }

    status = serial_open(p_device, baudrate, flow_control, protocol_serial_cback);
    if (status < 0)
    {
        TRACE_ERR("serial_open failed");
        return -1;
    }
    protocol_cb.p_callback = p_callback;
    protocol_cb.rx_state = IDLE;

    return 0;

}

/*
 * protocol_set_baudrate
 */
int protocol_set_baudrate(int baudrate, int flow_control)
{
    return (serial_set_baudrate(baudrate, flow_control));
}

/*
 * protocol_close
 */
int protocol_close(void)
{
    if (protocol_cb.p_callback == NULL)
    {
        TRACE_ERR("not opened");
        return -1;
    }

    serial_close();

    return protocol_init();
}

/*
 * protocol_send
 */
int protocol_send(protocol_type_t protocol, uint16_t id, uint8_t *p_data,
        int data_len)
{
    uint8_t cmd[PROTOCOL_CMD_MAX_LEN];
    uint8_t *p = cmd;
    int rv;

    TRACE_DBG("protocol:%d id:0x%x len:%d", protocol, id, data_len);

    if (protocol > PROTOCOL_TYPE_HCI_DIAG)
    {
        TRACE_ERR("bad protocol type:%d", protocol);
        return -1;
    }

    if (protocol == PROTOCOL_TYPE_WICED)
    {
        UINT8_TO_STREAM(p, PROTOCOL_WICED_PKT);
        UINT16_TO_STREAM(p, id);
        UINT16_TO_STREAM(p, data_len);
    }
    else if (protocol == PROTOCOL_TYPE_HCI_CMD)
    {
        UINT8_TO_STREAM(p, PROTOCOL_HCI_CMD_PKT);
        UINT16_TO_STREAM(p, id);
        UINT8_TO_STREAM(p, data_len);
    }
    else if (protocol == PROTOCOL_TYPE_HCI_ACL)
    {
        UINT8_TO_STREAM(p, PROTOCOL_HCI_ACL_PKT);
        UINT16_TO_STREAM(p, id);
        UINT16_TO_STREAM(p, data_len);
    }
    else if (protocol == PROTOCOL_TYPE_HCI_SCO)
    {
        UINT8_TO_STREAM(p, PROTOCOL_HCI_SCO_PKT);
        UINT16_TO_STREAM(p, id);
        UINT8_TO_STREAM(p, data_len);
    }
    else if (protocol == PROTOCOL_TYPE_HCI_DIAG)
    {
        UINT8_TO_STREAM(p, 0x07);
        UINT8_TO_STREAM(p, id);
    }
    else
    {
        return -1;
    }

    if (data_len > 0)
    {
        ARRAY_TO_STREAM(p, p_data, data_len);
    }

    rv = serial_write(cmd, p - cmd);
    if (rv < 0)
    {
        return rv;
    }

    return rv;
}

/*
 * protocol_cback
 */
static void protocol_cback (protocol_event_t event, uint16_t id, uint8_t *p_data,
        int data_len)
{
    uint16_t hci_opcode;
    uint8_t *p = p_data;
    int handled = 0;

    switch(event)
    {
    case PROTOCOL_EVENT_RX_HCI_EVENT:
        handled = hci_event_handler((uint8_t)id, p_data, data_len);
        break;
    case PROTOCOL_EVENT_RX_HCI_ACL:
        handled = hci_acl_handler(id, p_data, data_len);
        break;
    case PROTOCOL_EVENT_RX_HCI_SCO:
        handled = hci_sco_handler(id, p_data, data_len);
        break;

    case PROTOCOL_EVENT_RX_WICED_EVENT:
        handled = wiced_event_handler(id, p_data, data_len);
        break;

    default:
        TRACE_ERR("unknown event:%d", event);
        break;
    }

    if ((handled == 0) &&
        (protocol_cb.p_callback))
    {
        protocol_cb.p_callback(event, id, p_data, data_len);
    }
}

/*
 * protocol_serial_cback
 */
static void protocol_serial_cback(serial_event_t event, uint8_t *p_data, int data_len)
{
    uint16_t cpy_len;

    //TRACE_DBG_FULL("len:%d", data_len);
    if (data_len == 0)
        return;

    while(data_len > 0)
    {
        //TRACE_DBG_FULL("byte:%x length:%d", *p_data, data_len);

        switch (protocol_cb.rx_state)
        {
        case IDLE:
            data_len--;
            switch (*p_data++)
            {
            case PROTOCOL_HCI_CMD_PKT:
                protocol_cb.rx_state = HCI_CMD_W4_OP_L;
                //TRACE_DBG_FULL("HCI_CMD");
                break;

            case PROTOCOL_HCI_ACL_PKT:
                protocol_cb.rx_state = HCI_ACL_W4_CH_L;
                //TRACE_DBG_FULL("HCI_ACL");
                break;

            case PROTOCOL_HCI_SCO_PKT:
                protocol_cb.rx_state = HCI_SCO_W4_CH_L;
                //TRACE_DBG_FULL("HCI_SCO");
                break;

            case PROTOCOL_HCI_EVT_PKT:
                protocol_cb.rx_state = HCI_EVT_W4_EVT;
                //TRACE_DBG_FULL("HCI_EVT");
                break;

            case PROTOCOL_WICED_PKT:
                protocol_cb.rx_state = WICED_W4_CMD;
                //TRACE_DBG_FULL("WICED_EVT");
                break;

            default:
                protocol_cb.rx_state = IDLE;
                break;
            }
        break;

        // HCI command Packet
        case HCI_CMD_W4_OP_L:
            data_len--;
            protocol_cb.opcode = *p_data++;
            protocol_cb.rx_state = HCI_CMD_W4_OP_H;
            break;
        case HCI_CMD_W4_OP_H:
            data_len--;
            protocol_cb.opcode |= *p_data++ << 8;
            protocol_cb.rx_state = HCI_CMD_W4_LEN;
            //TRACE_DBG_FULL("HCI Opcode:0x%04X", protocol_cb.opcode);
            break;
        case HCI_CMD_W4_LEN:
            data_len--;
            protocol_cb.length = *p_data++;
            //TRACE_DBG_FULL("HCI CmdLen:%02X", protocol_cb.opcode);
            if (protocol_cb.length == 0)
            {
                protocol_cb.rx_state = IDLE;
                TRACE_ERR("Err: HCI Cmd Received (len:0). Ignored");
            }
            else
            {
                protocol_cb.data_counter = 0;
                protocol_cb.rx_state = HCI_CMD_W4_DATA;
            }
            break;
        case HCI_CMD_W4_DATA:
            cpy_len = MIN(protocol_cb.length, data_len);
            memcpy(&protocol_cb.rx_data[protocol_cb.data_counter], p_data, cpy_len);
            protocol_cb.data_counter += cpy_len;
            p_data += cpy_len;
            data_len -= cpy_len;

            if (protocol_cb.data_counter >= protocol_cb.length)
            {
                protocol_cb.rx_state = IDLE;
                TRACE_ERR("Err: HCI Cmd Received (len:%d. Ignored",
                        protocol_cb.length);
            }
            break;

        // HCI ACL Packet
        case HCI_ACL_W4_CH_L:
            data_len--;
            protocol_cb.con_hdl = *p_data++;
            protocol_cb.rx_state = HCI_ACL_W4_CH_H;
            break;

        case HCI_ACL_W4_CH_H:
            data_len--;
            protocol_cb.con_hdl |= *p_data++ << 8;
            //TRACE_DBG_FULL("HCI_ACL_CON_HDL:0x%04x", protocol_cb.con_hdl);
            protocol_cb.rx_state = HCI_ACL_W4_LEN_L;
            break;

        case HCI_ACL_W4_LEN_L:
            data_len--;
            protocol_cb.length = *p_data++;
            protocol_cb.rx_state = HCI_ACL_W4_LEN_H;
            break;

        case HCI_ACL_W4_LEN_H:
            data_len--;
            protocol_cb.length |= *p_data++ << 8;
            //TRACE_DBG_FULL("HCI_ACL Len:%d", protocol_cb.length);
            if (protocol_cb.length == 0)
            {
                protocol_cb.rx_state = IDLE;
                protocol_cback(PROTOCOL_EVENT_RX_HCI_ACL,
                        protocol_cb.con_hdl, NULL, 0);
            }
            else
            {
                protocol_cb.data_counter = 0;
                protocol_cb.rx_state = HCI_ACL_W4_DATA;
            }
            break;

        case HCI_ACL_W4_DATA:
            cpy_len = MIN(protocol_cb.length, data_len);
            memcpy(&protocol_cb.rx_data[protocol_cb.data_counter], p_data, cpy_len);
            protocol_cb.data_counter += cpy_len;
            p_data += cpy_len;
            data_len -= cpy_len;

            if (protocol_cb.data_counter >= protocol_cb.length)
            {
                protocol_cb.rx_state = IDLE;
                protocol_cback(PROTOCOL_EVENT_RX_HCI_ACL,
                        protocol_cb.con_hdl,
                        protocol_cb.rx_data,
                        protocol_cb.length);
            }
            break;

        // HCI SCO Packet
        case HCI_SCO_W4_CH_L:
            data_len--;
            protocol_cb.con_hdl = *p_data++;
            protocol_cb.rx_state = HCI_SCO_W4_CH_H;
            break;

        case HCI_SCO_W4_CH_H:
            data_len--;
            protocol_cb.con_hdl |= *p_data++ << 8;
            //TRACE_DBG_FULL("HCI_SCO_CON_HDL:0x%04x", protocol_cb.con_hdl);
            protocol_cb.rx_state = HCI_SCO_W4_LEN;
            break;

        case HCI_SCO_W4_LEN:
            data_len--;
            protocol_cb.length = *p_data++;
            //TRACE_DBG_FULL("HCI_SCO Len:%d", protocol_cb.length);
            if (protocol_cb.length == 0)
            {
                protocol_cb.rx_state = IDLE;
                protocol_cback(PROTOCOL_EVENT_RX_HCI_SCO,
                        protocol_cb.con_hdl, NULL, 0);
            }
            else
            {
                protocol_cb.data_counter = 0;
                protocol_cb.rx_state = HCI_SCO_W4_DATA;
            }
            break;

        case HCI_SCO_W4_DATA:
            cpy_len = MIN(protocol_cb.length, data_len);
            memcpy(&protocol_cb.rx_data[protocol_cb.data_counter], p_data, cpy_len);
            protocol_cb.data_counter += cpy_len;
            p_data += cpy_len;
            data_len -= cpy_len;

            if (protocol_cb.data_counter >= protocol_cb.length)
            {
                protocol_cb.rx_state = IDLE;
                protocol_cback(PROTOCOL_EVENT_RX_HCI_SCO,
                        protocol_cb.con_hdl,
                        protocol_cb.rx_data,
                        protocol_cb.length);
            }
            break;

        // HCI Event Packet
        case HCI_EVT_W4_EVT:
            data_len--;
            protocol_cb.event = *p_data++;
            //TRACE_DBG_FULL("HCI EVT:0x%x", protocol_cb.event);
            protocol_cb.rx_state = HCI_EVT_W4_LEN;
            break;
        case HCI_EVT_W4_LEN:
            data_len--;
            protocol_cb.length = *p_data++;
            //TRACE_DBG_FULL("HCI EVT len:0x%x", protocol_cb.length);
            if (protocol_cb.length == 0)
            {
                protocol_cb.rx_state = IDLE;
                protocol_cback(PROTOCOL_EVENT_RX_HCI_EVENT,
                        protocol_cb.event, NULL, 0);
            }
            else
            {
                protocol_cb.data_counter = 0;
                protocol_cb.rx_state = HCI_EVT_W4_DATA;
            }
            break;
        case HCI_EVT_W4_DATA:
            cpy_len = MIN(protocol_cb.length, data_len);
            //TRACE_DBG_FULL("evt cpy_len:%d", cpy_len);
            memcpy(&protocol_cb.rx_data[protocol_cb.data_counter], p_data, cpy_len);
            protocol_cb.data_counter += cpy_len;
            p_data += cpy_len;
            data_len -= cpy_len;

            if (protocol_cb.data_counter >= protocol_cb.length)
            {
                protocol_cb.rx_state = IDLE;
                protocol_cback(PROTOCOL_EVENT_RX_HCI_EVENT,
                        protocol_cb.event,
                        protocol_cb.rx_data,
                        protocol_cb.length);
            }
            break;

        // WICED Packet
        case WICED_W4_CMD:
            data_len--;
            protocol_cb.opcode = *p_data++;
            protocol_cb.rx_state = WICED_W4_GROUP;
            break;
        case WICED_W4_GROUP:
            data_len--;
            protocol_cb.opcode |= *p_data++ << 8;
            TRACE_DBG_FULL("Wiced Opcode:0x%04X ", protocol_cb.opcode);
            protocol_cb.rx_state = WICED_W4_LEN_L;
            break;
        case WICED_W4_LEN_L:
            data_len--;
            protocol_cb.length = *p_data++;
            protocol_cb.rx_state = WICED_W4_LEN_H;
            break;
        case WICED_W4_LEN_H:
            data_len--;
            protocol_cb.length |= *p_data++ << 8;
            //TRACE_DBG_FULL("Wiced CMD Len:%d ", protocol_cb.length);
            if (protocol_cb.length == 0)
            {
                protocol_cb.rx_state = IDLE;
                protocol_cback(PROTOCOL_EVENT_RX_WICED_EVENT,
                        protocol_cb.opcode, NULL, 0);
            }
            else
            {
                protocol_cb.data_counter = 0;
                protocol_cb.rx_state = WICED_W4_DATA;
            }
            break;
        case WICED_W4_DATA:
            cpy_len = MIN(protocol_cb.length, data_len);
            memcpy(&protocol_cb.rx_data[protocol_cb.data_counter], p_data, cpy_len);
            protocol_cb.data_counter += cpy_len;
            p_data += cpy_len;
            data_len -= cpy_len;

            if (protocol_cb.data_counter >= protocol_cb.length)
            {
                protocol_cb.rx_state = IDLE;
                protocol_cback(PROTOCOL_EVENT_RX_WICED_EVENT,
                        protocol_cb.opcode,
                        protocol_cb.rx_data,
                        protocol_cb.length);
            }
            break;

        default:
            TRACE_ERR("Unknown state:%d", protocol_cb.rx_state);
            protocol_cb.rx_state = IDLE;
            break;
        }
    }
}
