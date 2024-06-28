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

#include <pthread.h>
#include <string.h>

#include "hci.h"
#include "utils.h"
#include "protocol.h"

/*
 * Definitions
 */
#define HCI_DATA_SIZE_MAX           (1024)
#define HCI_CMD_PARAM_SIZE_MAX      255
#define HCI_EVT_PARAM_SIZE_MAX      255

#define HCI_CHIP_ID_ROM_ADDRESS     0xD21C

/*
 * Structures
 */
typedef struct
{
    uint8_t num_cmd;
    uint16_t wait_opcode;
    uint16_t received_opcode;
    pthread_cond_t cond;
    pthread_mutex_t mutex;
    uint8_t rx_data[HCI_DATA_SIZE_MAX];
    uint16_t rx_data_len;
} hci_protocol_cb_t;

typedef struct
{
    uint8_t chip_id;
    uint8_t rom_value;
    uint16_t rom_addr;
} hci_chip_id_t;

/*
 * Global variables
 */
hci_protocol_cb_t hci_cb;
hci_chip_id_t hci_chip_id_table[] =
{
        {CHIP_ID_20706A1, 0x03, HCI_CHIP_ID_ROM_ADDRESS},
        {CHIP_ID_20706A2, 0x21, HCI_CHIP_ID_ROM_ADDRESS},
};

/*
 * Local functions
 */
static int hci_cmd_send_receive(uint16_t opcode, uint8_t *p_tx_data, uint8_t tx_length,
        uint8_t *p_rx_data, uint8_t rx_length);
static char *hci_get_opcode_desc(uint16_t opcode);

/*
 * hci_init
 */
int hci_init(void)
{
    TRACE_DBG("");
    memset(&hci_cb, 0, sizeof(hci_cb));
    pthread_cond_init(&hci_cb.cond, NULL);
    pthread_mutex_init(&hci_cb.mutex, NULL);
    return 0;
}

/*
 * hci_cmd_reset
 *
 * Send HCI_Reset command
 */
int hci_cmd_reset(void)
{
    int status;
    uint8_t hci_status;

    status = hci_cmd_send_receive(HCI_RESET, NULL, 0, &hci_status, (uint8_t)sizeof(hci_status));
    if (status < 0)
    {
        TRACE_ERR("hci_cmd_send_receive failed");
        return status;
    }
    if (hci_status != HCI_STATUS_SUCCESS)
        return 0 - hci_status;

    return 0;
}

/*
 * hci_cmd_read_ram
 *
 * Send HCI_ReadRam VSC
 */
int hci_cmd_read_ram(uint32_t fw_address, uint8_t length, uint8_t *p_data)
{
    int status;
    uint8_t tx_param[5];
    uint8_t *p;
    uint8_t rx_param[HCI_EVT_PARAM_SIZE_MAX];
    uint8_t hci_status;

    if ((p_data == NULL) ||
        (length == 0))
    {
        TRACE_ERR("no buffer or null length");
        return -1;
    }

    TRACE_DBG("address:0x%X length:%d", fw_address, length);

    p = tx_param;
    UINT32_TO_STREAM(p, fw_address);
    UINT8_TO_STREAM(p, length);
    status = hci_cmd_send_receive(HCI_READ_RAM, tx_param, p - tx_param, rx_param,
            (uint8_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("hci_cmd_send_receive failed");
        return status;
    }

    if (status < ( 1 + length))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, 1 + length);
        return -1;
    }

    hci_status = rx_param[0];
    if (hci_status != HCI_STATUS_SUCCESS)
        return 0 - hci_status;

    memcpy(p_data, &rx_param[1], length);

    return length;
}

/*
 * hci_cmd_write_ram
 *
 * Send HCI_WriteRam VSC
 */
int hci_cmd_write_ram(uint32_t fw_address, uint8_t length, uint8_t *p_data)
{
    int status;
    uint8_t tx_param[HCI_CMD_PARAM_SIZE_MAX];
    uint8_t *p;
    uint8_t hci_status;

    TRACE_DBG("address:0x%X length:%d", fw_address, length);

    if ((p_data == NULL) ||
        (length == 0)    ||
        (length > (HCI_CMD_PARAM_SIZE_MAX - 4)))
    {
        TRACE_ERR("no buffer or wrong length");
        return -1;
    }

    p = tx_param;
    UINT32_TO_STREAM(p, fw_address);
    memcpy(p, p_data, length);
    p += length;
    status = hci_cmd_send_receive(HCI_WRITE_RAM, tx_param, p - tx_param, &hci_status,
            (uint8_t)sizeof(hci_status));
    if (status < 0)
    {
        TRACE_ERR("hci_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(hci_status))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(hci_status));
        return -1;
    }

    if (hci_status != HCI_STATUS_SUCCESS)
    {
        TRACE_ERR("failed hci_status:0x%x", hci_status);
        return 0 - hci_status;
    }

    return length;
}

/*
 * hci_cmd_sector_erase
 *
 * Send HCI_SectorErase VSC
 */
int hci_cmd_sector_erase(uint32_t fw_address)
{
    int status;
    uint8_t tx_param[sizeof(uint32_t)];
    uint8_t *p;
    uint8_t hci_status;

    TRACE_DBG("address:0x%X", fw_address);

    p = tx_param;
    UINT32_TO_STREAM(p, fw_address);
    status = hci_cmd_send_receive(HCI_SECTOR_ERASE, tx_param, p - tx_param, &hci_status,
            (uint8_t)sizeof(hci_status));
    if (status < 0)
    {
        TRACE_ERR("hci_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(hci_status))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(hci_status));
        return -1;
    }

    if (hci_status != HCI_STATUS_SUCCESS)
    {
        TRACE_ERR("failed hci_status:0x%x", hci_status);
        return 0 - hci_status;
    }

    return 0;
}

/*
 * hci_cmd_read_chip_id
 *
 * Send HCI_ReadVerboseCfgVerInfo VSC (to read ChipId)
 */
int hci_cmd_read_chip_id(void)
{
    int status;
    uint8_t rx_param[7];
    uint8_t *p;

    TRACE_DBG("");

    status = hci_cmd_send_receive(HCI_READ_VERBOSE_CFG_VER_INFO, NULL, 0, rx_param,
            (uint8_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("hci_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != HCI_STATUS_SUCCESS)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }
    STREAM_TO_UINT8(status, p);

    return (int)status;
}

/*
 * hci_cmd_check_chip_id
 *
 * Send ReadRam VSC (to check the ChipId)
 */
int hci_cmd_check_chip_id(uint8_t chip_id)
{
    int status;
    uint8_t rx_param[1];
    uint8_t *p;
    int i;

    TRACE_DBG("");

    for (i = 0 ; i < (sizeof(hci_chip_id_table) / sizeof(hci_chip_id_table[0])) ; i++)
    {
        if (hci_chip_id_table[i].chip_id == chip_id)
            break;
    }
    if (i >= (sizeof(hci_chip_id_table) / sizeof(hci_chip_id_table[0])))
    {
        TRACE_ERR("Unsupported ChipId:0x%x", chip_id);
        return -1;
    }

    status = hci_cmd_read_ram(hci_chip_id_table[i].rom_addr, (uint8_t)sizeof(rx_param), rx_param);
    if (status < 0)
    {
        TRACE_ERR("hci_cmd_read_ram failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != hci_chip_id_table[i].rom_value)
    {
        TRACE_ERR("Rom value does not match expect:0x%x got:0x%x", hci_chip_id_table[i].rom_value, status);
        return -1;
    }

    return (int)hci_chip_id_table[i].chip_id;
}

/*
 * hci_cmd_download_minidriver
 *
 * Send HCI DownloadMinidriver VSC
 */
int hci_cmd_download_minidriver(void)
{
    int status;
    uint8_t hci_status;

    status = hci_cmd_send_receive(HCI_DWNLD_MINIDRV, NULL, 0, &hci_status, (uint8_t)sizeof(hci_status));
    if (status < 0)
    {
        TRACE_ERR("hci_cmd_send_receive failed");
        return status;
    }
    if (hci_status != HCI_STATUS_SUCCESS)
        return 0 - hci_status;

    return 0;
}

/*
 * hci_cmd_launch_ram
 *
 * Send the HCI_LunchRam VSC
 */
int hci_cmd_launch_ram(uint32_t fw_address)
{
    int status;
    uint8_t tx_param[sizeof(uint32_t)];
    uint8_t *p;
    uint8_t hci_status;

    TRACE_DBG("address:0x%X", fw_address);

    p = tx_param;
    UINT32_TO_STREAM(p, fw_address);
    status = hci_cmd_send_receive(HCI_LAUNCH_RAM, tx_param, p - tx_param, &hci_status,
            (uint8_t)sizeof(hci_status));
    if (status < 0)
    {
        TRACE_ERR("failed");
        return status;
    }

    if (status != sizeof(hci_status))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(hci_status));
        return -1;
    }

    if (hci_status != HCI_STATUS_SUCCESS)
        return 0 - hci_status;

    return 0;
}

/*
 * hci_cmd_send_receive
 */
static int hci_cmd_send_receive(uint16_t opcode, uint8_t *p_tx_data, uint8_t tx_length,
        uint8_t *p_rx_data, uint8_t rx_length)
{
    int status;
    int cpy_len;
    struct timespec ts;

    TRACE_DBG("opcode:0x%x (%s) tl:%d rl:%d", opcode, hci_get_opcode_desc(opcode), tx_length, rx_length);

    if (p_rx_data == NULL)
    {
        TRACE_ERR("no Rx buffer");
        return -1;
    }

    if (hci_cb.wait_opcode != 0)
    {
        TRACE_ERR("cmd pending");
        return -1;
    }
    hci_cb.wait_opcode = opcode;
    hci_cb.rx_data_len = 0;

    /* Send the HCI command */
    status = protocol_send(PROTOCOL_TYPE_HCI_CMD, opcode, p_tx_data, tx_length);
    if (status < 0)
    {
        TRACE_ERR("protocol_send failed");
        hci_cb.wait_opcode = 0;
        return status;
    }

    pthread_mutex_lock(&hci_cb.mutex);
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 2;
    status = 0;
    while ((hci_cb.wait_opcode != hci_cb.received_opcode) && (status == 0))
    {
        status = pthread_cond_timedwait(&hci_cb.cond, &hci_cb.mutex, &ts);
    }
    pthread_mutex_unlock(&hci_cb.mutex);
    if (status != 0)
    {
        TRACE_ERR("Command timeout");
        hci_cb.wait_opcode = 0;
        return -1;
    }

    hci_cb.wait_opcode = 0;
    hci_cb.received_opcode = 0;

    if (hci_cb.rx_data_len > rx_length)
    {
        TRACE_ERR("failed rx buffer too small (%d/%d)", hci_cb.rx_data_len, rx_length);
        memcpy(p_rx_data, hci_cb.rx_data, rx_length);
        status = rx_length;
    }
    else
    {
        memcpy(p_rx_data, hci_cb.rx_data, hci_cb.rx_data_len);
        status = hci_cb.rx_data_len;
    }
    return status;
}



/*
 * hci_event_handler
 *
 * Handles Received HCI Events
 */
int hci_event_handler(uint8_t event, uint8_t *p_data, uint16_t length)
{
    uint16_t hci_opcode;
    uint8_t num_cmd;
    int handled = 0;

    TRACE_DBG("event:0x%x len:%d", event, length);
    switch(event)
    {
    case HCI_EVENT_COMMAND_COMPLETE:
        handled = 1;
        STREAM_TO_UINT8(hci_cb.num_cmd, p_data);
        STREAM_TO_UINT16(hci_opcode, p_data);
        TRACE_DBG("CmdCplt opcode:0x%04x (%s) status:0x%x", hci_opcode,
                hci_get_opcode_desc(hci_opcode), *p_data);

        if ((hci_cb.wait_opcode) &&
            (hci_cb.wait_opcode == hci_opcode))
        {
            handled = 1;
            memcpy(hci_cb.rx_data, p_data, length - 1 - 2);
            hci_cb.rx_data_len = length - 1 - 2;
            pthread_mutex_lock(&hci_cb.mutex);
            hci_cb.received_opcode = hci_opcode;
            pthread_cond_signal(&hci_cb.cond);
            pthread_mutex_unlock(&hci_cb.mutex);
        }
        else
        {
            TRACE_ERR("unexpected CmdCplt opcode:0x%04x", hci_opcode);
        }
        break;
    }

    return handled;
}

/*
 * hci_acl_handler
 *
 *  Handles Received HCI ACL Packets
 *
 */
int hci_acl_handler(uint16_t handle, uint8_t *p_data, uint16_t length)
{
    int handled = 0;
    TRACE_DBG("handle:0x%x len:%d", handle, length);
    return handled;
}

/*
 * hci_sco_handler
 *
 * Handles Received HCI SCO Packets
 */
int hci_sco_handler(uint16_t handle, uint8_t *p_data, uint16_t length)
{
    int handled = 0;
    TRACE_DBG("handle:0x%x len:%d", handle, length);
    return handled;
}

/*
 * hci_get_opcode_desc
 */
static char *hci_get_opcode_desc(uint16_t opcode)
{
    switch(opcode)
    {
    case HCI_RESET: return "HCI_RESET";
    case HCI_WRITE_LOOPBACK: return "HCI_WRITE_LOOPBACK";

    case HCI_READ_VERBOSE_CFG_VER_INFO: return "HCI_READ_VERBOSE_CFG_VER_INFO";
    case HCI_READ_RAM: return "HCI_READ_RAM";
    case HCI_WRITE_RAM: return "HCI_WRITE_RAM";
    case HCI_SECTOR_ERASE: return "HCI_SECTOR_ERASE";
    case HCI_DWNLD_MINIDRV: return "HCI_DWNLD_MINIDRV";
    case HCI_LAUNCH_RAM: return "HCI_LAUNCH_RAM";
    }
    return "Unknown HCI Opcode";
}
