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

#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "wiced.h"
#include "utils.h"
#include "hci.h"
#include "protocol.h"

/*
 * Group codes
 */
#define HCI_CONTROL_GROUP_DEVICE                0x00
#define HCI_PLATFORM_GROUP                      0xD0

/*
 * Device Group Commands
 */
#define HCI_CONTROL_COMMAND_RESET               ((HCI_CONTROL_GROUP_DEVICE << 8) | 0x01)    /* Restart controller */
#define HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA     ((HCI_CONTROL_GROUP_DEVICE << 8) | 0x05)    /* Download previously saved NVRAM chunk */
#define HCI_CONTROL_COMMAND_READ_BUFF_STATS     ((HCI_CONTROL_GROUP_DEVICE << 8) | 0x11)    /* Read Buffer Statistics */
/*
 * Platform (Customer specific) Group Commands
 */
#define HCI_PLATFORM_COMMAND_BUTTON             ((HCI_PLATFORM_GROUP << 8) | 0x20)          /* Button Simulation */
#define HCI_PLATFORM_COMMAND_AUDIO_INSERT       ((HCI_PLATFORM_GROUP << 8) | 0x21)          /* Audio Insert Simulation */
#define HCI_PLATFORM_COMMAND_BLE_ADV            ((HCI_PLATFORM_GROUP << 8) | 0x22)          /* LE Adv Simulation */
#define HCI_PLATFORM_COMMAND_SWITCH             ((HCI_PLATFORM_GROUP << 8) | 0x23)          /* LRAC Switch Simulation */
#define HCI_PLATFORM_COMMAND_LRAC_TRACE_LEVEL   ((HCI_PLATFORM_GROUP << 8) | 0x24)          /* LRAC Set Trace Level */
#define HCI_PLATFORM_COMMAND_VSC_WRAPPER        ((HCI_PLATFORM_GROUP << 8) | 0x25)          /* VSC Wrapper */
#define HCI_PLATFORM_COMMAND_JITTER_TARGET_SET  ((HCI_PLATFORM_GROUP << 8) | 0x28)          /* Jitter Buffer Target */
#define HCI_PLATFORM_COMMAND_ELNA_GAIN_SET      ((HCI_PLATFORM_GROUP << 8) | 0x29)          /* eLNA Gain Set */
#define HCI_PLATFORM_COMMAND_EF_ERASE           ((HCI_PLATFORM_GROUP << 8) | 0x30)          /* Embedded Flash Erase */
#define HCI_PLATFORM_COMMAND_EF_WRITE           ((HCI_PLATFORM_GROUP << 8) | 0x32)          /* Embedded Flash Write */
#define HCI_PLATFORM_COMMAND_AUDIO_INSERT_EXT   ((HCI_PLATFORM_GROUP << 8) | 0x33)          /* Audio Insertion Extended Simulation */

/*
 * Device Group Events
 */
#define HCI_CONTROL_EVENT_COMMAND_STATUS        (( HCI_CONTROL_GROUP_DEVICE << 8) | 0x01)   /* Command status event for the requested operation */
#define HCI_CONTROL_EVENT_WICED_TRACE           (( HCI_CONTROL_GROUP_DEVICE << 8) | 0x02)   /* WICED trace packet */
#define HCI_CONTROL_EVENT_HCI_TRACE             (( HCI_CONTROL_GROUP_DEVICE << 8) | 0x03)   /* Bluetooth protocol trace */
#define HCI_CONTROL_EVENT_DEVICE_STARTED        (( HCI_CONTROL_GROUP_DEVICE << 8) | 0x05)   /* Device completed power up initialization */
#define HCI_CONTROL_EVENT_READ_BUFFER_STATS     (( HCI_CONTROL_GROUP_DEVICE << 8) | 0x0F)  /* Read Buffer statistics event */

/*
 * Platform (Customer specific) Group Events
 */
#define HCI_PLATFORM_EVENT_VSC_CMD_CPLT         ((HCI_PLATFORM_GROUP << 8) | 0x25)          /* VSC Wrapper Command Complete event */
#define HCI_PLATFORM_EVENT_COMMAND_STATUS       ((HCI_PLATFORM_GROUP << 8) | 0xFF)          /* Command status event for the requested operation */


#define WICED_DATA_SIZE_MAX                     1024

/* Embedded Flash Page Size */
#define EF_PAGE_SIZE                            (4 * 1024)

typedef struct
{
    int cmd_pending;
    pthread_cond_t cond;
    pthread_mutex_t mutex;
    uint8_t rx_data[WICED_DATA_SIZE_MAX];
    uint16_t rx_data_len;
} wiced_cb_t;

/*
 * Local functions
 */
static int wiced_cmd_vsc_wrapper(uint16_t opcode, uint8_t *p_tx_param, uint16_t tx_param_len,
        uint8_t *p_rx_param, uint16_t rx_param_len);
static int wiced_cmd_send_receive(uint16_t opcode, uint8_t *p_tx_data, uint16_t tx_length,
        uint8_t *p_rx_data, uint16_t rx_length);

/*
 * Global variables
 */
wiced_cb_t wiced_cb;

/*
 * wiced_init
 */
int wiced_init(void)
{
    TRACE_DBG("");

    memset(&wiced_cb, 0, sizeof(wiced_cb));

    pthread_cond_init(&wiced_cb.cond, NULL);
    pthread_mutex_init(&wiced_cb.mutex, NULL);

    return 0;
}

/*
 * wiced_event_handler
 */
int wiced_event_handler(uint16_t opcode, uint8_t *p_data, uint16_t length)
{
    int handled = 0;

    switch(opcode)
    {
    case HCI_CONTROL_EVENT_WICED_TRACE:
        handled = 1;
        TRACE_INFO("WICED TRACE:%s", p_data);
        break;
    case HCI_CONTROL_EVENT_HCI_TRACE:
        handled = 1;
        TRACE_INFO("WICED HCI TRACE");
        break;

    case HCI_CONTROL_EVENT_DEVICE_STARTED:
        handled = 1;
        TRACE_INFO("WICED DEVICE STARTED");
        break;

    case HCI_PLATFORM_EVENT_VSC_CMD_CPLT:
    case HCI_CONTROL_EVENT_READ_BUFFER_STATS:
    case HCI_PLATFORM_EVENT_COMMAND_STATUS:
    case HCI_CONTROL_EVENT_COMMAND_STATUS:
        handled = 1;
        TRACE_DBG("Wiced Device Cmd Status OpCode:0x%x", opcode);
        handled = 1;
        memcpy(wiced_cb.rx_data, p_data, length);
        wiced_cb.rx_data_len = length;
        pthread_mutex_lock(&wiced_cb.mutex);
        wiced_cb.cmd_pending = 0;
        pthread_cond_signal(&wiced_cb.cond);
        pthread_mutex_unlock(&wiced_cb.mutex);
        break;

    default:
        TRACE_DBG("Unhandled Wiced event opcode:0x%04X", opcode);
        break;
    }

    return handled;
}

/*
 * wiced_cmd_nvram_write
 */
int wiced_cmd_nvram_write(uint16_t nvram_id, uint8_t *p_data, uint16_t length)
{
    int status;
    uint8_t rx_param[1];
    uint8_t tx_param[WICED_DATA_SIZE_MAX];
    uint8_t *p;

    TRACE_DBG("nvram_id:0x%04x length:%d", nvram_id, length);

    p = tx_param;
    UINT16_TO_STREAM(p, nvram_id);
    memcpy(p, p_data, length);
    p += length;

    status = wiced_cmd_send_receive(HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA, tx_param, p - tx_param,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_reset
 */
int wiced_cmd_reset(void)
{
    int status;
    uint8_t rx_param[1];
    uint8_t *p;

    TRACE_DBG("");

    status = wiced_cmd_send_receive(HCI_CONTROL_COMMAND_RESET, NULL, 0,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_read_buffer_stat
 */
int wiced_cmd_read_buffer_stat(wiced_bt_buffer_statistics_t *p_buffer_stat, int nb_buffer_stat)
{
    int status;
    uint8_t rx_param[512];
    uint8_t *p;
    int i;
    int nb_buf_stat = 0;

    TRACE_DBG("nb_buffer_stat:%d", nb_buffer_stat);

    status = wiced_cmd_send_receive(HCI_CONTROL_COMMAND_READ_BUFF_STATS, NULL, 0,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status > sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    for (i = 0 ; i < nb_buffer_stat ; i ++)
    {
        if (status >= ( 1 + 2 + 2 + 2 + 2))
        {
            STREAM_TO_UINT8(p_buffer_stat[i].pool_id, p);
            STREAM_TO_UINT16(p_buffer_stat[i].pool_size, p);
            STREAM_TO_UINT16(p_buffer_stat[i].current_allocated_count, p);
            STREAM_TO_UINT16(p_buffer_stat[i].max_allocated_count, p);
            STREAM_TO_UINT16(p_buffer_stat[i].total_count, p);
            status -= 1 + 2 + 2 + 2 + 2;
            nb_buf_stat++;
        }
    }

    return nb_buf_stat;
}

/*
 * wiced_cmd_platform_button
 */
int wiced_cmd_platform_button(uint8_t button_id)
{
    int status;
    uint8_t tx_param[1];
    uint8_t rx_param[1];
    uint8_t *p;

    TRACE_DBG("");

    p = tx_param;
    UINT8_TO_STREAM(p, button_id);
    status = wiced_cmd_send_receive(HCI_PLATFORM_COMMAND_BUTTON, tx_param, p - tx_param,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_platform_audio_insert
 */
int wiced_cmd_platform_audio_insert(uint8_t message_id)
{
    int status;
    uint8_t tx_param[1];
    uint8_t rx_param[1];
    uint8_t *p;

    TRACE_DBG("");

    p = tx_param;
    UINT8_TO_STREAM(p, message_id);
    status = wiced_cmd_send_receive(HCI_PLATFORM_COMMAND_AUDIO_INSERT, tx_param, p - tx_param,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_platform_audio_insert_ext
 */
int wiced_cmd_platform_audio_insert_ext(uint8_t command)
{
    int status;
    uint8_t tx_param[1];
    uint8_t rx_param[1];
    uint8_t *p;

    TRACE_DBG("");

    p = tx_param;
    UINT8_TO_STREAM(p, command);
    status = wiced_cmd_send_receive(HCI_PLATFORM_COMMAND_AUDIO_INSERT_EXT, tx_param, p - tx_param,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_platform_switch
 */
int wiced_cmd_platform_switch(uint8_t prevent_glitch)
{
    int status;
    uint8_t tx_param[1];
    uint8_t rx_param[1];
    uint8_t *p;

    TRACE_DBG("");

    p = tx_param;
    UINT8_TO_STREAM(p, prevent_glitch);
    status = wiced_cmd_send_receive(HCI_PLATFORM_COMMAND_SWITCH, tx_param, p - tx_param,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_platform_ble_adv
 */
int wiced_cmd_platform_ble_adv(uint8_t mode)
{
    int status;
    uint8_t tx_param[1];
    uint8_t rx_param[1];
    uint8_t *p;

    TRACE_DBG("");

    p = tx_param;
    UINT8_TO_STREAM(p, mode);
    status = wiced_cmd_send_receive(HCI_PLATFORM_COMMAND_BLE_ADV, tx_param, p - tx_param,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_platform_lrac_trace_level_set
 */
int wiced_cmd_platform_lrac_trace_level_set(uint8_t trace_level)
{
    int status;
    uint8_t tx_param[1];
    uint8_t rx_param[1];
    uint8_t *p;

    TRACE_DBG("trace_level:%d", trace_level);

    p = tx_param;
    UINT8_TO_STREAM(p, trace_level);
    status = wiced_cmd_send_receive(HCI_PLATFORM_COMMAND_LRAC_TRACE_LEVEL, tx_param, p - tx_param,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_fw_spi_debug_enable
 */
int wiced_cmd_fw_spi_debug_enable(uint8_t enable)
{
    int status;
    uint8_t tx_param[255];
    uint8_t *p;

    TRACE_DBG("enable:%d", enable);

    /* Prepare Command Parameter */
    p = tx_param;
    UINT8_TO_STREAM(p, 0);
    UINT8_TO_STREAM(p, 0);
    UINT16_TO_STREAM(p, 0);
    UINT8_TO_STREAM(p, enable);
    UINT32_TO_STREAM(p, 0xFFFFFFFF);

    status = wiced_cmd_vsc_wrapper(HCI_BCS_TIMELINE, tx_param, p - tx_param,
            NULL, 0);
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_vsc_wrapper failed");
        return status;
    }

    return status;
}

/*
 * wiced_cmd_vsc_wrapper
 */
static int wiced_cmd_vsc_wrapper(uint16_t opcode, uint8_t *p_tx_param, uint16_t tx_param_len,
        uint8_t *p_rx_param, uint16_t rx_param_len)
{
    int status;
    uint8_t tx_buffer[512];
    uint8_t rx_buffer[512];
    uint8_t *p;
    int i;
    uint8_t hci_status;

    TRACE_DBG("VSC opcode:0x%04x tx_param_len:%d", opcode, tx_param_len);

    /* Prepare the VSC Wrapper parameters */
    p = &tx_buffer[0];
    UINT16_TO_STREAM(p, opcode);
    memcpy(p, p_tx_param, tx_param_len);
    p += tx_param_len;

    /* Send he VSC Wrapper command */
    status = wiced_cmd_send_receive(HCI_PLATFORM_COMMAND_VSC_WRAPPER, tx_buffer, p - tx_buffer,
            rx_buffer, (uint16_t)sizeof(rx_buffer));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status > sizeof(rx_buffer))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_buffer));
        return -1;
    }

    p = &rx_buffer[0];
    STREAM_TO_UINT8(hci_status, p);
    status--;
    if (hci_status != 0)
    {
        TRACE_ERR("failed hci_status:%d", hci_status);
        return (0 - hci_status);
    }

    if (p_rx_param)
    {
        memset(p_rx_param, 0, rx_param_len);
        if (rx_param_len >= status)
        {
            memcpy(p_rx_param, rx_buffer, status);
        }
        else
        {
            TRACE_ERR("too much parameters eceived (%d/%d)", status, rx_param_len);
            return -1;
        }
    }

    return status;
}

/*
 * wiced_cmd_jitter_buffer_target_set
 */
int wiced_cmd_jitter_buffer_target_set(uint8_t jitter_buffer_target)
{
    int status;
    uint8_t tx_param[1];
    uint8_t rx_param[1];
    uint8_t *p;

    TRACE_DBG("Jitter Buffer Target depth:%d", jitter_buffer_target);

    p = tx_param;
    UINT8_TO_STREAM(p, jitter_buffer_target);
    status = wiced_cmd_send_receive(HCI_PLATFORM_COMMAND_JITTER_TARGET_SET, tx_param, p - tx_param,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_elna_gain_set
 */
int wiced_cmd_elna_gain_set(int8_t elna_gain)
{
    int status;
    uint8_t tx_param[1];
    uint8_t rx_param[1];
    uint8_t *p;

    TRACE_DBG("eLNA Gain:%d", (int)elna_gain);

    p = tx_param;
    UINT8_TO_STREAM(p, elna_gain);
    status = wiced_cmd_send_receive(HCI_PLATFORM_COMMAND_ELNA_GAIN_SET, tx_param, p - tx_param,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_emb_flash_erase
 */
int wiced_cmd_emb_flash_erase(uint32_t offset, uint32_t length)
{
    int status;
    uint8_t tx_param[8];
    uint8_t rx_param[1];
    uint8_t *p;

    TRACE_DBG("Erase offset:0x%x len:%d", offset, length);

    p = tx_param;
    UINT32_TO_STREAM(p, offset);
    UINT32_TO_STREAM(p, length);
    status = wiced_cmd_send_receive(HCI_PLATFORM_COMMAND_EF_ERASE, tx_param, p - tx_param,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_emb_flash_write
 */
int wiced_cmd_emb_flash_write(uint32_t offset, uint8_t *p_data, uint32_t length)
{
    int status;
    uint8_t tx_param[1024];
    uint8_t rx_param[1];
    uint8_t *p;

    TRACE_DBG("write offset:0x%x len:%d", offset, length);

    p = tx_param;
    UINT32_TO_STREAM(p, offset);
    memcpy(p, p_data, length);
    p += length;

    status = wiced_cmd_send_receive(HCI_PLATFORM_COMMAND_EF_WRITE, tx_param, p - tx_param,
            rx_param, (uint16_t)sizeof(rx_param));
    if (status < 0)
    {
        TRACE_ERR("wiced_cmd_send_receive failed");
        return status;
    }

    if (status != sizeof(rx_param))
    {
        TRACE_ERR("wrong length received (%d/%d)", status, (int)sizeof(rx_param));
        return -1;
    }
    p = rx_param;
    STREAM_TO_UINT8(status, p);
    if (status != 0)
    {
        TRACE_ERR("failed hci_status:%d", status);
        return (0 - status);
    }

    return (int)status;
}

/*
 * wiced_cmd_write_binary_file_to_flash
 */
int wiced_cmd_write_binary_file_to_flash(char *p_bin_file, uint32_t offset)
{
    int status;
    int file_desc;
    uint8_t *p_buffer, *p;
    uint32_t size;
    struct stat file_stat;
    int file_len;
    int aligned_file_len;
    ssize_t read_size;
    uint32_t write_len;

    TRACE_DBG("file:%s offset:0x%x", p_bin_file, offset);

    if (offset & (EF_PAGE_SIZE - 1))
    {
        fprintf(stderr, "offset:0x%x must be multiple of EF_PAGE_SIZE (4K)\n");
        return -1;
    }

    file_desc = open(p_bin_file, O_RDONLY);
    if (file_desc < 0)
    {
        TRACE_ERR("Cannot open %s file", p_bin_file);
        return file_desc;
    }

    status = stat(p_bin_file, &file_stat);
    if (status < 0)
    {
        TRACE_ERR("Cannot stat %s file", p_bin_file);
        return status;
    }

    file_len = file_stat.st_size;
    TRACE_DBG("file size:%d", file_len);

    aligned_file_len = file_len + 3;
    aligned_file_len &= ~0x3;
    TRACE_DBG("32 bits 'aligned' file size:%d", aligned_file_len);

    p_buffer = malloc(aligned_file_len);
    if (p_buffer == NULL)
    {
        TRACE_ERR("Cannot allocate %d bytes", aligned_file_len);
        return -1;
    }
    memset(p_buffer, 0, aligned_file_len);

    read_size = read(file_desc, p_buffer, file_len);
    if (read_size != file_len)
    {
        TRACE_ERR("Cannot read %s file", p_bin_file);
        return -1;
    }

    p = p_buffer;
    while(aligned_file_len)
    {
        if ((offset % EF_PAGE_SIZE) == 0)
        {
            status = wiced_cmd_emb_flash_erase(offset, EF_PAGE_SIZE);
            if (status < 0)
            {
                TRACE_ERR("Cannot erase flash offset:%x length:%d", offset, EF_PAGE_SIZE);
                return status;
            }
        }
        if (aligned_file_len > 512)
            write_len = 512;
        else
            write_len = aligned_file_len;

        status = wiced_cmd_emb_flash_write(offset, p, write_len);
        if (status < 0)
        {
            TRACE_ERR("Cannot write flash offset:%x length:%d", offset, write_len);
            return status;
        }


        offset += write_len;
        p += write_len;
        aligned_file_len -= write_len;
    }
    free(p_buffer);

    return 0;
}

/*
 * wiced_cmd_send_receive
 */
static int wiced_cmd_send_receive(uint16_t opcode, uint8_t *p_tx_data, uint16_t tx_length,
        uint8_t *p_rx_data, uint16_t rx_length)
{
    int status;
    int cpy_len;
    struct timespec ts;

    TRACE_DBG("opcode:0x%04x tl:%d rl:%d", opcode, tx_length, rx_length);

    if (wiced_cb.cmd_pending)
    {
        TRACE_ERR("Wiced Cmd pending");
        return -1;
    }

    if (p_rx_data != NULL)
    {
        wiced_cb.cmd_pending = 1;
    }

    wiced_cb.rx_data_len = 0;

    /* Send the HCI command */
    status = protocol_send(PROTOCOL_TYPE_WICED, opcode, p_tx_data, tx_length);
    if (status < 0)
    {
        TRACE_ERR("protocol_send failed");
        wiced_cb.cmd_pending = 0;
        return status;
    }

    if (p_rx_data != NULL)
    {
        pthread_mutex_lock(&wiced_cb.mutex);
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 2;
        status = 0;
        while (wiced_cb.cmd_pending && (status == 0))
        {
            status = pthread_cond_timedwait(&wiced_cb.cond, &wiced_cb.mutex, &ts);
        }
        pthread_mutex_unlock(&wiced_cb.mutex);
        if (status != 0)
        {
            TRACE_ERR("Command timeout");
            wiced_cb.cmd_pending = 0;
            return -1;
        }

        if (wiced_cb.rx_data_len > rx_length)
        {
            TRACE_ERR("failed rx buffer too small (%d/%d)", wiced_cb.rx_data_len, rx_length);
            memcpy(p_rx_data, wiced_cb.rx_data, rx_length);
            status = rx_length;
        }
        else
        {
            memcpy(p_rx_data, wiced_cb.rx_data, wiced_cb.rx_data_len);
            status = wiced_cb.rx_data_len;
        }
    }
    else
    {
        utils_msleep(100);
    }
    return status;
}
