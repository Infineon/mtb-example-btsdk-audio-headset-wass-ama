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

#include "lrac.h"
#include "wiced.h"

/*
 * Definitions
 */
/* From 20721-B1_Bluetooth/include/hal/wiced_hal_nvram.h */
enum
{
    /* Wiced applications */
    WICED_NVRAM_VSID_START              = 0x200,
    WICED_NVRAM_VSID_END                = 0x3FFF
};

/* From lrac_headset/app_nvram.c */
enum
{
    NVRAM_ID_LRAC_INFO =                WICED_NVRAM_VSID_START + 1,
    NVRAM_ID_LOCAL_BDADDR,
    NVRAM_ID_PEER_LRAC_BDADDR,
    NVRAM_ID_SLEEP,
    NVRAM_ID_VOICE_PROMPT_FS,
};

/*
 * lrac_local_bdaddr_write
 */
int lrac_local_bdaddr_write(uint8_t *p_bdaddr)
{
    TRACE_DBG("BdAddr:%02X:%02X:%02X:%02X:%02X:%02X",
            p_bdaddr[0], p_bdaddr[1], p_bdaddr[2],
            p_bdaddr[3], p_bdaddr[4], p_bdaddr[5]);

    return wiced_cmd_nvram_write(NVRAM_ID_LOCAL_BDADDR, p_bdaddr, BD_ADDR_LEN);
}

/*
 * lrac_config_write
 */
int lrac_config_write(uint8_t *p_lrac_config)
{
    TRACE_DBG("LRAC Config P/S:%d L/R:%d", p_lrac_config[0], p_lrac_config[1]);

    return wiced_cmd_nvram_write(NVRAM_ID_LRAC_INFO, p_lrac_config, 2);
}

/*
 * lrac_peer_bdaddr_write
 */
int lrac_peer_bdaddr_write(uint8_t *p_bdaddr)
{
    TRACE_DBG("BdAddr:%02X:%02X:%02X:%02X:%02X:%02X",
            p_bdaddr[0], p_bdaddr[1], p_bdaddr[2],
            p_bdaddr[3], p_bdaddr[4], p_bdaddr[5]);

    return wiced_cmd_nvram_write(NVRAM_ID_PEER_LRAC_BDADDR, p_bdaddr, BD_ADDR_LEN);
}

/*
 * lrac_button_sent
 */
int lrac_button_sent(uint8_t button_id)
{
    TRACE_DBG("button Id:%d", button_id);

    return wiced_cmd_platform_button(button_id);
}

/*
 * lrac_audio_insert_sent
 */
int lrac_audio_insert_sent(uint8_t message_id)
{
    TRACE_DBG("message Id:%d", message_id);

    return wiced_cmd_platform_audio_insert(message_id);
}

/*
 * lrac_audio_insert_ext_sent
 */
int lrac_audio_insert_ext_sent(uint8_t command)
{
    TRACE_DBG("command:%d", command);

    return wiced_cmd_platform_audio_insert_ext(command);
}

/*
 * lrac_ble_adv_sent
 */
int lrac_ble_adv_sent(uint8_t mode)
{
    TRACE_DBG("mode:%d", mode);

    return wiced_cmd_platform_ble_adv(mode);
}

/*
 * lrac_switch_sent
 */
int lrac_switch_sent(uint8_t prevent_glitch)
{
    TRACE_DBG("");

    return wiced_cmd_platform_switch(prevent_glitch);
}

/*
 * lrac_read_buffer_stat
 */
int lrac_read_buffer_stat(void)
{
    int status;
    int i;
    wiced_bt_buffer_statistics_t buff_stat[10];

    status = wiced_cmd_read_buffer_stat(&buff_stat[0], sizeof(buff_stat) / sizeof(buff_stat[0]));
    if (status < 0)
        return status;

    for (i = 0 ; i < status ; i++)
    {
        TRACE_INFO("pool_id:%d size:%d\tcurrent/max/total: %d/%d/%d",
                buff_stat[i].pool_id,
                buff_stat[i].pool_size,
                buff_stat[i].current_allocated_count,
                buff_stat[i].max_allocated_count,
                buff_stat[i].total_count);
    }
    return status;
}

/*
 * lrac_trace_level_set
 */
int lrac_trace_level_set(uint8_t trace_level)
{
    TRACE_DBG("trace_level:%d", trace_level);

    return wiced_cmd_platform_lrac_trace_level_set(trace_level);
}

/*
 * lrac_sleep_config
 */
int lrac_sleep_config(uint8_t sleep_enable)
{
    TRACE_DBG("sleep_enable:%d", sleep_enable);

    return wiced_cmd_nvram_write(NVRAM_ID_SLEEP, &sleep_enable, 1);
}
