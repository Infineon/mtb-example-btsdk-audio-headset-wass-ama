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

#include "wiced_bt_dev.h"
#include "wiced_bt_lrac.h"
#include "wiced_hal_nvram.h"
#ifdef VOICE_PROMPT
#include "wiced_bt_voice_prompt.h"
#endif

/*
 * Definitions
 */
#define APP_NVRAM_PROFILES_A2DP_SINK_MASK   0x01
#define APP_NVRAM_PROFILES_AVRC_CT_MASK     0x02
#define APP_NVRAM_PROFILES_HFP_HS_MASK      0x04
typedef uint8_t app_nvram_profiles_t;

enum
{
    /* Do not change the order of the NVRAM entries */
    NVRAM_ID_LINK_KEYS = WICED_NVRAM_VSID_START,
    NVRAM_ID_LRAC_INFO,
    NVRAM_ID_LOCAL_BDADDR,
    NVRAM_ID_PEER_LRAC_BDADDR,
    NVRAM_ID_SLEEP,
    NVRAM_ID_VOICE_PROMPT_FS,
    NVRAM_ID_PAIRING_INFO_LRAC,         /* Peer LRAC Device Pairing Info. */
    NVRAM_ID_LOCAL_IRK,
    NVRAM_ID_GFPS_ACCOUNT_KEY,
};

enum
{
    APP_NVRAM_SLEEP_MODE_DISABLED = 0,
    APP_NVRAM_SLEEP_MODE_TRANSPORT,
    APP_NVRAM_SLEEP_MODE_NO_TRANSPORT
};
typedef uint8_t app_nvram_sleep_mode_t;

/*
 * Structures
 */
typedef struct
{
    wiced_bt_lrac_role_t role;
    wiced_bt_lrac_audio_side_t audio_side;
} app_nvram_lrac_info_t;

typedef struct
{
    app_nvram_sleep_mode_t sleep_mode;
} app_nvram_sleep_t;

/*
 * app_nvram_init
 */
wiced_result_t app_nvram_init(void);

/*
 * app_nvram_local_bdaddr_read
 * This function returns the Local BdAddr stored in NVRAM.
 * Note: a pending app_nvram_local_bdaddr_write may be added for Manufacturng configuration
 */
void app_nvram_local_bdaddr_read(uint8_t *p_bdaddr);

/*
 * app_nvram_pairing_info_read
 */
wiced_result_t app_nvram_pairing_info_read(wiced_bt_device_sec_keys_t *p_data);

/*
 * app_nvram_pairing_info_write
 */
void app_nvram_pairing_info_write(wiced_bt_device_sec_keys_t *p_data);

/*
 * app_nvram_pairing_info_delete
 */
void app_nvram_pairing_info_delete(void);

/*
 * app_nvram_lrac_info_read
 */
wiced_result_t app_nvram_lrac_info_read(app_nvram_lrac_info_t *p_data);

/*
 * app_nvram_lrac_info_write
 */
wiced_result_t app_nvram_lrac_info_write(app_nvram_lrac_info_t *p_data);

/*
 * app_nvram_lrac_bdaddr_set
 */
wiced_result_t app_nvram_lrac_bdaddr_set(wiced_bt_device_address_t bdaddr);

/*
 * app_nvram_lrac_bdaddr_get
 */
wiced_result_t app_nvram_lrac_bdaddr_get(uint8_t *p_bdaddr);

/*
 * app_nvram_write
 */
wiced_result_t app_nvram_write(uint16_t nvram_id, uint8_t *p_data, uint16_t length);

/*
 * app_nvram_sleep_get
 */
wiced_result_t app_nvram_sleep_get(app_nvram_sleep_t *p_sleep);

/*
 * app_nvram_local_irk_update
 *
 * Update Local Identity Resolving Key to NVRAM
 */
void app_nvram_local_irk_update(uint8_t *p_key);

/*
 * app_nvram_local_irk_get
 *
 * Acquire Local Identity Resolving Key from NVRAM
 */
wiced_bool_t app_nvram_local_irk_get(uint8_t *p_key);

#ifdef VOICE_PROMPT
/*
 * app_nvram_voice_prompt_config_get
 */
wiced_result_t app_nvram_voice_prompt_config_get(wiced_bt_voice_prompt_config_t *p_vpfs);

/*
 * app_nvram_voice_prompt_config_set
 */
wiced_result_t app_nvram_voice_prompt_config_set(wiced_bt_voice_prompt_config_t *p_vpfs);
#endif /* VOICE_PROMPT */
