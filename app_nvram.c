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

#include "app_nvram.h"
#include "wiced_hal_rand.h"
#include "app_trace.h"
#include "wiced_bt_dev.h"

/*
 * Definitions
 */

/*
 * Local functions
 */

/*
 * Global variables
 */

/*
 * app_nvram_init
 */
wiced_result_t app_nvram_init(void)
{
    return WICED_BT_SUCCESS;
}

/*
 * app_nvram_pairing_info_read
 */
wiced_result_t app_nvram_pairing_info_read(wiced_bt_device_sec_keys_t *p_data)
{
    uint16_t nb_bytes;
    wiced_result_t status;

    if (p_data == NULL)
        return WICED_BT_BADARG;

    nb_bytes = wiced_hal_read_nvram(NVRAM_ID_PAIRING_INFO_LRAC,
                                    sizeof(wiced_bt_device_sec_keys_t),
                                    (uint8_t *) p_data,
                                    &status);

    if ((nb_bytes == sizeof(wiced_bt_device_sec_keys_t)) &&
        (status == WICED_BT_SUCCESS))
    {
        return WICED_BT_SUCCESS;
    }

    return WICED_BT_ERROR;
}

/*
 * app_nvram_pairing_info_write
 */
void app_nvram_pairing_info_write(wiced_bt_device_sec_keys_t *p_data)
{
    uint16_t nb_bytes;
    wiced_result_t status;

    nb_bytes = wiced_hal_write_nvram(NVRAM_ID_PAIRING_INFO_LRAC,
                                     sizeof(wiced_bt_device_sec_keys_t),
                                     (uint8_t *) p_data,
                                     &status);

    if ((nb_bytes != sizeof(wiced_bt_device_sec_keys_t)) ||
        (status != WICED_BT_SUCCESS))
    {
        APP_TRACE_ERR("wiced_hal_write_nvram failed %d nb_bytes: %d\n",
                      status,
                      nb_bytes);
    }
}

/*
 * app_nvram_pairing_info_delete
 */
void app_nvram_pairing_info_delete(void)
{
    wiced_result_t status;

    wiced_hal_delete_nvram(NVRAM_ID_PAIRING_INFO_LRAC, &status);

    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_hal_delete_nvram failed %d\n", status);
    }
}

/*
 * app_nvram_lrac_info_read
 */
wiced_result_t app_nvram_lrac_info_read(app_nvram_lrac_info_t *p_data)
{
    uint16_t nb_bytes;
    wiced_result_t status;

    if (p_data == NULL)
        return WICED_BT_BADARG;

    /* Read the NVRAM ID */
    nb_bytes = wiced_hal_read_nvram(NVRAM_ID_LRAC_INFO, sizeof(app_nvram_lrac_info_t),
            (uint8_t *)p_data, &status);

    if ((nb_bytes != sizeof(app_nvram_lrac_info_t)) ||
        (status != WICED_BT_SUCCESS))
    {
        return WICED_BT_ERROR;
    }

    return WICED_BT_SUCCESS;
}

/*
 * app_nvram_lrac_info_write
 */
wiced_result_t app_nvram_lrac_info_write(app_nvram_lrac_info_t *p_data)
{
    uint16_t nb_bytes;
    wiced_result_t status;

    nb_bytes = wiced_hal_write_nvram(NVRAM_ID_LRAC_INFO, sizeof(app_nvram_lrac_info_t),
            (uint8_t *)p_data, &status);
    if ((nb_bytes != sizeof(app_nvram_lrac_info_t)) ||
        (status != WICED_BT_SUCCESS))
    {
        APP_TRACE_ERR("wiced_hal_write_nvram failed %d\n", status);
        return WICED_BT_ERROR;
    }

    return WICED_BT_SUCCESS;
}

/*
 * app_nvram_lrac_bdaddr_set
 * This function is called when the LRAC Link is established.
 */
wiced_result_t app_nvram_lrac_bdaddr_set(wiced_bt_device_address_t bdaddr)
{
    wiced_bt_device_address_t addr_in_nvram;
    uint16_t nb_bytes;
    wiced_result_t status;

    /* Check if the NVRAM shall be updated. */
    if (app_nvram_lrac_bdaddr_get(addr_in_nvram) == WICED_BT_SUCCESS)
    {
        if (memcmp((void *) bdaddr,
                   (void *) addr_in_nvram,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            return WICED_BT_SUCCESS;
        }
    }

    /* Write The Peer's LRAC device address */
   nb_bytes = wiced_hal_write_nvram(NVRAM_ID_PEER_LRAC_BDADDR,
                                    BD_ADDR_LEN,
                                    bdaddr,
                                    &status);

   if ((nb_bytes != BD_ADDR_LEN) ||
       (status != WICED_BT_SUCCESS))
   {
       APP_TRACE_ERR("wiced_hal_write_nvram (Peer LRAC BdAddr) failed (%d, %d)\n", nb_bytes, status);
       return WICED_BT_ERROR;
   }

   return WICED_BT_SUCCESS;
}

/*
 * app_nvram_lrac_bdaddr_get
 */
wiced_result_t app_nvram_lrac_bdaddr_get(uint8_t *p_bdaddr)
{
    uint16_t nb_bytes;
    wiced_result_t status;
    uint16_t nvram_id;

    /* Read this NVRAM ID */
    nb_bytes = wiced_hal_read_nvram(NVRAM_ID_PEER_LRAC_BDADDR, BD_ADDR_LEN, p_bdaddr, &status);

    if ((nb_bytes == BD_ADDR_LEN) &&
        (status == WICED_BT_SUCCESS))
    {
        return WICED_BT_SUCCESS;
    }

    /* No Peer LRAC BdAddr known */
    return WICED_BT_ERROR;
}

/*
 * app_nvram_create_random_bdaddr
 * This function creates a Random BdAddr (starting with 0x20719B0
 */
static void app_nvram_create_random_bdaddr(uint8_t *p_bdaddr)
{
    uint32_t rnd;

    p_bdaddr[0] = 0x20;
    p_bdaddr[1] = 0x71;
    p_bdaddr[2] = 0x9B;
    p_bdaddr[3] = 0x10;

    rnd = wiced_hal_rand_gen_num();
    p_bdaddr[4] = rnd & 0xFF;
    p_bdaddr[5] = (rnd >> 8) & 0xFF;
}

/*
 * app_nvram_local_bdaddr_read
 * This function returns the Local BdAddr stored in NVRAM.
 * Note: a pending app_nvram_local_bdaddr_write may be added for Manufacturing configuration
 */
void app_nvram_local_bdaddr_read(uint8_t *p_bdaddr)
{
    uint16_t nb_bytes;
    wiced_result_t result;

    nb_bytes = wiced_hal_read_nvram(NVRAM_ID_LOCAL_BDADDR,
                sizeof(wiced_bt_device_address_t), p_bdaddr, &result);

    if ((nb_bytes != sizeof(wiced_bt_device_address_t)) ||
        (result != WICED_BT_SUCCESS))
    {
        /* Create a Random BdAddr */
        app_nvram_create_random_bdaddr(p_bdaddr);

        /* Write it in NVRAM */
         wiced_hal_write_nvram(NVRAM_ID_LOCAL_BDADDR,
                               sizeof(wiced_bt_device_address_t),
                               p_bdaddr,
                               &result);
    }
}

/*
 * app_nvram_write
 * This function is, typically, called by the HCI during Manufacturing to configure the device.
 */
wiced_result_t app_nvram_write(uint16_t nvram_id, uint8_t *p_data, uint16_t length)
{
    wiced_result_t result;

    wiced_hal_write_nvram(nvram_id, length, p_data, &result);

    return result;
}

/*
 * app_nvram_sleep_get
 */
wiced_result_t app_nvram_sleep_get(app_nvram_sleep_t *p_sleep)
{
    uint16_t nb_bytes;
    wiced_result_t status;
    uint16_t nvram_id;

    /* Read this NVRAM ID */
    nb_bytes = wiced_hal_read_nvram(NVRAM_ID_SLEEP, sizeof(app_nvram_sleep_t), (uint8_t *)p_sleep,
            &status);
    if ((nb_bytes != sizeof(app_nvram_sleep_t)) ||
        (status != WICED_BT_SUCCESS))
    {
        return WICED_BT_ERROR;
    }

    return WICED_BT_SUCCESS;
}

/*
 * app_nvram_local_irk_update
 *
 * Update Local Identity Resolving Key to NVRAM
 */
void app_nvram_local_irk_update(uint8_t *p_key)
{
    uint16_t nb_bytes;
    wiced_result_t result;

    nb_bytes = wiced_hal_write_nvram(NVRAM_ID_LOCAL_IRK,
                                     BTM_SECURITY_LOCAL_KEY_DATA_LEN,
                                     p_key,
                                     &result);

    WICED_BT_TRACE("irk_update (result: %d, nb_bytes: %d)\n", result, nb_bytes);
}

/*
 * app_nvram_local_irk_get
 *
 * Acquire Local Identity Resolving Key from NVRAM
 */
wiced_bool_t app_nvram_local_irk_get(uint8_t *p_key)
{
    uint16_t nb_bytes;
    wiced_result_t result;

    nb_bytes = wiced_hal_read_nvram(NVRAM_ID_LOCAL_IRK,
                                    BTM_SECURITY_LOCAL_KEY_DATA_LEN,
                                    p_key,
                                    &result);

    if ((nb_bytes == BTM_SECURITY_LOCAL_KEY_DATA_LEN) && (result == WICED_BT_SUCCESS))
    {
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

#ifdef VOICE_PROMPT
/*
 * app_nvram_voice_prompt_config_get
 */
wiced_result_t app_nvram_voice_prompt_config_get(wiced_bt_voice_prompt_config_t *p_vpfs)
{
    uint16_t nb_bytes;
    wiced_result_t status;

    /* Read this NVRAM ID */
    nb_bytes = wiced_hal_read_nvram(NVRAM_ID_VOICE_PROMPT_FS,
            sizeof(wiced_bt_voice_prompt_config_t), (uint8_t *)p_vpfs,
            &status);
    if ((nb_bytes != sizeof(wiced_bt_voice_prompt_config_t)) ||
        (status != WICED_BT_SUCCESS))
    {
        return WICED_BT_ERROR;
    }

    return WICED_BT_SUCCESS;
}

/*
 * app_nvram_voice_prompt_config_set
 */
wiced_result_t app_nvram_voice_prompt_config_set(wiced_bt_voice_prompt_config_t *p_vpfs)
{
    uint16_t nb_bytes;
    wiced_result_t status;

    /* Write this NVRAM ID */
    nb_bytes = wiced_hal_write_nvram(NVRAM_ID_VOICE_PROMPT_FS,
            sizeof(wiced_bt_voice_prompt_config_t), (uint8_t *)p_vpfs,
            &status);
    if ((nb_bytes != sizeof(wiced_bt_voice_prompt_config_t)) ||
        (status != WICED_BT_SUCCESS))
    {
        return WICED_BT_ERROR;
    }

    return WICED_BT_SUCCESS;
}
#endif /* VOICE_PROMPT */
