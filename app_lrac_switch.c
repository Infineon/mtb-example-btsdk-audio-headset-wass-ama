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

#ifdef AMA_ENABLED
#include <ama.h>
#endif
#include "app_lrac.h"
#include "app_a2dp_sink.h"
#include "app_avrc_ct.h"
#include "app_handsfree.h"
#include "app_nvram.h"
#include "app_main.h"
#include "app_bt.h"
#include "app_volume.h"
#include "app_trace.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_avrc_ct.h"
#include "wiced_platform.h"
#include "wiced_platform_audio_common.h"
#include "wiced_memory.h"
#include "wiced_bt_ble.h"
#ifdef APP_OFU_SUPPORT
#include "ofu/app_ofu.h"
#endif


/*
 * Types
 */
/* Switch Synchronization "Is Ready" function type */
typedef wiced_bool_t (app_lrac_switch_sync_is_ready_t)(void);

/* Switch Synchronization Data Get function type */
typedef wiced_result_t (app_lrac_switch_sync_get_t)(void *p_opaque, uint16_t *p_sync_data_len);

/* Switch Synchronization Data Set function type */
typedef wiced_result_t (app_lrac_switch_sync_set_t)(void *p_opaque, uint16_t sync_data_len);

/*
 * Structures
 */
typedef struct
{
    app_lrac_switch_sync_is_ready_t *p_ready;  /* Optional */
    app_lrac_switch_sync_get_t *p_get;
    app_lrac_switch_sync_set_t *p_set;
} app_lrac_switch_sync_fct_t;


/*
 * Global variables
 */
const static app_lrac_switch_sync_fct_t app_lrac_switch_sync_fct[] =
{
    /*
     * Application's files
     */
    {
        .p_ready = app_lrac_ready_to_switch,
        .p_get = app_lrac_switch_get,
        .p_set = app_lrac_switch_set
    },
    {
        .p_ready = NULL,
        .p_get = platform_switch_get,
        .p_set = platform_switch_set
    },
#if defined (OTA_FW_UPGRADE) && (APP_OFU_SUPPORT)
    {
        .p_ready = app_ofu_ready_to_switch,
        .p_get = app_ofu_switch_get,
        .p_set = app_ofu_switch_set,
    },
#endif
#ifdef VOLUME_EFFECT
    {
        .p_ready = app_volume_effect_ready_to_switch,
        .p_get = NULL,
        .p_set = NULL,
    },
#endif
    /*
     * Libraries
     */
    {
        .p_ready = NULL,
        .p_get = wiced_bt_a2dp_sink_lrac_switch_get,
        .p_set = wiced_bt_a2dp_sink_lrac_switch_set
    },
    {
        .p_ready = NULL,
        .p_get = wiced_bt_avrc_ct_lrac_switch_get,
        .p_set = wiced_bt_avrc_ct_lrac_switch_set
    },
    {
        .p_ready = NULL,
        .p_get = wiced_bt_hfp_hf_lrac_switch_get,
        .p_set = wiced_bt_hfp_hf_lrac_switch_set
    },
#ifdef AMA_ENABLED
    {
        .p_ready = ama_ready_to_switch,
        .p_get = NULL,
        .p_set = NULL
    },
#endif

    /*
     * Main application (probably safer to make it last)
     */
    {
        .p_ready = app_main_switch_is_ready,
        .p_get = app_main_switch_get,
        .p_set = app_main_switch_set
    },
};

/*
 * app_lrac_switch_is_ready
 */
wiced_bool_t app_lrac_switch_is_ready(void)
{
    uint8_t nb_sync_get_fct;
    uint8_t tag;
    wiced_bool_t is_ready;

    nb_sync_get_fct = sizeof(app_lrac_switch_sync_fct) / sizeof(app_lrac_switch_sync_fct[0]);

    /* Call every "Is Ready"  functions to check if every module is ready to Switch */
    for (tag = 0 ; tag < nb_sync_get_fct ; tag++)
    {
        if (app_lrac_switch_sync_fct[tag].p_ready != NULL)
        {
            is_ready = app_lrac_switch_sync_fct[tag].p_ready();
            if (is_ready == WICED_FALSE)
            {
                APP_TRACE_ERR("Module %d is not ready to Switch\n", tag);
                return WICED_FALSE;
            }
        }
    }
    return WICED_TRUE;
}

/*
 * app_lrac_switch_data_collect
 */
wiced_result_t app_lrac_switch_data_collect(void)
{
    uint16_t sync_data_len;
    uint8_t tag;
    uint8_t nb_sync_get_fct;
    wiced_result_t status = WICED_BT_ERROR;
    wiced_bool_t last = WICED_FALSE;
    uint8_t *switch_data;


    switch_data = wiced_bt_lrac_share_buf_lock_and_get(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_COLLECT_BUF);
    if (switch_data == NULL)
    {
        APP_TRACE_ERR("Cannot allocate switch_data\n");
        return WICED_NO_MEMORY;
    }

    if (wiced_bt_lrac_share_buf_length() < WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX)
    {
        APP_TRACE_ERR("Share buffer too small\n");
        wiced_bt_lrac_share_buf_unlock(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_COLLECT_BUF);
        return WICED_NO_MEMORY;
    }

    nb_sync_get_fct = sizeof(app_lrac_switch_sync_fct) / sizeof(app_lrac_switch_sync_fct[0]);

    /* Collect Switch Synchronization data from LRAC Library */
    for (tag = 0 ; tag < nb_sync_get_fct ; tag++)
    {
        if (app_lrac_switch_sync_fct[tag].p_get)
        {
            sync_data_len = WICED_BT_LRAC_SWITCH_BLOB_SIZE_MAX;
            status = app_lrac_switch_sync_fct[tag].p_get(&switch_data[0], &sync_data_len);
            if (status == WICED_SUCCESS)
            {
                /* Check if this is the last tag */
                if (tag == (nb_sync_get_fct -1))
                    last = WICED_TRUE;

                /* Send Switch Data */
                status = wiced_bt_lrac_switch_data_rsp(last, tag, &switch_data[0], sync_data_len);
                if (status != WICED_BT_SUCCESS)
                {
                    APP_TRACE_ERR("wiced_bt_lrac_switch_data_rsp for tag:%d failed\n", tag);
                    break;
                }
            }
            else
            {
                APP_TRACE_ERR("p_get for tag:%d failed\n", tag);
                break;
            }
        }
    }

    wiced_bt_lrac_share_buf_unlock(WICED_BT_LRAC_SHARE_BUF_ID_SWITCH_COLLECT_BUF);

    return status;
}

/*
 * app_lrac_switch_data_apply
 */
wiced_result_t app_lrac_switch_data_apply(uint8_t tag, uint8_t *p_data, uint16_t length)
{
    uint8_t nb_sync_set_fct;
    wiced_result_t status;

    nb_sync_set_fct = sizeof(app_lrac_switch_sync_fct) / sizeof(app_lrac_switch_sync_fct[0]);

    if (tag < nb_sync_set_fct)
    {
        if (app_lrac_switch_sync_fct[tag].p_set)
        {
            status = app_lrac_switch_sync_fct[tag].p_set(p_data, length);
        }
        else
        {
            APP_TRACE_ERR("No Sync Set function for tag:%d\n", tag);
            status = WICED_BT_ERROR;
        }
    }
    else
    {
        APP_TRACE_ERR("Wrong tag:%d\n", tag);
        status = WICED_BT_ERROR;
    }
    return status;
}
