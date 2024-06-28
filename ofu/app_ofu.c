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
#ifdef OTA_FW_UPGRADE
#include "app_ofu.h"
#include "app_ofu_lrac.h"
#include "app_ofu_srv.h"
#include "app_ofu_clt.h"
#include "app_ofu_spp.h"
#include "app_ofu_ble.h"
#include "app_trace.h"

/*
 * Definitions
 */
typedef struct
{
    app_ofu_callback_t *p_callback;
} app_ofu_cb_t;

/*
 * Global variables
 */
static app_ofu_cb_t app_ofu_cb;

/*
 * app_ofu_init
 */
wiced_result_t app_ofu_init(app_ofu_callback_t *p_callback)
{
    wiced_result_t status;

    APP_OFU_TRACE_DBG("\n");

    memset(&app_ofu_cb, 0, sizeof(app_ofu_cb));

    app_ofu_cb.p_callback = p_callback;

    /* Initialize the OFU Server (common to SPP, LE and LRAC) */
    status = app_ofu_srv_init();
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("app_ofu_srv_init failed\n");
        return status;
    }

    /* Initialize the OFU Client (used by LRAC only) */
    status = app_ofu_clt_init();
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("app_ofu_clt_init failed\n");
        return status;
    }

    /* Initialize the OFU SPP Server */
    status = app_ofu_spp_init(p_callback);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("app_ofu_lrac_init failed\n");
        return status;
    }

    /* Initialize the OFU LE Server */
    status = app_ofu_ble_init(p_callback);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("app_ofu_ble_init failed\n");
        return status;
    }

    /* Initialize the OFU LRAC (used as Client or Server over LRAC) */
    status = app_ofu_lrac_init(p_callback);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("app_ofu_lrac_init failed\n");
        return status;
    }

    return WICED_BT_SUCCESS;
}

/*
 * app_ofu_ready_to_switch
 */
wiced_bool_t app_ofu_ready_to_switch(void)
{
    if (app_ofu_spp_ready_to_switch() == WICED_FALSE)
    {
        return WICED_FALSE;
    }

   return WICED_TRUE;
}

/*
 * app_ofu_switch_get
 */
wiced_result_t app_ofu_switch_get(void *p_opaque, uint16_t *p_sync_data_len)
{
    /*
     * We don't plan to support PS-Switch when OFU is ongoing (main app checks that) or when SPP
     * is connected.
     * So, we don't really need to implement the Switch Get/Set API. But we add them here just to
     * be ready in case it's needed (and for future backward compatibility)
     */
    *p_sync_data_len = 1;

    return WICED_BT_SUCCESS;
}

/*
 * app_ofu_switch_set
 */
wiced_result_t app_ofu_switch_set(void *p_opaque, uint16_t sync_data_len)
{
    /*
     * We don't plan to support PS-Switch when OFU is ongoing (main app checks that) or when SPP
     * is connected.
     * So, we don't really need to implement the Switch Get/Set API. But we add them here just to
     * be ready in case it's needed (and for future backward compatibility)
     */
    return WICED_BT_SUCCESS;
}
#endif
