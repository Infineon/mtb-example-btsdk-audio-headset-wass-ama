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

#include "app_cpu_clock.h"
#include "wiced.h"
#include "wiced_hal_cpu_clk.h"
#include "app_trace.h"

/*
 * Definitions
 */
typedef struct
{
    uint8_t requests[APP_CPU_CLOCK_REQUESTER_MAX];
} app_cpu_clock_t;

/*
 * Global variables
 */
static app_cpu_clock_t app_cpu_clock;

/*
 * app_cpu_clock_init
 */
wiced_result_t app_cpu_clock_init(void)
{
    memset(&app_cpu_clock, 0, sizeof(app_cpu_clock));
    return WICED_BT_SUCCESS;
}

/*
 * app_cpu_clock_increase
 */
wiced_result_t app_cpu_clock_increase(app_cpu_clock_requester_t requester)
{
    int i;
    wiced_bool_t is_high_speed;

#ifdef APP_CPU_CLOCK_DEBUG
    APP_TRACE_DBG("requester:%d\n", requester);
#endif

    if (requester >= APP_CPU_CLOCK_REQUESTER_MAX)
    {
        APP_TRACE_ERR("bad requester:%d\n",requester);
        return WICED_BT_BADARG;
    }

    /* Check if a module already requested for High Speed CPU clock */
    for (i = 0, is_high_speed = WICED_FALSE ; i < APP_CPU_CLOCK_REQUESTER_MAX ; i++)
    {
        if (app_cpu_clock.requests[i])
        {
            is_high_speed = WICED_TRUE;     /* Already running High Speed clock */
            break;
        }
    }

    app_cpu_clock.requests[requester] = 1;

    /* If this is the first Request */
    if (is_high_speed == WICED_FALSE)
    {
#ifdef APP_CPU_CLOCK_DEBUG
        APP_TRACE_DBG("Set CPU Clock to 96MHz\n");
#endif
        wiced_update_cpu_clock(WICED_TRUE, WICED_CPU_CLK_96MHZ);
    }

    return WICED_BT_SUCCESS;
}

/*
 * app_cpu_clock_decrease
 */
wiced_result_t app_cpu_clock_decrease(app_cpu_clock_requester_t requester)
{
    int i;
    wiced_bool_t is_high_speed;

#ifdef APP_CPU_CLOCK_DEBUG
    APP_TRACE_DBG("requester:%d\n", requester);
#endif

    if (requester >= APP_CPU_CLOCK_REQUESTER_MAX)
    {
        APP_TRACE_ERR("bad requester:%d\n",requester);
        return WICED_BT_BADARG;
    }

    app_cpu_clock.requests[requester] = 0;

    /* Check if a module still Requested for High Speed CPU clock */
    for (i = 0, is_high_speed = WICED_FALSE ; i < APP_CPU_CLOCK_REQUESTER_MAX ; i++)
    {
        if (app_cpu_clock.requests[i])
        {
            is_high_speed = WICED_TRUE;     /* Already running High Speed clock */
            break;
        }
    }

    /* High Speed CPU Clock not anymore Requested */
    if (is_high_speed == WICED_FALSE)
    {
#ifdef APP_CPU_CLOCK_DEBUG
        APP_TRACE_DBG("Set CPU Clock to 48MHz\n");
#endif
        wiced_update_cpu_clock(WICED_FALSE, WICED_CPU_CLK_96MHZ);
    }

    return WICED_BT_SUCCESS;
}
