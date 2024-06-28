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

#include "wiced.h"

typedef enum
{
    APP_CPU_CLOCK_REQUESTER_PS_SWITCH = 0,
    APP_CPU_CLOCK_REQUESTER_SCO,
#ifdef VOICE_PROMPT
    APP_CPU_CLOCK_AUDIO_INSERT,
#endif
    /* Add other 'Requester' (e.g. Audio Insert) here ... */
    APP_CPU_CLOCK_REQUESTER_MAX
} app_cpu_clock_requester_t;

/*
 * app_cpu_clock_init
 */
wiced_result_t app_cpu_clock_init(void);

/*
 * app_cpu_clock_increase
 */
wiced_result_t app_cpu_clock_increase(app_cpu_clock_requester_t requester);

/*
 * app_cpu_clock_decrease
 */
wiced_result_t app_cpu_clock_decrease(app_cpu_clock_requester_t requester);
