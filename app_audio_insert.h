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

#include <stdint.h>
#include "wiced.h"
#include "app_lrac.h"

typedef enum
{
    APP_AUDIO_INSERT_STOPPED,
} app_audio_insert_event_t;

typedef enum
{
    APP_AUDIO_INSERT_STOP_REQ_REGULAR = 0,  /* Regular Stop request */
    APP_AUDIO_INSERT_STOP_REQ_SUSPEND,      /* Temporary suspend Audio Insertion. */
    APP_AUDIO_INSERT_STOP_REQ_FLUSH,        /* Stop and Flush any pending Audio Insertion request */
    APP_AUDIO_INSERT_STOP_REQ_MAX = APP_AUDIO_INSERT_STOP_REQ_FLUSH
} app_audio_insert_stop_req_t;

typedef struct
{
    wiced_result_t status;
} app_audio_insert_event_data_stopped_t;

typedef union
{
    app_audio_insert_event_data_stopped_t stopped;
} app_audio_insert_event_data_t;

typedef void (app_audio_insert_callback_t)(app_audio_insert_event_t event,
        app_audio_insert_event_data_t *p_data);

/*
 * app_audio_insert_init
 */
wiced_result_t app_audio_insert_init(app_audio_insert_callback_t *p_callback);

/*
 * app_audio_insert_start_req
 */
wiced_result_t app_audio_insert_start_req(uint8_t audio_file_index);

/*
 * app_audio_insert_resume
 * Resume (Restart) AudioInsert if needed
 */
void app_audio_insert_resume(void);

/*
 * app_audio_insert_stop_req
 */
wiced_result_t app_audio_insert_stop_req(app_audio_insert_stop_req_t stop_request);

/*
 * app_audio_insert_is_started
 */
wiced_bool_t app_audio_insert_is_started(void);

/*
 * app_audio_insert_handler
 * This function handles the LRAC Audio Insert Events
 */
void app_audio_insert_lrac_event_handler(wiced_bt_lrac_event_t event,
        wiced_bt_lrac_event_data_t *p_data);
