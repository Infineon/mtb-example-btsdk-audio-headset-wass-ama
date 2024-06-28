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
#include "utils.h"

/*
 * lrac_local_bdaddr_write
 */
int lrac_local_bdaddr_write(uint8_t *p_bdaddr);

/*
 * lrac_config_write
 */
int lrac_config_write(uint8_t *p_lrac_config);

/*
 * lrac_peer_bdaddr_write
 */
int lrac_peer_bdaddr_write(uint8_t *p_bdaddr);

/*
 * lrac_button_sent
 */
int lrac_button_sent(uint8_t button_id);

/*
 * lrac_audio_insert_sent
 */
int lrac_audio_insert_sent(uint8_t message_id);

/*
 * lrac_audio_insert_ext_sent
 */
int lrac_audio_insert_ext_sent(uint8_t command);

/*
 * lrac_ble_adv_sent
 */
int lrac_ble_adv_sent(uint8_t mode);

/*
 * lrac_switch_sent
 */
int lrac_switch_sent(uint8_t prevent_glitch);

/*
 * lrac_read_buffer_stat
 */
int lrac_read_buffer_stat(void);

/*
 * lrac_trace_level_set
 */
int lrac_trace_level_set(uint8_t trace_level);

/*
 * lrac_sleep_config
 */
int lrac_sleep_config(uint8_t sleep_enable);
