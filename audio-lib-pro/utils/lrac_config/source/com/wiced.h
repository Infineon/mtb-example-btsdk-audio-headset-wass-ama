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

typedef struct
{
    uint8_t     pool_id;                    /**< pool id */
    uint16_t    pool_size;                  /**< pool buffer size */
    uint16_t    current_allocated_count;    /**< number of  buffers currently allocated */
    uint16_t    max_allocated_count;        /**< maximum number of buffers allocated at any time */
    uint16_t    total_count;                /**< total number of buffers */
} wiced_bt_buffer_statistics_t;

/*
 * wiced_init
 */
int wiced_init(void);

/*
 * wiced_event_handler
 */
int wiced_event_handler(uint16_t opcode, uint8_t *p_data, uint16_t length);

/*
 * wiced_cmd_reset
 */
int wiced_cmd_reset(void);

/*
 * wiced_cmd_read_buffer_stat
 */
int wiced_cmd_read_buffer_stat(wiced_bt_buffer_statistics_t *p_buffer_stat, int nb_buffer_stat);

/*
 * wiced_cmd_nvram_write
 */
int wiced_cmd_nvram_write(uint16_t nvram_id, uint8_t *p_data, uint16_t length);

/*
 * wiced_cmd_platform_button
 */
int wiced_cmd_platform_button(uint8_t button_id);

/*
 * wiced_cmd_platform_audio_insert
 */
int wiced_cmd_platform_audio_insert(uint8_t message_id);

/*
 * wiced_cmd_platform_audio_insert_ext
 */
int wiced_cmd_platform_audio_insert_ext(uint8_t command);

/*
 * wiced_cmd_platform_ble_adv
 */
int wiced_cmd_platform_ble_adv(uint8_t mode);

/*
 * wiced_cmd_platform_switch
 */
int wiced_cmd_platform_switch(uint8_t prevent_glitch);

/*
 * wiced_cmd_platform_lrac_trace_level_set
 */
int wiced_cmd_platform_lrac_trace_level_set(uint8_t trace_level);

/*
 * wiced_cmd_fw_spi_debug_enable
 */
int wiced_cmd_fw_spi_debug_enable(uint8_t enable);

/*
 * wiced_cmd_jitter_buffer_target_set
 */
int wiced_cmd_jitter_buffer_target_set(uint8_t jitter_buffer_target);

/*
 * wiced_cmd_elna_gain_set
 */
int wiced_cmd_elna_gain_set(int8_t elna_gain);

/*
 * wiced_cmd_write_binary_file_to_flash
 */
int wiced_cmd_write_binary_file_to_flash(char *p_bin_file, uint32_t offset);
