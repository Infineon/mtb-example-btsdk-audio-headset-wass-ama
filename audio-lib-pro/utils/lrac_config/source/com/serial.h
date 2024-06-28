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

#ifndef WICED_SERIAL_H_
#define WICED_SERIAL_H_

#include <stdint.h>

typedef enum
{
    WICED_SERIAL_EVENT_DISCONNECT = 0,
    WICED_SERIAL_EVENT_RX_DATA
} serial_event_t;

typedef enum
{
    WICED_IOCTL_OP_SCO_UP,
    WICED_IOCTL_OP_SCO_DOWN
} wiced_ioctl_cmd_t;

typedef union
{
    uint16_t sco_handle;
} wiced_ioctl_data_t;


typedef void (serial_callback_t)(serial_event_t event, uint8_t *p_data, int data_len);

int serial_init(void);
int serial_open(char *p_device, int baudrate, int flow_control, serial_callback_t *p_callback);
int serial_close(void);
int serial_set_baudrate(int baudrate, int flow_control);
int serial_write(uint8_t *p_data, int data_len);
int wiced_ioctl(wiced_ioctl_cmd_t op, wiced_ioctl_data_t *p_data);

#endif /* WICED_SERIAL_H_ */
