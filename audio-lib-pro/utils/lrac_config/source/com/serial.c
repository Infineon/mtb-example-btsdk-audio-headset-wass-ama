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

#include "serial.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

#include "utils.h"

/*
 * Definitions
 */
typedef struct
{
    int fd;
    serial_callback_t *p_callback;
    pthread_t thread;
} serial_cb_t;

typedef struct
{
    unsigned short sco_handle;
    char burst;
} wiced_ioctl_sco_ctrl_t;

#define IOCTL_BTWUSB_ADD_VOICE_CHANNEL    0x1009
#define IOCTL_BTWUSB_REMOVE_VOICE_CHANNEL 0x100a

/*
 * Globals
 */
static serial_cb_t serial_cb;

/*
 * Local functions
 */
static void *serial_thread(void *);

/*
 * serial_init
 */
int serial_init(void)
{
    TRACE_DBG("");
    memset(&serial_cb, 0, sizeof(serial_cb));
    serial_cb.fd = -1;
    return 0;
}

/*
 * serial_open
 */
int serial_open(char *p_device, int baudrate, int flow_control, serial_callback_t *p_callback)
{
    int status = 0;
    int fd;

    TRACE_DBG("Port %s", p_device);

    if (serial_cb.fd >= 0)
    {
        TRACE_ERR("Port already opened");
        return -1;
    }

    /* Open the Bluetooth controller device */
    fd = open(p_device, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        TRACE_ERR("serial_open open(%s) failed", p_device);
        perror("open");
        return -1;
    }

    serial_cb.fd = fd;
    serial_cb.p_callback = p_callback;

    /* Change Baudrate for UART device (containing 'tty') only */
    if (strstr(p_device, "tty"))
    {
        status = serial_set_baudrate(baudrate, flow_control);
        if (status < 0)
        {
            serial_init();
            return -1;
        }
    }

    if (pthread_create(&serial_cb.thread, NULL, serial_thread, NULL) < 0)
    {
        TRACE_ERR("pthread_create failed");
        serial_init();
        return -1;
    }

    return status;
}

/*
 * serial_close
 */
int serial_close(void)
{
    TRACE_DBG("serial_close");

    if (serial_cb.fd < 0)
    {
        TRACE_ERR("serial_close Port not opened");
        return -1;
    }

    if (close(serial_cb.fd) < 0)
    {
        TRACE_ERR("serial_close close failed");
        return -1;
    }

    serial_init();

    pthread_cancel(serial_cb.thread);

    return 0;
}

/*
 * serial_set_baudrate
 */
int serial_set_baudrate(int baudrate, int flow_control)
{
    uint32_t baud;
    uint8_t data_bits;
    uint16_t parity;
    uint8_t stop_bits;
    struct termios termios;

    TRACE_DBG("serial_set_baudrate baudrate:%d HW-FlowControl:%d", baudrate, flow_control);

    if (serial_cb.fd < 0)
    {
        TRACE_ERR("serial_set_baudrate Port not opened");
        return -1;
    }

    switch(baudrate)
    {
    case 600:
        baud = B600;
        break;
    case 1200:
        baud = B1200;
        break;
    case 9600:
        baud = B9600;
        break;
    case 19200:
        baud = B19200;
        break;
    case 57600:
        baud = B57600;
        break;
    case 115200:
        baud = B115200;
        break;
    case 230400:
        baud = B230400;
        break;
    case 460800:
        baud = B460800;
        break;
    case 921600:
        baud = B921600;
        break;
#if defined(B1000000)
    case 1000000:
        baud = B1000000;
        break;
#endif
#if defined(B2000000)
    case 2000000:
        baud = B2000000;
        break;
#endif
#if defined(B2500000)
    case 2500000:
        baud = B2500000;
        break;
#endif
#if defined(B3000000)
    case 3000000:
        baud = B3000000;
        break;
#endif
#if defined(B4000000)
    case 4000000:
        baud = B4000000;
        break;
#endif
    default:
        TRACE_ERR("serial_set_baudrate bad baudrate:%d", baudrate);
        return -1;
    }

    data_bits = CS8;    /* 8 Bits */
    parity = 0;         /* No Parity */
    stop_bits = 0;      /* 1 Stop bit */

    tcflush(serial_cb.fd, TCIOFLUSH);

    tcgetattr(serial_cb.fd, &termios);

    /* Configure in default raw mode */
    cfmakeraw(&termios);

    /* Clear out what can be overriden */
    termios.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
    termios.c_cflag |= parity | data_bits | stop_bits;

    if (flow_control)
    {
        termios.c_cflag |= CRTSCTS;
    }

    tcsetattr(serial_cb.fd, TCSANOW, &termios);

    tcflush(serial_cb.fd, TCIOFLUSH);

    tcsetattr(serial_cb.fd, TCSANOW, &termios);

    tcflush(serial_cb.fd, TCIOFLUSH);
    tcflush(serial_cb.fd, TCIOFLUSH);

    cfsetospeed(&termios, baud);
    cfsetispeed(&termios, baud);
    tcsetattr(serial_cb.fd, TCSANOW, &termios);
    return 0;

}

/*
 * serial_write
 */
int serial_write(uint8_t *p_data, int data_len)
{
    ssize_t size;

    TRACE_DUMP_DBG_FULL("HCI TX >> ", p_data, data_len);

    if (serial_cb.fd < 0)
    {
        TRACE_ERR("serial_write Port not opened");
        return -1;
    }

    size = write(serial_cb.fd, p_data, data_len);
    if (size < 0)
    {
        TRACE_ERR("serial_write write failed");
        return -1;
    }
    else if (size < data_len)
    {
        TRACE_ERR("serial_write write partial write (%d instread of %d",
                (int)size, data_len);
        return -1;
    }

    return (int)size;
}

/*
 * serial_thread
 */
static void *serial_thread(void *arg)
{
    uint8_t buffer[1024];
    ssize_t size;

    TRACE_DBG("started");

    do
    {
        size = read(serial_cb.fd, &buffer, sizeof(buffer));
        if (size < 0)
        {
            TRACE_ERR("read failed");
            if (serial_cb.p_callback)
                serial_cb.p_callback(WICED_SERIAL_EVENT_DISCONNECT, NULL, 0);
            break;
        }
        if (serial_cb.p_callback)
        {
            TRACE_DUMP_DBG_FULL("HCI RX << ", buffer, (int)size);
            serial_cb.p_callback(WICED_SERIAL_EVENT_RX_DATA, buffer, (int)size);
        }
    } while(1);
    TRACE_DBG("exit");
    return NULL;
}

/*
 * wiced_ioctl
 */
int wiced_ioctl(wiced_ioctl_cmd_t op, wiced_ioctl_data_t *p_data)
{
    wiced_ioctl_sco_ctrl_t ioctl_data;
    int rv;

    switch (op)
    {
    case WICED_IOCTL_OP_SCO_UP:
        TRACE_DBG("wiced_ioctl: Received WICED_IOCTL_OP_SCO_UP ioctl");
        /* Command only supported by Bluetooth USB driver */
        if (1)
        {
            ioctl_data.sco_handle = p_data->sco_handle;
            ioctl_data.burst = 48;
            rv = ioctl(serial_cb.fd, IOCTL_BTWUSB_ADD_VOICE_CHANNEL, &ioctl_data);
            if (rv < 0)
            {
                TRACE_DBG("USERIAL_Ioctl: USERIAL_OP_SCO_UP failed");
            }
        }
        break;

    case WICED_IOCTL_OP_SCO_DOWN:
        TRACE_DBG("wiced_ioctl: Received WICED_IOCTL_OP_SCO_DOWN ioctl");
        /* Command only supported by Bluetooth USB driver */
        if (1)
        {
            ioctl_data.sco_handle = p_data->sco_handle;
            ioctl_data.burst = 0;
            rv = ioctl(serial_cb.fd, IOCTL_BTWUSB_REMOVE_VOICE_CHANNEL, &ioctl_data);
            if (rv < 0)
            {
                TRACE_DBG("USERIAL_Ioctl: USERIAL_OP_SCO_DOWN failed");
            }
        }
        break;

    default:
        break;
    }

    return rv;
}
