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

#include <stdio.h>
#include <stdint.h>
#include <stdint.h>
#include <pthread.h>
#include <stdarg.h>
#include <unistd.h>
#include <time.h>

#include "utils.h"

/*
 * Global variables
 */
trace_level_t trace_level = TRACE_LEVEL_INFO;
pthread_mutex_t trace_mutex = PTHREAD_MUTEX_INITIALIZER;


/*
 * utils_trace_dump
 */
void utils_trace_dump(char *prefix, uint8_t *buf, int size)
{
    int i;

    utils_trace_lock();

    for (i = 0; i < size; i++)
    {
        if ((i%16) == 0)
        {
            printf("%s0x%04X :", prefix, i);
        }
        printf(" %02X", buf[i]);
        if (((i%16) == 15) && (size > (i+1)))
        {
            fputs("\n", stdout);
        }
    }
    fputs("\n", stdout);

    utils_trace_unlock();
}

/*
 * utils_sleep
 *
 * Sleep requested number of seconds
 */
void utils_sleep(int duration)
{
    TRACE_DBG("Sleep for %d seconds...", duration);
    sleep(duration);
}

/*
 * utils_msleep
 *
 * Sleep requested number of milliseconds
 */
void utils_msleep(int duration)
{
    struct timespec sleep_dur;

    TRACE_DBG("Sleep for %d ms-seconds...", duration);
    sleep_dur.tv_sec = duration / 1000;
    sleep_dur.tv_nsec = 1000 * 1000 * (duration % 1000);
    nanosleep(&sleep_dur, NULL);
}

/*
 * utils_trace_lock
 */
void utils_trace_lock(void)
{
    pthread_mutex_lock(&trace_mutex);
}

/*
 * utils_trace_unlock
 */
void utils_trace_unlock(void)
{
    pthread_mutex_unlock(&trace_mutex);
}

/*
 * trace_level_set
 */
void trace_level_set (trace_level_t level)
{
    trace_level = level;
}

/*
 * trace_level_get
 */
trace_level_t trace_level_get (void)
{
    return trace_level;
}
