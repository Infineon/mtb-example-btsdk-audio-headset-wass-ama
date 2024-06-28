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

#include <stdio.h>
#include <stdint.h>

#ifndef BD_ADDR_LEN
#define BD_ADDR_LEN           6
#endif

#ifndef DEV_CLASS_LEN
#define DEV_CLASS_LEN           3
#endif

typedef enum
{
    TRACE_LEVEL_ERROR = 0,
    TRACE_LEVEL_INFO,
    TRACE_LEVEL_DEBUG,
    TRACE_LEVEL_DEBUG_FULL
} trace_level_t;

extern trace_level_t trace_level;

void utils_trace_lock(void);
void utils_trace_unlock(void);
void utils_trace_dump(char *prefix, uint8_t *buf, int size);


#define TRACE_INFO(format, ...) \
    do { \
        if (trace_level >= TRACE_LEVEL_INFO) \
        { \
            utils_trace_lock();\
            printf("%s: " format, __FUNCTION__, ##__VA_ARGS__); \
            printf("\n"); \
            utils_trace_unlock();\
        } \
    } while (0)

#define TRACE_DBG(format, ...) \
        do { \
            if (trace_level >= TRACE_LEVEL_DEBUG) \
            { \
                utils_trace_lock();\
                printf("%s: " format, __FUNCTION__, ##__VA_ARGS__); \
                printf("\n"); \
                utils_trace_unlock();\
            } \
        } while (0)

#define TRACE_DBG_FULL(format, ...) \
        do { \
            if (trace_level >= TRACE_LEVEL_DEBUG_FULL) \
            { \
                utils_trace_lock();\
                printf("%s: " format, __FUNCTION__, ##__VA_ARGS__); \
                printf("\n"); \
                utils_trace_unlock();\
            } \
        } while (0)

#define TRACE_ERR(format, ...) \
        do { \
            utils_trace_lock();\
            printf("ERROR %s: " format, __FUNCTION__, ##__VA_ARGS__); \
            printf("\n"); \
            utils_trace_unlock();\
        } while (0)

#define TRACE_DUMP(a, b, c) \
    do { \
        utils_trace_dump(a, b, c); \
    } while (0)

#define TRACE_DUMP_DBG(a, b, c) \
    do { \
        if (trace_level >= TRACE_LEVEL_DEBUG) \
        { \
            utils_trace_dump(a, b, c); \
        } \
    } while (0)

#define TRACE_DUMP_DBG_FULL(a, b, c) \
    do { \
        if (trace_level >= TRACE_LEVEL_DEBUG_FULL) \
        { \
            utils_trace_dump(a, b, c); \
        } \
    } while (0)


/*
 * Some macros to write in a Stream (Little Endian mode)
 */
#define UINT32_TO_STREAM(p, u32) {*(p)++ = (uint8_t)(u32); *(p)++ = (uint8_t)((u32) >> 8); *(p)++ = (uint8_t)((u32) >> 16); *(p)++ = (uint8_t)((u32) >> 24);}
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}
#define BDADDR_TO_STREAM(p, a)   {register int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (uint8_t) a[BD_ADDR_LEN - 1 - ijk];}
#define ARRAY_TO_STREAM(p, a, len) {register int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (uint8_t) a[ijk];}
#define REVERSE_ARRAY_TO_STREAM(p, a, len)  {register int ijk; for (ijk = 0; ijk < len; ijk++) *(p)++ = (uint8_t) a[len - 1 - ijk];}

/*
 * Some macros to read from a Stream (Little Endian mode)
 */
#define STREAM_TO_UINT8(u8, p)   {u8 = (uint8_t)(*(p)); (p) += 1;}
#define STREAM_TO_UINT16(u16, p) {u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); (p) += 2;}
#define STREAM_TO_UINT32(u32, p) {u32 = (((uint32_t)(*(p))) + ((((uint32_t)(*((p) + 1)))) << 8) + ((((uint32_t)(*((p) + 2)))) << 16) + ((((uint32_t)(*((p) + 3)))) << 24)); (p) += 4;}
#define STREAM_TO_BDADDR(a, p)   {register int ijk; register uint8_t *pbda = (uint8_t *)a + BD_ADDR_LEN - 1; for (ijk = 0; ijk < BD_ADDR_LEN; ijk++) *pbda-- = *p++;}
#define STREAM_TO_DEVCLASS(a, p) {register int ijk; register uint8_t *_pa = (uint8_t *)a + DEV_CLASS_LEN - 1; for (ijk = 0; ijk < DEV_CLASS_LEN; ijk++) *_pa-- = *p++;}
#define STREAM_TO_ARRAY(a, p, len) {register int ijk; for (ijk = 0; ijk < len; ijk++) ((UINT8 *) a)[ijk] = *p++;}
#define REVERSE_STREAM_TO_ARRAY(a, p, len) {register int ijk; register uint8_t *_pa = (uint8_t *)a + len - 1; for (ijk = 0; ijk < len; ijk++) *_pa-- = *p++;}

/*
 * trace_level_set
 */
void trace_level_set (trace_level_t level);

/*
 * trace_level_get
 */
trace_level_t trace_level_get (void);

/*
 * utils_sleep
 *
 * Sleep requested number of seconds
 */
void utils_sleep(int duration);

/*
 * utils_sleep
 *
 * Sleep requested number of milli-seconds
 */
void utils_msleep(int duration);
