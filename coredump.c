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

#include <stdint.h>
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"

#define COREDUMP_CPU_REGS_NVRAM_ID           (0x200 - 1)
#define COREDUMP_CPU_REGS_EXT_NVRAM_ID       (0x200 - 2)

/*
 * structure containing coredump information
 */
typedef struct
{
    uint32_t sp;
    uint32_t pc;
    uint32_t lr;
    uint32_t r[7];     /* R0-R6 */
} coredump_cpu_regs_t;

typedef struct
{
    uint32_t msp;
    uint32_t mr[5];     /* R0-R3, R12 */
    uint32_t mpc;
    uint32_t mlr;
    uint32_t mxpsr;
    uint32_t psp;
    uint32_t pr[5];     /* R0-R2, R12 */
    uint32_t ppc;
    uint32_t plr;
    uint32_t pxpsr;

    // full register set at exception
    uint32_t r[13];     /* R0-R12 */
    uint32_t sp;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
} coredump_cpu_regs_extended_t;
/*
 * Function declaration
 */
extern uint8_t mpaf_cfa_ConfigVSRead(uint16_t vsID, uint8_t itemLength, uint8_t* payload);
extern uint32_t mpaf_cfa_ConfigVSDelete(uint16_t vsID);
void coredump_read(void);
void coredump_erase(uint16_t nvram_id);


/*
 * coredump_read
 * This function checks if a CoreDump information entry is present is the NVRAM and displays it
 */
void coredump_read(void)
{
    coredump_cpu_regs_t cpu_regs;
    coredump_cpu_regs_extended_t cpu_regs_extended;
    uint8_t nb_read;
    wiced_result_t status;

    nb_read = mpaf_cfa_ConfigVSRead(COREDUMP_CPU_REGS_NVRAM_ID, sizeof(cpu_regs),
            (uint8_t *)&cpu_regs);
    if (nb_read != sizeof(cpu_regs))
    {
        WICED_BT_TRACE("No CPU Register coredump found\n");
    }
    else
    {
        WICED_BT_TRACE("CoreDump CPU Registers:\n");
        WICED_BT_TRACE("PC:0x%08X SP:0x%08X LR:0x%08X\n",
                cpu_regs.pc, cpu_regs.sp, cpu_regs.lr);

        WICED_BT_TRACE("R0=0x%08X R1=0x%08X R2=0x%08X\n",
                cpu_regs.r[0], cpu_regs.r[1], cpu_regs.r[2]);
        WICED_BT_TRACE("R3=0x%08X R4=0x%08X R5=0x%08X\n",
                cpu_regs.r[3], cpu_regs.r[4], cpu_regs.r[5]);
        WICED_BT_TRACE("R6=0x%08X\n",
                cpu_regs.r[6]);

        /*
         * Erase the Coredump entry from the NVRAM
         * This can be done later (once the coredump has been fully investigated)
         * Do it here for the moment
         */
        coredump_erase(COREDUMP_CPU_REGS_NVRAM_ID);

        WICED_BT_TRACE("\n");

    }

    nb_read = mpaf_cfa_ConfigVSRead(COREDUMP_CPU_REGS_EXT_NVRAM_ID, sizeof(cpu_regs_extended),
            (uint8_t *)&cpu_regs_extended);
    if (nb_read != sizeof(cpu_regs_extended))
    {
        WICED_BT_TRACE("No CPU Registers Extended coredump found\n");
    }
    else
    {
        WICED_BT_TRACE("CoreDump CPU Registers Extended:\n");

        WICED_BT_TRACE("MPC:0x%08X MSP:0x%08X MLR:0x%08X MXPSR:0x%08X\n",
                cpu_regs_extended.mpc, cpu_regs_extended.msp, cpu_regs_extended.mlr,
                cpu_regs_extended.mxpsr);

        WICED_BT_TRACE("MR0=0x%08X MR1=0x%08X MR2=0x%08X\n",
                cpu_regs_extended.mr[0], cpu_regs_extended.mr[1], cpu_regs_extended.mr[2]);
        WICED_BT_TRACE("MR3=0x%08X MR4=0x%08X\n",
                cpu_regs_extended.mr[3], cpu_regs_extended.mr[4]);

        WICED_BT_TRACE("PC:0x%08X SP:0x%08X LR:0x%08X\n",
                cpu_regs_extended.pc, cpu_regs_extended.sp, cpu_regs_extended.lr);

        WICED_BT_TRACE("R0=0x%08X R1=0x%08X R2=0x%08X\n",
                cpu_regs_extended.r[0], cpu_regs_extended.r[1], cpu_regs_extended.r[2]);
        WICED_BT_TRACE("R3=0x%08X R4=0x%08X R5=0x%08X\n",
                cpu_regs_extended.r[3], cpu_regs_extended.r[4], cpu_regs_extended.r[5]);
        WICED_BT_TRACE("R6=0x%08X R7=0x%08X R8=0x%08X\n",
                cpu_regs_extended.r[6]);

        /*
         * Erase the Coredump entry from the NVRAM
         * This can be done later (once the coredump has been fully investigated)
         * Do it here for the moment
         */
        coredump_erase(COREDUMP_CPU_REGS_EXT_NVRAM_ID);
    }
}

/*
 * coredump_erase
 * This function erases a CoreDump information entry from the NVRAM.
 * Note, that only ONE CoreDump entry can be present in NVRAM and the FW will not overwrite.
 * So, this entry must be erased once investigated (to allow the FW to write a new one for next crash)
 */
void coredump_erase(uint16_t vsID)
{
    int32_t status;

    status = mpaf_cfa_ConfigVSDelete(vsID);
    if (status == WICED_FALSE)
    {
        WICED_BT_TRACE("Err: coredump_erase id:%d failed\n", vsID);
    }
}
