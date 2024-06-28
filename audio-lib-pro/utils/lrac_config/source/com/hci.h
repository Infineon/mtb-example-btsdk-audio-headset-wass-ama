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

/*
 *  Definitions for HCI groups
 */
#define HCI_GRP_LINK_CONTROL_CMDS       (0x01 << 10)            /* 0x0400 */
#define HCI_GRP_LINK_POLICY_CMDS        (0x02 << 10)            /* 0x0800 */
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS (0x03 << 10)            /* 0x0C00 */
#define HCI_GRP_INFORMATIONAL_PARAMS    (0x04 << 10)            /* 0x1000 */
#define HCI_GRP_STATUS_PARAMS           (0x05 << 10)            /* 0x1400 */
#define HCI_GRP_TESTING_CMDS            (0x06 << 10)            /* 0x1800 */
#define HCI_GRP_VENDOR_SPECIFIC         (0x3F << 10)            /* 0xFC00 */

/*
 * Some HCI OpCode definitions
 */
#define HCI_RESET                       (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)

#define HCI_READ_LOCAL_VERSION          (0x0001 | HCI_GRP_INFORMATIONAL_PARAMS)
#define HCI_READ_BD_ADDR                (0x0009 | HCI_GRP_INFORMATIONAL_PARAMS)

#define HCI_WRITE_LOOPBACK              (0x0002 | HCI_GRP_TESTING_CMDS)

/*
 * Vendor Specific commands
 */
#define HCI_WRITE_BDADDR                (0x0001 | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_FACTORY_COMMIT_BDADDR       (0x0010 | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_DWNLD_MINIDRV               (0x002E | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_WRITE_RAM                   (0x004C | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_READ_RAM                    (0x004D | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_LAUNCH_RAM                  (0x004E | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_READ_VERBOSE_CFG_VER_INFO   (0x0079 | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_ENABLE_BTW                  (0x0053 | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_UPGRADE_FIRMWARE            (0x0122 | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_BCS_TIMELINE                (0x0140 | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_SECTOR_ERASE                (0x035E | HCI_GRP_VENDOR_SPECIFIC)
#define HCI_CHIP_ERASE                  (0x03CE | HCI_GRP_VENDOR_SPECIFIC)

/*
 * HCI Events
 */
#define HCI_EVENT_CONNECTION_COMPLETE   0x03
#define HCI_EVENT_COMMAND_COMPLETE      0x0E
#define HCI_EVENT_NUM_CPLT_PKT          0x13

/*
 * HCI Status
 */
#define HCI_STATUS_SUCCESS              0x00

/*
 * Chip Id
 */
#define CHIP_ID_20706A1                 0x66        /* 20706A1 ChipId */
#define CHIP_ID_20706A2                 0x7E        /* 20706A2 ChipId */

/*
 * Function definitions
 */
int hci_init(void);

int hci_cmd_reset(void);
int hci_cmd_read_chip_id(void);
int hci_cmd_check_chip_id(uint8_t chip_id);
int hci_cmd_read_ram(uint32_t fw_address, uint8_t length, uint8_t *p_data);
int hci_cmd_write_ram(uint32_t fw_address, uint8_t length, uint8_t *p_data);
int hci_cmd_sector_erase(uint32_t fw_address);
int hci_cmd_download_minidriver(void);
int hci_cmd_launch_ram(uint32_t fw_address);

/* The following functions are for internal use only (no API) */
int hci_event_handler(uint8_t event, uint8_t *p_data, uint16_t length);
int hci_acl_handler(uint16_t handle, uint8_t *p_data, uint16_t length);
int hci_sco_handler(uint16_t handle, uint8_t *p_data, uint16_t length);
