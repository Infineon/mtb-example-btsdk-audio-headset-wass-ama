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
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <getopt.h>
#include <libgen.h>
#include <ctype.h>

#include "utils.h"
#include "protocol.h"
#include "serial.h"
#include "utils.h"
#include "hci.h"
#include "wiced.h"
#include "lrac.h"

/*
 * Definitions
 */
#define TOOL_VERSION        "0.5"
#define LINUX_DEV_PREFIX    "/dev/ttyS"
/*
 * Global variables
 */
char *p_device = NULL;
char device[100] = { 0 };

int baudrate = 115200;
int flow_control = 1;

uint8_t local_bdaddr[BD_ADDR_LEN];
uint8_t local_bdaddr_command = 0;

uint8_t peer_bdaddr[BD_ADDR_LEN];
uint8_t peer_bdaddr_command = 0;

uint8_t lrac_config[2] = {0xFF, 0xFF};
uint8_t lrac_config_command = 0;

uint8_t button_command = 0;
uint8_t button_id;

uint8_t audio_insert_command = 0;
uint8_t audio_insert_id;

uint8_t audio_insert_ext_command = 0;
uint8_t audio_insert_ext_id;

uint8_t ble_adv_command = 0;
uint8_t ble_adv_mode;

uint8_t switch_command = 0;
uint8_t prevent_glitch;

uint8_t buffer_stat_command = 0;

uint8_t lrac_trace_level_command = 0;
uint8_t lrac_trace_level;

uint8_t fw_spi_logging_command = 0;
uint8_t fw_spi_logging;

uint8_t sleep_enable;
uint8_t sleep_enable_command = 0;

uint8_t jitter_buffer_target;
uint8_t jitter_buffer_target_command = 0;

int8_t elna_gain;
uint8_t elna_gain_command = 0;

char *p_write_binary_file;
uint8_t write_binary_file_command = 0;

uint32_t flash_offset = 0;

int16_t nvwrite_id;
uint8_t nvwrite_id_command = 0;

uint8_t *p_nvdata = NULL;
uint32_t nvdata_length = 0;

/*
 * hci_event_cback
 *
 * Handles HCI Events which has not been handled by HCI
 */
static void hci_event_cback (uint8_t event , uint8_t *p_data, int data_len)
{
    TRACE_DBG("HCI Event:0x%x len:%d ", event, data_len);
}

/*
 * wiced_event_cback
 *
 * Handles Wiced Events which has not been handled by Wiced
 */
static void wiced_event_cback (uint16_t opcode , uint8_t *p_data, int data_len)
{
    TRACE_INFO(" Wiced OpCode:0x%x len:%d ", opcode, data_len);
}

/*
 * protocol_cback
 *
 * Handles HCI and Wiced Events which has not been handled (should not be called)
 */
static void protocol_cback(protocol_event_t event, uint16_t id, uint8_t *p_data, int data_len)
{
    TRACE_INFO("event:%d id:0x%x len:%d", event, id, data_len);

    switch(event)
    {
    case PROTOCOL_EVENT_RX_WICED_EVENT:
        wiced_event_cback(id, p_data, data_len);
        break;

    case PROTOCOL_EVENT_RX_HCI_EVENT:
        hci_event_cback(id, p_data, data_len);
        break;

    default:
        TRACE_ERR("Unexpected Protocol event:%d len:%d", event, data_len);
        break;
    }
}

/*
 * print_usage
 */
void print_usage(char *p_name)
{
     printf("Utility program to configure LRAC device\n");
     printf("USAGE:     %s [OPTION]... \n", basename(p_name));
     printf("    -help             get option help information\n");
     printf("    -device dev       UART Port (e.g. COM11 or /dev/ttyS10)\n");
     printf("    -baudrate rate    UART Baudrate (default is 115200)\n");
     printf("    -flowcontrol f    UART RTS/CTS Flow control (default is 1)\n");
     printf("    -bdaddr addr      Set Local BdAddr (e.g. -bdaddr 001122334455\n");
     printf("    -peer addr        Set Peer BdAddr (e.g. -peer 001122334455\n");
     printf("    -config cfg       Set (P)rimary/(S)econdary and (L)eft/(R)ight\n");
     printf("                      (parameter PL, PR, SL or SR\n");
     printf("    -button id        Simulate Button Press (e.g. --button 1)\n");
     printf("    -audio_insert id  Audio Insert (e.g. --audio_insert 1)\n");
     printf("    -audio_insert_ext cmd  Audio Insert Extended. cmd can be:\n");
     printf("                             0 Audio Insert Stop\n");
     printf("                             1 Audio Insert Suspend\n");
     printf("                             2 Audio Insert Stop & Flush\n");
     printf("                             3 Audio Insert Resume\n");
     printf("    -ble_adv mode     Set LE Adv Mode (mode [0..8]\n");
     printf("    -switch           LRAC Switch\n");
     printf("    -sleep 0/1/2      PDS Sleep disable/transport/no-transport\n");
     printf("    -buf_stat         Print buffer statistics\n");
     printf("    -lrac_trace level Set LRAC Trace Level[0..2]\n");
     printf("    -verbose level    Verbose Debug level [0..3]\n");
     printf("    -fw_spi_logging   Enable/Disable SPI [0/1]\n");
     printf("    -jitter_target t  Set the Jitter Buffer Target depth [0..100]\n");
     printf("    -elna gain        Set the eLNA Gain [-128..127]\n");
     printf("    -wbftf file       Write Binary File To Flash\n");
     printf("    -foffset offset   Flash Offset (Hexadecimal value)\n");
     printf("    -nvwrite id       Write NVRAM Id in flash (Hexadecimal value)\n");
     printf("    -data XX...       NVRAM Data (see -nvwrite)\n");

     printf("\n");
     printf("Version %s\n", TOOL_VERSION);
     printf("Copyright Cypress 2017-2018\n");
};

/*
 * parse_com_port
 */
int parse_com_port(char *p_dev_name)
{
    int windows_com_port_nb;
    int status;

    TRACE_DBG("Device:%s", p_dev_name);

    /* Check if the device parameter is passed using the Windows format (e.g. COM5) */
    status = sscanf(p_dev_name, "COM%d", &windows_com_port_nb);
    if (status == 1)
    {
        TRACE_DBG("Windows COM port format:COM%d", windows_com_port_nb);
        if (windows_com_port_nb > 64)
        {
            TRACE_ERR("Cygwin does not handle COM ports higher than 64");
            TRACE_ERR("Use the DeviceManager to force Windows to use a lower number");
        }
        sprintf(device, "%s%d", LINUX_DEV_PREFIX, windows_com_port_nb-1);
        TRACE_DBG("COM%d->%s", windows_com_port_nb, device);
        p_device = device;
    }
    else
    {
        /* Check if the device parameter is passed using the Linux/Cygwin format */
        status = sscanf(p_dev_name, "/dev/ttyS%d", &windows_com_port_nb);
        if ((status == 1) &&
            (windows_com_port_nb > 64))
        {
            TRACE_ERR("Cygwin does not handle COM ports higher than 64");
            TRACE_ERR("Use the DeviceManager to force Windows to use a lower number");
        }
        /* Linux/Cygwin device format */
        p_device = optarg;
    }
    return 0;
}

/*
 * hexachar_to_int
 */
int hexachar_to_int(char hex_char)
{
    if ((hex_char >= '0') && (hex_char <= '9'))
    {
        return hex_char - '0';
    }
    if ((hex_char >= 'a') && (hex_char <= 'f'))
    {
        return hex_char - 'a' + 10;
    }
    if ((hex_char == 'A') || (hex_char <= 'F'))
        return hex_char - 'A' + 10;

    return -1;
}
/*
 * parse_local_bdaddr
 */
int parse_local_bdaddr(char *p_bdaddr_string)
{
    int i = 0;
    uint8_t *p = p_bdaddr_string;
    int value;
    uint8_t val;


    if (strlen(p_bdaddr_string) != (BD_ADDR_LEN * 2))
    {
        TRACE_ERR("Wrong BdAddr parameter (12 hex expected)");
        return -1;
    }

    while(*p)
    {
        value = hexachar_to_int(*p);
        if (value < 0)
        {
            TRACE_ERR("Wrong BdAddr parameter (%c)", *p);
            return -1;
        }
        p++;
        val = value << 4;

        value = hexachar_to_int(*p);
        if (value < 0)
        {
            TRACE_ERR("Wrong BdAddr parameter (%c)", *p);
            return -1;
        }
        p++;
        val |= value;

        local_bdaddr[i++] = val;
    }
    local_bdaddr_command = 1;
    TRACE_DBG("BdAddr:%02X:%02X:%02X:%02X:%02X:%02X",
            local_bdaddr[0], local_bdaddr[1], local_bdaddr[2],
            local_bdaddr[3], local_bdaddr[4], local_bdaddr[5]);
    return 0;
}

/*
 * parse_peer_bdaddr
 */
int parse_peer_bdaddr(char *p_bdaddr_string)
{
    int i = 0;
    uint8_t *p = p_bdaddr_string;
    int value;
    uint8_t val;


    if (strlen(p_bdaddr_string) != (BD_ADDR_LEN * 2))
    {
        TRACE_ERR("Wrong BdAddr parameter (12 hex expected)");
        return -1;
    }

    while(*p)
    {
        value = hexachar_to_int(*p);
        if (value < 0)
        {
            TRACE_ERR("Wrong BdAddr parameter (%c)", *p);
            return -1;
        }
        p++;
        val = value << 4;

        value = hexachar_to_int(*p);
        if (value < 0)
        {
            TRACE_ERR("Wrong BdAddr parameter (%c)", *p);
            return -1;
        }
        p++;
        val |= value;

        peer_bdaddr[i++] = val;
    }
    peer_bdaddr_command = 1;
    TRACE_DBG("BdAddr:%02X:%02X:%02X:%02X:%02X:%02X",
            peer_bdaddr[0], peer_bdaddr[1], peer_bdaddr[2],
            peer_bdaddr[3], peer_bdaddr[4], peer_bdaddr[5]);
    return 0;
}

/*
 * parse_lrac_config
 */
int parse_lrac_config(char *p_config_string)
{
    if ((p_config_string[0] == 'P') || (p_config_string[0] == 'p'))
        lrac_config[0] = 0x00;
    else if ((p_config_string[0] == 'S') || (p_config_string[0] == 's'))
        lrac_config[0] = 0x01;
    else if ((p_config_string[0] == 'U') || (p_config_string[0] == 'u'))
        lrac_config[0] = 0xFF;
    else
    {
        fprintf(stderr, "Wrong Primary/Secondary Config (%c)", p_config_string[0]);
        return -1;
    }

    if ((p_config_string[1] == 'L') || (p_config_string[1] == 'l'))
        lrac_config[1] = 0x00;
    else if ((p_config_string[1] == 'R') || (p_config_string[1] == 'r'))
        lrac_config[1] = 0x01;
    else if ((p_config_string[1] == 'U') || (p_config_string[1] == 'u'))
        lrac_config[1] = 0xFF;
    else
    {
        fprintf(stderr, "Wrong Left/Right Config (%c)", p_config_string[1]);
        return -1;
    }
    lrac_config_command = 1;
    TRACE_DBG("Config P/S:%d L/R:%d", lrac_config[0], lrac_config[1]);
    return 0;
}

/*
 * string_to_hex
 */
uint32_t string_to_hex(uint8_t **pp_nvdata, char *p_string)
{
    int length;
    uint8_t *p_buffer, *p;
    int value;
    uint8_t val;

    if (p_string == NULL)
        return 0;

    if (pp_nvdata == NULL)
    {
        fprintf(stderr, "pp_nvdata is null\n");
        return 0;
    }

    length = strlen(p_string);
    if (length & 1)
    {
        fprintf(stderr, "Odd Hex string length:%d\n", length);
        return 0;
    }

    p_buffer = malloc(length/2);
    if (p_buffer == NULL)
    {
        fprintf(stderr, "malloc(%d) failed\n", length/2);
        return 0;
    }

    p = p_buffer;
    while(*p_string != '\0')
    {
        value = hexachar_to_int(*p_string);
        if (value < 0)
        {
            free(p_buffer);
            TRACE_ERR("Wrong BdAddr parameter (%c)", *p_string);
            return -1;
        }
        p_string++;
        val = value << 4;

        value = hexachar_to_int(*p_string);
        if (value < 0)
        {
            free(p_buffer);
            TRACE_ERR("Wrong BdAddr parameter (%c)", *p_string);
            return -1;
        }
        p_string++;
        val |= value;

        *p++ = val;
    }

    *pp_nvdata = p_buffer;

    return length / 2;
}

/*
 * parse_cmd_line
 */
int parse_cmd_line(int argc, char **argv)
{
    int opt;
    int option_index = 0;
    struct option long_options[] =
    {
        /* All the supported parameters must be defined here even if compiled out because the
         * index number must be identical whatever the compilation options are (see below switch
         * block).  Validity of the option is checked in the switch.  Index 1 to 25 are
         * only to set trace levels
         */
            {"help", no_argument, 0, 'h'},                  /* Help => no parameter */
            {"device", required_argument, 0, 'd'},          /* Device => 1 parameter */
            {"baudrate", required_argument, 0, 'b'},        /* Baudrate => 1 parameter */
            {"flowcontrol", required_argument, 0, 'f'},     /* FlowControl (RTS/CTS) => 1 parameter */
            {"bdaddr", required_argument, 0, 'l'},          /* Local BdAddr => 1 parameter */
            {"peer", required_argument, 0, 'p'},            /* Peer BdAddr => 1 parameter */
            {"config", required_argument, 0, 'c'},          /* LRAC Config => 1 parameter */
            {"verbose", required_argument, 0, 'v'},         /* Verbose => 1 parameter */
            {"button", required_argument, 0, 'u' },         /* Button => 1 parameter */
            {"audio_insert", required_argument, 0, 'a' },   /* Audio Insert => 1 parameter */
            {"audio_insert_ext", required_argument, 0, 'w' },/* Audio Insert Extended => 1 parameter */
            {"ble_adv", required_argument, 0, 'e' },        /* LE Adv => 1 parameter */
            {"switch", required_argument, 0, 's' },         /* LRAC Switch => 1 parameter */
            {"buf_stat", no_argument, 0, 'g' },             /* Buffer Stats => no parameter */
            {"lrac_trace", required_argument, 0, 'i' },     /* LRAC Trace Level => 1 parameter */
            {"fw_spi_logging", required_argument, 0, 'j' }, /* FW SPI Logging => 1 parameter */
            {"sleep", required_argument, 0, 'k' },          /* Sleep Config => 1 parameter */
            {"jitter_target", required_argument, 0, 'm' },  /* Jitter Buffer Target => 1 parameter */
            {"elna", required_argument, 0, 'n' },           /* eLNA Gain => 1 parameter */
            {"wbftf", required_argument, 0, 'o' },          /* Write Bin file to Flash => 1 parameter */
            {"foffset", required_argument, 0, 'q' },        /* Flash Offset => 1 parameter */
            {"nvwrite", required_argument, 0, 'r' },        /* NVRAM Write Id => 1 parameter */
            {"data", required_argument, 0, 't' },           /* Data => 1 parameter */

            {NULL, 0, NULL, 0}
    };

    while (1)
    {
        opt = getopt_long_only (argc, argv, "b:d:f:v:l:c:h:alsgij", long_options, &option_index);
        if (opt == -1)
        {
            break;
        }

        switch (opt)
        {
        case 'd':
            parse_com_port(optarg);
            break;

        case 'b':
            baudrate = atoi(optarg);
            break;

        case 'f':
            flow_control = atoi(optarg);
            break;

        case 'v':
            trace_level_set(atoi(optarg));
            break;

        case 'l':
            if (parse_local_bdaddr(optarg) < 0)
            {
                print_usage(argv[0]);
                return -1;
            }
            break;

        case 'p':
            if (parse_peer_bdaddr(optarg) < 0)
            {
                print_usage(argv[0]);
                return -1;
            }
            break;

        case 'c': /* LRAC config */
            if (parse_lrac_config(optarg) < 0)
            {
                print_usage(argv[0]);
                return -1;
            }
            break;

        case 'u':
            button_id = atoi(optarg);
            button_command = 1;
            break;

        case 'a':
            audio_insert_id = atoi(optarg);
            audio_insert_command = 1;
            break;

        case 'w':
            audio_insert_ext_id = atoi(optarg);
            audio_insert_ext_command = 1;
            break;

        case 'e':
            ble_adv_command = 1;
            ble_adv_mode = atoi(optarg);
            printf("ble_adv_mode:%d\n", ble_adv_mode);
            break;

        case 's':
            switch_command = 1;
            prevent_glitch = atoi(optarg);
            printf("prevent_glitch:%d\n", prevent_glitch);
            break;

        case 'g':
            buffer_stat_command = 1;
            break;

        case 'i':
            lrac_trace_level = atoi(optarg);
            lrac_trace_level_command = 1;
            break;

        case 'j':
            fw_spi_logging = atoi(optarg);
            fw_spi_logging_command = 1;
            break;

        case 'k':
            sleep_enable = atoi(optarg);
            sleep_enable_command = 1;
            break;

        case 'm':
            jitter_buffer_target = atoi(optarg);
            jitter_buffer_target_command = 1;
            break;

        case 'n':
            {
                long elna_gain_lint;
                char *ptr;

                elna_gain_lint = strtol(optarg, &ptr, 10);

                if ((elna_gain_lint > 127) ||
                    (elna_gain_lint < -128))
                {
                    fprintf(stderr, "wrong eLNA Gain %ld\n", elna_gain_lint);
                    print_usage(argv[0]);
                    return -1;
                }
                elna_gain = (int8_t)elna_gain_lint;
                elna_gain_command = 1;
            }
            break;

        case 'o':
            p_write_binary_file = optarg;
            write_binary_file_command = 1;
            break;

        case 'q':
            {
                char *ptr;
                flash_offset = strtol(optarg, &ptr, 16);
                if (flash_offset == 0)
                {
                    fprintf(stderr, "invalid offset %s\n", optarg);
                    return -1;
                }
            }
            break;

        case 'r':
            {
                char *ptr;
                nvwrite_id = strtol(optarg, &ptr, 16);
                if (nvwrite_id == 0)
                {
                    fprintf(stderr, "invalid NVRAM Id %s\n", optarg);
                    return -1;
                }
                TRACE_DBG("nvwrite_id:0x%X", nvwrite_id);
                nvwrite_id_command = 1;
            }
            break;

            case 't':
            nvdata_length = string_to_hex(&p_nvdata, optarg);
            TRACE_DBG("nvdata_length:%d", nvdata_length);
            break;

        case 'h':
        default:
            print_usage(argv[0]);
            return -1;
            break;
        }
    }
    return 0;
}

/*
 * Main
 */
int main(int argc, char *argv[])
{
    int status;
    int i;
    uint8_t tx_param[255];
    uint8_t rx_param[255];
    uint8_t *p;

    /* Parse the command line parameters */
    status = parse_cmd_line(argc, argv);
    if (status < 0)
    {
        /* the cmd line parser failed of help menu */
        /* error traces are done in bte_parse_cmd_line */
        return status;
    }

    if (p_device == NULL)
    {
        TRACE_ERR("No device selected");
        print_usage(argv[0]);
        return -1;
    }

    /* Initialization */
    protocol_init();
    hci_init();
    wiced_init();

    /* Open the Protocol (Com port). */
    status = protocol_open(p_device, baudrate, flow_control, protocol_cback);
    if (status < 0)
    {
        TRACE_ERR("protocol_open failed");
        return status;
    }

    /* Sleep some time to allow the Serial driver to start */
    utils_msleep(500);

    /* If Change Local BdAddr parameter present */
    if (local_bdaddr_command)
    {
        /* Set the Local Bdaddr */
        status = lrac_local_bdaddr_write(local_bdaddr);
        if (status < 0)
        {
            TRACE_ERR("lrac_local_bdaddr_write failed");
            return status;
        }
    }

    /* If Change Peer BdAddr parameter present */
    if (peer_bdaddr_command)
    {
        /* Set the Peer Bdaddr */
        status = lrac_peer_bdaddr_write(peer_bdaddr);
        if (status < 0)
        {
            TRACE_ERR("lrac_peer_bdaddr_write failed");
            return status;
        }
    }

    /* If Change LRAC Configuration parameter present */
    if (lrac_config_command)
    {
        /* Set the LRAC config */
        status = lrac_config_write(&lrac_config[0]);
        if (status < 0)
        {
            TRACE_ERR("lrac_config_write failed");
            return status;
        }
    }

    if (button_command)
    {
        /* Sent the button Id */
        status = lrac_button_sent(button_id);
        if (status < 0)
        {
            TRACE_ERR("lrac_button_sent failed");
            return status;
        }
    }

    if (audio_insert_command)
    {
        printf("audio_insert:%d\n", audio_insert_id);
        /* Sent the Audio Insert command*/
        status = lrac_audio_insert_sent(audio_insert_id);
        if (status < 0)
        {
            TRACE_ERR("lrac_audio_insert_sent failed");
            return status;
        }
    }

    if (audio_insert_ext_command)
    {
        printf("audio_insert_ext:%d\n", audio_insert_ext_id);
        /* Sent the Audio Insert Extended command*/
        status = lrac_audio_insert_ext_sent(audio_insert_ext_id);
        if (status < 0)
        {
            TRACE_ERR("lrac_audio_insert_ext_sent failed");
            return status;
        }
    }

    if (ble_adv_command)
    {
        printf("ble_adv mode:%d\n", ble_adv_mode);
        /* Sent the LE Adv command */
        status = lrac_ble_adv_sent(ble_adv_mode);
        if (status < 0)
        {
            TRACE_ERR("lrac_ble_adv_sent failed");
            return status;
        }
    }

    if (switch_command)
    {
        printf("switch prevent_glitch:%d\n", prevent_glitch);
        /* Sent the Switch command */
        status = lrac_switch_sent(prevent_glitch);
        if (status < 0)
        {
            TRACE_ERR("lrac_switch_sent failed");
            return status;
        }
    }

    if (buffer_stat_command)
    {
        printf("Buffer Statistics\n");
        /* Sent the Read Buffer Statistics command */
        status = lrac_read_buffer_stat();
        if (status < 0)
        {
            TRACE_ERR("lrac_read_buffer_stat failed");
            return status;
        }
    }

    if (lrac_trace_level_command)
    {
        printf("Set LRAC Trace Level:%d\n", lrac_trace_level);
        /* Sent the Set LRAC Trace Level command */
        status = lrac_trace_level_set(lrac_trace_level);
        if (status < 0)
        {
            TRACE_ERR("lrac_read_buffer_stat failed");
            return status;
        }
    }

    if (fw_spi_logging_command)
    {
        printf("FW SPI Logging:%d\n", fw_spi_logging);
        /* Sent the Set LRAC Trace Level command */
        status = wiced_cmd_fw_spi_debug_enable(fw_spi_logging);
        if (status < 0)
        {
            TRACE_ERR("wiced_cmd_fw_spi_debug_enable failed");
            return status;
        }
    }

    if (sleep_enable_command)
    {
        printf("Sleep enable:%d\n", sleep_enable);
        /* Write the Sleep Configuration in NVRAM */
        status = lrac_sleep_config(sleep_enable);
        if (status < 0)
        {
            TRACE_ERR("wiced_cmd_fw_spi_debug_enable failed");
            return status;
        }
    }

    if (jitter_buffer_target_command)
    {
        printf("Jitter Buffer Target depth:%d\n", jitter_buffer_target);
        /* Send the Jitter Buffer Target depth */
        status = wiced_cmd_jitter_buffer_target_set(jitter_buffer_target);
        if (status < 0)
        {
            TRACE_ERR("wiced_cmd_jitter_buffer_target_set failed");
            return status;
        }
    }

    if (elna_gain_command)
    {
        printf("eLNA Gain:%d\n", (int)elna_gain);
        /* Send the eLNA Gain */
        status = wiced_cmd_elna_gain_set(elna_gain);
        if (status < 0)
        {
            TRACE_ERR("wiced_cmd_elna_gain_set failed");
            return status;
        }
    }

    if (write_binary_file_command)
    {
        if (flash_offset == 0)
        {
            fprintf(stderr, "invalid offset %d\n", flash_offset);
            print_usage(argv[0]);
            return -1;
        }
        printf("Write Binary File:%s To Flash offset:0x%x\n", p_write_binary_file, flash_offset);
        /* Send the Binary file */
        status = wiced_cmd_write_binary_file_to_flash(p_write_binary_file, flash_offset);
        if (status < 0)
        {
            TRACE_ERR("wiced_cmd_write_binary_file_to_flash failed");
            return status;
        }
    }

    if (nvwrite_id_command)
    {
        TRACE_INFO("NVRAM Write Id:0x%x length:%d", nvwrite_id, nvdata_length);

        status = wiced_cmd_nvram_write(nvwrite_id, p_nvdata, nvdata_length);
        free(p_nvdata);
        if (status < 0)
        {
            TRACE_ERR("wiced_cmd_nvram_write failed");
            return status;
        }
    }

    /* If Change Local BdAddr or LRAC Configuration parameter(s) present */
    if (local_bdaddr_command || lrac_config_command)
    {
        /* Reset the Chip */
        status = wiced_cmd_reset();
        if (status < 0)
        {
            TRACE_ERR("wiced_cmd_reset failed");
            return status;
        }
    }

    /* Close the Protocol (Com port). */
    status = protocol_close();
    if (status < 0)
    {
        TRACE_ERR("protocol_close failed");
        return status;
    }

    TRACE_INFO("Success");

    return 0;
}
