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

#include "app_bt.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_app_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_memory.h"
#include "app_main.h"
#include "app_trace.h"
#ifndef AMA_ENABLED
#include "app_ble.h"
#endif
#include "app_handsfree.h"
#include "app_nvram.h"
#include "app_audio_insert.h"
#include "bt_hs_spk_control.h"
#include "bt_hs_spk_handsfree.h"

/*
 * Definitions
 */

/*
 * Structures
 */

/*
 * External definitions
 */
wiced_result_t wiced_bt_dev_set_local_name (char *p_name);

/*
 * Local functions
 */
static wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data);

/*
 * Global variables
 */

/*
 * app_bt_init
 */
wiced_result_t app_bt_init(void)
{
    wiced_result_t status;

#ifdef CYW20721B2
    wiced_bt_dev_set_vse_callback_num(2);
#endif

    APP_TRACE_DBG("call wiced_bt_stack_init\n");
    status = wiced_bt_stack_init(app_bt_management_callback, &wiced_bt_cfg_settings,
            wiced_app_cfg_buf_pools);
    if( status != WICED_BT_SUCCESS )
    {
        APP_TRACE_ERR("wiced_bt_stack_init returns error: %d\n", status);
        return status;
    }
    return status;
}

/*
 *  Management callback receives various notifications from the stack
 */
static wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
#ifdef APP_TRACE_ENABLED
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
#endif
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    uint8_t                             pairing_result;
    wiced_bt_ble_connection_param_update_t *p_ble_conn_param_update;
    wiced_bt_device_address_t           lrac_peer_addr;
    uint8_t *p_linkkey;

    APP_TRACE_DBG("bt_management event:%d\n", event);

    switch( event )
    {
        /* Bluetooth stack enabled */
        case BTM_ENABLED_EVT:
            if( p_event_data->enabled.status != WICED_BT_SUCCESS )
            {
                APP_TRACE_ERR("BTM_ENABLED_EVT status:%d\n", p_event_data->enabled.status);
            }
            else
            {
                APP_TRACE_DBG("Free Bytes before Init:%d\n", wiced_memory_get_free_bytes());

                /* Main post initialization */
                app_main_post_init();

                WICED_BT_TRACE("Free Bytes After Init:%d\n", wiced_memory_get_free_bytes());
                app_main_free_memory_check();
            }
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PIN_REQUEST_EVT:
            APP_TRACE_DBG("remote address= %B\n", p_event_data->pin_request.bd_addr);
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS,
                    WICED_PIN_CODE_LEN, (uint8_t *) &pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            // If this is just works pairing, accept. Otherwise send event to the MCU to confirm the same value.
            WICED_BT_TRACE("BTM_USER_CONFIRMATION_REQUEST_EVT %B\n", p_event_data->user_confirmation_request.bd_addr);
            if (p_event_data->user_confirmation_request.just_works)
            {
                WICED_BT_TRACE("just_works\n");
                wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                        p_event_data->user_confirmation_request.bd_addr);
            }
            else
            {
                WICED_BT_TRACE("Not support user_confirmation_request\n");
                wiced_bt_dev_confirm_req_reply(WICED_BT_UNSUPPORTED,
                        p_event_data->user_confirmation_request.bd_addr);
            }
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            APP_TRACE_DBG("Err: PassKey Notification. BDA %B, Key %d \n",
                    p_event_data->user_passkey_notification.bd_addr,
                    p_event_data->user_passkey_notification.passkey );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            APP_TRACE_DBG("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT (%B)\n",
                    p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);

            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
//            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_ALL_PROFILES_NO;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT:
            APP_TRACE_DBG("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT (%B, io_cap: 0x%02X) \n",
                          p_event_data->pairing_io_capabilities_br_edr_response.bd_addr,
                          p_event_data->pairing_io_capabilities_br_edr_response.io_cap);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* Use the default security for LE */
            APP_TRACE_DBG("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %B\n",
                    p_event_data->pairing_io_capabilities_ble_request.bd_addr);
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;
            if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
            }
            APP_TRACE_DBG( "Pairing Result: %d\n", pairing_result);
            if (pairing_result != WICED_BT_SUCCESS)
            {
                /* Check if the pairing complete event is used for PS-link. */
                app_lrac_config_peer_addr_get(lrac_peer_addr);

                if (memcmp((void *) lrac_peer_addr,
                           (void *) p_pairing_cmpl->bd_addr,
                           sizeof(wiced_bt_device_address_t)) == 0)
                {   /* This link key update event is used for the PS link. */
                    app_lrac_link_key_reset();
                }
            }
            break;

        case BTM_SECURITY_FAILED_EVT:
            APP_TRACE_DBG("BTM Sec Failed Status:%d BdAddr:%B\n",
                    p_event_data->security_failed.status,
                    p_event_data->security_failed.bd_addr);
            /* Check if the pairing complete event is used for PS-link. */
            app_lrac_config_peer_addr_get(lrac_peer_addr);

            if (memcmp((void *) lrac_peer_addr,
                       (void *) p_event_data->security_failed.bd_addr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {   /* This link key update event is used for the PS link. */
                app_lrac_link_key_reset();
            }
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
#ifdef APP_TRACE_ENABLED
            p_encryption_status = &p_event_data->encryption_status;

            APP_TRACE_DBG( "Encryption BdAddr:%B res:%d\n", p_encryption_status->bd_addr,
                    p_encryption_status->result );
#endif
#if (WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED == TRUE)
            if (p_encryption_status->transport == BT_TRANSPORT_LE)
                le_peripheral_encryption_status_changed(p_encryption_status);
#endif
            break;

        case BTM_SECURITY_REQUEST_EVT:
            if (bt_hs_spk_control_pairability_get())
            {
                wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            }
            else
            {
                // Pairing not allowed, return error
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            APP_TRACE_DBG("Updating LinkKey for BdAddr:%B\n",
                    p_event_data->paired_device_link_keys_update.bd_addr);

            /* Check if the link key is used for LRAC. */
            app_lrac_config_peer_addr_get(lrac_peer_addr);

            if (memcmp((void *) lrac_peer_addr,
                       (void *) p_event_data->paired_device_link_keys_update.bd_addr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {   /* This link key update event is used for the PS link. */
                /* Update key data. */
                app_lrac_link_key_update(&p_event_data->paired_device_link_keys_update.key_data);

#ifndef HCI_TRACE_OVER_TRANSPORT
                {
                    p_linkkey = p_event_data->paired_device_link_keys_update.key_data.br_edr_key;

                    WICED_BT_TRACE("BR/EDR LinkKey:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                            p_linkkey[0], p_linkkey[1], p_linkkey[2], p_linkkey[3],
                            p_linkkey[4], p_linkkey[5], p_linkkey[6], p_linkkey[7],
                            p_linkkey[8], p_linkkey[9], p_linkkey[10], p_linkkey[11],
                            p_linkkey[12], p_linkkey[13], p_linkkey[14], p_linkkey[15]);

                    WICED_BT_TRACE("LE local pairing security level:%02d\n",
                            p_event_data->paired_device_link_keys_update.key_data.le_keys.sec_level);
                    p_linkkey = p_event_data->paired_device_link_keys_update.key_data.le_keys.lltk ;
                    WICED_BT_TRACE("LE LLTK:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                            p_linkkey[0], p_linkkey[1], p_linkkey[2], p_linkkey[3], p_linkkey[4], p_linkkey[5], p_linkkey[6], p_linkkey[7],
                            p_linkkey[8], p_linkkey[9], p_linkkey[10], p_linkkey[11], p_linkkey[12], p_linkkey[13], p_linkkey[14], p_linkkey[15]);
                }
#endif
            }
            else
            {   /* This link key update event is used for AP link. */
                result = bt_hs_spk_control_btm_event_handler_link_key(event, &p_event_data->paired_device_link_keys_update) ? WICED_BT_SUCCESS : WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */
            APP_TRACE_DBG("LinkKey Request for BdAddr:%B\n",
                    p_event_data->paired_device_link_keys_request.bd_addr);

            /* Check if the link key request is used for LRAC. */
            app_lrac_config_peer_addr_get(lrac_peer_addr);

            if (memcmp((void *) lrac_peer_addr,
                       (void *) p_event_data->paired_device_link_keys_request.bd_addr,
                       sizeof(wiced_bt_device_address_t)) == 0)
            {   /* This link key request event is used for the PS link. */
                if (bt_hs_spk_control_misc_data_content_check((uint8_t *) app_lrac_link_key_get(), sizeof(wiced_bt_device_sec_keys_t)))
                {
                    memcpy((void *) &p_event_data->paired_device_link_keys_request.key_data,
                           (void *) app_lrac_link_key_get(),
                           sizeof(wiced_bt_device_sec_keys_t));

                    APP_TRACE_DBG("LinkKey Found\n");
#ifndef HCI_TRACE_OVER_TRANSPORT
                    {
                        uint8_t *p_linkkey;
                        p_linkkey = p_event_data->paired_device_link_keys_request.key_data.br_edr_key;
                        WICED_BT_TRACE(" *******************************************************\n");
                        WICED_BT_TRACE(" LinkKey:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                                p_linkkey[0], p_linkkey[1], p_linkkey[2], p_linkkey[3],
                                p_linkkey[4], p_linkkey[5], p_linkkey[6], p_linkkey[7],
                                p_linkkey[8], p_linkkey[9], p_linkkey[10], p_linkkey[11],
                                p_linkkey[12], p_linkkey[13], p_linkkey[14], p_linkkey[15]);
                        WICED_BT_TRACE(" *******************************************************\n");
                    }
#endif

                    result = WICED_BT_SUCCESS;
                }
                else
                {
                    result = WICED_BT_ERROR;
                }
            }
            else
            {   /* This link key request event is used for the AP link. */
                result = bt_hs_spk_control_btm_event_handler_link_key(event, &p_event_data->paired_device_link_keys_request) ? WICED_BT_SUCCESS : WICED_BT_ERROR;
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            app_nvram_local_irk_update(p_event_data->local_identity_keys_update.local_key_data);
            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            if (app_nvram_local_irk_get(p_event_data->local_identity_keys_request.local_key_data) == WICED_FALSE)
            {
                result = WICED_BT_NO_RESOURCES;
            }
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            WICED_BT_TRACE("BLE_ADVERT_STATE_CHANGED_EVT:%d\n", p_event_data->ble_advert_state_changed);
#ifndef AMA_ENABLED
            app_ble_advert_state_changed(p_event_data->ble_advert_state_changed);
#endif
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            p_ble_conn_param_update = &p_event_data->ble_connection_param_update;
            WICED_BT_TRACE ("BTM LE Connection Update event status:%d addr:%B interval:%d latency:%d timout:%d\n",
                                p_ble_conn_param_update->status,
                                p_ble_conn_param_update->bd_addr,
                                p_ble_conn_param_update->conn_interval,
                                p_ble_conn_param_update->conn_latency,
                                p_ble_conn_param_update->supervision_timeout);
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            /* LRAC handler */
            wiced_bt_lrac_power_mode_change_handler(&p_event_data->power_mgmt_notification);

            bt_hs_spk_control_btm_event_handler_power_management_status(&p_event_data->power_mgmt_notification);

            /* check to do eavesdropping recover when AP Link get into ACTIVE mode */
            /* NOTE: to check here because eavesdropping procedure will be rejected if
             * AP Link did NOT get into ACTIVE mode already */
            app_lrac_config_peer_addr_get(lrac_peer_addr);
            if (memcmp(p_event_data->power_mgmt_notification.bd_addr,
                        lrac_peer_addr,
                        sizeof(wiced_bt_device_address_t)) != 0)
            {
                if (p_event_data->power_mgmt_notification.status == WICED_POWER_STATE_ACTIVE)
                {
                    app_main_lrac_eavesdropping_state_recover();
                }
            }

            /* Check if Audio Insert must be resumed */
            app_audio_insert_resume();
            break;

        case BTM_SCO_CONNECTED_EVT:
        case BTM_SCO_DISCONNECTED_EVT:
        case BTM_SCO_CONNECTION_REQUEST_EVT:
        case BTM_SCO_CONNECTION_CHANGE_EVT:
            app_handsfree_sco_management_callback_handler(event, p_event_data);
            break;
        case BTM_BLE_PHY_UPDATE_EVT:
            /* LE PHY Update to 1M or 2M */
            WICED_BT_TRACE("PHY config is updated as TX_PHY : %dM, RX_PHY : %dM\n",
                    p_event_data->ble_phy_update_event.tx_phy,
                    p_event_data->ble_phy_update_event.rx_phy);
            break;
#ifdef CYW20721B2
        case BTM_BLE_REMOTE_CONNECTION_PARAM_REQ_EVT:
            result = bt_hs_spk_control_btm_event_handler_ble_remote_conn_param_req(
                    p_event_data->ble_rc_connection_param_req.bd_addr,
                    p_event_data->ble_rc_connection_param_req.min_int,
                    p_event_data->ble_rc_connection_param_req.max_int,
                    p_event_data->ble_rc_connection_param_req.latency,
                    p_event_data->ble_rc_connection_param_req.timeout);
            break;
#endif /* CYW20721B2 */
        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
    }
    return result;
}

wiced_bool_t app_bt_connection_status_callback (wiced_bt_device_address_t bdaddr,
        uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle,
        wiced_bt_transport_t transport, uint8_t reason)
{
    wiced_bt_device_address_t lrac_peer_addr;

    /* Check if this connection status change event is for LRAC peer device. */
    if (transport == BT_TRANSPORT_BR_EDR)
    {
        app_lrac_config_peer_addr_get(lrac_peer_addr);

        if (memcmp((void *) bdaddr,
                   (void *) lrac_peer_addr,
                   sizeof(wiced_bt_device_address_t)) == 0)
        {
            app_lrac_connection_status_handler(p_features, is_connected, handle, reason);
            return WICED_FALSE;
        }
    }

    return WICED_TRUE;
}

/*
 *  Prepare extended inquiry response data.  Current version publishes audio sink
 *  services.
 */
void app_bt_eir_write(char *p_name, wiced_bool_t lras_uuid_only)
{
    uint8_t buffer[sizeof(uint8_t) + sizeof(uint8_t) + LEN_UUID_128];
    uint8_t *p = buffer;
    char *p_device_name;
    uint8_t uuid128[LEN_UUID_128] = {WICED_BT_LRAC_UUID128};
    bt_hs_spk_eir_config_t eir = {0};

    memset(p, 0, sizeof(buffer));

    if( p_name != NULL)
    {
        p_device_name = p_name;
    }
    else
    {
        p_device_name = (char*)wiced_bt_cfg_settings.device_name;
    }
    WICED_BT_TRACE("Device Name:%s\n", p_device_name);

    wiced_bt_dev_set_local_name(p_device_name);

    /* Set EIR via the bt_hs_spk_lib. */
    // Write UUID128 in EIR
    UINT8_TO_STREAM(p, 1 + LEN_UUID_128);
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_128BITS_UUID_TYPE);  // EIR type full list of 128 bit service UUIDs
    ARRAY_TO_STREAM(p, uuid128, LEN_UUID_128);

    eir.p_dev_name              = p_device_name;
    eir.default_uuid_included   = lras_uuid_only ? WICED_FALSE : WICED_TRUE;
    eir.app_specific.included   = WICED_TRUE;
    eir.app_specific.p_content  = buffer;
    eir.app_specific.len        = sizeof(uint8_t) + sizeof(uint8_t) + LEN_UUID_128;

    bt_hs_spk_write_eir(&eir);
}
