/*******************************************************************************
* File Name: app_bt_cfg.c
* Version: 1.0
*
* Description: Runtime Bluetooth stack configuration parameters
*
*******************************************************************************
* (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "app_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"

/* Null-Terminated Local Device Name */

/* Device Name */
uint8_t BT_LOCAL_NAME[] = { 'B', 'L', 'E', ' ', 'C', 'T', 'S', ' ', 'C', 'l', 'i', 'e', 'n', 't', '\0' };

/* TODO - Set your app settings as per use case */
/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name                         = (uint8_t *)BT_LOCAL_NAME,                                   /**< Local device name (NULL terminated) */
    .device_class                        = {0x00, 0x00, 0x00},                                         /**< Local device class */
    .security_requirement_mask           = BTM_SEC_NONE,                                               /**< Security requirements mask (BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT (see #wiced_bt_sec_level_e)) */

    .max_simultaneous_links              = 3,                                                          /**< Maximum number simultaneous links to different devices */

    .br_edr_scan_cfg =                                              /* BR/EDR scan config */
    {
        .inquiry_scan_type               = BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .inquiry_scan_interval           = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  (0 to use default) */
        .inquiry_scan_window             = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window (0 to use default) */

        .page_scan_type                  = BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .page_scan_interval              = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  (0 to use default) */
        .page_scan_window                = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window (0 to use default) */
    },

    .ble_scan_cfg =                                                 /* BLE scan settings  */
    {
        .scan_mode                       = BTM_BLE_SCAN_MODE_PASSIVE,                                  /**< BLE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

        /* Advertisement scan configuration */
        .high_duty_scan_interval         = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,               /**< High duty scan interval */
        .high_duty_scan_window           = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,                 /**< High duty scan window */
        .high_duty_scan_duration         = 5,                                                          /**< High duty scan duration in seconds (0 for infinite) */

        .low_duty_scan_interval          = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_INTERVAL,                /**< Low duty scan interval  */
        .low_duty_scan_window            = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW,                  /**< Low duty scan window */
        .low_duty_scan_duration          = 5,                                                          /**< Low duty scan duration in seconds (0 for infinite) */

        /* Connection scan configuration */
        .high_duty_conn_scan_interval    = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL,          /**< High duty cycle connection scan interval */
        .high_duty_conn_scan_window      = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,            /**< High duty cycle connection scan window */
        .high_duty_conn_duration         = 0,                                                          /**< High duty cycle connection duration in seconds (0 for infinite) */

        .low_duty_conn_scan_interval     = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL,           /**< Low duty cycle connection scan interval */
        .low_duty_conn_scan_window       = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,             /**< Low duty cycle connection scan window */
        .low_duty_conn_duration          = 30,                                                         /**< Low duty cycle connection duration in seconds (0 for infinite) */

        /* Connection configuration */
        .conn_min_interval               = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                     /**< Minimum connection interval */
        .conn_max_interval               = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                     /**< Maximum connection interval */
        .conn_latency                    = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection latency */
        .conn_supervision_timeout        = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection link supervision timeout */
    },

    .ble_advert_cfg =                                               /* BLE advertisement settings */
    {
        .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */
                                           BTM_BLE_ADVERT_CHNL_38 |
                                           BTM_BLE_ADVERT_CHNL_39,

        .high_duty_min_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,            /**< High duty undirected connectable minimum advertising interval */
        .high_duty_max_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,            /**< High duty undirected connectable maximum advertising interval */
        .high_duty_duration              = 0,                                                          /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */

        .low_duty_min_interval           = 1024,                                                       /**< Low duty undirected connectable minimum advertising interval */
        .low_duty_max_interval           = 1024,                                                       /**< Low duty undirected connectable maximum advertising interval */
        .low_duty_duration               = 60,                                                         /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */

        .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High duty directed connectable minimum advertising interval */
        .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High duty directed connectable maximum advertising interval */

        .low_duty_directed_min_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low duty directed connectable minimum advertising interval */
        .low_duty_directed_max_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low duty directed connectable maximum advertising interval */
        .low_duty_directed_duration      = 30,                                                         /**< Low duty directed connectable advertising duration in seconds (0 for infinite) */

        .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        .high_duty_nonconn_duration      = 30,                                                         /**< High duty non-connectable advertising duration in seconds (0 for infinite) */

        .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        .low_duty_nonconn_duration       = 0                                                           /**< Low duty non-connectable advertising duration in seconds (0 for infinite) */
    },

    .gatt_cfg =                                                     /* GATT configuration */
    {
        .appearance                     = APPEARANCE_GENERIC_TAG,                                      /**< GATT appearance (see gatt_appearance_e) */
        .client_max_links               = 1,                                                           /**< Client config: maximum number of servers that local client can connect to  */
        .server_max_links               = 1,                                                           /**< Server config: maximum number of remote clients connections allowed by the local */
        .max_attr_len                   = 512,                                                         /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
        .max_mtu_size                   = 517                                                          /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    },

    .rfcomm_cfg =                                                   /* RFCOMM configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of simultaneous connected remote devices*/
        .max_ports                      = 0                                                            /**< Maximum number of simultaneous RFCOMM ports */
    },

    .l2cap_application =                                            /* Application managed l2cap protocol configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        .max_psm                        = 0,                                                           /**< Maximum number of application-managed BR/EDR PSMs */
        .max_channels                   = 0,                                                           /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        .max_le_psm                     = 0,                                                           /**< Maximum number of application-managed LE PSMs */
        .max_le_channels                = 0,                                                           /**< Maximum number of application-managed LE channels */
        /* LE L2cap fixed channel configuration */
        .max_le_l2cap_fixed_channels    = 0,                                                           /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */

        .max_rx_mtu                     = 515
    },

    .avdt_cfg =
    /* Audio/Video Distribution configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum simultaneous audio/video links */
        .max_seps                       = 0                                                            /**< Maximum number of stream end points */
    },

    .avrc_cfg =                                                     /* Audio/Video Remote Control configuration */
    {
        .roles                          = 0,                                                           /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
        .max_links                      = 0                                                            /**< Maximum simultaneous remote control links */
    },

    /* LE Address Resolution DB size  */
    .addr_resolution_db_size            = 5,                                                           /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/

    /* Interval of  random address refreshing */
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,            /**< Interval of  random address refreshing - secs */
    /* BLE white list size */
    .ble_white_list_size                = 0,                                                           /**< Maximum number of white list devices allowed. Cannot be more than 128 */

    .default_ble_power_level            = 12                                                           /**< Default LE power level, Refer lm_TxPwrTable table for the power range */
};

const cybt_platform_config_t bt_platform_cfg_settings =
{
    .hci_config =
    {
        .hci_transport = CYBT_HCI_UART,

        .hci =
        {
            .hci_uart =
            {
                .uart_tx_pin = CYBSP_BT_UART_TX,
                .uart_rx_pin = CYBSP_BT_UART_RX,
                .uart_rts_pin = CYBSP_BT_UART_RTS,
                .uart_cts_pin = CYBSP_BT_UART_CTS,

                .baud_rate_for_fw_download = 115200,
                .baud_rate_for_feature     = 115200,

                .data_bits = 8,
                .stop_bits = 1,
                .parity = CYHAL_UART_PARITY_NONE,
                .flow_control = WICED_TRUE
            }
        }
    },

    .controller_config =
    {
        .bt_power_pin      = CYBSP_BT_POWER,
        .sleep_mode =
        {
            #if (bt_0_power_0_ENABLED == 1) /* BT Power control is enabled in the LPA */
            #if (CYCFG_BT_LP_ENABLED == 1) /* Low power is enabled in the LPA, use the LPA configuration */
            .sleep_mode_enabled = true,
            .device_wakeup_pin = CYCFG_BT_DEV_WAKE_GPIO,
            .host_wakeup_pin = CYCFG_BT_HOST_WAKE_GPIO,
            .device_wake_polarity = CYCFG_BT_DEV_WAKE_POLARITY,
            .host_wake_polarity = CYCFG_BT_HOST_WAKE_IRQ_EVENT
            #else /* Low power is disabled in the LPA, disable low power */
            .sleep_mode_enabled = false
            #endif
            #else /* BT Power control is disabled in the LPA – default to BSP low power configuration */
            .sleep_mode_enabled = true,
            .device_wakeup_pin = CYBSP_BT_DEVICE_WAKE,
            .host_wakeup_pin = CYBSP_BT_HOST_WAKE,
            .device_wake_polarity = CYBT_WAKE_ACTIVE_LOW,
            .host_wake_polarity = CYBT_WAKE_ACTIVE_LOW
            #endif
        }
    },

    .task_mem_pool_size    = 2048
};


/* [] END OF FILE */
