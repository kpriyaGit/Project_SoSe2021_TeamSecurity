/******************************************************************************
* File Name: customservice_client.c
*
* Description: This is the source code for Bluetooth LE Stack functions
*
*
* Related Document: See README.md
*
*******************************************************************************/
/*******************************************************************************
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
/*******************************************************************************
*        Header Files
*******************************************************************************/
#include "wiced_bt_stack.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "cycfg_gatt_db.h"
#include "stdio.h"
#include "wiced_bt_dev.h"
#include "app_bt_utils.h"
#include "customservice_client.h"
#include "cyabs_rtos.h"
#include "timers.h"
#include <string.h>
#include "cybt_platform_trace.h"
#include "wiced_memory.h"
#include "app_bt_cfg.h"
#include <stdlib.h>
#include "cy_pdl.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
/* LED pin assignment for advertising event */
#define ADV_LED_GPIO                    CYBSP_USER_LED1

/* PWM frequency of LED's in Hz when blinking */
#define ADV_LED_PWM_FREQUENCY           (1)

/* Custom Service UUID - used for service discovery */
#define CUSTOM_SERVICE_UUID0               (0x7E48B57B)
#define CUSTOM_SERVICE_UUID1               (0xAD6D3E68)
#define CUSTOM_SERVICE_UUID2               (0xB55D4BAF)
#define CUSTOM_SERVICE_UUID3               (0x185341CF)

/* Custom Service UUID length: 16 bytes = 128 bits*/
#define CUSTOM_SERVICE_UUID_LEN            (16)

/* Current Time characteristic descriptor length - used to subscribe for notification */
#define CCCD_LENGTH                     2
/* Value used to derive handle of CCCD of current time characteristic */
#define GATT_CCCD_HANDLE                (3)

/* The input message size (inclusive of the string terminating character '\0').
 * Edit this macro to suit your message size.
 */
#define MAX_MESSAGE_SIZE                     (100u)
/* Size of the message block that can be processed by Crypto hardware for AES encryption */
#define AES128_ENCRYPTION_LENGTH             (uint32_t)(16u)

#define AES128_KEY_LENGTH                    (uint32_t)(16u)

/* Number of bytes per line to be printed on the UART terminal. */
#define BYTES_PER_LINE                       (16u)

/*******************************************************************************
*        Enumerations and Constants
*******************************************************************************/
/* PWM Duty Cycle of LED's for different states */
enum
{
    LED_ON_DUTY_CYCLE = 0,
    LED_BLINKING_DUTY_CYCLE= 50,
    LED_OFF_DUTY_CYCLE = 100
} led_duty_cycles;

/* This enumeration combines the advertising, connection states from two different
 * callbacks to maintain the status in a single state variable */
typedef enum
{
    APP_BT_ADV_OFF_CONN_OFF,
    APP_BT_ADV_ON_CONN_OFF,
    APP_BT_ADV_OFF_CONN_ON
} app_bt_adv_conn_mode_t;

/*******************************************************************************
*        Structures
*******************************************************************************/
typedef struct
{
    uint8_t  datafield_1;
    uint8_t  datafield_2;
    uint8_t  datafield_3;
    float32  datafield_4;
    uint8_t  datafield_5;
    int8_t   datafield_6;
    int16_t  datafield_7;
}vehicleData_t;

/*******************************************************************************
*        Variable Definitions
*******************************************************************************/
static cyhal_pwm_t                 adv_led_pwm;
static uint16_t                    bt_connection_id = 0;
static app_bt_adv_conn_mode_t      app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;
static vehicleData_t			   vehicleDataNotif;
static uint16_t                    custom_service_handle = 0;
static bool                        custom_service_found = false;
static bool                        button_press_for_adv = true;
static bool                        notify_val = true;
uint8_t decrypted_msg[MAX_MESSAGE_SIZE];

cy_stc_crypto_aes_state_t aes_state;

/* Key used for AES encryption*/
uint8_t aes_key[AES128_KEY_LENGTH] = {0xAA, 0xBB, 0xCC, 0xDD,
                                      0xEE, 0xFF, 0xFF, 0xEE,
                                      0xDD, 0xCC, 0xBB, 0xAA,
                                      0xAA, 0xBB, 0xCC, 0xDD,};
/*******************************************************************************
*        Function Prototypes
*******************************************************************************/
static void                   adv_led_update                 (void);
static void                   ble_app_init                   (void);
static void                   ble_app_set_advertisement_data (void);
static wiced_bt_gatt_status_t enable_gatt_notification       (bool notify);
static void                   print_notification_data        (wiced_bt_gatt_data_t notif_data);
static void 				  button_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event);

/* GATT Event Callback Functions */
static wiced_bt_gatt_status_t ble_app_connect_callback       (wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t ble_app_gatt_event_handler     (wiced_bt_gatt_evt_t  event, wiced_bt_gatt_event_data_t *p_event_data);

/* Functions for decrypting the message */
void decrypt_message(uint8_t* message, uint8_t size);

/**************************************************************************************************
* Function Name: app_bt_management_callback()
***************************************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management events from
*   the BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*************************************************************************************************/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth Controller and Host Stack Enabled */

            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                /* Bluetooth is enabled */
                wiced_bt_dev_read_local_addr(bda);
                printf("Local Bluetooth Address: ");
                print_bd_address(bda);

                /* Perform application-specific initialization */
                ble_app_init();
            }
            else
            {
            	printf( "Bluetooth Disabled \n");
            }

            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

            /* Advertisement State Changed */
            p_adv_mode = &p_event_data->ble_advert_state_changed;
            printf("Advertisement State Change: %s\n", get_bt_advert_mode_name(*p_adv_mode));

            if (BTM_BLE_ADVERT_OFF == *p_adv_mode)
            {
                /* Advertisement Stopped */
                printf("Advertisement stopped\n");

                /* Check connection status after advertisement stops */
                if(bt_connection_id == 0)
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;
                }
                else
                {
                    app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
                }
            }
            else
            {
                /* Advertisement Started */
                printf("Advertisement started\n");
                app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
            }

            /* Update Advertisement LED to reflect the updated state */
            adv_led_update();
            break;

        default:
            printf("Unhandled Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));
            break;
    }

    return status;
}

/**************************************************************************************************
* Function Name: ble_app_init()
***************************************************************************************************
* Summary:
*   This function handles application level initialization tasks and is called from the BT
*   management callback once the BLE stack enabled event (BTM_ENABLED_EVT) is triggered
*   This function is executed in the BTM_ENABLED_EVT management callback.
*
* Parameters:
*   None
*
* Return:
*   None
*
*************************************************************************************************/
static void ble_app_init(void)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    printf("\n***********************************************\n");
    printf("**Discover device with \"BLE Custom Service Client\" name*\n");
    printf("***********************************************\n\n");

    /* Initialize the PWM used for Advertising LED */
    rslt = cyhal_pwm_init(&adv_led_pwm, ADV_LED_GPIO , NULL);

    /* PWM init failed. Stop program execution */
    if (CY_RSLT_SUCCESS !=  rslt)
    {
        printf("Advertisement LED PWM Initialization has failed! \n");
        CY_ASSERT(0);
    }
	
    /* Initialize GPIO for button interrupt*/
    rslt = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                                CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    /* GPIO init failed. Stop program execution */
    if (CY_RSLT_SUCCESS !=  rslt)
    {
        printf("Button GPIO init failed! \n");
        CY_ASSERT(0);
    }

    /* Configure GPIO interrupt. */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, button_interrupt_handler, NULL);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            BUTTON_INTERRUPT_PRIORITY, true);

    /* Disable pairing for this application */
    wiced_bt_set_pairable_mode(WICED_FALSE, 0);

    /* Set Advertisement Data */
    ble_app_set_advertisement_data();

    /* Register with BT stack to receive GATT callback */
    status = wiced_bt_gatt_register(ble_app_gatt_event_handler);
    printf("GATT event Handler registration status: %s \n", get_bt_gatt_status_name(status));

    /* Initialize GATT Database */
    status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    printf("GATT database initialization status: %s \n", get_bt_gatt_status_name(status));

    /* Start Undirected LE Advertisements after userbutton Press
     * The corresponding parameters are contained in 'app_bt_cfg.c' */

    printf("Press User button to start advertising.....\n");

}

/**************************************************************************************************
* Function Name: ble_app_gatt_event_handler()
***************************************************************************************************
* Summary:
*   This function handles GATT events from the BT stack.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                   : BLE GATT event code of one byte length
*   wiced_bt_gatt_event_data_t *p_event_data    : Pointer to BLE GATT event structures
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t ble_app_gatt_event_handler(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data)
{
	wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

	uint32_t Temp_UUID1 = p_event_data->discovery_result.discovery_data.group_value.service_type.uu.uuid128[0];
	uint32_t Temp_UUID2 = p_event_data->discovery_result.discovery_data.group_value.service_type.uu.uuid128[1];
	uint32_t Temp_UUID3 = p_event_data->discovery_result.discovery_data.group_value.service_type.uu.uuid128[2];
	uint32_t Temp_UUID4 = p_event_data->discovery_result.discovery_data.group_value.service_type.uu.uuid128[3];


	/* Call the appropriate callback function based on the GATT event type, and
	 * pass the relevant event parameters to the callback function */
	switch ( event )
	{
	case GATT_CONNECTION_STATUS_EVT:
		status = ble_app_connect_callback( &p_event_data->connection_status );
		break;

	case GATT_DISCOVERY_RESULT_EVT:
		/* Check if Custom service UUID is present*/

		if (((CUSTOM_SERVICE_UUID0) == Temp_UUID1) &&
				((CUSTOM_SERVICE_UUID1) == Temp_UUID2) &&
				((CUSTOM_SERVICE_UUID2) == Temp_UUID3) &&
				((CUSTOM_SERVICE_UUID3) == Temp_UUID4))
		{
			/* Copy the Custom service UUID*/
			custom_service_handle = p_event_data->discovery_result.discovery_data.group_value.s_handle;
			custom_service_found = true;
		}
		break;

	case GATT_DISCOVERY_CPLT_EVT:
		if(custom_service_found)
		{
			printf("Custom service found\n");
			if(WICED_BT_GATT_SUCCESS != enable_gatt_notification(notify_val))
			{
				printf("Enable notification failed!\n");
			}
		}
		else
		{
			printf("Custom service not found\n");
		}
		break;

	case GATT_OPERATION_CPLT_EVT:
		switch (p_event_data->operation_complete.op)
		{
		case GATTC_OPTYPE_WRITE:
			/* Check if GATT operation of enable/disable notification is success. */
			if ((p_event_data->operation_complete.response_data.handle == (custom_service_handle + GATT_CCCD_HANDLE))
					&& (WICED_BT_GATT_SUCCESS == p_event_data->operation_complete.status))
			{
				if(notify_val)
				{
					printf("Notifications enabled\n");
				}
				else
				{
					printf("Notifications disabled\n");
				}
			}
			else
			{
				printf("CCCD update failed. Error: %d\n", p_event_data->operation_complete.status);
			}
			break;

		case GATTC_OPTYPE_NOTIFICATION:
			/* Function call to print the time and date notification */

			print_notification_data(p_event_data->operation_complete.response_data.att_value);

			break;
		}
		break;

		default:
			status = WICED_BT_GATT_SUCCESS;
			break;
	}

	return status;
}

/**************************************************************************************************
* Function Name: ble_app_set_advertisement_data()
***************************************************************************************************
* Summary:
*   This function configures the advertisement packet data
*
**************************************************************************************************/
static void ble_app_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3] = { 0 };
    uint8_t adv_flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t adv_appearance[] = { BIT16_TO_8( APPEARANCE_GENERIC_CLOCK ) };
    uint8_t num_elem = 0;

    /* Advertisement Element for Flags */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len = sizeof(uint8_t);
    adv_elem[num_elem].p_data = &adv_flag;
    num_elem++;

    /* Advertisement Element for Name */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len = app_gap_device_name_len;
    adv_elem[num_elem].p_data = app_gap_device_name;
    num_elem++;

    /* Advertisement Element for Appearance */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    adv_elem[num_elem].len = sizeof(adv_appearance);
    adv_elem[num_elem].p_data = adv_appearance;
    num_elem++;

    /* Set Raw Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/**************************************************************************************************
* Function Name: ble_app_connect_callback()
***************************************************************************************************
* Summary:
*   This callback function handles connection status changes.
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_conn_status  : Pointer to data that has connection details
*
* Return:
*  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
*
**************************************************************************************************/
static wiced_bt_gatt_status_t ble_app_connect_callback(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_discovery_param_t gatt_discovery_setup = {0};

    if ( NULL != p_conn_status )
    {
        if ( p_conn_status->connected )
        {
            /* Device has connected */
            printf("Connected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d'\n", p_conn_status->conn_id );

            /* Store the connection ID */
            bt_connection_id = p_conn_status->conn_id;

            /* After connection, successive button presses must enable/disable
            notification from server */
            button_press_for_adv = false;

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;

            /* Send GATT service discovery request */
            gatt_discovery_setup.s_handle = 1;
            gatt_discovery_setup.e_handle = 0xFFFF;
            gatt_discovery_setup.uuid.len = CUSTOM_SERVICE_UUID_LEN;
            gatt_discovery_setup.uuid.uu.uuid128[0] = CUSTOM_SERVICE_UUID0;
            gatt_discovery_setup.uuid.uu.uuid128[1] = CUSTOM_SERVICE_UUID1;
            gatt_discovery_setup.uuid.uu.uuid128[2] = CUSTOM_SERVICE_UUID2;
            gatt_discovery_setup.uuid.uu.uuid128[3] = CUSTOM_SERVICE_UUID3;

            if(WICED_BT_GATT_SUCCESS != (status = wiced_bt_gatt_send_discover(
                                                 bt_connection_id,
                                                 GATT_DISCOVER_SERVICES_BY_UUID,
                                                 &gatt_discovery_setup)))
            {
                printf("GATT Discovery request failed. Error code: %d, Conn id: %d\n", status, bt_connection_id);
            }
            else
            {
            	printf("Service Discovery started\n");
            }

        }
        else
        {
            /* Device has disconnected */
            printf("Disconnected : BDA " );
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d', Reason '%s'\n", p_conn_status->conn_id, get_bt_gatt_disconn_reason_name(p_conn_status->reason) );

            /* Set the connection id to zero to indicate disconnected state */
            bt_connection_id = 0;

            /* Reset the flags */
            custom_service_found = false;

            /*First button press after the disconnection must start the advertisement */
            button_press_for_adv = true;

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;

        }

        /* Update Advertisement LED to reflect the updated state */
        adv_led_update();

        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/*******************************************************************************
* Function Name: adv_led_update()
********************************************************************************
*
* Summary:
*   This function updates the advertising LED state based on BLE advertising/
*   connection state
*
*******************************************************************************/
static void adv_led_update(void)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;
    cy_rslt_t start_rslt = CY_RSLT_SUCCESS;
    /* Stop the advertising led pwm */
    rslt = cyhal_pwm_stop(&adv_led_pwm);
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("Failed to stop PWM !!\n");
    }

    /* Update LED state based on BLE advertising/connection state.
     * LED OFF for no advertisement/connection, LED blinking for advertisement
     * state, and LED ON for connected state  */
    switch(app_bt_adv_conn_state)
    {
        case APP_BT_ADV_OFF_CONN_OFF:
            rslt = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_OFF_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;

        case APP_BT_ADV_ON_CONN_OFF:
            rslt = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_BLINKING_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;

        case APP_BT_ADV_OFF_CONN_ON:
            rslt = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_ON_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;

        default:
            /* LED OFF for unexpected states */
            rslt = cyhal_pwm_set_duty_cycle(&adv_led_pwm, LED_OFF_DUTY_CYCLE, ADV_LED_PWM_FREQUENCY);
            break;
    }
    /* Check if update to PWM parameters is successful*/
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("Failed to set duty cycle parameters!!\n");
    }

    /* Start the advertising led pwm */
    start_rslt = cyhal_pwm_start(&adv_led_pwm);

    /* Check if PWM started successfully */
    if (CY_RSLT_SUCCESS != start_rslt)
    {
        printf("Failed to start PWM !!\n");
    }
}

/*****************************************************************************************
* Function Name: enable_gatt_notification()
******************************************************************************************
* Summary:
*   Enable GATT notification from the server.
*
* Parameters:
*   None
*
* Return:
*   wiced_bt_gatt_status_t  : Status code from wiced_bt_gatt_status_e.
*
****************************************************************************************/
static wiced_bt_gatt_status_t enable_gatt_notification(bool notify)
{
    wiced_bt_gatt_value_t      *p_write = NULL;
    wiced_bt_gatt_status_t     status = WICED_BT_GATT_SUCCESS;

    p_write = malloc(sizeof(wiced_bt_gatt_value_t) + CCCD_LENGTH);

    if (p_write)
    {
        memset(p_write, 0, sizeof(wiced_bt_gatt_value_t) + CCCD_LENGTH);
        p_write->handle   = custom_service_handle + GATT_CCCD_HANDLE;
        p_write->len      = CCCD_LENGTH;
        p_write->value[0] = 1; //1 - enable or 0 - disable notification;

        if(TRUE == notify)
        {
        	status = wiced_bt_gatt_send_write(bt_connection_id, GATT_WRITE, p_write);
        }
        else
        {
        	/* disable notifications*/
        	 p_write->value[0] = 0; //1 - enable or 0 - disable notification;
        }
        free(p_write);
    }
    return status;
}

/*****************************************************************************************
* Function Name: print_notification_data()
******************************************************************************************
* Summary:
*   Parses the notification data to get date, time and other fields.
*
* Parameters:
*   wiced_bt_gatt_data_t notif_data: Notification packet from GATT server
*
* Return:
*   None
*
****************************************************************************************/
static void print_notification_data(wiced_bt_gatt_data_t notif_data)
{

	printf("\n\nReceived data (before decryption) is  %s \n", notif_data.p_data);
	printf("\nDecrypting message.....\n");

	decrypt_message(notif_data.p_data, notif_data.len);

	vehicleDataNotif.datafield_1		= decrypted_msg[0];
	vehicleDataNotif.datafield_2		= decrypted_msg[1];
	vehicleDataNotif.datafield_3		= decrypted_msg[2];
	vehicleDataNotif.datafield_4		= (decrypted_msg[6] << 24) |
										  (decrypted_msg[5] << 16) |
										  (decrypted_msg[4] << 8) | decrypted_msg[3];
	vehicleDataNotif.datafield_5		= decrypted_msg[7];
	vehicleDataNotif.datafield_6		= decrypted_msg[8];
	vehicleDataNotif.datafield_7		= (decrypted_msg[10] << 8) | decrypted_msg[9];

    printf("\ndatafield_1 = %d\n", vehicleDataNotif.datafield_1);
    printf("datafield_2 = %d\n", vehicleDataNotif.datafield_2);
    printf("datafield_3 = %d\n", vehicleDataNotif.datafield_3);

    printf("datafield_4 = %0.02f\n", vehicleDataNotif.datafield_4);
    printf("datafield_5 = %d\n", vehicleDataNotif.datafield_5);
    printf("datafield_6 = %d\n", vehicleDataNotif.datafield_6);
    printf("datafield_7 = %d\n", vehicleDataNotif.datafield_7);
}

/*******************************************************************************
* Function Name: button_interrupt_handler()
********************************************************************************
*
* Summary:
*   This interrupt handler notifies the button task of a button press event.
*
* Parameters:
*   void *handler_arg:                     Not used
*   cyhal_gpio_irq_event_t event:          Not used
*
* Return:
*   None
*
*******************************************************************************/
void button_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(button_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*******************************************************************************
* Function Name: button_task()
********************************************************************************
*
* Summary:
*   This task starts Bluetooth LE advertisment on first button press and enables
*   or disables notifications from the server upon successive button presses.
*
* Parameters:
*   void *pvParameters:                Not used
*
* Return:
*   None
*
*******************************************************************************/
void button_task(void *pvParameters)
{
    wiced_result_t wiced_result = WICED_BT_ERROR;
    wiced_bt_gatt_status_t gatt_status;
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(button_press_for_adv)
        {
            wiced_result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                         0, NULL);
            /* Failed to start advertisement, inform user */
            if (WICED_BT_SUCCESS != wiced_result)
            {
                printf("Failed to start advertisement! Error code: %X \n",
                       wiced_result);
            }
        }
        else
        {
        	if((custom_service_found == true) && (bt_connection_id != 0))
            {
                /* Toggle the flag to enable/disable CCCD based on previous state */
                notify_val = !notify_val;

                gatt_status = enable_gatt_notification(notify_val);
                if(WICED_BT_GATT_SUCCESS != gatt_status)
                {
                    printf("Enable/Disable notification failed! Error code: %X \n"
                           ,gatt_status);
                }
            }
        }
    }
}


/*******************************************************************************
* Function Name: decrypt_message
********************************************************************************
* Summary: Function used to decrypt the message.
*
* Parameters:
*  char * message - pointer to the message to be decrypted
*  uint8_t size   - size of message to be decrypted.
*
* Return:
*  void
*
*******************************************************************************/
void decrypt_message(uint8_t* message, uint8_t size)
{
    uint8_t aes_block_count = 0;

    aes_block_count =  (size % AES128_ENCRYPTION_LENGTH == 0) ?
                       (size / AES128_ENCRYPTION_LENGTH)
                       : (1 + size / AES128_ENCRYPTION_LENGTH);

    /* Initializes the AES operation by setting key and key length */
    Cy_Crypto_Core_Aes_Init(CRYPTO, aes_key, CY_CRYPTO_KEY_AES_128, &aes_state);

    /* Start decryption operation*/
    for (int i = 0; i < aes_block_count ; i++)
    {
        /* Perform AES ECB Decryption mode of operation */
        Cy_Crypto_Core_Aes_Ecb(CRYPTO, CY_CRYPTO_DECRYPT,
                               (decrypted_msg + AES128_ENCRYPTION_LENGTH * i),
                               (message + AES128_ENCRYPTION_LENGTH * i),
                               &aes_state);

        /* Wait for Crypto Block to be available */
        Cy_Crypto_Core_WaitForReady(CRYPTO);
    }

    decrypted_msg[size]='\0';

    /* Print the decrypted message on the UART terminal */
    printf("\r\nResult of Decryption : %s \n", decrypted_msg);

    Cy_Crypto_Core_Aes_Free(CRYPTO, &aes_state);
}

/********************************************* [] END OF FILE ************************************************************/
