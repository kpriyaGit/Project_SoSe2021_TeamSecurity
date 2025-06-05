/******************************************************************************
 * File Name: customservice_server.c
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
#include "app_bt_cfg.h"
#include "wiced_bt_stack.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "cybt_platform_trace.h"
#include "wiced_memory.h"
#include "cyhal.h"
#include "wiced_bt_dev.h"
#include "app_bt_utils.h"
#include "customservice_server.h"
#include "cyabs_rtos.h"
#include "timers.h"
#include "GeneratedSource/cycfg_gatt_db.h"

#include "cy_pdl.h"
#include "cy_utils.h"
#include "cy_crypto_core_aes_v2.h"
#include "cyb0644abzi_s2d44.h"


/*******************************************************************************
 *        Variable Definitions
 *******************************************************************************/
static uint16_t                  bt_connection_id = 0;
static TimerHandle_t             custom_service_app_msec_timer;
TaskHandle_t  button_task_handle;
cy_stc_crypto_aes_state_t aes_state;

/* Variable to hold the encrypted message */
uint8_t encrypted_msg[MAX_MESSAGE_SIZE];

/* Key used for AES encryption*/
uint8_t aes_key[AES128_KEY_LENGTH] = {0xAA, 0xBB, 0xCC, 0xDD,
		0xEE, 0xFF, 0xFF, 0xEE,
		0xDD, 0xCC, 0xBB, 0xAA,
		0xAA, 0xBB, 0xCC, 0xDD,};

/*******************************************************************************
 *        Function Prototypes
 *******************************************************************************/;
 static void                   ble_app_init                   			(void);
 static void                   custom_service_scan_result_cback         (wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );
 static void                   custom_service_send_notification         (void);
 static void                   custom_service_app_msec_timer_cb         (TimerHandle_t timer_handle);
 gatt_db_lookup_table_t*       custom_service_get_attribute             (uint16_t handle);

 /* GATT Event Callback Functions */
 static wiced_bt_gatt_status_t ble_app_write_handler          (wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id);
 //static wiced_bt_gatt_status_t ble_app_read_handler           (wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id);
 static wiced_bt_gatt_status_t ble_app_connect_callback       (wiced_bt_gatt_connection_status_t *p_conn_status);
 static wiced_bt_gatt_status_t ble_app_server_callback        (uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data);
 static wiced_bt_gatt_status_t ble_app_gatt_event_handler     (wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);
 static bool encrypt_message								  (uint8_t* message, uint8_t msg_size);


 /******************************************************************************
  *                          Function Definitions
  ******************************************************************************/

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
	 wiced_result_t result = WICED_BT_SUCCESS;
	 wiced_bt_device_address_t bda = { 0 };

	 switch (event)
	 {
	 case BTM_ENABLED_EVT:
		 /* Bluetooth Controller and Host Stack Enabled */

		 if (WICED_BT_SUCCESS == p_event_data->enabled.status)
		 {
			 /* Bluetooth is enabled */
			 wiced_bt_dev_read_local_addr(bda);
			 printf("\n Local Bluetooth Address: ");
			 print_bd_address(bda);

			 /* Perform application-specific initialization */
			 ble_app_init();
		 }

		 break;

	 case BTM_BLE_SCAN_STATE_CHANGED_EVT:

		 if(p_event_data->ble_scan_state_changed == BTM_BLE_SCAN_TYPE_HIGH_DUTY)
		 {
			 printf("\n Scan State Change: BTM_BLE_SCAN_TYPE_HIGH_DUTY\n");
		 }
		 else if(p_event_data->ble_scan_state_changed == BTM_BLE_SCAN_TYPE_LOW_DUTY)
		 {
			 printf("\n Scan State Change: BTM_BLE_SCAN_TYPE_LOW_DUTY\n");
		 }
		 else if(p_event_data->ble_scan_state_changed == BTM_BLE_SCAN_TYPE_NONE)
		 {
			 printf("\n Scan stopped\n");
		 }
		 else
		 {
			 printf("\n Invalid scan state\n");
		 }
		 break;

	 default:
		 printf("\n Unhandled Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));
		 break;
	 }

	 return result;
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
  *  None
  *
  *************************************************************************************************/
 static void ble_app_init(void)
 {
	 cy_rslt_t              cy_result = CY_RSLT_SUCCESS;
	 //wiced_result_t result;
	 wiced_bt_gatt_status_t status    = WICED_BT_GATT_ERROR;

	 /* Initialize GPIO for button interrupt*/
	     cy_result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
	                                 CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
	     /* GPIO init failed. Stop program execution */
	     if (CY_RSLT_SUCCESS !=  cy_result)
	     {
	         printf("Button GPIO init failed! \n");
	         CY_ASSERT(0);
	     }

	 /* Configure GPIO interrupt. */
	     cyhal_gpio_register_callback(CYBSP_USER_BTN, button_interrupt_handler, NULL);
	     cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
	                             BUTTON_INTERRUPT_PRIORITY, true);


	 /* Initialize the timer for notification with 5000ms timeout period*/
	 custom_service_app_msec_timer = xTimerCreate("custom_service_timer", pdMS_TO_TICKS(5000), pdTRUE,
			 NULL, custom_service_app_msec_timer_cb);


	 /* Timer init failed. Stop program execution */
	 if (NULL == custom_service_app_msec_timer)
	 {
		 printf("\n [Error] : Timer Initialization failed 0x%lX\r\n", cy_result);
		 CY_ASSERT(0);
	 }

	 /* Disable pairing for this application */
	 wiced_bt_set_pairable_mode(WICED_FALSE, 0);

	 /* Register with BT stack to receive GATT callback */
	 status=wiced_bt_gatt_register(ble_app_gatt_event_handler);
	 printf("\n GATT event Handler registration status: %s \n",get_bt_gatt_status_name(status));

	 /* Initialize GATT Database */
	 status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
	 printf("\n GATT database initialization status: %s \n",get_bt_gatt_status_name(status));

	 printf("Press user button to start scanning...\n");

 }

 /***************************************************************************************
  * Function Name: void custom_service_scan_result_cback()
  ****************************************************************************************
  * Summary:
  *   This function is registered as a callback to handle the scan results.
  *   When the desired device is found, it will try to establish connection with
  *   that device.
  *
  * Parameters:
  *   wiced_bt_ble_scan_results_t *p_scan_result: Details of the new device found.
  *   uint8_t                     *p_adv_data      : Advertisement data.
  *
  * Return:
  *   None
  *
  ****************************************************************************************/
 void custom_service_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
 {
	 wiced_result_t         result = WICED_BT_SUCCESS;
	 uint8_t                length = 0u;
	 uint8_t                *p_data = NULL;
	 uint8_t                client_device_name[15] = {'B','L','E',' ','C','T','S',' ','C','l','i','e','n','t','\0'};

	 if (p_scan_result)
	 {
		 p_data = wiced_bt_ble_check_advertising_data(p_adv_data,
				 BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,
				 &length);

		 /* Check if the peer device's name is "BLE Custom Service Client" */
		 if ((length == strlen((const char *)client_device_name)) &&
				 (memcmp(p_data, (uint8_t *)client_device_name, length) == 0))
		 {
			 printf("\nFound the peer device! BD Addr: ");
			 print_bd_address(p_scan_result->remote_bd_addr);

			 /* Device found. Stop scan. */
			 if((result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE,
					 custom_service_scan_result_cback))!= WICED_BT_SUCCESS)
			 {
				 printf("\r\nscan off status %d\n", result);
			 }
			 else
			 {
				 printf("\n Scan completed\n\n");
			 }

			 printf("\n Initiating connection\n");
			 /* Initiate the connection */
			 if(wiced_bt_gatt_le_connect(p_scan_result->remote_bd_addr,
					 p_scan_result->ble_addr_type,
					 BLE_CONN_MODE_HIGH_DUTY,
					 WICED_TRUE)!= WICED_TRUE)
			 {
				 printf("\rwiced_bt_gatt_connect failed\n");
			 }
		 }
		 else
		 {
			 printf("BD Addr: ");
			 print_bd_address(p_scan_result->remote_bd_addr);
			 return;    //Skip - This is not the device we are looking for.
		 }
	 }
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
	 wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
	 wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;

	 /* Call the appropriate callback function based on the GATT event type, and pass the relevant event
	  * parameters to the callback function */
	 switch ( event )
	 {
	 case GATT_CONNECTION_STATUS_EVT:
		 status = ble_app_connect_callback( &p_event_data->connection_status );
		 break;

	 case GATT_ATTRIBUTE_REQUEST_EVT:
		 p_attr_req = &p_event_data->attribute_request;
		 status = ble_app_server_callback( p_attr_req->conn_id, p_attr_req->request_type, &p_attr_req->data );
		 break;

	 default:
		 status = WICED_BT_GATT_SUCCESS;
		 break;
	 }

	 return status;
 }

 /**************************************************************************************************
  * Function Name: ble_app_set_value()
  ***************************************************************************************************
  * Summary:
  *   This function handles writing to the attribute handle in the GATT database using the
  *   data passed from the BT stack. The value to write is stored in a buffer
  *   whose starting address is passed as one of the function parameters
  *
  * Parameters:
  *   uint16_t attr_handle                    : Attribute handle for write operation
  *   uint16_t conn_id                        : Connection ID
  *   uint8_t *p_val                          : Pointer to the buffer that stores the data to be written
  *   uint16_t len                            : Length of data to be written
  *
  * Return:
  *   wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
  *
  **************************************************************************************************/
 static wiced_bt_gatt_status_t ble_app_set_value(uint16_t attr_handle, uint16_t conn_id, uint8_t *p_val, uint16_t len)
 {
	 int i = 0;
	 wiced_bool_t isHandleInTable = WICED_FALSE;
	 wiced_bool_t validLen = WICED_FALSE;
	 wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;
	 gatt_db_lookup_table_t *puAttribute;


	 /* Check for a matching handle entry */
	 for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
	 {
		 if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
		 {
			 /* Detected a matching handle in external lookup table */
			 isHandleInTable = WICED_TRUE;

			 /* Check if the buffer has space to store the data */
			 validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);

			 if (validLen)
			 {
				 /* Value fits within the supplied buffer; copy over the value */
				 app_gatt_db_ext_attr_tbl[i].cur_len = len;
				 memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
				 res = WICED_BT_GATT_SUCCESS;

				 /* Add code for any action required when this attribute is written.
				  * In this case, we update the IAS led based on the IAS alert
				  * level characteristic value */

				 switch ( attr_handle )
				 {
				 case HDLD_CUSTOM_SERVICE_CUSTOM_CHARACTERISTIC_CLIENT_CHAR_CONFIG:
					 if (len!= app_custom_service_custom_characteristic_client_char_config_len)
					 {
						 printf("\n Invalid attribute length\n");
						 return WICED_BT_GATT_INVALID_ATTR_LEN;
					 }

					 app_custom_service_custom_characteristic_client_char_config[0] = p_val[0];
					 app_custom_service_custom_characteristic_client_char_config[1] = p_val[1];

					 /* Update GATT DB */
					 if ((puAttribute = custom_service_get_attribute(attr_handle)) != NULL)
					 {
						 memcpy(puAttribute->p_data, (uint8_t *)app_custom_service_custom_characteristic_client_char_config,
								 puAttribute->max_len);
					 }

					 if(app_custom_service_custom_characteristic_client_char_config[0])
					 {
						 printf("\r\nNotifications enabled\n");
						 if(pdPASS != xTimerStart(custom_service_app_msec_timer, 0))
						 {
							 printf("\n \rMillisecond timer start failed\n");
						 }
					 }
					 else
					 {
						 printf("\n \rNotifications disabled\n");
						 xTimerStop(custom_service_app_msec_timer, 0);
					 }
					 break;

				 }
			 }
			 else
			 {
				 /* Value to write does not meet size constraints */
				 res = WICED_BT_GATT_INVALID_ATTR_LEN;
			 }
			 break;
		 }
	 }

	 if (!isHandleInTable)
	 {
		 /* TODO: Add code to read value for handles not contained within generated lookup table.
		  * This is a custom logic that depends on the application, and is not used in the
		  * current application. If the value for the current handle is successfully written in the
		  * below code snippet, then set the result using:
		  * res = WICED_BT_GATT_SUCCESS; */
		 switch ( attr_handle )
		 {
		 default:
			 /* The write operation was not performed for the indicated handle */
			 printf("\n Write Request to Invalid Handle: 0x%x\n", attr_handle);
			 res = WICED_BT_GATT_WRITE_NOT_PERMIT;
			 break;
		 }
	 }

	 return res;
 }

 /**************************************************************************************************
  * Function Name: ble_app_write_handler()
  ***************************************************************************************************
  * Summary:
  *   This function handles Write Requests received from the client device
  *
  * Parameters:
  *   wiced_bt_gatt_write_t *p_write_req          : Pointer that contains details of Write Request
  *                                                 including the attribute handle
  *   uint16_t conn_id                            : Connection ID
  *
  * Return:
  *  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
  *
  **************************************************************************************************/
 static wiced_bt_gatt_status_t ble_app_write_handler(wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id)
 {
	 wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

	 /* Attempt to perform the Write Request */
	 status = ble_app_set_value(p_write_req->handle, conn_id, p_write_req->p_val, p_write_req->val_len);

	 return status;
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
	 wiced_result_t result;

	 if ( NULL != p_conn_status )
	 {
		 if ( p_conn_status->connected )
		 {
			 /* Device has connected */
			 printf("\n Connected : BD Addr: " );
			 print_bd_address(p_conn_status->bd_addr);
			 printf("\n Connection ID '%d'\n", p_conn_status->conn_id );

			 /* Store the connection ID */
			 bt_connection_id = p_conn_status->conn_id;

		 }
		 else
		 {
			 /* Device has disconnected */
			 printf("\nDisconnected : BD Addr: " );
			 print_bd_address(p_conn_status->bd_addr);
			 printf("\n Connection ID '%d', Reason '%s'\n", p_conn_status->conn_id, get_bt_gatt_disconn_reason_name(p_conn_status->reason) );

			 /* Set the connection id to zero to indicate disconnected state */
			 bt_connection_id = 0;

			 /*restart the scan*/
			 result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, custom_service_scan_result_cback);
			 if(WICED_BT_PENDING != result)
			 {
				 printf("\n Cannot restart scanning. Error: %d \n", result);
			 }
			 else
			 {
				 printf("\r\nScanning.....\n");
			 }

		 }

		 status = WICED_BT_GATT_SUCCESS;
	 }

	 return status;
 }

 /**************************************************************************************************
  * Function Name: ble_app_server_callback()
  ***************************************************************************************************
  * Summary:
  *   This function handles GATT server events from the BT stack.
  *
  * Parameters:
  *   uint16_t conn_id                            : Connection ID
  *   wiced_bt_gatt_request_type_t type           : Type of GATT server event
  *   wiced_bt_gatt_request_data_t *p_data        : Pointer to GATT server event data
  *
  * Return:
  *  wiced_bt_gatt_status_t: See possible status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
  *
  **************************************************************************************************/
 static wiced_bt_gatt_status_t ble_app_server_callback(uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data)
 {
	 wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

	 switch ( type )
	 {
	 case GATTS_REQ_TYPE_READ:
		 /* Attribute read request */
		 //status = ble_app_read_handler( &p_data->read_req, conn_id );
		 break;

	 case GATTS_REQ_TYPE_WRITE:
		 /* Attribute write request */
		 status = ble_app_write_handler( &p_data->write_req, conn_id );
		 break;
	 }

	 return status;
 }

 /*********************************************************************
  * Function Name: static void custom_service_send_notification()
  **********************************************************************
  * Summary:
  *   Send GATT notification every millisecond.
  *
  * Parameters:
  *   None
  *
  * Return:
  *   None
  *
  **********************************************************************/

 static void custom_service_send_notification(void)
 {

	 wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
	 bool encryption_status = FALSE;

	 printf("Notification Data...\n");


	 encryption_status = encrypt_message(app_custom_service_custom_characteristic, app_custom_service_custom_characteristic_len);

	 if(TRUE == encryption_status)
	 {
		 printf("\n Sending the Encrypted message %s \n", encrypted_msg);
		 status = wiced_bt_gatt_send_notification(bt_connection_id,
				 HDLC_CUSTOM_SERVICE_CUSTOM_CHARACTERISTIC_VALUE,
				 strlen(encrypted_msg),
				 encrypted_msg);
	 }

	 if (WICED_BT_GATT_SUCCESS != status)
	 {
		 printf("\n Send notification failed\n");
	 }
	 else
	 {
		 printf("\nNotification sent by server\n");
	 }



 }

 /****************************************************************************
  * Function Name: void custom_service_app_msec_timer_cb()
  *****************************************************************************
  * Summary:
  *   Millisecond timer callback.
  *   Send GATT notifications if enabled by the GATT Client.
  *
  * Parameters:
  *   timer_handle :  Software timers are reference variable.
  *
  * Return:
  *   None
  *
  ****************************************************************************/

 static void custom_service_app_msec_timer_cb(TimerHandle_t timer_handle)
 {
	 /* Send GATT Notification */
	 if ((app_custom_service_custom_characteristic_client_char_config[0]) &&
			 (bt_connection_id !=0))
	 {
		 /* sending GATT notification. */
		 custom_service_send_notification();
	 }
 }

 /*************************************************************************************
  * Function Name: gatt_db_lookup_table_t* custom_service_get_attribute()
  **************************************************************************************
  * Summary: Find attribute description by handle
  *
  * Parameters:
  *   uint16_t handle: Attribute handle
  *
  * Return:
  *   gatt_db_lookup_table_t*: Pointer to BLE GATT attribute handle
  *
  *************************************************************************************/
 gatt_db_lookup_table_t* custom_service_get_attribute(uint16_t handle)
 {
	 for (uint16_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
	 {
		 if (app_gatt_db_ext_attr_tbl[i].handle == handle)
		 {
			 return (&app_gatt_db_ext_attr_tbl[i]);
		 }
	 }

	 printf("\n Attribute not found:%x\n", handle);

	 return NULL;
 }

 /*******************************************************************************
  * Function Name: button_interrupt_handler
  ********************************************************************************
  *
  * Summary:
  *   This interrupt handler enables or disables GATT notifications upon button
  *   press.
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
  * Function Name: button_task
  ********************************************************************************
  *
  * Summary:
  *   This task enables or disables notifications from the server upon button press.
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
     wiced_result_t result;
     for(;;)
     {
         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
         result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE,
        		 custom_service_scan_result_cback);
         if ((WICED_BT_PENDING == result) || (WICED_BT_BUSY == result))
         {
             printf("\r\nScanning.....\n");
         }
         else
         {
             printf("\rError: Starting scan failed. Error code: %d\n", result);
             return;
         }
     }
 }

 /*******************************************************************************
  * Function Name: encrypt_message
  ********************************************************************************
  * Summary: Function used to encrypt the message.
  *
  * Parameters:
  *  char * message - pointer to the message to be encrypted
  *  uint8_t size   - size of message to be encrypted.
  *
  * Return:
  *  void
  *
  *******************************************************************************/
 bool encrypt_message(uint8_t* message, uint8_t msg_size)
 {
	 uint8_t aes_block_count = 0;

	 aes_block_count =  (msg_size % AES128_ENCRYPTION_LENGTH == 0) ?
			 (msg_size / AES128_ENCRYPTION_LENGTH)
			 : (1 + msg_size / AES128_ENCRYPTION_LENGTH);

	 /* Initializes the AES operation by setting key and key length */
	 Cy_Crypto_Core_Aes_Init(CRYPTO, aes_key, CY_CRYPTO_KEY_AES_128, &aes_state);

	 for (int i = 0; i < aes_block_count ; i++)
	 {
		 /* Perform AES ECB Encryption mode of operation */
		 Cy_Crypto_Core_Aes_Ecb(CRYPTO, CY_CRYPTO_ENCRYPT,
				 (encrypted_msg + AES128_ENCRYPTION_LENGTH * i),
				 (message + AES128_ENCRYPTION_LENGTH * i),
				 &aes_state);

		 /* Wait for Crypto Block to be available */
		 Cy_Crypto_Core_WaitForReady(CRYPTO);
	 }

	 Cy_Crypto_Core_Aes_Free(CRYPTO, &aes_state);
	 return TRUE;
 }


 /* [] END OF FILE */

