/******************************************************************************
* File Name: customservice_server.h
*
* Description: This file contains macros, and function prototypes used in
*              customservice_server.h file.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/

#ifndef CUSTOMSERVICE_SERVER_H_
#define CUSTOMSERVICE_SERVER_H_

/*******************************************************************************
*        Header Files
*******************************************************************************/
#include "wiced_bt_dev.h"
#include <FreeRTOS.h>
#include <task.h>
#include "timers.h"

/*******************************************************************************
 * Extern/ Global variables
 ******************************************************************************/
extern TaskHandle_t  button_task_handle;

/******************************************************************************
 *                           Constants and Enumerations
 ******************************************************************************/
#define STRING_BUFFER_SIZE                  (80)

/* Macros for button interrupt and button task */
/* Interrupt priority for the GPIO connected to the user button */
#define BUTTON_INTERRUPT_PRIORITY       (7u)
#define BUTTON_TASK_PRIORITY            (configMAX_PRIORITIES - 1)
#define BUTTON_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE * 2)

/* The input message size (inclusive of the string terminating character '\0').
 * Edit this macro to suit your message size.
 */
#define MAX_MESSAGE_SIZE                     (20u)

/* Size of the message block that can be processed by Crypto hardware for
 * AES encryption.
 */
#define AES128_ENCRYPTION_LENGTH            (uint32_t)(16u)
#define AES128_KEY_LENGTH                   (uint32_t)(16u)

/* Number of bytes per line to be printed on the UART terminal. */
#define BYTES_PER_LINE                       (16u)


/*******************************************************************************
*        Function Prototypes
*******************************************************************************/

/*Button interrupt handler*/
void button_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event);
void button_task(void *pvParameters);

/* Callback function for Bluetooth stack management events */
wiced_bt_dev_status_t app_bt_management_callback (wiced_bt_management_evt_t event,
                                                  wiced_bt_management_evt_data_t *p_event_data);



#endif /* CUSTOMSERVICE_SERVER_H_ */
