/*************************************************************************//**
* @file led.h
*
* @brief
*   This file contains the function prototypes to handle LED used to display
*   device state.
*
* @version KitProg3 v2.40
*/
/*
* Related Documents:
*   002-27868 - KITPROG3 V1.2X EROS
*   002-23369 - KITPROG3 IROS
*   002-26377 - KITPROG3 1.1X TEST PLAN
*
*
******************************************************************************
* (c) (2018-2021), Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, associated documentation and materials ("Software") is
* owned by Cypress Semiconductor Corporation or one of its
* affiliates ("Cypress") and is protected by and subject to worldwide
* patent protection (United States and foreign), United States copyright
* laws and international treaty provisions. Therefore, you may use this
* Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software ("EULA"). If
* no EULA applies, then any reproduction, modification, translation,
* compilation, or representation of this Software is prohibited without
* the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO
* WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,
* BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE. Cypress reserves the right to make
* changes to the Software without notice. Cypress does not assume any
* liability arising out of the application or use of the Software or any
* product or circuit described in the Software. Cypress does not authorize
* its products for use in any products where a malfunction or failure
* of the Cypress product may reasonably be expected to result in significant
* property damage, injury or death ("High Risk Product").
* By including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*****************************************************************************/

#if !defined(LED_H)
#define LED_H
/*****************************************************************************
* MACRO Definition
*****************************************************************************/

/* Macro below are intended to be used in host communication to control LEDs
 * from PC application, e.g. PSoC Programmer */
#define GREEN_LED_NUM               (0u)
#define RED_LED_NUM                 (2u)
#define AMBER_LED_NUM               (3u)
#define LED_OFF                     (0u)
#define LED_BLINK                   (1u)
#define LED_ON                      (3u)

/*****************************************************************************
* Type Definition
*****************************************************************************/

typedef enum
{
    LED_ALL_OFF = 0u,

    LED_CMSIS_BULK_READY,
    LED_CMSIS_BULK_PROGRAMMING,
    LED_CMSIS_BULK_SUCCESS,
    LED_CMSIS_BULK_ERROR,

    LED_CMSIS_HID_READY,
    LED_CMSIS_HID_PROGRAMMING,
    LED_CMSIS_HID_SUCCESS,
    LED_CMSIS_HID_ERROR,

    LED_CMSIS_BULK_2_READY,
    LED_CMSIS_BULK_2_PROGRAMMING,
    LED_CMSIS_BULK_2_SUCCESS,
    LED_CMSIS_BULK_2_ERROR,

    LED_NUM_OF_STATES   /* not going to be used directly */
}led_state_enum;

/*****************************************************************************
* Function Prototypes
*****************************************************************************/
void Led_Init(void);
void Led_SetState(led_state_enum state);

#endif /*LED_H*/

/* [] END OF FILE */
