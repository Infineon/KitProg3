/*************************************************************************//**
* @file led.c
*
* @brief
*   This file provides the source code to handle LEDs used to display device
*   state.
*
* @version KitProg3 v2.21
*/
/*
* Related Documents:
*   002-27868 - KITPROG3 V1.2X EROS
*   002-23369 - KITPROG3 IROS
*   002-26377 - KITPROG3 1.1X TEST PLAN
*
*
******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation or a
* subsidiary of Cypress Semiconductor Corporation. All rights
* reserved.
*
* This software, associated documentation and materials ("Software") is
* owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide
* patent protection (United States and foreign), United States copyright
* laws and international treaty provisions. Therefore, you may use this
* Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software ("EULA"). If
* no EULA applies, then any reproduction, modification, translation,
* compilation, or representation of this Software is prohibited without the
* express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO
* WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,
* BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE. Cypress reserves the right to make
* changes to the Software without notice. Cypress does not assume any
* liability arising out of the application or use of the Software or any
* product or circuit described in the Software. Cypress does not
* authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability
*****************************************************************************/
#include "led.h"
#include "device.h"
#include "version.h"

#define LED_RED_OFF                   (0x00u) /* x0 in Led_Control_Reg */
#define LED_RED_ON                    (0x01u) /* x0 in Led_Control_Reg */
#define LED_AMBER_BREATHING_1HZ       (0x00u) /* 0x in Led_Control_Reg */
#define LED_AMBER_DISCRETE            (0x02u) /* 1x in Led_Control_Reg */

/* LED Green shift register values (2 bits)*/
#define LED_GREEN_OFF                 (0x00u)
#define LED_GREEN_8HZ                 (0x01u)
#define LED_GREEN_ON                  (0x03u)

/* LED Amber shift register values (32 bits)*/
#define LED_AMBER_2HZ_2PULULSE_1S_OFF (0xFFFFF0F0u)
#define LED_AMBER_8HZ                 (0x55555555u)
#define LED_AMBER_ON                  (0xFFFFFFFFu)
#define LED_AMBER_128MS_1875MS        (0x00000003u)

/*******************************************************************************
* Function Name: Led_Init()
********************************************************************************
* Summary:
* Initialize the LEDs depending on kit HW revision. Disables pins used for green
* and red LEDs in the case kit has only one led.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Led_Init(void)
{
    Clk_Brea1_Enable();
    Clk_Brea2_Enable();
    PWM_16Hz_Start();
    ShiftReg_LED_Green_Start();
    ShiftReg_LED_Amber_Start();
    if (!KitHasThreeLeds())
    {
        /* Set unused Red and Green LEDs pins to Hi-Z to save power */
        CyPins_SetPinDriveMode(LED_Red_0, PIN_DM_ALG_HIZ);
        CyPins_SetPinDriveMode(LED_Green_0, PIN_DM_ALG_HIZ);
    }
    else
    {
        /* Initialize Amber LED 16Hz shift register for permanent on */
        ShiftReg_LED_Amber_WriteRegValue(LED_AMBER_ON);
    }
}

/*******************************************************************************
* Function Name: Led_SetState()
********************************************************************************
* Summary:
* Set LEDs state
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Led_SetState(led_state_enum state)
{
    /* This value encodes the mode of the Amber LED (breathing/discrete) and the Red LED (on/off) on kits with three LEDs */
    static const uint8_t controlValues3Leds[LED_NUM_OF_STATES] = {
        [LED_ALL_OFF] =                  (LED_AMBER_BREATHING_1HZ | LED_RED_OFF),

        [LED_CMSIS_BULK_READY] =         (LED_AMBER_DISCRETE | LED_RED_OFF),
        [LED_CMSIS_BULK_PROGRAMMING] =   (LED_AMBER_DISCRETE | LED_RED_OFF),
        [LED_CMSIS_BULK_SUCCESS] =       (LED_AMBER_DISCRETE | LED_RED_OFF),
        [LED_CMSIS_BULK_ERROR] =         (LED_AMBER_DISCRETE | LED_RED_ON),

        [LED_CMSIS_HID_READY] =          (LED_AMBER_BREATHING_1HZ | LED_RED_OFF),
        [LED_CMSIS_HID_PROGRAMMING] =    (LED_AMBER_BREATHING_1HZ | LED_RED_OFF),
        [LED_CMSIS_HID_SUCCESS] =        (LED_AMBER_BREATHING_1HZ | LED_RED_OFF),
        [LED_CMSIS_HID_ERROR] =          (LED_AMBER_BREATHING_1HZ | LED_RED_ON),

        [LED_CMSIS_BULK_2_READY] =       (LED_AMBER_DISCRETE | LED_RED_OFF),
        [LED_CMSIS_BULK_2_PROGRAMMING] = (LED_AMBER_DISCRETE | LED_RED_OFF),
        [LED_CMSIS_BULK_2_SUCCESS] =     (LED_AMBER_DISCRETE | LED_RED_OFF),
        [LED_CMSIS_BULK_2_ERROR] =       (LED_AMBER_DISCRETE | LED_RED_ON)
    };
    /* This value encodes the mode of the Amber LED (breathing/discrete) on kits with one LED) */
    static const uint8_t controlValues1Led[LED_NUM_OF_STATES] = {
        [LED_ALL_OFF] =                  LED_AMBER_BREATHING_1HZ,

        [LED_CMSIS_BULK_READY] =         LED_AMBER_DISCRETE,
        [LED_CMSIS_BULK_PROGRAMMING] =   LED_AMBER_DISCRETE,
        [LED_CMSIS_BULK_SUCCESS] =       LED_AMBER_DISCRETE,
        [LED_CMSIS_BULK_ERROR] =         LED_AMBER_DISCRETE,

        [LED_CMSIS_HID_READY] =          LED_AMBER_BREATHING_1HZ,
        [LED_CMSIS_HID_PROGRAMMING] =    LED_AMBER_DISCRETE,
        [LED_CMSIS_HID_SUCCESS] =        LED_AMBER_BREATHING_1HZ,
        [LED_CMSIS_HID_ERROR] =          LED_AMBER_DISCRETE,

        [LED_CMSIS_BULK_2_READY] =       LED_AMBER_DISCRETE,
        [LED_CMSIS_BULK_2_PROGRAMMING] = LED_AMBER_DISCRETE,
        [LED_CMSIS_BULK_2_SUCCESS] =     LED_AMBER_DISCRETE,
        [LED_CMSIS_BULK_2_ERROR] =       LED_AMBER_DISCRETE
    };
    /* Green LED shift register values */
    static const uint8_t greenLedSrValues[LED_NUM_OF_STATES] = {
        [LED_ALL_OFF] =                  LED_GREEN_OFF,

        [LED_CMSIS_BULK_READY] =         LED_GREEN_OFF,
        [LED_CMSIS_BULK_PROGRAMMING] =   LED_GREEN_8HZ,
        [LED_CMSIS_BULK_SUCCESS] =       LED_GREEN_ON,
        [LED_CMSIS_BULK_ERROR] =         LED_GREEN_OFF,

        [LED_CMSIS_HID_READY] =          LED_GREEN_OFF,
        [LED_CMSIS_HID_PROGRAMMING] =    LED_GREEN_8HZ,
        [LED_CMSIS_HID_SUCCESS] =        LED_GREEN_ON,
        [LED_CMSIS_HID_ERROR] =          LED_GREEN_OFF,

        [LED_CMSIS_BULK_2_READY] =       LED_GREEN_OFF,
        [LED_CMSIS_BULK_2_PROGRAMMING] = LED_GREEN_8HZ,
        [LED_CMSIS_BULK_2_SUCCESS] =     LED_GREEN_ON,
        [LED_CMSIS_BULK_2_ERROR] =       LED_GREEN_OFF
    };

    /* Amber LED shift register values, 1LED version */
    static const uint32_t amberLedSrValues[LED_NUM_OF_STATES] = {
        [LED_ALL_OFF] =                  LED_AMBER_ON,

        [LED_CMSIS_BULK_READY] =         LED_AMBER_ON,
        [LED_CMSIS_BULK_PROGRAMMING] =   LED_AMBER_8HZ,
        [LED_CMSIS_BULK_SUCCESS] =       LED_AMBER_ON,
        [LED_CMSIS_BULK_ERROR] =         LED_AMBER_128MS_1875MS,

        [LED_CMSIS_HID_READY] =          LED_AMBER_ON,
        [LED_CMSIS_HID_PROGRAMMING] =    LED_AMBER_8HZ,
        [LED_CMSIS_HID_SUCCESS] =        LED_AMBER_ON,
        [LED_CMSIS_HID_ERROR] =          LED_AMBER_128MS_1875MS,

        [LED_CMSIS_BULK_2_READY] =       LED_AMBER_2HZ_2PULULSE_1S_OFF,
        [LED_CMSIS_BULK_2_PROGRAMMING] = LED_AMBER_8HZ,
        [LED_CMSIS_BULK_2_SUCCESS] =     LED_AMBER_2HZ_2PULULSE_1S_OFF,
        [LED_CMSIS_BULK_2_ERROR] =       LED_AMBER_128MS_1875MS
    };
    if (!KitHasThreeLeds())
    {
        ShiftReg_LED_Amber_WriteRegValue(amberLedSrValues[state]);
        LedControlReg_Write(controlValues1Led[state]);
    }
    else
    {
        ShiftReg_LED_Green_WriteRegValue(greenLedSrValues[state]);
        LedControlReg_Write(controlValues3Leds[state]);
    }
}

/* [] END OF FILE */
