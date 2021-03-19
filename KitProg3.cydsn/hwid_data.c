/*************************************************************************//**
* @file hwid_data.c
*
* @brief
*  This file contains data that encodes the capabilities of the kit based on
*  the kit hardware identifier (HWID).
*
* File Version : 1.0.43
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

#include "version.h"

/**structure that defines kit capabilities based on kit hardware ID (HWID) */
const kitprog_properties_t kitprogConfiguration[MAX_SUPPORTED_HWID+1] = 
{
    [1u] = /* HWID is 0x01u */
    {
        .kitHasVoltMeasure = true,
        /* some kit with ID=1 have power control, others not 
         * can be updated during power system initialization */
        .kitHasPowerControl = true,
        .kitHasThreeLeds = true,
        .kitHasTwoButtons = false,
        .kitHasSecondaryUart = false,
        .kitHasUartHwFlowControl = false,
        .kitHasSpecialRts = false,
        .kitHasI2cBridge = true,
        .kitHasSpiBridge = false,
        .kitHasGpioBridge = false,
        .kitIsMiniProg = false,
        .kitHasPowerCycleProg = false,
        .kitSupportedVoltages = V1_8_SUPPORT_MASK | V2_5_SUPPORT_MASK | V3_3_SUPPORT_MASK | V5_0_SUPPORT_MASK
    },
    [2u] = /* HWID is 0x02u */
    {
        .kitHasVoltMeasure = true,
        .kitHasPowerControl = true,
        .kitHasThreeLeds = true,
        .kitHasTwoButtons = true,
        .kitHasSecondaryUart = false,
        .kitHasUartHwFlowControl = true,
        .kitHasSpecialRts = false,
        .kitHasI2cBridge = true,
        .kitHasSpiBridge = true,
        .kitHasGpioBridge = false,
        .kitIsMiniProg = false,
        .kitHasPowerCycleProg = true,
        .kitSupportedVoltages = V1_8_SUPPORT_MASK | V2_5_SUPPORT_MASK | V3_3_SUPPORT_MASK
    },
    [3u] = /* HWID is 0x03u */
    {
        .kitHasVoltMeasure = true,
        .kitHasPowerControl = false,
        .kitHasThreeLeds = false,
        .kitHasTwoButtons = false,
        .kitHasSecondaryUart = false,
        .kitHasUartHwFlowControl = false,
        .kitHasSpecialRts = false,
        .kitHasI2cBridge = true,
        .kitHasSpiBridge = false,
        .kitHasGpioBridge = false,
        .kitIsMiniProg = false,
        .kitHasPowerCycleProg = false,
        .kitSupportedVoltages = V_NO_SUPPORT_MASK
    },
    [5u] = /* HWID is 0x05u */
    {
        .kitHasVoltMeasure = true,
        .kitHasPowerControl = true,
        .kitHasThreeLeds = true,
        .kitHasTwoButtons = true,
        .kitHasSecondaryUart = false,
        .kitHasUartHwFlowControl = true,
        .kitHasSpecialRts = false,
        .kitHasI2cBridge = true,
        .kitHasSpiBridge = true,
        .kitHasGpioBridge = false,
        .kitIsMiniProg = true,
        .kitHasPowerCycleProg = true,
        .kitSupportedVoltages = V1_8_SUPPORT_MASK | V2_5_SUPPORT_MASK | V3_3_SUPPORT_MASK | V5_0_SUPPORT_MASK
    },
    [6u] = /* HWID is 0x06u */
    {
        .kitHasVoltMeasure = true,
        .kitHasPowerControl = true,
        .kitHasThreeLeds = false,
        .kitHasTwoButtons = false,
        .kitHasSecondaryUart = false,
        .kitHasUartHwFlowControl = false,
        .kitHasSpecialRts = false,
        .kitHasI2cBridge = true,
        .kitHasSpiBridge = false,
        .kitHasGpioBridge = false,
        .kitIsMiniProg = false,
        .kitHasPowerCycleProg = true,
        .kitSupportedVoltages = V_NO_SUPPORT_MASK
    },
    [7u] = /* HWID is 0x07u */
    {
        .kitHasVoltMeasure = true,
        .kitHasPowerControl = true,
        .kitHasThreeLeds = true,
        .kitHasTwoButtons = true,
        .kitHasSecondaryUart = true,
        .kitHasUartHwFlowControl = true,
        .kitHasSpecialRts = false,
        .kitHasI2cBridge = true,
        .kitHasSpiBridge = true,
        .kitHasGpioBridge = false,
        .kitIsMiniProg = false,
        .kitHasPowerCycleProg = true,
        .kitSupportedVoltages = V1_8_SUPPORT_MASK | V2_5_SUPPORT_MASK | V3_3_SUPPORT_MASK
    },
    [8u] = /* HWID is 0x08u */
    {
        .kitHasVoltMeasure = true,
        .kitHasPowerControl = false,
        .kitHasThreeLeds = false,
        .kitHasTwoButtons = false,
        .kitHasSecondaryUart = false,
        .kitHasUartHwFlowControl = false,
        .kitHasSpecialRts = false,
        .kitHasI2cBridge = false,
        .kitHasSpiBridge = false,
        .kitHasGpioBridge = false,
        .kitIsMiniProg = false,
        .kitHasPowerCycleProg = false,
        .kitSupportedVoltages = V_NO_SUPPORT_MASK
    },
    [10u] = /* HWID is 0x0Au */
    {
        .kitHasVoltMeasure = true,
        .kitHasPowerControl = true,
        .kitHasThreeLeds = true,
        .kitHasTwoButtons = true,
        .kitHasSecondaryUart = false,
        .kitHasUartHwFlowControl = true,
        .kitHasSpecialRts = false,
        .kitHasI2cBridge = true,
        .kitHasSpiBridge = true,
        .kitHasGpioBridge = false,
        .kitIsMiniProg = true,
        .kitHasPowerCycleProg = true,
        .kitSupportedVoltages = V1_8_SUPPORT_MASK | V2_5_SUPPORT_MASK | V3_3_SUPPORT_MASK | V5_0_SUPPORT_MASK
    },
    [11u] = /* HWID is 0x0Bu */
    {
        .kitHasVoltMeasure = true,
        .kitHasPowerControl = false,
        .kitHasThreeLeds = false,
        .kitHasTwoButtons = false,
        .kitHasSecondaryUart = false,
        .kitHasUartHwFlowControl = true,
        .kitHasSpecialRts = false,
        .kitHasI2cBridge = true,
        .kitHasSpiBridge = false,
        .kitHasGpioBridge = false,
        .kitIsMiniProg = false,
        .kitHasPowerCycleProg = false,
        .kitSupportedVoltages = V_NO_SUPPORT_MASK
    },
    [12u] = /* HWID is 0x0Cu */
    {
        .kitHasVoltMeasure = true,
        .kitHasPowerControl = true,
        .kitHasThreeLeds = false,
        .kitHasTwoButtons = false,
        .kitHasSecondaryUart = true,
        .kitHasUartHwFlowControl = true,
        .kitHasSpecialRts = true,
        .kitHasI2cBridge = true,
        .kitHasSpiBridge = true,
        .kitHasGpioBridge = false,
        .kitIsMiniProg = false,
        .kitHasPowerCycleProg = true,
        .kitSupportedVoltages = V_NO_SUPPORT_MASK
    },
    [13u] = /* HWID is 0x0Du */
    {
        .kitHasVoltMeasure = true,
        .kitHasPowerControl = true,
        .kitHasThreeLeds = false,
        .kitHasTwoButtons = false,
        .kitHasSecondaryUart = true,
        .kitHasUartHwFlowControl = true,
        .kitHasSpecialRts = false,
        .kitHasI2cBridge = true,
        .kitHasSpiBridge = false,
        .kitHasGpioBridge = true,
        .kitIsMiniProg = false,
        .kitHasPowerCycleProg = true,
        .kitSupportedVoltages = V_NO_SUPPORT_MASK
    },
    [14u] = /* HWID is 0x0Eu */
    {
        .kitHasVoltMeasure = true,
        .kitHasPowerControl = true,
        .kitHasThreeLeds = false,
        .kitHasTwoButtons = false,
        .kitHasSecondaryUart = false,
        .kitHasUartHwFlowControl = true,
        .kitHasSpecialRts = false,
        .kitHasI2cBridge = true,
        .kitHasSpiBridge = false,
        .kitHasGpioBridge = false,
        .kitIsMiniProg = false,
        .kitHasPowerCycleProg = false,
        .kitSupportedVoltages = V_NO_SUPPORT_MASK
    }
};

/* [] END OF FILE */
