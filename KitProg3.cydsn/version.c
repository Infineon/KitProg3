/*************************************************************************//**
* @file version.c
*
* @brief
*  This file provides the source code to handle version information.
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

#include <stdint.h>
#include <stdbool.h>
#include "device.h"
#include "build_number.h"
#include "version.h"


#define VER_MAJOR                        (2u)
#define VER_MINOR                        (21u)

/*****************************************************************************
* Data Structure Definition
*****************************************************************************/

typedef struct{
    uint8_t signature[16];
    uint8_t verMajor;
    uint8_t verMinor;
    uint16_t verBuild;
}version_struct_t;

/*****************************************************************************
* Local Variables
*****************************************************************************/

/*
* Signature that is flash located and can be read by Software from the KitProg
* HEX-file.
* to avoid discard by linker it must be volatile
*/
static volatile const version_struct_t VERSION =
{
    "Cypress-KitProg3",
    VER_MAJOR,
    VER_MINOR,
    BUILD_NUMBER
};

/** Hardware Identifier of kit board */
static uint8_t kitProgHwId = 0u;

static bool regulatorFaulty = false;
static bool powerControlFaulty = false;

/*****************************************************************************
* Local Functions
*****************************************************************************/

/*******************************************************************************
* Function Name: ReadHwVersion()
********************************************************************************
* Summary:
* Calculates the hardware version of the KitProg board.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void ReadHwVersion(void)
{
    kitProgHwId = (~Pin_HWVersion_Read()) & Pin_HWVersion_MASK;

    /* These pins no longer used, set them to HiZ analogue to save power */
    Pin_HWVersion_SetDriveMode(Pin_HWVersion_DM_ALG_HIZ);
}


/*****************************************************************************
* Global Functions
*****************************************************************************/

/******************************************************************************
*  KitHasVoltageMeas
***************************************************************************//**
* Check if kit has on-board capability to measure target voltage level.
*
* @return  True if kit has on-board capability to measure voltage
*
******************************************************************************/
bool KitHasVoltageMeas(void)
{
    return kitprogConfiguration[kitProgHwId].kitHasVoltMeasure;
}


/******************************************************************************
*  KitHasPowerControl
***************************************************************************//**
* Check if kit has ability to turn on/off target chip power.
*
* @return  True if kit has on-board digital potentiometer
*
******************************************************************************/
bool KitHasPowerControl(void)
{
    /* if kit has broken of absent power control report no power control supported */
    return ( powerControlFaulty ? false : kitprogConfiguration[kitProgHwId].kitHasPowerControl );
}

/******************************************************************************
*  MarkRegulatorFaulty
***************************************************************************//**
* Mark that possibility to control power output is absent or faulty
*
******************************************************************************/
void MarkPowerControlFaulty(void)
{
   powerControlFaulty = true;
}

/******************************************************************************
*  KitHasThreeLeds
***************************************************************************//**
* Check if current kit has three LEDs to indicate current kit's state.
*
* @return  True if kit has three LEDs, assuming that false means kit with
           single LED indicator.
*
******************************************************************************/
bool KitHasThreeLeds(void)
{
    return kitprogConfiguration[kitProgHwId].kitHasThreeLeds;
}

/******************************************************************************
*  KitHasTwoButtons
***************************************************************************//**
* Check if current kit has two buttons.
*
* @return  True if kit has two buttons, assuming that false means kit with
           single button to switch modes and applications.
*
******************************************************************************/
bool KitHasTwoButtons(void)
{
    return kitprogConfiguration[kitProgHwId].kitHasTwoButtons;
}

/******************************************************************************
*  KitHasSecondaryUart
***************************************************************************//**
* Check if current kit has secondary UART.
*
* @return  True if kit has secondary UART.
*
******************************************************************************/
bool KitHasSecondaryUart(void)
{
    return kitprogConfiguration[kitProgHwId].kitHasSecondaryUart;
}


/******************************************************************************
*  KitHasUartHwFlowControl
***************************************************************************//**
* Check if current kit has hardware flow control for UART.
*
* @return  True if kit supports hardware flow control for UART.
*
******************************************************************************/
bool KitHasUartHwFlowControl(void)
{
    return kitprogConfiguration[kitProgHwId].kitHasUartHwFlowControl;
}

/******************************************************************************
*  KitHasSpecialRtsPowerup
***************************************************************************//**
* Check if current kit has special behavior of RTS signal on power up.
*
* @return  True if kit mast have 200ms RTS stretch on high when turned on.
*
******************************************************************************/
bool KitHasSpecialRts(void)
{
    return kitprogConfiguration[kitProgHwId].kitHasSpecialRts;
}

/******************************************************************************
*  KitHasI2cBridge
***************************************************************************//**
* Check if kit supports I2C bridging.
*
* @return  True if kit support I2C bridging.
*
******************************************************************************/
bool KitHasI2cBridge(void)
{
    return kitprogConfiguration[kitProgHwId].kitHasI2cBridge;
}


/******************************************************************************
*  KitHasSpiBridge
***************************************************************************//**
* Check if kit supports SPI bridging.
*
* @return  True if kit support SPI bridging.
*
******************************************************************************/
bool KitHasSpiBridge(void)
{
    return kitprogConfiguration[kitProgHwId].kitHasSpiBridge;
}

/******************************************************************************
*  KitHasGpioBridge
***************************************************************************//**
* Check if kit supports GPIO bridging.
*
* @return  True if kit support GPIO bridging.
*
******************************************************************************/
bool KitHasGpioBridge(void)
{
    return kitprogConfiguration[kitProgHwId].kitHasGpioBridge;
}

/******************************************************************************
*  KitIsMiniProg
***************************************************************************//**
* Check if kit is standalone programmer (MiniProg4).
*
* @return  True if kit is MiniProg4.
*
******************************************************************************/
bool KitIsMiniProg(void)
{
    return kitprogConfiguration[kitProgHwId].kitIsMiniProg;
}

/******************************************************************************
*  KitIsMiniProg
***************************************************************************//**
* Check if kit is standalone programmer (MiniProg4).
*
* @return  True if kit is MiniProg4.
*
******************************************************************************/
bool KitHasPowerCycleProg(void)
{
    return kitprogConfiguration[kitProgHwId].kitHasPowerCycleProg;
}

/******************************************************************************
*  GetKitSupportedVoltages
***************************************************************************//**
* Get bitmask of kit supported voltages.
*
* @return  Bitmask with supported voltages
*          V1_8_SUPPORT_MASK, V2_5_SUPPORT_MASK,
*          V3_3_SUPPORT_MASK, V5_0_SUPPORT_MASK
*
******************************************************************************/
uint8_t GetKitSupportedVoltages(void)
{

    /* if kit has broken of absent I2C potentiometer report no voltages supported */
    return ( regulatorFaulty ? V_NO_SUPPORT_MASK : kitprogConfiguration[kitProgHwId].kitSupportedVoltages );
}


/******************************************************************************
*  MarkRegulatorFaulty
***************************************************************************//**
* Mark that I2C potentiometer is absent or faulty
*
******************************************************************************/
void MarkRegulatorFaulty(void)
{
    regulatorFaulty = true;
}


/******************************************************************************
*  System_GetHwId
***************************************************************************//**
* Returns hardware ID for the kit.
*
* @return  Hardware ID.
*
******************************************************************************/
uint8_t System_GetHwId(void)
{
    return kitProgHwId;
}


/******************************************************************************
*  System_GetMajorVersion
***************************************************************************//**
* Returns major version of firmware.
*
* @return  Firmware major version.
*
******************************************************************************/
uint8_t System_GetMajorVersion(void)
{
    return VERSION.verMajor;
}


/******************************************************************************
*  System_GetMinorVersion
***************************************************************************//**
* Returns major version of firmware.
*
* @return  Firmware major version.
*
******************************************************************************/
uint8_t System_GetMinorVersion(void)
{
    return VERSION.verMinor;
}


/******************************************************************************
*  System_GetBuildNumber
***************************************************************************//**
* Returns major version of firmware.
*
* @return  Firmware major version.
*
******************************************************************************/
uint16_t System_GetBuildNumber(void)
{
    return VERSION.verBuild;
}

/******************************************************************************
*  System_Init
***************************************************************************//**
* Prepare System Module. Provides initialization of Kit features.
*
******************************************************************************/
void System_Init(void)
{
    ReadHwVersion();
}

/* [] END OF FILE */
