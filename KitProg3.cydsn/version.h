/*************************************************************************//**
* @file version.h
*
* @brief
*  This file contains the function prototypes and constants used in
*  the version.c.
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

#if !defined(VERSION_H)
#define VERSION_H

#include <stdint.h>
#include <stdbool.h>

/*****************************************************************************
* MACRO Definition
*****************************************************************************/

#define MAX_SUPPORTED_HWID               (31u)

#define V_NO_SUPPORT_MASK                (0x00u)
#define V1_8_SUPPORT_MASK                (0x01u)
#define V2_5_SUPPORT_MASK                (0x02u)
#define V3_3_SUPPORT_MASK                (0x04u)
#define V5_0_SUPPORT_MASK                (0x08u)

/*****************************************************************************
* Data Structure Definition
*****************************************************************************/

typedef struct
{
    bool kitHasVoltMeasure;        /**< Kit has Voltage Measurement supported */
    bool kitHasPowerControl;       /**< Kit has ability to turn on and off power for target board */
    bool kitHasThreeLeds;          /**< Kit has three lEDs, assuming in false case kit has single LED */
    bool kitHasTwoButtons;         /**< Kit has two buttons, assuming in false case kit has single button */
    bool kitHasSecondaryUart;      /**< Kit has second UART */
    bool kitHasUartHwFlowControl;  /**< Kit supports hardware flow control for UART */
    bool kitHasSpecialRts;         /**< Kit has 200ms RTS stretch on high when turned on */
    bool kitHasI2cBridge;          /**< Kit supports I2C bridging */
    bool kitHasSpiBridge;          /**< Kit supports SPI bridging */
    bool kitHasGpioBridge;         /**< Kit supports GPIO bridging */
    bool kitIsMiniProg;            /**< Kit is MiniProg, a standalone programmer */
    bool kitHasPowerCycleProg;  /**< Kit can be programmed during power cycle */
    bool kitHasUartIndicator;
    uint8_t kitSupportedVoltages;  /**< Bitmask of supported target voltages */
} kitprog_properties_t;

/*****************************************************************************
* KitProg3 Unique ID Definition
*****************************************************************************/
typedef struct __attribute__((packed))
{
    uint32_t signature;     // Unique ID Header (PSoC5 silicon ID) - 4 bytes
    uint8_t mbedBoardId[4]; // mbed ID - 4 bytes
    uint16_t uid;           // Unique kit ID - 2 bytes
    uint8_t featureList[8]; // Kit feature list - 8 bytes
    uint8_t name[32];       // Full kit name - 32 bytes
    uint8_t hwId;           // Kit HW ID - 1 byte
    uint32_t targetSiid;    // Target Silicon ID - 4 bytes
    uint8_t verMinor;       // Record Version (1 byte for major part,
    uint8_t verMajor;       // 1 byte for minor part) - 2 bytes
    uint8_t reserved[6];    // Reserved for future use - 6 bytes
    uint8_t checksum;       // Record checksum - 1 byte
} unique_id_struct_t;

/*****************************************************************************
* External Variables
*****************************************************************************/

extern const kitprog_properties_t kitprogConfiguration[];

/*****************************************************************************
* External Function Prototypes
*****************************************************************************/

/******************************************************************************
*  System_Init
***************************************************************************//**
* Prepare System Module. Provides initialization of Kit features.
*
******************************************************************************/
void System_Init(void);

/******************************************************************************
*  KitHasVoltageMeas
***************************************************************************//**
* Check if kit has on-board capability to measure target voltage level.
*
* @return  True if kit has on-board capability to measure voltage
*
******************************************************************************/
bool KitHasVoltageMeas(void);


/******************************************************************************
*  KitHasPowerControl
***************************************************************************//**
* Check if kit has ability to turn on/off target chip power.
*
* @return  True if kit has on-board digital potentiometer
*
******************************************************************************/
bool KitHasPowerControl(void);

/******************************************************************************
*  MarkRegulatorFaulty
***************************************************************************//**
* Mark that possibility to control power output is absent or faulty
*
******************************************************************************/
void MarkPowerControlFaulty(void);

/******************************************************************************
*  KitHasThreeLeds
***************************************************************************//**
* Check if current kit has three LEDs to indicate current kit's state.
*
* @return  True if kit has three LEDs, assuming that false means kit with
           single LED indicator.
*
******************************************************************************/
bool KitHasThreeLeds(void);

/******************************************************************************
*  KitHasTwoButtons
***************************************************************************//**
* Check if current kit has two buttons.
*
* @return  True if kit has two buttons, assuming that false means kit with
           single button to switch modes and applications.
*
******************************************************************************/
bool KitHasTwoButtons(void);


/******************************************************************************
*  KitHasSecondaryUart
***************************************************************************//**
* Check if current kit has secondary UART.
*
* @return  True if kit has secondary UART.
*
******************************************************************************/
bool KitHasSecondaryUart(void);


/******************************************************************************
*  KitHasUartHwFlowControl
***************************************************************************//**
* Check if current kit has hardware flow control for UART.
*
* @return  True if kit supports hardware flow control for UART.
*
******************************************************************************/
bool KitHasUartHwFlowControl(void);

/******************************************************************************
*  KitHasSpecialRtsPowerup
***************************************************************************//**
* Check if current kit has special behavior of RTS signal on power up.
*
* @return  True if kit mast have 200ms RTS stretch on high when turned on.
*
******************************************************************************/
bool KitHasSpecialRts(void);

/******************************************************************************
*  KitHasI2cBridge
***************************************************************************//**
* Check if kit supports I2C bridging.
*
* @return  True if kit support I2C bridging.
*
******************************************************************************/
bool KitHasI2cBridge(void);


/******************************************************************************
*  KitHasSpiBridge
***************************************************************************//**
* Check if kit supports SPI bridging.
*
* @return  True if kit support SPI bridging.
*
******************************************************************************/
bool KitHasSpiBridge(void);


/******************************************************************************
*  KitHasGpioBridge
***************************************************************************//**
* Check if kit supports GPIO bridging.
*
* @return  True if kit support GPIO bridging.
*
******************************************************************************/
bool KitHasGpioBridge(void);


/******************************************************************************
*  KitIsMiniProg
***************************************************************************//**
* Check if kit is standalone programmer (MiniProg4).
*
* @return  True if kit is MiniProg4.
*
******************************************************************************/
bool KitIsMiniProg(void);

/******************************************************************************
*  KitIsMiniProg
***************************************************************************//**
* Check if kit can be programmed during power cycle
*
* @return  boolean value
*
******************************************************************************/
bool KitHasPowerCycleProg(void);


/******************************************************************************
*  KitHasUartIndicator
***************************************************************************//**
* Check if current kit supports UART LED indication
*
* @return  True if kit has LED UART indication
*
******************************************************************************/
bool KitHasUartIndicator(void);


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
uint8_t GetKitSupportedVoltages(void);


/******************************************************************************
*  MarkRegulatorFaulty
***************************************************************************//**
* Mark that I2C potentiometer is absent or faulty
*
******************************************************************************/
void MarkRegulatorFaulty(void);


/******************************************************************************
*  System_GetHwId
***************************************************************************//**
* Returns hardware ID for the kit.
*
* @return  Hardware ID.
*
******************************************************************************/
uint8_t System_GetHwId(void);

/******************************************************************************
*  System_GetMajorVersion
***************************************************************************//**
* Returns major version of firmware.
*
* @return  Firmware major version.
*
******************************************************************************/
uint8_t System_GetMajorVersion(void);


/******************************************************************************
*  System_GetMinorVersion
***************************************************************************//**
* Returns major version of firmware.
*
* @return  Firmware major version.
*
******************************************************************************/
uint8_t System_GetMinorVersion(void);


/******************************************************************************
*  System_GetBuildNumber
***************************************************************************//**
* Returns major version of firmware.
*
* @return  Firmware major version.
*
******************************************************************************/
uint16_t System_GetBuildNumber(void);


#endif /* VERSION_H */


/* [] END OF FILE */
