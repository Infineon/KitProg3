/*************************************************************************//**
* @file power.c
*
* @brief
*  This file provides the source code to handle the power control.
*
* @version KitProg3 v2.50
*/
/*
* Related Documents:
*   002-27868 - KITPROG3 V1.2X EROS
*   002-23369 - KITPROG3 IROS
*   002-26377 - KITPROG3 1.1X TEST PLAN
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

#include <stdbool.h>
#include <stdint.h>
#include "power.h"
#include "device.h"
#include "version.h"
#include "bridgesInterface.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/
#define DEFAULT_1_8V                (1800u)

#define GET_ACCURACY(a,b)           (((a) < (b)) ? ((b) - (a)) : ((a) - (b)))

#define DIGPOT_SLAVE_ADDR           (0x28u)
#define DIGPOT_NV_WRITE_DELAY       (30u)
#define DIGPOT_WR_IVR_REG           (0x00u)
#define DIGPOT_CTRL_REG             (0x02u)
#define DIGPOT_WR_MODE              (0x80u)
#define DIGPOT_IVR_MODE             (0x00u)

#define POWER_OFF_TIMEOUT           (0x32u)
#define POWER_ON_TIMEOUT            (0x0Au)

#define AMUX_SEL_REFERNCE           (0u)
#define AMUX_SEL_TARGET_VDD         (1u)
#define REFERENCE_MILLIVOLTS        (1024)
#define VOLTS_SCALE                 (1000)

#define MIN_REQUIRED_SUPPLY         (4000u)
#define VDD_DETECTED_LEVEL          (1800u)

#define POWER_STABILIZE_DELAY       (200u)   /*msec*/

/* lower accuracy required for VDD above this threshold */
#define HI_VOLT_THRESHOLD           (4500u)  /*mV*/
#define HI_VOLT_ACCURACY            (500u)   /*mV*/
#define LO_VOLT_ACCURACY            (90u)    /*mV*/

/*****************************************************************************
* Global Variable Declarations
*****************************************************************************/
static uint16_t requestedVoltage = 0u;

/*****************************************************************************
* Local Function Prototypes
*****************************************************************************/
static void DigPotentiometerRegWrite(uint8_t digPotReg, uint8_t digPotData);
static void DigPotentiometerWriteIvrMode(uint8_t digPotData);
static uint8_t DigPotentiometerRegRead(void);

/******************************************************************************
*  Power_Init
***************************************************************************//**
* Initializes power subsystem.
*
******************************************************************************/
void Power_Init(void)
{
    static bool powerInitialized = false;
    int32_t measuredRef;
    int32_t gain;

    /* select reference voltage on ADC input and perform ADC gain calibration */
    AdcMux_Start();
    AdcMux_Select(AMUX_SEL_REFERNCE);

    ADC_DelSig_Start();
    measuredRef = ADC_DelSig_Read32();

    /* Calculate ADC gain (ADC counts per volt), half of reference added to
    * perform correct rounding after divide */
    gain = (measuredRef * VOLTS_SCALE) + (REFERENCE_MILLIVOLTS / 2);
    gain = gain / REFERENCE_MILLIVOLTS;
    ADC_DelSig_SetGain(gain);

    /* select target VDD on ADC input */
    AdcMux_Select(AMUX_SEL_TARGET_VDD);

    CyDelay(POWER_STABILIZE_DELAY);

    if (!powerInitialized)
    {
        /* check if power is available before wehave turned it on */
        if (Power_GetVoltage() > VDD_DETECTED_LEVEL)
        {
            /* No power control available,
             * or external power allied without our ability to control it */
            MarkPowerControlFaulty();
        }
        else /* do not power kit if external voltage applied */
        {
            /* power all KIT at start by default except miniprog */
            if ( !KitIsMiniProg() )
            {
                CyPins_SetPin(Pin_VoltageEn_0);
            }
        }
        powerInitialized = true;
    }
    /* Start I2C communication for digital potentiometer */
    I2C_POT_Start();

    /* Check if kit has Digital Potentiometer */
    if ((GetKitSupportedVoltages() != 0u))
    {
        /* And if it will ACK it's address */
        if (I2C_POT_MasterSendStart(DIGPOT_SLAVE_ADDR, 0u) == I2C_POT_MSTR_NO_ERROR)
        {
            (void)I2C_POT_MasterSendStop();
        }
        else
        {
            I2C_POT_Stop();
            MarkRegulatorFaulty();
        }
    }
}

/******************************************************************************
*  Power_GetVoltage
***************************************************************************//**
* returns the VDD of the target device
*
******************************************************************************/
uint16_t Power_GetVoltage(void)
{
    int32_t targetValue;
    targetValue = ADC_DelSig_Read32();

    if( targetValue < 0)
    {
        targetValue = 0;
    }

    /* Account for the resistive divider 3/2 in the hardware used to sample
    * the input voltage  (1.5K + 3K) */
    targetValue = targetValue * 3;
    targetValue = targetValue / 2;

    /* Convert ADC counts to mVolt */
    return ((uint16_t)ADC_DelSig_CountsTo_mVolts(targetValue));
}

/******************************************************************************
*  Power_SetRequestedVoltage
***************************************************************************//**
* Set target voltage by control of digital potentiometer.
*
* @param[in]  desiredVoltage    Desired voltage in mV
*
* @return  True if requested voltage set succesfully.
*
******************************************************************************/
bool Power_SetRequestedVoltage(uint16_t desiredVoltage)
{
    uint16_t voltage;
    uint8_t regVal;
    uint16_t prevDifference;
    uint16_t curDifference;

    /* Remember requested voltage in static variable */
    requestedVoltage = desiredVoltage;

    if ( !KitIsMiniProg() )
    {
        /* All kits except MiniProg turns on target by default */
        Pin_VoltageEn_Write(1u);
    }

    /* Enable  WR register write operation */
    DigPotentiometerRegWrite(DIGPOT_CTRL_REG, DIGPOT_WR_MODE);

    /* Measure current voltage */
    voltage = Power_GetVoltage();
    regVal = DigPotentiometerRegRead();
    curDifference = GET_ACCURACY( desiredVoltage, voltage );

    bool rampUp = ( desiredVoltage > voltage );

    /* ramping through the desired voltage */
    bool done = false;
    while (!done)
    {
        prevDifference = curDifference;

        if ( rampUp )
        {
            if (regVal < (uint8_t)UINT8_MAX)
            {
                regVal++;
            }
        }
        else
        {
            if (regVal > 0u)
            {
                regVal--;
            }
        }
        DigPotentiometerRegWrite(DIGPOT_WR_IVR_REG, regVal);
        CyDelay(1);
        voltage = Power_GetVoltage();
        curDifference = GET_ACCURACY( desiredVoltage, voltage );

        if ( rampUp )
        {
            /* stepping stop, the optimal value has just been passed
             * or no possibility to increase */
            done = ( voltage > desiredVoltage ) || ( regVal == (uint8_t)UINT8_MAX );
        }
        else
        {
            /* stepping stop, the optimal value has just been passed
             * or no possibility to decrease */
            done = ( voltage < desiredVoltage ) || (regVal == 0u);
        }
    }

    if (curDifference > prevDifference)
    {
        /* step backward to optimal value */
        if ( rampUp )
        {
            regVal--;
        }
        else
        {
            regVal++;
        }

        DigPotentiometerRegWrite(DIGPOT_WR_IVR_REG, regVal);
        CyDelay(1);
    }

    voltage = Power_GetVoltage();
    curDifference = GET_ACCURACY( desiredVoltage, voltage );

    /* ACK and write data into DigPot in IVR mode */
    DigPotentiometerWriteIvrMode(regVal);
    DigPotentiometerRegWrite(DIGPOT_CTRL_REG, DIGPOT_IVR_MODE);

    /* Clear I2C DigPot buffers */
    I2C_POT_MasterClearReadBuf();
    I2C_POT_MasterClearWriteBuf();

    return ( (desiredVoltage > HI_VOLT_THRESHOLD) ? (curDifference < HI_VOLT_ACCURACY)
                                                : (curDifference < LO_VOLT_ACCURACY) );
}
/******************************************************************************
*  Power_GetRequestedVoltage
***************************************************************************//**
* Returns last reqested target voltage.
*
* @return Latest requested voltage.
*
******************************************************************************/
uint16_t Power_GetRequestedVoltage(void)
{
  return requestedVoltage;
}

/******************************************************************************
*  DigPotentiometerWriteIvrMode
***************************************************************************//**
* Write configuration data into DigPot in IVR mode.
*
* @param[in]    digPotData      Configuration data for IVR register

******************************************************************************/
static void DigPotentiometerWriteIvrMode(uint8_t digPotData)
{
    /* Enable IVR register write operation  */
    DigPotentiometerRegWrite(DIGPOT_CTRL_REG, DIGPOT_IVR_MODE);

    /* Write voltage value into IVR and WR DigPot registers*/
    DigPotentiometerRegWrite(DIGPOT_WR_IVR_REG, digPotData);

    /* Wait for internal No-Volatile Write Cycle Time*/
    CyDelay(DIGPOT_NV_WRITE_DELAY);
}

/******************************************************************************
*  DigPotentiometerRegWrite
***************************************************************************//**
* Write configuration data into DigPot Ctrl and data registers.
*
* @param[in]    digPotReg       Internal register number
* @param[in]    digPotData      Configuration data for internal registers

******************************************************************************/
static void DigPotentiometerRegWrite(uint8_t digPotReg, uint8_t digPotData)
{
    /* Write data into DigPot  registers */
    (void)I2C_POT_MasterSendStart(DIGPOT_SLAVE_ADDR, 0u);
    (void)I2C_POT_MasterWriteByte(digPotReg);
    (void)I2C_POT_MasterWriteByte(digPotData);
    (void)I2C_POT_MasterSendStop();
}

/******************************************************************************
*  DigPotentiometerRegRead
***************************************************************************//**
* Read configuration data from DigPot data register.
*
* @return  Configuration register data
*
******************************************************************************/
static uint8_t DigPotentiometerRegRead(void)
{
    uint8_t regData;

    /* Read data from DigPot  registers */
    (void)I2C_POT_MasterSendStart(DIGPOT_SLAVE_ADDR, 0u);
    (void)I2C_POT_MasterWriteByte(DIGPOT_WR_IVR_REG);
    (void)I2C_POT_MasterSendRestart(DIGPOT_SLAVE_ADDR, 1u);
    regData = I2C_POT_MasterReadByte(I2C_POT_NAK_DATA);
    (void)I2C_POT_MasterSendStop();

    return regData;
}

/* [] END OF FILE */
