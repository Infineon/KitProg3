/*
 * Copyright (c) 2013-2017 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------
 *
 * $Date:        1. December 2017
 * $Revision:    V2.0.0
 *
 * Project:      CMSIS-DAP Source
 * Title:        DAP_vendor.c CMSIS-DAP Vendor Commands
 *
 *---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------
 * Portions Copyright 2018-2024, Cypress Semiconductor Corporation
 * (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.
 * All rights reserved.
 *
 * This software, associated documentation and materials (“Software”) is
 * owned by Cypress Semiconductor Corporation or one of its
 * affiliates (“Cypress”) and is protected by and subject to worldwide
 * patent protection (United States and foreign), United States copyright
 * laws and international treaty provisions. Therefore, you may use this
 * Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software (“EULA”). If
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
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *
 * THE CYPRESS COPYRIGHTED PORTIONS ARE NOT SUBMISSIONS AS SET FORTH IN THE
 * APACHE LICENSE VERSION 2.0 OR ANY OTHER LICENSE HAVING SIMILAR PROVISIONS.
 *---------------------------------------------------------------------------*/

#include "DAP_config.h"
#include "DAP.h"
#include "kitprog.h"
#include "led.h"
#include "DAP_vendor.h"
#include "bridgesInterface.h"
#include "power.h"
#include "version.h"
#include "app_switch.h"
#include "usbinterface.h"

/* Enumeration for Response bytes of CMSIS-DAP Vendor Command 0 (Get Info) */
enum {
    V0R_CMD_STATUS = 1u,
    V0R_MAJOR_VERSION,
    V0R_BLANK,
    V0R_MINOR_VERSION,
    V0R_BLANK_2,
    V0R_HWID,
    V0R_BLANK_3,
    V0R_PROTOCOL_MAJOR_VERSION,
    V0R_PROTOCOL_MINOR_VERSION,
    V0R_BUILD_LO,
    V0R_BUILD_HI
};

/* Enumeration for Response bytes of CMSIS-DAP Vendor Command 16 (Get Probe Info/Capabilities) */
enum {
    V16R_CMD_STATUS = 1u,
    V16R_SUPPORTED_INTERFACES,
    V16R_UART_LED,
    V16R_I2C_CLOCK,
    V16R_MIN_SPI_BYTE1,
    V16R_MIN_SPI_BYTE2,
    V16R_MIN_SPI_BYTE3,
    V16R_MIN_SPI_BYTE4,
    V16R_MAX_SPI_BYTE1,
    V16R_MAX_SPI_BYTE2,
    V16R_MAX_SPI_BYTE3,
    V16R_MAX_SPI_BYTE4,
    V16R_SPI_SS,
    V16R_VOLTAGES,
};

/* Enumeration for Response bytes of CMSIS-DAP Vendor Command 4 (Get Power) */
enum {
    V4R_CMD_STATUS = 1u,
    V4R_POWER_SUPPLY,
    V4R_VTARG_BYTE1,
    V4R_VTARG_BYTE2,
    V4R_POT_BYTE1,
    V4R_POT_BYTE2,
    V4R_POT_STATUS,
};

/* Enumeration for Response bytes of CMSIS-DAP Vendor Command 4 (Set Power) */
enum {
    V4R_SUBCOMMAND = 1u,
    V4R_POWERMODE,
    V4R_VOLTAGE_BYTE1,
    V4R_VOLTAGE_BYTE2,
};

/* Enumeration for Request of CMSIS-DAP Vendor Command 17 (Set Acquire Parameter) */
enum {
    SET_ACQUIRE_PARAM_TIMEOUT = 0u,
    SET_ACQUIRE_PARAM_DAP_HANDSHAKE,
    SET_ACQUIRE_PARAM_DAP_AP
};

static uint16_t customAcquireTimeout = 0u;

//**************************************************************************************************
/**
\defgroup DAP_Vendor_Adapt_gr Adapt Vendor Commands
\ingroup DAP_Vendor_gr
@{

The file DAP_vendor.c provides template source code for extension of a Debug Unit with
Vendor Commands. Copy this file to the project folder of the Debug Unit and add the
file to the MDK-ARM project under the file group Configuration.
*/

/** Process DAP Vendor Command and prepare Response Data
\param request   pointer to request data
\param response  pointer to response data
\return          number of bytes in response (lower 16 bits)
                 number of bytes in request (upper 16 bits)
*/
uint32_t DAP_ProcessVendorCommand(const uint8_t *request, uint8_t *response) {
    uint32_t num = ID_DAP_DEF_CASE_RESP_LEN;
    response[GENERAL_RESPONSE_COMMAND] = request[GENERAL_REQUEST_COMMAND];        /* copy Command ID */

    switch (request[GENERAL_REQUEST_COMMAND])
    {
        case ID_DAP_Vendor0:
            /* Command 0x80 */
            {
                uint16_t buildNumber = System_GetBuildNumber();
                response[V0R_CMD_STATUS] = CMD_STAT_SUCCESS;
                response[V0R_MAJOR_VERSION] = System_GetMajorVersion();
                response[V0R_BLANK] = 0u;
                response[V0R_MINOR_VERSION] = System_GetMinorVersion();
                response[V0R_BLANK_2] = 0u;
                response[V0R_HWID] = System_GetHwId();
                response[V0R_BLANK_3] = 0u;
                response[V0R_PROTOCOL_MAJOR_VERSION] = (uint8)KHPI_VER_MAJOR;
                response[V0R_PROTOCOL_MINOR_VERSION] = (uint8)KHPI_VER_MINOR;
                response[V0R_BUILD_LO] = LO8(buildNumber);
                response[V0R_BUILD_HI] = HI8(buildNumber);
                num = ((1UL << 16) | ID_DAP_V0_LEN); /* returns one byte in the request (0x80) in the first 16 bits and 12 bytes in the response*/
                break;
            }
        case ID_DAP_Vendor1:
            /* Command 0x81 */
            HandleReset();
            /* No reply feasible to be received by the host */
            break;
        case ID_DAP_Vendor2:
            /* Command 0x82 */
            HandleModeSwitch(request, response);
            num = ((2UL << 16) | CMD_CODE_CMD_STAT_LEN); /* Two bytes in the request (0x82 and mode) */
            /* No reply feasible to be received by the host */
            break;
        case ID_DAP_Vendor3:
            /* Command 0x83 */
            num = ((2UL << 16) | HandleLedCmd(request, response)); /* Two bytes in the request (0x83 and mode) */
            break;
        case ID_DAP_Vendor4:
            /* Command 0x84 */
            num = GetSetPower(request, response);
            break;
        case ID_DAP_Vendor5:
            /* Command 0x85 */
            if ((DAP_Data.debug_port == DAP_PORT_SWD) && (KitSuportsSwd()))
            {
                /* Disconnect SWD pins from DSI */
                Swd_SetPinsDsiConnect(false);
                num = HandleAcquire(request, response);
            }
            else
            {
                /* Move response pointer back to byte 0 */
                response[GENERAL_RESPONSE_COMMAND] = ID_DAP_Invalid;
                num = ID_DAP_DEF_CASE_RESP_LEN;
            }
            break;
        case ID_DAP_Vendor10:
            /* Command 0x8A */
            if (KitHasGpioBridge())
            {
                num = Bridge_GpioSetMode(request, response);
            }
            else
            {
                response[PROTOCOL_CMD_OFFSET] = ID_DAP_Invalid;
                num = ID_DAP_DEF_CASE_RESP_LEN;
            }
            break;
        case ID_DAP_Vendor11:
            /* Command 0x8B */
            if (KitHasGpioBridge())
            {
                num = Bridge_GpioSetState(request, response);
            }
            else
            {
                response[PROTOCOL_CMD_OFFSET] = ID_DAP_Invalid;
                num = ID_DAP_DEF_CASE_RESP_LEN;
            }
            break;
        case ID_DAP_Vendor12:
            /* Command 0x8C */
            if (KitHasGpioBridge())
            {
                num = Bridge_GpioReadState(request, response);
            }
            else
            {
                response[PROTOCOL_CMD_OFFSET] = ID_DAP_Invalid;
                num = ID_DAP_DEF_CASE_RESP_LEN;
            }
            break;
        case ID_DAP_Vendor13:
            /* Command 0x8D */
            if (KitHasGpioBridge())
            {
                num = Bridge_GpioStateChanged(request, response);
            }
            else
            {
                response[PROTOCOL_CMD_OFFSET] = ID_DAP_Invalid;
                num = ID_DAP_DEF_CASE_RESP_LEN;
            }
            break;

        case ID_DAP_Vendor16:
            /* Command 0x90 */
            num = ((1UL << 16) | GetCapabilities(request, response));
            break;
        case ID_DAP_Vendor17:
            /* Command 0x91 */
            num = ((2UL << 16) | SetAcquireOption(request, response));
            break;
        case ID_DAP_Vendor18:
            /* Command 0x92 */
            num = ((1UL << 16) | GetUidData(request, response));
            break;
        case ID_DAP_Vendor19:
            /* Command 0x93 */
            if (KitHasUartHwFlowControl())
            {
                num = ((1UL << 16) | GetSetHwContol(request, response));
            }
            else
            {
                response[PROTOCOL_CMD_OFFSET] = ID_DAP_Invalid;
                num = ID_DAP_DEF_CASE_RESP_LEN;
            }
            break;
        default:
        {
            /* Move response pointer back to byte 0 */
            response[GENERAL_RESPONSE_COMMAND] = ID_DAP_Invalid;
            num = ID_DAP_DEF_CASE_RESP_LEN;
            break;
        }
    }

    return (num);
}


/******************************************************************************
*  handleReset
***************************************************************************//**
* Triggers reset of FW.
*
******************************************************************************/
void HandleReset(void)
{
    CySoftwareReset();
}


/******************************************************************************
*  handleModeSwith
***************************************************************************//**
* Executes an mode switch.
*
* @param[in] request Pointer to incoming USB packet
*
* @param[in] response Pointer to the memory for storing response

*
******************************************************************************/
void HandleModeSwitch(const uint8_t *request, uint8_t *response)
{
    switch (request[GENERAL_REQUEST_SUBCOMMAND])
    {
        case MODE_BOOTLOADER:
        {
            Bootloadable_Load();
            break;
        }
        case MODE_CMSIS_DAP2X:
        {
            currentMode = MODE_BULK;
            /* Write the current mode setting to the EEPROM. */
            if (CySetTemp() == CYRET_SUCCESS)
            {
                (void)EEPROM_ModeStorage_ByteWrite(currentMode, 0u, 0u);
                EEPROM_ModeStorage_Stop();
                /* Provide a 20us delay to ensure that the SPC has completed the write operation. */
                CyDelayUs(20u);
            }
            CySoftwareReset();
            break;
        }
        case MODE_CMSIS_DAP1X:
        {
            currentMode = MODE_HID;
            /* Write the current mode setting to the EEPROM. */
            if (CySetTemp() == CYRET_SUCCESS)
            {
                (void)EEPROM_ModeStorage_ByteWrite(currentMode, 0u, 0u);
                EEPROM_ModeStorage_Stop();
                /* Provide a 20us delay to ensure that the SPC has completed the write operation. */
                CyDelayUs(20u);
            }
            CySoftwareReset();
            break;
        }
        case MODE_CUSTOM_APP:
        {
            if (CySetTemp() == CYRET_SUCCESS)
            {
                /* write BULK mode to EEPROM to avoid loop in HID if DAPLink is damaged */
                (void)EEPROM_ModeStorage_ByteWrite(MODE_BULK, 0u, 0u);
                /* Set 2nd Application Active (Custom Application) */
                (void)Bootloadable_SetActiveApplication(CUSTOM_APP_ID);
                EEPROM_ModeStorage_Stop();
                /* Provide a 20us delay to ensure that the SPC has completed the write operation. */
                CyDelayUs(20u);
            }
            CySoftwareReset();
            break;
        }
        case MODE_CMSIS_DAP2X_DOUBLEUARTS:
        {
            if (KitHasSecondaryUart())
            {
                currentMode = MODE_BULK2UARTS;
                /* Write the current mode setting to the EEPROM. */
                if (CySetTemp() == CYRET_SUCCESS)
                {
                    (void)EEPROM_ModeStorage_ByteWrite(currentMode, 0u, 0u);
                    EEPROM_ModeStorage_Stop();
                    /* Provide a 20us delay to ensure that the SPC has completed the write operation. */
                    CyDelayUs(20u);
                }
                CySoftwareReset();
            }
            else
            {
                response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
            }
            break;
        }

        default:
        {
            response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
            break;
        }
    }
}


/******************************************************************************
*  handleLedCMD
***************************************************************************//**
* Handles LED operation
*
* @param[in] request Pointer to incoming USB packet
*
* @param[in] response Pointer to the memory for storing response
*
******************************************************************************/
uint32_t HandleLedCmd(const uint8_t *request, uint8_t *response)
{
    uint32_t retVal = 0u;

    switch (request[GENERAL_REQUEST_SUBCOMMAND])
    {
        case LED_STATE_READY:
        {
            if (currentMode == MODE_HID)
            {
                Led_SetState(LED_CMSIS_HID_READY);
            }
            else if (currentMode == MODE_BULK2UARTS)
            {
                Led_SetState(LED_CMSIS_BULK_2_READY);
            }
            else
            {
                Led_SetState(LED_CMSIS_BULK_READY);
            }
            response[GENERAL_RESPONSE_STATUS] = CMD_STAT_SUCCESS;
            retVal++;
            break;
        }
        case LED_STATE_PROGRAMMING:
        {
            if (currentMode == MODE_HID)
            {
                Led_SetState(LED_CMSIS_HID_PROGRAMMING);
            }
            else if (currentMode == MODE_BULK2UARTS)
            {
                Led_SetState(LED_CMSIS_BULK_2_PROGRAMMING);
            }
            else
            {
                Led_SetState(LED_CMSIS_BULK_PROGRAMMING);
            }
            response[GENERAL_RESPONSE_STATUS] = CMD_STAT_SUCCESS;
            retVal++;
            break;
        }
        case LED_STATE_SUCCESS:
        {
            if (currentMode == MODE_HID)
            {
                Led_SetState(LED_CMSIS_HID_SUCCESS);
            }
            else if (currentMode == MODE_BULK2UARTS)
            {
                Led_SetState(LED_CMSIS_BULK_2_SUCCESS);
            }
            else
            {
                Led_SetState(LED_CMSIS_BULK_SUCCESS);
            }
            response[GENERAL_RESPONSE_STATUS] = CMD_STAT_SUCCESS;
            retVal++;
            break;
        }
        case LED_STATE_ERROR:
        {
            if (currentMode == MODE_HID)
            {
                Led_SetState(LED_CMSIS_HID_ERROR);
            }
            else if (currentMode == MODE_BULK2UARTS)
            {
                Led_SetState(LED_CMSIS_BULK_2_ERROR);
            }
            else
            {
                Led_SetState(LED_CMSIS_BULK_ERROR);
            }
            response[GENERAL_RESPONSE_STATUS] = CMD_STAT_SUCCESS;
            retVal++;
            break;
        }
        default:
        {
            /* Other modes are not supported */
            response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
            retVal++;
            break;
        }
    }
    retVal++;

    return (retVal);
}


/******************************************************************************
*  HandleAcquire
***************************************************************************//**
* Executes acquire request.
*
* @param[in] request Pointer to incoming USB packet
*
* @param[out] response Pointer to the memory for storing response
*
* @return Result of acquire attempt.
*        0x00 - device was not acquired
*        0x01 - device was acquired
*
******************************************************************************/
uint32_t HandleAcquire(const uint8_t *request, uint8_t *response)
{
    uint32_t retVal = 3u;
    uint8_t resp = ACQUIRE_FAIL;

    /* Get DUT, AcquireMode and number of tries from request */
    uint8_t swdAcquireMode = request[PROTOCOL_MODE_OFSET];
    uint8_t swdAcquireDut = request[PROTOCOL_DUT_OFSET];
    uint8_t number = request[PROTOCOL_N_OFSET];

    /* Set actual timeout and perceived timeout */
    Swd_SetActualAcquireTimeout(swdAcquireDut, swdAcquireMode);
    uint16_t cachedCustomAcquireTimeout = ((customAcquireTimeout == 0u) ? perceivedTimeout : customAcquireTimeout);

    for (uint8_t count = 0u; (count < number) && (resp != ACQUIRE_PASS); count++)
    {
        WaitVendorResponse();
        uint16_t startTimerVal = Timer_CSTick_ReadCounter();
        uint16_t lastWaitRespTimerVal = startTimerVal;

        /* First try with proper power mode */
        resp = (uint8_t)(Swd_Acquire((uint32_t)(swdAcquireDut), (uint32_t)(swdAcquireMode), &request[PROTOCOL_CUSTOM_SEQ_OFSET]));

        if (((uint16_t)(lastWaitRespTimerVal - Timer_CSTick_ReadCounter())) >= ((uint16_t)(TIMER_CSTICK_HALF_RATE)))
        {
            WaitVendorResponse();
            lastWaitRespTimerVal = Timer_CSTick_ReadCounter();
        }

        while (((uint16_t)(startTimerVal - Timer_CSTick_ReadCounter())) <= cachedCustomAcquireTimeout)
        {
            if (resp != ACQUIRE_PASS)
            {
                /* Try to acquire without changing of power settings */
                resp = (uint8_t)(Swd_Acquire((uint32)(swdAcquireDut),(uint32_t)(ACQUIRE_IDLE), &request[PROTOCOL_CUSTOM_SEQ_OFSET]));
            }
            else
            {
                /* ACQUIRE_PASS */
                break;
            }

            /* Send Wait Response to the  host every 0.5 second */
            if (((uint16_t)(lastWaitRespTimerVal - Timer_CSTick_ReadCounter())) >= ((uint16_t)(TIMER_CSTICK_HALF_RATE)))
            {
                WaitVendorResponse();
                lastWaitRespTimerVal = Timer_CSTick_ReadCounter();
            }
        }
    }

    if (resp == ACQUIRE_WAIT)
    {
        resp = ACQUIRE_FAIL;
    }

    response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_SUCCESS;
    response[PROTOCOL_RESP_DATA_OFFSET] = resp;

    /* Leave SDA line as output to allow CMSIS-DAP operation */
    SWD_SET_SDA_OUT;

    return (retVal);
}

/******************************************************************************
*  GetCapabilities
***************************************************************************//**
* Executes get probe info/capabilities command.
*
* @param[in] request Pointer to incoming USB packet
*
* @param[out] response Pointer to the memory for storing response
*
* @return Returns size of response packet.
*
******************************************************************************/
uint32_t GetCapabilities(const uint8_t *request, uint8_t *response)
{
    (void) request;
    uint32_t num = 0u;

    response[V16R_CMD_STATUS] = CMD_STAT_SUCCESS;        
    response[V16R_SUPPORTED_INTERFACES] = (KitHasI2cBridge() ? I2C_AVAILABILITY_MASK : 0x00u) |
                                          (KitHasSpiBridge() ? SPI_AVAILIBILITY_MASK : 0x00u) |
                                           DAPH_AVAILIBILITY_MASK | DAPB_AVAILIBILITY_MASK |
                                          (KitHasPowerControl() ? ON_OFF_SW_AVAILIBILITY_MASK : 0x00u) |
                                          (KitHasVoltageMeas() ? VMEAS_AVAILIBILITY_MASK : 0x00u)|
                                          (KitHasGpioBridge() ? GPIO_AVAILIBILITY_MASK : 0x00u);
    response[V16R_UART_LED] = ((KitHasThreeLeds() ? THREE_LED_KIT_MASK : ONE_LED_KIT_MASK) +
                               (KitHasSecondaryUart() ? TWO_UART_MASK : ONE_UART_MASK));
    response[V16R_I2C_CLOCK] = (GetKitSupportedGpioPins() | I2C_SPEEDS_MASK);
    response[V16R_MIN_SPI_BYTE1] = LO8(LO16(SOURCECLK_IMO/SPI_COMP_DIVIDER/SPI_DIVIDER_MAX));
    response[V16R_MIN_SPI_BYTE2] = HI8(LO16(SOURCECLK_IMO/SPI_COMP_DIVIDER/SPI_DIVIDER_MAX));
    response[V16R_MIN_SPI_BYTE3] = LO8(HI16(SOURCECLK_IMO/SPI_COMP_DIVIDER/SPI_DIVIDER_MAX));
    response[V16R_MIN_SPI_BYTE4] = HI8(HI16(SOURCECLK_IMO/SPI_COMP_DIVIDER/SPI_DIVIDER_MAX));
    response[V16R_MAX_SPI_BYTE1] = LO8(LO16(SOURCECLK_IMO/SPI_COMP_DIVIDER/SPI_DIVIDER_MIN));
    response[V16R_MAX_SPI_BYTE2] = HI8(LO16(SOURCECLK_IMO/SPI_COMP_DIVIDER/SPI_DIVIDER_MIN));
    response[V16R_MAX_SPI_BYTE3] = LO8(HI16(SOURCECLK_IMO/SPI_COMP_DIVIDER/SPI_DIVIDER_MIN));
    response[V16R_MAX_SPI_BYTE4] = HI8(HI16(SOURCECLK_IMO/SPI_COMP_DIVIDER/SPI_DIVIDER_MIN));
    response[V16R_SPI_SS] = GetKitSupportedSpiSs();
    response[V16R_VOLTAGES] = GetKitSupportedVoltages();
    num = PROB_CAP_RESP_LEN;

    return (num);
}

/******************************************************************************
*  GetSetPower
***************************************************************************//**
* Executes get/set Power Vendor command.
*
* @param[in] request Pointer to incoming USB packet
*
* @param[out] response Pointer to the memory for storing response
*
* @return size of response packet.
*
******************************************************************************/
uint32_t GetSetPower(const uint8_t *request, uint8_t *response)
{
    uint32_t num = 0u;

    if (request[GENERAL_REQUEST_SUBCOMMAND] == CMD_POWER_SET)
    {
        /* Set power */
        if (request[V4R_POWERMODE] == CMD_POWER_OFF)
        {
            /* Power off */
            Pin_VoltageEn_Write(CMD_POWER_OFF);
            response[GENERAL_RESPONSE_STATUS] = CMD_STAT_SUCCESS;
            num = CMD_CODE_CMD_STAT_LEN;
        }
        else if (request[V4R_POWERMODE] == CMD_POWER_ON)
        {
            /* Power on */
            /* Measure currently present voltage */
            if ((Power_GetVoltage() >= POWER_1000_MV) && (Pin_VoltageEn_Read() == 0x00u) && KitIsMiniProg())
            {
                response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_OP_FAIL;
                response[GENERAL_RESPONSE_RESULT] = POWER_NOT_ACHIEVED;
                num = CMD_CODE_CMD_STAT_ERR_LEN;
            }
            else
            {
                Pin_VoltageEn_Write(CMD_POWER_ON);

                response[GENERAL_RESPONSE_STATUS] = CMD_STAT_SUCCESS;
                num = CMD_CODE_CMD_STAT_LEN;
            }
        }
        else if (request[V4R_POWERMODE] == CMD_POWER_VOLT_SET)
        {
            /* Configure digital pot. if it is available */
            if (GetKitSupportedVoltages() != 0u)
            {
                /* Measure currently present voltage */
                /* If currently voltage is supplied */
                if ((Power_GetVoltage() >= 1000u) && (Pin_VoltageEn_Read() == 0x00u) && KitIsMiniProg())
                {
                    response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_OP_FAIL;
                    response[GENERAL_RESPONSE_RESULT] = POWER_NOT_ACHIEVED;
                    num = CMD_CODE_CMD_STAT_ERR_LEN;
                }
                else
                {
                    uint16_t reqVoltage;
                    if (Pin_VoltageEn_Read() == 0u)
                    {
                        Pin_VoltageEn_Write(CMD_POWER_ON);
                        /* powering stabilization delay */
                        CyDelay(10u);
                    }
                    else
                    {
                        Pin_VoltageEn_Write(CMD_POWER_ON);
                    }
                    reqVoltage = (((uint16_t)(request[V4R_VOLTAGE_BYTE1]))|(((uint16_t)(request[V4R_VOLTAGE_BYTE2])) << 8u));

                    /* Try to set voltage */
                    if (Power_SetRequestedVoltage(reqVoltage))
                    {
                        response[GENERAL_RESPONSE_STATUS] = CMD_STAT_SUCCESS;
                        num = CMD_CODE_CMD_STAT_LEN;
                    }
                    else
                    {
                        response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_OP_FAIL;
                        response[GENERAL_RESPONSE_RESULT] = CMD_STAT_VOLT_SET_FAILED;
                        num = CMD_CODE_CMD_STAT_ERR_LEN;
                    }
                }
            }
            else
            {
                response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_OP_FAIL;
                response[GENERAL_RESPONSE_RESULT] = ID_DAP_Invalid;
                num = CMD_CODE_CMD_STAT_ERR_LEN;
            }
        }
        else
        {
            response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
            num = CMD_CODE_CMD_STAT_LEN;
        }
    }
    else if (request[GENERAL_REQUEST_SUBCOMMAND] == CMD_POWER_GET)
    {
        /* Get power */
        uint16_t reqVoltage = Power_GetRequestedVoltage();
        uint16_t voltage = Power_GetVoltage();

        response[V4R_CMD_STATUS] = CMD_STAT_SUCCESS;
        response[V4R_POWER_SUPPLY] = Pin_VoltageEn_Read();
        response[V4R_VTARG_BYTE1] = LO8(voltage);
        response[V4R_VTARG_BYTE2] = HI8(voltage);
        response[V4R_POT_BYTE1] = LO8(reqVoltage);
        response[V4R_POT_BYTE2] = HI8(reqVoltage);
        response[V4R_POT_STATUS] = ((GetKitSupportedVoltages() != 0u) ? 1u : 0u);

        num = ID_DAP_V4_LEN;
    }
    else
    {
        /* Wrong argument */
        response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
        num = CMD_CODE_CMD_STAT_LEN;
    }
    return num;
}

/******************************************************************************
*  SetAcquireTimeout
***************************************************************************//**
* Sets Acquire Timeout in seconds
*
* @param[in] request Pointer to incoming USB packet
*
* @param[out] response Pointer to the memory for storing response
*
* @return size of response packet.
*
******************************************************************************/
static uint32_t SetAcquireTimeout(const uint8_t *request, uint8_t *response)
{
    uint32_t retVal = CMD_CODE_CMD_STAT_LEN;
    uint8_t desiredTimeout = request[ID_DAP_V17_VALUE_OFFSET];

    if (desiredTimeout >= MAX_TIMEOUT_IN_SECONDS)
    {
        desiredTimeout = MAX_TIMEOUT_IN_SECONDS;
    }
    customAcquireTimeout = (uint16_t)desiredTimeout * (uint16_t)TIMER_CSTICK_RATE;

    response[GENERAL_RESPONSE_STATUS] = CMD_STAT_SUCCESS;

 return retVal;
}

/******************************************************************************
*  SetAcquireDapHandshake
***************************************************************************//**
* Sets DAP Handshake type for Acquire flow
*
* @param[in] request Pointer to incoming USB packet
*
* @param[out] response Pointer to the memory for storing response
*
* @return size of response packet.
*
******************************************************************************/
static uint32_t SetAcquireDapHandshake(const uint8_t *request, uint8_t *response)
{
    if (SetHandshakeType(request[ID_DAP_V17_VALUE_OFFSET]))
    {
        response[GENERAL_RESPONSE_STATUS] = CMD_STAT_SUCCESS;
    }
    else
    {
        response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
    }
    return CMD_CODE_CMD_STAT_LEN;
}

/******************************************************************************
*  SetAcquireDapAp
***************************************************************************//**
* Sets DAP AP for Acquire flow
*
* @param[in] request Pointer to incoming USB packet
*
* @param[out] response Pointer to the memory for storing response
*
* @return size of response packet.
*
******************************************************************************/
static uint32_t SetAcquireDapAp(const uint8_t *request, uint8_t *response)
{
    Swd_SetApSelect(request[ID_DAP_V17_VALUE_OFFSET]);
    response[GENERAL_RESPONSE_STATUS] = CMD_STAT_SUCCESS;
    return CMD_CODE_CMD_STAT_LEN;
}

/******************************************************************************
*  SetAcquireOption
***************************************************************************//**
* Sets temporary options for Acquire flow
*
* @param[in] request Pointer to incoming USB packet
*
* @param[out] response Pointer to the memory for storing response
*
* @return size of response packet.
*
******************************************************************************/
uint32_t SetAcquireOption(const uint8_t *request, uint8_t *response)
{
    uint32_t retVal;

    switch (request[GENERAL_REQUEST_SUBCOMMAND])
    {
        case SET_ACQUIRE_PARAM_TIMEOUT:
            retVal = SetAcquireTimeout(request, response);
            break;
        case SET_ACQUIRE_PARAM_DAP_HANDSHAKE:
            retVal = SetAcquireDapHandshake(request, response);
            break;
        case SET_ACQUIRE_PARAM_DAP_AP:
            retVal = SetAcquireDapAp(request, response);
            break;
        default:
            /* Other parameters are not supported */
            response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
            retVal = CMD_CODE_CMD_STAT_LEN;
            break;
    }

    return (retVal);
}

/******************************************************************************
*  waitVendorResponse()
***************************************************************************//**
* Sends wait for 0x85(Acquire command) Responses to the host
******************************************************************************/
void WaitVendorResponse(void)
{
    uint8_t ep;
    uint8_t responseSize = DAP_PACKET_SIZE;
    static const uint8_t acquireWaitResponse[DAP_PACKET_SIZE] = { [0] = ID_DAP_Vendor5, [1] = CMD_STAT_WAIT };

    uint8_t cachedCurrentMode = currentMode;
    if ((cachedCurrentMode == MODE_BULK) || (cachedCurrentMode == MODE_BULK2UARTS))
    {
        /* Mark USB busy, flag is cleared in the USB EP ISR */
        ep = CMSIS_BULK_IN_EP;
        USB_ResponseIdle = false;
        while(USBFS_IN_BUFFER_EMPTY != USBFS_GetEPState(ep))
        {
            /* Wait for the data to be received by the host */
        }
        uint32_t intrMask = CyUsbIntDisable();
        USBFS_LoadInEP(ep, acquireWaitResponse, 2u);
        CyUsbIntEnable(intrMask);
        while(USBFS_IN_BUFFER_EMPTY != USBFS_GetEPState(ep))
        {
            /* Wait for the data to be received by the host */
        }
    }
    else
    {
        if (cachedCurrentMode == MODE_HID)
        {
            ep = CMSIS_HID_IN_EP;
            uint32_t intrMask = CyUsbIntDisable();
            USBFS_LoadInEP(ep, acquireWaitResponse, responseSize);
            CyUsbIntEnable(intrMask);
            while (USBFS_IN_BUFFER_EMPTY != USBFS_GetEPState(ep))
            {
                /* Wait for the data to be received by the host */
            }
        }
        else
        {
            /* Only Two modes are possible for CMSIS-DAP interface */
        }
    }
}


/******************************************************************************
*  CalculateUniqIdChecksum
***************************************************************************//**
* Calculate UID Record's checksum
*
* @param[in] uidRecord Pointer to uid record
*
* @return computed unique ID record checksum
*
******************************************************************************/
static uint8_t CalculateUniqIdChecksum(const unique_id_struct_t *uidRecord)
{
    uint8_t checksum = 0u;
    const uint8_t *uidAddress = (const uint8_t *)uidRecord;
    const uint8_t checkSumPos = sizeof(unique_id_struct_t) - sizeof(checksum);

    for (uint8_t index = 0u; index < checkSumPos; index++)
    {
        checksum += uidAddress[index];
    }

    return (uint8_t)(CRC8_2S_COMP_BASE - checksum);
}


/******************************************************************************
*  GetUidData()
***************************************************************************//**
* Returns to the host KitProg3 UID
*
* @param[in] request Pointer to incoming USB packet
*
* @param[out] response Pointer to the memory for storing response
*
* @returns size of response packet.
*
******************************************************************************/
uint32_t GetUidData(const uint8_t *request, uint8_t *response)
{
    (void)request;
    uint32_t num = 0u;

    const unique_id_struct_t *uidRecord = (unique_id_struct_t *)UNIQUE_ID_ADDRESS;

    /* Calculate unique id record's checksum */
    uint8_t uidChecksum = CalculateUniqIdChecksum(uidRecord);
    bool uidIsValid = ((uidRecord->signature == PSOC5_SIID) && (uidRecord->checksum == uidChecksum));

    response[GENERAL_RESPONSE_STATUS] = DAP_OK;

    if (uidIsValid)
    {
        const size_t uidRecSize = sizeof(unique_id_struct_t) - sizeof(uidRecord->signature);
        const uint8_t *uidRecPayload = (const uint8_t *)uidRecord;
        response[GENERAL_RESPONSE_RESULT] = UNIQUE_ID_VALID;
        (void)memcpy(&response[GENERAL_RESPONSE_RESULT + 1], &uidRecPayload[sizeof(uidRecord->signature)], uidRecSize);
        
        // Workaround for AI device, which has no DAPLink support encoded in UID Record
        if ((uidRecord->uid == UID_CY8CKIT_062S2_AI) && ((strncmp((const char *)uidRecord->mbedBoardId, (const char *)MBED_ID_UNSPECIFIED, sizeof(uidRecord->mbedBoardId) )) == 0 ))
        {
            (void)memcpy(&response[GENERAL_RESPONSE_RESULT + 1], (uint8_t *)MBED_ID_CY8CKIT_062S2_AI, sizeof(uidRecord->mbedBoardId));
            response[RESP_UID_PROG_OPT_AI] = UID_PROG_OPT_AI;
            response[RESP_UID_CHECKSUM_AI] = UID_CHECKSUM_AI;
        }
        num += uidRecSize + 3UL;
    }
    else
    {
        response[GENERAL_RESPONSE_RESULT] = UNIQUE_ID_INVALID;
        num += 3UL;
    }

    return num;
}

///@}
