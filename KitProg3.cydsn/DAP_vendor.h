/*************************************************************************//**
* @file DAP_vendor.h
*
* @brief
*   This file contains the function prototypes and constants used in
*   the DAP_vendor.c
*
* @version KitProg3 v2.50
*/
/*
* Related Documents:
*   002-27868 - KITPROG3 V1.2X EROS
*   002-23369 - KITPROG3 IROS
*   002-26377 - KITPROG3 1.1X TEST PLAN
*
*
******************************************************************************
* (c) (2018-2023), Cypress Semiconductor Corporation (an Infineon company)
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

#ifndef DAP_VENDOR_H
#define DAP_VENDOR_H

#include <stdbool.h>
#include <stdint.h>
#include "device.h"
#include "swd.h"


#define KHPI_VER                            "2.04"
#define KHPI_VER_MAJOR                      (2u)
#define KHPI_VER_MINOR                      (4u)

#define CMD_STAT_SUCCESS                    (0x00u)
#define CMD_STAT_WAIT                       (0x01u)
#define CMD_STAT_VOLT_SET_FAILED            (0x80u)
#define CMD_STAT_FAIL_INV_PAR               (0x81u)
#define CMD_STAT_FAIL_OP_FAIL               (0x82u)

#define LED_STATE_READY                     (0x00u)
#define LED_STATE_PROGRAMMING               (0x01u)
#define LED_STATE_SUCCESS                   (0x02u)
#define LED_STATE_ERROR                     (0x03u)

#define MODE_BOOTLOADER                     (0x00u)
#define MODE_CMSIS_DAP2X                    (0x01u)
#define MODE_CMSIS_DAP1X                    (0x02u)
#define MODE_CUSTOM_APP                     (0x03u)
#define MODE_CMSIS_DAP2X_DOUBLEUARTS        (0x04u)

#define SWD_XFER_SIZE                       (0x04u)

#define ID_DAP_V0_LEN                       (0x0cu)
#define ID_DAP_DEF_CASE_RESP_LEN            (0x01u)
#define CMD_CODE_CMD_STAT_LEN               (0x02u)
#define CMD_CODE_CMD_STAT_ERR_LEN           (0x03u)
#define ID_DAP_V4_LEN                       (0x08u)
#define ID_DAP_V17_VALUE_OFFSET             (0x02u)

#define RESP_85_SIZE_IMM                    (0x02u)

#define I2C_AVAILABILITY_MASK               (0x01u)
#define SPI_AVAILIBILITY_MASK               (0x02u)
#define DAPH_AVAILIBILITY_MASK              (0x04u)
#define DAPB_AVAILIBILITY_MASK              (0x08u)
#define ON_OFF_SW_AVAILIBILITY_MASK         (0x10u)
#define VMEAS_AVAILIBILITY_MASK             (0x20u)
#define GPIO_AVAILIBILITY_MASK              (0x40u)

#define ONE_LED_KIT_MASK                    (0x01u)
#define THREE_LED_KIT_MASK                  (0x03u)

#define ONE_UART_MASK                       (0x10u)
#define TWO_UART_MASK                       (0x20u)

/* Maximum divider that can be applied to SPI clock */
#define SPI_DIVIDER_MAX                    (((uint32_t)(UINT16_MAX))/2u)

/* In-component internal frequency divided applied to SPI clock */
#define SPI_COMP_DIVIDER                    (0x02u)

/* Minimum divider that can be applied to SPI clock */
#define SPI_DIVIDER_MIN                     (0x02u)

/* Supported SPI SS pins */
#define SPI_SS_LINES_MASK                   (0x07u)
#define SPI_SS_LINES_E_MASK                 (0x01u)

#define I2C_SPEEDS_MASK                     (0x0fu)

#define GPIO_PIN_35_MASK                    (0x10u)
#define GPIO_PIN_36_MASK                    (0x20u)

#define CMD_POWER_SET                       (0x10u)
#define CMD_POWER_GET                       (0x11u)
#define CMD_POWER_OFF                       (0x00u)
#define CMD_POWER_ON                        (0x01u)
#define CMD_POWER_VOLT_SET                  (0x02u)

#define PROB_CAP_RESP_LEN                   (15u)

#define POWER_1000_MV                       (1000u)
#define POWER_NOT_ACHIEVED                  (0x80u)

#define CUSTOM_APP_ID                       (1u)
#define TIMER_CSTICK_RATE                   (800u)
#define TIMER_CSTICK_HALF_RATE              (400u)
#define MAX_TIMEOUT_IN_SECONDS              (30u)

#define PSOC5_SIID                          (0x6970122Eu)
#define UNIQUE_ID_VALID                     (0x00u)
#define UNIQUE_ID_INVALID                   (0xFFu)
#define UNIQUE_ID_ADDRESS                   (CYDEV_EE_BASE + 16u)
#define PRI_UART_FLOW_CTRL_BYTE             (8u)
#define SEC_UART_FLOW_CTRL_BYTE             (9u)
#define UART_MODE_ADDRESS                   (CYDEV_EE_BASE + PRI_UART_FLOW_CTRL_BYTE)
#define CRC8_2S_COMP_BASE                   (0x0100u)

/* Enum for Basic DAP Vendor Response */
enum
{
    GENERAL_RESPONSE_COMMAND = 0u,
    GENERAL_RESPONSE_STATUS = 1u,
    GENERAL_RESPONSE_RESULT = 2u,
};

/* Enum for Basic DAP Vendor Request */
enum
{
    GENERAL_REQUEST_COMMAND = 0u,
    GENERAL_REQUEST_SUBCOMMAND = 1u,
};

/* Enum for GPIO Request */
enum
{
    GPIO_REQUEST_COMMAND = 0u,
    GPIO_REQUEST_PIN = 1u,
    GPIO_REQUEST_STATE_MODE = 2u,
};

/* Enum for Get/Set UART flow control mode */
enum
{
    CONFIG_UART_REQUEST_COMMAND = 1u,
    CONFIG_UART_REQUEST_PORT = 2u,
    CONFIG_UART_REQUEST_MODE = 3u,
};

/* Function prototypes */
void HandleReset(void);
void HandleModeSwitch(const uint8_t *request, uint8_t *response);
uint32_t HandleLedCmd(const uint8_t *request, uint8_t *response);
uint32_t HandleAcquire(const uint8_t *request, uint8_t *response);
uint32_t GetCapabilities(const uint8_t *request, uint8_t *response);
uint32_t GetSetPower(const uint8_t *request, uint8_t *response);
void WaitVendorResponse(void);
uint32_t SetAcquireOption(const uint8_t *request, uint8_t *response);
uint32_t GetUidData(const uint8_t *request, uint8_t *response);
extern uint32_t GetSetHwContol(const uint8_t *request, uint8_t *response);

#endif /* DAP_VENDOR_H */

