/*************************************************************************//**
* @file jtag.h
*
* @brief
*  This file contains the function prototypes and constants for jtag communication.
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

#if !defined(JTAG_H)
#define JTAG_H

#include <stdint.h>
#include "cypins.h"
#include "TDI.h"
#include "TDO_SWO.h"

/*****************************************************************************
* MACRO Definition
*****************************************************************************/
/* Define shared pins with SWD interface */
#define JTAG_SET_TMS_OUT            (SWD_SET_SDA_OUT)
#define JTAG_SET_TMS_IN             (SWD_SET_SDA_IN)
#define JTAG_SET_TCK_OUT            (SWD_SET_SCK_OUT)
#define JTAG_SET_TCK_IN             (SWD_SET_SCK_IN)
#define JTAG_SET_XRES_OUT           (SWD_SET_XRES_OUT)
#define JTAG_SET_XRES_IN            (SWD_SET_XRES_IN)

#define JTAG_SET_TMS_LO             (SWD_SET_SDA_LO)
#define JTAG_SET_TMS_HI             (SWD_SET_SDA_HI)
#define JTAG_SET_TCK_LO             (SWD_SET_SCK_LO)
#define JTAG_SET_TCK_HI             (SWD_SET_SCK_HI)
#define JTAG_SET_XRES_LO            (SWD_SET_XRES_LO)
#define JTAG_SET_XRES_HI            (SWD_SET_XRES_HI)


/* Programming pin drive modes */
#define JTAG_SET_TDI_OUT             CyPins_SetPinDriveMode(TDI_0, TDI_DM_STRONG)
#define JTAG_SET_TDI_IN              CyPins_SetPinDriveMode(TDI_0, TDI_DM_DIG_HIZ)
#define JTAG_SET_TDO_OUT             CyPins_SetPinDriveMode(TDO_SWO_0, TDO_SWO_DM_STRONG)
#define JTAG_SET_TDO_IN              CyPins_SetPinDriveMode(TDO_SWO_0, TDO_SWO_DM_DIG_HIZ)

/* Bit banding of the peripheral addresses for flexibility in addressing TDI and TDO */
/* Convert Peripheral address to peripheral bit map region */
#define BITBAND_PERI_REF            (0x40000000u)
#define BITBAND_PERI_BASE           (0x42000000u)

#define JTAG_TDI_DATA               (*((volatile uint8_t *)(((BITBAND_PERI_BASE) + (((TDI__DR)-(BITBAND_PERI_REF))*32u) + ((TDI_SHIFT)*4u)))))
#define JTAG_TDO_DATA               (*((volatile uint8_t *)(((BITBAND_PERI_BASE) + (((TDO_SWO__DR)-(BITBAND_PERI_REF))*32u) + ((TDO_SWO_SHIFT)*4u)))))

#define JTAG_TDI                    (*((volatile uint8_t *)(((BITBAND_PERI_BASE) + (((TDI__PS)-(BITBAND_PERI_REF))*32u) + ((TDI_SHIFT)*4u)))))
#define JTAG_TDO                    (*((volatile uint8_t *)(((BITBAND_PERI_BASE) + (((TDO_SWO__PS)-(BITBAND_PERI_REF))*32u) + ((TDO_SWO_SHIFT)*4u)))))

#define JTAG_SET_TDI_LO             ((JTAG_TDI_DATA) = 0u)
#define JTAG_SET_TDI_HI             ((JTAG_TDI_DATA) = 1u)
#define JTAG_SET_TDO_LO             ((JTAG_TDO_DATA) = 0u)
#define JTAG_SET_TDO_HI             ((JTAG_TDO_DATA) = 1u)

#define JTAG_GET_TDI                (JTAG_TDI)
#define JTAG_GET_TDO                (JTAG_TDO)

#endif /* JTAG_H */
/* [] END OF FILE */