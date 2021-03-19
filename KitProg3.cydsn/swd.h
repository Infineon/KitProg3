/*************************************************************************//**
* @file swd.h
*
* @brief
*  This file contains the function prototypes and constants used in
*  the swd.c.
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
#if !defined(SWD_H)
#define SWD_H

#include <stdint.h>
#include <stdbool.h>
#include "cypins.h"
#include "TCLK_SWDCLK.h"
#include "TMS_SWDIO.h"
#include "SWDXRES.h"


/*****************************************************************************
* MACRO Definition
*****************************************************************************/
#define BIT0                        (0x01u)
#define BIT1                        (0x02u)
#define BIT2                        (0x04u)
#define BIT3                        (0x08u)
#define BIT4                        (0x10u)
#define BIT5                        (0x20u)
#define BIT6                        (0x40u)
#define BIT7                        (0x80u)

/* Programming pin drive modes */
#define SWD_SET_SDA_OUT             CyPins_SetPinDriveMode(TMS_SWDIO_0, TMS_SWDIO_DM_STRONG)
#define SWD_SET_SDA_IN              CyPins_SetPinDriveMode(TMS_SWDIO_0, TMS_SWDIO_DM_DIG_HIZ)
#define SWD_SET_SCK_OUT             CyPins_SetPinDriveMode(TCLK_SWDCLK_0, TCLK_SWDCLK_DM_STRONG)
#define SWD_SET_SCK_IN              CyPins_SetPinDriveMode(TCLK_SWDCLK_0, TCLK_SWDCLK_DM_DIG_HIZ)
#define SWD_SET_XRES_OUT            CyPins_SetPinDriveMode(SWDXRES_0, SWDXRES_DM_STRONG)
#define SWD_SET_XRES_IN             CyPins_SetPinDriveMode(SWDXRES_0, SWDXRES_DM_DIG_HIZ)

/* SWD line communication macros */
#define SWD_XFER_SIZE               (0x04u)
#define SWD_OFLOW                   (BIT7)
#define SWD_UFLOW                   (BIT6)
#define SWD_DONE                    (BIT5)
#define SWD_PERR                    (BIT3)
#define SWD_FAULT                   (BIT2)
#define SWD_WAIT                    (BIT1)
#define SWD_ACK                     (BIT0)
#define SWD_ACK_BITS                ((SWD_FAULT) | (SWD_WAIT) | (SWD_ACK))
#define SWD_ERROR                   (SWD_ACK_BITS)

/* Acquire result macros */
#define ACQUIRE_PASS                (0x01u)
#define ACQUIRE_FAIL                (0x00u)
#define ACQUIRE_WAIT                (0x02u)

/* Bit banding of the SRAM addresses in order to address each bit of a byte separately. */
#define BIT_ADDRESS                 (0x20000100u)
#define SWD_BYTE                    (*((volatile uint8_t *)(BIT_ADDRESS)))

/* Convert SRAM address to SRAM bit map region */
#define BITBAND_SRAM_REF            (CYREG_SRAM_DATA_MBASE)
#define BITBAND_SRAM_BASE           (CYREG_SRAM_DATA_MBASE + 0x2000000u)

#define SWD_DATA_BIT_B0             (*((volatile uint8_t *)(((BITBAND_SRAM_BASE) + ((BIT_ADDRESS)-(BITBAND_SRAM_REF))*32u + (0u*4u)))))
#define SWD_DATA_BIT_B1             (*((volatile uint8_t *)(((BITBAND_SRAM_BASE) + ((BIT_ADDRESS)-(BITBAND_SRAM_REF))*32u + (1u*4u)))))
#define SWD_DATA_BIT_B2             (*((volatile uint8_t *)(((BITBAND_SRAM_BASE) + ((BIT_ADDRESS)-(BITBAND_SRAM_REF))*32u + (2u*4u)))))
#define SWD_DATA_BIT_B3             (*((volatile uint8_t *)(((BITBAND_SRAM_BASE) + ((BIT_ADDRESS)-(BITBAND_SRAM_REF))*32u + (3u*4u)))))
#define SWD_DATA_BIT_B4             (*((volatile uint8_t *)(((BITBAND_SRAM_BASE) + ((BIT_ADDRESS)-(BITBAND_SRAM_REF))*32u + (4u*4u)))))
#define SWD_DATA_BIT_B5             (*((volatile uint8_t *)(((BITBAND_SRAM_BASE) + ((BIT_ADDRESS)-(BITBAND_SRAM_REF))*32u + (5u*4u)))))
#define SWD_DATA_BIT_B6             (*((volatile uint8_t *)(((BITBAND_SRAM_BASE) + ((BIT_ADDRESS)-(BITBAND_SRAM_REF))*32u + (6u*4u)))))
#define SWD_DATA_BIT_B7             (*((volatile uint8_t *)(((BITBAND_SRAM_BASE) + ((BIT_ADDRESS)-(BITBAND_SRAM_REF))*32u + (7u*4u)))))

/* Bit banding of the peripheral addresses for flexibility in addressing SWDIO and SWDCLK */
/* Convert Peripheral address to peripheral bit map region */
#define BITBAND_PERI_REF            (0x40000000u)
#define BITBAND_PERI_BASE           (0x42000000u)

#define SWD_BITS                    (TCLK_SWDCLK__DR)
#define SWD_SDA                     (*((volatile uint8_t *)(((BITBAND_PERI_BASE) + ((SWD_BITS)-(BITBAND_PERI_REF))*32u + ((TMS_SWDIO_SHIFT)*4u)))))
#define SWD_SCK                     (*((volatile uint8_t *)(((BITBAND_PERI_BASE) + ((SWD_BITS)-(BITBAND_PERI_REF))*32u + ((TCLK_SWDCLK_SHIFT)*4u)))))
#define SDA_PS                      (*((volatile uint8_t *)(((BITBAND_PERI_BASE) + ((TMS_SWDIO__PS)-(BITBAND_PERI_REF))*32u + ((TMS_SWDIO_SHIFT)*4u)))))
#define SCL_PS                      (*((volatile uint8_t *)((((BITBAND_PERI_BASE) + ((TCLK_SWDCLK__PS)-(BITBAND_PERI_REF))*32u + ((TCLK_SWDCLK_SHIFT)*4u))))))
#define XRES_PS                     (*((volatile uint8_t *)((((BITBAND_PERI_BASE) + ((SWDXRES__PS)-(BITBAND_PERI_REF))*32u + ((SWDXRES_SHIFT)*4u))))))

#define SWD_SET_SCK_LO              ((SWD_SCK) = 0u)
#define SWD_SET_SCK_HI              ((SWD_SCK) = 1u)
#define SWD_SET_SDA_LO              ((SWD_SDA) = 0u)
#define SWD_SET_SDA_HI              ((SWD_SDA) = 1u)
#define SWD_SET_XRES_HI             CyPins_SetPin(SWDXRES_0)
#define SWD_SET_XRES_LO             CyPins_ClearPin(SWDXRES_0)

#define SWD_GET_SDA                 (SDA_PS)
#define SWD_GET_SCL                 (SCL_PS)
#define SWD_GET_XRES                (XRES_PS)

/* One Clock on the SWDCLK line */
#define SWD_CLOCK_BIT               ({SWD_SET_SCK_LO;CY_NOP;CY_NOP;CY_NOP;SWD_SET_SCK_HI;CY_NOP;CY_NOP;CY_NOP;})

/* Acquire target*/
#define ACQUIRE_PSoC4               (0x00u)
#define ACQUIRE_PSoC5_ES2           (0x01u)
#define ACQUIRE_PSoC6_BLE           (0x02u)
#define ACQUIRE_TVII                (0x03u)
#define ACQUIRE_CYW20829            (0x04u)
#define ACQUIRE_AUTODETECT          (0xFFu)

/* Acquire mode */
#define ACQUIRE_RESET               (0x00u)
#define ACQUIRE_POWER_CYCLE         (0x01u)
#define ACQUIRE_IDLE                (0x02u)

/* SWD data macros */
#define SWD_DATA_SIZE               (5u)
#define ONE_VOLT                    (1000u)

/*****************************************************************************
* Global Variables
*****************************************************************************/

extern bool swdHwAccelerationAllowed;
extern uint16_t perceivedTimeout;
/*****************************************************************************
* Function Prototypes
*****************************************************************************/

void Swd_Init(void);
void Swd_HwAccelPossibility(void);
void Swd_SetPinsDsiConnect(bool connect);
void Swd_SetHwClock(uint32_t curClock);
void Swd_SetHwIdleClk(uint8_t idleCycles);
uint8_t Swd_TransferHw(uint32_t request, uint32_t *data);
uint32_t Swd_Acquire(uint32_t dut, uint32_t acquireMode);
void Swd_SetActualAcquireTimeout(uint8_t acquireDUT, uint8_t acquireMode);
void Swd_SetApSelect(const uint8_t ap);
bool SetHandshakeType(const uint8_t handshakeType);

#endif /* SWD_H */


/* [] END OF FILE */
