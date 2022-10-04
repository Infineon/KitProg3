/*************************************************************************//**
* @file swd.c
*
* @brief
*  This file provides the source code to handle SWD programming.
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

#include "power.h"
#include "swd.h"
#include "DAP_config.h"
#include "DAP_vendor.h"
#include "DAP.h"
#include "version.h"
#include <stdbool.h>
#include <stdint.h>

/* Compile-time bit manipulations */
#define UINT8_BIT0(u8)        (((uint8_t)u8 >> 0u) & 1u)
#define UINT8_BIT1(u8)        (((uint8_t)u8 >> 1u) & 1u)
#define UINT8_BIT2(u8)        (((uint8_t)u8 >> 2u) & 1u)
#define UINT8_BIT3(u8)        (((uint8_t)u8 >> 3u) & 1u)
#define UINT8_BIT4(u8)        (((uint8_t)u8 >> 4u) & 1u)
#define UINT8_BIT5(u8)        (((uint8_t)u8 >> 5u) & 1u)
#define UINT8_BIT6(u8)        (((uint8_t)u8 >> 6u) & 1u)
#define UINT8_BIT7(u8)        (((uint8_t)u8 >> 7u) & 1u)
#define UINT8_PARITY(u8)      ((UINT8_BIT0(u8)+UINT8_BIT1(u8)+UINT8_BIT2(u8)+UINT8_BIT3(u8)+ \
                               UINT8_BIT4(u8)+UINT8_BIT5(u8)+UINT8_BIT6(u8)+UINT8_BIT7(u8)) & 1u)
#define UINT16_PARITY(u16)    ((UINT8_PARITY(HI8(u16))+UINT8_PARITY(LO8(u16))) & 1u)
#define UINT32_PARITY(u32)    ((UINT16_PARITY(HI16(u32))+UINT16_PARITY(LO16(u32))) & 1u)
#define UINT32_FOUR_UINT8(u32) {LO8(LO16(u32)), HI8(LO16(u32)), LO8(HI16(u32)), HI8(HI16(u32))}
#define UINT32_FOUR_UINT8_AND_PARITY(u32) UINT32_FOUR_UINT8(u32), UINT32_PARITY(u32)
#define SWD_WRITE_BLOCK(u8_cmd,u32_data) {u8_cmd, UINT32_FOUR_UINT8_AND_PARITY(u32_data)}

/* Runtime transform macros */
#define SHIFT_LEFT(cmd,bits) (((uint32_t)(uint8_t)(cmd))<<(bits))
#define PACK_UINT8_TO_UINT32(u8_ptr)        ((SHIFT_LEFT((u8_ptr[3u]),24u))|(SHIFT_LEFT((u8_ptr[2u]),16u))| \
                                             (SHIFT_LEFT((u8_ptr[1u]),8u))|(SHIFT_LEFT((u8_ptr[0u]),0u)))
#define PACK_UINT8_TO_UINT32_MSB(u8_ptr)    ((SHIFT_LEFT((u8_ptr[0u]),24u))|(SHIFT_LEFT((u8_ptr[1u]),16u))| \
                                             (SHIFT_LEFT((u8_ptr[2u]),8u))|(SHIFT_LEFT((u8_ptr[3u]),0u)))

/* SWD request phase bits */
#define SWD_REQ_START     (1u<<0u)
#define SWD_REQ_AP        (1u<<1u)
#define SWD_REQ_DP        (0u<<1u)
#define SWD_REQ_R         (1u<<2u)
#define SWD_REQ_W         (0u<<2u)
#define SWD_REQ_STOP      (0u<<6u)
#define SWD_REQ_PARK      (1u<<7u)

#define DP_W_ABORT        (0x81u)
#define DP_W_CTRL_STAT    (0xA9u)
#define DP_W_SELECT       (0xB1u)
#define DP_W_RDBUFF       (0x99u)
#define DP_R_IDCODE       (0xA5u)
#define DP_R_RDBUFF       (0xBDu)
#define AP_R_BASE         (0xB7u)
#define AP_R_DRW          (0x9Fu)
#define AP_W_CSW          (0xA3u)
#define AP_W_TAR          (0x8Bu)
#define AP_W_DRW          (0xBBu)

/* PSoC5 specific values */
#define PSOC5_TM_REG            (0xEA7E30A9u)
#define PSOC5_PORT_ACQUIRE_KEY  (0x7B0C06DBu)
#define PSOC5_DP_IDCODE         (0x2BA01477u)
#define PSOC5_PORT_ACQ_COM_POS  (0x01u)
#define PSOC5_PORT_ACQ_KEY_POS  (0x02u)

#define MAX_OVERALL_COMMAND_INDEX   (59u)
#define MAX_LONG_COMMAND_INDEX      (55u)

/* Delays after power on and power off while acquiring */
#define PWR_OFF_DELAY_US                (100000u)
#define PWR_ON_DELAY_US                 (100u)

/* Idle cycle count before SWD header segment sending */
#define SWD_IDLE_CYCLES (20u)

/* Timeouts in Timer_CSTick ticks (800Hz rate, 800 ticks == 1000ms) */
/* XRES deassert timeout */
#define XRES_DEASSERT_TIMEOUT_TICKS    (80u)

/* PSoC4 Acquire timeout */
#define PSOC4_AQUIRE_TIMEOUT_DEFAULT_TICKS                  (2u)
#define PSOC4_AQUIRE_TIMEOUT_RESET_TICKS                    (2u)
#define PSOC4_AQUIRE_TIMEOUT_POWER_CYCLE_TICKS              (4u)

/* PSoC3 and PSoC5 Acquire timeout */
#define PSOC3_5_AQUIRE_TIMEOUT_DEFAULT_TICKS                  (2u)
#define PSOC3_5_AQUIRE_TIMEOUT_RESET_TICKS                    (2u)
#define PSOC3_5_AQUIRE_TIMEOUT_POWER_CYCLE_TICKS              (4u)
#define PSOC3_5_AQUIRE_TIMEOUT_POWER_CYCLE_UNSUPPORTED_TICKS  (240u)

/* PSoC6 and TVII Acquire timeout */
#define PSOC6_TVII_AQUIRE_TIMEOUT_TICKS                     (800u)
#define PSOC6_TVII_HALF_AQUIRE_TIMEOUT_TICKS                (PSOC6_TVII_AQUIRE_TIMEOUT_TICKS/2u)

/* CYW20829 and TVII Acquire timeout */
#define CYW20829_AQUIRE_TIMEOUT_TICKS                       (800u)
#define CYW20829_HALF_AQUIRE_TIMEOUT_TICKS                  (CYW20829_AQUIRE_TIMEOUT_TICKS/2u)

/* ROM table */
#define BASE_ADDR_MASK      (0xFFFFF000u)
#define BASE_FORMAT_MASK    (0x00000003u)
#define PIDR0_ADDR          (0x00000FE0u)
#define PIDR4_ADDR          (0x00000FD0u)
#define PIDR_0_3_VALID_MASK (0x000FF000u)
#define PIDR_0_3_VALID_VAL  (0x000B4000u)
#define FAMILY_ID_MASK      (0x00000FFFu)

/* Device Family list */
#define PSoC4_MCU           (0x92u)
#define PSoC4A              (0x93u)
#define PSoC4B              (0x94u)
#define PSoC4C              (0x95u)
#define PSoC4C_TS           (0x96u)
#define PSoC4A_BLE          (0x9Eu)
#define PSoC4A_L            (0xA0u)
#define PSoC4A_M            (0xA1u)
#define PSoC4A_BLE256       (0xA3u)
#define PSoC4A_DS2          (0xA7u)
#define PSoC4A_S1           (0xA9u)
#define PSoC4A_BLE256DMA    (0xAAu)
#define PSoC4A_S2           (0xABu)
#define PSoC4B_S0           (0xACu)
#define PSOC4A_BLEII        (0xAEu)
#define PSoC4A_S3           (0xB5u)
#define PSoC4A_MC           (0xB8u)
#define PSoC4A_S4           (0xBEu)
#define PSoC4A_HV_192k      (0xC2u)

#define PSoC6A_BLE2          (0x100u)
#define TraveoII_B_E_1M      (0x101u)
#define PSoC6A_2M            (0x102u)
#define TraveoII_B_H_8M      (0x103u)
#define TraveoII_B_E_2M      (0x104u)
#define PSoC6A_512K          (0x105u)
#define TraveoII_C_2D_6M     (0x106u)
#define TraveoII_B_H_4M      (0x107u)
#define TraveoII_B_E_4M      (0x108u)
#define TraveoII_C_2D_4M     (0x10Bu)
#define TraveoII_C_H_4M      (0x10Cu)
#define TraveoII_C_E_2M      (0x10Du)
#define PSOC6A_256K          (0x10Eu)
#define TraveoII_C_2D_6M_DDR (0x10Fu)
#define CYW20828             (0x110u)

/* SWD UDB related defines */
#define SWD_DATA_WRITE_REG          (* (reg32 *) SWD_1_data_dp_u0__F0_REG)
#define SWD_DATA_READ_REG           (* (reg32 *) SWD_1_data_dp_u0__F1_REG)
#define SWD_DATA_AUX_CTL_REG        (* (reg32 *) SWD_1_data_dp_u0__DP_AUX_CTL_REG)
#define SWD_PREAMBLE_WRITE_REG      (* (reg8 *)  SWD_1_preamble_dp_u0__F0_REG)
#define SWD_PREAMBLE_READ_REG       (* (reg8 *)  SWD_1_preamble_dp_u0__F1_REG)
#define SWD_PREAMBLE_AUX_CTL_REG    (* (reg32 *) SWD_1_preamble_dp_u0__DP_AUX_CTL_REG)
#define SWD_DATA_COUNT_AUX_CTL_REG  (* (reg8 *)  SWD_1_DataCounter__CONTROL_AUX_CTL_REG)
#define SWD_ACK_COUNT_AUX_CTL_REG   (* (reg8 *)  SWD_1_AckCounter__CONTROL_AUX_CTL_REG)
#define SWD_PRE_COUNT_AUX_CTL_REG   (* (reg8 *)  SWD_1_PreambleCounter__CONTROL_AUX_CTL_REG)
#define SWD_PRTY_COUNT_AUX_CTL_REG  (* (reg8 *)  SWD_1_ParityCounter__CONTROL_AUX_CTL_REG)
#define SWD_IDLE_COUNT_AUX_CTL_REG  (* (reg8 *)  SWD_1_IdleCounter__CONTROL_AUX_CTL_REG)
#define SWD_STATUS_REG              (* (reg8 *)  SWD_1_StatusReg__STATUS_REG)
#define SWD_IDLE_COUNT_PERIOD_REG   (* (reg8 *)  SWD_1_IdleCounter__PERIOD_REG)
#define SWD_ACK_OK_PATTERN_REG      (* (reg8 *)  SWD_1_preamble_dp_u0__D0_REG)

#define FIFO_8_CLEAR            (0x03u)
#define FIFO_32_CLEAR           (0x03030303u)

#define WRITE_OP                (0x01u)
#define READ_OP                 (0x03u)
#define SKIP_OP                 (0x04u)
#define NO_OP                   (0x00u)

#define GET_ACK(x) (((x) >> 4) & 0x07u)

#define ONEGHZ_CLK              (1000000000u)
#define MASTER_CLK              (64000000u)
#define IMO_CLK                 (24000000u)
#define MAX_SWD_CLK             (16000000u)
#define HW_MIN_IDLE_CYCLES      (1u)
#define HW_MAX_IDLE_CYCLES      (64u)

/* Park Bit=1, Stop Bit=0, Parity Bit, A3 Bit, A2 Bit, RnW Bit, APnDP Bit, Start Bit=1 */
#define HW_PREAMBLE_TEMPLATE    (0x81u)

/* Parity for values in 0x0..0xF range is 0b0110100110010110 => 0x6996 */
#define PARITY_MAGIC            (0x6996u)

/* HW preamble from SWD transfer request creation macro */
#define HW_PREAMBLE(req)        ((HW_PREAMBLE_TEMPLATE) | ((uint8_t)((req) & 0x0Fu) << 1u) | ((((PARITY_MAGIC) >> (uint8_t)((req) & 0x0Fu)) & 1u) << 5u))

typedef struct __attribute__((packed)) {
    uint8_t command;
    uint8_t data[sizeof(uint32_t)];
    uint8_t dataParity;
} SwdWriteDataBlock_t;

enum SwdWriteBlockIndex_e {
    ACQUIRE_DP_PSOC3_5,       /* Initialize Debug Port CTRL/STAT for PSOC3/5                */
    INIT_DP_CR_PSOC4,         /* Initialize Debug Port CTRL/STAT for PSOC4                  */
    INIT_DP_CR_PSOC6_CYW20829,/* Initialize Debug Port CTRL/STAT for PSOC6, TVII, CYW20829  */
    SEL_AP_BANK0,             /* APSEL=0x00, APBANKSEL=0x0, PRESCALER=0x0                   */
    SEL_AP_BANKF,             /* APSEL=0x00, APBANKSEL=0xF, PRESCALER=0x0                   */
    SET_CSW_DEFAULT,          /* Set CSW 32-bit transfer mode                               */
    SET_CSW_PROTECT,          /* Set CSW Prot=0x23, DeviceEn=1, 32-bit transfer mode        */
    SET_TAR_TMR_PSOC3,        /* Set TAR to PSOC3 Test Mode register as destination         */
    SET_TAR_TMR_PSOC4,        /* Set TAR to PSOC4 Test Mode register as destination         */
    SET_TAR_TMR_PSOC5,        /* Set TAR to PSOC5 Test Mode register as destination         */
    SET_TAR_TMR_PSOC6,        /* Set TAR to PSOC6 Test Mode register as destination         */
    SET_TAR_TMR_TVII,         /* Set TAR to TVII Test Mode register as destination          */
    SET_TAR_TMR_CYW20829,     /* Set TAR to CYW20829 Test Mode register as destination      */
    SET_DWR_TMR_PSOC3_5,      /* Write data into Test Mode register for PSOC3,PSOC5         */
    SET_DWR_TMR_PSOCX,        /* Write data into Test Mode register for PSOC4,PSOC6,TVII    */
    CLR_ERRORS,               /* Write to ABORT DP register                                 */
    NUM_OF_ITEMS
};

/* SWD Requests*/
static SwdWriteDataBlock_t swdWriteBlockDict[NUM_OF_ITEMS] = {
    [ACQUIRE_DP_PSOC3_5]        = SWD_WRITE_BLOCK(DP_W_RDBUFF,    0x7B0C06DBu), /* Acquire Debug Port */
    [INIT_DP_CR_PSOC4]          = SWD_WRITE_BLOCK(DP_W_CTRL_STAT, 0x54000000u), /* Initialize Debug Port CTRL/STAT                            */
    [INIT_DP_CR_PSOC6_CYW20829] = SWD_WRITE_BLOCK(DP_W_CTRL_STAT, 0x50000000u), /* Initialize Debug Port CTRL/STAT                           */
    [SEL_AP_BANK0]              = SWD_WRITE_BLOCK(DP_W_SELECT,    0x00000000u), /* Select SYS AP                                            */
    [SEL_AP_BANKF]              = SWD_WRITE_BLOCK(DP_W_SELECT,    0x000000F0u), /* APSEL=0x00, APBANKSEL=0xF, PRESCALER=0x0                */
    [SET_CSW_DEFAULT]           = SWD_WRITE_BLOCK(AP_W_CSW,       0x00000002u), /* CSW 32-bit transfer mode                               */
    [SET_CSW_PROTECT]           = SWD_WRITE_BLOCK(AP_W_CSW,       0x23000052u), /* CSW Prot=0x23, DeviceEn=1, 32-bit transfer mode       */
    [SET_TAR_TMR_PSOC3]         = SWD_WRITE_BLOCK(AP_W_TAR,       0x00050210u), /* Set TAR to PSOC3 Test Mode register as destination   */
    [SET_TAR_TMR_PSOC4]         = SWD_WRITE_BLOCK(AP_W_TAR,       0x40030014u), /* Set TAR to PSOC4 Test Mode register as destination   */
    [SET_TAR_TMR_PSOC5]         = SWD_WRITE_BLOCK(AP_W_TAR,       0x40050210u), /* Set TAR to PSOC5 Test Mode register as destination  */
    [SET_TAR_TMR_PSOC6]         = SWD_WRITE_BLOCK(AP_W_TAR,       0x40260100u), /* Set TAR to PSOC6 Test Mode register as destination */
    [SET_TAR_TMR_TVII]          = SWD_WRITE_BLOCK(AP_W_TAR,       0x40261100u), /* Set TAR to TVII Test Mode register as destination */
    [SET_TAR_TMR_CYW20829]      = SWD_WRITE_BLOCK(AP_W_TAR,       0x40200400u), /* Set TAR to CYW20829 Acuire flag in BootRom RAM as destination      */
    [SET_DWR_TMR_PSOC3_5]       = SWD_WRITE_BLOCK(AP_W_DRW,       0xEA7E30A9u), /* Write data into Test Mode register for PSOC3/5                      */
    [SET_DWR_TMR_PSOCX]         = SWD_WRITE_BLOCK(AP_W_DRW,       0x80000000u), /* Write data into Test Mode register for PSOC4,PSOC6,TVII,CYW20829 */
    [CLR_ERRORS]                = SWD_WRITE_BLOCK(DP_W_ABORT,     0x0000001Eu), /* ORUNERRCLR=1, WDERRCLR=1, STKERRCLR=1, STKCMPCLR=1, DAPABORT=0  */
};

/* Set Test Mode register as destination index */
static const enum SwdWriteBlockIndex_e genTestAddrIndex[6u] = {
    SET_TAR_TMR_PSOC4,
    SET_TAR_TMR_PSOC5,
    SET_TAR_TMR_PSOC6,
    SET_TAR_TMR_TVII,
    SET_TAR_TMR_CYW20829,
    SET_TAR_TMR_PSOC3,
};

/* Write data into Test Mode register index */
static const enum SwdWriteBlockIndex_e genTestDataIndex[6u] = {
    SET_DWR_TMR_PSOCX,    /* PSOC4 */
    SET_DWR_TMR_PSOC3_5,  /* PSOC5 */
    SET_DWR_TMR_PSOCX,    /* PSOC6 */
    SET_DWR_TMR_PSOCX,    /* TVII */
    SET_DWR_TMR_PSOCX,    /* CYW20829 */
    SET_DWR_TMR_PSOC3_5,  /* PSOC3 */
};

/*****************************************************************************
* Local Variable Declarations
*****************************************************************************/

/* 8-bit header */
static uint8_t gbHeader;

/* 32-bit data */
static uint8_t gbData[SWD_XFER_SIZE];

/* gbData parity */
static uint8_t gbParity;

/* 8-bit status */
static uint8_t gbStatsReg;

/* 3-bit ACK response */
static uint8_t gbAck;
static uint8_t ack0 = 0u;
static uint8_t ack1 = 0u;
static uint8_t ack2 = 0u;

/* 12bit value of Family ID */
static uint16_t familyId = 0u;
uint16_t perceivedTimeout;

/* DAP handshake type */
typedef enum {
    DAP_HANDSHAKE_DEFAULT = 0u,
    DAP_HANDSHAKE_LINERESET,
    DAP_HANDSHAKE_NO_DORMANT,
    DAP_HANDSHAKE_DORMANT_SHORT,
    DAP_HANDSHAKE_DORMANT_FULL
} en_dapHandshake_t;
static en_dapHandshake_t dapHandshakeType = DAP_HANDSHAKE_DEFAULT;

/* Forward declaration of default handshake function */
static void SwitchJtagToSwd(void);
/* Pointer to actual handshake function */
static void (*dapHandshake)(void) = &SwitchJtagToSwd;

/* Acquire timeout amount */
static uint16_t acquireTimeout = PSOC6_TVII_HALF_AQUIRE_TIMEOUT_TICKS;
bool swdHwAccelerationAllowed = true;
static uint32_t hwClockSwdPeriod = ONEGHZ_CLK / DAP_DEFAULT_SWJ_CLOCK; /* SWD source clock period in ns,  */
static uint32_t idleGuardPeriod = 8u * (ONEGHZ_CLK / DAP_DEFAULT_SWJ_CLOCK);   /* idle cycles duration in us */
static uint8_t hwIdleCycles = 1u;

/*****************************************************************************
* Local Functions
*****************************************************************************/

/******************************************************************************
*  SwdCountBits
***************************************************************************//**
* Counts the number of '1's in the byte passed.
*
* @param[in] data  Byte for which number of ones is to be calculated
*
* @return Number of ones in the byte.
*
******************************************************************************/
static uint8_t SwdCountBits(uint8_t data)
{
    uint8_t countBit = 0;
    uint8_t curData = data;
    for (; curData != 0u; curData >>= 1)
    {
        countBit += curData & 1u;
    }

    return countBit;
}

/******************************************************************************
*  SwdComputeDataParity
***************************************************************************//**
* Calculates the parity for the data segment.
*
* @param[in]  pbData    Pointer to the address location of data whose parity has
*                       to be calculated.
*
* @return  Parity of the data segment.
*
******************************************************************************/
static uint8_t SwdComputeDataParity(const uint8_t *pbData)
{
    uint8_t count = 0u;
    for (uint8_t index = 0u; index < SWD_XFER_SIZE; index++)
    {
        count += SwdCountBits(pbData[index]);
    }

    return (count & 1u);
}

/******************************************************************************
*  SwdClockTrn
***************************************************************************//**
* Puts the PSoC 5LP in the turn around mode of SWD programming.
* SDA remain undriven
******************************************************************************/
static void SwdClockTrn(void)
{
    SWD_SET_SDA_IN;
    SWD_CLOCK_BIT;
}

/******************************************************************************
*  SwdLineReset
***************************************************************************//**
* Reset the SWD bus by clocking 60 times.
* SDA remain driven in HI
******************************************************************************/
static void SwdLineReset(void)
{
    SWD_SET_SCK_OUT;

    /* see 001-11605  4.2.1.2. Clocks more than 52 cycles when SDA in HIGH to reset the SWD bus */
    SWD_SET_SDA_HI;
    SWD_SET_SDA_OUT;
    for ( uint32_t clk = 0u; clk < 60u; clk++)
    {
        SWD_CLOCK_BIT;
    }
    /* per ARM IHI 0031C at least two Idle cycles (SDA LO) before sending header is required
       this will be done in the SWDPutHeaderSegment function */
}

/******************************************************************************
*  SwdGetAckSegment
***************************************************************************//**
* Retrieves the Ack bits from the SWD lines. Fill result in ack0, ack1 and ack2
* variables. Ack = 0:fail, 1:ACK, 2:WAIT, 4:FAULT
*
******************************************************************************/
static void SwdGetAckSegment(void)
{
    /* get ACK bits, SDA is expected to be controlled by target, SDA drive mode has been changed in SWDClockTrn  */

    SWD_SET_SCK_LO;
    ack0 = SWD_GET_SDA;
    SWD_SET_SCK_HI;

    SWD_SET_SCK_LO;
    ack1 = SWD_GET_SDA;
    SWD_SET_SCK_HI;

    SWD_SET_SCK_LO;
    ack2 = SWD_GET_SDA;
    SWD_SET_SCK_HI;
}

/******************************************************************************
*  SwdPutByte
***************************************************************************//**
* Writes a byte on to the SWD lines.
*
* @param[in] bPutData  Byte to be written on to the SWD lines.
*
******************************************************************************/
static void SwdPutByte(uint8_t bPutData)
{
    /* SDA is expected to be controlled by host */
    SWD_BYTE = bPutData;

    SWD_SET_SCK_LO; SWD_SDA = SWD_DATA_BIT_B0; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_SDA = SWD_DATA_BIT_B1; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_SDA = SWD_DATA_BIT_B2; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_SDA = SWD_DATA_BIT_B3; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_SDA = SWD_DATA_BIT_B4; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_SDA = SWD_DATA_BIT_B5; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_SDA = SWD_DATA_BIT_B6; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_SDA = SWD_DATA_BIT_B7; SWD_SET_SCK_HI;
}

/******************************************************************************
*  SwdGetByte
***************************************************************************//**
* Reads a byte from the SWD lines.
*
* @return Byte read from the SWD lines.
*
******************************************************************************/
static uint8_t SwdGetByte(void)
{
    /* SDA is expected to be controlled by target */
    SWD_SET_SCK_LO; SWD_DATA_BIT_B0 = SWD_GET_SDA; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_DATA_BIT_B1 = SWD_GET_SDA; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_DATA_BIT_B2 = SWD_GET_SDA; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_DATA_BIT_B3 = SWD_GET_SDA; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_DATA_BIT_B4 = SWD_GET_SDA; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_DATA_BIT_B5 = SWD_GET_SDA; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_DATA_BIT_B6 = SWD_GET_SDA; SWD_SET_SCK_HI;
    SWD_SET_SCK_LO; SWD_DATA_BIT_B7 = SWD_GET_SDA; SWD_SET_SCK_HI;

    return SWD_BYTE;
}

/******************************************************************************
*  SwdPutDataSegment
***************************************************************************//**
* Puts the complete data segment received on to the SWD lines.
*
* @param[in] pbData Pointer to the address with data to write.
*
******************************************************************************/
static void SwdPutDataSegment(const uint8_t *pbData)
{
    SWD_SET_SDA_OUT;
    for (uint8_t index = 0u; index < SWD_XFER_SIZE; index++)
    {
        SwdPutByte(pbData[index]);
    }
}

/******************************************************************************
*  SwdPutHeaderSegment
***************************************************************************//**
* Puts the Header segment on to the SWD lines.
*
******************************************************************************/
static void SwdPutHeaderSegment(void)
{
    SWD_SET_SDA_LO;
    SWD_SET_SDA_OUT;

    /* at least two Idle cycles (SDA LO) before sending header is required */
    /* especially for PSoC 5 acquiring the idle cycles are unwanted */
    for (uint32_t clk = 0u; clk < SWD_IDLE_CYCLES; clk++)
    {
        SWD_CLOCK_BIT;
    }
    SwdPutByte(gbHeader);
}

/******************************************************************************
*  SwdPutData
***************************************************************************//**
* Calls the functions to write data segment on to the SWD lines and checks the
* acknowledgment to decide whether the write was successful or not.
*
* @param[in]  pbData    Pointer to the address location where data to be written
*                       to the SWD lines is stored.
*
******************************************************************************/
static void SwdPutData(const uint8_t  *pbData)
{
    gbStatsReg = 0u;
    SwdPutHeaderSegment();

    SwdClockTrn();

    SwdGetAckSegment();
    gbAck = ack0 | (ack1 << 1u) | (ack2 << 2u);

    SwdClockTrn();

    switch (gbAck)
    {
        case SWD_ACK:
        {
            SwdPutDataSegment(pbData);

            if (gbParity != 0u)
            {
                SWD_SET_SDA_HI;
            }
            else
            {
                SWD_SET_SDA_LO;
            }

            /* 46th clock ends SWD transaction */
            SWD_CLOCK_BIT;

            /* keep Data low to not start next SWD transaction on 47th clock */
            SWD_SET_SDA_LO;
            /* 47th-54th clock is required to move Addr/Data from DAP to AHB bus. CDT 127910 */
            for ( uint32_t clk = 47u; clk < 55u; clk++)
            {
                SWD_CLOCK_BIT;
            }
            SWD_SET_SDA_IN;
            SWD_SET_SDA_HI;
            SWD_SET_SCK_HI;

            gbStatsReg |= gbAck;
            break;
        }

        case SWD_WAIT:
        case SWD_FAULT:
        {
            SWD_SET_SDA_IN;
            SWD_SET_SDA_HI;
            SWD_SET_SCK_HI;
            gbStatsReg |= gbAck;
            break;
        }

        default:
        {
            SWD_SET_SDA_IN;
            SWD_SET_SDA_HI;
            SWD_SET_SCK_HI;

            /* Protocol Error - no response from target */
            gbStatsReg |= SWD_ERROR;
            break;
        }
    }
  return;
}

/******************************************************************************
*  SwdGetDataSegment
***************************************************************************//**
* Reads the data segment from the SWD lines.
*
* @param[out]   pbData   Pointer to the address where data to be written to
*                        the SWD lines is stored.
*
******************************************************************************/
static void SwdGetDataSegment(uint8_t  *pbData)
{
    SWD_SET_SDA_IN;

    for (uint8_t index = 0u; index < SWD_XFER_SIZE; index++)
    {
        pbData[index] = SwdGetByte();
    }
}

/******************************************************************************
*  SwdGetData
***************************************************************************//**
* Invokes the functions to read a data segment from the SWD lines and checks the
* acknowledgment to decide whether the read operation was successful or not.
*
* @param[out] pbData    Pointer to the address location where data read from the
*                       SWD lines has to be stored.
*
******************************************************************************/
static void SwdGetData(uint8_t *pbData)
{
    gbStatsReg = 0u;
    SwdPutHeaderSegment();
    SwdClockTrn();
    SwdGetAckSegment();
    gbAck = ack0 | (ack1 << 1u) | (ack2 << 2u);

    switch (gbAck)
    {
        /* This handles error in pre-ES10 silicon for reads */
        case SWD_ACK+SWD_WAIT:

        /* This handles normal expected response for ACK=OK */
        case SWD_ACK:
        {
            uint8_t parity;

            SwdGetDataSegment(pbData);
            SWD_SET_SCK_LO;
            parity = SWD_GET_SDA;
            SWD_SET_SCK_HI;
            SwdClockTrn();

            if (SwdComputeDataParity(pbData) != parity)
            {
                gbStatsReg |= SWD_PERR;
            }

            SWD_SET_SDA_IN;
            SWD_SET_SDA_HI;
            SWD_SET_SCK_HI;
            gbStatsReg |= gbAck;
            break;
        }

        case SWD_WAIT:
        case SWD_FAULT:
        {
            SwdClockTrn();

            (void)memset(pbData, 0, SWD_XFER_SIZE);

            SWD_SET_SDA_IN;
            SWD_SET_SDA_HI;
            SWD_SET_SCK_HI;
            gbStatsReg |= gbAck;
            break;
        }

        default:
        {
            SwdClockTrn();
            SWD_SET_SDA_IN;
            SWD_SET_SDA_HI;
            SWD_SET_SCK_HI;

            /* Protocol Error - no response from target */
            gbStatsReg |= SWD_ERROR;
            break;
        }
    }
  return;
}

/******************************************************************************
*  SwdAcquirePutdataBlock
***************************************************************************//**
* Puts data to SWD line
*
* @param[in]  dataBlock   Pointer to SWDWriteDataBlock_t
*
* @return  Boolean value representing if SWD packet ACK data.
*
******************************************************************************/
static bool SwdAcquirePutdataBlock(const SwdWriteDataBlock_t *dataBlock)
{
    SWD_SET_SDA_OUT;
    SwdPutByte(dataBlock->command);
    SwdClockTrn();
    SwdGetAckSegment();
    SwdClockTrn();
    SwdPutDataSegment(dataBlock->data);

    if (dataBlock->dataParity != 0u)
    {
        SWD_SET_SDA_HI;
    }
    else
    {
        SWD_SET_SDA_LO;
    }

    SWD_CLOCK_BIT; /* clock parity bit */

    /*
     * Idle Cycles - toggle clock with the data line put LO
     * push the SWD-data into the silicons register (latch it up)
     * issue is seen with PSoC5 LP, after Test Mode key was issued
     * but it was not actually applied without some more trailing clocks
     */
    if (dataBlock->dataParity != 0u)
    {
        SWD_SET_SDA_LO;
    }
    SWD_CLOCK_BIT;
    SWD_CLOCK_BIT;
    SWD_CLOCK_BIT;

    SWD_SET_SCK_LO;

    return (ack0 != 0u) && (ack1 == 0u) && (ack2 == 0u);
}

/******************************************************************************
*  SwdWriteBlock
***************************************************************************//**
* Executes SWD-write command on the bus.
* Command, data and parity is provided in block
* result (ACK) is returned to buf_ACK buffer.
*
* @param[in]  dataBlock    pointer to SWDWriteDataBlock_t
* @param[out] buf_ACK   buffer for return (ACK).
*
******************************************************************************/
static void SwdWriteBlock(const SwdWriteDataBlock_t *dataBlock, uint8_t *buf_ACK)
{
    /* bRetry - should only take 2 at most */
    register uint8_t retry = 10u;

    gbHeader = dataBlock->command;
    gbParity = dataBlock->dataParity;

    do
    {
        SwdPutData(dataBlock->data);
        retry--;
    }while ((retry != 0u) && ((gbStatsReg & SWD_ACK_BITS) == SWD_ACK_BITS));

    buf_ACK[0u] = gbStatsReg | SWD_DONE;
    return;
}

/******************************************************************************
*  SwitchJtagToSwd
***************************************************************************//**
*  Generates JTAG to SWD sequence:
*   - 60-cycles Line Reset
*   - 2-bytes 0xE79E, transmitted least-significant-bit (LSB) first.
*   - 60-cycles Line Reset
*
*******************************************************************************/
static void SwitchJtagToSwd(void)
{
    SwdLineReset();
    /* Clock J2S 16-bit header */
    SwdPutByte(0x9Eu);  /* see 001-11605 4.5.1.5 JTAG to SWD */
    SwdPutByte(0xE7u);
    SwdLineReset();
}

/******************************************************************************
*  SwitchDormantToSwd
***************************************************************************//**
*  Generates Dormant to SWD sequence:
*   - 8-cycle with SWDIOTMS in HIGH
*   - 128-bit alert sequence 0x19BC0EA2 E3DDAFE9 86852D95 6209F392 transmitted LSB first.
*   - 4-with SWDIOTMS LOW
*   - 8 bit activation code sequence 0x1A, for SWD, LSB first
*   - at least 50 cycles with SWDIOTMS HIGH
*******************************************************************************/
static void SwitchDormantToSwd(void)
{
    static const uint8_t swdSeqDormantToSwd[] =
    {
        /* At least 8 SWCLK cycles with SWDIO high */
        0xffu,
        /* Selection alert sequence */
        0x92u, 0xf3u, 0x09u, 0x62u, 0x95u, 0x2du, 0x85u, 0x86u,
        0xe9u, 0xafu, 0xddu, 0xe3u, 0xa2u, 0x0eu, 0xbcu, 0x19u,
        /*
         * 4 SWCLK cycles with SWDIO low ...
         * + SWD activation code 0x1a ...
         * + at least 8 SWCLK cycles with SWDIO high
         */
        0xa0u, /**< ((0x00)      & GENMASK(3, 0)) | ((0x1a << 4) & GENMASK(7, 4)) */
        0xf1u, /**< ((0x1a >> 4) & GENMASK(3, 0)) | ((0xff << 4) & GENMASK(7, 4)) */
        0xffu,
        /* At least 50 SWCLK cycles with SWDIO high */
        0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu,
        /* At least 2 idle (low) cycles */
        0x00u
    };

    SWD_SET_SDA_OUT;
    for (uint32_t idx = 0u; idx < sizeof(swdSeqDormantToSwd); idx++)
    {
        SwdPutByte(swdSeqDormantToSwd[idx]);
    }
}

/******************************************************************************
*  SwitchJtagToDormant
***************************************************************************//**
*  Generates Dormant to SWD sequence:
*   - at least 5 cycls with SWDIOTMS in HIGH
*   - 31-bit JTAG-to-DS sequence 0x33BBBBBA transmitted LSB first.
*******************************************************************************/
static void SwitchJtagToDormant(void)
{
    static const uint8_t swdSeqJtagToSormant[] = {
        /* At least 5 TCK cycles with TMS high */
        0xffu,
        /*
         * Still one TCK cycle with TMS high followed by 31 bits JTAG-to-DS
         * select sequence 0xba, 0xbb, 0xbb, 0x33,
         */
        0x75u, /**< ((0xff >> 7) & GENMASK(0, 0)) | ((0xba << 1) & GENMASK(7, 1)) */
        0x77u, /**< ((0xba >> 7) & GENMASK(0, 0)) | ((0xbb << 1) & GENMASK(7, 1)) */
        0x77u, /**< ((0xbb >> 7) & GENMASK(0, 0)) | ((0xbb << 1) & GENMASK(7, 1)) */
        0x67u, /**< ((0xbb >> 7) & GENMASK(0, 0)) | ((0x33 << 1) & GENMASK(7, 1)) */
    };

    SWD_SET_SDA_OUT;
    for (uint32_t idx = 0u; idx < sizeof(swdSeqJtagToSormant); idx++)
    {
        SwdPutByte(swdSeqJtagToSormant[idx]);
    }
}

/******************************************************************************
*  SwitchJtagToDormantToSwd
***************************************************************************//**
*  Generates JTAG to Dormant then Dormant to SWD sequence
*******************************************************************************/
static void SwitchJtagToDormantToSwd(void)
{
    SwitchJtagToDormant();
    SwitchDormantToSwd();
}

/******************************************************************************
*  SetHandshakeFunction
***************************************************************************//**
* Set pointer to DAP handshake function
*
* @param[in]  handshakeType - handshake type
*
******************************************************************************/
static void SetHandshakeFunction(const en_dapHandshake_t handshakeType)
{
    switch(handshakeType)
    {
        case DAP_HANDSHAKE_LINERESET:
            dapHandshake = &SwdLineReset;
            break;
        case DAP_HANDSHAKE_DORMANT_SHORT:
            dapHandshake = &SwitchDormantToSwd;
            break;
        case DAP_HANDSHAKE_DORMANT_FULL:
            dapHandshake = &SwitchJtagToDormantToSwd;
            break;
        default:
            dapHandshake = &SwitchJtagToSwd;
            break;
    }
}

/******************************************************************************
*  SwdAcquireResetOrPowerCycle
***************************************************************************//**
* Invoke XRES Toggle or Power Cycle of target for initiate acquire flow
*
* @param[in]  acquireMode
*             0 - Reset
*             1 - Power Cycle.
*
* @return none.
*
******************************************************************************/
static void SwdAcquireResetOrPowerCycle(uint32_t acquireMode)
{
    if (acquireMode == ACQUIRE_RESET)
    {
        /* reset acquire */
        (void)RESET_TARGET();
    }
    else if (acquireMode == ACQUIRE_POWER_CYCLE)
    {
        /* PowerCycle mode acquire. */
        if (KitHasPowerCycleProg())
        {
            /* only MP4 supported */
            Pin_VoltageEn_Write(CMD_POWER_OFF);
            CyDelay(PWR_OFF_DELAY_US/1000u);
            Pin_VoltageEn_Write(CMD_POWER_ON);
            CyDelayUs(PWR_ON_DELAY_US);
        }
    }
    else
    {
        /* Do nothing */
    }
}

/******************************************************************************
*  SwdAcquirePSoC3
***************************************************************************//**
* Acquires PSoC 3 Devices.
*
* @param[in]  acquireMode   Reset or Power cycle acquire.
*
* @return Flag whether the PSoC 3 is acquired or not (ACQUIRE_PASS /
*         ACQUIRE_FAIL).
*
******************************************************************************/
static uint32_t SwdAcquirePSoC3(uint32_t acquireMode)
{
    /* PSoC 3 specific SWD Sequence */
    static const enum SwdWriteBlockIndex_e PSoC3Sequence [] = {
        SET_TAR_TMR_PSOC3,
        SET_DWR_TMR_PSOC3_5,
    };

    /* Acquire algorithm for PSoC 3 */

    uint8_t enableInterrupts = CyEnterCriticalSection();
    SwdAcquireResetOrPowerCycle(acquireMode);

    /* Execute DAP handshake. Reset SWD bus by default */
    (*dapHandshake)();
    uint16_t tmrVal = Timer_CSTick_ReadCounter();
    uint16_t tmrDiff;
    bool swdAcquired;
    do
    {
        /* Send debug key */
        /* 99 db 06 0c 7b */
        swdAcquired = SwdAcquirePutdataBlock(&swdWriteBlockDict[ACQUIRE_DP_PSOC3_5]);
        /* Timer_CSTick is running at 800Hz rate */
        /* It is used as a time source here instead of loop counter */
        /* This ensures we will get `real` timeout value */
        tmrDiff = tmrVal - Timer_CSTick_ReadCounter();
    }
    while ((tmrDiff < acquireTimeout) && (!swdAcquired));

    uint32_t resp;
    if (swdAcquired)
    {
        for (uint32_t index = 0u; swdAcquired && (index < (sizeof(PSoC3Sequence)/sizeof(enum SwdWriteBlockIndex_e))); index++)
        {
            uint8_t ack;
            enum SwdWriteBlockIndex_e idx = PSoC3Sequence[index];
            SwdWriteBlock(&swdWriteBlockDict[idx], &ack);
            if ((ack & SWD_ACK_BITS) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        if (swdAcquired)
        {
            /* Read RDBUFF to check AP transfer status */
            gbHeader = DP_R_RDBUFF;
            SwdGetData(gbData);

            /* Check status for the Read RDBUFF code request */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        if (!swdAcquired)
        {
            /* clear swd error flow */
            /* a. Execute SWD reset sequence */
            SwdLineReset();
            uint8_t ack;
            /* b. read SWD ID Code */
            gbHeader = DP_R_IDCODE;
            SwdGetData(gbData);
            /* c. write to ABORT DP register */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK )
            {
                SwdWriteBlock(&swdWriteBlockDict[CLR_ERRORS], &ack);
            }
        }
        resp = ((swdAcquired) ? ACQUIRE_PASS : ACQUIRE_FAIL);
    }
    else
    {
        resp = ACQUIRE_WAIT;
    }
    CyExitCriticalSection(enableInterrupts);
    return (resp);
}

/******************************************************************************
*  SwdAcquirePSoC4
***************************************************************************//**
* Acquires PSoC 4 Devices.
*
* @param[in]  acquireMode   Reset or Power cycle acquire.
*
* @return Flag whether the PSoC 4 is acquired or not (ACQUIRE_PASS /
*         ACQUIRE_FAIL).
*
******************************************************************************/
static uint32_t SwdAcquirePSoC4(uint32_t acquireMode)
{
    /* PSoC 4 specific SWD Sequence */
    static const enum SwdWriteBlockIndex_e PSoC4Sequence [] = {
        INIT_DP_CR_PSOC4,
        SEL_AP_BANK0,
        SET_CSW_DEFAULT,
        SET_TAR_TMR_PSOC4,
        SET_DWR_TMR_PSOCX
    };

    /* Acquire algorithm for PSoC 4 */

    uint8_t enableInterrupts = CyEnterCriticalSection();
    SwdAcquireResetOrPowerCycle(acquireMode);

    /* Try to acquire SWD port */
    uint16_t tmrVal = Timer_CSTick_ReadCounter();
    uint16_t tmrDiff;
    bool swdAcquired;
    do
    {
        /* Execute DAP handshake. Reset SWD bus by default */
        (*dapHandshake)();

        /* Read SWD ID code */
        gbHeader = DP_R_IDCODE;

        /* gbData contains actual ID */
        SwdGetData(gbData);
        swdAcquired = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK);

        /* Timer_CSTick is running at 800Hz rate */
        /* It is used as a time source here instead of loop counter */
        /* This ensures we will get `real` timeout value */
        tmrDiff = tmrVal - Timer_CSTick_ReadCounter();
    }
    while ((tmrDiff < acquireTimeout) && (!swdAcquired));

    uint32_t resp;
    if (swdAcquired)
    {
        /* Configure Debug Port (DAP) */
        for (uint32_t index = 0u; swdAcquired && (index < (sizeof(PSoC4Sequence)/sizeof(enum SwdWriteBlockIndex_e))); index++)
        {
            uint8_t ack;
            enum SwdWriteBlockIndex_e idx = PSoC4Sequence[index];
            SwdWriteBlock(&swdWriteBlockDict[idx], &ack);
            if ((ack & SWD_ACK_BITS) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        if (swdAcquired)
        {
            /* Read RDBUFF to check AP transfer status */
            gbHeader = DP_R_RDBUFF;
            SwdGetData(gbData);

            /* Check status for the Read RDBUFF code request */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        if (!swdAcquired)
        {
            /* clear swd error flow */
            /* a. Execute SWD reset sequence */
            SwdLineReset();
            uint8_t ack;
            /* b. read SWD ID Code */
            gbHeader = DP_R_IDCODE;
            SwdGetData(gbData);
            /* c. write to ABORT DP register */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK )
            {
                SwdWriteBlock(&swdWriteBlockDict[CLR_ERRORS], &ack);
            }
        }
        resp = ((swdAcquired) ? ACQUIRE_PASS : ACQUIRE_FAIL);
    }
    else
    {
        resp = ACQUIRE_WAIT;
    }
    CyExitCriticalSection(enableInterrupts);
    return (resp);
}

/******************************************************************************
*  SwdAcquirePSoC5
***************************************************************************//**
* Acquires PSoC 5ES2.
*
* @param[in]  acquireMode   Reset or Power cycle acquire.
*
* @return Flag whether the PSoC 5ES2 is acquired or not (ACQUIRE_PASS /
*         ACQUIRE_FAIL).
*
******************************************************************************/
static uint32_t SwdAcquirePSoC5(uint32_t acquireMode)
{
    /* See BROS (001-63649, section 4.5.1.11) for PSoC5 LP acquire sequence  */
    /* The case "Unknown NVL settings pre-factory (SWD)" is implemented here */

    uint8_t enableInterrupts = CyEnterCriticalSection();
    SwdAcquireResetOrPowerCycle(acquireMode);

    /* Execute DAP handshake. Reset SWD bus by default */
    (*dapHandshake)();
    uint16_t tmrVal = Timer_CSTick_ReadCounter();
    uint16_t tmrDiff;
    bool swdAcquired;
    do
    {
        /* Send debug key */
        /* 99 db 06 0c 7b */
        swdAcquired = SwdAcquirePutdataBlock(&swdWriteBlockDict[ACQUIRE_DP_PSOC3_5]);
        /* Timer_CSTick is running at 800Hz rate */
        /* It is used as a time source here instead of loop counter */
        /* This ensures we will get `real` timeout value */
        tmrDiff = tmrVal - Timer_CSTick_ReadCounter();
    }
    while ((tmrDiff < acquireTimeout) && (!swdAcquired));

    uint32_t resp;
    if (swdAcquired)
    {
        /* put Test Port Acquire Address */
        /* 8b 10 02 05 00  */
        enum SwdWriteBlockIndex_e idx = genTestAddrIndex[ACQUIRE_PSoC5_ES2];
        swdAcquired = SwdAcquirePutdataBlock(&swdWriteBlockDict[idx]);

        if (swdAcquired)
        {
            /* put Test Port Acquire Data */
            /* bb a9 30 7e ea */
            idx = genTestDataIndex[ACQUIRE_PSoC5_ES2];
            swdAcquired = SwdAcquirePutdataBlock(&swdWriteBlockDict[idx]);

            if (!swdAcquired)
            {
                swdAcquired = SwdAcquirePutdataBlock(&swdWriteBlockDict[idx]);
            }

            SWD_CLOCK_BIT; /* push the SWD-data into the silicon's register (latch it up) */
            SWD_CLOCK_BIT; /* issue is seen with PSoC5 LP, after Test Mode key was issued */
            SWD_CLOCK_BIT; /* but it was not actually applied without some more trailing clocks */

            if (swdAcquired)
            {
                CyDelay(1u); /* Wait at least 15 uS for cache to clear */
                SwitchJtagToSwd(); /* Perform SWD Reset */

                /* Get ID code */
                /* a5 */
                gbHeader = DP_R_IDCODE;
                SwdGetData(gbData);

                if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) != SWD_ACK )
                {
                    swdAcquired = false;
                }
            }

            if (swdAcquired)
            {
                /* Read RDBUFF to check AP transfer status */
                gbHeader = DP_R_RDBUFF;
                SwdGetData(gbData);

                /* Check status for the Read RDBUFF code request */
                if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) != SWD_ACK )
                {
                    swdAcquired = false;
                }
            }

            if (!swdAcquired)
            {
                /* clear swd error flow */
                /* a. Execute SWD reset sequence */
                SwdLineReset();
                uint8_t ack;
                /* b. read SWD ID Code */
                gbHeader = DP_R_IDCODE;
                SwdGetData(gbData);
                /* c. write to ABORT DP register */
                if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK )
                {
                    SwdWriteBlock(&swdWriteBlockDict[CLR_ERRORS], &ack);
                }
            }
        }
        resp = ((swdAcquired) ? ACQUIRE_PASS : ACQUIRE_FAIL);
    }
    else
    {
        resp = ACQUIRE_WAIT;
    }
    CyExitCriticalSection(enableInterrupts);
    return (resp);
}

/******************************************************************************
*  SwdAcquireTVII
***************************************************************************//**
* Acquires TraveoII.
*
* @param[in]  acquireMode   Reset or Power cycle acquire.
*
* @return Flag whether the TraveoII is acquired or not (ACQUIRE_PASS /
*         ACQUIRE_FAIL).
*
******************************************************************************/
static uint32_t SwdAcquireTVII(uint32_t acquireMode)
{
    /* TVII SWD Sequence */
    static const enum SwdWriteBlockIndex_e TVIISequence [] = {
        INIT_DP_CR_PSOC6_CYW20829,
        SEL_AP_BANK0,
        SET_CSW_DEFAULT,
        SET_TAR_TMR_TVII,
        SET_DWR_TMR_PSOCX
    };

    /* Acquire algorithm for TVII */

    uint8_t enableInterrupts = CyEnterCriticalSection();
    SwdAcquireResetOrPowerCycle(acquireMode);

    /* Try to acquire SWD port */
    uint16_t tmrVal = Timer_CSTick_ReadCounter();
    uint16_t tmrDiff;
    bool swdAcquired;
    do
    {
        /* Execute DAP handshake. JTAG to SWD sequence by default */
        (*dapHandshake)();

        /* Read SWD ID code */
        gbHeader = DP_R_IDCODE;

        /* gbData contains actual ID */
        SwdGetData(gbData);
        swdAcquired = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK);

        /* Timer_CSTick is running at 800Hz rate */
        /* It is used as a time source here instead of loop counter */
        /* This ensures we will get `real` timeout value */
        tmrDiff = tmrVal - Timer_CSTick_ReadCounter();
    }
    while ((tmrDiff < acquireTimeout) && (!swdAcquired));

    uint32_t resp;
    if (swdAcquired)
    {
        /* Reset SWD bus */
        SwdLineReset();

        /* Read SWD ID code */
        gbHeader = DP_R_IDCODE;
        SwdGetData(gbData);

        swdAcquired = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK );

        for (uint32_t index = 0u; swdAcquired && (index < (sizeof(TVIISequence)/sizeof(enum SwdWriteBlockIndex_e))); index++)
        {
            uint8_t ack;
            enum SwdWriteBlockIndex_e idx = TVIISequence[index];
            SwdWriteBlock(&swdWriteBlockDict[idx], &ack);
            if ((ack & SWD_ACK_BITS) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        if (swdAcquired)
        {
            /* Read RDBUFF to check AP transfer status */
            gbHeader = DP_R_RDBUFF;
            SwdGetData(gbData);

            /* Check status for the Read RDBUFF code request */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        if (!swdAcquired)
        {
            /* clear swd error flow */
            /* a. Execute SWD reset sequence */
            SwdLineReset();
            uint8_t ack;
            /* b. read SWD ID Code */
            gbHeader = DP_R_IDCODE;
            SwdGetData(gbData);
            /* c. write to ABORT DP register */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK )
            {
                SwdWriteBlock(&swdWriteBlockDict[CLR_ERRORS], &ack);
            }
        }
        resp = ((swdAcquired) ? ACQUIRE_PASS : ACQUIRE_FAIL);
    }
    else
    {
        resp = ACQUIRE_WAIT;
    }
    CyExitCriticalSection(enableInterrupts);
    return (resp);
}

/******************************************************************************
*  SwdAcquirePSoC6BLE
***************************************************************************//**
* Acquires PSoC 6 BLE.
*
* @param[in]  acquireMode   Reset or Power cycle acquire.
*
* @return Flag whether the PSoC 6 BLE is acquired or not (ACQUIRE_PASS /
*         ACQUIRE_FAIL).
*
******************************************************************************/
static uint32_t SwdAcquirePSoC6BLE(uint32_t acquireMode)
{
    /* PSoC6 specific SWD Sequence */
    static const enum SwdWriteBlockIndex_e PSoC6Sequence [] = {
        INIT_DP_CR_PSOC6_CYW20829,
        SEL_AP_BANK0,
        SET_CSW_DEFAULT,
        SET_TAR_TMR_PSOC6,
        SET_DWR_TMR_PSOCX
    };

    /* Acquire algorithm for PSoC 6 */

    uint8_t enableInterrupts = CyEnterCriticalSection();
    SwdAcquireResetOrPowerCycle(acquireMode);

    /* Try to acquire SWD port */
    uint16_t tmrVal = Timer_CSTick_ReadCounter();
    uint16_t tmrDiff;
    bool swdAcquired;
    do
    {
        /* Execute DAP handshake. JTAG to SWD sequence by default */
        (*dapHandshake)();

        /* Read SWD ID code */
        gbHeader = DP_R_IDCODE;

        /* gbData contains actual ID */
        SwdGetData(gbData);
        swdAcquired = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK);

        /* Timer_CSTick is running at 800Hz rate */
        /* It is used as a time source here instead of loop counter */
        /* This ensures we will get `real` timeout value */
        tmrDiff = tmrVal - Timer_CSTick_ReadCounter();
    }
    while ((tmrDiff < acquireTimeout) && (!swdAcquired));

    uint32_t resp;
    if (swdAcquired)
    {
        /* Reset SWD bus */
        SwdLineReset();

        /* Read SWD ID code */
        gbHeader = DP_R_IDCODE;
        SwdGetData(gbData);

        swdAcquired = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK );

        for (uint32_t index = 0u; swdAcquired && (index < (sizeof(PSoC6Sequence)/sizeof(enum SwdWriteBlockIndex_e))); index++)
        {
            uint8_t ack;
            enum SwdWriteBlockIndex_e idx = PSoC6Sequence[index];
            SwdWriteBlock(&swdWriteBlockDict[idx], &ack);
            if ((ack & SWD_ACK_BITS) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        if (swdAcquired)
        {
            /* Read RDBUFF to check AP transfer status */
            gbHeader = DP_R_RDBUFF;
            SwdGetData(gbData);

            /* Check status for the Read RDBUFF code request */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        if (!swdAcquired)
        {
            /* clear swd error flow */
            /* a. Execute SWD reset sequence */
            SwdLineReset();
            uint8_t ack;
            /* b. read SWD ID Code */
            gbHeader = DP_R_IDCODE;
            SwdGetData(gbData);
            /* c. write to ABORT DP register */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK )
            {
                SwdWriteBlock(&swdWriteBlockDict[CLR_ERRORS], &ack);
            }
        }
        resp = ((swdAcquired) ? ACQUIRE_PASS : ACQUIRE_FAIL);
    }
    else
    {
        resp = ACQUIRE_WAIT;
    }
    CyExitCriticalSection(enableInterrupts);
    return (resp);
}

/******************************************************************************
*  SwdAcquireCYW20829
***************************************************************************//**
* Acquires CYW20829 Devices.
*
* @param[in]  acquireMode   Reset or Power cycle acquire.
*
* @return Flag whether the CYW20829 is acquired or not (ACQUIRE_PASS /
*         ACQUIRE_FAIL).
*
******************************************************************************/
static uint32_t SwdAcquireCYW20829(uint32_t acquireMode)
{
    /* PSoC6 specific SWD Sequence */
    static const enum SwdWriteBlockIndex_e CYW20829Sequence [] = {
        INIT_DP_CR_PSOC6_CYW20829,
        SEL_AP_BANK0,
        SET_CSW_PROTECT,
        SET_TAR_TMR_CYW20829,
        SET_DWR_TMR_PSOCX
    };

    /* Acquire algorithm for CYW20829 */

    uint8_t enableInterrupts = CyEnterCriticalSection();
    SwdAcquireResetOrPowerCycle(acquireMode);

    /* Try to acquire SWD port */
    uint16_t tmrVal = Timer_CSTick_ReadCounter();
    uint16_t tmrDiff;
    bool swdAcquired;
    do
    {
        /* Execute DAP handshake. JTAG to SWD sequence by default */
        (*dapHandshake)();

        /* Read SWD ID code */
        gbHeader = DP_R_IDCODE;

        /* gbData contains actual ID */
        SwdGetData(gbData);
        swdAcquired = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK);

        /* Timer_CSTick is running at 800Hz rate */
        /* It is used as a time source here instead of loop counter */
        /* This ensures we will get `real` timeout value */
        tmrDiff = tmrVal - Timer_CSTick_ReadCounter();
    }
    while ((tmrDiff < acquireTimeout) && (!swdAcquired));

    uint32_t resp;
    if (swdAcquired)
    {
        /* Reset SWD bus */
        SwdLineReset();

        /* Read SWD ID code */
        gbHeader = DP_R_IDCODE;
        SwdGetData(gbData);

        swdAcquired = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK );

        for (uint32_t index = 0u; swdAcquired && (index < (sizeof(CYW20829Sequence)/sizeof(enum SwdWriteBlockIndex_e))); index++)
        {
            uint8_t ack;
            enum SwdWriteBlockIndex_e idx = CYW20829Sequence[index];
            SwdWriteBlock(&swdWriteBlockDict[idx], &ack);
            if ((ack & SWD_ACK_BITS) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        if (swdAcquired)
        {
            /* Read RDBUFF to check AP transfer status */
            gbHeader = DP_R_RDBUFF;
            SwdGetData(gbData);

            /* Check status for the Read RDBUFF code request */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        if (!swdAcquired)
        {
            /* clear swd error flow */
            /* a. Execute SWD reset sequence */
            SwdLineReset();
            uint8_t ack;
            /* b. read SWD ID Code */
            gbHeader = DP_R_IDCODE;
            SwdGetData(gbData);
            /* c. write to ABORT DP register */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK )
            {
                SwdWriteBlock(&swdWriteBlockDict[CLR_ERRORS], &ack);
            }
        }
        resp = ((swdAcquired) ? ACQUIRE_PASS : ACQUIRE_FAIL);
    }
    else
    {
        resp = ACQUIRE_WAIT;
    }
    CyExitCriticalSection(enableInterrupts);
    return (resp);
}

/******************************************************************************
*  SwdGetFamilyId
***************************************************************************//**
* Obtain Family ID of MCU from rom table on the first call and save its value in
* the global variable familyID.
* Next time only the stored value will be returned
*
* @return 12 bit Family ID
*
******************************************************************************/
static uint32_t SwdGetFamilyId(void)
{
    static const enum SwdWriteBlockIndex_e genPrepReadPIDR0Sequence [] = {
        SEL_AP_BANK0,
        SET_CSW_PROTECT
    };
    if (familyId == 0u)
    {
        /* Read BASE register */
        gbHeader = AP_R_BASE;
        SwdGetData(gbData); /* first dummy read */
        SwdGetData(gbData);

        bool familyIdValid = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK );
        /* Check status for the Read SWD ID code request*/
        if (familyIdValid)
        {
            uint32_t romBaseAddr = PACK_UINT8_TO_UINT32(gbData);
            /* check format of BASE register */
            if ((romBaseAddr & BASE_FORMAT_MASK) == BASE_FORMAT_MASK)
            {
                romBaseAddr &= BASE_ADDR_MASK;

                /* Switch back to SYS_AP */
                for (uint32_t index = 0u; familyIdValid && (index < (sizeof(genPrepReadPIDR0Sequence)/sizeof(enum SwdWriteBlockIndex_e))); index++)
                {
                    uint8_t ack;
                    enum SwdWriteBlockIndex_e idx = genPrepReadPIDR0Sequence[index];
                    SwdWriteBlock(&swdWriteBlockDict[idx], &ack);
                    if ((ack & SWD_ACK_BITS) != SWD_ACK )
                    {
                        familyIdValid = false;
                    }
                }
            }
            else
            {
                familyIdValid = false;
            }

            if (familyIdValid)
            {
                /* Prepare reading from PIDR0-PIDR3 */
                uint32_t pidr0_addr = romBaseAddr + PIDR0_ADDR;
                SwdWriteDataBlock_t dataBlock = SWD_WRITE_BLOCK(AP_W_TAR, pidr0_addr);
                uint8_t ack;
                SwdWriteBlock(&dataBlock, &ack);
                if ((ack & SWD_ACK_BITS) != SWD_ACK )
                {
                    familyIdValid = false;
                }
            }

            uint32_t periIdReg_0_3 = 0u;

            /* read PIDR0-PIDR3 */
            for (uint32_t index = 0u; familyIdValid && (index < 5u); index++)
            {
                /* Read PIDRX register data*/
                gbHeader = AP_R_DRW; /* read DRW */
                SwdGetData(gbData);
                familyIdValid = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK );
                if (familyIdValid)
                {
                    /* a 2nd dummy read must be done to get the results. */
                    if (index != 0u)
                    {
                        periIdReg_0_3 |= ((uint32_t)gbData[0]) << ((index - 1u) * 8u);
                    }
                }
                else
                {
                    familyIdValid = false;
                }
            }

            if (familyIdValid)
            {
                /* Prepare reading from PIDR4-PIDR7 */
                uint32_t pidr4_addr = romBaseAddr + PIDR4_ADDR;
                SwdWriteDataBlock_t dataBlock = SWD_WRITE_BLOCK(AP_W_TAR, pidr4_addr);
                uint8_t ack;
                SwdWriteBlock(&dataBlock, &ack);
                if ((ack & SWD_ACK_BITS) != SWD_ACK )
                {
                    familyIdValid = false;
                }
            }

            uint32_t periIdReg_4_7 = 0u;
            /* read PIDR4-PIDR7 */
            for (uint32_t index = 0; familyIdValid && (index < 5u); index++)
            {
                /* Read PIDRX register data*/
                gbHeader = AP_R_DRW; /* read DRW */
                SwdGetData(gbData);
                familyIdValid = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK );
                if (familyIdValid)
                {
                    /* a 2nd dummy read must be done to get the results. */
                    if (index != 0u)
                    {
                        periIdReg_4_7 |= ((uint32_t)gbData[0]) << ((index - 1u) * 8u);
                    }
                }
                else
                {
                    familyIdValid = false;
                }
            }

            /* validate PIDR values and extract family ID */
            if ( familyIdValid && ( (periIdReg_0_3 & PIDR_0_3_VALID_MASK) == PIDR_0_3_VALID_VAL ) && ( periIdReg_4_7 == 0u ) )
            {
                familyId = (uint16_t)(periIdReg_0_3 & FAMILY_ID_MASK);
            }
        }
    }
    else
    {
        /* Family ID is already known, no need to get it again */
    }

    return familyId;
}

/******************************************************************************
*  SwdAcquireAutodetect
***************************************************************************//**
* Acquires PSoC 4, PSoC 5, PSoC 6 and TVII.
* The MCU Family will be obtained from ROM Table
* @param[in]  acquireMode   Reset or Power cycle acquire.
*
* @return Flag whether the MCU is acquired or not (ACQUIRE_PASS /
*         ACQUIRE_FAIL).
*
******************************************************************************/
static uint32_t SwdAcquireAutodetect(uint32_t acquireMode)
{
    /* Generic SWD Sequence */
    static const enum SwdWriteBlockIndex_e genInitSequence [] = {
        INIT_DP_CR_PSOC6_CYW20829,
        SET_CSW_DEFAULT,
        SEL_AP_BANKF
};
    /* Unified acquire algorithm for PSoC devices */

    uint8_t enableInterrupts = CyEnterCriticalSection();
    SwdAcquireResetOrPowerCycle(acquireMode);

    /* Try to acquire SWD port */
    uint16_t tmrVal = Timer_CSTick_ReadCounter();
    uint16_t tmrDiff;
    bool swdAcquired;
    do
    {
        /* Execute DAP handshake. JTAG to SWD sequence by default */
        (*dapHandshake)();

        /* Read SWD ID code */
        gbHeader = DP_R_IDCODE;

        /* gbData contains actual ID */
        SwdGetData(gbData);
        swdAcquired = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK);

        /* Timer_CSTick is running at 800Hz rate */
        /* It is used as a time source here instead of loop counter */
        /* This ensures we will get `real` timeout value */
        tmrDiff = tmrVal - Timer_CSTick_ReadCounter();
    }
    while ((tmrDiff < acquireTimeout) && (!swdAcquired));

    uint32_t resp;
    if (swdAcquired)
    {
        /* Reset SWD bus */
        SwdLineReset();

        /* Read SWD ID code */
        gbHeader = DP_R_IDCODE;
        SwdGetData(gbData);

        swdAcquired = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK );

        uint32_t dpIdCode = 0u;

        if (swdAcquired)
        {
            dpIdCode = PACK_UINT8_TO_UINT32(gbData);
        }

        for (uint32_t index = 0u; swdAcquired && (index < (sizeof(genInitSequence)/sizeof(enum SwdWriteBlockIndex_e))); index++)
        {
            uint8_t ack;
            enum SwdWriteBlockIndex_e idx = genInitSequence[index];
            SwdWriteBlock(&swdWriteBlockDict[idx], &ack);
            if ((ack & SWD_ACK_BITS) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        uint8_t dut = ACQUIRE_AUTODETECT;

        /* Obtain Family ID from ROM Table */
        if (swdAcquired)
        {
            if (SwdGetFamilyId() != 0u)
            {
                /* map Family Id to dut */
                switch(familyId)
                {
                    case PSoC4_MCU:
                    case PSoC4A:
                    case PSoC4B:
                    case PSoC4C:
                    case PSoC4C_TS:
                    case PSoC4A_BLE:
                    case PSoC4A_L:
                    case PSoC4A_M:
                    case PSoC4A_BLE256:
                    case PSoC4A_DS2:
                    case PSoC4A_S1:
                    case PSoC4A_BLE256DMA:
                    case PSoC4A_S2:
                    case PSoC4B_S0:
                    case PSOC4A_BLEII:
                    case PSoC4A_S3:
                    case PSoC4A_MC:
                    case PSoC4A_S4:
                    case PSoC4A_HV_192k:
                        dut = ACQUIRE_PSoC4;
                        break;

                    case PSoC6A_BLE2:
                    case PSoC6A_2M:
                    case PSoC6A_512K:
                        dut = ACQUIRE_PSoC6_BLE;
                        break;

                    case TraveoII_B_E_1M:
                    case TraveoII_B_H_8M:
                    case TraveoII_B_E_2M:
                    case TraveoII_C_2D_6M:
                    case TraveoII_B_H_4M:
                    case TraveoII_B_E_4M:
                    case TraveoII_C_2D_4M:
                    case TraveoII_C_H_4M:
                    case TraveoII_C_E_2M:
                    case TraveoII_C_2D_6M_DDR:
                        dut = ACQUIRE_TVII;
                        break;

                    case CYW20828:
                        dut = ACQUIRE_CYW20829;
                        break;

                    default:
                        if (dpIdCode == PSOC5_DP_IDCODE)
                        {
                            /* PSoC 5 cannot be identified by rom table entries, only by DAP IDCODE */
                            dut = ACQUIRE_PSoC5_ES2;
                        }
                        break;
                }
            }
            else
            {
                swdAcquired = false;
            }
        }

        if (swdAcquired)
        {
            if (dut != ACQUIRE_AUTODETECT)
            {
                uint8_t ack;
                /* write Test Mode register address */
                enum SwdWriteBlockIndex_e idx = genTestAddrIndex[dut];
                SwdWriteBlock(&swdWriteBlockDict[idx], &ack);
                if ((ack & SWD_ACK_BITS) != SWD_ACK )
                {
                    swdAcquired = false;
                }

                if (swdAcquired)
                {
                    /* write Test Mode register value */
                    idx = genTestDataIndex[dut];
                    SwdWriteBlock(&swdWriteBlockDict[idx], &ack);
                    if ((ack & SWD_ACK_BITS) != SWD_ACK )
                    {
                        swdAcquired = false;
                    }
                }
            }
            else
            {
                swdAcquired = false;
            }
        }

        if (swdAcquired)
        {
            /* Read RDBUFF to check AP transfer status */
            gbHeader = DP_R_RDBUFF;
            SwdGetData(gbData);

            /* Check status for the Read RDBUFF code request */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) != SWD_ACK )
            {
                swdAcquired = false;
            }
        }

        if (!swdAcquired)
        {
            /* clear swd error flow */
            /* a. Execute SWD reset sequence */
            SwdLineReset();
            uint8_t ack;
            /* b. read SWD ID Code */
            gbHeader = DP_R_IDCODE;
            SwdGetData(gbData);
            /* c. write to ABORT DP register */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK )
            {
                SwdWriteBlock(&swdWriteBlockDict[CLR_ERRORS], &ack);
            }
        }
        resp = ((swdAcquired) ? ACQUIRE_PASS : ACQUIRE_FAIL);
    }
    else
    {
        resp = ACQUIRE_WAIT;
    }
    CyExitCriticalSection(enableInterrupts);
    return (resp);
}

/******************************************************************************
*  SwdAcquireCustom
***************************************************************************//**
* Acquires PSoC4, PSoC5, PSoC 6 and TVII.
* Can be used to acquire any target tht supports SWD communication
* @param[in]  acquireMode   Reset or Power cycle acquire.
* @param[in]  initSequence  Host provided Acquire Sequence.
* @param[in]  possiblyPSoC5 true/false.
*
* @return Flag whether the MCU is acquired or not (ACQUIRE_PASS /
*         ACQUIRE_FAIL).
*
******************************************************************************/
static uint32_t SwdAcquireCustom(uint32_t acquireMode, const uint8_t * initSequence, bool possiblyPSoC5)
{
    /* Acquire algorithm */

    uint8_t enableInterrupts = CyEnterCriticalSection();
    SwdAcquireResetOrPowerCycle(acquireMode);

    uint16_t tmrVal = Timer_CSTick_ReadCounter();
    uint16_t tmrDiff;
    bool swdAcquired;
    const uint8_t numOfCommands = initSequence[0];
    uint8_t commandIndex = 1u;
    uint8_t currentCommand = initSequence[commandIndex];
    uint8_t tempIndex;

    /* Execute DAP handshake. Specific for PSoC5 devices */
    if (possiblyPSoC5)
    {
        (*dapHandshake)();
    }

    /* Try to acquire SWD port */
    do
    {
        /* Execute DAP handshake. JTAG to SWD sequence by default */
        if (!possiblyPSoC5)
        {
            (*dapHandshake)();
        }

        /* For read commands execute one and move command index by 1 */
        if ((currentCommand & SWD_REQ_R) == SWD_REQ_R)
        {
            gbHeader = currentCommand;
            SwdGetData(gbData);
            swdAcquired = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK);
            tempIndex = 1u;
        }
        else
        {
            const uint8_t *pointerToReg = &initSequence[commandIndex + 1u];
            uint32_t reg = (PACK_UINT8_TO_UINT32_MSB(pointerToReg));
            SwdWriteDataBlock_t command = SWD_WRITE_BLOCK(initSequence[commandIndex], reg);
            swdAcquired = SwdAcquirePutdataBlock(&command);
            tempIndex = 5u;
        }

        /* Timer_CSTick is running at 800Hz rate */
        /* It is used as a time source here instead of loop counter */
        /* This ensures we will get `real` timeout value */
        tmrDiff = tmrVal - Timer_CSTick_ReadCounter();
    }
    while ((tmrDiff < acquireTimeout) && (!swdAcquired));

    /* move overall commandIndex */
    commandIndex += tempIndex;
    currentCommand = initSequence[commandIndex];

    uint32_t resp;
    if(swdAcquired)
    {
        /* Reset SWD bus, not applicable for PSoC5 devices */
        if (!possiblyPSoC5)
        {
            SwdLineReset();
        }

        for (uint8_t index = 1u; swdAcquired && (index < numOfCommands); index++)
        {
            if (commandIndex > MAX_OVERALL_COMMAND_INDEX)
            {
                swdAcquired = false;
                break;
            }

            /* For read commands execute one and move command index by 1 */
            if ((currentCommand & SWD_REQ_R) == SWD_REQ_R)
            {
                gbHeader = currentCommand;
                SwdGetData(gbData);
                swdAcquired = ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK);
                commandIndex ++;
            }
            else
            {
                if (commandIndex > MAX_LONG_COMMAND_INDEX)
                {
                    swdAcquired = false;
                    break;
                }

                const uint8_t *pointerToReg = &initSequence[commandIndex + 1u];
                uint32_t reg = (PACK_UINT8_TO_UINT32_MSB(pointerToReg));
                SwdWriteDataBlock_t command = SWD_WRITE_BLOCK(initSequence[commandIndex], reg);
                swdAcquired = SwdAcquirePutdataBlock(&command);

                if (swdAcquired && possiblyPSoC5 && (currentCommand == AP_W_DRW) &&
                (reg == PSOC5_TM_REG))
                {
                    CyDelay(1u);
                    SwitchJtagToSwd();
                }

                commandIndex += 5u;
            }
            currentCommand = initSequence[commandIndex];
        }

        if (!swdAcquired)
        {
            /* clear swd error flow */
            /* a. Execute SWD reset sequence */
            SwdLineReset();
            uint8_t ack;
            /* b. read SWD ID Code */
            gbHeader = DP_R_IDCODE;
            SwdGetData(gbData);
            /* c. write to ABORT DP register */
            if ((gbStatsReg & (SWD_ACK_BITS | SWD_PERR)) == SWD_ACK )
            {
                SwdWriteBlock(&swdWriteBlockDict[CLR_ERRORS], &ack);
            }
        }
        resp = ((swdAcquired) ? ACQUIRE_PASS : ACQUIRE_FAIL);
    }
    else
    {
        resp = ACQUIRE_WAIT;
    }
    CyExitCriticalSection(enableInterrupts);
    return (resp);
}

/*****************************************************************************
* Global Functions
*****************************************************************************/

/******************************************************************************
*  SetHandshakeType
***************************************************************************//**
* Store DAP Handshake type for Acquire flow
*
* @param[in] handshakeType Value of Handshake type
*
* @return    true if the type of handshake was successfully set.
*
******************************************************************************/
bool SetHandshakeType(const uint8_t handshakeType)
{
    bool result = true;
    switch (handshakeType)
    {
        case (uint8_t)DAP_HANDSHAKE_DEFAULT:
            dapHandshakeType = DAP_HANDSHAKE_DEFAULT;
            break;
        case (uint8_t)DAP_HANDSHAKE_LINERESET:
            dapHandshakeType = DAP_HANDSHAKE_LINERESET;
            break;
        case (uint8_t)DAP_HANDSHAKE_NO_DORMANT:
            dapHandshakeType = DAP_HANDSHAKE_NO_DORMANT;
            break;
        case (uint8_t)DAP_HANDSHAKE_DORMANT_SHORT:
            dapHandshakeType = DAP_HANDSHAKE_DORMANT_SHORT;
            break;
        case (uint8_t)DAP_HANDSHAKE_DORMANT_FULL:
            dapHandshakeType = DAP_HANDSHAKE_DORMANT_FULL;
            break;
        default:
            /* Other modes are not supported */
            result = false;
            break;
    }
    return result;
}


/******************************************************************************
*  Swd_SetPinsDsiConnect
***************************************************************************//**
* Selects either the DSI output signal or the data register to drive the
* TMS_SWDIO and TCLK_SWDCLK pins.
*
* @param[in]  connect  Source of control (true - DSI, false - Data register)
*
******************************************************************************/
void Swd_SetPinsDsiConnect(bool connect)
{
    /* Digital I/O Controlled Through DSI */
    static bool swdPinsDsiDriven = true;

    if (swdPinsDsiDriven != connect)
    {
        if (connect)
        {
            /* drive by DSI output signal */
            *((reg8 *)TMS_SWDIO__BYP) |= TMS_SWDIO_MASK;
            *((reg8 *)TMS_SWDIO__BIE) |= TMS_SWDIO_MASK;
            *((reg8 *)TCLK_SWDCLK__BYP) |= TCLK_SWDCLK_MASK;
        }
        else
        {
            /* drive by Data register */
            *((reg8 *)TMS_SWDIO__BYP) &= ~((uint8_t)TMS_SWDIO_MASK);
            *((reg8 *)TMS_SWDIO__BIE) &= ~((uint8_t)TMS_SWDIO_MASK);
            *((reg8 *)TCLK_SWDCLK__BYP) &= ~((uint8_t)TCLK_SWDCLK_MASK);
        }
        swdPinsDsiDriven = connect;
    }
}

/******************************************************************************
*  Swd_SetHwClock
***************************************************************************//**
* Selects maximum SWD Clock value in Hz for SWD Hardware
*
* @param[in]  curClock  Desired frequency in Hz
*
* Note: The clock source should be twice as large as SWDCLK
*
******************************************************************************/
void Swd_SetHwClock(uint32_t curClock)
{
    uint32_t desiredClock = curClock;
    if (desiredClock > MAX_SWD_CLK)
    {
        desiredClock = MAX_SWD_CLK;
    }
    uint32_t divider;
    uint32_t masterRest = MASTER_CLK % (desiredClock * 2u);
    uint32_t imoRest = IMO_CLK % (desiredClock * 2u);
    /* Find the nearest clock source */
    if (masterRest <= imoRest)
    {
        /* MASTER_CLK will be used */
        divider = MASTER_CLK / (desiredClock * 2u);
        if (masterRest != 0u)
        {
            /* increment divider to avoid over-clocking */
            divider++;
        }
        if (divider > 0u)
        {
            /* consider the +1 factor */
            divider--;
        }
        if (Clock_SWD_GetSourceRegister() != (uint8)CYCLK_SRC_SEL_PLL)
        {
            Clock_SWD_Stop();
            Clock_SWD_SetSourceRegister(CYCLK_SRC_SEL_PLL);
            Clock_SWD_SetDivider((uint16_t)divider);
            Clock_SWD_Start();
        }
        else
        {
            Clock_SWD_SetDivider((uint16_t)divider);
        }
        /* Update real clock period */
        hwClockSwdPeriod = ONEGHZ_CLK / MASTER_CLK * ( divider + 1u ) * 2u;
    }
    else
    {
        /* IMO_CLK will be used */
        divider = IMO_CLK / (desiredClock * 2u);
        if (imoRest != 0u)
        {
            /* increment divider to avoid over-clocking */
            divider++;
        }
        if (divider > 0u)
        {
            /* consider the +1 factor */
            divider--;
        }
        if (Clock_SWD_GetSourceRegister() != (uint8_t)CYCLK_SRC_SEL_IMO)
        {
            Clock_SWD_Stop();
            Clock_SWD_SetSourceRegister(CYCLK_SRC_SEL_IMO);
            Clock_SWD_SetDivider((uint16_t)divider);
            Clock_SWD_Start();
        }
        else
        {
            Clock_SWD_SetDivider((uint16_t)divider);
        }
        /* Update real clock period */
        hwClockSwdPeriod = ONEGHZ_CLK / IMO_CLK * ( divider + 1u )* 2u;
    }

    /* Update idle clock guard period */
    idleGuardPeriod = ((uint32_t)hwIdleCycles + 1u) * hwClockSwdPeriod / 1000u;
}

/******************************************************************************
*  Swd_SetHwIdleClk
***************************************************************************//**
* Set count of idle cycles after each transfer for SWD Hardware
*
* @param[in]  idleCycles  Desired count of idle cycles
*
* Note: Possible range is 1..64, the desired value will be rounded to the nearest
*       possible value
*
******************************************************************************/
void Swd_SetHwIdleClk(uint8_t idleCycles)
{
    if (idleCycles == 0u)
    {
        /* Round to the lowest possible value */
        hwIdleCycles = HW_MIN_IDLE_CYCLES;
    }
    else
    {
        if (idleCycles > HW_MAX_IDLE_CYCLES)
        {
            hwIdleCycles = HW_MAX_IDLE_CYCLES;
        }
        else
        {
            hwIdleCycles = idleCycles;
        }
    }
    SWD_IDLE_COUNT_PERIOD_REG = (hwIdleCycles * 2u) - 1u;
    idleGuardPeriod = ((uint32_t)hwIdleCycles + 1u) * hwClockSwdPeriod / 1000u;
}

/******************************************************************************
*  Swd_TransferHw
***************************************************************************//**
* Hardware accelerated SWD Transfer I/O
*
* @param[in]  request Bit field of A[3:2] RnW APnDP
* @param[in]  data    Pointer to Send/Receive 32 bit data word
*
* @return     ack     received from SWD slave ack
******************************************************************************/
uint8_t Swd_TransferHw(uint32_t request, uint32_t *data)
{
    static const uint8_t hwPreamble[16u] =
    {
        /* index is [A3 Bit, A2 Bit, RnW Bit, APnDP Bit] ==> Park Bit=1, Stop Bit=0, Parity Bit, A3 Bit, A2 Bit, RnW Bit, APnDP Bit, Start Bit=1 */
        [0x0u] = HW_PREAMBLE(0x0u), /* A3=0,A2=0,RnW=0,APnDP=0 ==> 0b0000 ==> 0b10000001*/
        [0x1u] = HW_PREAMBLE(0x1u), /* A3=0,A2=0,RnW=0,APnDP=1 ==> 0b0001 ==> 0b10100011*/
        [0x2u] = HW_PREAMBLE(0x2u), /* A3=0,A2=0,RnW=1,APnDP=0 ==> 0b0010 ==> 0b10100101*/
        [0x3u] = HW_PREAMBLE(0x3u), /* A3=0,A2=0,RnW=1,APnDP=1 ==> 0b0011 ==> 0b10000111*/
        [0x4u] = HW_PREAMBLE(0x4u), /* A3=0,A2=1,RnW=0,APnDP=0 ==> 0b0100 ==> 0b10101001*/
        [0x5u] = HW_PREAMBLE(0x5u), /* A3=0,A2=1,RnW=0,APnDP=1 ==> 0b0101 ==> 0b10001011*/
        [0x6u] = HW_PREAMBLE(0x6u), /* A3=0,A2=1,RnW=1,APnDP=0 ==> 0b0110 ==> 0b10001101*/
        [0x7u] = HW_PREAMBLE(0x7u), /* A3=0,A2=1,RnW=1,APnDP=1 ==> 0b0111 ==> 0b10101111*/
        [0x8u] = HW_PREAMBLE(0x8u), /* A3=1,A2=0,RnW=0,APnDP=0 ==> 0b1000 ==> 0b10110001*/
        [0x9u] = HW_PREAMBLE(0x9u), /* A3=1,A2=0,RnW=0,APnDP=1 ==> 0b1001 ==> 0b10010011*/
        [0xAu] = HW_PREAMBLE(0xAu), /* A3=1,A2=0,RnW=1,APnDP=0 ==> 0b1010 ==> 0b10010101*/
        [0xBu] = HW_PREAMBLE(0xBu), /* A3=1,A2=0,RnW=1,APnDP=1 ==> 0b1011 ==> 0b10110111*/
        [0xCu] = HW_PREAMBLE(0xCu), /* A3=1,A2=1,RnW=0,APnDP=0 ==> 0b1100 ==> 0b10011001*/
        [0xDu] = HW_PREAMBLE(0xDu), /* A3=1,A2=1,RnW=0,APnDP=1 ==> 0b1101 ==> 0b10111011*/
        [0xEu] = HW_PREAMBLE(0xEu), /* A3=1,A2=1,RnW=1,APnDP=0 ==> 0b1110 ==> 0b10111101*/
        [0xFu] = HW_PREAMBLE(0xFu)  /* A3=1,A2=1,RnW=1,APnDP=1 ==> 0b1111 ==> 0b10011111*/
    };

    uint8_t ack;
    uint8_t status;
    uint8_t preamble = hwPreamble[(uint8_t)(request & 0x0000000Fu)];

    SWD_PREAMBLE_WRITE_REG = preamble;

    (void)SWD_STATUS_REG; /* Clear any pending bit */

    if ((request & DAP_TRANSFER_RnW) != 0u)
    {
        /* Read transaction */
        SWD_Control_Write(READ_OP|SKIP_OP);

        do
        {
            status = SWD_STATUS_REG;
        } while (status == 0u);
        *data = SWD_DATA_READ_REG;
    }
    else
    {
        /* Write transaction */
        SWD_DATA_WRITE_REG = *data;

        SWD_Control_Write(WRITE_OP|SKIP_OP);

        do
        {
            status = SWD_STATUS_REG;
        } while (status == 0u);
    }
    ack = (uint8_t)(GET_ACK(SWD_PREAMBLE_READ_REG));
    SWD_Control_Write(NO_OP); /* Clear all */

    /* Delay for idle cycles */
    if ( idleGuardPeriod > (uint32_t)UINT16_MAX )
    {
        CyDelay(idleGuardPeriod/1000u);
    }
    else
    {
        CyDelayUs((uint16_t)idleGuardPeriod);
    }
    return ack;
}

/******************************************************************************
*  Swd_HwAccelPossibility
***************************************************************************//**
* Check HW acceleration possibility
* This function set the variable swdHwAccelerationAllowed to true when the SWD
* configuration meet with the HW implementation:
*  DAP_Data.swd_conf.turnaround  == 1U
*  DAP_Data.swd_conf.data_phase  == 0U
*
******************************************************************************/
void Swd_HwAccelPossibility(void)
{
    swdHwAccelerationAllowed = (
        (DAP_Data.swd_conf.turnaround  == 1U) &&
        (DAP_Data.swd_conf.data_phase  == 0U));
    if (swdHwAccelerationAllowed)
    {
        setSWD_Transfer_ptr(SWD_IMPL_HW);
    }
    else
    {
        if (DAP_Data.fast_clock != 0u)
        {
            setSWD_Transfer_ptr(SWD_IMPL_SW_FAST);
        }
        else
        {
            setSWD_Transfer_ptr(SWD_IMPL_SW_SLOW);
        }
    }
}

/******************************************************************************
*  Swd_Init
***************************************************************************//**
* Initializes SWD side of communication.
*
******************************************************************************/
void Swd_Init (void)
{
    gbHeader = 0u;
    gbStatsReg = SWD_DONE | SWD_ACK;
    gbAck = 0u;

    (void)memset(gbData, 0, SWD_XFER_SIZE);

    /* Disconnect SWD pins from DSI */
    Swd_SetPinsDsiConnect(false);
    /* init SWD UDB */
    Clock_SWD_Enable();

    /* Set FIFOs as single buffer */
    SWD_DATA_AUX_CTL_REG |= FIFO_32_CLEAR;
    SWD_PREAMBLE_AUX_CTL_REG |= FIFO_8_CLEAR;

    /* Set ACK Pattern */
    SWD_ACK_OK_PATTERN_REG = 0x10u;

    /* SWD_IDLE_COUNT_PERIOD_REG calculates as (DAP_Data.transfer.idle_cycles * 2u - 1u)
       and idle_cycles should be rounded to possible range 1..64 */
    hwIdleCycles = DAP_Data.transfer.idle_cycles;
    if (hwIdleCycles > 0u)
    {
        if (hwIdleCycles > HW_MAX_IDLE_CYCLES)
        {
            hwIdleCycles = HW_MAX_IDLE_CYCLES;
        }
    }
    else
    {
        hwIdleCycles = HW_MIN_IDLE_CYCLES;
    }
    SWD_IDLE_COUNT_PERIOD_REG = (hwIdleCycles * 2u) - 1u;

    /* Init counters */
    SWD_DATA_COUNT_AUX_CTL_REG |= (1u << 5u);
    SWD_ACK_COUNT_AUX_CTL_REG  |= (1u << 5u);
    SWD_PRE_COUNT_AUX_CTL_REG  |= (1u << 5u);
    SWD_PRTY_COUNT_AUX_CTL_REG |= (1u << 5u);
    SWD_IDLE_COUNT_AUX_CTL_REG |= (1u << 5u);

    SWD_Control_Write(NO_OP);
    (void)SWD_STATUS_REG; /* Clear any pending bit */

}

/******************************************************************************
*  Swd_SetApSelect
***************************************************************************//**
* Selects AP.
*
* @param[in]  ap
*
* @return  None
*
******************************************************************************/
void Swd_SetApSelect(const uint8_t ap)
{
    uint32_t swdData = ((uint32_t)ap) << 24u;
    (void)memcpy(&(swdWriteBlockDict[SEL_AP_BANK0].data), &swdData, sizeof(swdData));
    swdWriteBlockDict[SEL_AP_BANK0].dataParity = UINT32_PARITY(swdData);

    swdData |= 0x000000F0u;
    (void)memcpy(&(swdWriteBlockDict[SEL_AP_BANKF].data), &swdData, sizeof(swdData));
    swdWriteBlockDict[SEL_AP_BANKF].dataParity = UINT32_PARITY(swdData);
}

/******************************************************************************
*  Swd_Acquire
***************************************************************************//**
* Performs Acquire procedure .
*
* @param[in]  dut          Target chip to acquire.
* @param[in]  acquireMode  Acquiring mode (0 - Reset, 1 - Power Cycle)
* @param[in]  initSequence Pointer to init sequence for the custom acquiring
*
* @return  Device acquiring status
*
******************************************************************************/
uint32_t Swd_Acquire(uint32_t dut, uint32_t acquireMode, const uint8_t *initSequence)
{
    uint32_t status = ACQUIRE_FAIL;
    bool mp4PowerAcquireProtection = false;
    bool possiblePSoC5 = false;

    if (dut != ACQUIRE_CUSTOM)
    {
        (void)initSequence;
    }
    else
    {
        const uint8_t *pointerToReg = &initSequence[PSOC5_PORT_ACQ_KEY_POS];
        uint32_t reg = (PACK_UINT8_TO_UINT32_MSB(pointerToReg));
        /* Check before acquire if PSoC5 Port Acquire Key is issued */
        if ((initSequence[PSOC5_PORT_ACQ_COM_POS] == DP_W_RDBUFF) && (reg == PSOC5_PORT_ACQUIRE_KEY))
        {
           possiblePSoC5 = true;
        }
    }

    en_dapHandshake_t currentHandshake = DAP_HANDSHAKE_NO_DORMANT;
    //TODO This has to be protection that ensures we do not try to apply
    // voltage for power mode acquired, but device have own power or too big
    // capacitance applied that makes impossible to use power mode acquire.
    // Should be decoupled with power module instead of directly write to
    // voltage enable pin
    if ((acquireMode == ACQUIRE_POWER_CYCLE) && (KitIsMiniProg()))
    {
        Pin_VoltageEn_Write(0u);
        CyDelay(100u);
        if (Power_GetVoltage() > ONE_VOLT)
        {
            mp4PowerAcquireProtection = true;
        }
    }

    if (!mp4PowerAcquireProtection)
    {
        /* routines below return ACQUIRE_PASS/ ACQUIRE_FAIL or ACQUIRE_WAIT */
        switch (dut)
        {
            case ACQUIRE_PSoC4:
                currentHandshake = (dapHandshakeType == DAP_HANDSHAKE_DEFAULT) ? DAP_HANDSHAKE_LINERESET : dapHandshakeType;
                SetHandshakeFunction(currentHandshake);
                status = SwdAcquirePSoC4(acquireMode);
                break;
            case ACQUIRE_PSoC5_ES2:
                currentHandshake = (dapHandshakeType == DAP_HANDSHAKE_DEFAULT) ? DAP_HANDSHAKE_LINERESET : dapHandshakeType;
                SetHandshakeFunction(currentHandshake);
                status = SwdAcquirePSoC5(acquireMode);
                break;
            case ACQUIRE_PSoC6_BLE:
                currentHandshake = (dapHandshakeType == DAP_HANDSHAKE_DEFAULT) ? DAP_HANDSHAKE_NO_DORMANT : dapHandshakeType;
                SetHandshakeFunction(currentHandshake);
                status = SwdAcquirePSoC6BLE(acquireMode);
                break;
            case ACQUIRE_TVII:
                currentHandshake = (dapHandshakeType == DAP_HANDSHAKE_DEFAULT) ? DAP_HANDSHAKE_NO_DORMANT : dapHandshakeType;
                SetHandshakeFunction(currentHandshake);
                status = SwdAcquireTVII(acquireMode);  /* this routine already returns ACQUIRE_PASS */
                break;
            case ACQUIRE_CYW20829:
                currentHandshake = (dapHandshakeType == DAP_HANDSHAKE_DEFAULT) ? DAP_HANDSHAKE_NO_DORMANT : dapHandshakeType;
                SetHandshakeFunction(currentHandshake);
                status = SwdAcquireCYW20829(acquireMode);  /* this routine already returns ACQUIRE_PASS */
                break;
            case ACQUIRE_AUTODETECT:
                currentHandshake = (dapHandshakeType == DAP_HANDSHAKE_DEFAULT) ? DAP_HANDSHAKE_NO_DORMANT : dapHandshakeType;
                SetHandshakeFunction(currentHandshake);
                status = SwdAcquireAutodetect(acquireMode);
                break;
            case ACQUIRE_CUSTOM:
                currentHandshake = (dapHandshakeType == DAP_HANDSHAKE_DEFAULT) ? DAP_HANDSHAKE_NO_DORMANT : dapHandshakeType;
                SetHandshakeFunction(currentHandshake);
                status = SwdAcquireCustom(acquireMode, initSequence, possiblePSoC5);
                break;
            case ACQUIRE_PSoC3:
                currentHandshake = (dapHandshakeType == DAP_HANDSHAKE_DEFAULT) ? DAP_HANDSHAKE_NO_DORMANT : dapHandshakeType;
                SetHandshakeFunction(currentHandshake);
                status = SwdAcquirePSoC3(acquireMode);
                break;
            default: /* Unknown acquire method  */
                break;
        }
    }
    return status;
}
/******************************************************************************
*  Swd_SetActualAcquireTimeout
***************************************************************************//**
* Sets Timeout for acquiring SWD Target pins in proper Acquiring Mode
*
* @param[in]  acquireDUT   Target chip to acquire.
* @param[in]  acquireMode  Acquiring mode (0 - Reset, 1- Power Cycle)
*
* @return  none
*
******************************************************************************/
void Swd_SetActualAcquireTimeout(uint8_t acquireDUT, uint8_t acquireMode)
{
    /* Default value*/
    acquireTimeout = 0u;

    /* Set actual value according to DUT and PowerMode */
    switch (acquireDUT)
    {
        case ACQUIRE_PSoC4:
            switch(acquireMode)
            {
                case ACQUIRE_RESET:
                    acquireTimeout = PSOC4_AQUIRE_TIMEOUT_RESET_TICKS;
                    break;

                case ACQUIRE_POWER_CYCLE:
                    if (KitIsMiniProg())
                    {
                        /* only MP4 supported */
                        acquireTimeout = PSOC4_AQUIRE_TIMEOUT_POWER_CYCLE_TICKS;
                    }
                    else
                    {
                        acquireTimeout = PSOC4_AQUIRE_TIMEOUT_DEFAULT_TICKS;
                    }
                    break;

                default:
                    acquireTimeout = PSOC4_AQUIRE_TIMEOUT_DEFAULT_TICKS;
                    break;
            }
            perceivedTimeout = acquireTimeout;
            break;

        case ACQUIRE_PSoC5_ES2:
            switch(acquireMode)
            {
                case ACQUIRE_RESET:
                    /* reset acquire */
                    acquireTimeout = PSOC3_5_AQUIRE_TIMEOUT_RESET_TICKS;
                    break;

                case ACQUIRE_POWER_CYCLE:
                    /* PowerCycle mode acquire. */
                    if (KitIsMiniProg())
                    {
                        acquireTimeout = PSOC3_5_AQUIRE_TIMEOUT_POWER_CYCLE_TICKS;
                    }
                    else
                    {
                        /* Not supported. */
                        acquireTimeout = PSOC3_5_AQUIRE_TIMEOUT_POWER_CYCLE_UNSUPPORTED_TICKS;
                    }
                    break;

                default:
                    acquireTimeout = PSOC3_5_AQUIRE_TIMEOUT_DEFAULT_TICKS;
                    break;
            }
            perceivedTimeout = acquireTimeout;
            break;

        case ACQUIRE_CYW20829:
            acquireTimeout = CYW20829_HALF_AQUIRE_TIMEOUT_TICKS;
            perceivedTimeout = CYW20829_AQUIRE_TIMEOUT_TICKS;
            break;

        case ACQUIRE_PSoC3:
            acquireTimeout = PSOC3_5_AQUIRE_TIMEOUT_DEFAULT_TICKS;
            perceivedTimeout = acquireTimeout;
            break;
        case ACQUIRE_PSoC6_BLE:
        case ACQUIRE_TVII:
        case ACQUIRE_AUTODETECT:
        case ACQUIRE_CUSTOM:
            acquireTimeout = PSOC6_TVII_HALF_AQUIRE_TIMEOUT_TICKS;
            perceivedTimeout = PSOC6_TVII_AQUIRE_TIMEOUT_TICKS;
            break;

        default: /* Unknown acquire method  */
            break;
    }
}

/** Reset Target Device with custom specific I/O pin or command sequence.
This function allows the optional implementation of a device specific reset sequence.
It is called when the command \ref DAP_ResetTarget and is for example required
when a device needs a time-critical unlock sequence that enables the debug port.
\return 0 = no device specific reset sequence is implemented.\n
        1 = a device specific reset sequence is implemented.
*/
uint8_t RESET_TARGET(void)
{
    /* save current reset drive mode */
    uint8_t resetDriveMode = CyPins_ReadPinDriveMode(SWDXRES_0);

    /* set /XRES to low for RESET_DELAY usec*/
    CyPins_SetPinDriveMode(SWDXRES_0, SWDXRES_DM_STRONG);
    SWD_SET_XRES_LO;
    CyDelayUs(RESET_DELAY);

    /* set /XRES hi */
    SWD_SET_XRES_HI;

    /* wait until /XRES reaches logical "1", but not more than 100 ms */
    uint16_t tmrVal = Timer_CSTick_ReadCounter();
    uint16_t tmrDiff = 0u;
    bool xresAsserted;
    do
    {
        tmrDiff = tmrVal - Timer_CSTick_ReadCounter();
        xresAsserted = (SWD_GET_XRES == 0u);
    }
    while ((tmrDiff < XRES_DEASSERT_TIMEOUT_TICKS) && xresAsserted);

    /* restore original drive mode */
    CyPins_SetPinDriveMode(SWDXRES_0, resetDriveMode);

    /* "1u" indicated that device reset is implemeted */
    return (1U);
}


/* [] END OF FILE */
