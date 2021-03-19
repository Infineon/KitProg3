/*******************************************************************************
* File Name: `$INSTANCE_NAME`.h
* Version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
*
* Description:
*  Contains the function prototypes, constants and register definition
*  of the SPI Master Component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SPIM_`$INSTANCE_NAME`_H)
#define CY_SPIM_`$INSTANCE_NAME`_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component `$CY_COMPONENT_NAME` requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */


/***************************************
*   Conditional Compilation Parameters
***************************************/

#define `$INSTANCE_NAME`_INTERNAL_CLOCK             (`$InternalClockUsed`u)

#if(0u != `$INSTANCE_NAME`_INTERNAL_CLOCK)
    #include "`$INSTANCE_NAME`_IntClock.h"
#endif /* (0u != `$INSTANCE_NAME`_INTERNAL_CLOCK) */

#define `$INSTANCE_NAME`_MODE                       (`$Mode`u)
#define `$INSTANCE_NAME`_DATA_WIDTH                 (`$NumberOfDataBits`u)
#define `$INSTANCE_NAME`_MODE_USE_ZERO              (`$ModeUseZero`u)
#define `$INSTANCE_NAME`_BIDIRECTIONAL_MODE         (`$BidirectMode`u)

/* Internal interrupt handling */
#define `$INSTANCE_NAME`_TX_BUFFER_SIZE             (`$TxBufferSize`u)
#define `$INSTANCE_NAME`_RX_BUFFER_SIZE             (`$RxBufferSize`u)
#define `$INSTANCE_NAME`_INTERNAL_TX_INT_ENABLED    (`$InternalTxInterruptEnabled`u)
#define `$INSTANCE_NAME`_INTERNAL_RX_INT_ENABLED    (`$InternalRxInterruptEnabled`u)

#define `$INSTANCE_NAME`_SINGLE_REG_SIZE            (8u)
#define `$INSTANCE_NAME`_USE_SECOND_DATAPATH        (`$INSTANCE_NAME`_DATA_WIDTH > `$INSTANCE_NAME`_SINGLE_REG_SIZE)

#define `$INSTANCE_NAME`_FIFO_SIZE                  (4u)
#define `$INSTANCE_NAME`_TX_SOFTWARE_BUF_ENABLED    ((0u != `$INSTANCE_NAME`_INTERNAL_TX_INT_ENABLED) && \
                                                     (`$INSTANCE_NAME`_TX_BUFFER_SIZE > `$INSTANCE_NAME`_FIFO_SIZE))

#define `$INSTANCE_NAME`_RX_SOFTWARE_BUF_ENABLED    ((0u != `$INSTANCE_NAME`_INTERNAL_RX_INT_ENABLED) && \
                                                     (`$INSTANCE_NAME`_RX_BUFFER_SIZE > `$INSTANCE_NAME`_FIFO_SIZE))


/***************************************
*        Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct
{
    uint8 enableState;
    uint8 cntrPeriod;
} `$INSTANCE_NAME`_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void  `$INSTANCE_NAME`_Init(void)                           `=ReentrantKeil($INSTANCE_NAME . "_Init")`;
void  `$INSTANCE_NAME`_Enable(void)                         `=ReentrantKeil($INSTANCE_NAME . "_Enable")`;
void  `$INSTANCE_NAME`_Start(void)                          `=ReentrantKeil($INSTANCE_NAME . "_Start")`;
void  `$INSTANCE_NAME`_Stop(void)                           `=ReentrantKeil($INSTANCE_NAME . "_Stop")`;

void  `$INSTANCE_NAME`_EnableTxInt(void)                    `=ReentrantKeil($INSTANCE_NAME . "_EnableTxInt")`;
void  `$INSTANCE_NAME`_EnableRxInt(void)                    `=ReentrantKeil($INSTANCE_NAME . "_EnableRxInt")`;
void  `$INSTANCE_NAME`_DisableTxInt(void)                   `=ReentrantKeil($INSTANCE_NAME . "_DisableTxInt")`;
void  `$INSTANCE_NAME`_DisableRxInt(void)                   `=ReentrantKeil($INSTANCE_NAME . "_DisableRxInt")`;

void  `$INSTANCE_NAME`_Sleep(void)                          `=ReentrantKeil($INSTANCE_NAME . "_Sleep")`;
void  `$INSTANCE_NAME`_Wakeup(void)                         `=ReentrantKeil($INSTANCE_NAME . "_Wakeup")`;
void  `$INSTANCE_NAME`_SaveConfig(void)                     `=ReentrantKeil($INSTANCE_NAME . "_SaveConfig")`;
void  `$INSTANCE_NAME`_RestoreConfig(void)                  `=ReentrantKeil($INSTANCE_NAME . "_RestoreConfig")`;

void  `$INSTANCE_NAME`_SetTxInterruptMode(uint8 intSrc)     `=ReentrantKeil($INSTANCE_NAME . "_SetTxInterruptMode")`;
void  `$INSTANCE_NAME`_SetRxInterruptMode(uint8 intSrc)     `=ReentrantKeil($INSTANCE_NAME . "_SetRxInterruptMode")`;
uint8 `$INSTANCE_NAME`_ReadTxStatus(void)                   `=ReentrantKeil($INSTANCE_NAME . "_ReadTxStatus")`;
uint8 `$INSTANCE_NAME`_ReadRxStatus(void)                   `=ReentrantKeil($INSTANCE_NAME . "_ReadRxStatus")`;
void  `$INSTANCE_NAME`_WriteTxData(`$RegSizeReplacementString` txData)  \
                                                            `=ReentrantKeil($INSTANCE_NAME . "_WriteTxData")`;
`$RegSizeReplacementString` `$INSTANCE_NAME`_ReadRxData(void) \
                                                            `=ReentrantKeil($INSTANCE_NAME . "_ReadRxData")`;
uint8 `$INSTANCE_NAME`_GetRxBufferSize(void)                `=ReentrantKeil($INSTANCE_NAME . "_GetRxBufferSize")`;
uint8 `$INSTANCE_NAME`_GetTxBufferSize(void)                `=ReentrantKeil($INSTANCE_NAME . "_GetTxBufferSize")`;
void  `$INSTANCE_NAME`_ClearRxBuffer(void)                  `=ReentrantKeil($INSTANCE_NAME . "_ClearRxBuffer")`;
void  `$INSTANCE_NAME`_ClearTxBuffer(void)                  `=ReentrantKeil($INSTANCE_NAME . "_ClearTxBuffer")`;
void  `$INSTANCE_NAME`_ClearFIFO(void)                              `=ReentrantKeil($INSTANCE_NAME . "_ClearFIFO")`;
void  `$INSTANCE_NAME`_PutArray(const `$RegSizeReplacementString` buffer[], uint8 byteCount) \
                                                            `=ReentrantKeil($INSTANCE_NAME . "_PutArray")`;
                                                            
void  `$INSTANCE_NAME`_DynSPIModeConfig(uint8 modeCPHA, uint8 modePOL, uint8 modeSB);

#if(0u != `$INSTANCE_NAME`_BIDIRECTIONAL_MODE)
    void  `$INSTANCE_NAME`_TxEnable(void)                   `=ReentrantKeil($INSTANCE_NAME . "_TxEnable")`;
    void  `$INSTANCE_NAME`_TxDisable(void)                  `=ReentrantKeil($INSTANCE_NAME . "_TxDisable")`;
#endif /* (0u != `$INSTANCE_NAME`_BIDIRECTIONAL_MODE) */

CY_ISR_PROTO(`$INSTANCE_NAME`_TX_ISR);
CY_ISR_PROTO(`$INSTANCE_NAME`_RX_ISR);


/***************************************
*   Variable with external linkage
***************************************/

extern uint8 `$INSTANCE_NAME`_initVar;


/***************************************
*           API Constants
***************************************/

#define `$INSTANCE_NAME`_TX_ISR_NUMBER     ((uint8) (`$INSTANCE_NAME`_TxInternalInterrupt__INTC_NUMBER))
#define `$INSTANCE_NAME`_RX_ISR_NUMBER     ((uint8) (`$INSTANCE_NAME`_RxInternalInterrupt__INTC_NUMBER))

#define `$INSTANCE_NAME`_TX_ISR_PRIORITY   ((uint8) (`$INSTANCE_NAME`_TxInternalInterrupt__INTC_PRIOR_NUM))
#define `$INSTANCE_NAME`_RX_ISR_PRIORITY   ((uint8) (`$INSTANCE_NAME`_RxInternalInterrupt__INTC_PRIOR_NUM))


/***************************************
*    Initial Parameter Constants
***************************************/

#define `$INSTANCE_NAME`_INT_ON_SPI_DONE    ((uint8) (`$IntOnSPIDone`u   << `$INSTANCE_NAME`_STS_SPI_DONE_SHIFT))
#define `$INSTANCE_NAME`_INT_ON_TX_EMPTY    ((uint8) (`$IntOnTXEmpty`u   << `$INSTANCE_NAME`_STS_TX_FIFO_EMPTY_SHIFT))
#define `$INSTANCE_NAME`_INT_ON_TX_NOT_FULL ((uint8) (`$IntOnTXNotFull`u << \
                                                                           `$INSTANCE_NAME`_STS_TX_FIFO_NOT_FULL_SHIFT))
#define `$INSTANCE_NAME`_INT_ON_BYTE_COMP   ((uint8) (`$IntOnByteComp`u  << `$INSTANCE_NAME`_STS_BYTE_COMPLETE_SHIFT))
#define `$INSTANCE_NAME`_INT_ON_SPI_IDLE    ((uint8) (`$IntOnSPIIdle`u   << `$INSTANCE_NAME`_STS_SPI_IDLE_SHIFT))

/* Disable TX_NOT_FULL if software buffer is used */
#define `$INSTANCE_NAME`_INT_ON_TX_NOT_FULL_DEF ((`$INSTANCE_NAME`_TX_SOFTWARE_BUF_ENABLED) ? \
                                                                        (0u) : (`$INSTANCE_NAME`_INT_ON_TX_NOT_FULL))

/* TX interrupt mask */
#define `$INSTANCE_NAME`_TX_INIT_INTERRUPTS_MASK    (`$INSTANCE_NAME`_INT_ON_SPI_DONE  | \
                                                     `$INSTANCE_NAME`_INT_ON_TX_EMPTY  | \
                                                     `$INSTANCE_NAME`_INT_ON_TX_NOT_FULL_DEF | \
                                                     `$INSTANCE_NAME`_INT_ON_BYTE_COMP | \
                                                     `$INSTANCE_NAME`_INT_ON_SPI_IDLE)

#define `$INSTANCE_NAME`_INT_ON_RX_FULL         ((uint8) (`$IntOnRXFull`u << \
                                                                          `$INSTANCE_NAME`_STS_RX_FIFO_FULL_SHIFT))
#define `$INSTANCE_NAME`_INT_ON_RX_NOT_EMPTY    ((uint8) (`$IntOnRXNotEmpty`u << \
                                                                          `$INSTANCE_NAME`_STS_RX_FIFO_NOT_EMPTY_SHIFT))
#define `$INSTANCE_NAME`_INT_ON_RX_OVER         ((uint8) (`$IntOnRXOver`u << \
                                                                          `$INSTANCE_NAME`_STS_RX_FIFO_OVERRUN_SHIFT))

/* RX interrupt mask */
#define `$INSTANCE_NAME`_RX_INIT_INTERRUPTS_MASK    (`$INSTANCE_NAME`_INT_ON_RX_FULL      | \
                                                     `$INSTANCE_NAME`_INT_ON_RX_NOT_EMPTY | \
                                                     `$INSTANCE_NAME`_INT_ON_RX_OVER)
/* Nubmer of bits to receive/transmit */
#define `$INSTANCE_NAME`_BITCTR_INIT            (((uint8) (`$INSTANCE_NAME`_DATA_WIDTH << 1u)) - 1u)


/***************************************
*             Registers
***************************************/
#if(CY_PSOC3 || CY_PSOC5)
    #define `$INSTANCE_NAME`_TXDATA_REG (* (`$RegDefReplacementString` *) \
                                                `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__F0_REG)
    #define `$INSTANCE_NAME`_TXDATA_PTR (  (`$RegDefReplacementString` *) \
                                                `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__F0_REG)
    #define `$INSTANCE_NAME`_RXDATA_REG (* (`$RegDefReplacementString` *) \
                                                `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__F1_REG)
    #define `$INSTANCE_NAME`_RXDATA_PTR (  (`$RegDefReplacementString` *) \
                                                `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__F1_REG)
#else   /* PSOC4 */
    #if(`$INSTANCE_NAME`_USE_SECOND_DATAPATH)
        #define `$INSTANCE_NAME`_TXDATA_REG (* (reg16 *) \
                                          `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__16BIT_F0_REG)
        #define `$INSTANCE_NAME`_TXDATA_PTR (  (reg16 *) \
                                          `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__16BIT_F0_REG)
        #define `$INSTANCE_NAME`_RXDATA_REG (* (reg16 *) \
                                          `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__16BIT_F1_REG)
        #define `$INSTANCE_NAME`_RXDATA_PTR (  (reg16 *) \
                                          `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__16BIT_F1_REG)
    #else
        #define `$INSTANCE_NAME`_TXDATA_REG (* (reg8 *) \
                                                `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__F0_REG)
        #define `$INSTANCE_NAME`_TXDATA_PTR (  (reg8 *) \
                                                `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__F0_REG)
        #define `$INSTANCE_NAME`_RXDATA_REG (* (reg8 *) \
                                                `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__F1_REG)
        #define `$INSTANCE_NAME`_RXDATA_PTR (  (reg8 *) \
                                                `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__F1_REG)
    #endif /* (`$INSTANCE_NAME`_USE_SECOND_DATAPATH) */
#endif     /* (CY_PSOC3 || CY_PSOC5) */

#define `$INSTANCE_NAME`_AUX_CONTROL_DP0_REG (* (reg8 *) \
                                        `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__DP_AUX_CTL_REG)
#define `$INSTANCE_NAME`_AUX_CONTROL_DP0_PTR (  (reg8 *) \
                                        `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__DP_AUX_CTL_REG)

#if(`$INSTANCE_NAME`_USE_SECOND_DATAPATH)
    #define `$INSTANCE_NAME`_AUX_CONTROL_DP1_REG  (* (reg8 *) \
                                        `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u1__DP_AUX_CTL_REG)
    #define `$INSTANCE_NAME`_AUX_CONTROL_DP1_PTR  (  (reg8 *) \
                                        `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u1__DP_AUX_CTL_REG)
#endif /* (`$INSTANCE_NAME`_USE_SECOND_DATAPATH) */

#define `$INSTANCE_NAME`_COUNTER_PERIOD_REG     (* (reg8 *) `$INSTANCE_NAME`_BSPIM_BitCounter__PERIOD_REG)
#define `$INSTANCE_NAME`_COUNTER_PERIOD_PTR     (  (reg8 *) `$INSTANCE_NAME`_BSPIM_BitCounter__PERIOD_REG)
#define `$INSTANCE_NAME`_COUNTER_CONTROL_REG    (* (reg8 *) `$INSTANCE_NAME`_BSPIM_BitCounter__CONTROL_AUX_CTL_REG)
#define `$INSTANCE_NAME`_COUNTER_CONTROL_PTR    (  (reg8 *) `$INSTANCE_NAME`_BSPIM_BitCounter__CONTROL_AUX_CTL_REG)

#define `$INSTANCE_NAME`_TX_STATUS_REG          (* (reg8 *) `$INSTANCE_NAME`_BSPIM_TxStsReg__STATUS_REG)
#define `$INSTANCE_NAME`_TX_STATUS_PTR          (  (reg8 *) `$INSTANCE_NAME`_BSPIM_TxStsReg__STATUS_REG)
#define `$INSTANCE_NAME`_RX_STATUS_REG          (* (reg8 *) `$INSTANCE_NAME`_BSPIM_RxStsReg__STATUS_REG)
#define `$INSTANCE_NAME`_RX_STATUS_PTR          (  (reg8 *) `$INSTANCE_NAME`_BSPIM_RxStsReg__STATUS_REG)

#define `$INSTANCE_NAME`_CONTROL_REG            (* (reg8 *) \
                                      `$INSTANCE_NAME`_BSPIM_CtrlReg__CONTROL_REG)
#define `$INSTANCE_NAME`_CONTROL_PTR            (  (reg8 *) \
                                      `$INSTANCE_NAME`_BSPIM_CtrlReg__CONTROL_REG))

#define `$INSTANCE_NAME`_TX_STATUS_MASK_REG     (* (reg8 *) `$INSTANCE_NAME`_BSPIM_TxStsReg__MASK_REG)
#define `$INSTANCE_NAME`_TX_STATUS_MASK_PTR     (  (reg8 *) `$INSTANCE_NAME`_BSPIM_TxStsReg__MASK_REG)
#define `$INSTANCE_NAME`_RX_STATUS_MASK_REG     (* (reg8 *) `$INSTANCE_NAME`_BSPIM_RxStsReg__MASK_REG)
#define `$INSTANCE_NAME`_RX_STATUS_MASK_PTR     (  (reg8 *) `$INSTANCE_NAME`_BSPIM_RxStsReg__MASK_REG)

#define `$INSTANCE_NAME`_TX_STATUS_ACTL_REG     (* (reg8 *) `$INSTANCE_NAME`_BSPIM_TxStsReg__STATUS_AUX_CTL_REG)
#define `$INSTANCE_NAME`_TX_STATUS_ACTL_PTR     (  (reg8 *) `$INSTANCE_NAME`_BSPIM_TxStsReg__STATUS_AUX_CTL_REG)
#define `$INSTANCE_NAME`_RX_STATUS_ACTL_REG     (* (reg8 *) `$INSTANCE_NAME`_BSPIM_RxStsReg__STATUS_AUX_CTL_REG)
#define `$INSTANCE_NAME`_RX_STATUS_ACTL_PTR     (  (reg8 *) `$INSTANCE_NAME`_BSPIM_RxStsReg__STATUS_AUX_CTL_REG)

#define `$INSTANCE_NAME`_A0_ADDR           `$INSTANCE_NAME`_BSPIM_`$VerilogSectionReplacementString`_Dp_u0__A0_REG

#if(`$INSTANCE_NAME`_USE_SECOND_DATAPATH)
    #define `$INSTANCE_NAME`_AUX_CONTROLDP1     (`$INSTANCE_NAME`_AUX_CONTROL_DP1_REG)
#endif /* (`$INSTANCE_NAME`_USE_SECOND_DATAPATH) */


/***************************************
*       Register Constants
***************************************/

/* Status Register Definitions */
#define `$INSTANCE_NAME`_STS_SPI_DONE_SHIFT             (0x00u)
#define `$INSTANCE_NAME`_STS_TX_FIFO_EMPTY_SHIFT        (0x01u)
#define `$INSTANCE_NAME`_STS_TX_FIFO_NOT_FULL_SHIFT     (0x02u)
#define `$INSTANCE_NAME`_STS_BYTE_COMPLETE_SHIFT        (0x03u)
#define `$INSTANCE_NAME`_STS_SPI_IDLE_SHIFT             (0x04u)
#define `$INSTANCE_NAME`_STS_RX_FIFO_FULL_SHIFT         (0x04u)
#define `$INSTANCE_NAME`_STS_RX_FIFO_NOT_EMPTY_SHIFT    (0x05u)
#define `$INSTANCE_NAME`_STS_RX_FIFO_OVERRUN_SHIFT      (0x06u)

#define `$INSTANCE_NAME`_STS_SPI_DONE           ((uint8) (0x01u << `$INSTANCE_NAME`_STS_SPI_DONE_SHIFT))
#define `$INSTANCE_NAME`_STS_TX_FIFO_EMPTY      ((uint8) (0x01u << `$INSTANCE_NAME`_STS_TX_FIFO_EMPTY_SHIFT))
#define `$INSTANCE_NAME`_STS_TX_FIFO_NOT_FULL   ((uint8) (0x01u << `$INSTANCE_NAME`_STS_TX_FIFO_NOT_FULL_SHIFT))
#define `$INSTANCE_NAME`_STS_BYTE_COMPLETE      ((uint8) (0x01u << `$INSTANCE_NAME`_STS_BYTE_COMPLETE_SHIFT))
#define `$INSTANCE_NAME`_STS_SPI_IDLE           ((uint8) (0x01u << `$INSTANCE_NAME`_STS_SPI_IDLE_SHIFT))
#define `$INSTANCE_NAME`_STS_RX_FIFO_FULL       ((uint8) (0x01u << `$INSTANCE_NAME`_STS_RX_FIFO_FULL_SHIFT))
#define `$INSTANCE_NAME`_STS_RX_FIFO_NOT_EMPTY  ((uint8) (0x01u << `$INSTANCE_NAME`_STS_RX_FIFO_NOT_EMPTY_SHIFT))
#define `$INSTANCE_NAME`_STS_RX_FIFO_OVERRUN    ((uint8) (0x01u << `$INSTANCE_NAME`_STS_RX_FIFO_OVERRUN_SHIFT))

/* TX and RX masks for clear on read bits */
#define `$INSTANCE_NAME`_TX_STS_CLR_ON_RD_BYTES_MASK    (0x09u)
#define `$INSTANCE_NAME`_RX_STS_CLR_ON_RD_BYTES_MASK    (0x40u)

/* StatusI Register Interrupt Enable Control Bits */
/* As defined by the Register map for the AUX Control Register */
#define `$INSTANCE_NAME`_INT_ENABLE     (0x10u) /* Enable interrupt from statusi */
#define `$INSTANCE_NAME`_TX_FIFO_CLR    (0x01u) /* F0 - TX FIFO */
#define `$INSTANCE_NAME`_RX_FIFO_CLR    (0x02u) /* F1 - RX FIFO */
#define `$INSTANCE_NAME`_FIFO_CLR       (`$INSTANCE_NAME`_TX_FIFO_CLR | `$INSTANCE_NAME`_RX_FIFO_CLR)

/* Bit Counter (7-bit) Control Register Bit Definitions */
/* As defined by the Register map for the AUX Control Register */
#define `$INSTANCE_NAME`_CNTR_ENABLE    (0x20u) /* Enable CNT7 */

/* Bi-Directional mode control bit */
#define `$INSTANCE_NAME`_CTRL_TX_SIGNAL_EN  (0x01u)
#define `$INSTANCE_NAME`_CTRL_CPOL_SET               (0x02u)
#define `$INSTANCE_NAME`_CTRL_CPHA_SET               (0x04u)
#define `$INSTANCE_NAME`_CTRL_FIFO_SRC_ALU_SET       (0x0Cu)
#define `$INSTANCE_NAME`_CTRL_FIFO_SRC_A0_2_SET      (0x04u)
#define `$INSTANCE_NAME`_CTRL_FIFO_SRC_A0_3_SET      (0x08u)
#define `$INSTANCE_NAME`_CTRL_SHIFT_SEL_6_SET		 (0x40u)
#define `$INSTANCE_NAME`_CTRL_SHIFT_DYN_SEL_1_SET	 (0x01u)
#define `$INSTANCE_NAME`_CTRL_SHIFT_DYN_SEL_2_SET	 (0x02u)

/* Datapath Auxillary Control Register definitions */
#define `$INSTANCE_NAME`_AUX_CTRL_FIFO0_CLR         (0x01u)
#define `$INSTANCE_NAME`_AUX_CTRL_FIFO1_CLR         (0x02u)
#define `$INSTANCE_NAME`_AUX_CTRL_FIFO0_LVL         (0x04u)
#define `$INSTANCE_NAME`_AUX_CTRL_FIFO1_LVL         (0x08u)
#define `$INSTANCE_NAME`_STATUS_ACTL_INT_EN_MASK    (0x10u)

/* Component disabled */
#define `$INSTANCE_NAME`_DISABLED   (0u)


/***************************************
*       Macros
***************************************/

/* Returns true if componentn enabled */
#define `$INSTANCE_NAME`_IS_ENABLED (0u != (`$INSTANCE_NAME`_TX_STATUS_ACTL_REG & `$INSTANCE_NAME`_INT_ENABLE))

/* Retuns TX status register */
#define `$INSTANCE_NAME`_GET_STATUS_TX(swTxSts) ( (uint8)(`$INSTANCE_NAME`_TX_STATUS_REG | \
                                                          ((swTxSts) & `$INSTANCE_NAME`_TX_STS_CLR_ON_RD_BYTES_MASK)) )
/* Retuns RX status register */
#define `$INSTANCE_NAME`_GET_STATUS_RX(swRxSts) ( (uint8)(`$INSTANCE_NAME`_RX_STATUS_REG | \
                                                          ((swRxSts) & `$INSTANCE_NAME`_RX_STS_CLR_ON_RD_BYTES_MASK)) )

/* Macros for setting value of the datapath's configuration register */
#define CFG_REG(num, A0) \
 (((A0) & 0x40000000) + 0x10040 + (num)) + /* Base addr */ \
 (((A0) & 0x300) << 4) +  /* Bank */ \
 (((A0) & 0xE) << 8) + /* Pair */ \
 (((A0) & 0x1) << 7)  /* UDB */ 

/***************************************
* The following code is DEPRECATED and 
* should not be used in new projects.
***************************************/

#define `$INSTANCE_NAME`_WriteByte   `$INSTANCE_NAME`_WriteTxData
#define `$INSTANCE_NAME`_ReadByte    `$INSTANCE_NAME`_ReadRxData
void  `$INSTANCE_NAME`_SetInterruptMode(uint8 intSrc)       `=ReentrantKeil($INSTANCE_NAME . "_SetInterruptMode")`;
uint8 `$INSTANCE_NAME`_ReadStatus(void)                     `=ReentrantKeil($INSTANCE_NAME . "_ReadStatus")`;
void  `$INSTANCE_NAME`_EnableInt(void)                      `=ReentrantKeil($INSTANCE_NAME . "_EnableInt")`;
void  `$INSTANCE_NAME`_DisableInt(void)                     `=ReentrantKeil($INSTANCE_NAME . "_DisableInt")`;

#define `$INSTANCE_NAME`_TXDATA                 (`$INSTANCE_NAME`_TXDATA_REG)
#define `$INSTANCE_NAME`_RXDATA                 (`$INSTANCE_NAME`_RXDATA_REG)
#define `$INSTANCE_NAME`_AUX_CONTROLDP0         (`$INSTANCE_NAME`_AUX_CONTROL_DP0_REG)
#define `$INSTANCE_NAME`_TXBUFFERREAD           (`$INSTANCE_NAME`_txBufferRead)
#define `$INSTANCE_NAME`_TXBUFFERWRITE          (`$INSTANCE_NAME`_txBufferWrite)
#define `$INSTANCE_NAME`_RXBUFFERREAD           (`$INSTANCE_NAME`_rxBufferRead)
#define `$INSTANCE_NAME`_RXBUFFERWRITE          (`$INSTANCE_NAME`_rxBufferWrite)

#define `$INSTANCE_NAME`_COUNTER_PERIOD         (`$INSTANCE_NAME`_COUNTER_PERIOD_REG)
#define `$INSTANCE_NAME`_COUNTER_CONTROL        (`$INSTANCE_NAME`_COUNTER_CONTROL_REG)
#define `$INSTANCE_NAME`_STATUS                 (`$INSTANCE_NAME`_TX_STATUS_REG)
#define `$INSTANCE_NAME`_CONTROL                (`$INSTANCE_NAME`_CONTROL_REG)
#define `$INSTANCE_NAME`_STATUS_MASK            (`$INSTANCE_NAME`_TX_STATUS_MASK_REG)
#define `$INSTANCE_NAME`_STATUS_ACTL            (`$INSTANCE_NAME`_TX_STATUS_ACTL_REG)

#define `$INSTANCE_NAME`_INIT_INTERRUPTS_MASK  (`$INSTANCE_NAME`_INT_ON_SPI_DONE     | \
                                                `$INSTANCE_NAME`_INT_ON_TX_EMPTY     | \
                                                `$INSTANCE_NAME`_INT_ON_TX_NOT_FULL_DEF  | \
                                                `$INSTANCE_NAME`_INT_ON_RX_FULL      | \
                                                `$INSTANCE_NAME`_INT_ON_RX_NOT_EMPTY | \
                                                `$INSTANCE_NAME`_INT_ON_RX_OVER      | \
                                                `$INSTANCE_NAME`_INT_ON_BYTE_COMP)
                                                
#define `$INSTANCE_NAME`_DataWidth                  (`$INSTANCE_NAME`_DATA_WIDTH)
#define `$INSTANCE_NAME`_InternalClockUsed          (`$INSTANCE_NAME`_INTERNAL_CLOCK)
#define `$INSTANCE_NAME`_InternalTxInterruptEnabled (`$INSTANCE_NAME`_INTERNAL_TX_INT_ENABLED)
#define `$INSTANCE_NAME`_InternalRxInterruptEnabled (`$INSTANCE_NAME`_INTERNAL_RX_INT_ENABLED)
#define `$INSTANCE_NAME`_ModeUseZero                (`$INSTANCE_NAME`_MODE_USE_ZERO)
#define `$INSTANCE_NAME`_BidirectionalMode          (`$INSTANCE_NAME`_BIDIRECTIONAL_MODE)
#define `$INSTANCE_NAME`_Mode                       (`$INSTANCE_NAME`_MODE)
#define `$INSTANCE_NAME`_DATAWIDHT                  (`$INSTANCE_NAME`_DATA_WIDTH)
#define `$INSTANCE_NAME`_InternalInterruptEnabled   (`$InternalInterruptEnabled`u)

#define `$INSTANCE_NAME`_TXBUFFERSIZE   (`$INSTANCE_NAME`_TX_BUFFER_SIZE)
#define `$INSTANCE_NAME`_RXBUFFERSIZE   (`$INSTANCE_NAME`_RX_BUFFER_SIZE)

#define `$INSTANCE_NAME`_TXBUFFER       `$INSTANCE_NAME`_txBuffer
#define `$INSTANCE_NAME`_RXBUFFER       `$INSTANCE_NAME`_rxBuffer

#endif /* (CY_SPIM_`$INSTANCE_NAME`_H) */


/* [] END OF FILE */
