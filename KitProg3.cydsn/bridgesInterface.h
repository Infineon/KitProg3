/*************************************************************************//**
* @file bridgesInterface.h
*
* @brief
*   This file contains the function prototypes, macros and constants used
*    in bridgesInterface.c file.
*
* @version KitProg3 v2.30
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

#if !defined(BRIDGESINTERFACE_H)
#define BRIDGESINTERFACE_H

#include "kitprog.h"
#include "project.h"
#include "DAP.h"
#include "DAP_vendor.h"

#define SPEED_I2C_50K                   (0x00000000u)
#define SPEED_I2C_100K                  (0x00000001u)
#define SPEED_I2C_400K                  (0x00000002u)
#define SPEED_I2C_1M                    (0x00000003u)

/* 10.666M SPI speed by default) */
#define SPEED_SPI_DEF                   (0x0061A800u)

#define I2C_COND_START                  (0x01u)
#define I2C_COND_STOP                   (0x02u)
#define I2C_COND_RESTART                (0x04u)

/* Divider values are calculated for 64MHz clock source */
/* ' - 1u' is applied due to _SetDivider function implementation */
/* I2C baud rate * 16 */
#define I2C_CLK_DIVIDER_50K             (80u - 1u)
#define I2C_CLK_DIVIDER_100K            (40u - 1u)
#define I2C_CLK_DIVIDER_400K            (10u - 1u)
#define I2C_CLK_DIVIDER_1M              (4u - 1u)

#define BRIDGE_INTERFACE_OUT_ENDP       (0x07u)
#define BRIDGE_INTERFACE_IN_ENDP        (0x06u)

#define BRIDGE_INTERFACE_ENDP_SIZE      (0x40u)

#define I2C_READ_FLAG                   (0x08u)


#define CMD_ID_SET_GET_INT_SPEED        (0x86u)
#define CMD_ID_RESTART_I2C_MSTR         (0x87u)
#define CMD_ID_I2C_TRANSACTION          (0x88u)
#define CMD_ID_SPI_DATA_TRANSFER        (0x89u)

#define CMD_I2C_TRANSFER_TYPE_MSK       (0x70u)
#define CMD_I2C_TRANSFER_TYPE_MSK_FULL  (0xF0u)

#define CMD_I2C_TRANSFER_READ_MSK       (0x08u)
#define CMD_I2C_TRANSFER_START_MSK      (0x01u)
#define CMD_I2C_TRANSFER_STOP_MSK       (0x02u)
#define CMD_I2C_TRANSFER_RESTART_MSK    (0x04u)

#define CMD_I2C_WRITE_W_S_RS            (0x01u)
#define CMD_I2C_READ_W_S_RS             (0x02u)
#define CMD_I2C_WRITE_WO_S_RS           (0x03u)
#define CMD_I2C_READ_WO_S_RS            (0x04u)


#define NACK                            (0x00u)
#define OK_ACK                          (0x01u)
#define RESTART_FAIL                    (0x00u)
#define RESTART_SUCCESS                 (0x01u)
#define TIMEOUT                         (0x32u)

#define CONTINUE                        (0x00u)
#define EXIT                            (0x01u)

#define CMD_STAT_SUCCESS                (0x00u)
#define CMD_STAT_WAIT                   (0x01u)
#define CMD_STAT_FAIL_INV_PAR           (0x81u)
#define CMD_STAT_FAIL_OP_FAIL           (0x82u)
#define CMD_TIMEOUT_SET                 (0x01u)

#define CMD_INVALID                     (0xFFu)

#define SPIM_CPHA_MASK                  (0x04u)
#define SPIM_CPOL_MASK                  (0x02u)
#define SPIM_SHIFT_DIR_MASK             (0x01u)
#define SPIM_CPHA_SHIFT                 (0x02u)
#define SPIM_CPOL_SHIFT                 (0x01u)
#define SPIM_SS_P15_3                   (0x00u)
#define SPIM_SS_P3_4                    (0x01u)
#define SPIM_SS_P3_6                    (0x02u)
#define SPIM_SS_DISABLE_ALL             (0x07u)
#define SPIM_SS_0_MSK                   (0x01u)
#define SPIM_SS_1_MSK                   (0x02u)
#define SPIM_SS_2_MSK                   (0x04u)
#define SPIM_CTRL_START                 (0x02u)
#define SPIM_CTRL_STOP                  (0x08u)

#define I2C_LINE_STAB_1_TIMEOUT         (0x25u)
#define I2C_LINE_STAB_2_TIMEOUT         (0x50u)

#define I2C_MULTIRAN_FLAG               (0x80u)
#define I2C_MULTIRAN_FLAG_SHIFTED       (0x08u)
#define BYTE_MULTR_SHIFT                (0x04u)

#define I2C_SDA_SCL_Mask                ((Pin_I2C_SCL__0__MASK) | (Pin_I2C_SDA__0__MASK))
#define PROTOCOL_CMD_OFFSET             (0x00u)
#define PROTOCOL_STATUS_OFFSET          (0x01u)
#define PROTOCOL_SETGET_OFFSET          (0x01u)
#define PROTOCOL_FLAGS_OFFSET           (0x01u)
#define PROTOCOL_INTERFACE_OFFSET       (0x02u)

#define PROTOCOL_LENGTH_OFFSET          (0x02u)
#define PROTOCOL_ADDRESS_OFFSET         (0x03u)

#define PROTOCOL_RESP_DATA_OFFSET       (0x02u)

#define PROTOCOL_SPEED_BYTE1_OFFSET     (0x02u)
#define PROTOCOL_SPEED_BYTE2_OFFSET     (0x03u)
#define PROTOCOL_SPEED_BYTE3_OFFSET     (0x04u)
#define PROTOCOL_SPEED_BYTE4_OFFSET     (0x05u)
#define PROTOCOL_MODE_OFFSET            (0x06u)

#define PROTOCOL_CPOL_FLAG              (0x02u)
#define PROTOCOL_CPHA_FLAG              (0x04u)
#define PROTOCOL_BORDER_FLAG            (0x01u)

#define BYTE_ORDER_REG_ADDR             (15u)

#define PROTOCOL_SPI_LENGTH_OFFSET      (0x01u)
#define PROTOCOL_SPI_SS_OFFSET          (0x02u)
#define PROTOCOL_SPI_CONTROL_OFFSET     (0x03u)
#define PROTOCOL_SPI_DATA_OFFSET        (0x04u)
#define PROTOCOL_SPI_CONTROL2_OFFSET    (0x07u)

#define PROTOCOL_SPEED_SET_B1_OFFSET    (0x03u)
#define PROTOCOL_SPEED_SET_B2_OFFSET    (0x04u)
#define PROTOCOL_SPEED_SET_B3_OFFSET    (0x05u)
#define PROTOCOL_SPEED_SET_B4_OFFSET    (0x06u)

#define PROTOCOL_SPEED_REPLY_B1_OFFSET  (0x02u)
#define PROTOCOL_SPEED_REPLY_B2_OFFSET  (0x03u)
#define PROTOCOL_SPEED_REPLY_B3_OFFSET  (0x04u)
#define PROTOCOL_SPEED_REPLY_B4_OFFSET  (0x05u)

#define PROTOCOL_I2C                    (0x00u)
#define PROTOCOL_SPI                    (0x01u)
#define PROTOCOL_SET                    (0x00u)
#define PROTOCOL_GET                    (0x01u)

#define PROTOCOL_DUT_OFSET              (0x01u)
#define PROTOCOL_MODE_OFSET             (0x02u)
#define PROTOCOL_N_OFSET                (0x03u)
#define PROTOCOL_CUSTOM_SEQ_OFSET       (0x04u)
#define PROTOCOL_ACQ_CMD_STAT           (0x01u)
#define PROTOCOL_ACQ_RES_OFFSET         (0x02u)

#define SPI_DELAY_AFTER_SS_SET          (50u)

#define GPIO_CLEAR_STATE                (0u)
#define GPIO_SET_STATE                  (1u)
#define GPIO_UNSUPPORTED                (0xFFu)

/* USB IN EndPoint Packet size */
#define USBINPACKETSIZE                 (64u)
/* USB OUT EndPoint Packet size */
#define USBOUTPACKETSIZE                (64u)

/* UART/SPI Source Clock Frequency */
#define SOURCECLK                       (64000000u)
#define SOURCECLK_IMO                   (24000000u)

/* USB-UART OUT EndPoint */
#define UART1_OUT_EP                     (5u)
#define UART2_OUT_EP                     (8u)

/* USB-UART IN EndPoint */
#define UART1_IN_EP                      (4u)
#define UART2_IN_EP                      (7u)

/* USB IN EndPoint Write Pointer */
#define USBUART1_INEP_WRITE_DATA_PTR    (USBFS_ARB_RW4_DR_PTR)
#define USBUART2_INEP_WRITE_DATA_PTR    (USBFS_ARB_RW7_DR_PTR)

/* USB IN EndPoint Read Pointer */
#define USBUART1_INEP_WRITE_PTR         (USBFS_ARB_RW4_WA_PTR)
#define USBUART2_INEP_WRITE_PTR         (USBFS_ARB_RW7_WA_PTR)

/* USB IN EndPoint Read MSB Pointer  */
#define USBUART1_INEP_WRITE_MSB_PTR     (USBFS_ARB_RW4_WA_MSB_PTR)
#define USBUART2_INEP_WRITE_MSB_PTR     (USBFS_ARB_RW7_WA_MSB_PTR)

/* USB IN EndPoint Count 0 Pointer */
#define USBUART1_INEP_CNT0_PTR          (USBFS_SIE_EP4_CNT0_PTR)
#define USBUART2_INEP_CNT0_PTR          (USBFS_SIE_EP7_CNT0_PTR)

/* USB IN EndPoint Count 1 Pointer */
#define USBUART1_INEP_CNT1_PTR          (USBFS_SIE_EP4_CNT1_PTR)
#define USBUART2_INEP_CNT1_PTR          (USBFS_SIE_EP7_CNT1_PTR)

/* USB IN EndPoint Write Pointer */
#define USBUART1_INEP_READ_PTR          (USBFS_ARB_RW4_RA_PTR)
#define USBUART2_INEP_READ_PTR          (USBFS_ARB_RW7_RA_PTR)

/* USB IN EndPoint Write MSB Pointer */
#define USBUART1_INEP_READ_MSB_PTR      (USBFS_ARB_RW4_RA_MSB_PTR)
#define USBUART2_INEP_READ_MSB_PTR      (USBFS_ARB_RW7_RA_MSB_PTR)

/* USB IN EndPoint Mode Register Pointer */
#define USBUART1_INEP_MODE_REG_PTR      (USBFS_SIE_EP4_CR0_PTR)
#define USBUART2_INEP_MODE_REG_PTR      (USBFS_SIE_EP7_CR0_PTR)

/* Structure for comPort */
typedef struct {
    uint8_t uartOutEp;
    uint8_t uartInEp;
    uint8_t uartHwErrorMask;
    uint8_t uartHwFifoNotEmptyMask;
    uint8_t uartSwBufferOverflowMask;
    uint8_t uartFifoLength;
    uint16_t uartTxBufferSize;
    uint8_t uartRtsPinMask;
    uint8_t (*UartReadRxStatus)(void);
    void (*UartClearRxBuffer)(void);
    void (*UartClearTxBuffer)(void);
    uint16_t (*UartGetRxBufferSize)(void);
    uint8_t (*UartReadRxData)(void);
    uint16_t (*UartGetTxBufferSize)(void);
    void (*UartPutArray)(const uint8 string[], uint16 byteCount);
    uint32_t uartRtsPinPc;
    uint32_t uartRtsPinByp;
    void (*UartClockStop)(void);
    void (*UartClockStart)(void);
    uint16_t (*UartClockGetDividerRegister)(void);
    void (*UartClockSetDividerRegister)(uint16 clkDivider, uint8 restart);
    uint8_t (*UartClockGetSourceRegister)(void);
    void (*UartClockSetSourceRegister)(uint8 clkSource);
    void (*UartStart)(void);
    void (*UartStop)(void);
} uart_bridge_t;

/* Structure for GPIO pin*/
typedef struct {
    volatile uint8_t previousState;
    volatile uint8_t currentState;
    uint8_t change;
    uint8_t pin;
    uint32_t pinReg;
} gpio_pin_t;

/* Max possible rate with IMO clock */
#define MAX_IMO_RATE              (3000000u)

/* RTS pin mode */
#define UART_RTS_N_NORMAL_MODE    (0u)
#define UART_RTS_N_FORCE_UP_MODE  (1u)

#define RTS_FLAG_NOT_SET          (0x00u)
#define RTS_FLAG_PWRON_OR_RESET   (0x01u)
#define RTS_FLAG_SWD_ACTIVE       (0x02u)
#define RTS_FLAG_USBUART_RTS      (0x04u)

/* GPIO State change transitions */
#define UNCHANGED                 (0x00u)
#define TRANSITION_LOW_HIGH       (0x01u)
#define TRANSITION_HIGH_LOW       (0x02u)

/* Function prototypes */
void Bridge_InterfaceHandler(void);
void Bridge_UartInterfaceHandler(void);

void Bridge_ExecuteCommands(void);
uint32_t Bridge_ProcessCommand(const uint8_t *request, uint8_t *response);

void Bridge_PrepareI2cInterface(void);
void Bridge_PrepareSpiInterface(void);
void Bridge_PrepareUartInterface(void);
void Bridge_PrepareGpioInterface(void);

void UsbUartStart(void);
void UartCtsRtsPinInit(void);

uint32_t Bridge_GpioSetMode(const uint8_t * request, uint8_t *response);
uint32_t Bridge_GpioSetState(const uint8_t * request, uint8_t *response);
uint32_t Bridge_GpioReadState(const uint8_t * request, uint8_t *response);
uint32_t Bridge_GpioStateChanged(const uint8_t * request, uint8_t *response);

#define isr_SWDXRES_INTERRUPT_INTERRUPT_CALLBACK
void isr_SWDXRES_Interrupt_InterruptCallback(void);
#define isr_RTSDelay_INTERRUPT_INTERRUPT_CALLBACK
void isr_RTSDelay_Interrupt_InterruptCallback(void);
#define GPIO_isr_INTERRUPT_INTERRUPT_CALLBACK
void GPIO_isr_Interrupt_InterruptCallback(void);

#endif /*BRIDGESINTERFACE_H*/


/* [] END OF FILE */
