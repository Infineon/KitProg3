/************************************************************************//**
* @file bridgesInterface.c
*
* @brief
*  Executable code for KitProg3
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

#include "bridgesInterface.h"
#include "version.h"
#include "DAP_config.h"
#include "usbinterface.h"

#define RTS_DELAY_IN_HIGH_MS     (200u)      /* 200ms */
#define RTS_DELAY_IN_HIGH_TICK   (160u)      /* 200ms = 160 ticks at 800Hz */
#define USBUART_LINE_CONTROL_DTR (0x0001u)
#define USBUART_LINE_CONTROL_RTS (0x0002u)

typedef enum
{
    STATE_I2C_READY = 0x00u,
    STATE_I2C_RW_STARTED
}stateI2cTransfer;

typedef enum
{
    STARTED = 0u,
    FINISHED,
    NOTHING_TO_HANDLE
}stateExecuteCommand;

static stateExecuteCommand commandExecution = NOTHING_TO_HANDLE;
static bool initCommand = false;
static uint8_t cmdCount = 0u;

static uint8_t i2cActSpeed = SPEED_I2C_1M;
static uint32_t spiActSpeed = SPEED_SPI_DEF;
static uint8_t bytesWritten = 0u;
static bool startedWr = false;
static bool addrWasSent = false;
static bool transferNack = false;
static stateI2cTransfer stateI2cOp = STATE_I2C_READY;

/* Request and Response array for Bridge Interface */
static  uint8_t bridgeRequest[BRIDGE_INTERFACE_ENDP_SIZE];
static  uint8_t bridgeResponse[BRIDGE_INTERFACE_ENDP_SIZE];

static uint8_t requestIndex;
static uint8_t responseIndex;

static uint16_t waitResponseTimer = 0u;

static uint8_t bufferI2c[64u];

static volatile gpio_pin_t gpioState[2u] = {
    { /* GPIO Pin HWID 0x0D Port 3 Pin 5 */
        .previousState = 0x00u,
        .currentState = 0x00u,
        .change = UNCHANGED,
        .pin = 0x35u,
        .pinReg = GPIO_GPIO_0,
    },
    { /* GPIO Pin HWID 0x0D Port 3 Pin 6 */
        .previousState = 0x00u,
        .currentState = 0x00u,
        .change = UNCHANGED,
        .pin = 0x36u,
        .pinReg = GPIO_SPI_SS_2_GPIO_1,
    },
};
volatile bool gpioChanged = false;
static const uint8_t waitResponse[DAP_PACKET_SIZE] = { [0] = ID_DAP_Vendor8, [1] = CMD_STAT_WAIT };

static void NoUartIndication(uint8_t value);
void (*WicedUartHciLed)(uint8_t value) = &NoUartIndication;
void (*WicedUartPeriLed)(uint8_t value) = &NoUartIndication;

static const uart_bridge_t uart[2u] = {
    {
    .uartOutEp = UART1_OUT_EP,
    .uartInEp = UART1_IN_EP,
    .uartHwErrorMask = (UART_Bridge_RX_STS_BREAK | UART_Bridge_RX_STS_PAR_ERROR | UART_Bridge_RX_STS_STOP_ERROR | UART_Bridge_RX_STS_OVERRUN),
    .uartHwFifoNotEmptyMask = UART_Bridge_RX_STS_FIFO_NOTEMPTY,
    .uartSwBufferOverflowMask = UART_Bridge_RX_STS_SOFT_BUFF_OVER,
    .uartFifoLength = UART_Bridge_FIFO_LENGTH,
    .uartTxBufferSize = UART_Bridge_TX_BUFFER_SIZE,
    .uartRtsPinMask = UART_RTS_MASK,
    .uartRtsPinPc = UART_RTS_0,
    .uartRtsPinByp = UART_RTS__BYP,
    .UartReadRxStatus = &UART_Bridge_ReadRxStatus,
    .UartClearRxBuffer = &UART_Bridge_ClearRxBuffer,
    .UartClearTxBuffer = &UART_Bridge_ClearTxBuffer,
    .UartGetRxBufferSize = &UART_Bridge_GetRxBufferSize,
    .UartReadRxData = &UART_Bridge_ReadRxData,
    .UartGetTxBufferSize = &UART_Bridge_GetTxBufferSize,
    .UartPutArray = &UART_Bridge_PutArray,
    .UartClockGetDividerRegister = &Clock_UART_GetDividerRegister,
    .UartClockSetDividerRegister = &Clock_UART_SetDividerRegister,
    .UartClockGetSourceRegister = &Clock_UART_GetSourceRegister,
    .UartClockSetSourceRegister = &Clock_UART_SetSourceRegister,
    .UartClockStart = &Clock_UART_Start,
    .UartClockStop = &Clock_UART_Stop,
    .UartStart = &UART_Bridge_Start,
    .UartStop = &UART_Bridge_Stop,
    .UartLed = &WicedUartHciLed
    },
    {
    .uartOutEp = UART2_OUT_EP,
    .uartInEp = UART2_IN_EP,
    .uartHwErrorMask = (UART_Bridge_2_RX_STS_BREAK | UART_Bridge_2_RX_STS_PAR_ERROR | UART_Bridge_2_RX_STS_STOP_ERROR | UART_Bridge_2_RX_STS_OVERRUN),
    .uartHwFifoNotEmptyMask = UART_Bridge_2_RX_STS_FIFO_NOTEMPTY,
    .uartSwBufferOverflowMask = UART_Bridge_2_RX_STS_SOFT_BUFF_OVER,
    .uartFifoLength = UART_Bridge_2_FIFO_LENGTH,
    .uartTxBufferSize = UART_Bridge_2_TX_BUFFER_SIZE,
    .uartRtsPinMask = UART_RTS_2_MASK,
    .uartRtsPinPc = UART_RTS_2_0,
    .uartRtsPinByp = UART_RTS_2__BYP,
    .UartReadRxStatus = &UART_Bridge_2_ReadRxStatus,
    .UartClearRxBuffer = &UART_Bridge_2_ClearRxBuffer,
    .UartClearTxBuffer = &UART_Bridge_2_ClearTxBuffer,
    .UartGetRxBufferSize = &UART_Bridge_2_GetRxBufferSize,
    .UartReadRxData = &UART_Bridge_2_ReadRxData,
    .UartGetTxBufferSize = &UART_Bridge_2_GetTxBufferSize,
    .UartPutArray = &UART_Bridge_2_PutArray,
    .UartClockGetDividerRegister = &Clock_UART_2_GetDividerRegister,
    .UartClockSetDividerRegister = &Clock_UART_2_SetDividerRegister,
    .UartClockGetSourceRegister = &Clock_UART_2_GetSourceRegister,
    .UartClockSetSourceRegister = &Clock_UART_2_SetSourceRegister,
    .UartClockStart = &Clock_UART_2_Start,
    .UartClockStop = &Clock_UART_2_Stop,
    .UartStart = &UART_Bridge_2_Start,
    .UartStop = &UART_Bridge_2_Stop,
    .UartLed = &WicedUartPeriLed,
    }
};

static uint32_t uartLastRate[2u] = {0u, 0u}; /* Last configured baud rate per UART */
/******************************************************************************
*  I2c_DsiBypassDisable
***************************************************************************//**
* Disable DSI bypass and let the Port logic data register drives the
* corresponding port pin.
*
*******************************************************************************/
static void I2c_DsiBypassDisable(void)
{
    /* Let the Port logic data register drives the corresponding port pin. */
    Pin_I2C_SDA_BYP = (uint8_t)(Pin_I2C_SDA_BYP & ~ (Pin_I2C_SDA_MASK | Pin_I2C_SCL_MASK));
}

/******************************************************************************
*  I2c_DsiBypassEnable
***************************************************************************//**
* Enable DSI bypass and  Let digital system interconnect (DSI) drive the
* corresponding port pin
*
*******************************************************************************/
static void I2c_DsiBypassEnable(void)
{
    /* Let digital system interconnect (DSI) drive the corresponding port pin. */
    Pin_I2C_SDA_BYP = Pin_I2C_SDA_BYP | (Pin_I2C_SDA_MASK | Pin_I2C_SCL_MASK);
}

/*******************************************************************************
*  I2c_Restart
***************************************************************************//**
* Restart try to free I2C bus from hanging devices
*
* @return True, if success
*
*******************************************************************************/
static bool I2c_Restart(void)
{
    uint8_t count;
    bool status;

    /* Stop the I2C hardware */
    I2C_UDB_Stop();

    /* Allow the DSI to drive the I2C SDA and SCL pins */
    I2c_DsiBypassDisable();

    /* Make sure the SDA line is not pulled low by master */
    CyPins_SetPin(Pin_I2C_SDA_0);

    /* send clock signal on SCL and wait for SDA to turn free */
    CyPins_ClearPin(Pin_I2C_SCL_0);
    for (count = 0u; count < TIMEOUT; count++)
    {
        if(CyPins_ReadPin(Pin_I2C_SDA_0) != 0u)
        {
            /* break loop if SDA released by slave */
            break;
        }

        /* clock on the SCL line */
        CyPins_SetPin(Pin_I2C_SCL_0);
        CyDelayUs(I2C_LINE_STAB_1_TIMEOUT);
        CyPins_ClearPin(Pin_I2C_SCL_0);
        CyDelayUs(I2C_LINE_STAB_1_TIMEOUT);
    }

    /* Generate stop condition */
    CyPins_ClearPin(Pin_I2C_SDA_0);
    CyPins_SetPin(Pin_I2C_SCL_0);
    CyDelayUs(I2C_LINE_STAB_2_TIMEOUT);
    CyPins_SetPin(Pin_I2C_SDA_0);

    /* wait for stable signal */
    CyDelayUs(I2C_LINE_STAB_2_TIMEOUT);

    /* If SCL and SCL lines free, return SUCCESS */
    if ((Pin_I2C_SDA_DR & I2C_SDA_SCL_Mask) == I2C_SDA_SCL_Mask)
    {
        status = true;
    }
    else
    {
        status = false;
    }

    /* Transfer the control back to the port control registers */
    I2c_DsiBypassEnable();

    /* Start the I2C hardware */
    I2C_UDB_Start();

    return (status);
}

/******************************************************************************
*  I2c_OpNonBlocking
***************************************************************************//**
* Starts i2c transaction (read or write) in non-blocking way. Or continue
* previously started transaction (no start/restart present in flags).
*
* @param[in] flags          Flags of the requested transaction
*    Bits:   0       1       2       3   4   5   6   7   8
*    Flags: start   stop    restart                     r/o
* @param[in]  length        Length of the data to be transmitted.
* @param[in]  commandData   The pointer to the request string.
*
* @return                   The result of starting i2c operation.
*                           0   - success
*                           !=0 - fail
******************************************************************************/
static uint8_t I2c_OpNonBlocking(uint8_t flags, uint8_t length, const uint8_t *commandData)
{
    uint8_t mode = 0u;
    uint8_t retVal = 0u;

    if ((flags & I2C_COND_STOP) == 0u)
    {
        mode |= I2C_UDB_MODE_NO_STOP;
    }

    if ((flags & (I2C_COND_START | I2C_COND_RESTART)) != 0u)
    {
        if ((Pin_I2C_SDA_DR & I2C_SDA_SCL_Mask) == I2C_SDA_SCL_Mask)
        {
            /* Send a stop condition */
            (void)I2C_UDB_MasterSendStop();

            /* For I2C lines to retain default levels and for I2C_UDB to complete transaction
               per Side Effects section of I2C_UDB_MasterSendStop() function. */
            CyDelayUs(10u);
            if(!I2c_Restart())
            {
                /* Indicate a nACK */
                retVal = 1u;
            }
        }
        transferNack = false;
    }
    else if (transferNack == true)
    {
        retVal = 1u;
    }
    else
    {
        /*Impossible case*/
    }

    if (retVal != 1u)
    {
        uint8_t exitFlag = 0;
        if (((flags & I2C_COND_START) == 0u) && ((flags & I2C_COND_RESTART) != 0x00u))
        {
            mode |= I2C_UDB_MODE_REPEAT_START;
        }
        else if ((flags & (I2C_COND_START | I2C_COND_RESTART)) == 0u)
        {
            addrWasSent = false;
            exitFlag = 1u;
            if ((flags & I2C_READ_FLAG) != 0x00u)
            {
                /* Prepare read buffer */
                I2C_UDB_DisableInt(); /* Lock from interrupt */
                I2C_UDB_MasterClearReadBuf();

                (void)memset(bufferI2c, 0x00, sizeof(bufferI2c));
                I2C_UDB_mstrRdBufPtr = (volatile uint8_t *)bufferI2c;

                I2C_UDB_ClearPendingInt();
                I2C_UDB_mstrStatus &= (uint8_t) ~I2C_UDB_MSTAT_XFER_HALT;
                I2C_UDB_mstrStatus &= (uint8_t) ~I2C_UDB_MSTAT_ERR_SHORT_XFER;

                /* Set end of transaction flag: Stop or Halt (following ReStart) */
                I2C_UDB_mstrControl = mode;

                /* Clear read status history */
                I2C_UDB_mstrStatus &= (uint8_t) ~I2C_UDB_MSTAT_RD_CMPLT;
                I2C_UDB_EnableInt();
                I2C_UDB_MasterClearReadBuf();
                (void)I2C_UDB_MasterClearStatus();

                bufferI2c[0] = I2C_UDB_DATA_REG;

                I2C_UDB_mstrRdBufIndex = 1u;
                /* Update size per updated index and 1st byte read */
                I2C_UDB_mstrRdBufSize   = length + 1u;
                I2C_UDB_state = I2C_UDB_SM_MSTR_RD_DATA;
                I2C_UDB_EnableInt();
                I2C_UDB_ACK_AND_RECEIVE;
                startedWr = false;
            }
            else
            {
                /* Prepare write buffer */
                I2C_UDB_DisableInt(); /* Lock from interrupt */
                (void)memset(bufferI2c, 0x00, sizeof(bufferI2c));
                (void)memcpy((void *)bufferI2c, (const void *)&commandData[0], length);
                I2C_UDB_DATA_REG   = bufferI2c[0];
                I2C_UDB_mstrWrBufPtr   = (volatile uint8_t *)bufferI2c;
                I2C_UDB_ClearPendingInt();
                I2C_UDB_mstrStatus &= (uint8_t) ~I2C_UDB_MSTAT_XFER_HALT;
                I2C_UDB_mstrStatus &= (uint8_t) ~I2C_UDB_MSTAT_ERR_SHORT_XFER;
                /* Set end of transaction flag: Stop or Halt (following ReStart) */
                I2C_UDB_mstrControl = mode;
                /* Clear write status history */
                I2C_UDB_mstrStatus &= (uint8_t) ~I2C_UDB_MSTAT_WR_CMPLT;
                I2C_UDB_EnableInt();
                bytesWritten = length;
                I2C_UDB_MasterClearWriteBuf();
                (void)I2C_UDB_MasterClearStatus();
                I2C_UDB_mstrWrBufIndex = 1u;
                I2C_UDB_mstrWrBufSize  = length;
                I2C_UDB_state = I2C_UDB_SM_MSTR_WR_DATA;
                I2C_UDB_EnableInt(); /* Release lock */
                I2C_UDB_TRANSMIT_DATA;
                startedWr = true;
            }
        }
        else
        {
            /*Immpossible case*/
        }

        if (exitFlag != 1u)
        {
            (void)I2C_UDB_MasterClearStatus();
            if ((flags & I2C_READ_FLAG) != 0x00u)
            {
                /* No need to store any data */
                (void)memset(bufferI2c, 0x00, sizeof(bufferI2c));
                retVal = I2C_UDB_MasterReadBuf(commandData[0u], (uint8_t *)bufferI2c, length, mode);
                startedWr = false;
            }
            else
            {
                (void)memset(bufferI2c, 0x00, sizeof(bufferI2c));
                (void)memcpy((void *)bufferI2c, (const void *)&commandData[1u], length);
                bytesWritten = length;
                retVal = I2C_UDB_MasterWriteBuf(commandData[0u], (uint8_t *)bufferI2c, length, mode);
                startedWr = true;
            }
            addrWasSent = true;
        }
    }

    return(retVal);
}


/******************************************************************************
*  I2c_GenericTransactionContinue
***************************************************************************//**
* Should be called after calling i2cOpNonBlocking() to catch it's completion.
* This function modifies global variable stateI2COp, when it is set to
* STATE_I2C_READY a new i2c operation can be started.
*
******************************************************************************/
static uint32_t I2c_GenericTransactionContinue(const uint8_t *request, uint8_t *response )
{
    (void)request;
    uint32_t num = 0u;
    uint8_t status = I2C_UDB_MasterStatus();
    uint8_t currentI2cUdbState = I2C_UDB_state;

    if ((currentI2cUdbState == I2C_UDB_SM_MSTR_HALT) || (currentI2cUdbState == I2C_UDB_SM_IDLE))
    {
        if (((status & I2C_UDB_MSTAT_XFER_INP) != 0u) || ((status & I2C_UDB_MSTAT_RD_CMPLT) != 0u) || ((status & I2C_UDB_MSTAT_WR_CMPLT) != 0u))
        {
            if ((status & I2C_UDB_MSTAT_ERR_MASK) == 0u)
            {
                if (startedWr == true)
                {
                    response[PROTOCOL_CMD_OFFSET] = ID_DAP_Vendor8;
                    response[PROTOCOL_STATUS_OFFSET] = DAP_OK;
                    response[PROTOCOL_RESP_DATA_OFFSET] = 0x01u;
                    num += 3u;

                    uint8_t cachedMstrWrBufIndex = I2C_UDB_mstrWrBufIndex;
                    for( uint8_t index = 0u; index < cachedMstrWrBufIndex; index++)
                    {
                        response[PROTOCOL_ADDRESS_OFFSET + index] = 0x01u;
                        num++;
                    }

                    if (I2C_UDB_mstrWrBufIndex != bytesWritten)
                    {
                        transferNack = true;
                    }
                    I2C_UDB_mstrWrBufPtr   = NULL;

                    (void)I2C_UDB_MasterClearStatus();
                    stateI2cOp = STATE_I2C_READY;
                }
                else
                {
                    response[PROTOCOL_CMD_OFFSET] = ID_DAP_Vendor8;
                    response[PROTOCOL_STATUS_OFFSET] = DAP_OK;
                    num += 2u;

                    I2C_UDB_mstrRdBufPtr = NULL;
                    uint8_t readLength = I2C_UDB_mstrRdBufIndex;

                    if (addrWasSent == true)
                    {
                        response[PROTOCOL_RESP_DATA_OFFSET] = 0x01u;
                        num++;
                        /* three bytes less than endpoint are CMD_CODE, CMD_STATUS and Address ACK/NACK status. */
                        for(uint8_t index = 0u; index < readLength; index++)
                        {
                            response[3u + index] = bufferI2c[index];
                        }
                        num += readLength;
                    }
                    else
                    {
                        for(uint8_t index = 0u; index < readLength; index++)
                        {
                            response[2u + index] = bufferI2c[1u + index];
                        }
                        num += readLength;

                    }
                    (void)I2C_UDB_MasterClearStatus();
                    stateI2cOp = STATE_I2C_READY;
                }
            }
            else if ((status & I2C_UDB_MSTAT_ERR_XFER) != 0u)
            {
                transferNack = true;
                if (startedWr == true)
                {
                    response[PROTOCOL_CMD_OFFSET] = ID_DAP_Vendor8;
                    response[PROTOCOL_STATUS_OFFSET] = DAP_OK;
                    response[PROTOCOL_RESP_DATA_OFFSET] = 0x00;
                    num += 3U;

                    I2C_UDB_mstrWrBufPtr = NULL;

                    stateI2cOp = STATE_I2C_READY;
                }
                else
                {
                    response[PROTOCOL_CMD_OFFSET] = ID_DAP_Vendor8;
                    response[PROTOCOL_STATUS_OFFSET] = DAP_OK;
                    num += 2u;

                    if (addrWasSent == true)
                    {
                        response[PROTOCOL_RESP_DATA_OFFSET] = 0x00u;
                        num++;
                    }
                    I2C_UDB_mstrRdBufPtr = NULL;

                    (void)I2C_UDB_MasterClearStatus();
                    stateI2cOp = STATE_I2C_READY;
                }
            }
            else
            {
                /* Impossible case */
            }
        }
    }
    return num;
}

/******************************************************************************
*  I2c_CheckForResponse
***************************************************************************//**
* Should be called after calling i2cOpNonBlocking() to catch it's completion.
* This function modifies global variable stateI2COp, when it is set to
* STATE_I2C_READY a new i2c operation can be started.
*
******************************************************************************/
static void I2c_CheckForResponse(const uint8_t *request, uint8_t *response)
{
    uint32_t num = I2c_GenericTransactionContinue(request, response);

    if (num != 0u)
    {
        /* move response index as execution of command is finished */
        if (cmdCount != 0u)
        {
            cmdCount--;
            responseIndex += (uint8_t)num;
        }

        /* if it was the last command in the execute_commands request or one command in request finish it */
        if (cmdCount == 0u)
        {
            commandExecution = FINISHED;
        }
    }

    if (USBFS_GetEPState(BRIDGE_INTERFACE_OUT_ENDP) == USBFS_OUT_BUFFER_FULL)
    {
        uint16_t receiveSize = USBFS_GetEPCount(BRIDGE_INTERFACE_OUT_ENDP);

        if (receiveSize > 0u)
        {
            uint8_t immBridgeRequest[BRIDGE_INTERFACE_ENDP_SIZE];
            uint8_t immBridgeResponse[BRIDGE_INTERFACE_ENDP_SIZE];

            uint32_t intrMask = CyUsbIntDisable();
            (void)USBFS_ReadOutEP(BRIDGE_INTERFACE_OUT_ENDP, immBridgeRequest, receiveSize);
            CyUsbIntEnable(intrMask);

            /* Only valid command during active I2C transaction is I2C master restart that
             * allows to interrupt current operation and restart I2C master */
            if (immBridgeRequest[PROTOCOL_CMD_OFFSET] == CMD_ID_RESTART_I2C_MSTR)
            {
                immBridgeResponse[PROTOCOL_CMD_OFFSET] = immBridgeRequest[PROTOCOL_CMD_OFFSET];
                //if restart clear buffers, move indexes to 0, clear i2cCount
                /* restart HW and record response */
                immBridgeResponse[PROTOCOL_STATUS_OFFSET] = (I2c_Restart() ? CMD_STAT_SUCCESS : CMD_STAT_FAIL_OP_FAIL);

                commandExecution = NOTHING_TO_HANDLE;
                stateI2cOp = STATE_I2C_READY;
                cmdCount = 0u;
            }
            else
            {
                immBridgeResponse[PROTOCOL_CMD_OFFSET] = CMD_INVALID;
            }

            intrMask = CyUsbIntDisable();
            USBFS_LoadInEP(BRIDGE_INTERFACE_IN_ENDP, immBridgeResponse, BRIDGE_INTERFACE_ENDP_SIZE);
            CyUsbIntEnable(intrMask);
        }
    }
 }

/******************************************************************************
*  I2c_ParseCommand
***************************************************************************//**
* Determines type of I2C request.
*
* @param[in] flags The pointer to the variable for I2C transaction to be set.
* At the start it is expected for it to contain not decoded data.
*
* @param[in] request The pointer to the memory that contains original host request
*
* @return True, if the request was decoded.
*
******************************************************************************/
static bool I2c_ParseCommand(uint8_t *flags, const uint8_t *request)
{
    bool retVal = false;

    switch (*flags)
    {
    case CMD_I2C_WRITE_W_S_RS:
        if ((request[PROTOCOL_FLAGS_OFFSET] & CMD_I2C_TRANSFER_START_MSK) == 0u)
        {
            *flags = CMD_I2C_TRANSFER_START_MSK;
        }
        else
        {
            *flags = CMD_I2C_TRANSFER_RESTART_MSK;
        }

        if ((request[PROTOCOL_FLAGS_OFFSET] & CMD_I2C_TRANSFER_STOP_MSK) == 0u)
        {
            /* No flags to be set in this case. */
        }
        else
        {
            *flags |= CMD_I2C_TRANSFER_STOP_MSK;
        }
        retVal = true;
        break;

    case CMD_I2C_READ_W_S_RS:
        /* Do not allow zero-length read requests */
        if (request[PROTOCOL_LENGTH_OFFSET] != 0u)
        {
            if ((request[PROTOCOL_FLAGS_OFFSET] & CMD_I2C_TRANSFER_START_MSK) == 0u)
            {
                *flags = CMD_I2C_TRANSFER_START_MSK;
            }
            else
            {
                *flags = CMD_I2C_TRANSFER_RESTART_MSK;
            }

            if ((request[PROTOCOL_FLAGS_OFFSET] & CMD_I2C_TRANSFER_STOP_MSK) == 0u)
            {
                /* No flags to be set in this case. */
            }
            else
            {
                *flags |= CMD_I2C_TRANSFER_STOP_MSK;
            }

            *flags |= CMD_I2C_TRANSFER_READ_MSK;
            retVal = true;
        }
        else
        {
            retVal = false;
        }

        break;

    case CMD_I2C_WRITE_WO_S_RS:
        if ((request[PROTOCOL_FLAGS_OFFSET] & I2C_READ_FLAG) != 0u)
        {
            *flags = CMD_I2C_TRANSFER_STOP_MSK;
        }
        else
        {
            *flags = 0u;
        }
        retVal = true;

        break;

    case CMD_I2C_READ_WO_S_RS:
        /* Do not allow zero-length read requests */
        if (request[PROTOCOL_LENGTH_OFFSET] != 0u)
        {
            if ((request[PROTOCOL_FLAGS_OFFSET] & I2C_READ_FLAG) != 0u)
            {
                *flags = CMD_I2C_TRANSFER_STOP_MSK;
            }
            else
            {
                *flags = 0u;
            }

            *flags |= CMD_I2C_TRANSFER_READ_MSK;
            retVal = true;
        }
        else
        {
            retVal = false;
        }

        break;

    default:
        /* Not supported command */
        break;
    }

    return (retVal);
}


/******************************************************************************
*  I2c_SetSpeed
***************************************************************************//**
* Sets I2C block input frequency for I2C line to match the speed that is
* provided in the request.
*
* @param[in] request The pointer to the request string.
*
* @param[in] response The pointer to the memory that will be used for storing
*   the response.
*
* @return Number of bytes in response buffer.
*
******************************************************************************/
static uint32_t I2c_SetSpeed(const uint8_t *request, uint8_t *response)
{
    uint32_t num = 0u;
    /* Combine bytes into 32-bit value */
    uint32_t desiredSpeed = ((uint32_t)request[PROTOCOL_SPEED_SET_B1_OFFSET]) |
                            (((uint32_t)request[PROTOCOL_SPEED_SET_B2_OFFSET]) << 8u) |
                            (((uint32_t)request[PROTOCOL_SPEED_SET_B3_OFFSET]) << 16u) |
                            (((uint32_t)request[PROTOCOL_SPEED_SET_B4_OFFSET]) << 24u);

    I2C_UDB_Stop();

    switch (desiredSpeed)
    {
    case SPEED_I2C_50K:
        Clock_I2C_SetDivider(I2C_CLK_DIVIDER_50K);
        i2cActSpeed = SPEED_I2C_50K;
        response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_SUCCESS;
        break;

    case SPEED_I2C_100K:
        Clock_I2C_SetDivider(I2C_CLK_DIVIDER_100K);
        i2cActSpeed = SPEED_I2C_100K;
        response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_SUCCESS;
        break;

    case SPEED_I2C_400K:
        Clock_I2C_SetDivider(I2C_CLK_DIVIDER_400K);
        i2cActSpeed = SPEED_I2C_400K;
        response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_SUCCESS;
        break;

    case SPEED_I2C_1M:
        Clock_I2C_SetDivider(I2C_CLK_DIVIDER_1M);
        i2cActSpeed = SPEED_I2C_1M;
        response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_SUCCESS;
        break;

    default:
        response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_FAIL_INV_PAR;
        break;
    }
    num ++;
    if (response[PROTOCOL_STATUS_OFFSET] != CMD_STAT_FAIL_INV_PAR)
    {
        response[PROTOCOL_SPEED_REPLY_B1_OFFSET] = i2cActSpeed;
        response[PROTOCOL_SPEED_REPLY_B2_OFFSET] = 0u;
        response[PROTOCOL_SPEED_REPLY_B3_OFFSET] = 0u;
        response[PROTOCOL_SPEED_REPLY_B4_OFFSET] = 0u;
        num += 4u;
    }

    /* Start the I2C and acknowledge */
    I2C_UDB_Start();
    return num;
}

/******************************************************************************
*  Spi_SetSpeed
***************************************************************************//**
* Sets SPI block input frequency for SPI line to match the speed that is
* provided in the request. If the requested speed cannot be achieved: smaller
* acceptable value is applied.
*
* @param[in] request The pointer to the request string.
*
* @param[out] response The pointer to the memory that will be used for storing
*   the response.
*
* @return (num of bytes in request << 16) | (num of bytes in response)
*
******************************************************************************/
static uint32_t Spi_SetSpeed(const uint8_t *request, uint8_t *response)
{
    uint32_t num = 0u;
    /* Combine bytes into 32-bit value */
    uint32_t desiredSpeed = ((uint32_t)request[PROTOCOL_SPEED_SET_B1_OFFSET]) |
                            (((uint32_t)request[PROTOCOL_SPEED_SET_B2_OFFSET]) << 8u) |
                            (((uint32_t)request[PROTOCOL_SPEED_SET_B3_OFFSET]) << 16u) |
                            (((uint32_t)request[PROTOCOL_SPEED_SET_B4_OFFSET]) << 24u);

    if (  KitHasSpiBridge() &&
          (desiredSpeed >= ((SOURCECLK_IMO/SPI_COMP_DIVIDER)/SPI_DIVIDER_MAX)) &&
          (desiredSpeed <= (((SOURCECLK_IMO/SPI_COMP_DIVIDER)/SPI_DIVIDER_MIN))) )
    {
        uint32_t resDivider = (uint32_t)((SOURCECLK_IMO/SPI_COMP_DIVIDER)/(desiredSpeed));

        uint32_t resSpeed = (uint32_t)(((SOURCECLK_IMO/SPI_COMP_DIVIDER)/(resDivider)));

        /* _GetDividerRegister() function returns divider value - 1. Take that into account. */
        if ((Clock_SPI_GetDividerRegister() + 1u) != (uint16_t)resDivider)
        {
            Clock_SPI_SetDivider((uint16_t)(resDivider-1u));
        }

        response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_SUCCESS;

        response[PROTOCOL_SPEED_REPLY_B1_OFFSET] = LO8(LO16(resSpeed));
        response[PROTOCOL_SPEED_REPLY_B2_OFFSET] = HI8(LO16(resSpeed));
        response[PROTOCOL_SPEED_REPLY_B3_OFFSET] = LO8(HI16(resSpeed));
        response[PROTOCOL_SPEED_REPLY_B4_OFFSET] = HI8(HI16(resSpeed));
        num += 5U;
        SPIM_HW_DynSPIModeConfig((request[PROTOCOL_SPI_CONTROL2_OFFSET] & SPIM_CPHA_MASK) >> SPIM_CPHA_SHIFT,\
                         (request[PROTOCOL_SPI_CONTROL2_OFFSET] & SPIM_CPOL_MASK) >> SPIM_CPOL_SHIFT,\
                          (request[PROTOCOL_SPI_CONTROL2_OFFSET] & SPIM_SHIFT_DIR_MASK));

        spiActSpeed = resSpeed;
    }
    else
    {
        if (KitHasSpiBridge())
        {
            response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_FAIL_INV_PAR;
            num++;
        }
        else
        {
            /* Move pointer back to byte 0 */
            response[PROTOCOL_CMD_OFFSET] = CMD_INVALID;

        }
    }
    return (num);
}

/******************************************************************************
*  Spi_GenericTransaction
***************************************************************************//**
* Executes an generic SPI transaction.
*
* @param[in]  controlByte Byte Control byte
* @param[in]  length      Data length
* @param[in]  commandData Pointer to command data buffer
* @param[out] returnData  Pointer to return data buffer
* @param[in]  SS_mode Selected slave device:
*                           0x01  -  P15_3
*                           0x02  -  P3_4
*                           0x04  -  P3_6
*
******************************************************************************/
static void Spi_GenericTransaction(uint8_t controlByte, uint8_t length, const uint8_t *commandData, uint8_t *returnData, uint8_t SS_mode)
{
    uint8_t index;

    /* Generate  Start signal - SS transaction High to LOW */
    if((controlByte & SPIM_CTRL_START) == SPIM_CTRL_START)
    {
        /* Disable SS for short time */
        SPI_SS_CTRL_Write(SPI_SS_CTRL_Read() | SS_mode);

        /* Short delay */
        CyDelayUs(SPI_DELAY_AFTER_SS_SET);

        /* Enable corresponded SS signal */
        SPI_SS_CTRL_Write(SPI_SS_CTRL_Read() ^ SS_mode);

        /* Clear SPI FIFO */
        SPIM_HW_ClearFIFO();
    }

    /* Perform SPI transaction */
    for( index = 0; index < length; index++)
    {
        /* Send the tx data */
        SPIM_HW_WriteTxData(commandData[index]);

        while((SPIM_HW_ReadStatus() & SPIM_HW_STS_RX_FIFO_NOT_EMPTY) == 0u)
        {
            /* Wait for the byte transfer to complete */
        }

        returnData[index] = SPIM_HW_ReadRxData();
    }

    /* Generate STOP signal - SS transition LOW to HIGH */
    if ((controlByte & SPIM_CTRL_STOP) == SPIM_CTRL_STOP)
    {
        SPI_SS_CTRL_Write(SPI_SS_CTRL_Read() | SS_mode);
    }
}

/******************************************************************************
*  I2cSpi_GetSetSpeed
***************************************************************************//**
* Handles the Set or Get Speed I2C/SPI Vendor request
*
* @param[in] request   The pointer to the request string.
* @param[out] response The pointer to the memory that will be used for storing
*   the response for the last packet.
* returns (num of bytes in request << 16) | (num of bytes in response)
*
******************************************************************************/
static uint32_t I2cSpi_GetSetSpeed(const uint8_t *request, uint8_t *response)
{
    uint32_t num = 0u;

    response[PROTOCOL_CMD_OFFSET] = request[PROTOCOL_CMD_OFFSET];
    num++;

    if (request[PROTOCOL_SETGET_OFFSET] == PROTOCOL_SET)
    {
        /* Set speed */
        if (request[PROTOCOL_INTERFACE_OFFSET] == PROTOCOL_I2C)
        {
            num += I2c_SetSpeed(request, response);
            num += (7UL << 16);
        }
        else if (request[PROTOCOL_INTERFACE_OFFSET] == PROTOCOL_SPI)
        {
            num += Spi_SetSpeed(request, response);
            num += (8UL << 16);
        }
        else
        {
            /* Unknown interface */
            response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_FAIL_INV_PAR;
            num++;
        }
    }
    else if (request[PROTOCOL_SETGET_OFFSET] == PROTOCOL_GET)
    {
        /* Get speed */
        if (request[PROTOCOL_INTERFACE_OFFSET] == PROTOCOL_I2C)
        {
            /* Interface I2C */
            response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_SUCCESS;
            response[PROTOCOL_SPEED_BYTE1_OFFSET] = i2cActSpeed;
            response[PROTOCOL_SPEED_BYTE2_OFFSET] = 0u;
            response[PROTOCOL_SPEED_BYTE3_OFFSET] = 0u;
            response[PROTOCOL_SPEED_BYTE4_OFFSET] = 0u;
            num += (3UL << 16) | 6UL; /* 3 bytes in request, 6 bytes in response */
        }
        else if (request[PROTOCOL_INTERFACE_OFFSET] == PROTOCOL_SPI)
        {
            /* Interface SPI */
            response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_SUCCESS;
            response[PROTOCOL_SPEED_BYTE1_OFFSET] = LO8(LO16(spiActSpeed));
            response[PROTOCOL_SPEED_BYTE2_OFFSET] = HI8(LO16(spiActSpeed));
            response[PROTOCOL_SPEED_BYTE3_OFFSET] = LO8(HI16(spiActSpeed));
            response[PROTOCOL_SPEED_BYTE4_OFFSET] = HI8(HI16(spiActSpeed));
            response[PROTOCOL_MODE_OFFSET] = 0u;
            num += (3UL << 16) | 7U;

            /* cpol */
            if ((SPIM_HW_CONTROL_REG & SPIM_HW_CTRL_CPOL_SET) != 0u)
            {
                response[PROTOCOL_MODE_OFFSET] = PROTOCOL_CPOL_FLAG;
            }

            /* cpha */
            if ((SPIM_HW_CONTROL_REG & SPIM_HW_CTRL_CPHA_SET) != 0u)
            {
                response[PROTOCOL_MODE_OFFSET] |= PROTOCOL_CPHA_FLAG;
            }

            /* Byte order */
            if ((CY_GET_XTND_REG8(CFG_REG(BYTE_ORDER_REG_ADDR,SPIM_HW_A0_ADDR)) & SPIM_HW_CTRL_SHIFT_SEL_6_SET) != 0u)
            {
                response[PROTOCOL_MODE_OFFSET] |= PROTOCOL_BORDER_FLAG;
            }
        }
        else
        {
            /* Invalid parameter for bridge interface*/
        }
    }
    else
    {
        /* Unknown interface/command */
        response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_FAIL_INV_PAR;
        num++;
    }
    return (num);
}
/******************************************************************************
*  BridgesInterfaceHandler
***************************************************************************//**
* Handles the Bridge Interface operation
*
******************************************************************************/
void Bridge_InterfaceHandler(void)
{
    if (USBFS_IN_BUFFER_EMPTY == USBFS_GetEPState(BRIDGE_INTERFACE_IN_ENDP))
    {
        /* Start handle only if IN endpoint is ready */
        switch (stateI2cOp)
        {
            case STATE_I2C_RW_STARTED:
            /* Check if the time to send alive message to host
            * Timer is down counting, that is why old minus new value is correct interval*/
            if ((waitResponseTimer - Timer_CSTick_ReadCounter()) >= (uint16_t)TIMER_CSTICK_RATE)
            {
                waitResponseTimer = Timer_CSTick_ReadCounter();
                uint32_t intrMask = CyUsbIntDisable();
                USBFS_LoadInEP(BRIDGE_INTERFACE_IN_ENDP, waitResponse, BRIDGE_INTERFACE_ENDP_SIZE);
                CyUsbIntEnable(intrMask);
            }
            /*  Check if any response is sent from slave device */
            I2c_CheckForResponse(&bridgeRequest[requestIndex], &bridgeResponse[responseIndex]);
            break;

            case STATE_I2C_READY:
            {
                /* If request is not executed proceed with it */
                switch (commandExecution)
                {
                    case STARTED:

                        Bridge_ExecuteCommands();
                        break;

                    case NOTHING_TO_HANDLE:
                        {
                         /* Receive request from host */
                        if (USBFS_GetEPState(BRIDGE_INTERFACE_OUT_ENDP) == USBFS_OUT_BUFFER_FULL)
                        {
                            /* Empty buffers to get rid from info from previouus transaction */
                            (void)memset(bridgeRequest, 0, sizeof(bridgeRequest));
                            (void)memset(bridgeResponse, 0, sizeof(bridgeResponse));
                            uint16_t receiveSize = USBFS_GetEPCount(BRIDGE_INTERFACE_OUT_ENDP);
                            if (receiveSize > 0u)
                            {
                                uint32_t intrMask = CyUsbIntDisable();
                                (void)USBFS_ReadOutEP(BRIDGE_INTERFACE_OUT_ENDP, bridgeRequest, receiveSize);
                                CyUsbIntEnable(intrMask);

                                /* init variables for request handling */
                                initCommand = true;
                                requestIndex = 0u;
                                responseIndex = 0u;
                                /* execute command */
                                Bridge_ExecuteCommands();
                            }
                        }
                        break;
                    }
                    case FINISHED:

                        /* Send response if there are any to send */
                        /* normal Response for other commands send to bridge if i2c transaction not started */
                        {
                            uint32_t intrMask = CyUsbIntDisable();
                            USBFS_LoadInEP(BRIDGE_INTERFACE_IN_ENDP, bridgeResponse, BRIDGE_INTERFACE_ENDP_SIZE);
                            CyUsbIntEnable(intrMask);
                        }
                        commandExecution = NOTHING_TO_HANDLE;
                        break;

                    default:
                        /* Not a possible option */
                        break;
                }
                break;
            }

            default:
                /* Not a possible option */
                break;
        }
    }
}

/******************************************************************************
*  Bridge_ExecuteCommand
***************************************************************************//**
* Executes commands sent to the device
*
******************************************************************************/
void Bridge_ExecuteCommands(void)
{
    if (initCommand)
    {
        if (bridgeRequest[requestIndex] == ID_DAP_ExecuteCommands)
        {
            bridgeResponse[responseIndex]= bridgeRequest[requestIndex];

            responseIndex++;
            requestIndex++;

            cmdCount = bridgeRequest[requestIndex];
            bridgeResponse[responseIndex] = (uint8_t)cmdCount;

            responseIndex++;
            requestIndex++;

            commandExecution = STARTED;
        }
        else
        {
            /* One command to process */
            (void)Bridge_ProcessCommand(&bridgeRequest[requestIndex], &bridgeResponse[responseIndex]);
            commandExecution = STARTED;

            if (stateI2cOp != STATE_I2C_RW_STARTED)
            {
                commandExecution = FINISHED;
            }
        }
        initCommand = false;
    }

    if ((commandExecution == STARTED) && (stateI2cOp != STATE_I2C_RW_STARTED))
    {
        while(cmdCount != 0u)
        {
            uint32_t num = Bridge_ProcessCommand(&bridgeRequest[requestIndex], &bridgeResponse[responseIndex]);
            requestIndex  += (uint8_t)(num >> 16);
            if (stateI2cOp == STATE_I2C_RW_STARTED)
            {
                break;
            }
            responseIndex += (uint8_t)num;
            cmdCount--;

        }
        if (cmdCount == 0u)
        {
            commandExecution = FINISHED;
        }
    }
}

/******************************************************************************
*  Bridge_ProcessCommand
***************************************************************************//**
* Processes commands and prepares response for all commands except I2C
* transaction
* @param[in] request The pointer to the request string.
*
* @param[out] response The pointer to the memory that will be used for storing
*   the response for the last packet.
*
* returns (num of bytes in request << 16) | (num of bytes in response)
******************************************************************************/
uint32_t Bridge_ProcessCommand(const uint8_t *request, uint8_t *response)
{
    uint32_t num = 0u;
    switch (*request)
    {
    case ID_DAP_ResetTarget:
        /* Command 0x0A */
        num = DAP_ProcessCommand(request, response);
        break;

    /* CMSIS-DAP provides range 0x80-0x9F for user(vendor) commands, 002-23370 for now limits our usage to range
       0x80-0x90, some of them are implemented here, some in DAP_vendor.c */
    case ID_DAP_Vendor0:
        /* Command 0x80 */
        num = DAP_ProcessVendorCommand(request, response);
        break;

    case ID_DAP_Vendor1:
        /* Command 0x81 */
        num = DAP_ProcessVendorCommand(request, response);
        break;

    case ID_DAP_Vendor2:
        /* Command 0x82 */
        num = DAP_ProcessVendorCommand(request, response);
        break;

    case ID_DAP_Vendor4:
        /* Command 0x84 */
        num = DAP_ProcessVendorCommand(request, response);
        break;

    case CMD_ID_SET_GET_INT_SPEED:
        /* Command 0x86 */
        num = I2cSpi_GetSetSpeed(request, response);
        break;

    case CMD_ID_RESTART_I2C_MSTR:
        /* Command 0x87 */
        {
            /* Restart HW */
            response[PROTOCOL_CMD_OFFSET] = request[PROTOCOL_CMD_OFFSET];
            num++;
            if(I2c_Restart() == true)
            {
                response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_SUCCESS;
            }
            else
            {
                response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_FAIL_OP_FAIL;
            }
            num++;
            num = ((1UL << 16)| num);
        }
        break;

    case CMD_ID_I2C_TRANSACTION:
        /* Command 0x88 */
        {
            uint8_t flags = (request[PROTOCOL_FLAGS_OFFSET] & CMD_I2C_TRANSFER_TYPE_MSK) >> BYTE_MULTR_SHIFT;
            if (I2c_ParseCommand(&flags, request))
            {
                if (I2c_OpNonBlocking(flags, request[PROTOCOL_LENGTH_OFFSET],
                               &request[PROTOCOL_ADDRESS_OFFSET]) == 0u)
                {
                    stateI2cOp = STATE_I2C_RW_STARTED;

                    uint32_t intrMask = CyUsbIntDisable();
                    USBFS_LoadInEP(BRIDGE_INTERFACE_IN_ENDP, waitResponse, BRIDGE_INTERFACE_ENDP_SIZE);
                    CyUsbIntEnable(intrMask);
                    waitResponseTimer = Timer_CSTick_ReadCounter();

                }
                else
                {
                    static const uint8_t failOpResponce[BRIDGE_INTERFACE_ENDP_SIZE] = {[0] = CMD_ID_I2C_TRANSACTION, [1] = CMD_STAT_FAIL_OP_FAIL};
                    /* Fail Responses*/
                    uint32_t intrMask = CyUsbIntDisable();
                    USBFS_LoadInEP(BRIDGE_INTERFACE_IN_ENDP, failOpResponce, BRIDGE_INTERFACE_ENDP_SIZE);
                    CyUsbIntEnable(intrMask);
                    commandExecution = NOTHING_TO_HANDLE;
                }
                num = ((((uint32_t)request[PROTOCOL_LENGTH_OFFSET]) + 4UL) << 16U);

            }
            else
            {
                response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_FAIL_INV_PAR;
                num = (1UL << 16) | 1UL;
            }
        }
        break;

    case CMD_ID_SPI_DATA_TRANSFER:
        /* Command 0x89 */
        if (KitHasSpiBridge())
        {
            response[PROTOCOL_CMD_OFFSET] = request[PROTOCOL_CMD_OFFSET];
            response[PROTOCOL_STATUS_OFFSET] = CMD_STAT_SUCCESS;
            num += 2u;

            Spi_GenericTransaction(request[PROTOCOL_SPI_CONTROL_OFFSET], request[PROTOCOL_SPI_LENGTH_OFFSET],
                                   &request[PROTOCOL_SPI_DATA_OFFSET], &response[PROTOCOL_RESP_DATA_OFFSET],
                                   request[PROTOCOL_SPI_SS_OFFSET]);
            num += request[PROTOCOL_SPI_LENGTH_OFFSET];
            num += ((4UL + (uint32_t)request[PROTOCOL_SPI_LENGTH_OFFSET]) << 16);
        }
        else
        {
            response[PROTOCOL_CMD_OFFSET] = CMD_INVALID;
            num++;
        }
        break;

    case ID_DAP_Vendor10:
        /* Command 0x8A */
        if (KitHasGpioBridge())
        {
            response[GENERAL_RESPONSE_COMMAND] = request[GENERAL_REQUEST_COMMAND];
            num += Bridge_GpioSetMode(request, response);
        }
        else
        {
            response[PROTOCOL_CMD_OFFSET] = ID_DAP_Invalid;
            num += ((1UL << 16) | 1UL);
        }
        break;

    case ID_DAP_Vendor11:
        /* Command 0x8B */
        if (KitHasGpioBridge())
        {
            response[GENERAL_RESPONSE_COMMAND] = request[GENERAL_REQUEST_COMMAND];
            num += Bridge_GpioSetState(request, response);
        }
        else
        {
            response[PROTOCOL_CMD_OFFSET] = ID_DAP_Invalid;
            num += ((1UL << 16) | 1UL);
        }
        break;

    case ID_DAP_Vendor12:
        /* Command 0x8C */
        if (KitHasGpioBridge())
        {
            response[GENERAL_RESPONSE_COMMAND] = request[GENERAL_REQUEST_COMMAND];
            num += Bridge_GpioReadState(request, response);
        }
        else
        {
            response[PROTOCOL_CMD_OFFSET] = ID_DAP_Invalid;
            num += ((1UL << 16) | 1UL);
        }
        break;

    case ID_DAP_Vendor13:
        /* Command 0x8D */
        if (KitHasGpioBridge())
        {
            response[GENERAL_RESPONSE_COMMAND] = request[GENERAL_REQUEST_COMMAND];
            num += Bridge_GpioStateChanged(request, response);
        }
        else
        {
            response[PROTOCOL_CMD_OFFSET] = ID_DAP_Invalid;
            num += ((1UL << 16) | 1UL);
        }
        break;

    case ID_DAP_Vendor16:
        /* Command 0x90 */
        num = DAP_ProcessVendorCommand(request, response);
        break;

    default:
        response[PROTOCOL_CMD_OFFSET] = ID_DAP_Invalid;
        num += ((1UL << 16) | 1UL);
        break;

    }
    return (num);
}

/******************************************************************************
*  Bridge_PrepareI2cInterface
***************************************************************************//**
* Enables I2C component and sets the pin modes.
*
*******************************************************************************/
void Bridge_PrepareI2cInterface(void)
{
    Clock_I2C_Start();

    CyPins_SetPinDriveMode(Pin_I2C_SDA_0, Pin_I2C_SDA_DM_OD_LO);
    CyPins_SetPinDriveMode(Pin_I2C_SCL_0, Pin_I2C_SCL_DM_OD_LO);

    I2C_UDB_Start();
}


/******************************************************************************
*  Bridge_PrepareSpiInterface
***************************************************************************//**
* Enables SPI component.
*
*******************************************************************************/
void Bridge_PrepareSpiInterface(void)
{
    Clock_SPI_Start();

    SPIM_HW_Start();
}

/*******************************************************************************
* Bridge_PrepareUartInterface
********************************************************************************
* Starts and prepares UART interface(s) for communication
*
*******************************************************************************/
void Bridge_PrepareUartInterface(void)
{
    UART_Bridge_Start();
    Clock_UART_Start();
    if (KitHasUartIndicator())
    {
        WicedUartHciLed = &LED_Red_Write;
    }

    if ( KitHasSecondaryUart() )
    {
        UART_Bridge_2_Start();
        Clock_UART_2_Start();
        if (KitHasUartIndicator())
        {
            WicedUartPeriLed = &LED_Green_Write;
        }
    }

}

/*******************************************************************************
* UartRtsnSetMode
********************************************************************************
* For the UART RTS pin of Kit with HWID 0x0C, the RTS pin can be switched
* among especial mode and normal operation
*
* @param[in] comPort - UART number
*
* @param[in] mode - RTS operation mode
*                  UART_RTS_N_NORMAL_MODE(0u) - hardware connected to the UART component, DM_STRONG
*                  UART_RTS_N_FORCE_UP_MODE(1u) - force up, DM_RES_UP
*******************************************************************************/
static void UartRtsnSetMode(uint8_t comPort, uint8_t mode)
{
    if (KitHasSpecialRts() && ((comPort == 0u) || ((comPort == 1u) && KitHasSecondaryUart())))
    {
        uart_bridge_t const *port = &uart[comPort];
        switch(mode)
        {
            case UART_RTS_N_NORMAL_MODE:
                /* enable bypass and going to DM_STRONG */
                *((reg8 *)port->uartRtsPinByp) |= port->uartRtsPinMask;
                CyPins_SetPinDriveMode(port->uartRtsPinPc, PIN_DM_STRONG);
                break;
            case UART_RTS_N_FORCE_UP_MODE:
                /* disable bypass and going to DM_RES_UP */
                CyPins_SetPin(port->uartRtsPinPc);
                CyPins_SetPinDriveMode(port->uartRtsPinPc, PIN_DM_RES_UP);
                *((reg8 *)port->uartRtsPinByp) &= ~((uint8_t)port->uartRtsPinMask);
                break;
            default:
                /* Impossible case*/
                break;
        }
    }
}

/*******************************************************************************
* UsbUartRtsEventHandler
********************************************************************************
* USB evant handling to enforce UART RTS pin state for Kit with HWID 0x0C
*
* @param[in] comPort - UART number
* @param[in] eventType - RTS_FLAG_PWRON_OR_RESET(0x01u) - powering on or target HW reset event
*                        RTS_FLAG_SWD_ACTIVE(0x02u) - SWD debugger is connected
*                        RTS_FLAG_USBUART_RTS(0x04u) - request from USB CDC
* @param[in] asserted - event asserted or withdrawn
*******************************************************************************/
static void UsbUartRtsEventHandler(uint8_t comPort, uint8_t eventType, bool asserted)
{
    static uint8_t usbUartRtsFlag[2u] = { RTS_FLAG_NOT_SET, RTS_FLAG_NOT_SET };
    if ((comPort == 0u) || ((comPort == 1u) && KitHasSecondaryUart()))
    {
        uint8_t prevRtsFlags = usbUartRtsFlag[comPort];
        uint8_t newRtsFlags;
        if (asserted)
        {
            newRtsFlags = prevRtsFlags | eventType;
            if ((prevRtsFlags == RTS_FLAG_NOT_SET) && (newRtsFlags != RTS_FLAG_NOT_SET))
            {
                /* at least one flag has been set */
                UartRtsnSetMode(comPort, UART_RTS_N_FORCE_UP_MODE);
            }
        }
        else
        {
            newRtsFlags = prevRtsFlags & (~eventType);
            if ((prevRtsFlags != RTS_FLAG_NOT_SET) && (newRtsFlags == RTS_FLAG_NOT_SET))
            {
                /* all flags has been cleared */
                UartRtsnSetMode(comPort, UART_RTS_N_NORMAL_MODE);
            }
        }
        if (prevRtsFlags != newRtsFlags)
        {
            usbUartRtsFlag[comPort] = newRtsFlags;
        }
    }
}

/*******************************************************************************
* UsbUartCalculateDivider
********************************************************************************
* Returns closest Clock Source and Divider values needed for given Baud Rate
*******************************************************************************/
static uint16_t UsbUartCalculateDivider(uint32_t dteRate, uint8_t * clockSource)
{
    uint16_t divider;

    /* Limit max and min values of dteRate */
    uint32_t rate = (dteRate < 50u) ? 50u : ((dteRate > 4000000u) ? 4000000u : dteRate);

    /* Calculate needed dividers for PLL clock to achieve dteRate */
    uint32_t normValue = UART_Bridge_OVER_SAMPLE_COUNT*rate;
    uint32_t dividerPll = ((SOURCECLK)/normValue);

    /* Multiply by two to omit division */
    if ((2u*((SOURCECLK)%(normValue))) >= normValue)
    {
        dividerPll++;
    }

    uint32_t ratePll = ((SOURCECLK)/(UART_Bridge_OVER_SAMPLE_COUNT*dividerPll));

    /* Do not calculate divider for IMO if requested rate is too high */
    if (rate > MAX_IMO_RATE)
    {
        *clockSource = CYCLK_SRC_SEL_PLL;
        divider =  (uint16_t)dividerPll;
        rate = ratePll;
    }
    else
    {
        uint32_t dividerImo = ((SOURCECLK_IMO)/normValue);
        if ((2u*((SOURCECLK_IMO)%(normValue))) >= normValue)
        {
            dividerImo++;
        }

        /* Get actual rate with calculated dividers */
        uint32_t rateImo = ((SOURCECLK_IMO)/(UART_Bridge_OVER_SAMPLE_COUNT*dividerImo));

        /* Calculate difference between requested and actual rates */
        uint32_t diffPll = ((dteRate >= ratePll) ? (dteRate - ratePll) : (ratePll - dteRate)) % dteRate;
        uint32_t diffImo = ((dteRate >= rateImo) ? (dteRate - rateImo) : (rateImo - dteRate)) % dteRate;

        /* If use of either Clock Source is possible or PLL option is preferred*/
        if ((diffPll == diffImo) || (diffPll < diffImo))
        {
            *clockSource = CYCLK_SRC_SEL_PLL;
            divider =  (uint16_t)dividerPll;
            rate = ratePll;
        }
        else
        {
            *clockSource = CYCLK_SRC_SEL_IMO;
            divider = (uint16_t)dividerImo;
            rate = rateImo;
        }
    }

    uint8_t portNr = USBFS_GetComPort();
    /* keep real data rate */
    uartLastRate[portNr] = rate;

    return divider;
}

/*******************************************************************************
* UsbUartSetClock
********************************************************************************
* Sets Source Clock and Divider value for UART
*******************************************************************************/
static void UsbUartSetClock(void)
{
    uint8_t mode = currentMode;
    uint8_t portNr = USBFS_GetComPort();
    const uart_bridge_t *port = (portNr == 0u) ? &uart[0u] :
                                (((portNr == 1u) &&
                                (mode == MODE_BULK2UARTS)) ? &uart[1u] : (void *)0);
    if (port != (void *)0)
    {
        uint8_t clockSample;

        /* Get requested baud rate */
        uint32_t dteRate = USBFS_GetDTERate();
        static uint32_t uartLastDteRate[2u] = {0u, 0u}; /* Last requested DTE Rate per UART*/
        if (dteRate != uartLastDteRate[portNr])
        {
            uartLastDteRate[portNr] = dteRate;
            uint16_t divider = UsbUartCalculateDivider(dteRate, &clockSample);
            bool uartStopped = false;

            if (port->UartClockGetSourceRegister() != clockSample)
            {
                /* Stop UART for new Clock */
                port->UartStop();
                uartStopped = true;
                port->UartClockStop();
                port->UartClockSetSourceRegister(clockSample);
                port->UartClockStart();
            }
            if ((port->UartClockGetDividerRegister() + 1u) != divider)
            {
                /* Stop UART for new clock if not stopped already */
                if (!uartStopped)
                {
                    port->UartStop();
                    uartStopped = true;
                }
                /* Set new Clock Frequency divider */
                port->UartClockSetDividerRegister(divider - 1u, 1u);
            }

            /* Restart UART if clock was changed */
            if (uartStopped)
            {
                port->UartClearRxBuffer();
                port->UartClearTxBuffer();
                port->UartStart();
            }
        }

        /* Get UART's real baud rate last set */
        uint32_t rate = uartLastRate[portNr];
        /* Update COM port structure with real rate */
        USBFS_linesCoding[portNr][USBFS_LINE_CODING_RATE] = LO8(LO16(rate));
        USBFS_linesCoding[portNr][USBFS_LINE_CODING_RATE + 1u] = HI8(LO16(rate));
        USBFS_linesCoding[portNr][USBFS_LINE_CODING_RATE + 2u] = LO8(HI16(rate));
        USBFS_linesCoding[portNr][USBFS_LINE_CODING_RATE + 3u] = HI8(HI16(rate));
    }
}

/*******************************************************************************
* UsbUartCheckLine
********************************************************************************
* Checks if CDC line was changed and if it was - adjusts UART speed and
* enforcement RTS state
*
*******************************************************************************/
static void UsbUartCheckLine(void)
{
    /* PROGTOOLS-2894: Call USBFS_IsLineChanged inside a critical section
       to avoid changing USBFS_transferState from idle state while executed */
    uint32_t intrMask = CyUsbIntDisable();
    uint8_t lineChangedState = USBFS_IsLineChanged();
    CyUsbIntEnable(intrMask);
    if((lineChangedState & USBFS_LINE_CONTROL_CHANGED) != 0u)
    {
        if (KitHasSpecialRts())
        {
            /* only for Kit with HWID 0x0C */
            uint16_t lineControl = USBFS_GetLineControl();
            uint8_t comPort = USBFS_GetComPort();
            if ( (lineControl & USBUART_LINE_CONTROL_DTR) != 0u )
            {
                /* DTR is on */
                if ( (lineControl & USBUART_LINE_CONTROL_RTS) != 0u )
                {
                    /* RTS can be driven by UART component */
                    UsbUartRtsEventHandler(comPort, RTS_FLAG_USBUART_RTS, false);
                }
                else
                {
                    /* RTS was requested to force off */
                    UsbUartRtsEventHandler(comPort, RTS_FLAG_USBUART_RTS, true);
                }
            }
            else
            {
                /* when DTR is off, it implies that RTS also shall be off */
                UsbUartRtsEventHandler(comPort, RTS_FLAG_USBUART_RTS, true);
            }
        }
    }
    if((lineChangedState & USBFS_LINE_CODING_CHANGED) != 0u)
    {
        UsbUartSetClock();
    }
}

/*******************************************************************************
* UsbUartTransmit
********************************************************************************
* Checks if USB host has sent data and transfers it to the UART Transmitter
*
*******************************************************************************/
static void UsbUartTransmit(uint8_t comPort)
{
    uint8_t mode = currentMode;
    if ((comPort == 0u) || ((comPort == 1u) && (mode == MODE_BULK2UARTS)))
    {
        uart_bridge_t const *port = &uart[comPort];

        /* Check for USB host packet */
        if(USBFS_GetEPState(port->uartOutEp) == USBFS_OUT_BUFFER_FULL)
        {
            /* Get size of packet */
            uint16_t size = USBFS_GetEPCount(port->uartOutEp);

            /* Get available UART buffer size */
            uint16_t uartBufferAvailable = port->uartTxBufferSize - port->UartGetTxBufferSize();

            /* Load data from endpoint only if the required space is available in the UART buffer */
            if (uartBufferAvailable >= size)
            {
                // LED ON for dedicated port
                (* port->UartLed)(1);
                /* Read from USB Buffer */
                static uint8_t usbuartTxBuffer[USBOUTPACKETSIZE];

                uint32_t intrMask = CyUsbIntDisable();
                (void)USBFS_ReadOutEP(port->uartOutEp, usbuartTxBuffer, size);
                CyUsbIntEnable(intrMask);

                /* Send to UART Tx */
                port->UartPutArray(usbuartTxBuffer, size);
            }
            // LED OFF for dedicated port
            (* port->UartLed)(0);
        }
    }
}

/*******************************************************************************
* UsbUartReceive
********************************************************************************
* Checks if the UART has received data and sends it to the USB host
*
*******************************************************************************/
static void UsbUartReceive(uint8_t comPort)
{
    uint8_t mode = currentMode;
    if ((comPort == 0u) || ((comPort == 1u) && (mode == MODE_BULK2UARTS)))
    {
         uart_bridge_t const *port = &uart[comPort];

        if (USBFS_bGetEPState(port->uartInEp) == USBFS_IN_BUFFER_EMPTY)
        {
            uint8_t rxStatus = port->UartReadRxStatus();

            /* Check status of UART_RX_STS_SOFT_BUFF_OVER */
            if((rxStatus & port->uartSwBufferOverflowMask) != 0u){
                port->UartClearRxBuffer();
            }

            uint16_t wCount = port->UartGetRxBufferSize();
            if (wCount > USBINPACKETSIZE)
            {
                wCount = USBINPACKETSIZE;
            }

            if(wCount == 0u)
            {
                if((rxStatus & port->uartHwErrorMask) != 0u)
                {
                    /* will clear HW FIFO if HWError occured */
                    for(uint8_t cnt = 0; cnt < port->uartFifoLength; cnt++ )
                    {
                        (void)port->UartReadRxData();
                    }
                }
            }

            /* Check if Rx has data then populate buffer */
            static uint8_t usbuartRxBuffer[USBINPACKETSIZE];
            if(wCount != 0u)
            {
                // LED ON for dedicated port
                (* port->UartLed)(1);

                /* Load from software UART RX Buffer */
                for(uint16_t index = 0; index < wCount; index++){
                    usbuartRxBuffer[index] = port->UartReadRxData();
                }
            }
            else
            {
                /* Try to load data from HW FIFO if it is not empty */
                rxStatus = port->UartReadRxStatus();
                uint8_t interSettings;
                while ( ((rxStatus & port->uartHwFifoNotEmptyMask) != 0u) && (wCount < USBINPACKETSIZE) )
                {
                    if (wCount == 0u)
                    {
                        interSettings = CyEnterCriticalSection();
                    }
                    /* at least one byte can be read */
                    usbuartRxBuffer[wCount] = port->UartReadRxData();
                    wCount++;
                    /* refresh the current state of the receiver status register */
                    rxStatus = port->UartReadRxStatus();
                }
                if (wCount != 0u)
                {
                    CyExitCriticalSection(interSettings);
                }
            }

            static uint16_t uartInEpLastSent[2u] = {0u, 0u}; /* The last sent size is required for the ZLP generation logic */

            if ( (wCount != 0u) || (uartInEpLastSent[comPort] == USBINPACKETSIZE) )
            {
                /* Send out data */
                uint32_t intrMask = CyUsbIntDisable();
                USBFS_LoadInEP(port->uartInEp, usbuartRxBuffer, wCount);
                CyUsbIntEnable(intrMask);

                uartInEpLastSent[comPort] = wCount;
            }
            // LED OFF for dedicated port
            (* port->UartLed)(0);
        }
    }
}

/*******************************************************************************
* Bridge_UartInterfaceHandler
********************************************************************************
* Handles USB-UART data transfers
*
*******************************************************************************/
void Bridge_UartInterfaceHandler(void)
{
    /* Check for Line configuration change, Reconfigure UART based on host parameters */
    UsbUartCheckLine();

    /* USB to UART Transmit */
    UsbUartTransmit(0u);

    /* UART to USB Receive */
    UsbUartReceive(0u);

    if (currentMode == MODE_BULK2UARTS)
    {
        /* Check for Line configuration change, Reconfigure UART based on host parameters */
        USBFS_SetComPort(1u);
        UsbUartCheckLine();
        USBFS_SetComPort(0u);

        /* USB to UART Transmit */
        UsbUartTransmit(1u);

        /* UART to USB Receive */
        UsbUartReceive(1u);
    }
}


/*******************************************************************************
* UsbUartStart
********************************************************************************
* Starts the USB UART bridge
*
*******************************************************************************/
void UsbUartStart(void)
{
    UART_Bridge_Start();

    /* Initialize CDC Interface for USB-UART Bridge */
    (void)USBFS_CDC_Init();


    Pin_UART_Tx_SetDriveMode(Pin_UART_Tx_DM_STRONG);

    if (KitHasSecondaryUart())
    {
        if (currentMode == MODE_BULK2UARTS)
        {
            UART_Bridge_2_Start();

            /* Initialize CDC Interface for second USB-UART Bridge */
            USBFS_SetComPort(1);
            (void)USBFS_CDC_Init();
            USBFS_SetComPort(0);

            UART_TX_2_SetDriveMode(UART_TX_2_DM_STRONG);
        }
        else
        {
            UART_Bridge_2_Stop();
            /* switch output Uart2 pins to HI-Z mode */
            UART_TX_2_SetDriveMode(UART_TX_2_DM_DIG_HIZ);
            UART_RTS_2_SetDriveMode(UART_RTS_2_DM_DIG_HIZ);
        }
    }
}

/*******************************************************************************
* UsbUartEventPwrOnOrReset
********************************************************************************
* Generate event of target hardware reset for kits with HWID 0x0C
*******************************************************************************/
static void UsbUartEventPwrOnOrReset(bool asserted)
{
    if (KitHasSpecialRts())
    {
        UsbUartRtsEventHandler(0u, RTS_FLAG_PWRON_OR_RESET, asserted);
        UsbUartRtsEventHandler(1u, RTS_FLAG_PWRON_OR_RESET, asserted);
    }
}

/*******************************************************************************
* isr_SWDXRES_Interrupt_InterruptCallback
********************************************************************************
* Handler for target hardware reset for kits with HWID 0x0C
*******************************************************************************/
void isr_SWDXRES_Interrupt_InterruptCallback(void)
{
    /* clear interrupt source */
    (void)SWDXRES_ClearInterrupt();
    /* generate event */
    UsbUartEventPwrOnOrReset(true);
    /* reload the timer counter for RTS delay */
    Timer_RTS_Delay_Stop();
    Timer_RTS_Delay_Start();
}

/*******************************************************************************
* isr_RTSDelay_Interrupt_InterruptCallback
********************************************************************************
* Handler for RTS delay expiration for kits with HWID 0x0C
*******************************************************************************/
void isr_RTSDelay_Interrupt_InterruptCallback(void)
{
    /* clear interrupt source */
    (void)Timer_RTS_Delay_ReadStatusRegister();
    /* generate event */
    UsbUartEventPwrOnOrReset(false);
}

/*******************************************************************************
* UartCtsRtsPinInit
********************************************************************************
* If the Kit does not have CTS/RTS lines, set the CTS pin to Resistive Pull
*  Down in order to avoid noise impact, this will pull lines for flow control
*  low and this will make sure that these lines (CTS/RTS) do not have logic high
*  as this will affect UART operation.
* If the Kit has HWID 0x0C the RTS pin does resistive pulling up and only after
*  a delay of RTS_DELAY_IN_HIGH(200 ms) is it initialized for strong DM.
*
*******************************************************************************/
void UartCtsRtsPinInit(void)
{
    /* CTS pin init */
    if(!KitHasUartHwFlowControl())
    {
        /* Set CTS logic to Internal Active-Low*/
        Control_Flow_En_Write(0u);

        CyPins_ClearPin(UART_CTS_0);
        CyPins_SetPinDriveMode(UART_CTS_0, UART_CTS_DM_RES_DWN);
        if (KitHasSecondaryUart())
        {
            CyPins_ClearPin(UART_CTS_2_0);
            CyPins_SetPinDriveMode(UART_CTS_2_0, UART_CTS_2_DM_RES_DWN);
        }
    }
    else
    {
        /* Set CTS logic to External */
        Control_Flow_En_Write(1u);
    }
    /* RTS pin init */
    if (KitHasSpecialRts())
    {
        /* This means that hardware flow control is also present. */
        UartRtsnSetMode(0u, UART_RTS_N_FORCE_UP_MODE);
        UartRtsnSetMode(1u, UART_RTS_N_FORCE_UP_MODE);
        /* KITPROG3-90: special behavior of RTS signal on power up
        * to be able powering up CYW20819 without autobaud mode.
        * by default bypass is disabled and drive mode is DM_RES_UP
        */
        CyDelay(RTS_DELAY_IN_HIGH_MS);
        /* enable bypass and going to DM_STRONG */
        UartRtsnSetMode(0u, UART_RTS_N_NORMAL_MODE);
        UartRtsnSetMode(1u, UART_RTS_N_NORMAL_MODE);

        /* setup timer for RTS delay for target hw reset*/
        Timer_RTS_Delay_WritePeriod(RTS_DELAY_IN_HIGH_TICK);
        (void)Timer_RTS_Delay_ReadStatusRegister();
        /* activate isr handler for timer */
        isr_RTSDelay_Start();
        /* activate isr handler for SWDXRES pin */
        (void)SWDXRES_ClearInterrupt();
        isr_SWDXRES_Start();
    }
    else
    {
        if (KitHasUartHwFlowControl())
        {
            /* enable bypass and going to DM_STRONG */
            UartRtsnSetMode(0u, UART_RTS_N_NORMAL_MODE);
            UartRtsnSetMode(1u, UART_RTS_N_NORMAL_MODE);
        }
        else
        {
            /* disable bypass and going to DM_RES_DWN */
            CyPins_ClearPin(UART_RTS_0);
            CyPins_SetPinDriveMode(UART_RTS_0, UART_RTS_DM_RES_DWN);
            *((reg8 *)UART_RTS__BYP) &= ~((uint8_t)UART_RTS_MASK);
            if (KitHasSecondaryUart())
            {
                CyPins_ClearPin(UART_RTS_2_0);
                CyPins_SetPinDriveMode(UART_RTS_2_0, UART_RTS_2_DM_RES_DWN);
                *((reg8 *)UART_RTS_2__BYP) &= ~((uint8_t)UART_RTS_2_MASK);
            }
        }
    }
}

/*******************************************************************************
* UsbUartEventSwdConnect
********************************************************************************
* Generate event on SWD debugger connect/disconnect for kits with HWID 0x0C
*******************************************************************************/
void UsbUartEventSwdConnect(bool asserted)
{
    if (KitHasSpecialRts())
    {
        UsbUartRtsEventHandler(0u, RTS_FLAG_SWD_ACTIVE, asserted);
        UsbUartRtsEventHandler(1u, RTS_FLAG_SWD_ACTIVE, asserted);
    }
}


/*******************************************************************************
* NoUartIndication
********************************************************************************
* Empty function for devices with no LED UART traffick indication
*******************************************************************************/
static void NoUartIndication(uint8_t value)
{
    (void)value;
}

/******************************************************************************
*  Gpio_DsiBypassDisable
***************************************************************************//**
* Disable DSI bypass and let the Port logic data register drives the
* corresponding port pin.
*
*******************************************************************************/
static void Gpio_DsiBypassDisable(void)
{
    /* Let the Port logic data register drives the corresponding port pin. */
    GPIO_BYP = (uint8_t)(GPIO_BYP & ~ GPIO__SPI_SS_2_GPIO_1__MASK);
}


/******************************************************************************
*  Bridge_PrepareGpioInterface
***************************************************************************//**
* Configures GPIO pins for work.
*
*******************************************************************************/
void Bridge_PrepareGpioInterface(void)
{
    /* Configure pins 3[5] and 3[6]pins */
    /* Bypass shared pin from control reg */
    Gpio_DsiBypassDisable();
    GPIO_SetDriveMode(GPIO_DM_ALG_HIZ);
}

/*******************************************************************************
* UpdateGpioState
********************************************************************************
* Update GPIO pin structures with transition change
*******************************************************************************/
static void UpdateGpioState(gpio_pin_t volatile * gpioPin)
{
    if (gpioPin->currentState == gpioPin->previousState)
    {
        gpioPin->change = UNCHANGED;
    }
    else if (gpioPin->currentState < gpioPin->previousState)
    {
        gpioPin->change = TRANSITION_HIGH_LOW;
    }
    else
    {
        gpioPin->change = TRANSITION_LOW_HIGH;
    }
}
/******************************************************************************
*  Bridge_GpioSetMode
***************************************************************************//**
* Sets Drive Mode of one GPIO Pin at the time
* @param[in] request The pointer to the request string.
*
* @param[out] response The pointer to the memory that will be used for storing
*   the response for the last packet.
*
* returns (num of bytes in request << 16) | (num of bytes in response)
*******************************************************************************/
uint32_t Bridge_GpioSetMode(const uint8_t * request, uint8_t *response)
{
    uint32_t retVal = 0u;

    /* Enum for supported GPIO drive modes */
    enum {
        HIZ = 0x00,
        RES_UP,
        RES_DWN,
        ODLO,
        ODHI,
        DM_STR,
        RES_UPDWN
    };

    const uint8_t modes[7u] =
    {
        [HIZ] = PIN_DM_DIG_HIZ,
        [RES_UP] = PIN_DM_RES_UP,
        [RES_DWN] = PIN_DM_RES_DWN,
        [ODLO] = PIN_DM_OD_LO,
        [ODHI] = PIN_DM_OD_HI,
        [DM_STR] = PIN_DM_STRONG,
        [RES_UPDWN] = PIN_DM_RES_UPDWN
    };
    bool modeIsValid = false;
    uint8_t desiredMode;

    /* Get pin number and desired mode from request buffer*/
    uint8_t pin = request[GPIO_REQUEST_PIN];
    uint8_t mode = request[GPIO_REQUEST_STATE_MODE];

    for(int8_t index = HIZ; index <= RES_UPDWN; index++)
    {
        if (mode == (uint8_t)index)
        {
            desiredMode = modes[index];
            modeIsValid = true;
        }
    }
    const volatile gpio_pin_t *curPin = ((pin == gpioState[0].pin) ? &gpioState[0] :
                                ((pin == gpioState[1].pin) ? &gpioState[1] :
                                (void *)0));
    bool valid = (curPin != (void *)0) ? true: false;
    if (valid && modeIsValid)
    {
        CyPins_SetPinDriveMode((reg8 *)curPin->pinReg, desiredMode);
        uint8_t curMode = CyPins_ReadPinDriveMode((reg8 *)curPin->pinReg);

        response[GENERAL_RESPONSE_STATUS] = (curMode == desiredMode) ? DAP_OK : DAP_ERROR;
    }
    else
    {
        response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
    }
    retVal += ((3UL << 16) | 2UL);
    return (retVal);
}

/******************************************************************************
*  Bridge_GpioSetState
***************************************************************************//**
* Sets state of one GPIO Pin at the time
* @param[in] request The pointer to the request string.
*
* @param[out] response The pointer to the memory that will be used for storing
*   the response for the last packet.
*
* returns (num of bytes in request << 16) | (num of bytes in response)
*******************************************************************************/
uint32_t Bridge_GpioSetState(const uint8_t * request, uint8_t *response)
{
    uint32_t retVal = 0u;

    /* Get pin number and desired state from request buffer*/
    uint8_t pin = request[GPIO_REQUEST_PIN];
    uint8_t state = request[GPIO_REQUEST_STATE_MODE];

    gpio_pin_t volatile *curPin = ((pin == gpioState[0].pin) ? &gpioState[0] :
                            ((pin == gpioState[1].pin) ? &gpioState[1] :
                            (void *)0));
    bool valid = (curPin != (void *)0) ? true : false;

    if (valid)
    {
        if (CyPins_ReadPinDriveMode((reg8 *)curPin->pinReg) == PIN_DM_DIG_HIZ)
        {
            response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
        }
        else
        {
            if (state == GPIO_CLEAR_STATE)
            {
                CyPins_ClearPin((reg8 *)curPin->pinReg);
            }
            else
            {
                CyPins_SetPin((reg8 *)curPin->pinReg);
            }
            /* Update previous and current states of GPIO pin */
            curPin->previousState = curPin->currentState;
            uint8_t curState = (CyPins_ReadPin((reg8 *)curPin->pinReg) != 0u) ? 1u : 0u;
            curPin->currentState = curState;

            /* Form Response */
            response[GENERAL_RESPONSE_STATUS] = (curState == state) ? DAP_OK : DAP_ERROR;
            gpioChanged = false;
        }
    }
    else
    {
        response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
    }

    retVal += ((3UL << 16) | 2UL);
    return (retVal);
}

/******************************************************************************
*  Bridge_GpioReadState
***************************************************************************//**
* Reads current state of one GPIO Pin at the time and returns it
* @param[in] request The pointer to the request string.
*
* @param[out] response The pointer to the memory that will be used for storing
*   the response for the last packet.
*
* returns (num of bytes in request << 16) | (num of bytes in response)
*******************************************************************************/
uint32_t Bridge_GpioReadState(const uint8_t * request, uint8_t *response)
{
    uint32_t retVal = 0u;

    /* Get pin number from request */
    uint8_t pin = request[GPIO_REQUEST_PIN];

    const volatile gpio_pin_t *curPin = ((pin == gpioState[0].pin) ? &gpioState[0] :
                            ((pin == gpioState[1].pin) ? &gpioState[1] :
                            (void *)0));
    bool valid = (curPin != (void *)0) ? true : false;

    if (valid)
    {
        response[GENERAL_RESPONSE_STATUS] = DAP_OK;
        response[GENERAL_RESPONSE_RESULT] = (CyPins_ReadPin((reg8 *)curPin->pinReg) != 0u) ? 1u : 0u;
        retVal += (2UL << 16) | 3UL;
    }
    else
    {
        response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
        retVal += (2UL << 16) | 2UL;
    }

    return (retVal);
}

/******************************************************************************
*  Bridge_GpioStateChanged
***************************************************************************//**
* Reads current state and compares to previous state, returns in response
* whether it has changed
* @param[in] request The pointer to the request string.
*
* @param[out] response The pointer to the memory that will be used for storing
*   the response for the last packet.
*
* returns (num of bytes in request << 16) | (num of bytes in response)
*******************************************************************************/
uint32_t Bridge_GpioStateChanged(const uint8_t * request, uint8_t *response)
{
    uint32_t retVal = 0u;

    /* Get pin number from request */
    uint8_t pin = request[GPIO_REQUEST_PIN];

    gpio_pin_t volatile *curPin = ((pin == gpioState[0].pin) ? &gpioState[0] :
                            ((pin == gpioState[1].pin) ? &gpioState[1] :
                            (void *)0));
    bool valid = (curPin != (void *)0) ? true : false;
    if (valid)
    {
        UpdateGpioState(curPin);
        uint8_t stateChange = curPin->change;
        gpioChanged = false;

        /* Form response */
        response[GENERAL_RESPONSE_STATUS] = DAP_OK;
        response[GENERAL_RESPONSE_RESULT] = stateChange;
        retVal += (2UL << 16) | 3UL;
    }
    else
    {
        response[GENERAL_RESPONSE_STATUS] = CMD_STAT_FAIL_INV_PAR;
        retVal += (2UL << 16) | 2UL;
    }

    return (retVal);
}


/*******************************************************************************
* GPIO_isr_Interrupt_InterruptCallback
********************************************************************************
* Handler for GPIO interrupt
*******************************************************************************/
void GPIO_isr_Interrupt_InterruptCallback(void)
{
    gpioChanged = true;
    /* Clear interrupt source */
    (void)GPIO_ClearInterrupt();
    /* Update current state of GPIO pins */
    gpioState[0].currentState = (CyPins_ReadPin((reg8 *)gpioState[0].pinReg) != 0u) ? 1u : 0u;
    gpioState[1].currentState = (CyPins_ReadPin((reg8 *)gpioState[1].pinReg) != 0u) ? 1u : 0u;
}

/* [] END OF FILE */
