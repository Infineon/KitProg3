/*******************************************************************************
* File Name: `$INSTANCE_NAME`_INT.c
* Version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
*
* Description:
*  This file provides all Interrupt Service Routine (ISR) for the SPI Master
*  component.
*
* Note:
*  None.
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "`$INSTANCE_NAME`_PVT.h"
`$CY_API_CALLBACK_HEADER_INCLUDE`

/* User code required at start of ISR */
/* `#START `$INSTANCE_NAME`_ISR_START_DEF` */

/* `#END` */


/*******************************************************************************
* Function Name: `$INSTANCE_NAME`_TX_ISR
********************************************************************************
*
* Summary:
*  Interrupt Service Routine for TX portion of the SPI Master.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  `$INSTANCE_NAME`_txBufferWrite - used for the account of the bytes which
*  have been written down in the TX software buffer.
*  `$INSTANCE_NAME`_txBufferRead - used for the account of the bytes which
*  have been read from the TX software buffer, modified when exist data to
*  sending and FIFO Not Full.
*  `$INSTANCE_NAME`_txBuffer[`$INSTANCE_NAME`_TX_BUFFER_SIZE] - used to store
*  data to sending.
*  All described above Global variables are used when Software Buffer is used.
*
*******************************************************************************/
CY_ISR(`$INSTANCE_NAME`_TX_ISR)
{
    #if(`$INSTANCE_NAME`_TX_SOFTWARE_BUF_ENABLED)
        uint8 tmpStatus;
    #endif /* (`$INSTANCE_NAME`_TX_SOFTWARE_BUF_ENABLED) */

    #ifdef `$INSTANCE_NAME`_TX_ISR_ENTRY_CALLBACK
        `$INSTANCE_NAME`_TX_ISR_EntryCallback();
    #endif /* `$INSTANCE_NAME`_TX_ISR_ENTRY_CALLBACK */

    /* User code required at start of ISR */
    /* `#START `$INSTANCE_NAME`_TX_ISR_START` */

    /* `#END` */
    
    #if(`$INSTANCE_NAME`_TX_SOFTWARE_BUF_ENABLED)
        /* Check if TX data buffer is not empty and there is space in TX FIFO */
        while(`$INSTANCE_NAME`_txBufferRead != `$INSTANCE_NAME`_txBufferWrite)
        {
            tmpStatus = `$INSTANCE_NAME`_GET_STATUS_TX(`$INSTANCE_NAME`_swStatusTx);
            `$INSTANCE_NAME`_swStatusTx = tmpStatus;

            if(0u != (`$INSTANCE_NAME`_swStatusTx & `$INSTANCE_NAME`_STS_TX_FIFO_NOT_FULL))
            {
                if(0u == `$INSTANCE_NAME`_txBufferFull)
                {
                   `$INSTANCE_NAME`_txBufferRead++;

                    if(`$INSTANCE_NAME`_txBufferRead >= `$INSTANCE_NAME`_TX_BUFFER_SIZE)
                    {
                        `$INSTANCE_NAME`_txBufferRead = 0u;
                    }
                }
                else
                {
                    `$INSTANCE_NAME`_txBufferFull = 0u;
                }

                /* Put data element into the TX FIFO */
                `$CySetRegReplacementString`(`$INSTANCE_NAME`_TXDATA_PTR, 
                                             `$INSTANCE_NAME`_txBuffer[`$INSTANCE_NAME`_txBufferRead]);
            }
            else
            {
                break;
            }
        }

        if(`$INSTANCE_NAME`_txBufferRead == `$INSTANCE_NAME`_txBufferWrite)
        {
            /* TX Buffer is EMPTY: disable interrupt on TX NOT FULL */
            `$INSTANCE_NAME`_TX_STATUS_MASK_REG &= ((uint8) ~`$INSTANCE_NAME`_STS_TX_FIFO_NOT_FULL);
        }

    #endif /* (`$INSTANCE_NAME`_TX_SOFTWARE_BUF_ENABLED) */

    /* User code required at end of ISR (Optional) */
    /* `#START `$INSTANCE_NAME`_TX_ISR_END` */

    /* `#END` */
    
    #ifdef `$INSTANCE_NAME`_TX_ISR_EXIT_CALLBACK
        `$INSTANCE_NAME`_TX_ISR_ExitCallback();
    #endif /* `$INSTANCE_NAME`_TX_ISR_EXIT_CALLBACK */
}


/*******************************************************************************
* Function Name: `$INSTANCE_NAME`_RX_ISR
********************************************************************************
*
* Summary:
*  Interrupt Service Routine for RX portion of the SPI Master.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  `$INSTANCE_NAME`_rxBufferWrite - used for the account of the bytes which
*  have been written down in the RX software buffer modified when FIFO contains
*  new data.
*  `$INSTANCE_NAME`_rxBufferRead - used for the account of the bytes which
*  have been read from the RX software buffer, modified when overflow occurred.
*  `$INSTANCE_NAME`_rxBuffer[`$INSTANCE_NAME`_RX_BUFFER_SIZE] - used to store
*  received data, modified when FIFO contains new data.
*  All described above Global variables are used when Software Buffer is used.
*
*******************************************************************************/
CY_ISR(`$INSTANCE_NAME`_RX_ISR)
{
    #if(`$INSTANCE_NAME`_RX_SOFTWARE_BUF_ENABLED)
        uint8 tmpStatus;
        `$RegSizeReplacementString` rxData;
    #endif /* (`$INSTANCE_NAME`_RX_SOFTWARE_BUF_ENABLED) */

    #ifdef `$INSTANCE_NAME`_RX_ISR_ENTRY_CALLBACK
        `$INSTANCE_NAME`_RX_ISR_EntryCallback();
    #endif /* `$INSTANCE_NAME`_RX_ISR_ENTRY_CALLBACK */

    /* User code required at start of ISR */
    /* `#START `$INSTANCE_NAME`_RX_ISR_START` */

    /* `#END` */
    
    #if(`$INSTANCE_NAME`_RX_SOFTWARE_BUF_ENABLED)

        tmpStatus = `$INSTANCE_NAME`_GET_STATUS_RX(`$INSTANCE_NAME`_swStatusRx);
        `$INSTANCE_NAME`_swStatusRx = tmpStatus;

        /* Check if RX data FIFO has some data to be moved into the RX Buffer */
        while(0u != (`$INSTANCE_NAME`_swStatusRx & `$INSTANCE_NAME`_STS_RX_FIFO_NOT_EMPTY))
        {
            rxData = `$CyGetRegReplacementString`(`$INSTANCE_NAME`_RXDATA_PTR);

            /* Set next pointer. */
            `$INSTANCE_NAME`_rxBufferWrite++;
            if(`$INSTANCE_NAME`_rxBufferWrite >= `$INSTANCE_NAME`_RX_BUFFER_SIZE)
            {
                `$INSTANCE_NAME`_rxBufferWrite = 0u;
            }

            if(`$INSTANCE_NAME`_rxBufferWrite == `$INSTANCE_NAME`_rxBufferRead)
            {
                `$INSTANCE_NAME`_rxBufferRead++;
                if(`$INSTANCE_NAME`_rxBufferRead >= `$INSTANCE_NAME`_RX_BUFFER_SIZE)
                {
                    `$INSTANCE_NAME`_rxBufferRead = 0u;
                }

                `$INSTANCE_NAME`_rxBufferFull = 1u;
            }

            /* Move data from the FIFO to the Buffer */
            `$INSTANCE_NAME`_rxBuffer[`$INSTANCE_NAME`_rxBufferWrite] = rxData;

            tmpStatus = `$INSTANCE_NAME`_GET_STATUS_RX(`$INSTANCE_NAME`_swStatusRx);
            `$INSTANCE_NAME`_swStatusRx = tmpStatus;
        }

    #endif /* (`$INSTANCE_NAME`_RX_SOFTWARE_BUF_ENABLED) */

    /* User code required at end of ISR (Optional) */
    /* `#START `$INSTANCE_NAME`_RX_ISR_END` */

    /* `#END` */
    
    #ifdef `$INSTANCE_NAME`_RX_ISR_EXIT_CALLBACK
        `$INSTANCE_NAME`_RX_ISR_ExitCallback();
    #endif /* `$INSTANCE_NAME`_RX_ISR_EXIT_CALLBACK */
}

/* [] END OF FILE */
