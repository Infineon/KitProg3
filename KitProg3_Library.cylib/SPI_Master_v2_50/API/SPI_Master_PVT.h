/*******************************************************************************
* File Name: `$INSTANCE_NAME_PVT`.h
* Version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
*
* Description:
*  This private header file contains internal definitions for the SPIM
*  component. Do not use these definitions directly in your application.
*
* Note:
*
********************************************************************************
* Copyright 2012-2015, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SPIM_PVT_`$INSTANCE_NAME`_H)
#define CY_SPIM_PVT_`$INSTANCE_NAME`_H

#include "`$INSTANCE_NAME`.h"


/**********************************
*   Functions with external linkage
**********************************/


/**********************************
*   Variables with external linkage
**********************************/

extern volatile uint8 `$INSTANCE_NAME`_swStatusTx;
extern volatile uint8 `$INSTANCE_NAME`_swStatusRx;

#if(`$INSTANCE_NAME`_TX_SOFTWARE_BUF_ENABLED)
    extern volatile `$RegSizeReplacementString` `$INSTANCE_NAME`_txBuffer[`$INSTANCE_NAME`_TX_BUFFER_SIZE];
    extern volatile uint8 `$INSTANCE_NAME`_txBufferRead;
    extern volatile uint8 `$INSTANCE_NAME`_txBufferWrite;
    extern volatile uint8 `$INSTANCE_NAME`_txBufferFull;
#endif /* (`$INSTANCE_NAME`_TX_SOFTWARE_BUF_ENABLED) */

#if(`$INSTANCE_NAME`_RX_SOFTWARE_BUF_ENABLED)
    extern volatile `$RegSizeReplacementString` `$INSTANCE_NAME`_rxBuffer[`$INSTANCE_NAME`_RX_BUFFER_SIZE];
    extern volatile uint8 `$INSTANCE_NAME`_rxBufferRead;
    extern volatile uint8 `$INSTANCE_NAME`_rxBufferWrite;
    extern volatile uint8 `$INSTANCE_NAME`_rxBufferFull;
#endif /* (`$INSTANCE_NAME`_RX_SOFTWARE_BUF_ENABLED) */

#endif /* CY_SPIM_PVT_`$INSTANCE_NAME`_H */


/* [] END OF FILE */
