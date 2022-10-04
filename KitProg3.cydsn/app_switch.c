/*************************************************************************//**
* @file app_switch.c
*
* @brief
*   Provides the source code to handle the application switching
*   for the bootloadable project.
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

#include "app_switch.h"
#include "Bootloadable.h"


/*******************************************************************************
* Function Name: Bootloadable_WriteFlashByte
****************************************************************************//**
*
* \brief
*   This API writes to flash the specified data.
*
* \param address
*   The address in flash.
*
* \param inputValue
*   One-byte data.
*
* \return
*   A status of the writing to flash procedure.
*
*******************************************************************************/
static cystatus Bootloadable_WriteFlashByte(const uint32_t address, const uint8_t inputValue)
{
    cystatus result = CYRET_SUCCESS;
    uint32_t flsAddr = address - CYDEV_FLASH_BASE;
    uint8_t  rowData[CYDEV_FLS_ROW_SIZE];

    #if !(CY_PSOC4)
        uint8_t arrayId = ( uint8_t )(flsAddr / CYDEV_FLS_SECTOR_SIZE);
    #endif  /* !(CY_PSOC4) */

    #if (CY_PSOC4)
        uint16_t rowNum = ( uint16_t )(flsAddr / CYDEV_FLS_ROW_SIZE);
    #else
        uint16_t rowNum = ( uint16_t )((flsAddr % CYDEV_FLS_SECTOR_SIZE) / CYDEV_FLS_ROW_SIZE);
    #endif  /* (CY_PSOC4) */

    uint32_t baseAddr = address - (address % CYDEV_FLS_ROW_SIZE);
    uint16_t idx;

    for(idx = 0u; idx < CYDEV_FLS_ROW_SIZE; idx++)
    {
        rowData[idx] = (uint8_t)Bootloadable_GET_CODE_DATA(baseAddr + idx);
    }

    rowData[address % CYDEV_FLS_ROW_SIZE] = inputValue;

    #if(CY_PSOC4)
        result = CySysFlashWriteRow((uint32_t) rowNum, rowData);
    #else
        result = CyWriteRowData(arrayId, rowNum, rowData);
    #endif  /* (CY_PSOC4) */

    #if(CY_PSOC5)
        /***************************************************************************
        * When writing to flash, data in the instruction cache can become stale.
        * Therefore, the cache data does not correlate to the data just written to
        * flash. A call to CyFlushCache() is required to invalidate the data in the
        * cache and force fresh information to be loaded from flash.
        ***************************************************************************/
        CyFlushCache();
    #endif /* (CY_PSOC5) */
    return (result);
}


/*******************************************************************************
* Function Name: Bootloadable_SetActiveApplication
****************************************************************************//**
*
* \brief
*   Sets the application which will be loaded after a next reset event.
*
* \details
* Theory:
*   This API sets in the Flash (metadata section) the given active application
*   number.
*
*   NOTE The active application number is not set directly, but the boolean
*   mark instead means that the application is active or not for the relative
*   metadata. Both metadata sections are updated. For example, if the second
*   application is to be set active, then in the metadata section for the first
*   application there will be a "0" written, which means that it is not active, and
*   for the second metadata section there will be a "1" written, which means that it is
*   active.
*
*   NOTE Intended for the combination project type ONLY!
*
* \param appId
*   The active application number to be written to flash (metadata section)
*   NOTE Possible values are:
*   0 - for the first application
*   1 - for the second application.
*   Any other number is considered invalid.
*
* \return
*   A status of writing to flash operation.
*   \n CYRET_SUCCESS - Returned if appId was successfully changed.
*   \n CYRET_BAD_PARAM - Returned if the parameter appID passed to the function has the
*                       same value as the active application ID.
*   \note - The other non-zero value is considered as a failure during writing to flash.
*
*   \note - This API does not update Bootloader_activeApp variable.
*
*******************************************************************************/
cystatus Bootloadable_SetActiveApplication(uint8_t appId)
{
    cystatus result = CYRET_SUCCESS;
    uint8_t buff[CYDEV_FLS_ROW_SIZE + CYDEV_ECC_ROW_SIZE];

    /* Use CySetTemp API to initialize SPC. */
    /* This step enables writing to flash memory. */
    /* It's required to change number of active application. */
    if (CYRET_SUCCESS != CySetTemp())
    {
        CyHalt(0x00u);
    }

    (void)CySetFlashEEBuffer(buff);

    result |= Bootloadable_WriteFlashByte((uint32_t) Bootloadable_MD_BTLDB_ACTIVE_OFFSET(0u), ((0u == appId) ? 1u : 0u));
    result |= Bootloadable_WriteFlashByte((uint32_t) Bootloadable_MD_BTLDB_ACTIVE_OFFSET(1u), ((1u == appId) ? 1u : 0u));

    return (result);
}


/* [] END OF FILE */
