/*************************************************************************//**
* @file app_switch.h
*
* @brief
*   This file contains the function prototypes and constants used in
*   app_switch.c file.
*
* @version KitProg3 v2.60
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

#if !defined(APP_SWITCH_H)
#define APP_SWITCH_H

#include <device.h>

#define Bootloadable_MD_SIZEOF                      (64u)
#define Bootloadable_MD_BASE_ADDR(appId)            (CYDEV_FLASH_BASE + (CYDEV_FLASH_SIZE - ((uint32_t)(appId) * CYDEV_FLS_ROW_SIZE) - Bootloadable_MD_SIZEOF))
#define Bootloadable_MD_BTLDB_ACTIVE_OFFSET(appId)  (Bootloadable_MD_BASE_ADDR(appId) + 16u)

#define TWO_UARTS_SWITCH_TIMEOUT    (2000u)
#define MODE_SWITCH_TIMEOUT          (100u)

cystatus Bootloadable_SetActiveApplication(uint8_t appId);

#endif /* APP_SWITCH_H */

/* [] END OF FILE */
