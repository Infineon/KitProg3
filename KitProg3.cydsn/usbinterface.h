/*************************************************************************//**
* @file usbinterface.h
*
* @brief
*   This file contains the function prototypes, macros and constants used
*    in usbinterface.c file.
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
#if !defined(USBINTERFACE_H)
#define USBINTERFACE_H

#include "DAP.h"

extern uint8_t  USB_Request [DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // Request  Buffer
extern volatile bool     USB_RequestFlag;       // Request  Buffer Usage Flag
extern volatile bool     USB_RequestPostponed;  // Request  was not transferred to the buffer
extern volatile uint8_t  USB_RequestMutex;      // Request  comes in right now!
extern volatile uint32_t USB_RequestIn;         // Request  Buffer In  Index
extern volatile uint32_t USB_RequestOut;        // Request  Buffer Out Index

extern volatile bool     USB_ResponseFlag;      // Response Buffer Usage Flag
extern volatile bool     USB_ResponsePostponed; // Response was not transferred to the buffer
extern volatile uint8_t  USB_ResponseMutex;     // Response goes out right now!
extern volatile uint32_t USB_ResponseIn;        // Response Buffer In  Index
extern volatile uint32_t USB_ResponseOut;       // Response Buffer Out Index

extern uint32_t USB_ResponseLen[DAP_PACKET_COUNT];
extern uint8_t  USB_Response[DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // Response Buffer
extern volatile bool     USB_ResponseIdle;      // Response Buffer Idle  Flag

void PrepareUsbInterface(void);
void PushToRequestBuffer(void);
void PopFromResponseBuffer(void);

#endif /*USBINTERFACE_H*/


/* [] END OF FILE */
