/************************************************************************/
/*----------------------------------------------------------------------------
 *      RL-ARM - USB
 *----------------------------------------------------------------------------
 *      Name:    usbd_user_hid.c
 *      Purpose: Human Interface Device Class User module
 *      Rev.:    V4.50
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------
 * Portions Copyright 2018, Cypress Semiconductor Corporation
 * or a subsidiary of Cypress Semiconductor Corporation. All rights
 * reserved.
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
 * so agrees to indemnify Cypress against all liability.
 * THE CYPRESS COPYRIGHTED PORTIONS ARE NOT SUBMISSIONS AS SET FORTH IN THE
 * APACHE LICENSE VERSION 2.0 OR ANY OTHER LICENSE HAVING SIMILAR PROVISIONS.
 *---------------------------------------------------------------------------*/

#include <string.h>
#include "DAP.h"
#include "DAP_config.h"
#include "kitprog.h"
#include "mutex.h"
#include "usbinterface.h"

/* mask number of bytes in response (lower 16 bits) in DAP result */
#define RESPONSE_MASK   (0x0000FFFFu)

volatile bool     USB_RequestFlag;       // Request  Buffer is Full Flag
volatile bool     USB_RequestPostponed;  // Request  was not transferred to the buffer
volatile uint8_t  USB_RequestMutex;      // Request  comes in right now!
volatile uint32_t USB_RequestIn;         // Request  Buffer In  Index / Place of next wrire
volatile uint32_t USB_RequestOut;        // Request  Buffer Out Index / Place of next read

volatile bool     USB_ResponseIdle;      // Response Buffer Idle  Flag
volatile bool     USB_ResponseFlag;      // Response Buffer Usage Flag
volatile bool     USB_ResponsePostponed; // Response was not transferred to the buffer
volatile uint8_t  USB_ResponseMutex;     // Response goes out right now!
volatile uint32_t USB_ResponseIn;        // Response Buffer In  Index
volatile uint32_t USB_ResponseOut;       // Response Buffer Out Index

uint32_t USB_ResponseLen[DAP_PACKET_COUNT];
uint8_t  USB_Request [DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // Request  Buffer
uint8_t  USB_Response[DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // Response Buffer


// USB HID Callback: when system initializes
void usbd_hid_init(void)
{
    USB_RequestFlag   = false;
    USB_RequestIn     = 0u;
    USB_RequestOut    = 0u;
    USB_ResponseIdle  = true;
    USB_ResponseFlag  = false;
    USB_ResponseIn    = 0u;
    USB_ResponseOut   = 0u;
    // mutexes and postponed request
    USB_RequestMutex = MUTEX_UNLOCKED;
    USB_RequestPostponed = false;
    USB_ResponseMutex = MUTEX_UNLOCKED;
    USB_ResponsePostponed = false;
}

// USB HID Callback: when data needs to be prepared for the host
uint32_t usbd_hid_get_report(uint8_t rtype, uint8_t rid, uint8_t *buf, uint8_t req)
{
    (void)(rid);
    uint32_t res = 0u;

    if (rtype == HID_REPORT_INPUT)
    {
        if (req == USBD_HID_REQ_EP_INT)
        {
            /* Init intermediate values for volatile variables USB_ResponseIn, USB_ResponseOut */
            uint32_t cachedUsbResponseOut = USB_ResponseOut;
            uint32_t cachedUsbResponseIn = USB_ResponseIn;
            bool cachedUsbResponseFlag = USB_ResponseFlag;

            if ((cachedUsbResponseOut != cachedUsbResponseIn) || cachedUsbResponseFlag)
            {
                res = USB_ResponseLen[cachedUsbResponseOut] & RESPONSE_MASK;
                (void)memcpy(buf, USB_Response[cachedUsbResponseOut], res);
                cachedUsbResponseOut++;
                if (cachedUsbResponseOut == DAP_PACKET_COUNT)
                {
                    cachedUsbResponseOut = 0u;
                }
                if (cachedUsbResponseOut == cachedUsbResponseIn)
                {
                    USB_ResponseFlag = false;
                }
                USB_ResponseOut = cachedUsbResponseOut;
            }
        }
    }
    return (res);
}

// USB HID Callback: when data is received from the host
void usbd_hid_set_report(uint8_t rtype, uint8_t rid, uint8_t *buf, uint8_t len, uint8_t req)
{
    (void)(req);
    (void)(rid);

    switch (rtype)
    {
        case HID_REPORT_OUTPUT:
        {
            if (len == 0u)
            {
                // Nothing to send
                break;
            }
            if (buf[0u] == ID_DAP_TransferAbort)
            {
                DAP_TransferAbort = 1u;
                break;
            }
            /* Init intermediate values for volatile variables USB_RequestIn, USB_RequestOut */
            uint32_t cachedUsbRequestIn = USB_RequestIn;
            uint32_t cachedUsbRequestOut = USB_RequestOut;
            bool cachedUsbRequestFlag = USB_RequestFlag;
            if (cachedUsbRequestFlag && (cachedUsbRequestIn == cachedUsbRequestOut))
            {
                break;  // Discard packet when buffer is full
            }
            // Store data into request packet buffer
            (void)memcpy(USB_Request[cachedUsbRequestIn], buf, len);
            cachedUsbRequestIn++;

            if (cachedUsbRequestIn == DAP_PACKET_COUNT)
            {
                cachedUsbRequestIn = 0u;
            }
            if (cachedUsbRequestIn == cachedUsbRequestOut)
            {
                cachedUsbRequestFlag = true;
            }
            USB_RequestIn = cachedUsbRequestIn;
            USB_RequestFlag = cachedUsbRequestFlag;
            break;
        }
        case HID_REPORT_FEATURE:
            break;
        default:
            /**/
            break;
    }
}

// Process USB HID Data
void usbd_hid_process(void)
{
    /* Init intermediate values for volatile variables USB_ResponseIn, USB_ResponseOut */
    uint32_t cachedUsbResponseOut = USB_ResponseOut;
    uint32_t cachedUsbResponseIn = USB_ResponseIn;
    bool cachedUsbResponseFlag = USB_ResponseFlag;
    uint32_t cachedUsbRequestIn = USB_RequestIn;
    uint32_t cachedUsbRequestOut = USB_RequestOut;
    bool cachedUsbRequestFlag = USB_RequestFlag;

    // Process pending requests
    if ((!cachedUsbResponseFlag) && ((cachedUsbRequestOut != cachedUsbRequestIn) || cachedUsbRequestFlag))
    {

        // Process DAP Command and prepare response
        USB_ResponseLen[cachedUsbResponseIn] = DAP_ExecuteCommand(USB_Request[cachedUsbRequestOut], USB_Response[cachedUsbResponseIn]);

        // Update request index and flag
        cachedUsbRequestOut++;
        if (cachedUsbRequestOut == DAP_PACKET_COUNT)
        {
            cachedUsbRequestOut = 0u;
        }

        if (cachedUsbRequestOut == cachedUsbRequestIn)
        {
            USB_RequestFlag = false;
        }
        USB_RequestOut = cachedUsbRequestOut;

        // Update response index and flag
        cachedUsbResponseIn++;
        if (cachedUsbResponseIn == DAP_PACKET_COUNT)
        {
            cachedUsbResponseIn = 0u;
        }
        if (cachedUsbResponseIn == cachedUsbResponseOut)
        {
            USB_ResponseFlag = true;
        }
        USB_ResponseIn = cachedUsbResponseIn;
        USB_ResponseOut = cachedUsbResponseOut;
        USB_RequestIn = cachedUsbResponseIn;

    }
}


// Process USB bulk Data
void usbd_bulk_process(void)
{
    // Load postponed request to buffer
    if (USB_RequestPostponed)
    {
        PushToRequestBuffer();
    }
    /* Init intermediate values for volatile variables USB_ResponseIn, USB_ResponseOut */
    uint32_t cachedUsbResponseOut = USB_ResponseOut;
    uint32_t cachedUsbResponseIn = USB_ResponseIn;
    bool cachedUsbResponseFlag = USB_ResponseFlag;
    uint32_t cachedUsbRequestIn = USB_RequestIn;
    uint32_t cachedUsbRequestOut = USB_RequestOut;
    bool cachedUsbRequestFlag = USB_RequestFlag;
    bool cachedUsbResponseIdle = USB_ResponseIdle;
    bool cachedUsbResponsePostponed = USB_ResponsePostponed;
    // Process pending requests
    if ((!cachedUsbResponseFlag) && ((cachedUsbRequestOut != cachedUsbRequestIn) || cachedUsbRequestFlag))
    {

        // Process DAP Command and prepare response
        USB_ResponseLen[cachedUsbResponseIn] = DAP_ExecuteCommand(USB_Request[(uint16_t)cachedUsbRequestOut], USB_Response[cachedUsbResponseIn]);

        // Update request index and flag
        cachedUsbRequestOut++;
        if (cachedUsbRequestOut == DAP_PACKET_COUNT)
        {
            cachedUsbRequestOut = 0u;
        }

        if (cachedUsbRequestOut == cachedUsbRequestIn)
        {
            cachedUsbRequestFlag = false;
        }

        // Update response index and flag
        cachedUsbResponseIn++;
        if (cachedUsbResponseIn == DAP_PACKET_COUNT)
        {
            cachedUsbResponseIn = 0u;
        }

        if (cachedUsbResponseIn == cachedUsbResponseOut)
        {
            cachedUsbResponseFlag = true;
        }
    }
    USB_ResponseIn = cachedUsbResponseIn;
    USB_ResponseFlag = cachedUsbResponseFlag;

    USB_RequestOut = cachedUsbRequestOut;
    USB_RequestFlag = cachedUsbRequestFlag;

    if (cachedUsbResponsePostponed || (cachedUsbResponseIdle && ((cachedUsbResponseOut != cachedUsbResponseIn) || cachedUsbResponseFlag)))
    {
        // initiate sending data back to the host, if any
        USB_ResponseIdle = false;
        PopFromResponseBuffer();
    }
}

