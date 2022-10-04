/************************************************************************//**
* @file usbinterface.c
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

#include <stdint.h>
#include "kitprog.h"
#include "usbinterface.h"
#include "USBFS_pvt.h"
#include "mutex.h"

#define WCID_READ_1                     (0x05u)
#define WCID_READ_2                     (0x04u)

/* USB Serial Number String Max. Length = LEN + DTYPE + 2*PREFIX + 2*NUMBER = 40 Bytes */
#define SN_LENGTH_MAX           (0x28u)

/* All USB-related interrupt bitmask */
#define USB_INTERRUPT_MASK ( \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_ARB_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_BUS_RESET_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_DP_INTC_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_EP_0_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_EP_1_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_EP_2_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_EP_3_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_EP_4_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_EP_5_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_EP_6_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_EP_7_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_EP_8_VECT_NUM)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_ord_int__INTC_NUMBER)))) | \
    ((uint32)((uint32)1u << (0x1Fu & (USBFS_SOF_VECT_NUM)))) )

/******************************************************************************
*  USBFS_HandleVendorRqst
***************************************************************************//**
* Handles vendor-specific USB requests, as part of Windows Compatible ID
* implementation.
*
* @return If the request was handled.
*
******************************************************************************/
uint8 USBFS_HandleVendorRqst(void)
{
    uint8_t requestHandled = USBFS_FALSE;

    /* MS OS Configurational Descriptor Bulk, issued only when device is in  */
    /* bulk mode with both programming (#0) and bridge (#1) on bulk endpoints */
    static const uint8_t MSOS_CONFIGURATION_DESCR_BULK[0x40u] = {
    /*  Length of the descriptor 4 bytes               */   0x40u, 0x00u, 0x00u, 0x00u,
    /*  Version of the descriptor 2 bytes              */   0x00u, 0x01u,
    /*  wIndex - Fixed:INDEX_CONFIG_DESCRIPTOR         */   0x04u, 0x00u,
    /*  bCount - Count of device functions.            */   0x02u,
    /*  Reserved : 7 bytes                             */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    /*  bFirstInterfaceNumber                          */   0x00u,
    /*  Reserved, set to 0x01                          */   0x01u,
    /*  WINUSB ID                                      */   (uint8_t)'W', (uint8_t)'I', (uint8_t)'N', (uint8_t)'U', (uint8_t)'S', (uint8_t)'B', 0x00u, 0x00u,
    /*  Secondary ID                                   */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    /*  Reserved                                       */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    /*  bFirstInterfaceNumber                          */   0x01u,
    /*  Reserved, set to 0x01                          */   0x01u,
    /*  WINUSB ID                                      */   (uint8_t)'W', (uint8_t)'I', (uint8_t)'N', (uint8_t)'U', (uint8_t)'S', (uint8_t)'B', 0x00u, 0x00u,
    /*  Secondary ID                                   */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    /*  Reserved                                       */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u
    };

    /* MS OS Configurational Descriptor Hid, issued only when device is in  */
    /* HID mode with only bridge (#1)interface on bulk endpoints */
    static const uint8_t MSOS_CONFIGURATION_DESCR_HID[0x28u] = {
    /*  Length of the descriptor 4 bytes               */   0x28u, 0x00u, 0x00u, 0x00u,
    /*  Version of the descriptor 2 bytes              */   0x00u, 0x01u,
    /*  wIndex - Fixed:INDEX_CONFIG_DESCRIPTOR         */   0x04u, 0x00u,
    /*  bCount - Count of device functions.            */   0x01u,
    /*  Reserved : 7 bytes                             */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    /*  bFirstInterfaceNumber                          */   0x01u,
    /*  Reserved, set to 0x01                          */   0x01u,
    /*  WINUSB ID                                      */   (uint8_t)'W', (uint8_t)'I', (uint8_t)'N', (uint8_t)'U', (uint8_t)'S', (uint8_t)'B', 0x00u, 0x00u,
    /*  Secondary ID                                   */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    /*  Reserved                                       */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    };

    /* MS OS Configurational Descriptor Dual Uart, issued only when device is in */
    /* dual uart mode with only programming (#0) interface on bulk endpoints     */
    static const uint8_t MSOS_CONFIGURATION_DESCR_DUAL_UART[0x28u] = {
    /*  Length of the descriptor 4 bytes               */   0x28u, 0x00u, 0x00u, 0x00u,
    /*  Version of the descriptor 2 bytes              */   0x00u, 0x01u,
    /*  wIndex - Fixed:INDEX_CONFIG_DESCRIPTOR         */   0x04u, 0x00u,
    /*  bCount - Count of device functions.            */   0x01u,
    /*  Reserved : 7 bytes                             */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    /*  bFirstInterfaceNumber                          */   0x00u,
    /*  Reserved, set to 0x01                          */   0x01u,
    /*  WINUSB ID                                      */   (uint8_t)'W', (uint8_t)'I', (uint8_t)'N', (uint8_t)'U', (uint8_t)'S', (uint8_t)'B', 0x00u, 0x00u,
    /*  Secondary ID                                   */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    /*  Reserved                                       */   0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    };

    /* WinUSB descriptor for programming (#0) interface */
    static const uint8_t WINUSB_DESCR_ZERO_INTERFACE[0x8eu] = {
    /*  Descriptor size in bytes (142 bytes)           */   0x8Eu, 0x00u, 0x00u, 0x00u,
    /*  Descriptor version number (1.00)               */   0x00u, 0x01u,
    /*  Extended Compatible ID OS descriptor id        */   0x05u, 0x00u,
    /*  Number of custom property sections that follow */   0x01u, 0x00u,
    /*  Length of custom property section (132 bytes)  */   0x84u, 0x00u, 0x00u, 0x00u,
    /*  String format (UTF-16LE Unicode)               */   0x01u, 0x00u, 0x00u, 0x00u,
    /*  Length of property name (40 bytes)             */   0x28u, 0x00u,
    /*  Property Name (DeviceInterfaceGUID)            */
        (uint8_t)'D', 0u, (uint8_t)'e', 0u, (uint8_t)'v', 0u, (uint8_t)'i', 0u, (uint8_t)'c', 0u, (uint8_t)'e', 0u,
        (uint8_t)'I', 0u, (uint8_t)'n', 0u, (uint8_t)'t', 0u, (uint8_t)'e', 0u, (uint8_t)'r', 0u, (uint8_t)'f', 0u,
        (uint8_t)'a', 0u, (uint8_t)'c', 0u, (uint8_t)'e', 0u, (uint8_t)'G', 0u, (uint8_t)'U', 0u, (uint8_t)'I', 0u,
        (uint8_t)'D', 0u,   0u,         0u,
    /*  Length of property data (78 bytes)             */   0x4eu, 0x00u, 0x00u, 0x00u,
    /*  Vendor-defined property data: {CDB3B5AD-293B-4663-AA36-1AAE46463776} */
        (uint8_t)'{', 0u, (uint8_t)'C', 0u, (uint8_t)'D', 0u, (uint8_t)'B', 0u, (uint8_t)'3', 0u, (uint8_t)'B', 0u,
        (uint8_t)'5', 0u, (uint8_t)'A', 0u, (uint8_t)'D', 0u, (uint8_t)'-', 0u, (uint8_t)'2', 0u, (uint8_t)'9', 0u,
        (uint8_t)'3', 0u, (uint8_t)'B', 0u, (uint8_t)'-', 0u, (uint8_t)'4', 0u, (uint8_t)'6', 0u, (uint8_t)'6', 0u,
        (uint8_t)'3', 0u, (uint8_t)'-', 0u, (uint8_t)'A', 0u, (uint8_t)'A', 0u, (uint8_t)'3', 0u, (uint8_t)'6', 0u,
        (uint8_t)'-', 0u, (uint8_t)'1', 0u, (uint8_t)'A', 0u, (uint8_t)'A', 0u, (uint8_t)'E', 0u, (uint8_t)'4', 0u,
        (uint8_t)'6', 0u, (uint8_t)'4', 0u, (uint8_t)'6', 0u, (uint8_t)'3', 0u, (uint8_t)'7', 0u, (uint8_t)'7', 0u,
        (uint8_t)'6', 0u, (uint8_t)'}', 0u, 0u, 0u
    };

    /* WinUSB descriptor for bridge (#1) interface */
    static const uint8_t WINUSB_DESCR_FIRST_INTERFACE[0x8eu] = {
    /*  Descriptor size in bytes (142 bytes)           */   0x8Eu, 0x00u, 0x00u, 0x00u,
    /*  Descriptor version number (1.00)               */   0x00u, 0x01u,
    /*  Extended Compatible ID OS descriptor id        */   0x05u, 0x00u,
    /*  Number of custom property sections that follow */   0x01u, 0x00u,
    /*  Length of custom property section (132 bytes)  */   0x84u, 0x00u, 0x00u, 0x00u,
    /*  String format (UTF-16LE Unicode)               */   0x01u, 0x00u, 0x00u, 0x00u,
    /*  Length of property name (40 bytes)             */   0x28u, 0x00u,
    /*  Property Name (DeviceInterfaceGUID)            */
        (uint8_t)'D', 0u, (uint8_t)'e', 0u, (uint8_t)'v', 0u, (uint8_t)'i', 0u, (uint8_t)'c', 0u, (uint8_t)'e', 0u,
        (uint8_t)'I', 0u, (uint8_t)'n', 0u, (uint8_t)'t', 0u, (uint8_t)'e', 0u, (uint8_t)'r', 0u, (uint8_t)'f', 0u,
        (uint8_t)'a', 0u, (uint8_t)'c', 0u, (uint8_t)'e', 0u, (uint8_t)'G', 0u, (uint8_t)'U', 0u, (uint8_t)'I', 0u,
        (uint8_t)'D',0u,0u,0u,
    /*  Length of property data (78 bytes)             */   0x4eu, 0x00u, 0x00u, 0x00u,
    /*  Vendor-defined property data: {88BAE032-5A81-49f0-BC3D-A4FF138216D6} */
        (uint8_t)'{', 0u, (uint8_t)'8', 0u, (uint8_t)'8', 0u, (uint8_t)'B', 0u, (uint8_t)'A', 0u, (uint8_t)'E', 0u,
        (uint8_t)'0', 0u, (uint8_t)'3', 0u, (uint8_t)'2', 0u, (uint8_t)'-', 0u, (uint8_t)'5', 0u, (uint8_t)'A', 0u,
        (uint8_t)'8', 0u, (uint8_t)'1', 0u, (uint8_t)'-', 0u, (uint8_t)'4', 0u, (uint8_t)'9', 0u, (uint8_t)'F', 0u,
        (uint8_t)'0', 0u, (uint8_t)'-', 0u, (uint8_t)'B', 0u, (uint8_t)'C', 0u, (uint8_t)'3', 0u, (uint8_t)'D', 0u,
        (uint8_t)'-', 0u, (uint8_t)'A', 0u, (uint8_t)'4', 0u, (uint8_t)'F', 0u, (uint8_t)'F', 0u, (uint8_t)'1', 0u,
        (uint8_t)'3', 0u, (uint8_t)'8', 0u, (uint8_t)'2', 0u, (uint8_t)'1', 0u, (uint8_t)'6', 0u, (uint8_t)'D', 0u,
        (uint8_t)'6', 0u, (uint8_t)'}', 0u, 0u, 0u,
    };
    uint8_t mode = currentMode;

    if (0u != ((CY_GET_REG8(USBFS_bmRequestType)) & USBFS_RQST_DIR_D2H))
    {
        switch (CY_GET_REG8(USBFS_wIndexLo))
        {
        case WCID_READ_1:
            if (((CY_GET_REG8(USBFS_wValueLo)) == 0x00u) &&
                ((mode == MODE_BULK) || (mode == MODE_BULK2UARTS)))
            {
                USBFS_currentTD.pData = (uint8_t *) &WINUSB_DESCR_ZERO_INTERFACE[0u];
                USBFS_currentTD.count = WINUSB_DESCR_ZERO_INTERFACE[0u];
                requestHandled  = USBFS_InitControlRead();
            }
            else
            {
                if ((CY_GET_REG8(USBFS_wValueLo)) == 0x01u)
                {
                    USBFS_currentTD.pData = (uint8_t *) &WINUSB_DESCR_FIRST_INTERFACE[0u];
                    USBFS_currentTD.count = WINUSB_DESCR_FIRST_INTERFACE[0u];
                    requestHandled  = USBFS_InitControlRead();
                }
            }
            break;

        case WCID_READ_2:
            if (mode == MODE_BULK)
            {
                USBFS_currentTD.pData = (uint8_t *) &MSOS_CONFIGURATION_DESCR_BULK[0u];
                USBFS_currentTD.count = MSOS_CONFIGURATION_DESCR_BULK[0u];
                requestHandled  = USBFS_InitControlRead();
            }
            else if (mode == MODE_HID)
            {
                USBFS_currentTD.pData = (uint8_t *) &MSOS_CONFIGURATION_DESCR_HID[0u];
                USBFS_currentTD.count = MSOS_CONFIGURATION_DESCR_HID[0u];
                requestHandled  = USBFS_InitControlRead();
            }
            else if (mode == MODE_BULK2UARTS)
            {
                USBFS_currentTD.pData = (uint8_t *) &MSOS_CONFIGURATION_DESCR_DUAL_UART[0u];
                USBFS_currentTD.count = MSOS_CONFIGURATION_DESCR_DUAL_UART[0u];
                requestHandled  = USBFS_InitControlRead();
            }
            else
            {
                /* Not supported mode */
            }
            break;

        default:
            requestHandled = USBFS_FALSE;
            break;
        }
    }



    return (requestHandled);
}

/******************************************************************************
*  GenerateUsbSerialNumberString
***************************************************************************//**
* This function fills parameter string descriptor with unique values regarding
* to die ID.
* This number is 16-digit number in hexadecimal string format
* e.g. 1F19172E03242400.
*
* @param[in] descr The pointer to the memory for storing the string
*
******************************************************************************/
static void GenerateUsbSerialNumberString(uint8_t * descr)
{
    static const char8 CYCODE hex[16u] = "0123456789ABCDEF";
    /* Check descriptor validation */
    if (descr != NULL)
    {
        descr[0] = ASCII_DOUBLE_CLAWS;
        descr[1] = USBFS_DESCR_STRING;

        /* put unique die ID into descriptor following string descriptor type format */
        for (uint32_t sym = 0u; sym < DIE_ID_LEN; sym++)
        {
            uint8_t idByte;

            idByte = CY_GET_XTND_REG8((void CYFAR *)(USBFS_DIE_ID + sym));
            descr[2u + (sym*4u)] = (uint8_t)hex[idByte >> 4u];
            descr[3u + (sym*4u)] = 0u;
            descr[4u + (sym*4u)] = (uint8_t)hex[idByte & 0x0Fu];
            descr[5u + (sym*4u)] = 0u;
        }
    }
}

/******************************************************************************
*  PrepareUsbInterface
***************************************************************************//**
* This function generate unique serial number then pass it to the call-back
* function.
*
******************************************************************************/
void PrepareUsbInterface(void)
{
    /* USB Serial Number String */
    static uint8_t snString[SN_LENGTH_MAX];
    /* Generate unique serial number string */
    GenerateUsbSerialNumberString(snString);

    /* Pass serial number string to the call-back function. */
    USBFS_SerialNumString(snString);
}

/******************************************************************************
*  PushToRequestBuffer
***************************************************************************//**
* read CMSIS-DAP packet from OUT EP then place it to head of CMSIS-DAP buffer
*
******************************************************************************/
void PushToRequestBuffer(void)
{
    if (TryLock(&USB_Mutex))
    {
        if (USB_RequestPostponed)
        {
            /* clear request postponed flag */
            USB_RequestPostponed = false;
        }
        bool cachedUsbRequestBufferFull = USB_RequestBufferFull;
        if ((USBFS_GetEPAckState(CMSIS_BULK_OUT_EP) != 0u) && (!cachedUsbRequestBufferFull))
        {
            /* ACKed transaction and buffer is not overflow */
            usbDapReadFlag = true;
            uint32_t intrMask = CyUsbIntDisable();
            uint16_t receiveSize = USBFS_ReadOutEP(CMSIS_BULK_OUT_EP, USB_Request[USB_RequestIn], DAP_PACKET_SIZE);
            CyUsbIntEnable(intrMask);
            if (receiveSize != 0u)
            {
                if (USB_Request[USB_RequestIn][0] == ID_DAP_TransferAbort)
                {
                    DAP_TransferAbort = 1u;
                }
                else
                {
                    USB_RequestIn++;
                    if (USB_RequestIn == DAP_PACKET_COUNT)
                    {
                        USB_RequestIn = 0u;
                    }
                    if (USB_RequestIn == USB_RequestOut)
                    {
                        /* Request buffer full */
                        USB_RequestBufferFull = true;
                    }
                }
            }
        }
        else
        {
            /* transaction not yet ACKed or there is no free space in request buffer */
            USB_RequestPostponed = true;
        }
        Unlock(&USB_Mutex);
    }
    else
    {
        /* transaction was postponed due to mutex lock */
        USB_RequestPostponed = true;
    }
}

/******************************************************************************
*  PopFromResponseBuffer
***************************************************************************//**
* read packet from response buffer then place CMSIS-DAP packet to IN EP
*
******************************************************************************/
void PopFromResponseBuffer(void)
{
    if (TryLock(&USB_Mutex))
    {
        if (USB_ResponsePostponed)
        {
            /* clear response postponed flag */
            USB_ResponsePostponed = false;
        }
        if (USBFS_GetEPState(CMSIS_BULK_IN_EP) == USBFS_IN_BUFFER_EMPTY)
        {
            bool cachedUsbRequestBufferFull = USB_RequestBufferFull;
            if ((USB_ResponseOut != USB_ResponseIn) || cachedUsbRequestBufferFull)
            {
                USB_ResponseIdle = false;
                uint32_t intrMask = CyUsbIntDisable();
                USBFS_LoadInEP(CMSIS_BULK_IN_EP, USB_Response[USB_ResponseOut], (uint16_t)(USB_ResponseLen[USB_ResponseOut]));
                CyUsbIntEnable(intrMask);
                uint32_t n = USB_ResponseOut + 1u;
                if (n == DAP_PACKET_COUNT)
                {
                    n = 0u;
                }
                USB_ResponseOut = n;
                if (cachedUsbRequestBufferFull)
                {
                    /* clear overflow flag */
                    USB_RequestBufferFull = false;
                }
            }
            else
            {
                USB_ResponseIdle = true;
            }
        }
        else
        {
            /* transaction was postponed due EP is still not empty */
            USB_ResponsePostponed = true;
        }
        Unlock(&USB_Mutex);
    }
    else
    {
        /* transaction was postponed due to mutex lock */
        USB_ResponsePostponed = true;
    }
}

/*******************************************************************************
* CyUsbIntEnable
********************************************************************************
* Enables all USB-relates interrupts at once
*
* @param[in] USB interrupt bitmask for enable
*
*******************************************************************************/
void CyUsbIntEnable(uint32_t intrMask)
{
    CY_SET_REG32(CY_INT_ENABLE_PTR, USB_INTERRUPT_MASK & intrMask);
}

/*******************************************************************************
* CyUsbIntDisable
********************************************************************************
* Disables all USB-relates interrupts at once
*
* returns prevous enabled USB interrupt bitmap
*
*******************************************************************************/
uint32_t CyUsbIntDisable(void)
{
    uint32_t intrMask = CY_GET_REG32(CY_INT_CLEAR_PTR) & USB_INTERRUPT_MASK;
    CY_SET_REG32(CY_INT_CLEAR_PTR, intrMask);
    return intrMask;
}

/* [] END OF FILE */
