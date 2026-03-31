/*************************************************************************//**
* @file kitprog.h
*
* @brief
*   This file contains the function prototypes and constants used in
*   main.c
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
* (c) (2018-2026), Infineon Technologies AG, 
* or an affiliate of Infineon Technologies AG. All rights reserved.
*
* This software, associated documentation and materials ("Software") is 
* owned by Infineon Technologies AG or one of its 
* affiliates ("Infineon") and is protected by and subject to worldwide 
* patent protection, worldwide copyright 
* laws, and international treaty provisions. Therefore, you may use this 
* Software only as provided in the license agreement accompanying the 
* software package from which you obtained this Software. If 
* no license agreement applies, then any use, reproduction, modification, 
* translation, or compilation of this Software is prohibited without 
* the express written permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS 
* SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF 
* NON-INFRINGEMENT OF THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH 
* AS WARRANTIES OF FITNESS FOR A SPECIFIC USE/PURPOSE OR MERCHANTABILITY. 
* Infineon reserves the right to make changes to the Software without notice. 
* You are responsible for properly designing, programming, and testing the 
* functionality and safety of your intended application of the Software, 
* as well as complying with any legal requirements related to its use. 
* Infineon does not guarantee that the Software will be free from 
* intrusion, data theft or loss, or other breaches (“Security Breaches”), 
* and Infineon shall have no liability arising out of any Security Breaches. 
* Unless otherwise explicitly approved by Infineon, the Software may not be 
* used in any application where a failure of the Product or any consequences 
* of the use thereof can reasonably be expected to result in personal injury.
*****************************************************************************/

#if !defined(KITPROG_H)
#define KITPROG_H

#include <stdbool.h>
#include <stdint.h>

/* USB endpoint usage */
#define CMSIS_HID_IN_EP                 (0x01u)
#define CMSIS_HID_OUT_EP                (0x02u)

#define CMSIS_BULK_IN_EP                (0x02u)
#define CMSIS_BULK_OUT_EP               (0x01u)

#define HID_REPORT_INPUT                (0x01u)
#define HID_REPORT_FEATURE              (0x02u)
#define HID_REPORT_OUTPUT               (0x03u)
#define USBD_HID_REQ_EP_INT             (0x04u)
#define USBD_HID_REQ_EP_CTRL            (0x05u)
#define USBD_HID_REQ_PERIOD_UPDATE      (0x06u)

/* ASCII symbols */
#define ASCII_DOUBLE_CLAWS              0x22u
#define ASCII_OPENING_BRACKET           0x28u

/*****************************************************************************
* MACRO Definition
*****************************************************************************/
#define IDLE                        (0x00u)
#define BUSY                        (0x02u)
#define ERROR                       (0xFFu)

/* USB endpoint usage */
#define HOST_IN_EP                  (0x01u)
#define HOST_OUT_EP                 (0x02u)
#define UART_INT_EP                 (0x05u)
#define UART_IN_EP                  (0x06u)
#define UART_OUT_EP                 (0x07u)

/* KitProg3 modes */
#define MODE_BULK                   (0x00u)
#define MODE_HID                    (0x01u)
#define MODE_BULK2UARTS             (0x02u)

/* Corresponding with mode and HWID USBFS devices */
#define MODE_KP3_BULK               (0x00u)
#define MODE_KP3_HID                (0x01u)
#define MODE_KP3_BULK2UARTS         (0x02u)
#define MODE_MP_BULK                (0x03u)
#define MODE_MP_HID                 (0x04u)
#define MODE_BULK2UARTS_HCI_PERI    (0x05u)
#define MODE_BULK_HCI_PERI          (0x06u)
#define MODE_HID_HCI_PERI           (0x07u)

#define USB_WAIT_FOR_VBUS           (0x01u)
#define USB_START_COMPONENT         (0x02u)
#define USB_WAIT_FOR_CONFIG         (0x03u)
#define USB_CONFIGURED              (0x04u)
#define USB_READY_BULK              (0x05u)
#define USB_READY_HID               (0x06u)
#define USB_HALT                    (0x07u)

#define DEVICE_ACQUIRE              (0x01u)
#define VERIFY_SILICON_ID           (0x02u)
#define ERASE_ALL_FLASH             (0x03u)
#define CHECKSUM_PRIVILIGED         (0x04u)
#define PROGRAM_FLASH               (0x05u)
#define VERIFY_FLASH                (0x06u)
#define PROGRAM_PROT_SETTINGS       (0x07u)
#define VERIFY_PROT_SETTINGS        (0x08u)
#define VERIFY_CHECKSUM             (0x09u)

#define ONE_MS_DELAY                (0x01u)
#define MODE_SWITCHING_TIMEOUT      (3000u)

#define STATUS_MSG_SIZE             (60u)
#define MSG_NUM_HEX_NOT_VALID       (10u)
#define MSG_NUM_SFLASH_SROM_ERROR   (11u)

/* Data shift */
#define SHIFT_28                    (28u)
#define SHIFT_20                    (20u)
#define SHIFT_12                    (12u)
#define SHIFT_4                     (4u)
#define SHIFT_2                     (2u)

#define BYTE_NIBBLE_BYTE            (0x0000000Fu)

/* DWT Program Counter Sample register */
#define DWT_PC_SAMPLE_ADDR          CYDEV_DWT_PC_SAMPLE

/* Slot#2 start address of PSoC5lp flash */
/* DAPLink Begins in Last 32 Kilobytes of KitProg3 Image */
#define SLOT2_BASE_ADDR            (uint32_t*)(0x00021800u - (1024u*32u))

#define DIE_ID_LEN                      0x08u

/* inline macros */
#ifndef   __STATIC_FORCEINLINE
  #define __STATIC_FORCEINLINE                   __attribute__((always_inline)) static inline
#endif

/*****************************************************************************
* Global Variable Declaration
*****************************************************************************/
extern volatile uint8_t currentMode;
extern volatile uint8_t currentFirstUartControl;
extern volatile uint8_t currentSecondUartControl;
extern volatile uint8_t currentFirstUartMode;
extern volatile bool usbResetDetected;
extern bool usbDapReadFlag;
extern volatile bool gpioChanged;

#define DEFAULT_UART_FLOW_CONTROL      (0x00u)
#define UART_NO_FLOW_CONTROL           (0x01u)
#define UART_HW_FLOW_CONTROL           (0x02u)
#define DEFAULT_UART_MODE              (0x00u)
#define UART_FULL_DUPLEX_MODE          (0x01u)
#define UART_HALF_DUPLEX_MODE          (0x02u)
    
/*****************************************************************************
* External Function Prototypes
*****************************************************************************/
void usbd_hid_init(void);
uint32_t usbd_hid_get_report(uint8_t rtype, uint8_t rid, uint8_t *buf, uint8_t req);
void usbd_hid_set_report(uint8_t rtype, uint8_t rid, uint8_t *buf, uint8_t len, uint8_t req);
void usbd_hid_process(void); /* Function from the official ARM library. */
void usbd_bulk_process(void); /* Function for handling CMSIS_DAP 2.0 */

#endif /* KITPROG_H */
