/*************************************************************************//**
* @file kitprog.h
*
* @brief
*   This file contains the function prototypes and constants used in
*   main.c
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

#define MODE_BULK                   (0x00u)
#define MODE_HID                    (0x01u)
#define MODE_BULK2UARTS             (0x02u)
#define MODE_MP_BULK                (0x03u)
#define MODE_MP_HID                 (0x04u)

#define MODE_MP4                    (0x02u)
#define MODE_MP4_CMSISDAP           (0x03u)

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
extern volatile bool usbResetDetected;
extern bool usbDapReadFlag;
extern volatile bool gpioChanged;

/*****************************************************************************
* External Function Prototypes
*****************************************************************************/
void usbd_hid_init(void);
uint32_t usbd_hid_get_report(uint8_t rtype, uint8_t rid, uint8_t *buf, uint8_t req);
void usbd_hid_set_report(uint8_t rtype, uint8_t rid, uint8_t *buf, uint8_t len, uint8_t req);
void usbd_hid_process(void); /* Function from the official ARM library. */
void usbd_bulk_process(void); /* Function for handling CMSIS_DAP 2.0 */

#endif /* KITPROG_H */
