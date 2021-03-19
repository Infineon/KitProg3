/*************************************************************************//**
* @file main.c
*
* @brief
*  main executable code for KitProg3
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

#include <stdint.h>
#include <stdbool.h>
#include "app_switch.h"
#include "swd.h"
#include "DAP.h"
#include "device.h"
#include "kitprog.h"
#include "led.h"
#include "power.h"
#include "version.h"
#include "bridgesInterface.h"
#include "usbinterface.h"

volatile uint8_t currentMode = MODE_HID;

/* DAPLink related definitions */
#define DAPLINK_BUILD_KEY        (0x9B939E8Fu)
#define DAPLINK_HIC_ID           (0x2E127069u)
#define DAPLINK_BUILD_KEY_OFFSET (0x08u)
#define DAPLINK_HIC_ID_OFFSET    (0x09u)


/* Buttons are resistive pull-Up, so accerted button when low level */
#define BUTTON_ASSERTED                 (0u)

/* Variable to identify if a USB bus reset has occurred. Set to TRUE in the USBFS_episr.c auto-generated file. */
volatile bool usbResetDetected  = false;

bool usbDapReadFlag = false; /* This global variable is set in USBFS_episr.c to signal EP2 interrupt. */


/*******************************************************************************
* main
********************************************************************************
* Handles KitProg3 initialisation, normally does not return.
*
*******************************************************************************/
int main(void)
{
    /* Enable global Interrupts */
    CyGlobalIntEnable;

    /* Start Time source timer as soon as possible */
    Timer_CSTick_Start();

    /* Retrieves the Hardware version. Defined in version.c. */
    System_Init();

    /* Disable SPI for kits without SPI support */
    /* TODO move code below to SPI initialization */
    if ( !KitHasSpiBridge() )
    {
        SPI_SS_0_SetDriveMode(SPI_SS_0_DM_DIG_HIZ);
        SPI_SS_1_SetDriveMode(SPI_SS_1_DM_DIG_HIZ);
        CyPins_SetPinDriveMode(GPIO_SPI_SS_2_GPIO_1, CY_PINS_DM_DIG_HIZ);
        SPI_MOSI_SetDriveMode(SPI_MOSI_DM_DIG_HIZ);
        SPI_SCLK_SetDriveMode(SPI_SCLK_DM_DIG_HIZ);
        SPI_MISO_SetDriveMode(SPI_MISO_DM_DIG_HIZ);
    }

    /* If CTS is not connected set the pin to Resistive Pull Down in order to avoid noise impact */
    UartCtsRtsPinInit(); /* CDT 248428 */

    Bridge_PrepareI2cInterface();

    if (KitHasSpiBridge())
    {
        Bridge_PrepareSpiInterface();
    }

    if (KitHasGpioBridge())
    {
        Bridge_PrepareGpioInterface();
    }
    
    Bridge_PrepareUartInterface();

    /* Initializes SWD interface */
    Swd_Init();

    /* Flag to determine if the USB is reset */
    usbResetDetected = false;

    /* Generate unique serial number */
    PrepareUsbInterface();

    /* Check DAPLink availability */
    const uint32_t *customAppSlot = SLOT2_BASE_ADDR;
    bool customAppIsDapLink = (customAppSlot[DAPLINK_BUILD_KEY_OFFSET] == DAPLINK_BUILD_KEY) &&
                         (customAppSlot[DAPLINK_HIC_ID_OFFSET] == DAPLINK_HIC_ID);

    /***********************************************************************************/
    /********************* Alternative Modes Initialization ******************************/
    /***********************************************************************************/
    /* EEPROM is used for the sticky functionality - the firmware remembers the last mode
     * it was in using the data in the EEPROM. By default, the FW starts up in KitProg mode. */

    /* TODO shall be in System_Init() */
    EEPROM_ModeStorage_Start();

    /* Provide a 10us delay for EEPROM data to stabilize.
     * Datasheet says 5us, set to 10us just to be sure. */
    CyDelayUs(10u);

    /* EEPROM reads are direct reads and do not require an API. Set the base address and read. */
    const reg8 * modeAddress = (reg8 *)CYDEV_EE_BASE;

    /* Copy the value to SRAM variable. This variable is maintained
     * and manipulated during the firmware operation. */
    currentMode = modeAddress[0u];

    /* Check valid values for mode */
    if (currentMode > MODE_BULK2UARTS)
    {
        /* In case if EEPROM was erased or stored value is invalid - default to BULK mode*/
        currentMode = MODE_BULK;
    }
    else if ((currentMode == MODE_BULK2UARTS) && (!KitHasSecondaryUart()))
    {
        /* In case if kit do not have 2 UARTs but for some reason stored mode has this feature */
        currentMode = MODE_BULK;
    }
    else
    {
        /* Do nothing: mode value is acceptable */
    }

    /***********************************************************************************/
    /**************** End of the Alternative Modes Initialization ************************/
    /***********************************************************************************/

    uint8_t currState = USB_WAIT_FOR_VBUS;
    
    while(true)
    {
        /* If user presses the Mode Button the firmware must switch modes. */
        if (ModeButton_Read() == BUTTON_ASSERTED)
        {
            uint32_t appSwitchCounter = 0u;
            while (ModeButton_Read() == BUTTON_ASSERTED)
            {
                CyDelay(1u);
                appSwitchCounter++;
                if (appSwitchCounter > TWO_UARTS_SWITCH_TIMEOUT)
                {
                    appSwitchCounter = TWO_UARTS_SWITCH_TIMEOUT;
                }
            }

            if ((appSwitchCounter > MODE_SWITCH_TIMEOUT) && (appSwitchCounter <= TWO_UARTS_SWITCH_TIMEOUT))
            {
                if ((appSwitchCounter == TWO_UARTS_SWITCH_TIMEOUT) && KitHasSecondaryUart())
                {
                    if (currentMode != MODE_BULK2UARTS)
                    {
                        currentMode = MODE_BULK2UARTS;
                    }
                    else
                    {
                        currentMode = MODE_BULK;
                    }
                }
                else if (appSwitchCounter < TWO_UARTS_SWITCH_TIMEOUT)
                {
                    if (currentMode == MODE_BULK)
                    {
                        if (customAppIsDapLink)
                        {
                            /* switch to DAPLink instead of switch to BULK */
                            /* write BULK mode to EEPROM - avoid loop in HID if DAPLink is damaged */
                            (void)EEPROM_ModeStorage_ByteWrite(MODE_BULK, 0u, 0u);
                            /* Set 2nd Application Active (Custom Application) */
                            (void)Bootloadable_SetActiveApplication(CUSTOM_APP_ID);
                            EEPROM_ModeStorage_Stop();
                            /* Provide a 20us delay to ensure that the SPC has completed the write operation. */
                            CyDelayUs(20u);
                            CySoftwareReset();
                        }
                    }
                    else if (currentMode == MODE_HID)
                    {
                        currentMode = MODE_BULK;
                    }
                    else
                    {
                        /* Do nothing */
                    }
                }
                else
                    {
                        /**/
                    }
            }

            /* Write the current mode setting to the EEPROM. */
            if (CySetTemp() == CYRET_SUCCESS)
            {
                (void)EEPROM_ModeStorage_ByteWrite(currentMode, 0u, 0u);
            }

            /* Force reset the state-machine. */
            currState = USB_HALT;
        }

        /* State machine which handles all the USB events, KitProg functionality,
         * and mass storage programming functionality. */
        switch(currState)
        {
            case USB_WAIT_FOR_VBUS:
            {
                /* Wait till powered via USB */
                if(USBFS_VBusPresent() != 0u)
                {
                    currState = USB_START_COMPONENT;
                }
                break;
            }
            case USB_START_COMPONENT:
            {
                /* Start USB operation at VDDD specified in the DWR */
                if (currentMode == MODE_BULK)
                {
                    if (KitIsMiniProg())
                    {
                        USBFS_Start(MODE_MP_BULK, USBFS_DWR_VDDD_OPERATION);
                    }
                    else
                    {
                        USBFS_Start(MODE_BULK, USBFS_DWR_VDDD_OPERATION);
                    }
                }
                else
                {
                    if (KitIsMiniProg())
                    {
                        USBFS_Start(MODE_MP_HID, USBFS_DWR_VDDD_OPERATION);
                    }
                    else
                    {
                        USBFS_Start(currentMode, USBFS_DWR_VDDD_OPERATION);
                    }
                }

                /* Initialize the power subsystem. */
                Power_Init();

                currState = USB_WAIT_FOR_CONFIG;
                break;
            }
            case USB_WAIT_FOR_CONFIG:
            {
                /* Wait for configuration from host. */
                if (USBFS_GetConfiguration() != 0u)
                {
                    if (currentMode != MODE_BULK2UARTS)
                    {
                        USBFS_EnableOutEP(BRIDGE_INTERFACE_OUT_ENDP);
                    }

                    while (USBFS_GetDeviceAddress() == 0u)
                    {
                        /* Wait cycle to get Device Address */
                    }

                    Led_Init();

                    if (currentMode == MODE_HID)
                    {
                        /* Arm the Endpoint. */
                        USBFS_EnableOutEP(CMSIS_HID_OUT_EP);

                        Led_SetState(LED_CMSIS_HID_READY);

                        currState = USB_READY_HID;
                    }
                    else
                    {
                        /* Enable USB out end points for CMSIS-DAP 2.0 */
                        USBFS_EnableOutEP(CMSIS_BULK_OUT_EP);

                        if (currentMode == MODE_BULK)
                        {
                            Led_SetState(LED_CMSIS_BULK_READY);
                        }
                        else
                        {
                            Led_SetState(LED_CMSIS_BULK_2_READY);
                        }

                        currState = USB_READY_BULK;
                    }

                    UsbUartStart();

                    /* 'usbResetDetected ' flag is set to false to indicate that the USB is
                     * connected and configured. This flag will be set inside USBFS_episr.c if
                     * the USB Bus Reset is detected */
                    usbResetDetected  = false;

                    /* Initialize USB hid state machine status and global flags. */
                    usbDapReadFlag = false;
                    usbd_hid_init();

                    /* set up the DAP_Data and SWD pins */
                    DAP_Setup();

                }
                break;
            }
            case USB_READY_BULK:
            {
                /* Handle CMSIS-DAP BULK requests */
                if (usbDapReadFlag)
                {
                    usbDapReadFlag = false;
                }

                usbd_bulk_process();

                /* Handle I2C/SPI Bridge requests */
                if (currentMode != MODE_BULK2UARTS)
                {
                    Bridge_InterfaceHandler();
                }

                /* Handle USB-UART data */
                Bridge_UartInterfaceHandler();

                /* Handle USB configurations change */
                /* If the host has send an new configuration, exit state machine to get reconfigured. */
                /* Using var cur_usbResetDetected to get current value of volatile usbResetDetected */
                bool curUsbResetDetected = usbResetDetected;
                if((USBFS_IsConfigurationChanged() != 0u) || curUsbResetDetected)
                {
                    Led_SetState(LED_ALL_OFF);
                    currState = USB_WAIT_FOR_CONFIG;
                }
                /* Loop is executed as long as the Vbus is not removed */
                if(USBFS_VBusPresent() == 0u)
                {
                    Led_SetState(LED_ALL_OFF);
                    /* turn off CMSIS-DAP port */
                    PORT_OFF();
                    currState = USB_HALT;
                }

                break;
            }
            case USB_READY_HID:
            {
                /* Handle CMSIS-DAP HID requests */
                if (usbDapReadFlag)
                {
                    usbDapReadFlag = false;
                    static uint8_t readbuffer[DAP_PACKET_SIZE];
                    uint8_t receivesize = (uint8_t)(USBFS_GetEPCount(CMSIS_HID_OUT_EP));
                    (void)USBFS_ReadOutEP(CMSIS_HID_OUT_EP, readbuffer, receivesize);
                    usbd_hid_set_report(HID_REPORT_OUTPUT, 1u, readbuffer, receivesize, 1u);
                }

                usbd_hid_process();

                if (USBFS_GetEPState(CMSIS_HID_IN_EP) == USBFS_IN_BUFFER_EMPTY)
                {
                    /* Send response if IN endpoint is empty */
                    static uint8_t writebuffer[DAP_PACKET_SIZE];
                    uint32_t discard = usbd_hid_get_report(HID_REPORT_INPUT, 1u, writebuffer, USBD_HID_REQ_EP_INT);
                    if (discard != 0u)
                    {
                        USBFS_LoadInEP(CMSIS_HID_IN_EP, writebuffer, DAP_PACKET_SIZE);
                    }
                }

                /* Handle I2C/SPI Bridge requests */
                Bridge_InterfaceHandler();

                /* Handle USB-UART data */
                Bridge_UartInterfaceHandler();

                /* Handle USB configurations change */
                bool cachedUsbResetDetected = usbResetDetected;
                /* If the host has send an new configuration, exit state machine to get reconfigured. */
                if((USBFS_IsConfigurationChanged() != 0u) || cachedUsbResetDetected)
                {
                    Led_SetState(LED_ALL_OFF);
                    currState = USB_WAIT_FOR_CONFIG;
                }
                /* Loop is executed as long as the Vbus is not removed */
                if(USBFS_VBusPresent() == 0u)
                {
                    Led_SetState(LED_ALL_OFF);
                    /* turn off SMSIS-DAP port */
                    PORT_OFF();
                    currState = USB_HALT;
                }
                break;
            }
            case USB_HALT:
            {
                /* If the Vbus is removed, stop the component and try to re-enumerate.
                * Set to true inside USBFS_episr.c [CY_ISR(USBFS_BUS_RESET_ISR)] */
                USBFS_Stop();
                USBFS_Force(USBFS_FORCE_SE0);
                /* USB spec. requires at least 10ms of SE0 state */
                CyDelay(20u);
                USBFS_Force(USBFS_FORCE_NONE);
                /* Reset the main state machine. */
                currState = USB_WAIT_FOR_VBUS;

                while ( ModeButton_Read() == BUTTON_ASSERTED)
                {
                     /* Wait till the user releases the Application buttons of kit. This is to
                     * protect from unintentional back to back mode switching. */
                }

                if ( KitHasTwoButtons() )
                {
                    while (ApplicationButton_Read() == BUTTON_ASSERTED)
                    {
                     /* Wait for user to release buttons*/
                    }
                }

                break;
            }
            default:
            {
                /*Impossible situation*/
                break;
            }
        }
    }
}


/* [] END OF FILE */
