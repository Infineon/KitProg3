/*************************************************************************//**
* @file main.c
*
* @brief
*  main executable code for KitProg3 bootloader
*
* @version KitProg3 v1.00
*/
/*
* Related Documents:
*   002-22949 - KITPROG3 1.00 EROS
* 	002-23369 - KITPROG2 IROS
* 	002-23558   KITPROG3 1.00 TEST PLAN
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

#include <project.h>
#include <stdint.h>
#include <stdbool.h>
#include "Bootloader.h"

/* Defines for Bootloader checksum validation */
#define Bootloader_GET_CODE_BYTE(addr)     (*((uint8  *)(CYDEV_FLASH_BASE + (addr))))

/* The bootloader always starts at 0 in flash */
#define Bootloader_CHECKSUM_START          (0u)    

/* Delay (debounce) applied to mode button for entering bootloader mode, in msec */
#define MODE_BUTTON_DELAY                  (100u)

/* Constants for appliaction activity flags in the flash metadata */
#define APP_INACTIVE                       (0u)
#define APP_ACTIVE                         (1u)

#define BOOTLOADER_ACTIVE_IDX              (0u)
#define KIT_PROG_ACTIVE_IDX                (1u)
#define CUSTOM_APP_ACTIVE_IDX              (2u)

/* Buttons are resistive pull-Up, so accerted button when low level */
#define BUTTON_ASSERTED				       (0u)



/******************************************************************************
*  Calc8BitFlashSum
***************************************************************************//**
* 
* This function calculates 8-bit checksum for proided flash area.
*
* @param[in]    start    The starting address to start summing data for.
* @param[in]    size     The number of bytes to read and compute the sum for.
*
* @return       The 8 bit sum for the provided data.
*
******************************************************************************/
static uint8_t Calc8BitFlashSum(uint32_t start, uint32_t size)
{
    uint8_t sum = 0u;
    uint32_t cur_size = size;
    while (cur_size > 0u)
    {
        cur_size--;
        sum += Bootloader_GET_CODE_BYTE(start + cur_size);
    }

    return sum;
}

/******************************************************************************
*  ValidateBootloader
***************************************************************************//**
*
* Function to validate the bootloader by verifying the bootloader checksum.
*
* @return TRUE if bootloader is valid FALSE otherwise.
*
******************************************************************************/
static bool ValidateBootloader(void)
{
	uint8_t calcedChecksum;
	calcedChecksum = Calc8BitFlashSum(Bootloader_CHECKSUM_START, 
                    *Bootloader_SizeBytesAccess - Bootloader_CHECKSUM_START);

	/* The checksum was included, so remove it */
	calcedChecksum -= *Bootloader_ChecksumAccess;
	calcedChecksum = 1u + ~calcedChecksum;

	/* Return true if bootloader is corect */
	return (calcedChecksum == *Bootloader_ChecksumAccess);
}

/*******************************************************************************
* main
********************************************************************************
* Handles bootloader operations, normally does not return.
*
*******************************************************************************/
int main()
{
    CyGlobalIntEnable;
    
    /* 
     * Use CySetTemp API to initialize SPC.
     * This step enables writing to flash memory required to change active application number. 
     */
    if ( CySetTemp() != CYRET_SUCCESS )
    {
        CyHalt(0u);
    }    
    
    /* Check the validity of bootloader by verifying the Bootloader checksum */
	if ( !ValidateBootloader() )
    {
        CyHalt(0u);
    }    
    
	/* Check if mode switch is pressed at entry */
	if( ModeSwitchPin_Read() == 0u )
	{
    	uint32_t counter;
        
     	/* Test if mode switch is pressed for more than 100ms */
		for(counter = 0u; counter <= MODE_BUTTON_DELAY; counter++)
		{
			CyDelay(1);

            if( ModeSwitchPin_Read() != BUTTON_ASSERTED )
            {
                break;
            }
		}
        
		/* 
         * If the mode switch was pressed for more than required timeout
         * set the flash run type as bootloader to wait for a bootload operation 
         */
		if(counter >= MODE_BUTTON_DELAY)
		{
            Bootloader_SetFlashByte((uint32) Bootloader_MD_BTLDB_ACTIVE_OFFSET(BOOTLOADER_ACTIVE_IDX), APP_ACTIVE);
            Bootloader_SetFlashByte((uint32) Bootloader_MD_BTLDB_ACTIVE_OFFSET(KIT_PROG_ACTIVE_IDX), APP_INACTIVE);
            Bootloader_SetFlashByte((uint32) Bootloader_MD_BTLDB_ACTIVE_OFFSET(CUSTOM_APP_ACTIVE_IDX), APP_INACTIVE);
            Bootloader_activeApp = BOOTLOADER_ACTIVE_IDX;
            
			/* Set the reset SR0 switch to indicate the */
			Bootloader_SET_RUN_TYPE(Bootloader_START_BTLDR);
		}
	}

	/* Start Bootloader */
	Bootloader_Start();

    for(;;)
    {
    }
}

/* [] END OF FILE */
