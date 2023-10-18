/******************************************************************************
* File Name:   main.c
*
Description: This is the source code for Code Flash Sector Data Updating 
*            Example using HAL APIs.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
 
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/* Select flash address to be tested.  */
#define TEST_TARGET_ADDR    (CY_FLASH_SM_SBM_END - CY_CODE_SES_SIZE_IN_BYTE)
#define TEST_TARGET_SIZE    CY_FLASH_SIZEOF_ROW

#define TEST_EMPTY_ADDR     (CY_FLASH_SM_SBM_END - CY_CODE_SES_SIZE_IN_BYTE*2)
#define TEST_EMPTY_SIZE     CY_FLASH_SIZEOF_ROW
/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* Main function
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    uint32_t index;
    cy_rslt_t     result;
    cyhal_flash_t cyhal_flash;
    uint8_t write_buff[CY_FLASH_SIZEOF_ROW];
    uint8_t read_buff[CY_FLASH_SIZEOF_ROW];

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, \
                                 CY_RETARGET_IO_BAUDRATE);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    printf("\r\n__________________________________________________________________________\r\n*\t\t"\
            "CE234963 HAL: Code Flash Sector Data Updating\r\n*\r\n*\tThis code example"\
            "shows how to update data for code flash sector\r\n*\t"\
            "UART Terminal Settings: Baud Rate - 115200 bps, 8N1\r\n*"\
            "\r\n__________________________________________________________________________\r\n\n");

    /* 1.Initialize code flash. */
    /* Initialize the data in RAM that will be written into target flash sector*/
    for(index = 0; index < TEST_TARGET_SIZE; index++)
    {
        write_buff[index] = (uint8_t)index;
    }

    /* Initialization */
    result = cyhal_flash_init(&cyhal_flash);
    /* flash init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    /*Enable code flash write function */
    Cy_Flashc_MainWriteEnable();
    printf("[1] Code flash Init success! \r\n\n");

    /* 2.Erase Target sector and Program source data. */
    printf("\r__________________________________________________________________________\r\n\n");
    printf("[2] Program source data into Target sector \r\n");

    printf("Target sector erase start.. \r\n");
    /* Erase code flash target sector */
    cyhal_flash_erase(&cyhal_flash, TEST_TARGET_ADDR);
    
    for(uint32_t PageNum = 0; PageNum < (CY_CODE_SES_SIZE_IN_BYTE/CY_FLASH_SIZEOF_ROW); PageNum++)
    {
        memset(read_buff, 0, sizeof(read_buff));
        cyhal_flash_read(&cyhal_flash, TEST_TARGET_ADDR + (PageNum * CY_FLASH_SIZEOF_ROW), read_buff, TEST_TARGET_SIZE);
        /* Verify erase state */
        for(uint32_t i = 0; i < CY_FLASH_SIZEOF_ROW; i++)
        {
           if(read_buff[i] != 0xFF)
           {
               CY_ASSERT(0);
           }
        }
    }
    printf("Target sector erase finished! \r\n\n");

    printf("Target sector program start.. \r\n");
    /* Programming code flash target sector */
    result = cyhal_flash_program(&cyhal_flash, TEST_TARGET_ADDR, (uint32_t*)write_buff);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Verify code flash write success */
    memset(read_buff, 0, sizeof(read_buff));
    cyhal_flash_read(&cyhal_flash, TEST_TARGET_ADDR, read_buff, TEST_TARGET_SIZE);
    for(uint32_t i = 0; i < TEST_TARGET_SIZE; i++)
    {
        if(write_buff[i] != read_buff[i])
        {
            CY_ASSERT(0);
        }
    }
    printf("Target sector program finished! \r\n\n");

    /* 3.Read entire sector data from target sector. the sector size is 32k or 8k,
        the data size smaller than the sector size, it's could any size. and program the data into empty sector. */
    printf("\r__________________________________________________________________________\r\n\n");
    printf("[3] Read out target sector data into RAM, and program to empty sector.\r\n");

    printf("Empty sector erase start.. \r\n");
    /* Erase empty sector */
    cyhal_flash_erase(&cyhal_flash, TEST_EMPTY_ADDR);

    /* Verify erase state */
    for(uint32_t PageNum = 0; PageNum < (CY_CODE_SES_SIZE_IN_BYTE/CY_FLASH_SIZEOF_ROW); PageNum++)
    {
        memset(read_buff, 0, sizeof(read_buff));
        cyhal_flash_read(&cyhal_flash, TEST_EMPTY_ADDR + (PageNum * CY_FLASH_SIZEOF_ROW), read_buff, TEST_EMPTY_SIZE);
        /* Verify erase state */
        for(uint32_t i = 0; i < CY_FLASH_SIZEOF_ROW; i++)
        {
           if(read_buff[i] != 0xFF)
           {
               CY_ASSERT(0);
           }
        }
    }
    printf("Empty sector erase finished! \r\n\n");
    
    printf("Empty sector program start.. \r\n");
    /* Programming code flash target sector */
    for(uint32_t PageNum = 0; PageNum < (CY_CODE_SES_SIZE_IN_BYTE/CY_FLASH_SIZEOF_ROW); PageNum++)
    {
        memset(write_buff, 0, sizeof(write_buff));
        cyhal_flash_read(&cyhal_flash, TEST_TARGET_ADDR + (PageNum * CY_FLASH_SIZEOF_ROW), write_buff, TEST_TARGET_SIZE);

        result = cyhal_flash_program(&cyhal_flash, TEST_EMPTY_ADDR + (PageNum * CY_FLASH_SIZEOF_ROW), (uint32_t*)write_buff);
        if (result != CY_RSLT_SUCCESS)
        {
            CY_ASSERT(0);
        }

        /* Verify erase state */
        memset(read_buff, 0, sizeof(read_buff));
        cyhal_flash_read(&cyhal_flash, TEST_EMPTY_ADDR + (PageNum * CY_FLASH_SIZEOF_ROW), read_buff, TEST_EMPTY_SIZE);
        for(uint32_t i = 0; i < CY_FLASH_SIZEOF_ROW; i++)
        {
           if(read_buff[i] != write_buff[i])
           {
               CY_ASSERT(0);
           }
        }
    }

    printf("Empty sector program finished! \r\n\n");
    
    /* 4.Data process, the new 512bytes data with readback data, it's could replace, insert, add, and so on. */
    printf("\r__________________________________________________________________________\r\n\n");

    printf("[4] Data process, and program new data into target sector\r\n");
    printf("Target sector erase start.. \r\n\n");
    /* Erase target sector */
    cyhal_flash_erase(&cyhal_flash, TEST_TARGET_ADDR);

    /* Verify */
    for(uint32_t PageNum = 0; PageNum < (CY_CODE_SES_SIZE_IN_BYTE/CY_FLASH_SIZEOF_ROW); PageNum++)
    {
        memset(read_buff, 0, sizeof(read_buff));
        cyhal_flash_read(&cyhal_flash, TEST_TARGET_ADDR + (PageNum * CY_FLASH_SIZEOF_ROW), read_buff, TEST_TARGET_SIZE);
        /* Verify erase state */
        for(uint32_t i = 0; i < CY_FLASH_SIZEOF_ROW; i++)
        {
           if(read_buff[i] != 0xFF)
           {
               CY_ASSERT(0);
           }
        }
    }
    printf("Target sector erase finished! \r\n\n");

    printf("Update target data and program.. \r\n");
    printf("Modify [0] and [10] data value \r\n");
    /* Programming */
    for(uint32_t PageNum = 0; PageNum < (CY_CODE_SES_SIZE_IN_BYTE/CY_FLASH_SIZEOF_ROW); PageNum++)
    {
        memset(write_buff, 0, sizeof(write_buff));
        cyhal_flash_read(&cyhal_flash, TEST_EMPTY_ADDR + (PageNum * CY_FLASH_SIZEOF_ROW), write_buff, TEST_EMPTY_SIZE);

        write_buff[0] =  0x55;
        write_buff[10] = 0xaa;

        result = cyhal_flash_program(&cyhal_flash, TEST_TARGET_ADDR + (PageNum * CY_FLASH_SIZEOF_ROW), (uint32_t*)write_buff);
        if (result != CY_RSLT_SUCCESS)
        {
            CY_ASSERT(0);
        }

        /* Verify erase state */
        memset(read_buff, 0, sizeof(read_buff));
        cyhal_flash_read(&cyhal_flash, TEST_TARGET_ADDR + (PageNum * CY_FLASH_SIZEOF_ROW), read_buff, TEST_TARGET_SIZE);
        for(uint32_t i = 0; i < CY_FLASH_SIZEOF_ROW; i++)
        {
           if(read_buff[i] != write_buff[i])
           {
               CY_ASSERT(0);
           }
        }
    }

    printf("Program finished! \r\n\n");
    printf("\r__________________________________________________________________________\r\n");
    
    for(;;)
    {

    }
}


/* [] END OF FILE */
