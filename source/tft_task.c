/******************************************************************************
*
* File Name: tft_task.c
*
* Description: This file contains task and functions related to the tft-task
* that demonstrates controlling a tft display using the EmWin Graphics Library.
*
* The project then displays scan information from scan task.
*
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "GUI.h"
#include "mtb_st7789v.h"
#include "tft_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define STARTUP_DELAY               (2000/*ms*/) /* Amount of time to show the startup logo */

/* Parameters for Newhaven 2.4″ 240×320 TFT display with a Sitronix ST7789 display controller w/8080 Parallel Interface. */
#define Y_STEP_SIZE					(15u)
#define NUMBER_OF_LINES				(16u)

/* The pins are defined by the st7789v library. If the display is being used
 *  on different hardware the mappings will be different. */
const mtb_st7789v_pins_t tft_pins =
{
    .db08 = CYBSP_J2_2,
    .db09 = CYBSP_J2_4,
    .db10 = CYBSP_J2_6,
    .db11 = CYBSP_J2_10,
    .db12 = CYBSP_J2_12,
    .db13 = CYBSP_D7,
    .db14 = CYBSP_D8,
    .db15 = CYBSP_D9,
    .nrd  = CYBSP_D10,
    .nwr  = CYBSP_D11,
    .dc   = CYBSP_D12,
    .rst  = CYBSP_D13
};


/*******************************************************************************
* Forward Function Prototypes
*******************************************************************************/

/*******************************************************************************
* Function Name: void tft_task(void *arg)
********************************************************************************
*
* Summary: The Following functions are performed in this task
*           1. Initializes the EmWin display engine
*           2. Displays startup screen for 3 seconds
*           3. In an infinite loop, displays messages from scan_task
*
* Parameters:
*  arg: task argument
*
* Return:
*  None
*
*******************************************************************************/
void tft_task(void *arg)
{
    cy_rslt_t result;
	char stringBuffer[LINE_LENGTH];
    extern QueueHandle_t stringQueue; /* extern declaration to the string Queue */
    char rxStringBuffer[LINE_LENGTH]; /* local buffer/storage for Queue message from UART task */
    char screenBuffer[NUMBER_OF_LINES][LINE_LENGTH]; /* ring buffer storage for TFT screen */
    BaseType_t headPointer = 0, indexScreenBuffer = 0; /* head pointer for circular screen buffer */

	/* create string queue */
	stringQueue = xQueueCreate(20, sizeof(stringBuffer));

    /* initialize screenBuffer with NULL strings */
    for(uint8_t i = 0; i < NUMBER_OF_LINES; i++)
    {
    	strcpy(screenBuffer[i], "");
    }

    /* Initialize the User LED */
    result = cyhal_gpio_init( P11_1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize the display controller */
    result = mtb_st7789v_init8(&tft_pins);
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    
    /* To avoid compiler warning */
    (void)result;
    
    GUI_Init();

	cyhal_gpio_toggle(P11_1);
	vTaskDelay(STARTUP_DELAY);
	cyhal_gpio_toggle(P11_1);

    /* Set font size, foreground color, and background color */
    GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetColor(GUI_WHITE);
    GUI_SetBkColor(GUI_BLACK);

	/* Clear the display */
	GUI_Clear();

	for(;;)
	{
		cyhal_gpio_toggle(P11_1);

		xQueueReceive(stringQueue, rxStringBuffer, portMAX_DELAY); /* get string from queue */
		indexScreenBuffer = headPointer; /* set screen line buffer index to head pointer */
		strcpy(screenBuffer[headPointer], rxStringBuffer); /* copy string to head of screen buffer */
		for(uint8_t i = 0; i < NUMBER_OF_LINES; i++) /* fill the screen (scroll upward) */
		{
			GUI_DispStringAtCEOL(screenBuffer[indexScreenBuffer], 0, (NUMBER_OF_LINES - 1 - i) * Y_STEP_SIZE); /* write TFT */
			if(indexScreenBuffer == 0) /* adjust the index */
				indexScreenBuffer = NUMBER_OF_LINES - 1;
			else
				indexScreenBuffer--;
		}
		headPointer++; /* move the head pointer forward */
		if(headPointer >= NUMBER_OF_LINES)
		{
			headPointer = 0;
		}
	}
}

/* END OF FILE */
