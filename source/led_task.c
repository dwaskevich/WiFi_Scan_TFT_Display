/*
 * led_task.c
 *
 *  Created on: Oct 17, 2022
 *      Author: Waskevich
 */

#include "cyhal.h"
#include "cybsp.h"
#include "freeRTOS.h"
#include "task.h"
#include "led_task.h"


void led_task(void *arg)
{
	(void)arg;

	cyhal_gpio_init(CYBSP_LED9, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

	for(;;)
	{
		cyhal_gpio_toggle_internal(CYBSP_LED9);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}


