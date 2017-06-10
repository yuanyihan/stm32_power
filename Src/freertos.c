/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "TASK.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId INPUTTaskHandle;
osThreadId OUTPUTTaskHandle;
osThreadId SERVICETaskHandle;
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void FUNC_INPUT(void const * argument);
void FUNC_OUTPUT(void const * argument);
void FUNC_SERVICE(void const * argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
	char i=0;
	i++;
}
/* USER CODE END 2 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of LED0Task */
  osThreadDef(INPUTTask, FUNC_INPUT, osPriorityNormal, 0, 128);
 INPUTTaskHandle = osThreadCreate(osThread(INPUTTask), NULL);

  /* definition and creation of LED1Task */
    osThreadDef(OUTPUTTask, FUNC_OUTPUT, osPriorityNormal, 0, 128);
 OUTPUTTaskHandle = osThreadCreate(osThread(OUTPUTTask), NULL);
	  osThreadDef(SERVICETask, FUNC_SERVICE, osPriorityNormal, 0, 128);
  SERVICETaskHandle = osThreadCreate(osThread(SERVICETask), NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* FUNC_LED0 function */
void FUNC_OUTPUT(void const * argument)
{

  /* USER CODE BEGIN FUNC_LED0 */
  /* Infinite loop */
  for(;;)
  {
		
		if(gpioled1==1){HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);}
		else{HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);}
		if(gpioled2==1){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);}
		else{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);}
		if(gpioled3==1){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);}
		else{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);}
		
		if(gpiomos1==1){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);}
		else{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);}
		if(gpiomos2==1){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);}
		else{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);}
		if(gpiomos3==1){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);}
		else{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);}
		
		if(gpiopwr1==1){HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);}
		else{HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);}
		if(gpiopwr2==1){HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);}
		else{HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);}
		if(gpiopwr3==1){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);}
		else{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);}
		
		if(gpioret1==1){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);}
		else{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);}
		if(gpioret2==1){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);}
		else{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);}
		
		taskYIELD();
  }
  /* USER CODE END FUNC_LED0 */
}
void FUNC_SERVICE(void const * argument)
{
portTickType PreviousWakeTime=0;
	PreviousWakeTime=xTaskGetTickCount();
  /* USER CODE BEGIN FUNC_LED0 */
  /* Infinite loop */
  for(;;)
  {
		//osDelayUntil(PreviousWakeTime,10);
		//osDelay(10);
		vTaskDelayUntil(&PreviousWakeTime,10/portTICK_RATE_MS);
   check_gpio_time();
    led1_num++;
    if((led1_num<led1_off))
    {
        gpioled1=0;
    }
    else if(led1_num<led1_off+led1_on)
    {
        gpioled1=1;
    }
    else
    {
        led1_num=0;
    }
    switch(HIMIN_LED1_F)
    {
    case 0:
        led1_off=100;
        led1_on=0;
        break;
    case 1:
        led1_off=150;
        led1_on=50;
        break;
    case 2:
        led1_off=50;
        led1_on=50;
        break;
    case 3:
        led1_off=200;
        led1_on=200;
        break;
    case 4:
        led1_off=50;
        led1_on=150;
        break;
    case 5:
        led1_off=0;
        led1_on=100;
        break;
    }
    led2_num++;
    if((led2_num<led2_off))
    {
        gpioled2=0;
    }
    else if(led2_num<led2_off+led2_on)
    {
        gpioled2=1;
    }
    else
    {
        led2_num=0;
    }
    switch(HIMIN_LED2_F)
    {
    case 0:
        led2_off=100;
        led2_on=0;
        break;
    case 1:
        led2_off=150;
        led2_on=50;
        break;
    case 2:
        led2_off=50;
        led2_on=50;
        break;
    case 3:
        led2_off=200;
        led2_on=200;
        break;
    case 4:
        led2_off=50;
        led2_on=150;
        break;
    case 5:
        led2_off=0;
        led2_on=100;
        break;
    }
led3_num++;
    if((led3_num<led3_off))
    {
        gpioled3=0;
    }
    else if(led3_num<led3_off+led3_on)
    {
        gpioled3=1;
    }
    else
    {
        led3_num=0;
    }
    switch(HIMIN_LED3_F)
    {
    case 0:
        led3_off=100;
        led3_on=0;
        break;
    case 1:
        led3_off=150;
        led3_on=50;
        break;
    case 2:
        led3_off=50;
        led3_on=50;
        break;
    case 3:
        led3_off=200;
        led3_on=200;
        break;
    case 4:
        led3_off=50;
        led3_on=150;
        break;
    case 5:
        led3_off=0;
        led3_on=100;
        break;
  }}
  /* USER CODE END FUNC_LED0 */
}
/* FUNC_LED1 function */
void FUNC_INPUT(void const * argument)
{
  /* USER CODE BEGIN FUNC_LED1 */
  /* Infinite loop */
  for(;;)
  {
	DoUsartFuction_Rxd();
  }
  /* USER CODE END FUNC_LED1 */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
