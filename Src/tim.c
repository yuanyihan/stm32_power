/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "gpio.h"
#include "usart.h"
#include "stdint.h"
#include "string.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* TIM6 init function */
void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 55999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}
/* TIM7 init function */
void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* Peripheral clock enable */
    __TIM6_CLK_ENABLE();

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
  else if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */

  /* USER CODE END TIM7_MspInit 0 */
    /* Peripheral clock enable */
    __TIM7_CLK_ENABLE();
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM6_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM6_IRQn);

  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspDeInit 0 */

  /* USER CODE END TIM7_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM7_CLK_DISABLE();
  /* USER CODE BEGIN TIM7_MspDeInit 1 */

  /* USER CODE END TIM7_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
int16_t gpio0_off=-1;
int16_t gpio0_on=0;
int16_t gpio1_off=-1;
int16_t gpio1_on=0;
void set_gpio_time(char gpio,char sta,char seconds)
{
    if(gpio==0)
    {
        if(sta==1)
        {
            gpio0_off=-1;
            gpio0_on=0;
            if((seconds>0)&&(seconds<100))
            {
                gpio0_on=100*seconds;
            }
            else
            {
                gpio0_off=0;
                gpio0_on=-1;
            }
        }
        if(sta==0)
        {
            gpio0_off=0;
            gpio0_on=-1;
            if((seconds>0)&&(seconds<100))
            {
                gpio0_off=100*seconds;
            }
            else
            {
                gpio0_off=-1;
                gpio0_on=0;
            }
        }
    }
    if(gpio==1)
    {
        if(sta==1)
        {
            gpio1_off=-1;
            gpio1_on=0;
            if((seconds>0)&&(seconds<100))
            {
                gpio1_on=100*seconds;
            }
            else
            {
                gpio1_off=0;
                gpio1_on=-1;
            }
        }
        if(sta==0)
        {
            gpio1_off=0;
            gpio1_on=-1;
            if((seconds>0)&&(seconds<100))
            {
                gpio1_off=100*seconds;
            }
            else
            {
                gpio1_off=-1;
                gpio1_on=0;
            }
        }
    }
}

void check_gpio_time(void)
{
    if((gpio0_off==-1))
    {
        if(gpio0_on>0)
        {
            gpio0_on--;
            gpiomos1=1	;
        }
        else
        {
            gpiomos1=0	;
        }
    }
    else if((gpio0_on==-1))
    {
        if(gpio0_off>0)
        {
            gpio0_off--;
            gpiomos1=0	;
        }
        else
        {
            gpiomos1=1	;
        }
    }
    if((gpio1_off==-1))
    {
        if(gpio1_on>0)
        {
            gpio1_on--;
            gpiopwr1=1	;
        }
        else
        {
            gpiopwr1=0	;
        }
    }
    else if((gpio1_on==-1))
    {
        if(gpio1_off>0)
        {
            gpio1_off--;
            gpiopwr1=0	;
        }
        else
        {
            gpiopwr1=1	;
        }
    }
}

uint16_t led1_off=100;
uint16_t led1_on=0;
uint16_t led1_num=0;
uint16_t led2_off=50;
uint16_t led2_on=50;
uint16_t led2_num=0;
uint16_t led3_off=100;
uint16_t led3_on=0;
uint16_t led3_num=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //10ms????,,?10ms????
{
//    static uint16_t led1_off=100;
//    static uint16_t led1_on=0;
//    static uint16_t led1_num=0;
//    static uint16_t led2_off=50;
//    static uint16_t led2_on=50;
//    static uint16_t led2_num=0;
//    static uint16_t led3_off=100;
//    static uint16_t led3_on=0;
//    static uint16_t led3_num=0;
//   check_gpio_time();
//    led1_num++;
//    if((led1_num<led1_off))
//    {
//        gpioled1=0;
//    }
//    else if(led1_num<led1_off+led1_on)
//    {
//        gpioled1=1;
//    }
//    else
//    {
//        led1_num=0;
//    }
//    switch(HIMIN_LED1_F)
//    {
//    case 0:
//        led1_off=100;
//        led1_on=0;
//        break;
//    case 1:
//        led1_off=150;
//        led1_on=50;
//        break;
//    case 2:
//        led1_off=50;
//        led1_on=50;
//        break;
//    case 3:
//        led1_off=200;
//        led1_on=200;
//        break;
//    case 4:
//        led1_off=50;
//        led1_on=150;
//        break;
//    case 5:
//        led1_off=0;
//        led1_on=100;
//        break;
//    }
//    led2_num++;
//    if((led2_num<led2_off))
//    {
//        gpioled2=0;
//    }
//    else if(led2_num<led2_off+led2_on)
//    {
//        gpioled2=1;
//    }
//    else
//    {
//        led2_num=0;
//    }
//    switch(HIMIN_LED2_F)
//    {
//    case 0:
//        led2_off=100;
//        led2_on=0;
//        break;
//    case 1:
//        led2_off=150;
//        led2_on=50;
//        break;
//    case 2:
//        led2_off=50;
//        led2_on=50;
//        break;
//    case 3:
//        led2_off=200;
//        led2_on=200;
//        break;
//    case 4:
//        led2_off=50;
//        led2_on=150;
//        break;
//    case 5:
//        led2_off=0;
//        led2_on=100;
//        break;
//    }


//    led3_num++;
//    if((led3_num<led3_off))
//    {
//        gpioled3=0;
//    }
//    else if(led3_num<led3_off+led3_on)
//    {
//        gpioled3=1;
//    }
//    else
//    {
//        led3_num=0;
//    }
//    switch(HIMIN_LED3_F)
//    {
//    case 0:
//        led3_off=100;
//        led3_on=0;
//        break;
//    case 1:
//        led3_off=150;
//        led3_on=50;
//        break;
//    case 2:
//        led3_off=50;
//        led3_on=50;
//        break;
//    case 3:
//        led3_off=200;
//        led3_on=200;
//        break;
//    case 4:
//        led3_off=50;
//        led3_on=150;
//        break;
//    case 5:
//        led3_off=0;
//        led3_on=100;
//        break;
//   }

    /*
    		if (Usart1_Buffer.buffer_time>0) {
    			Usart1_Buffer.buffer_time--;
    		} else {
    			if (Usart1_Buffer.buffer_std==BUFFER_STATE_RECEIVING) {
    				Usart1_Buffer.buffer_std=BUFFER_STATE_RECEIVED_END;
    				Usart1_Buffer.buffer_len=Usart1_Buffer.buffer_num;
    			}
    		}

    		if (Usart1_Buffer.buffer_std==BUFFER_STATE_REC_NO_END) {
    			Usart1_Buffer.buffer_num=0;
    			Usart1_Buffer.buffer_len=0;
    			Usart1_Buffer.buffer_temp=0;
    			Usart1_Buffer.buffer_time=0;
    			memset(Usart1_Buffer.buffer_data,0,100);
    			Usart1_Buffer.buffer_std=BUFFER_STATE_HOLD_ON;
    		}
    	*/
}


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
