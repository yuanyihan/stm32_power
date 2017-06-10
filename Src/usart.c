/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
BufferDataTypeDef Usart1_Buffer= {0};
char HIMIN_LED1_F=2;
char HIMIN_LED2_F=0;
char HIMIN_LED3_F=0;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART1_IRQn);

  }
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance==USART1)
    {

        if (Usart1_Buffer.buffer_temp =='S')
        {
            Usart1_Buffer.buffer_std=BUFFER_STATE_RECEIVING;
            Usart1_Buffer.buffer_num=0;
            Usart1_Buffer.buffer_len=0;
        }
        if (Usart1_Buffer.buffer_std==BUFFER_STATE_RECEIVING)
        {
            if(Usart1_Buffer.buffer_num<35)
            {
                Usart1_Buffer.buffer_data[Usart1_Buffer.buffer_num]=Usart1_Buffer.buffer_temp;
                Usart1_Buffer.buffer_num++;
            }
            else
            {
                Usart1_Buffer.buffer_temp ='#';
            }
        }
        if ((Usart1_Buffer.buffer_temp =='#')&&(Usart1_Buffer.buffer_std==BUFFER_STATE_RECEIVING))
        {
            Usart1_Buffer.buffer_std=BUFFER_STATE_RECEIVED_END;
            Usart1_Buffer.buffer_len=Usart1_Buffer.buffer_num;
        }
        HAL_UART_Receive_IT(&huart1,&Usart1_Buffer.buffer_temp,1);//COM
    }
}




void DoUsartFuction_Rxd()
{
    char seconds=0;
    if (Usart1_Buffer.buffer_std==BUFFER_STATE_RECEIVED_END)
    {
       if ( \
                (Usart1_Buffer.buffer_len >= 3) \
           )
        {
            if (Usart1_Buffer.buffer_data[0] =='S')
            {
                if (Usart1_Buffer.buffer_data[4] =='P')
                {


                    if(Usart1_Buffer.buffer_data[5]==0x30) //30:open
                    {
                        HIMIN_LED1_F=0;
                    }
                    if(Usart1_Buffer.buffer_data[5]==0x31) //32:close
                    {
                        HIMIN_LED1_F=1;
                    }
                    if(Usart1_Buffer.buffer_data[5]==0x32) //30:open
                    {
                        HIMIN_LED1_F=2;
                    }
                    if(Usart1_Buffer.buffer_data[5]==0x33) //32:close
                    {
                        HIMIN_LED1_F=3;
                    }
                    if(Usart1_Buffer.buffer_data[5]==0x34) //30:open
                    {
                        HIMIN_LED1_F=4;
                    }
                    if(Usart1_Buffer.buffer_data[5]==0x35) //32:close
                    {
                        HIMIN_LED1_F=5;
                    }

                    if(Usart1_Buffer.buffer_data[6]==0x30) //30:open
                    {
                        HIMIN_LED2_F=0;
                    }
                    if(Usart1_Buffer.buffer_data[6]==0x31) //32:close
                    {
                        HIMIN_LED2_F=1;
                    }
                    if(Usart1_Buffer.buffer_data[6]==0x32) //30:open
                    {
                        HIMIN_LED2_F=2;
                    }
                    if(Usart1_Buffer.buffer_data[6]==0x33) //32:close
                    {
                        HIMIN_LED2_F=3;
                    }
                    if(Usart1_Buffer.buffer_data[6]==0x34) //30:open
                    {
                        HIMIN_LED2_F=4;
                    }
                    if(Usart1_Buffer.buffer_data[6]==0x35) //32:close
                    {
                        HIMIN_LED2_F=5;
                    }

                    if(Usart1_Buffer.buffer_data[7]==0x30) //30:open
                    {
                        HIMIN_LED3_F=0;
                    }
                    if(Usart1_Buffer.buffer_data[7]==0x31) //32:close
                    {
                        HIMIN_LED3_F=1;
                    }
                    if(Usart1_Buffer.buffer_data[7]==0x32) //30:open
                    {
                        HIMIN_LED3_F=2;
                    }
                    if(Usart1_Buffer.buffer_data[7]==0x33) //32:close
                    {
                        HIMIN_LED3_F=3;
                    }
                    if(Usart1_Buffer.buffer_data[7]==0x34) //30:open
                    {
                        HIMIN_LED3_F=4;
                    }
                    if(Usart1_Buffer.buffer_data[7]==0x35) //32:close
                    {
                        HIMIN_LED3_F=5;
                    }

//

                    if(Usart1_Buffer.buffer_data[9]==0x30) //30:open
                    {
                        gpiopwr1=0;
                    }
                    if(Usart1_Buffer.buffer_data[9]==0x31) //32:close
                    {
                        gpiopwr1=1;
                    }

                    if(Usart1_Buffer.buffer_data[10]==0x30) //30:open
                    {
                        gpiopwr2=0;
                    }
                    if(Usart1_Buffer.buffer_data[10]==0x31) //32:close
                    {
                        gpiopwr2=1;
                    }

                    if(Usart1_Buffer.buffer_data[11]==0x30) //30:open
                    {
                        gpiopwr3=0;
                    }
                    if(Usart1_Buffer.buffer_data[11]==0x31) //32:close
                    {
                        gpiopwr3=1;
                    }

                    if(Usart1_Buffer.buffer_data[12]==0x30) //30:open
                    {
                        gpioret1=0;
                    }
                    if(Usart1_Buffer.buffer_data[12]==0x31) //32:close
                    {
                        gpioret1=1;
                    }

                    if(Usart1_Buffer.buffer_data[13]==0x30) //30:open
                    {
                        gpioret2=0;
                    }
                    if(Usart1_Buffer.buffer_data[13]==0x31) //32:close
                    {
                        gpioret2=1;
                    }
                    if(Usart1_Buffer.buffer_data[14]==0x30) //30:open
                    {
                        gpiomos1=0;
                    }
                    if(Usart1_Buffer.buffer_data[14]==0x31) //32:close
                    {
                        gpiomos1=1;
                    }

                    if(Usart1_Buffer.buffer_data[15]==0x30) //30:open
                    {
                        gpiomos2=0;
                    }
                    if(Usart1_Buffer.buffer_data[15]==0x31) //32:close
                    {
                        gpiomos2=1;
                    }

                    if(Usart1_Buffer.buffer_data[16]==0x30) //30:open
                    {
                        gpiomos3=0;
                    }
                    if(Usart1_Buffer.buffer_data[16]==0x31) //32:close
                    {
                        gpiomos3=1;
                    }
                }
                if (Usart1_Buffer.buffer_data[2] =='O')
                {
                    if (Usart1_Buffer.buffer_data[5] =='T')
                    {

                        if(Usart1_Buffer.buffer_data[3]=='F')
                        {
												if((Usart1_Buffer.buffer_data[10]>'9')||(Usart1_Buffer.buffer_data[11]>'9')){
												seconds=120;
												HIMIN_LED2_F=5;
											}
												else{
                        seconds=(Usart1_Buffer.buffer_data[10]-'0')*10+(Usart1_Buffer.buffer_data[11]-'0');
												HIMIN_LED2_F=0;
												}
                            if(Usart1_Buffer.buffer_data[4]=='0')
                            {
															
                                set_gpio_time(CTRL_FAN,CTRL_CLOSE,seconds);
                            }
                            else
                            {
                                set_gpio_time(CTRL_FAN,CTRL_OPEN,seconds);
                            }
                        }
                        if(Usart1_Buffer.buffer_data[3]=='L')
                        {
												if((Usart1_Buffer.buffer_data[10]>'9')||(Usart1_Buffer.buffer_data[11]>'9')){
												seconds=120;
												HIMIN_LED3_F=5;
											}
												else{
                        seconds=(Usart1_Buffer.buffer_data[10]-'0')*10+(Usart1_Buffer.buffer_data[11]-'0');
												HIMIN_LED3_F=0;
												}
                            if(Usart1_Buffer.buffer_data[4]=='0')
                            {
                                set_gpio_time(CTRL_LAMP,CTRL_CLOSE,seconds);
                            }
                            else
                            {
                                set_gpio_time(CTRL_LAMP,CTRL_OPEN,seconds);
                            }
                        }
                    }
                }
                Usart1_Buffer.buffer_std=BUFFER_STATE_REC_NO_END;
            }
        }
        Usart1_Buffer.buffer_std=BUFFER_STATE_REC_NO_END;
    }
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
