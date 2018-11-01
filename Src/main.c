
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include <string.h>
#include <math.h>

#include "user_app.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t Time = 0;

	float    Adcdata = 0;
	uint32_t Vchannel = 0;

	float    EcSq = 0;
	float    RSq = 0;
	float 	 Tempure = 0;
	
	float 	 temp = 0;
	
	uint16_t adc_data[5] = {0};
	float 	 SensorBuf[5] = {0};
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_DMA(&huart2,UART_RX_UART2.USART_RX_BUF,MAXSIZE);      
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		
	Rs485Init(  );
	RS485_TO_RX(  );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */		

#if 1		
		
//		if(UART_RX_UART2.USART_RX_Len)
		{
			RS485_TO_TX(  );
			Time = HAL_GetTick(  );
			
//			memset(Adc.Buf, 0, BUFLEN*2);
//					
//			Adc.PwmEc = false;
//			Adc.CollectEcEnable = true;
//			EC_HData = 0;
//			EC_LData = 0;
//						
//			HAL_Delay(3); ///timer = 2.5ms

//			if(Adc.PwmEc)
//			{
//				Adc.PwmEc = false;
//				for(uint8_t i = 0; i < BUFLEN*2; ++i)
//				{
//					if(i%2==0)
//					EC_HData += Adc.Buf[i];	
//					
//					if(i%2==1)
//					EC_LData += Adc.Buf[i];
//				}			
//				
//				EC_HData /= BUFLEN;
//				EC_LData /= BUFLEN;
//					
//				printf("\r\nEH = %d EL = %d\r\n",EC_HData,EC_LData);	
//				Adc.CollectEcEnable = false;
//			}			
			EcHandle( 10 );
			adc_data[2] = Adc.EC_HData;		
			adc_data[3] = Adc.EC_LData;	
			adc_data[0] = GetAdcData(ADC_CHANNEL_VREFINT, BUFLEN); //内部采样
			
			adc_data[1] = GetAdcData(ADC_CHANNEL_1, BUFLEN);  ///外部基准
			
			adc_data[4] = GetAdcData(ADC_CHANNEL_6, BUFLEN); ///温度
									
			Vchannel = VFULL * adc_data[0] * VREFEXT_CAL_VREF;
			
			Adcdata = (VREFINT_CAL_VREF * adc_data[1]);					
			
			Adcdata = (float)(Vchannel/Adcdata);
						
			temp = VREFINT_CAL_VREF * Adcdata;
			
			printf("内部采样 = %d, 外部基准 = %d  EC_H = %d EC_L = %d 温度 = %d \r\n",adc_data[0], adc_data[1],adc_data[2],adc_data[3],adc_data[4]);
			
			for(uint8_t i = 0, j = 1; i < 4; ++i, ++j)
			{
				SensorBuf[i] = (float)(temp*adc_data[j])/(adc_data[0] * VFULL);	
				
//				printf("SensorBuf[%d] = %.2f\r\n",i, SensorBuf[i]);
			}				
			
			printf("外部基准 = %.4f 内部基准 = %.4f\r\n", SensorBuf[0],(float)(temp*adc_data[1])/(adc_data[1] * VFULL));
		 ///EC	
			printf("EC_H = %.4f EC_L = %.4f\r\n", SensorBuf[1], SensorBuf[2]);
					
			EcSq = SensorBuf[1] - SensorBuf[2];
						
			if(EcSq > 0)
			{
				RSq = 2 * SensorBuf[2]; ///得到两探针间的电阻差
				RSq /= EcSq;   ///得到电感值
			}
			else
			RSq = 0;			
			
			Tempure = (float)(SensorBuf[3] - VREFEXT_CAL_VREF) * 100;
			
			Tempure -= 0.4;
								
			printf("EC = %.4f mS/cm 温度 = %.2f°C\r\n", (float)1/(2.5*RSq), Tempure);
				
			int16_t TemPureData = (int)((Tempure * 10) + 0.5);
						
			printf("温度 = %d°C, delay = %d\r\n", TemPureData,HAL_GetTick(  ) - Time);		
										
			UART_RX_UART2.USART_RX_Len=0;
			
			Rs485RevceHandle(  );
			RS485_TO_RX(  );		
		}				
		HAL_Delay(1000);
//		Time = 0;
#endif

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
