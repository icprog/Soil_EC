/**
  ******************************************************************************
  * File Name          : ADC.h
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __adc_H
#define __adc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

#include <stdbool.h>

#define VREFINT_CAL_ADDR                   							((uint16_t*) ((uint32_t)0x1FF80078U)) /* Internal voltage reference, address of parameter VREFINT_CAL: VrefInt ADC raw data acquired at temperature 30 DegC (tolerance: +-5 DegC), Vref+ = 3.0 V (tolerance: +-10 mV). */
#define VREFINT_CAL_VREF                   							((uint32_t) 3U)                    /* Analog voltage reference (Vref+) value with which temperature sensor has been calibrated in production (tolerance: +-10 mV) (unit: mV). */
#define VDD_APPLI                      		 							((uint32_t) 1220U)    /* Value of analog voltage supply Vdda (unit: mV) */
#define VFULL																						((uint32_t) 4095U)
	 
#define VREFEXT_CAL_VREF                   							((float)1.25)                    /* Analog voltage reference (Vref+) value with which temperature sensor has been calibrated in production (tolerance: +-10 mV) (unit: mV). */
	 

#define TEMP130_CAL_ADDR 																((uint16_t*) ((uint32_t) 0x1FF8007E))
#define TEMP30_CAL_ADDR 																((uint16_t*) ((uint32_t) 0x1FF8007A))
#define VDD_CALIB 																			((uint16_t) (300))
#define VDD_APPLI2 																			((uint16_t) (330))	 


/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc;

/* USER CODE BEGIN Private defines */

#define BUFLEN		11

typedef struct u_adc
{
	uint16_t Data;
	uint32_t Buf[BUFLEN*2];
	uint32_t EC_HData;
	uint32_t EC_LData;
	
	/*************ADC采集完成***********/
	bool 		 Complete;
	
	/*************ADC采集EC使能：防止ADC采集冲突：timer定时采集***********/
	bool     CollectEcEnable;
	
	/*************ADC Pwm状态：true才开始采集数据***********/
	bool 		 PwmEc;
	
//	uint32_t HBuf[BUFLEN];
//	uint32_t LBuf[BUFLEN];
}adc_t;
	 
extern adc_t Adc;


/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_ADC_Init(void);

/* USER CODE BEGIN Prototypes */

uint16_t GetAdcData(uint32_t Channel, uint8_t Counter);

uint16_t GetAdcVref(uint32_t Channel, uint8_t Counter);

void		 GetEcAdc(uint32_t Channel, uint8_t Counter);

void 		 GetEcHAdc(uint32_t Channel, uint8_t Counter);

void 		 GetEcLAdc(uint32_t Channel, uint8_t Counter);

void 		 EcHandle(uint8_t CollectNum);

void		 Bublesort(uint32_t *a,uint8_t n);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
