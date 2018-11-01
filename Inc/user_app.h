

#ifndef __USER_APP__H
#define __USER_APP__H

#ifdef __cplusplus
 extern "C" {
#endif	

#include "stm32l0xx_hal.h"
	 
#define RS485_TO_TX(  )					HAL_GPIO_WritePin(Rs485_DE_GPIO_Port, Rs485_DE_Pin, GPIO_PIN_SET)
#define RS485_TO_RX(  )					HAL_GPIO_WritePin(Rs485_DE_GPIO_Port, Rs485_DE_Pin, GPIO_PIN_RESET)

#define GET_CRC(__X__,DATA)    ((__X__)[1] = ((DATA & 0xff00) >> 8), (__X__)[0] = (DATA & 0x00ff))
	 
	 
void Rs485Init(void);
	
void Rs485RevceHandle(void);	 

uint16_t CalcCRC16(uint8_t *data, uint8_t len);
	 
#ifdef __cplusplus
}
#endif
#endif 

