

#include <string.h>
#include "user_app.h"
#include "usart.h"

uint8_t Rs485Addr = 0;

/*
*RS485�� ��ʼ��485
*������  ��
*����ֵ����
*/
void Rs485Init(void)
{
	RS485_TO_RX(  );
	Rs485Addr = 0x20;
}

/*
*Rs485RevceHandle�� Rs485���ݴ���
*������  			 			����������
*����ֵ��			 			��
*/
void Rs485RevceHandle(int16_t *SenSorBuf)
{
	uint8_t temp[15] = {0};
	uint8_t len = 0;
	
	RS485_TO_TX(  );

	if(CalcCRC16(UART_RX_UART2.USART_RX_BUF,UART_RX_UART2.USART_RX_Len) == 0)
	{		
		///�ж�Rs485������
		switch(UART_RX_UART2.USART_RX_BUF[1])
		{
			case 0x03:  ///1���㲥���� fe030400000000F53C
				if(UART_RX_UART2.USART_RX_BUF[0] == 0xFE && UART_RX_UART2.USART_RX_Len == 0x09)
				{
					///fe030400000000F53C
					temp[len++] = 0xfe;
					temp[len++] = 0x03;
					temp[len++] = 0x04;
				
					temp[len++] = Rs485Addr;
					temp[len++] = 0x00;
					temp[len++] = 0x00;
					temp[len++] = 0x00;
				
					CalcCRC16(temp,len);
					
					HAL_Delay(200); ///�������1s��ʱ
					HAL_UART_Transmit(&huart2, temp, (len+2),0xFFFF);	
				}
				else if(UART_RX_UART2.USART_RX_BUF[0] == Rs485Addr) ///2����ȡ����ָ��¶�*10��EC*1000
				{
					memset(SensorData, 0, 2);
					AdcHandle( );
					
					///rev: 200300000002C2BA
					temp[len++] = Rs485Addr;
					temp[len++] = 0x03;
					temp[len++] = 0x04;
				
					temp[len++] = (SensorData[0]>>8 & 0xff);
					temp[len++] = (SensorData[0]>>0 & 0xff);
					temp[len++] = (SensorData[1]>>8 & 0xff);
					temp[len++] = (SensorData[1]>>0 & 0xff);
				
					CalcCRC16(temp,len);
											
					HAL_UART_Transmit(&huart2, temp, (len+2),0xFFFF);							
				}
			
				break;
			
			
			case 0xA5: ///3��ֱ�������ѹ
			
					///rev: 0xA5	0x03	0x00	0x00	0x00	0x02	0xc4	0x38
			
				break;
			
			case 0x06: ///�޸ĵ�ַ
				if(UART_RX_UART2.USART_RX_BUF[0] == Rs485Addr && UART_RX_UART2.USART_RX_Len == 0x09)	
				{
					///Rs485Addr	0x06	0x04	0x00	0x00	0x00	new	0x7b	0xa7
					
					memcpy(temp, UART_RX_UART2.USART_RX_BUF, UART_RX_UART2.USART_RX_Len);
					
					HAL_UART_Transmit(&huart2, temp, UART_RX_UART2.USART_RX_Len,0xFFFF);	
					
					///Rs485Addr = new
					
					Rs485Addr = temp[6];
				}
			
			
				break;
			
			default:
				break;		
		}
	}	
	memset(UART_RX_UART2.USART_RX_BUF, 0, UART_RX_UART2.USART_RX_Len);
	UART_RX_UART2.USART_RX_Len = 0;	
	
	Adc.CollectEcEnable = true;
}

/*
 *	CalcCRC16:	����CRC16У��ֵ
 *	data:		����ָ��
 *	len:		���ݳ���
 *	����ֵ��	16λ��CRCУ��ֵ
 */
uint16_t CalcCRC16(uint8_t *data, uint8_t len)
{
	uint16_t result = 0xffff;
	uint8_t i, j;

	for (i=0; i<len; i++)
	{
		result ^= data[i];
		for (j=0; j<8; j++)
		{
			if ( result&0x01 )
			{
					result >>= 1;
					result ^= 0xa001;
			}
			else
			{
					result >>= 1;
			}
		}
	}
	GET_CRC(&(data[len]), result);
	
	return result;
}

