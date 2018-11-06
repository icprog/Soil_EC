

#include <string.h>
#include "user_app.h"
#include "usart.h"

uint8_t Rs485Addr = 0;

#define USER_FLASH_SIZE			32 * 1024

uint32_t writeFlashData = 0x55555555;
uint32_t addr = 0x08007FF8;

/*
 *	FlashRead32:		读取4字节(32位)数据
 *	参数ReadAddr：		该数据在Flash中的地址
 *	返回值：			返回读取到的数据		
 */
uint32_t FlashRead32(uint32_t ReadAddr )
{
	if(ReadAddr<FLASH_BASE||ReadAddr>FLASH_BASE+USER_FLASH_SIZE)
		return 0;
	
	return (uint32_t)(*(__IO uint32_t *)ReadAddr);
}

/*
 *	FlashRead16:		读取2字节(16位)数据
 *	参数ReadAddr：		该数据在Flash中的地址
 *	返回值：			返回读取到的数据		
 */
uint16_t FlashRead16(uint32_t ReadAddr )
{
	if(ReadAddr<FLASH_BASE||ReadAddr>FLASH_BASE+USER_FLASH_SIZE)
		return 0;
	return (uint16_t)(*(__IO uint16_t *)ReadAddr);
}

/*
 *	FlashReadPage:		读取1页数据
 *	参数PageAddr：		页地址
 *	参数pBuffer：		保存读取到的数据的指针
 *	返回值：			1成功 0失败	
 */
uint8_t FlashReadPage(uint32_t PageAddr, uint32_t *pBuffer)
{
	uint32_t Address = 0;
	if(PageAddr<FLASH_BASE||PageAddr+FLASH_PAGE_SIZE>FLASH_BASE+USER_FLASH_SIZE)
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
	if(PageAddr%FLASH_PAGE_SIZE!=0)
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
	Address = PageAddr;
	while (Address < PageAddr+FLASH_PAGE_SIZE)
	{
		*pBuffer = *(__IO uint32_t *)Address;
		Address = Address + 4;
		pBuffer++;
	}
	return 1;
}

/*
 *	FlashWritePage:		写1页数据
 *	参数PageAddr：		页地址
 *	参数pBuffer：		用于写入Flash中的数据指针
 *	返回值：			1成功 0失败	
 */
uint8_t FlashWritePage( uint32_t PageAddr, uint32_t *pPageBuffer)
{
	uint32_t Address = 0, PAGEError =0;
	if(PageAddr<FLASH_BASE||PageAddr+FLASH_PAGE_SIZE>FLASH_BASE+USER_FLASH_SIZE)
		return 0;
	
	if(PageAddr%FLASH_PAGE_SIZE!=0)
		return 0;
	//解锁
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = PageAddr;
	EraseInitStruct.NbPages     = 1;
//	printf("PageAddr:%x\r\n",PageAddr);
	
	for(uint8_t i = 0; i < 10; i ++)
	{
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
		{
			printf("ERASE FLASH ERROR: %x i = %d\r\n",HAL_FLASH_GetError(), i);
			HAL_Delay(50);
			if(i == 9)
			{
				HAL_FLASH_Lock();

				return 0;
			}
		}
		else
			break;
	}
	
	Address=PageAddr;
	while (Address < PageAddr+FLASH_PAGE_SIZE)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *pPageBuffer) == HAL_OK)
		{
			Address = Address + 4;
			pPageBuffer++;
		}
		else
		{
			printf("Write Flash Error\r\n");
			HAL_FLASH_Lock();
			return 0;
		}
	}
	HAL_FLASH_Lock();
	return 1;
}

/*
 *	FlashWrite32:		写4字节(32位)数据,写的数据不能跨页(目前的需求用不到跨页写数据)
 *	参数WriteAddr：		该数据在Flash中的地址
 *	pBuffer:			用于写入Flash中的数据指针
 *	NumToWrite			数据长度(小于页大小/4)
 *	返回值：			1成功 0失败			
 */
uint8_t FlashWrite32( uint32_t WriteAddr, uint32_t * pBuffer, uint16_t NumToWrite )
{
	uint32_t FLASH_BUF [ FLASH_PAGE_SIZE/4 ]={0};
	uint32_t PageAdd;	//WriteAddr对应的页地址
	uint32_t OffSet;	//页地址的偏移  PageAdd+OffSet=WriteAddr
	uint16_t i=0;
	//获取WriteAddr对应的页地址
	if(WriteAddr<FLASH_BASE||WriteAddr+NumToWrite*4>FLASH_BASE+USER_FLASH_SIZE)
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
		
	if(WriteAddr%4!=0)
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}	
	if(NumToWrite>FLASH_PAGE_SIZE/4)
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
	PageAdd=WriteAddr & 0xffffff00;		//L072的页大小为128(0x80),后两位清零即可
	OffSet=WriteAddr & 0x000000ff;
	if(OffSet/FLASH_PAGE_SIZE>0)//
	{
		PageAdd+=FLASH_PAGE_SIZE;
		OffSet-=FLASH_PAGE_SIZE;
	}
	if(PageAdd+FLASH_PAGE_SIZE < WriteAddr+NumToWrite*4)//数据跨页
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
	if(FlashReadPage(PageAdd,FLASH_BUF)!=1)	//先读一页，避免误擦除
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
	//修改旧数据
	
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_BUF[OffSet/4+i]=*pBuffer;
		pBuffer++;
	}
	return FlashWritePage(PageAdd,FLASH_BUF);	//擦除旧数据、写入新数据
}

/*
 *	FlashWrite16:		写2字节(16位)数据,写的数据不能跨页(目前的需求用不到跨页写数据)
 *	参数WriteAddr：		该数据在Flash中的地址
 *	pBuffer:			用于写入Flash中的数据指针
 *	NumToWrite			数据长度(小于页大小)
 *	返回值：			1成功 0失败			
 */
uint8_t FlashWrite16( uint32_t WriteAddr, uint16_t * pBuffer, uint16_t NumToWrite )
{
	uint32_t FLASH_BUF [ FLASH_PAGE_SIZE/4 ];
	uint32_t PageAdd;	//WriteAddr对应的页地址
	uint32_t OffSet;	//页地址的偏移  PageAdd+OffSet=WriteAddr
//	uint16_t i;
	
	//获取WriteAddr对应的页地址
	if(WriteAddr<FLASH_BASE||WriteAddr+NumToWrite*2>USER_FLASH_SIZE+FLASH_BASE)
		return 0;
		
	if(WriteAddr%2!=0)
		return 0;
		
	if(NumToWrite>FLASH_PAGE_SIZE/4*2)
		return 0;
	
	PageAdd=WriteAddr & 0xffffff00;
	OffSet=WriteAddr & 0x000000ff;
	if(OffSet/FLASH_PAGE_SIZE>0)//
	{
		PageAdd+=FLASH_PAGE_SIZE;
		OffSet-=FLASH_PAGE_SIZE;
	}
	if(PageAdd+FLASH_PAGE_SIZE < WriteAddr+NumToWrite*2)//数据跨页
		return 0;
	if(FlashReadPage(PageAdd,FLASH_BUF)!=1)	//先读一页，避免误擦除
		return 0;
	//修改旧数据
	memcpy(FLASH_BUF+OffSet/4,pBuffer,NumToWrite*2);
//	for(i=0;i<NumToWrite/2;i++)
//	{
//		uint32_t temp1,temp2;
//		//将16位数据拼接为32位
//		temp1=*pBuffer;
//		temp2=*(pBuffer+1);
//		FLASH_BUF[OffSet/4+i]=(temp2<<16|temp1);
//		pBuffer+=2;
//	}
//	for(uint8_t j=0;j<NumToWrite%2;j++)
//	{
//		FLASH_BUF[OffSet/4+i]=FLASH_BUF[OffSet/4+i] & 0xffff0000;	//清空后2字节
//		FLASH_BUF[OffSet/4+i]=FLASH_BUF[OffSet/4+i] | *pBuffer;
//	}
	if(FlashWritePage(PageAdd,FLASH_BUF)!=1)	//擦除旧数据、写入新数据
		return 0;
	return 1;
}

/*
*RS485： 初始化485
*参数：  无
*返回值：无
*/
void Rs485Init(void)
{
	RS485_TO_TX(  );
	Rs485Addr = 0xF8;
	
	if(FlashRead16(POSITIVE_ADDR)==0||FlashRead16(POSITIVE_ADDR)==0xffff)
	{
		Para.Positive = true;
	}
	else
	{
		Para.Positive = FlashRead16(POSITIVE_ADDR);
	}
	
	if(FlashRead16(COMPENSATE_ADDR)==0||FlashRead16(COMPENSATE_ADDR)==0xffff)
	{
		Para.Data = 0;
	}
	else
	{
		Para.Data = FlashRead16(COMPENSATE_ADDR);
	}
	
	for(uint8_t i = 0; i < 5; ++i)
	{
		printf("Para.Positive = %d Para.Data = %d\r\n ",Para.Positive,Para.Data);
	
		HAL_Delay(200);
	}
	
	RS485_TO_RX(  );
}

/*
*Rs485RevceHandle： Rs485数据处理
*参数：  			 			传感器数据
*返回值：			 			无
*/
void Rs485RevceHandle(int16_t *SenSorBuf)
{
	uint8_t temp[15] = {0};
	uint8_t len = 0;
	
	RS485_TO_TX(  );

	if(CalcCRC16(UART_RX_UART2.USART_RX_BUF,UART_RX_UART2.USART_RX_Len) == 0)
	{		
		///判断Rs485功能码
		switch(UART_RX_UART2.USART_RX_BUF[1])
		{
			case 0x03:  ///1：广播处理 fe030400000000F53C
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
					
					HAL_Delay(200); ///配合主机1s延时
					HAL_UART_Transmit(&huart2, temp, (len+2),0xFFFF);	
				}
				else if(UART_RX_UART2.USART_RX_BUF[0] == Rs485Addr) ///2：获取数据指令：温度*10、EC*1000
				{
					memset(SensorData, 0, 2);
					AdcHandle( );
					
					///rev: f80300000002D062
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
			
			
			case 0xA5: ///3：直接输出电压
			
					///rev: 0xA5	0x03	0x00	0x00	0x00	0x02	0xc4	0x38
			
				break;
			
			case 0x06: ///修改地址
				if(UART_RX_UART2.USART_RX_BUF[0] == Rs485Addr && UART_RX_UART2.USART_RX_Len == 0x09)	
				{
					///Rs485Addr	0x06	0x04	0x00	0x00	0x00	new	0x7b	0xa7
					
					memcpy(temp, UART_RX_UART2.USART_RX_BUF, UART_RX_UART2.USART_RX_Len);
					
					HAL_UART_Transmit(&huart2, temp, UART_RX_UART2.USART_RX_Len,0xFFFF);	
					
					///Rs485Addr = new
					
					Rs485Addr = temp[6];
				}		
				break;
				
			case 0x07: ///f80700010028F1BD
				if(UART_RX_UART2.USART_RX_BUF[0] == Rs485Addr && UART_RX_UART2.USART_RX_Len == 0x08)
				{
					if(UART_RX_UART2.USART_RX_BUF[3])
					{
					  Para.Positive = true;
					}
					else
						Para.Positive = false;
					
					Para.Data  = UART_RX_UART2.USART_RX_BUF[4] << 8;
				  Para.Data |= UART_RX_UART2.USART_RX_BUF[5];
									
					FlashWrite16(POSITIVE_ADDR, (uint16_t *)Para.Positive, 1);					
					FlashWrite16(COMPENSATE_ADDR, &Para.Data, 1);
					
				  temp[len++] = Rs485Addr;
					temp[len++] = 0x07;
					temp[len++] = 0x04;			
					temp[len++] = 0x00;
					temp[len++] = 0x01;
					
					temp[len++] = 0x00;
					temp[len++] = 0x01;
				
					CalcCRC16(temp,len);
				
					RS485_TO_TX(  );
					
					HAL_UART_Transmit(&huart2, temp, (len+2),0xFFFF);
					
					HAL_NVIC_SystemReset(		);
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
 *	CalcCRC16:	计算CRC16校验值
 *	data:		数据指针
 *	len:		数据长度
 *	返回值：	16位的CRC校验值
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

