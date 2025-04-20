#include "main.h"
#include "spi.h"
#include "DRV8353.H"

DRV_8353_info DRV_info; 

uint16_t SPI_ReadWrite_DRV8353(uint16_t ReadAddr)    
{
	uint16_t value; 
	DRV8353_NSS_LOW();     
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ReadAddr, (uint8_t*)&value, 1, 1000);    
	DRV8353_NSS_HIGH();  
  return value;      
}   
void Set_DRV8353(void)   
{   
	DRV8353_ENABLE_ON(); 
	HAL_Delay(1000);///   
	DRV8353_PWM_cotr_OFF();
                        //                               写 地址 10-8                
	SPI_ReadWrite_DRV8353(0x1420); //address 02 0b0 0010 100 0010 0000=0x1420
	SPI_ReadWrite_DRV8353(0x1bff); //address 03 0b0 0011 011 1111 1111
	SPI_ReadWrite_DRV8353(0x27ff); //address 04 0b0 0100 111 1111 1111
	SPI_ReadWrite_DRV8353(0x2968); //address 05 0b0 0101 001 0110 1000
	SPI_ReadWrite_DRV8353(0x32c3); //address 06 0b0 0110 010 1100 0011
  HAL_Delay(300); 
	DRV_info.TIMP0 = SPI_ReadWrite_DRV8353(0x8000);//取地址0x00
	DRV_info.TIMP1 = SPI_ReadWrite_DRV8353(0x8800);//取地址0x01
  DRV_info.TIMP2= SPI_ReadWrite_DRV8353(0x9000);//取地址0x02
  DRV_info.TIMP3= SPI_ReadWrite_DRV8353(0x9800);//取地址0x03
  DRV_info.TIMP4= SPI_ReadWrite_DRV8353(0xA000);//取地址0x04
  DRV_info.TIMP5= SPI_ReadWrite_DRV8353(0xA800);//取地址0x05
  DRV_info.TIMP6= SPI_ReadWrite_DRV8353(0xB000);//取地址0x06
	DRV8353_PWM_cotr_ON(); 
}
