/*
 * SPI_SendData.c
 *
 *  Created on: Apr 23, 2022
 *      Author: Truong
 */



#include <string.h>
#include "stm32f446xx_spi_driver.h"
#include <stdio.h>
void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++)
		;
}

void SPI1_GPIOInits(void)
{
	GPIO_Handle_t pSPIins;
	pSPIins.pGPIOx = GPIOB;
	pSPIins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	pSPIins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	pSPIins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pSPIins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD;

	// SCLK
	pSPIins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&pSPIins);

	// MOSI
	pSPIins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&pSPIins);
	// MISO
	//	pSPIins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14 ;
	//	GPIO_Init(&pSPIins) ;
	// NSS
	//	pSPIins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12 ;
	//	GPIO_Init(&pSPIins) ;
}

void SPI1_Inints(void)
{
	SPI_Handle_t pSPIHandle;
	pSPIHandle.pSPIx = SPI1;
	pSPIHandle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	pSPIHandle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	pSPIHandle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // 8MHZ
	pSPIHandle.SPI_Config.SPI_DS = SPI_DS_8BITS;
	pSPIHandle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	pSPIHandle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	pSPIHandle.SPI_Config.SPI_SSM = SPI_SSM_EN;
	SPI_Init(&pSPIHandle);
}
void GPIO_BT_Inint(void)
{
	GPIO_Handle_t GpioBt;
	GpioBt.pGPIOx = GPIOC;
	GpioBt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBt.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_FAST;
	GpioBt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD;
	GPIO_Init(&GpioBt);
}
int main(void)
{
	char Data[] = "Hello World";
	uint8_t Size_of_Data = strlen(Data);
	SPI1_GPIOInits();
	SPI1_Inints();
	 GPIO_BT_Inint();
	//  SPI_SSOEConfig(SPI1, ENABLE); // // Chỉ dùng khi SSM = 0 ;
	  SPI_SSIConfig(SPI1, ENABLE); // Chỉ dùng khi SSM = 1 ;
	  while(1)
	{
		while( ! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		delay() ;
		SPI_PeripheralControl(SPI1 ,ENABLE); // Bật SPI2
			 // SPI_SendData(SPI2, (uint8_t *)&Size_of_Data,1 );
		SPI_SendData(SPI1, (uint8_t *)Data, Size_of_Data);
		while (SPI_GetFlagStatus(SPI1,SPI_BSY_FLAG));
		SPI_PeripheralControl(SPI1 ,DISABLE); // Tắt SPI2

	}


	return 0;
}
