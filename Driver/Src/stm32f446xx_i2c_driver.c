/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Apr 23, 2022
 *      Author: Truong
 */

#include "stm32f446xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
/// Ngắt

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);



uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_PreScaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, systempCLK;
	uint8_t temp, ahbp, apb1p;
	uint8_t clksrc; // Xác định nguồn cung cấp clock cho MCU
	clksrc = (RCC->CFGR >> 2) & (0x3);
	if (clksrc == 0)
	{
		// HSI
		systempCLK = 16000000;
	}
	else if (clksrc == 1)
	{
		// HSE
		systempCLK = 8000000;
	}
	else
	{
		// PLL
		;
	}
	// AHB
	temp = (RCC->CFGR >> 4 & 0xF);
	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}
	// APB1
	temp = (RCC->CFGR >> 10 & 0x7);
	if (temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (systempCLK / ahbp) / apb1p;

	return pclk1;
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{

	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)

{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 = (1 << I2C_CR1_PE);
	}
	else
	{

		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

void I2C_Init(I2C_Handle_t *pI2Chandle)
{
	I2C_PeriClockControl(pI2Chandle->pI2Cx, ENABLE);
	uint32_t tempreg = 0;
	//  ACKing control bit
	tempreg |= pI2Chandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2Chandle->pI2Cx->CR1 |= tempreg;

	// Config the FREQ filed of CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2Chandle->pI2Cx->CR2 |= (tempreg & 0x3F);

	// Program device own address
	tempreg = 0;
	tempreg = pI2Chandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 < 14); // Phải luôn set bit này lên 1
	pI2Chandle->pI2Cx->OAR1 |= tempreg;

	// CCR calculation
	uint16_t ccr_value = 0;
	tempreg = 0;

	if (pI2Chandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2Chandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{

		tempreg |= (1 << 15); // Set bit chon Fast mode
		tempreg |= (pI2Chandle->I2C_Config.I2C_FMDutyCycle << 14);
		if (pI2Chandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2Chandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2Chandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2Chandle->pI2Cx->CCR |= tempreg;

	//  Trise configuration
	tempreg = 0;

	if (pI2Chandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1; // maximum trise in standard mode is 1000ns
	}
	else
	{
		// Fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1; // maximum trise in standard mode is 300ns
	}

	pI2Chandle->pI2Cx->TRISE |= (tempreg & 0x3F);
}

void I2C_Deinit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/*************************************************************************************************/

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{

	SlaveAddr = SlaveAddr << 1; // Lấy địa chỉ 7 bit
	SlaveAddr |= 1;				// Chọn chức năng nhận data từ slave
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // Lấy địa chỉ 7 bit
	SlaveAddr &= ~(1);			// Chọn chức năng gửi data đến slave
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;

	// check for device mode
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		// Device is in  Master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			if (pI2CHandle->RxSize == 1)
			{
				// first disable the ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				//  clear the ADDR flag ( read SR1 , read SR2 )
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
	}
	else
	{
		// Device is in  Slave mode
	}
}

 void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == I2C_ACK_ENABLE)
	{
		// Enable the ACK
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		// disable the ACK
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

void I2C_MasterSendData(I2C_Handle_t *pI2Chandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR)
{

	// 1 . generate the START condition
	I2C_GenerateStartCondition(pI2Chandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in the SR1.
	// note that until SB is cleared SCL will be stretched, that is ( pulled to low)
	while (!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_SB_FLAG))
		;

	// 3. send the address of the slave read write it set to w(0) ( total 8 bit )
	I2C_ExecuteAddressPhaseWrite(pI2Chandle->pI2Cx, SlaveAddr);

	// 4.  confirm that address phase is completed by checking the ADDR flag in the SR1.
	while (!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_ADDR_FLAG))
		;

	// 5. Clear the ADDR flag according  to its  software sequence
	// note that until ADDR is cleared SCL will be stretched, that is ( pulled to low)
	I2C_ClearADDRFlag(pI2Chandle);

	// 6. send data until the length becomes zero
	while (Len > 0)
	{
		while (!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_TxE_FLAG))
			; // Wait till TXT set

		pI2Chandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// 7. when length becomes 0,wait for TXE =1 and BTF = 1 before generating the STOP condition
	// Note. TXE = 1 , BTF = 0 , means that both SR and DR  are empty  and next transmission should begin
	// When BTF = 1 	SCL will be stretch ( pulled to Low )
	while (!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_TxE_FLAG))
		;

	while (!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_BTF_FLAG))
		;

	// 8.generate the STOP condition
	if (SR == I2C_ENABLE_SR)
	{
		I2C_GenerateStopCondition(pI2Chandle->pI2Cx);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2Chandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR)
{
	// 1 . generate the START condition
	I2C_GenerateStartCondition(pI2Chandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in the SR1.
	// note that until SB is cleared SCL will be stretched, that is ( pulled to low)
	while (!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_SB_FLAG))
		;

	// 3.  send the address of the slave with R/W bit  set to 1
	I2C_ExecuteAddressPhaseRead(pI2Chandle->pI2Cx, SlaveAddr);

	// 4. Wait until address phase is completed by checking the ADDR flag in reg SR1

	while (!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_ADDR_FLAG))
		;

	// 5.1  procedure to read only one byte from slave

	if (Len == 1)
	{

		//  disable Acking.
		I2C_ManageAcking(pI2Chandle->pI2Cx, I2C_ACK_DISABLE);

		//  clear the ADDR flag
		I2C_ClearADDRFlag(pI2Chandle);

		//  wait until RXNE becomes 1
		while (!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_RxNE_FLAG))
			;

		if (SR == I2C_ENABLE_SR)
		{
			I2C_GenerateStopCondition(pI2Chandle->pI2Cx);
		}
		// read the data into buffer
		*pRxBuffer = pI2Chandle->pI2Cx->DR;
	}

	// 5.2 read data from slave when length is greater than 1

	if (Len > 1)
	{
		//  clear the ADDR flag
		I2C_ClearADDRFlag(pI2Chandle);

		// read data until Len become zero
		for (uint32_t i = Len; i > 0; i--)
		{
			//  wait until RXNE becomes 1
			while (!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_RxNE_FLAG))
				;

			if (i == 2) // reading the last 2 bytes
			{
				//  disable Acking.
				I2C_ManageAcking(pI2Chandle->pI2Cx, I2C_ACK_DISABLE);
				if (SR == I2C_ENABLE_SR)
				{
					I2C_GenerateStopCondition(pI2Chandle->pI2Cx);
				}
			}
			// read the data into buffer
			*pRxBuffer = pI2Chandle->pI2Cx->DR;
			// increment the buffer address
			pRxBuffer++;
		}
	}
	//  re enable ACKing
	if (pI2Chandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2Chandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

//----------------------------------------------------------- Cấu hình ngắt ------------------------------------------------------------------------------------

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) // Xét mức ưu tiên ngắt
{

	// 1. Tìm IPR resister (Interrupt Priority Registers)
	uint8_t iprx = IRQNumber / 4;		  // XÁc định xem ngắt nằm ở thanh ghi nào từ 0 - 59
	uint8_t iprx_section = IRQNumber % 4; // Xác định xme ngắt nằm ở section nào ( mỗi thanh ghi có 4 section , 8 bit cho 1 section => 1 thanh ghi kiểm soát mức ưu tiên cho 4 ngắt)
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) = (IRQPriority << shift_amount);

	// Giả sử ngắt nằm ở thanh ghi thứ 1 => địa chỉ là Pr_base_Addr + 1*4 = pr_base_addr 0x04   (Do đó nhân 4 vì mỗi lần ofset là 0x04)
	// giả sử nằm ở section 1 => bắt đầu từ bit thứ 8 => lùi sang trái 8 lần . do đó phải nhân với 8
	// Nhuwng do chỉ có 4 bit cao của mỗi section mới có tác dụng nên ta phải lùi sang trái 4 bit . vD mức ưu tiên 0000_0001 => 0001_0000
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) // khởi tạo ngắt
{
	// config NVIC register

	if (EnorDi == ENABLE) // Các thanh ghi này chỉ có thể enable ko thể disable
	{
		if (IRQNumber <= 31) // 0-31
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64) // 32-63
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32); // Giả sử  45 %32 = 13 => vị trí thứ 13 trong thanh ghi  ( bắt daauf từ 0)
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) // 64 -95
		{
			// program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber % 64); // Giả sử  70 %64 = 6 => vị trí thứ 6 trong thanh ghi  ( Bắt đầu từ 0)
		}
	}
	else // Các thanh ghi này chỉ có thể disable  ko thể enable
	{
		if (IRQNumber <= 31) // 0-31
		{
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64) // 32-63
		{
			// program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) // 64 -95
		{
			// program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->SR = Sr;

		// Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Implement the code to enable ITEVEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; // Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->SR = Sr;

		// Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Implement the code to enable ITEVEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxLen > 0)
	{
		// 1. load the data into DR.
		pI2CHandle->pI2Cx->DR = *pI2CHandle->pTxBuffer;
		// 2.decrement the TX length
		pI2CHandle->TxLen--;
		// 3.Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	// We have to do that data reception
	if (pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1)
	{
		if (pI2CHandle->RxSize == 2) // reading the last 2 bytes
		{
			//  disable Acking.
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}
		// Read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize == 0)
	{
		// close the I2C data reception and notify the application
		// 		1.generate the STOP condition

		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// 		2. Close the I2C RX
		I2C_CloseReceiveData(pI2CHandle);

		// 		3. notified the application

		I2C_ApplicationCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

 void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{

	// Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Implement the code to disable ITEVEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

 void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Implement the code to disable ITEVEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	// Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN); // Event interrupt enable
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN); // Buffer interrupt enable

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	// 1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode

	if (temp1 && temp3) // Start condition generated
	{
		// SB flag is set
		// Chỉ thực hiện với master vì slave ko có start nên SB luôn  = 0
		//  in this block, let's execute the address phase
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	// 2. Handle For interrupt generated by ADDR event
	// Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if (temp1 && temp3)
	{
		// ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	// 3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if (temp1 && temp3)
	{
		// BTF flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//  make sure that TXE is also set
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE)) // confirm that BTF and TXE both are set.
			{												 // BTF =1, TXE =1
				if (pI2CHandle->TxLen == 0)
				{
					// 1. generate the STOP condition
					if (pI2CHandle->SR == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// 2.  reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					// 3 . notify the application about transmission complete.
					I2C_ApplicationCallback(pI2CHandle, I2C_EV_TX_CPMLT);
				}
			}
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	// 4. Handle For interrupt generated by STOPF event
	//  Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	if (temp1 && temp3)
	{
		// Stop flag is set
		// Clear stop flag
		//			 1 . Read SR1  < Đã thực hiện ở  : temp3 =  pI2CHandle->pI2Cx->SR1 &(1<< I2C_SR1_STOPF) ; >
		//			2. Write CR1  < something >
		pI2CHandle->pI2Cx->CR1 |= 0x000;
		// notify the application that STOP is generated by the master or STOP is detected.
		I2C_ApplicationCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);
	// 5. Handle For interrupt generated by TXE event
	if (temp1 && temp2 && temp3)
	{
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) // Xác nhận là đang trong chế độ master
		{
			// TXe flag is set => DR empty
			// we have to do the data transmission.
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else   // Xác nhận là đang trong chế độ slave
		{
			// make sure that slave is a really in transmitter MODE
			if (pI2CHandle->pI2Cx->SR2 & (1<< I2C_SR2_TRA))
			{
				I2C_ApplicationCallback(pI2CHandle, I2C_EV_DATA_REQ) ;
			}

		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
	// 6. Handle For interrupt generated by RXNE event

	if (temp1 && temp2 && temp3)
	{
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) // Xác nhận là đang trong chế độ master
		{
			// RXNe flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else   // Xác nhận là đang trong chế độ slave
		{
			// make sure that slave is a really in reciever  MODE
			if (! ( pI2CHandle->pI2Cx->SR2 & (1<< I2C_SR2_TRA)) )
			{
				I2C_ApplicationCallback(pI2CHandle, I2C_EV_DATA_RCV) ;
			}
		}

	}
}

void  I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		// Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Implement the code to enable ITEVEN Control Bit
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Implement the code to enable ITERREN Control Bit
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else
	{
		// Implement the code to disable ITBUFEN Control Bit
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

		// Implement the code to disable ITEVEN Control Bit
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

		// Implement the code to disable ITERREN Control Bit
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1, temp2;

	// Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if (temp1 && temp2)
	{
		// This is Bus error

		// Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		// Implement the code to notify the application about the error
		I2C_ApplicationCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
	if (temp1 && temp2)
	{
		// This is arbitration lost error

		// Implement the code to clear the arbitration lost error flag

		// Implement the code to notify the application about the error
	}

	/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if (temp1 && temp2)
	{
		// This is ACK failure error

		// Implement the code to clear the ACK failure error flag

		// Implement the code to notify the application about the error
	}

	/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if (temp1 && temp2)
	{
		// This is Overrun/underrun

		// Implement the code to clear the Overrun/underrun error flag

		// Implement the code to notify the application about the error
	}

	/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if (temp1 && temp2)
	{
		// This is Time out error

		// Implement the code to clear the Time out error flag

		// Implement the code to notify the application about the error
	}
}




// -------------------------- Slave --------------------------------------------------------------------------------------------------------------


void I2C_SlaveSendData(I2C_RegDef_t *pI2C , uint8_t data )
{
	pI2C->DR = data ;

}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t)pI2C->DR ;
}


