/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: Apr 23, 2022
 *      Author: Truong
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"
/**
 * @brief Tạo 1 cấu trúc để lựa chọn các chức năng cho I2C
 *
 */
typedef struct
{
	uint32_t I2C_SCLSpeed; // @ I2C_SCLSpeed
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;	 //@ I2C_ACKControl
	uint8_t I2C_FMDutyCycle; // @ I2C_FMDutyCycle
} I2C_Config_t;

/**
 * @brief Tạo 1 cấu trúc để quản lý I2C
 *
 */

typedef struct
{
	/* data */
	I2C_RegDef_t *pI2Cx;	 // Chứa địa chỉ của I2Cx
	I2C_Config_t I2C_Config; // Chứa config settings của  I2C
	//// Cấu trúc thêm cho ngắt I2C
	uint8_t *pTxBuffer; // Lưu địa chỉ Tx buffer
	uint8_t *pRxBuffer; // Lưu địa chỉ cho Rx Bufer
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState; // do I2C truyềnbans song công nên chỉ cần tạo 1 biến lưu trạng tái của truyền hoặc nhận
	uint8_t DevAddr;   // Lưu địa chỉ của slave hoặc tạo địa chỉ của chính nó
	uint32_t RxSize;   // size của vùng nhớ chứa data recive
	uint8_t SR;		   // Kiểm tra xem có start tiếp hay dừng
} I2C_Handle_t;

/*
 * @ I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM 100000	   // Standard-mode (up to 100 kHz)
#define I2C_SCL_SPEED_SM4K 400000  // Fast-mode (up to 400 kHz)
#define I2C_SCL_SPEED_SM2K 200000  // Fast-mode (up to 200 kHz)
#define I2C_SCL_SPEED_SM1M 1000000 // Fast-mode Plus (up to 1 MHz)

/*
 *@ I2C_ACKControl
 */
#define I2C_ACK_ENABLE 1
#define I2C_ACK_DISABLE 0

/*
 * @ I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2 0
#define I2C_FM_DUTY_16_9 1

// Các cờ thông báo trong I2C

#define I2C_SB_FLAG (1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG (1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG (1 << I2C_SR1_BTF)
#define I2C_ADD10_FLAG (1 << I2C_SR1_ADD10)
#define I2C_STOPF_FLAG (1 << I2C_SR1_STOPF)
#define I2C_RxNE_FLAG (1 << I2C_SR1_RxNE)
#define I2C_TxE_FLAG (1 << I2C_SR1_RxNE)
#define I2C_BERR_FLAG (1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG (1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG (1 << I2C_SR1_AF)
#define I2C_OVR_FLAG (1 << I2C_SR1_OVR)
#define I2C_PECERR_FLAG (1 << I2C_SR1_PECERR)
#define I2C_TIMEOUT_FLAG (1 << I2C_SR1_TIMEOUT)
#define I2C_SMBALERT_FLAG (1 << I2C_SR1_SMBALERT)

#define I2C_DISABLE_SR RESET // no start replace
#define I2C_ENABLE_SR SET	 // start replace

/*
 * Trạng thái I2C
 */
#define I2C_READY 0
#define I2C_BUSY_IN_RX 1
#define I2C_BUSY_IN_TX 2

/*
 * I2C application events macros
 */

#define I2C_EV_TX_CPMLT 0
#define I2C_EV_STOP 1
#define I2C_EV_RX_CMPLT 2
#define I2C_ERROR_BERR 3
#define I2C_ERROR_ARLO 4
#define I2C_ERROR_AF 5
#define I2C_ERROR_OVR 6
#define I2C_ERROR_TIMEOUT 7

// Slave
#define I2C_EV_DATA_REQ 8
#define I2C_EV_DATA_RCV	9




/**********************************************************************************************************************
 *                      Các API thường dùng
 * *******************************************************************************************************************/

/*
 * Peripharal Clock setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
// Cấn 1 tham chiếu  đến địa chỉ  I2C cần dùng , 1 tham trị En or Di để bật/tắt xung clock

/***
 * Init and De-Init
 */
void I2C_Init(I2C_Handle_t *pI2Chandle);
void I2C_Deinit(I2C_RegDef_t *pI2Cx);

/**
 * @brief Truyển nhận dữ liệu
 *
 */

void I2C_MasterSendData(I2C_Handle_t *pI2Chandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR); // SR : start replace
void I2C_MasterReceiveData(I2C_Handle_t *pI2Chandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SR);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C , uint8_t data );
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

/**
 * @brief Cấu hình ngắt I2C và trình xử lý ngắt I2C
 *
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);		 // khởi tạo ngắt
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority); // Xét mức ưu tiên ngắt

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/**
 * @brief Mội số API điều khiển I2C
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void  I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/**
 * @brief Application callback
 *
 */
__weak void I2C_ApplicationCallback(I2C_Handle_t *pI2Chandle, uint8_t AppEv);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
