/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Apr 23, 2022
 *      Author: Truong
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/**
 * @brief Tạo 1 cấu trúc để lựa chọn các chức năng cho SPI
 *
 */
typedef struct
{
    uint8_t SPI_DeviceMode; /* @SPI_DeviceMode */
    uint8_t SPI_BusConfig;  /* @SPI_BusConfig */
    uint8_t SPI_SclkSpeed;  /* @SPI_SclkSpeed */
    uint8_t SPI_DS;         /*@SPI_DS*/
    uint8_t SPI_CPHA;       /* @SPI_CPHA */
    uint8_t SPI_CPOL;       /* @SPI_CPOL */
    uint8_t SPI_SSM;        /* @SPI_SSM */
} SPI_Config_t;

/**
 * @brief Tạo 1 cấu trúc để quản lý SPI
 *
 */

typedef struct
{
    /* data */
    SPI_RegDef_t *pSPIx;     // Chứa địa chỉ của SPIx
    SPI_Config_t SPI_Config; // Chứa config settings cuar SPI

    // Các biến dùng trong ngắt SPI
    uint8_t *pTxBuffer; // Dùng lưu địa chỉ của  Tx buffer < Địa chỉ nơi lưu trữ dữ liệu muốn truyền đi >
    uint8_t *pRxBuffer; // Dùng lưu địa chỉ của Rx buffer < Địa chỉ nơi lưu trữ dữ liệu muốn nhận  về >
    uint32_t TxLen;     // Độ dài TxBuffer
    uint32_t RxLen;     // Độ dài  Rx buffer
    uint8_t TxState;
    uint8_t RxState;

} SPI_Handle_t;

/**********************************************************************************************************************
 *                      Các API thường dùng
 * *******************************************************************************************************************/

/*
 * Peripharal Clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
// Cấn 1 tham chiếu  đến địa chỉ  SPI cần dùng , 1 tham trị En or Di để bật/tắt xung clock

/***
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIhandle);
void SPI_Deinit(SPI_RegDef_t *pSPIx);

/**
 * @brief Truyển nhận dữ liệu
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);   // Địa  chỉ SPI sử dụng , con trỏ Data và kích thước dữ liệu truyền đi
void SPi_ReciveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len); // Tương tự API gửi

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIhandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPi_ReciveDataIT(SPI_Handle_t *pSPIhandle, uint8_t *pRxBuffer, uint32_t len);

/**
 * @brief Cấu hình ngắt SPI và trình xử lý ngắt SPI
 *
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);      // khởi tạo ngắt
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority); // Xét mức ưu tiên ngắt
void SPI_IRQHandling(SPI_Handle_t *pSPIhandle);                      // Trình xử lý ngắt

/**
 * @brief Mội số API điều khiển SPI
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIhandle);
void SPI_CloseReception(SPI_Handle_t *pSPIhandle);

/**
 * @brief Application callback
 *
 */
void SPI_ApplicationCallback(SPI_Handle_t *pSPIhandle, uint8_t AppEv);

/**********************************************************************************************************************
 *                      Các macros thường dùng
 * *******************************************************************************************************************/

/*
 *   @SPI_DeviceMode
 *  MẶC ĐỊNH LÀ SLAVE
 * NẰm trong thanh ghi CR1 -> MSTR
 */
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0

/*
 *   @SPI_BusConfig
 * CR1 -> BIDIOE & BIDIMODE
 *  Nếu bit BIDIMODE = 0 ( tryền song công  ) thì ko cần qau tâm BIDIOE
 *  SPI_BUS_CONFIG_SIMPLEX_TXONLY   ko cần thiết vì chỉ cần ngắt kết nối chân này là đc
 * Đối với SPI_BUS_CONFIG_SIMPLEX_RXONLY là bit 10 trong CR1 . Chúng tâ cần cấu hình nó trong bit riêng vì SPI chỉ có xung khi master có yêu cầu
 */

#define SPI_BUS_CONFIG_FD 1             // Truyền song công
#define SPI_BUS_CONFIG_HD 2             // Truyền bán song công
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3 // chỉ gửi

/*
 *   @SPI_SclkSpeed
 * Trong thanh ghi CR1 -> BR   ( Baud rate)  /  có 3 bit cấu hình tốc độ
 */

#define SPI_SCLK_SPEED_DIV2 0   // 000
#define SPI_SCLK_SPEED_DIV4 1   // 001
#define SPI_SCLK_SPEED_DIV8 2   // 010
#define SPI_SCLK_SPEED_DIV16 3  // 011
#define SPI_SCLK_SPEED_DIV32 4  // 100
#define SPI_SCLK_SPEED_DIV64 5  // 101
#define SPI_SCLK_SPEED_DIV128 6 // 110
#define SPI_SCLK_SPEED_DIV256 7 // 111

/*
 *   @SPI_DS
 * Buffer là 4 bit -> 16 bit
 * CR2 -> DS
 */

#define SPI_DS_8BITS 0
#define SPI_DS_16BITS 1

/*
 *   @SPI_CPHA
 * CR1 -> CPHA
 */
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

/*
 *   @SPI_CPOL
 * CR1 -> CPOL
 */
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0
/*
 *   @SPI_SSM
 * chọn slave theo phần mềm hoặc không
 */
#define SPI_SSM_EN 1
#define SPI_SSM_DI 0

/*
 * LSBFIRST
 */
#define SPI_LSBFIRST_LSB 1
#define SPI_LSBFIRST_MSB 0

// Các macros dùng trong ngắt của SPI
#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2

/**
 * @brief possible SPI application events
 *
 */
#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_ERR 3
#define SPI_EVENT_CRC__ERR 4

// Các cờ thông báo trong SPI
#define SPI_TXE_FLAG (1 << SPI_SR_TXE)
#define SPI_RXE_FLAG (1 << SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG (1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG (1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG (1 << SPI_SR_CRCERR)
#define SPI_SR_MODF_FLAG (1 << SPI_SR_MODF)
#define SPI_OVR_FLAG (1 << SPI_SR_OVR)
#define SPI_BSY_FLAG (1 << SPI_SR_BSY)
#define SPI_FRE_FLAG (1 << SPI_SR_FRE)

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
