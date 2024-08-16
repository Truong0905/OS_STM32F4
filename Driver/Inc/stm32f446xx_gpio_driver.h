/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Mar 22, 2022
 *      Author: Truong
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/**
 * @brief  Tạo cấu trúc chọn chức năng của GPIO
 *
 */
typedef struct
{
    uint8_t GPIO_PinNumber; /* @GPIO_PIN_NUMBER */
    uint8_t GPIO_PinMode;   /* @GPIO_PIN_MODE */
    uint8_t GPIO_PinSpeed;  /* @GPIO_PIN_SPEED */
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;

} GPIO_PinConfig_t;

/**
 * @brief Tạo 1 cấu trúc quản lý GPIO
 *
 */
typedef struct
{
    /* data */
    GPIO_RegDef_t *pGPIOx;           // Chứa địa chỉ của GPIOx port
    GPIO_PinConfig_t GPIO_PinConfig; // Chứa GPIO pin config settings
} GPIO_Handle_t;

/**
 * Macro  các pin mode
 * @GPIO_PIN_MODE
 */
// non-Interrupt mode
#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3
// Interrupt mode
#define GPIO_MODE_IT_FT 4  //  input falling edge
#define GPIO_MODE_IT_RT 5  //  input rising edge
#define GPIO_MODE_IT_RFT 6 // IT : input rising edge, falling edge trigger

/**
 * Macro các kiểu output
 *
 */
#define GPIO_OP_TYPE_PP 0 // push pull
#define GPIO_OP_TYPE_OD 1 // open drain

/**
 * Macro các  output speed
 * @GPIO_PIN_SPEED
 */
#define GPIO_OP_SPEED_LOW 0
#define GPIO_OP_SPEED_MEDIUM 1
#define GPIO_OP_SPEED_FAST 2
#define GPIO_OP_SPEED_HIGH 3

/**
 * GPIO pin pull up or pull down config macro
 *
 */
#define GPIO_NO_PUD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

/**
 * Macro PIN NUMBER
 * @GPIO_PIN_NUMBER
 */
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

/**********************************************************************************************************************
 *                      Các API thường dùng
 * *******************************************************************************************************************/

/*
 * Peripharal Clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
// Cấn 1 tham chiếu  đến địa chỉ  GPIO cần dùng , 1 tham trị En or Di để bật/tắt xung clock

/***
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOhandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

/***
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
// Cần biến con trỏ dể trỏ đến địa chỉ của Port GPIO caanf dùng, 1 biến để xác định Pin cần dùng và đây là hàm có trả về 1 hoặc 0 để chỉ trạng thái pin
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
// Vì 1 port có 16pin nên dùng hàm uint16_t và ta cũng cần 1 tham chiếu đến địa chỉ Port GPIO cần dùng
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
// Tuoơng tự read pin nhưng có thêm đối số đển biết đc pin đang set hay reset
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/***
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);      // khởi tạo ngắt
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority); // Xét mức ưu tiên ngắt
void GPIO_IRQHandling(uint8_t PinNumber);                             // Trình xử lý ngắt

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
