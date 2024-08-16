/*
 * stm32f767xx_gpio_driver.c
 *
 *  Created on: Mar 14, 2022
 *      Author: Truong
 */

#include "stm32f446xx_gpio_driver.h"

/*
 * Peripharal Clock setup
 */

/**  GPIO_PeriClockControl
 * @brief       Dùng để bật hoặc tắt xung clock cấp cho GPIO port
 *
 * @param pGPIOx        địa chỉ của port GPIO
 * @param EnorDi        ENABLE or DISABLE macros
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
     if (EnorDi == ENABLE)
     {
          if (pGPIOx == GPIOA)
          {
               GPIOA_PCLK_EN();
          }
          else if (pGPIOx == GPIOB)
          {
               GPIOB_PCLK_EN();
          }
          else if (pGPIOx == GPIOC)
          {
               GPIOC_PCLK_EN();
          }
          else if (pGPIOx == GPIOD)
          {
               GPIOD_PCLK_EN();
          }
          else if (pGPIOx == GPIOE)
          {
               GPIOE_PCLK_EN();
          }
          else if (pGPIOx == GPIOF)
          {
               GPIOF_PCLK_EN();
          }
          else if (pGPIOx == GPIOG)
          {
               GPIOG_PCLK_EN();
          }
          else if (pGPIOx == GPIOH)
          {
               GPIOH_PCLK_EN();
          }
     }
     else
     {
          if (pGPIOx == GPIOA)
          {
               GPIOA_PCLK_DI();
          }
          else if (pGPIOx == GPIOB)
          {
               GPIOB_PCLK_DI();
          }
          else if (pGPIOx == GPIOC)
          {
               GPIOC_PCLK_DI();
          }
          else if (pGPIOx == GPIOD)
          {
               GPIOD_PCLK_DI();
          }
          else if (pGPIOx == GPIOE)
          {
               GPIOE_PCLK_DI();
          }
          else if (pGPIOx == GPIOF)
          {
               GPIOF_PCLK_DI();
          }
          else if (pGPIOx == GPIOG)
          {
               GPIOG_PCLK_DI();
          }
          else if (pGPIOx == GPIOH)
          {
               GPIOH_PCLK_DI();
          }
     }
}

/***
 * Init and De-Init
 */

/************************************************************************************************************************************/

/** GPIO_Init
 * @brief   Khởi tạo pin
 *
 * @param pGPIOhandle   chứa địa chỉ của port GPIO chứa pin sửa dụng  và cấu hình pin
 */
void GPIO_Init(GPIO_Handle_t *pGPIOhandle)
{
     GPIO_PeriClockControl(pGPIOhandle->pGPIOx, ENABLE);

     uint32_t temp = 0;                                                // Biến tạm thời
                                                                       // 1. Config mode of gpio pin
     if (pGPIOhandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) // Nhỏ hơn hoặc bằng 3 là non-interrupt mode
     {
          temp = (pGPIOhandle->GPIO_PinConfig.GPIO_PinMode << (2 * (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber)));
          // Xác định xem pin thuộc mode gì và nằm ở vị trí pin bao nhiêu << nhân 2 vì mỗi pin có 2 bit điều khiển
          pGPIOhandle->pGPIOx->MODER &= ~(0x03 << pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber); // clear 2 bit cần setting trước khi setting
          pGPIOhandle->pGPIOx->MODER |= temp;
          // Truy cập vào địa chỉ GPIO cần setup -> Truy cập đến địa chỉ thanh ghi setup mode
     }
     else
     {
          // interrupt mode
          if (pGPIOhandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) // Xung sườn xuóng
          {
               // 1. configure the falling trigger selection register (FTSR)
               EXTI->FTSR |= (1 << (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber));
               // Clear the corresponding RTSR bit
               EXTI->RTSR &= ~(1 << (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber));
          }
          else if (pGPIOhandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) // Xung sườn lên
          {
               // 1. configure the Rising trigger selection register (RTSR)
               EXTI->RTSR |= (1 << (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber));
               // Clear the corresponding FTSR bit
               EXTI->FTSR &= ~(1 << (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber));
          }
          else if (pGPIOhandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) // 2 cạnh
          {
               // 1. configure the FTSR and RTSR
               EXTI->FTSR |= (1 << (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber));
               EXTI->RTSR |= (1 << (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber));
          }

          // 2. configure the GPIO port selection in SYSCFG_EXTI
          uint8_t temp1 = (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber) / 4; // Xác định xem PinX thuôc thanh ghi EXTICR nào. ĐÂy là mảng nên bắt đầu từ ko nên nếu chia cho 4 dư 1 nghĩa là thanh ghi thứ 2
          uint8_t temp2 = (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber) % 4; // Xác định xem Pin thuộc EXTI nào
          uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOhandle->pGPIOx);
          SYSCFG_PLCK_EN();
          SYSCFG->EXTICR[temp1] = portcode << (4 * temp2);
          // 3. enable the EXTI interrupt delivery using IMR  ( Interrupt mask register )
          EXTI->IMR |= (1 << (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber));
     }
     temp = 0;
     // 2. config the speed
     temp = (pGPIOhandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber)));
     pGPIOhandle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber); // clear 2 bit cần setting trước khi setting
     pGPIOhandle->pGPIOx->OSPEEDR |= temp;
     temp = 0;
     // 3. config the pull up or the pull down settings
     temp = (pGPIOhandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber)));
     pGPIOhandle->pGPIOx->PUPDR &= ~(0x03 << pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber); // clear 2 bit cần setting trước khi setting
     pGPIOhandle->pGPIOx->PUPDR |= temp;
     temp = 0;
     // 4. config out put type
     temp = (pGPIOhandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber));
     pGPIOhandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber); // clear 2 bit cần setting trước khi setting
     pGPIOhandle->pGPIOx->OTYPER |= temp;
     temp = 0;
     // 5. config the alt functionality
     if (pGPIOhandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
     {
          uint8_t temp1, temp2;
          temp1 = pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber / 8; // Xác định xem nằm ở GPIO alternate function low OR highh register . Chia lấy phần nguyên
          temp2 = pGPIOhandle->GPIO_PinConfig.GPIO_PinNumber % 8; // Xác định xem thuộc bit bao nhiêu của GPIO alternate function low/highh register . Chia lấy phần dư
          pGPIOhandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
          pGPIOhandle->pGPIOx->AFR[temp1] |= pGPIOhandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
          // Do mỗi pin cần 4 bit điều khiển
     }
};

/************************************************************************************************************************************/
/**
 * @brief  Dừng sử dụng GPIO pin
 *
 * @param pGPIOx  địa chỉ của port GPIO chưa pin
 */
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
{
     if (pGPIOx == GPIOA)
     {
          GPIOA_REG_RESET();
     }
     else if (pGPIOx == GPIOB)
     {
          GPIOB_REG_RESET();
     }
     else if (pGPIOx == GPIOC)
     {
          GPIOC_REG_RESET();
     }
     else if (pGPIOx == GPIOD)
     {
          GPIOD_REG_RESET();
     }
     else if (pGPIOx == GPIOE)
     {
          GPIOE_REG_RESET();
     }
     else if (pGPIOx == GPIOF)
     {
          GPIOF_REG_RESET();
     }
     else if (pGPIOx == GPIOG)
     {
          GPIOG_REG_RESET();
     }
     else if (pGPIOx == GPIOH)
     {
          GPIOH_REG_RESET();
     }
}

/***
 * Data read and write
 */

/************************************************************************************************************************************/

/**
 * @brief  Đọc dữ liệu từ 1 Pin
 *
 * @param pGPIOx         địa chỉ của port GPIO chưa pin
 * @param PinNumber    Pin số bao nhiêu
 * @return uint8_t       GIá trị đọc được là 1 hay 0
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
     uint8_t value;
     value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); // GPIO port input data register dichj chuyển về bit 0th
     // Vầ cần trả về giá trị 1 hoặc 0 nên phải lui về bit 0th
     return value;
}

/************************************************************************************************************************************/

/**
 * @brief  Đọc dữ liệu từ 1 port GPIO
 *
 * @param pGPIOx    địa chỉ của port GPIO
 * @return uint16_t  kiểu dữ liệu 16 bit vì 1 port có 16 chân
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
     uint16_t value;
     value = (uint16_t)(pGPIOx->IDR);
     return value;
}

/************************************************************************************************************************************/

/**
 * @brief Ghi dữ liệu ra 1 pin
 *
 * @param pGPIOx  địa chỉ của port GPIO
 * @param PinNumber Pin số bao nhiêu
 * @param value  GPIO_PIN_SET  or GPIO_PIN_RESET
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
     if (value == GPIO_PIN_SET)
     {
          pGPIOx->ODR |= (1 << PinNumber); // GPIO port output data register
     }
     else
     {
          pGPIOx->ODR &= ~(1 << PinNumber);
     }
}

/************************************************************************************************************************************/

/**
 * @brief   Ghi dữ liệu ra 1 porrt
 *
 * @param pGPIOx địa chỉ của port GPIO
 * @param value Giá trị cần ghi ra có kiểu 16 bit vì 1 port có 16 chân
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
     pGPIOx->ODR = value;
}

/************************************************************************************************************************************/

/**
 * @brief Đổi giá trị đầu ra trước đó 1 pin
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
     pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber); // Phép Xor 1^1 = 0 ; 0^1 = 1
}

/************************************************************************************************************************************/

/***
 * IRQ Configuration and ISR handling
 */

/**
 * @brief   Cấu hình ngắt cho pin
 *
 * @param IRQNumber      vị trí trong bảng vector table
 * @param EnorDi             enable hoặc disable
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

/**
 * @brief   Xét mức ưu tiên ngắt
 *
 * @param IRQPriority     Mức ưu tiên
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
     // 1. Tìm IPR resister (Interrupt Priority Registers)
     uint8_t iprx = IRQNumber / 4;         // XÁc định xem ngắt nằm ở thanh ghi nào từ 0 - 59
     uint8_t iprx_section = IRQNumber % 4; // Xác định xme ngắt nằm ở section nào ( mỗi thanh ghi có 4 section , 8 bit cho 1 section => 1 thanh ghi kiểm soát mức ưu tiên cho 4 ngắt)
     uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
     *(NVIC_PR_BASE_ADDR + iprx) = (IRQPriority << shift_amount);

     // Giả sử ngắt nằm ở thanh ghi thứ 1 => địa chỉ là Pr_base_Addr + 1*4 = pr_base_addr 0x04   (Do đó nhân 4 vì mỗi lần ofset là 0x04)
     // giả sử nằm ở section 1 => bắt đầu từ bit thứ 8 => lùi sang trái 8 lần . do đó phải nhân với 8
     // Nhuwng do chỉ có 4 bit cao của mỗi section mới có tác dụng nên ta phải lùi sang trái 4 bit . vD mức ưu tiên 0000_0001 => 0001_0000
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
     // Xóa bit thông báo ngắt trên Pending registor
     if (EXTI->PR & (1 << PinNumber)) // nếu pin thứ x có ngắt
     {
          // xóa bit
          EXTI->PR |= (1 << PinNumber); // Lưu ý xóa ở đây là set lên 1 thì có nghĩa là xóa thông báo
     }
}
