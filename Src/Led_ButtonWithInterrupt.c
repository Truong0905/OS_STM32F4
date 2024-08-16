#include <string.h>
#include "stm32f446xx.h"
void delay(void)
	{
		for(uint32_t i = 0 ;i<5000;i++);
	}
int main(void)
{		GPIO_Handle_t GpioLed, GpioBt ;
		memset(&GpioLed,0,sizeof(GpioLed));  // đặt tất cả dữ liệu của cấu trúc về 0  #include <string.h>
		memset(&GpioBt,0,sizeof(GpioBt));
		// cấu hình nháy led

	GpioLed.pGPIOx = GPIOC ;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10 ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT ;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed =GPIO_OP_SPEED_FAST ;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP ;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUD ;
	GPIO_Init(&GpioLed);
	 // Cấu hình button phát ngắt theo sườn xuống
	GpioBt.pGPIOx = GPIOC ;
	GpioBt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13 ;
	GpioBt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT ;
	GpioBt.GPIO_PinConfig.GPIO_PinSpeed =GPIO_OP_SPEED_FAST ;
	GpioBt.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUD ;
 	GPIO_Init(&GpioBt);

	// cấu hình ngắt
	  GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10,NVIC_IRQ_PRI15) ;
	  GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE) ;
	  for (;;)
	  {
			 delay();
	  }

	 return 0;

}

void EXTI15_10_IRQHandler (void)
{
	// delay();
	GPIO_IRQHandling (GPIO_PIN_NO_13) ; // Xóa thông báo ngắt đến
	//Viết chương trình ngắt từ đây
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_10);


}
