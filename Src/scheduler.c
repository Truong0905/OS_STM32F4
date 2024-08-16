/*
 * scheduler.c
 *
 *  Created on: Apr 23, 2022
 *      Author: Truong
 */

/*
 * scheduler.c
 *
 *  Created on: Apr 18, 2022
 *      Author: Truong
 */


#include "stm32f446xx.h"
#include "scheduler_driver.h"



void task1_handler(void);
void task2_handler(void);
void task3_handler(void);
void task4_handler(void);



void enable_processor_fault(void) ;
__attribute__((naked))	void init_scheduler_stack(uint32_t scheduler_top_of_stack);  // Do Gán địa  chỉ bắt đầu của ngăn xếp scheduler  vào MSP  nên ko dùng code C đc mà phải dùng code asm
__attribute__((naked)) void switch_sp_to_psp(void) ;
void init_task_stack(void);
void init_systick_timer(uint32_t tick_hz);
void save_psp_value(uint32_t current_psp_value);
void update_next_task(void);
void task_delay(uint32_t tick_count );
void idle_task(void);
void Update_global_tick_count(void);
void unblock_tasks(void);
void schedule (void);

void GPIOInits(void)
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT ;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed =GPIO_OP_SPEED_FAST ;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP ;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUD ;



	// Led  orange
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10 ;
	GPIO_Init(&GpioLed);

	//Led red
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12 ;
	GPIO_Init(&GpioLed);


	// Led yellow
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2 ;
	GPIO_Init(&GpioLed);
	// Led  green
	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11 ;
	GPIO_Init(&GpioLed);







}

/*void delay ( void )
{
	for (uint32_t i = 0 ; i< 500000*4 ;i++);
}*/

 uint8_t current_task = 1 ; // Task 1 is running
uint32_t g_tick_count = 0 ;

 typedef struct
 {
       uint32_t psp_value;
       uint32_t block_count;
       uint8_t current_state;
       void (*task_handler) (void);
  }TCB_t;  //  task control block(TCB)

  TCB_t user_tasks[MAX_TASKS];

int main(void)
{
	/****************MSP*************************************/


	 enable_processor_fault(); // Do  chúng ta dùng các vùng nhớ làm stack nên phải đề phòng tránh xâm  nhập các vùng nhớ này

	//  initialized the scheduler stack, that is MSP,
	 init_scheduler_stack(SCHEDULER_STACK_START);

	// Lưu địa chỉ của các  task handler
	// tasks stack initialization to store the dummy frames
	 init_task_stack();

	 GPIOInits();

	// generate systick timer exception
	 init_systick_timer(TICK_HZ);

	 switch_sp_to_psp();
	/****************PSP*************************************/
	 task1_handler();

	for(;;);

}

void task1_handler(void)
{
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_12);
		task_delay(2000);
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_12);
		task_delay(2000);

	}
}


void task2_handler(void)
{
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_10);
		task_delay(1000);
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_10);
		task_delay(1000);
	}
}


void task3_handler(void)
{
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_2);
		task_delay(500);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_2);
		task_delay(500);
	}
}


void task4_handler(void)
{
	while(1)
	{
		 GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_11);
		task_delay(125);
		 GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_11);
		task_delay(125);
	}
}

void enable_processor_fault(void)
{
	// enable all configurable exceptions like usage fault, mem manage fault and bus fault
uint32_t *pSHCSR = (uint32_t*)0xE000ED24;
*pSHCSR |= ( 1 << 16); //mem manage
*pSHCSR |= ( 1 <<17); //bus fault
*pSHCSR |=  (1<< 18); //usage fault
}

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014 ;
	uint32_t *pSCSR = (uint32_t*)0xE000E010 ;
	uint32_t count_value = SYSTICK_TIM_CLK/tick_hz -1 ;
	//1. Clear the value of SVR
	 *pSRVR &= ~(0x00FFFFFFFF) ; // Do chỉ sử dụng 24 bit
	// 1.  load the value into SVR
	 *pSRVR |= count_value ;
	 // do some settings
	 *pSCSR |= (1<<1) ; //Enables SysTick exception request:
	 *pSCSR |= (1<<2) ; // Indicates the clock source: Processor clock.
	 // enable the systick
	 *pSCSR |= (1<<0) ; // Enables the counter:


}


__attribute__((naked))	void init_scheduler_stack(uint32_t scheduler_top_of_stack)
{
	 __asm volatile ("MSR MSP,%0": :"r"(scheduler_top_of_stack):); // Gán đại chỉ bắt đầu của ngăn xếp scheduler  vào MSP
	 __asm volatile ("BX LR") ; // LR chứa giá trị của returrn address và câu lệnh BX có nghĩa là copy giá trị của LR vào PC


}



void init_task_stack(void)
{
	user_tasks[0].current_state = TASK_READY_STATE ;
	user_tasks[1].current_state = TASK_READY_STATE ;
	user_tasks[2].current_state = TASK_READY_STATE ;
	user_tasks[3].current_state = TASK_READY_STATE ;
	user_tasks[4].current_state = TASK_READY_STATE ;

	user_tasks[0].psp_value = IDLE_STACK_START ;
	user_tasks[1].psp_value = T1_STACK_START ;
	user_tasks[2].psp_value = T2_STACK_START ;
	user_tasks[3].psp_value = T3_STACK_START ;
	user_tasks[4].psp_value = T4_STACK_START ;

	user_tasks[0].task_handler = idle_task;
	user_tasks[1].task_handler = task1_handler;
	user_tasks[2].task_handler = task2_handler;
	user_tasks[3].task_handler = task3_handler;
	user_tasks[4].task_handler = task4_handler;


	uint32_t *pPSP ;
	for (int i= 0 ; i<MAX_TASKS ; i++)
	{
		pPSP =(uint32_t *)user_tasks[i].psp_value;
		pPSP-- ;
		*pPSP = DUMMY_XPSR ; //  0x01000000
		// PC
		pPSP-- ;
		*pPSP =(uint32_t)user_tasks[i].task_handler;

		// LR
		pPSP-- ;
		*pPSP =0xFFFFFFFD;

		for (int j = 0;j<13; j++) // khởi tạo R0 -> R12 bằng 0
		{
			pPSP-- ;
			*pPSP =0;
		}

		user_tasks[i].psp_value= (uint32_t)pPSP ;


	}
}

void save_psp_value(uint32_t current_psp_value)
{
	user_tasks[current_task].psp_value= current_psp_value ;
}

uint32_t get_psp_value(void)
{
	return user_tasks[current_task].psp_value;
}
__attribute__((naked)) void switch_sp_to_psp(void)
{
	//1. initialize the PSP with TASK1 stack start address
		//  get the value of PSP of current_task
	__asm volatile ("PUSH {LR}") ; // Do LR chứa giá trị return về hàm main nhưng sau đây ta lại nhảy vào hàm khác nên LR sẽ bị đổi => cần lưu lại
	__asm volatile ("BL get_psp_value"); // Giá trị returrn sẽ tự động lưu vào R0
	__asm volatile ("MSR PSP, R0") ; // initialize PSP
	__asm volatile ("POP {LR}") ; // Lấy lại giá trị LR
	//2. change SP to PSP using CONTROL register
	__asm  volatile("MOV R0 , #0x02");
	__asm volatile ("MSR CONTROL, R0") ; // set bit thứ 2 của thanh ghi control thì sẽ chuyển qua dùng PSP
	__asm volatile("BX LR"); // copy LR vào PC đề thoát khỏi hàm

}

void update_next_task(void)
{
	int state = TASK_BLOCKED_STATE ;

	for (int i = 0 ; i< MAX_TASKS ; i++)
	{
		// current_task = {0 ,1,2,3,4}
		current_task ++ ;
		current_task = current_task % MAX_TASKS ;  // Giả sử task tiếp là task 3 => 2 %4 = 2
		state = user_tasks[current_task].current_state ;
		if ((state == TASK_READY_STATE) && (current_task != 0 ))
			break ;

	}
	if (state != TASK_READY_STATE)
		current_task = 0 ;


}

void schedule (void)
{
	uint32_t *pICSR = (uint32_t *)0xE000ED04 ;
		// pend the PenSV exception
	*pICSR |= (1<<28);
}



void task_delay(uint32_t tick_count )
{
	// disable interrupt  trước khi muốn sử dụng ác biến global
	 INTERRUPT_DISABLE() ;
	if (current_task)   // Trừ 0 ra vì nó là task IDLE
	{
	user_tasks[current_task].block_count = g_tick_count + tick_count ;
	user_tasks[current_task].current_state =TASK_BLOCKED_STATE ;
	schedule();
	}

	// enable the interrupt
INTERRUPT_ENABLE() ;

}
void idle_task(void)
{
	while(1);
}

__attribute__((naked)) void PendSV_Handler (void)
{

		/*Save the context of current tesk */

		//1. Get current running tesk's PSP value
		__asm volatile ("MRS R0,PSP");
		//2. Using that PSP value store SF2( R4 to R11)
			//Nếu dùng lệnh PUSH ở đây thì MSP sẽ bị ảnh hưởng vì trong handler mode thì MSP luôn được sử dụng => phải sử dụng store instruction
		__asm volatile ("STMDB R0!,{R4-R11}") ;  // Lưu giá trị R4-R11 vào địa chỉ lưu trong R0 , cứ mỗi lần lưu thì sẽ giảm dần địa chỉ đồng thời địa chỉ đó đc lưu lại vào R0 < do có bổ sung thêm dấu "!" >

		__asm volatile ("PUSH {LR}") ; // Do LR chứa giá trị return về hàm main nhưng sau đây ta lại nhảy vào hàm khác nên LR sẽ bị đổi => cần lưu lại
		//3. Save the current value of PSP
		__asm volatile ("BL save_psp_value") ; // R0 là đối số đầu của hàm



		/*Retrieve the context of next task */

		//1. Decide next task to run
		__asm volatile ("BL update_next_task") ;
		//2. get its past PSP value
		__asm volatile ("BL get_psp_value"); // Giá trị returrn sẽ tự động lưu vào R0
		//3. Using that PSP value retrieve SF2(R4 to R11)
		__asm volatile ("LDMIA R0!,{R4-R11}") ;
		//4. update PSP and exit
		__asm volatile ("MSR PSP,R0") ;
		__asm volatile ("POP {LR}") ; // Lấy lại giá trị LR
		__asm volatile("BX LR"); // copy LR vào PC đề thoát khỏi hàm
}


void Update_global_tick_count(void)

{
	g_tick_count ++ ;
}

void unblock_tasks(void)
{
	for (int i = 1 ; i< MAX_TASKS ; i++)  // Do phần tử thứ 0 là IDLE_task nên i bắt đầu từ 1
	{
		if(user_tasks[i].current_state != TASK_READY_STATE)
		{
			if(user_tasks[i].block_count == g_tick_count)
			{
				user_tasks[i].current_state = TASK_READY_STATE ;
			}
		}
	}
}
void SysTick_Handler (void)
{

	uint32_t *pICSR = (uint32_t *)0xE000ED04 ;
	Update_global_tick_count();
	unblock_tasks();
	// pend the PenSV exception
	*pICSR |= (1<<28);

}



void HardFault_Handler(void)
{
;
}
void MemManage_Handler(void)
{

}
void BusFault_Handler(void)
{

}
void UsageFault_Handler(void)
{

}


