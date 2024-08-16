/*
 * scheduler_driver.h
 *
 *  Created on: Apr 23, 2022
 *      Author: Truong
 */

#ifndef INC_SCHEDULER_DRIVER_H_
#define INC_SCHEDULER_DRIVER_H_

/* Tính toán vùng nhớ stack cho mỗi task và scheduler */

#define SIZE_TASK_STACK 1024U
#define SIZE_SCHEDULER_STACK 1024U

#define SRAM_START 0x20000000U
#define SIZE ((128) * (1024))
#define SRAM_END ((SRAM_START) + (SIZE))
#define T1_STACK_START SRAM_END // RAM_END
#define T2_STACK_START ((SRAM_END) - (1 * SIZE_TASK_STACK))
#define T3_STACK_START ((SRAM_END) - (2 * SIZE_TASK_STACK))
#define T4_STACK_START ((SRAM_END) - (3 * SIZE_TASK_STACK))
#define IDLE_STACK_START ((SRAM_END) - (4 * SIZE_TASK_STACK))
#define SCHEDULER_STACK_START ((SRAM_END) - (5 * SIZE_TASK_STACK))

#define HSI_CLK 16000000U
#define SYSTICK_TIM_CLK HSI_CLK

#define TICK_HZ 1000U

#define MAX_TASKS 5 // 4 task user  and 1 task idle

#define DUMMY_XPSR 0x01000000U

#define TASK_BLOCKED_STATE 0xFF
#define TASK_READY_STATE 0x00

#define INTERRUPT_DISABLE()               \
    do                                    \
    {                                     \
        __asm volatile("MOV R0 ,#0x01");  \
        __asm volatile("MSR PRIMASK,R0"); \
    } while (0)
#define INTERRUPT_ENABLE()                \
    do                                    \
    {                                     \
        __asm volatile("MOV R0 ,#0x00");  \
        __asm volatile("MSR PRIMASK,R0"); \
    } while (0)

#endif /* INC_SCHEDULER_DRIVER_H_ */
