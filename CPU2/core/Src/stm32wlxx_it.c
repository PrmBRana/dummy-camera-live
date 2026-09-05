/**
 * stm32wlxx_it.c
 *
 * Minimal interrupt handler file for CPU2 (M0+).
 */

#include "stm32wlxx_hal.h"

volatile uint32_t g_system_tick_ms = 0;

void SysTick_Handler(void)
{
    g_system_tick_ms++;
    HAL_IncTick();
}

void SysTick_Init_CPU2(uint32_t sys_freq_hz)
{
    /* Configure SysTick to generate 1ms interrupt */
    SysTick->LOAD = (sys_freq_hz / 1000UL) - 1UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

__attribute__((weak)) void CPU2_Poll_IPC(void)
{
}

void CPU2_Delay_Ms(uint32_t ms)
{
    uint32_t start = g_system_tick_ms;
    while ((g_system_tick_ms - start) < ms)
    {
        CPU2_Poll_IPC();
        __NOP();
    }
}