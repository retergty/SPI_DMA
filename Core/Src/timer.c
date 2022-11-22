#include "timer.h"
#include "main.h"
void Timer3Init(uint16_t psc, uint16_t arr)
{
  //240MHZ定时器时钟输入
  __HAL_RCC_TIM3_CLK_ENABLE();
  TIM_Base_InitTypeDef TIM_Base;
  TIM_Base.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  TIM_Base.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_Base.CounterMode = TIM_COUNTERMODE_UP;
  TIM_Base.Period = arr;
  TIM_Base.Prescaler = psc;
  TIM_Base.RepetitionCounter = 0;
  TIM_Base_SetConfig(TIM3, &TIM_Base);
  TIM3->CR1 |= TIM_CR1_CEN;
}