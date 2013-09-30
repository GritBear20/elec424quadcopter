

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
volatile uint32_t msTicks;                       /* timeTicks counter */

void SysTick_Handler(void) {
  msTicks++;                                     /* increment timeTicks counter */
}

__INLINE static void Delay (uint32_t dlyTicks) {
  uint32_t curTicks = msTicks;

  while ((msTicks - curTicks) < dlyTicks);
}


TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

uint16_t Speed = 30;

uint16_t PrescalerValue = 0;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void Config_Timer(void);
void SetMotor(int motorNum, uint16_t motorVal);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
   if (SysTick_Config (SystemCoreClock / 1000)) { /* Setup SysTick for 1 msec interrupts */
    ;                                            /* Handle Error */
    while (1);
  }
  SystemInit();
  /* System Clocks Configuration */
  RCC_Configuration();

  /* GPIO Configuration */
  GPIO_Configuration();

  /*Configure Timer*/
  Config_Timer();

 
 

  while (1)
  {	SetMotor(4,0);
	SetMotor(1,Speed);
	Delay(2000);
	//GPIOB->BSRR=1<<5;
 	SetMotor(1,0);
	SetMotor(2,Speed);
	Delay(2000);
	SetMotor(2,0);
	SetMotor(3,Speed);
	Delay(2000);
	//GPIOB->BSRR=1<<21;
	SetMotor(3,0);
	SetMotor(4,Speed);
	Delay(2000);
	}
}

void SetMotor(int motorNum, uint16_t motorVal){
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = motorVal;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  if (motorNum==1){
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

 	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  }
  else if (motorNum==2){
        TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  }
  else if (motorNum==3){
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  }
  else if (motorNum==4){
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  }

}

void Config_Timer(void){
	 /* -----------------------------------------------------------------------
    Timer Configuration: 

    Timer duty cycle = (TIM_CCR/ TIM_ARR)* 100
  ----------------------------------------------------------------------- */
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_ARRPreloadConfig(TIM4, ENABLE);
   /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  TIM_CtrlPWMOutputs(TIM3,ENABLE);
  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
  TIM_CtrlPWMOutputs(TIM4,ENABLE);
}



void RCC_Configuration(void)
{
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  /* GPIOA and GPIOB clock e
nable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

}

/**
  * @brief  Configure the TIM3 Ouput Channels.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

  //RCC->APB2ENR|=0x08;
  //GPIOB->CRL=0x00100000;

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
