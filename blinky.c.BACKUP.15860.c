

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
volatile uint32_t msTicks;                       /* timeTicks counter */

<<<<<<< HEAD
//define the system init function
//use HSE to drive PLL as system clock frequency
//frequency is 72MHz
void initHSE(){
	//pre-defined variables
	uint32_t HSEStatusCheck;
	int StartUpCounter = 0;

	//-----------------------------------------------------------------
	/* Reset the RCC clock configuration to the default reset state(for debug purpose) */
	/* Set HSION bit */
	RCC->CR |= (uint32_t)0x00000001;
 
	/* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
	RCC->CFGR &= (uint32_t)0xF8FF0000;
 
	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFFF;
 
	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;
	 
	/* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
	RCC->CFGR &= (uint32_t)0xFF80FFFF;
	 
	/* Disable all interrupts and clear pending bits  */
	RCC->CIR = 0x009F0000;	
	
	//-----------------------------------------------------------------------------------------------
	//enable HSE:
	//wait until the HSEON is ready
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);
	do {
		 HSEStatusCheck = RCC->CR & RCC_CR_HSERDY;
		 StartUpCounter++;
	 } while((HSEStatusCheck == 0) && (StartUpCounter != 0x1000));
	
	if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
 		HSEStatusCheck = (uint32_t)0x01;
 	} else {
 		HSEStatusCheck = (uint32_t)0x00;
 	}

	//----------------------------------------------------------------------------------------------
	//enable PLL and set it as the system clock source
	if (HSEStatusCheck == (uint32_t)0x01) {
		/* Enable Prefetch Buffer */
	 	FLASH->ACR |= FLASH_ACR_PRFTBE;
	 
	 	/* Flash 2 wait state */
	 	FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
	 	FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
	 
	 	/* HCLK = SYSCLK */
	 	RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
	 
	 	/* PCLK2 = HCLK */
	 	RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
	 
	 	/* PCLK1 = HCLK */
	 	RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;
	 
	 	/*  PLL configuration: PLLCLK = HSE/2 * 9 = 72 MHz */
	 	/* Set Bits to 0 */
		 RCC->CFGR &= (uint32_t)((uint32_t)~(
		  RCC_CFGR_PLLMULL ));
		 /* Set Bits to 1 */
		 RCC->CFGR |= (uint32_t)(
	  	  RCC_CFGR_PLLXTPRE |  //1 means divide HSE by 2
		  RCC_CFGR_PLLSRC   |
		  RCC_CFGR_PLLMULL9 );
	 
	 	/* Enable PLL */
	 	RCC->CR |= RCC_CR_PLLON;
	 
	 	/* Wait till PLL is ready */
	 	while((RCC->CR & RCC_CR_PLLRDY) == 0)
	 	{
	 	}
	 
	 	/* Select PLL as system clock source */
	 	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	 	RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
	 
	 	/* Wait till PLL is used as system clock source */
	 	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
	 	{
	 	}
	}
}

//increment the counter
=======
>>>>>>> miralab02
void SysTick_Handler(void) {
  msTicks++;                                     /* increment timeTicks counter */
}

__INLINE static void Delay (uint32_t dlyTicks) {
  uint32_t curTicks = msTicks;

  while ((msTicks - curTicks) < dlyTicks);
}

<<<<<<< HEAD
int main(void){	
	//using HSE to drive PLL
	//PLL is system clock and it is set at 72MHz
	initHSE(); 
		
	if (SysTick_Config (SystemCoreClock / 1000)) {
        //Setup the system interrupt to once 1ms
        ;                                          
		while (1);
	}
  
    //GPIOB-5 is the LED!
	RCC->APB2ENR |=0x08;// Enable  GPIOB
	GPIOB->CRL = 0x00100000;	//set GIPOB-5 to output
=======

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
>>>>>>> miralab02

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
