#include "stm32f10x.h"


//define a counter for the delay 
volatile uint32_t msTicks;

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
void SysTick_Handler(void) {
  msTicks++;
}

//Define the delay function :
__INLINE static void Delay (uint32_t dlyTicks) {
  uint32_t curTicks = msTicks;
  //stop when the counter counts is bigger than the set up value for delay
  //otherwise keep looping int the delay
  while ((msTicks - curTicks) < dlyTicks);
}

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

   

 	while(1)
    {
		Delay (500);                                //delay 0.5 sec
		GPIOB->BSRR = 1<<5 ;//set the 5th bit to one so pin 5 is on
		Delay (500);                                //delay 0.5 sec
		GPIOB->BSRR = 1<<21;//set the 21th bit to one so pin5 is off
    }
}
