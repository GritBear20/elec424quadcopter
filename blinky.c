#include "stm32f10x.h"


//define a counter for the delay 
volatile uint32_t msTicks;


void initHSI(){
	uint32_t HSIStatus;
	int StartUpCounter = 0;

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

	RCC->CR |= ((uint32_t)RCC_CR_HSION);
	 do {
	 HSIStatus = RCC->CR & RCC_CR_HSIRDY;
	 StartUpCounter++;
	 } while((HSIStatus == 0) && (StartUpCounter != 0x0500));
	 
	 if ((RCC->CR & RCC_CR_HSIRDY) != RESET) {
	 HSIStatus = (uint32_t)0x01;
	 } else {
	 HSIStatus = (uint32_t)0x00;
	 }

	if (HSIStatus == (uint32_t)0x01) {
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
	 
	 /*  PLL configuration: PLLCLK = HSI * 9 = 72 MHz */
	 
	 /* Set Bits to 0 */
	 RCC->CFGR &= (uint32_t)((uint32_t)~(
	  RCC_CFGR_PLLSRC   |
	  RCC_CFGR_PLLXTPRE |
	  RCC_CFGR_PLLMULL ));
	 /* Set Bits to 1 */
	 RCC->CFGR |= (uint32_t)(
	  RCC_CFGR_PLLSRC_HSI_Div2 |
	  RCC_CFGR_PLLMULL16 );
	 
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


//define the system init function
void init(){
	//pre-defined variables
	uint32_t HSEStatusCheck;
	int StartUpCounter = 0;

	//-----------------------------------------------------------------
	/* Reset the RCC clock configuration to the default reset state(for debug purpose) */
	/* Set HSEON(16) bit */
	RCC->CR |= (uint32_t)0x00010001;

	/* Reset SW(1:0)->01: HSE is selected as system clock (for now), 
	HPRE(7:4) AHB prescaler->0xxx: SYSCLK not divided to get HCLK
	PRE1(10:8) APB low-speed prescaler (APB1)->100: HCLK divided by 2
	PPRE2(13:11) APB high-speed prescaler (APB2)->0xx: HCLK not divided
	ADCPRE(15:14) ADC prescaler -> 00, devided by 2
	and MCO(26:24) Microcontroller clock output -> 100 (SYSCLK) */
	RCC->CFGR &= (uint32_t)0xF8FF0801;

	/* Reset HSION(0) -> 0, HSEON(16) -> 1, HSEBYP(18) -> 0, CSSON(19) -> 0 and PLLON(24) -> 0 bits */
	RCC->CR &= (uint32_t)0xFEF3FFFE; 
	 
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
	 
	 	/*  PLL configuration: PLLCLK = HSI/2 * 9 = 36 MHz */
	 
	 	/* Set Bits to 0 */
	 	/* Reset PLLSRC(16)->1, PLLXTPRE(17)->1, PLLMUL(21:18)-> 011 and USBPRE(22)->0 bits 
		PLLSRC(16)->1: HSE oscillator clock selected as PLL input clock (16MHz)
		PLLXTPRE(17)->1: divided HSE by 2 (16/2 = 8MHz), must be done before PLLON is set as 1
		PLLMUL(21:18)-> 0111: times HSE by 9 {0xFF (11 01->D) (11 11->F)  FFFF}
		USBPRE(22)->1: PLL clock is not divided
		*/
		RCC->CFGR &= (uint32_t)0xFFDFFFFF;
	 
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
	initHSI();

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
