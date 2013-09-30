#include "stm32f10x.h"
#include "lab2.h"

//define a counter for the delay 
volatile uint32_t msTicks;

//define additional values
MotorSpeeds motorSpeedsPtr;
uint32_t cnt10ms;
uint32_t cnt100ms;
uint32_t cnt1000ms1;
uint32_t cnt1000ms2;
uint32_t cnt1000ms3;
uint32_t cnt2000ms;

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

void switchGreenLed(void){}
void switchRedLed(void){}

//manual scheduler
//it is called every 10ms
void manualSchedule(void){
	//check tasks
	//1st, every 10second run the detectEmergency
	if(cnt10ms - msTicks >= 10){
		cnt10ms = msTicks;
		//since it has the highest priority, it gets executed immediately
		detectEmergency(); 
	}
	
	//2nd, every 100second run the refreshSensorData
	else if(cnt100ms - msTicks > 100){
		cnt100ms = msTicks;
		//this gets executed secondarily
		refreshSensorData();
	}

	//3rd, calculateOrientation at once every second
	else if(cnt1000ms1 - msTicks > 1000){
		//need to further make sure that it doesn't collide with the other two tasks
		if(cnt100ms - msTicks < 96 && cnt10ms - msTicks < 6){
			cnt1000ms1 = msTicks;
			calculateOrientation();
		}
	}
	//4th, updatePid at onces every second
	else if(cnt1000ms2 - msTicks > 1000){
		cnt1000ms2 = msTicks;
		updatePid(&motorSpeedsPtr);
	}
	//5th, blink Green LED at 0.5Hz, (on for 1 second, off for 1 sec)
	else if(cnt1000ms3 - msTicks > 1000){
		cnt1000ms3 = msTicks;
		switchGreenLed();
	}
	//6th, blink Red LED at 0.25Hz, (on for 2 second, off for 2 sec)
	else if(cnt2000ms - msTicks > 2000){
		cnt2000ms = msTicks;
		switchRedLed();
	}else{
		logDebugInfo();
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
		//minimize 
		cnt10ms = msTicks;
		cnt100ms = msTicks + 1;
		cnt1000ms1 = msTicks + 2;
		cnt1000ms2 = msTicks + 3;
		cnt1000ms3 = msTicks + 4;
		cnt2000ms = msTicks + 5;
		manualSchedule();
    }
}
