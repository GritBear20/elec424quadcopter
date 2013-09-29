#include "stm32f10x.h"


//define a counter for the delay 
volatile uint32_t msTicks;

//define the system init function
void init(){
	//to do: set RCC_CR  Clock control register

	//to do: set RCC_CFGR Clock configuration register

	//to do: set RCC_CIR Clock interrupt register

	//to do: APB2 peripheral reset register (RCC_APB2RSTR) (maybe)

	//to do: APB1 peripheral reset register (RCC_APB1RSTR) (maybe)

	//to do: AHB peripheral clock enable register (RCC_AHBENR) (maybe)

	//to do: APB2 peripheral clock enable register (RCC_APB2ENR) (maybe)

	//to do: APB1 peripheral clock enable register (RCC_APB1ENR)

	//to do: Backup domain control register (RCC_BDCR)

	//to do: Control/status register (RCC_CSR)

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
	init();

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
