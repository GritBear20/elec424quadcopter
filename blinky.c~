#include "stm32f10x.h"

#define STACK_TOP 0x20002000										/* This can move quite a lot ! */

volatile uint32_t msTicks;                       /* timeTicks counter */

void SysTick_Handler(void) {
  msTicks++;                                     /* increment timeTicks counter */
}

/*	Four vectors - the starting stack pointer value, code entry point and NMI and Hard-Fault handlers */
__INLINE static void Delay (uint32_t dlyTicks) {
  uint32_t curTicks = msTicks;

  while ((msTicks - curTicks) < dlyTicks);
}

int main(void){	
	 if (SysTick_Config (SystemCoreClock / 1000)) { /* Setup SysTick for 1 msec interrupts */
    ;                                            /* Handle Error */
    while (1);
  }
  

	RCC->APB2ENR |= 0x10 | 0x04 | 0x08;/* Enable the GPIOA (bit 2) and GPIOC (bit 8)  and GPIOB*/
	GPIOB->CRL = 0x00100000;	//set GIPOB-5 to out

   
		//						/* A short delay */
						/* Copy bit 0 of counter into GPIOC:Pin 5 */
 
 	while(1)
    {
		Delay (500);                                 /* delay  100 Msec    */
		GPIOB->BSRR = 1<<5 ;
		Delay (500);                                 /* delay  100 Msec    */
		GPIOB->BSRR = 1<<21;
    }
}
