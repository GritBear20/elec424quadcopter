#include "stm32f10x.h"

volatile uint32_t msTicks;                       /* timeTicks counter */

GPIO_InitTypeDef theLedType = {GPIO_Pin_5, GPIO_Speed_10MHz, GPIO_Mode_Out_PP};

void SysTick_Handler(void) {
  msTicks++;                                     /* increment timeTicks counter */
}

__INLINE static void Delay (uint32_t dlyTicks) {
  uint32_t curTicks = msTicks;

  while ((msTicks - curTicks) < dlyTicks);
}

__INLINE static void LED_Config(void) {
	GPIO_Init(GPIOB, &theLedType);		/* Configure the LEDs */
}

__INLINE static void LED_On (uint32_t led) {
	GPIO_SetBits(GPIOB, GPIO_Pin_5);	/* Turn On  LED */
}

__INLINE static void LED_Off (uint32_t led) {
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);	/* Turn Off LED */
}

int main (void) {
  if (SysTick_Config (SystemCoreClock / 1000)) { /* Setup SysTick for 1 msec interrupts */
    ;                                            /* Handle Error */
    //while (1);
  }
  
  LED_Config();                                  /* configure the LEDs */                            
 
  while(1){
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
	Delay (1000);
  }

  while(1) {
    LED_On (0x100);                              /* Turn  on the LED   */
    Delay (1000);                                 /* delay  100 Msec    */
    LED_Off (0x100);                             /* Turn off the LED   */
    Delay (1000);                                 /* delay  100 Msec    */
  }
}
