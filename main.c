/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "lab3.h"

//define additional values
MotorSpeeds motorSpeeds;
#define MAXSPEED 50
const float scaling = (float)MAXSPEED/(float)0xff;
//Lab 03:
//user task priorities
#define mainDETECT_EMER_PRIORITY            ( 0 )
#define mainREFRESH_SENSOR_PRIORITY            ( 1 )
#define mainCALCULATE_PRIORITY            ( 2 )
#define mainUPDATE_PID_PRIORITY            ( 3 )
#define mainLOG_DEBUG_PRIORITY            ( 7 )
#define mainFLASH_GREEN_LED             ( 4 )
#define mainFLASH_RED_LED             ( 4 )

//user task wait time
#define WAIT_TIME_DETECT 10
#define WAIT_TIME_REFRESH 100
#define WAIT_TIME_MOTOR 1000
#define WAIT_TIME_RED_LED 2000
#define WAIT_TIME_GREEN_LED 1000

/* Task method defined */
static void prvDetectEmergencyTask( void *pvParameters );
static void prvRefreshSensorTask( void *pvParameters );
static void prvCalculateOrientationTask( void *pvParameters );
static void prvUpdatePidTask( void *pvParameters );
static void prvLogDebugTask( void *pvParameters );
static void prvRedLedTask( void *pvParameters);
static void prvGreenLedTask(void *pvParameters);

void initHSE();
void switchGreenLed(void);
void switchRedLed(void);

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

uint16_t Speed = 30;

uint16_t PrescalerValue = 0;

/* Timer & Motor functions-----------------------------------------------*/
void RCC_Configuration(void);//configure RCC
void GPIO_Configuration(void); //configure the io pins
void Config_Timer(void); // configure the timer
void SetMotor(int motorNum, uint16_t motorVal) ; //drive moter x by speed motorVal
void UpdateMotor(void); //update all the speeds for the 4 motors for part 3 
/*   ---------------------------------------------------------*/


/* A simple task that echoes all the characters that are received on COM0 
(USART1). */

int main(void)
{
	

  initHSE();
    //minimize collision possibility by adding an offset

  if (SysTick_Config (SystemCoreClock / 1000)) { /* Setup SysTick for 1 msec interrupts */
    ;                                            /* Handle Error */
    while (1);
  }
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
  /* Set the Vector Table base address at 0x08000000 */
  NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

    /* System Clocks Configuration */
  RCC_Configuration();

  /* GPIO Configuration */
  GPIO_Configuration();

  /*Configure Timer*/
  Config_Timer();

  //create tasks

    xTaskCreate( prvDetectEmergencyTask, ( signed char * ) "Detect", configMINIMAL_STACK_SIZE, NULL, mainDETECT_EMER_PRIORITY, NULL );
    xTaskCreate( prvRefreshSensorTask, ( signed char * ) "Refresh", configMINIMAL_STACK_SIZE, NULL, mainREFRESH_SENSOR_PRIORITY, NULL );
    xTaskCreate( prvCalculateOrientationTask, ( signed char * ) "Calculate", configMINIMAL_STACK_SIZE, NULL, mainCALCULATE_PRIORITY, NULL );
    xTaskCreate( prvUpdatePidTask, ( signed char * ) "Update", configMINIMAL_STACK_SIZE, NULL, mainUPDATE_PID_PRIORITY, NULL );
    xTaskCreate( prvLogDebugTask, ( signed char * ) "LogDebug", configMINIMAL_STACK_SIZE, NULL, mainLOG_DEBUG_PRIORITY, NULL );
    xTaskCreate( prvRedLedTask, ( signed char * ) "RedLed", configMINIMAL_STACK_SIZE, NULL, mainFLASH_RED_LED, NULL );
    xTaskCreate( prvGreenLedTask, ( signed char * ) "GreenLed", configMINIMAL_STACK_SIZE, NULL, mainFLASH_GREEN_LED, NULL );
 // start task scheduler 
 vTaskStartScheduler();

}



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
	 
	 	/* PCLK1 = HCLK / 2 */
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


void SetMotor(int motorNum, uint16_t motorVal){
	/*
	This function takes two inputs :
	a. motorNum, the number of the motor 
	b. motorVal, the speed (PWM duty cyle) of the motor 
	based on the two inputs the function set up the PWM signals on TIMER3 and TIMER4
	*/
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//use PWM mode
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //enable output state
  TIM_OCInitStructure.TIM_Pulse = motorVal;//set pulse - the duty cycle
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//set polarity to high
  if (motorNum==1){//initialize the PWM signal by choosing different Channels & pins
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

    Timer duty cycle = (TIM_CCR/ Timer Period)* 100%
  ----------------------------------------------------------------------- */
  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock / 36000000) - 1;//set Timer 3 =  36MHZ
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//set counter mode = up

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);//use the same setting for timer 3 and timer 4
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

void GPIO_Configuration(void)
{
	//configure GPIO
  GPIO_InitTypeDef GPIO_InitStructure;
  //enable PB0,1,8,9 (the motors)and PB4 (the red led)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//choose alternative function !(the timers )
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//set speed
  GPIO_Init(GPIOB, &GPIO_InitStructure);//initiazlied the pins to GPIOB

  RCC->APB2ENR|=0x08;//enable GPIOB
  GPIOB->CRL|=0x00110000;//set GIPOB-5 and 4 to output
}

//set motor speed according to MAXSPEED and motorSpeeds struct
void UpdateMotor(void){
	SetMotor(1,MAXSPEED*motorSpeeds.m1);
	SetMotor(2,MAXSPEED*motorSpeeds.m2);
	SetMotor(3,MAXSPEED*motorSpeeds.m3);
	SetMotor(4,MAXSPEED*motorSpeeds.m4);
}

int flag=0;
void switchGreenLed(void){
    if (flag){
    	//green led on
        GPIOB->BSRR&=~1<<21;
	GPIOB->BSRR|=1<<5;
   }
    else{
    	//green led off
        GPIOB->BSRR&=~1<<5;
	GPIOB->BSRR|=1<<21;
	}
   flag=~flag;//toggle the flag
	
}

int flag2=0;
void switchRedLed(void){
 
    if (flag2){
	//disable JTAG, PB4 becomes GPIO to turn red led on
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
        GPIOB->BSRR&=~1<<20;
	GPIOB->BSRR|=1<<4;
   }
    else{
	//enable JTAG (red led off)
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , DISABLE);
        GPIOB->BSRR&=~1<<4;
	GPIOB->BSRR|=1<<20;
	}
   flag2=~flag2;//toggle the flag
}


/*------------------Task Setup----------------------------------*/

//Detect Emergency run every 10 ms
static void prvDetectEmergencyTask( void *pvParameters )
{
    portTickType xLastExecutionTime;
	/* Just to remove the compiler warning about the unused parameter. */
	( void ) pvParameters;
    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
    {

        vTaskDelayUntil( &xLastExecutionTime, WAIT_TIME_DETECT );
        
        // run detectEmergency()
        detectEmergency();
        
        
    }
}

//define the semaphore
xSemaphoreHandle xSemaphore = NULL;

//Frefresh Sensor every 100 ms
static void prvRefreshSensorTask( void *pvParameters )
{
    portTickType xLastExecutionTime;
   
	/* Just to remove the compiler warning about the unused parameter. */
	( void ) pvParameters;

    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
    {
        
        vTaskDelayUntil( &xLastExecutionTime, WAIT_TIME_REFRESH );
        
        // Request data from the sensors.
        if( xSemaphore != NULL )//if xSemaphore is not null
        {   // check semaphore every 10 ticks, take it if it is available
            if( xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE )
            {
                refreshSensorData();
                xSemaphoreGive(xSemaphore); //release it 
            }
        }
        else{
            //if semaphore is null, create a new one.
            refreshSensorData();
            vSemaphoreCreateBinary( xSemaphore );
        }

        
        
    }

}
//caclulate orientation
static void prvCalculateOrientationTask( void *pvParameters )
{
    portTickType xLastExecutionTime;
   
	/* Just to remove the compiler warning about the unused parameter. */
	( void ) pvParameters;
    xLastExecutionTime = xTaskGetTickCount();
   for(;;){ 
if( xSemaphore != NULL )
    {
        // See if we can obtain the semaphore.  If the semaphore is not available
        // wait 10 ticks to see if it becomes free.
        if( xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE )
        {
            xSemaphore=NULL;//destroy the semaphore so the higher priority task will not be blocked. 
            calculateOrientation();
        }
    }else{
	 vTaskDelayUntil( &xLastExecutionTime, 200);//if semaphore null (not created) we will wait for 200 ms 
	}
    }
    
}

//updatePid once every second 
static void prvUpdatePidTask( void *pvParameters )
{
    portTickType xLastExecutionTime;
    
	/* Just to remove the compiler warning about the unused parameter. */
	( void ) pvParameters;
    
    for( ;; )
    {
        // C
        vTaskDelayUntil( &xLastExecutionTime, WAIT_TIME_MOTOR );
        updatePid(&motorSpeeds);
        UpdateMotor();
        
    }
}
//when available , debug task
static void prvLogDebugTask( void *pvParameters )
{
    portTickType xLastExecutionTime;
    
	/* Just to remove the compiler warning about the unused parameter. */
	( void ) pvParameters;
    
    for( ;; )
    {
        // C
        vTaskDelayUntil( &xLastExecutionTime, WAIT_TIME_DETECT );
        logDebugInfo();
        
    }
}



//toggle red led every 2 seconds
static void prvRedLedTask( void *pvParameters )
{
    portTickType xLastExecutionTime;
    
	/* Just to remove the compiler warning about the unused parameter. */
	( void ) pvParameters;
    
    for( ;; )
    {
        // C
        vTaskDelayUntil( &xLastExecutionTime, WAIT_TIME_RED_LED );
        switchRedLed();
        
    }
}
//toggle green led every 1 second 
static void prvGreenLedTask(void *pvParameters){
    portTickType xLastExecutionTime;
    
	/* Just to remove the compiler warning about the unused parameter. */
	( void ) pvParameters;
    
    for( ;; )
    {
        // C
        vTaskDelayUntil( &xLastExecutionTime, WAIT_TIME_GREEN_LED );
        switchGreenLed();
        
    }
}

