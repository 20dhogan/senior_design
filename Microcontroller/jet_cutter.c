#include "jet_cutter.h"

//Global Varialbes
extern QueueHandle_t xQueueState;
extern QueueHandle_t xQueueUART;
extern QueueHandle_t xQueueXStepPosition;
extern QueueHandle_t xQueueYStepPosition;
extern QueueHandle_t xQueueZStepPosition;
extern QueueHandle_t xQueueReset;
extern QueueHandle_t xQueue_Xsteps_to_move;
extern QueueHandle_t xQueue_Ysteps_to_move;
extern QueueHandle_t xQueue_Xsteps_counter;
extern QueueHandle_t xQueue_Ysteps_counter;
extern QueueHandle_t xQueue_number_of_ticks;
extern QueueHandle_t xQueue_reached_state;
extern QueueHandle_t xQueue_x_cw;
extern QueueHandle_t xQueue_y_cw;

void TIM4_Init(void){
	//Enable clock for TIM4
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
	
	//CONFIGURE CONTROL REGISTER 1
	TIM4->CR1 &= ~TIM_CR1_CMS; //SET EDGE ALIGNMENT
	TIM4->CR1 &= ~TIM_CR1_DIR; //UP COUNT
	
	//CONFIGURE CONTROL REGISTER 2
	TIM4->CR2 &= ~TIM_CR2_MMS;
	TIM4->CR2 |= TIM_CR2_MMS_2;
	
	//CONFIGURE DMA/INTERRUPT CONTROL REGISTER
	TIM4->DIER |= TIM_DIER_TIE;
	TIM4->DIER |= TIM_DIER_UIE; //DOUBLE CHECK
	
	//CONFIGURE PRESCALAR ARR
	TIM4->PSC = 0; // PRESCALAR
	//TIM4->ARR = 99; //ARR = (1,000,000/f_iqr) - 1
  //TIM4->CCR1 = 99 / 2;
	TIM4->ARR = 10000;
	TIM4->CCR1 = 10000/2;
	
	//CONFIGURE CHANNELS
	TIM4->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
	
	//CONFIGURE THE PIN AS OUTPUT
	TIM4->CCER |= TIM_CCER_CC1E;

	//ENABLE THE TIMER
	TIM4->CR1 |= TIM_CR1_CEN; // ENABLE CONTROL REGISTER 1
	
	//Enable interrupt
	NVIC_SetPriority(TIM4_IRQn, 9);
	NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQHandler(){

	if((TIM4->SR & TIM_SR_CC1IF) != 0){
		
		//Add function here
		main_timer_function();
		
		TIM4->SR &= ~TIM_SR_CC1IF; //clear flag
	}
	
	if((TIM4->SR & TIM_SR_UIF) != 0)
		TIM4->SR &= ~TIM_SR_UIF;
	return;
}


void BUTTON_LED_Init(void){
	//Enable GPIO for LED
	GPIOA->MODER &= 0xFFFFF3FF;
	GPIOA->MODER |= 0x00000400;
	
	//BUTTON
	//Enable GPIO for Button
	//GPIOC->MODER &= 0xF3FFFFFF;	
}

void motor_and_sensor_init(void){
	/*//set outputs and inputs
	GPIOC->MODER &= 0x00000000;
	GPIOC->MODER |= 0x00015555; //pins 0-8 as outputs, pins 9-15 as inputs
	
	GPIOC->PUPDR &= 0x00000000;
	GPIOC->PUPDR |= 0x55540000; //inputs set as pull up */
	
	// Configure GPIOC pins 0�8 as outputs, 9�15 as inputs
	GPIOC->MODER &= ~(0x0003FFFF);
	GPIOC->MODER |=  (0x00015555);

	// Enable pull-ups on GPIOC pins 9�15
	GPIOC->PUPDR &= ~(0xFFFC0000);
	GPIOC->PUPDR |=  (0x55540000);
  
	//Set pins for limit switches
	GPIOA->MODER &= 0xFFFFFC00; //Set PC 0-5 as input 00
	
	GPIOA->PUPDR &= 0xFFFFFC00;//Set PC 0-5 as no pull
}

void delay_init(void){
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void prvSetupHardware(){
	//startup the 16 MHz HSI
	RCC->CR |= RCC_CR_HSION;
  while((RCC->CR & RCC_CR_HSIRDY) == 0);
	
	//switch to the HSI
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
	SystemCoreClockUpdate();
	
	//Enalbe preipheral clocks
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	
	//Enable uart
	USART3_Init();
	
	//Enable timer
	TIM4_Init();
	
	motor_and_sensor_init();
	
	//Enable delay
	delay_init();
	
	BUTTON_LED_Init();
}