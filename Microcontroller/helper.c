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

//Helper Functions**********************************************************
bool gpio_Read(GPIO_TypeDef *port, uint8_t pin){
	uint16_t x = 0;
	x |= port->IDR;
	x &= (1 << pin);
	if(x == (1 << pin)){
		return true;
	}
	else{
		return false;
	}
}

void gpio_Write(GPIO_TypeDef *port, uint8_t pin, bool val){
	if(val == true){
		port->ODR |= (1 << pin); //write a 1
	} else {
		port->ODR &= ~(1 << pin); //write a 0
	} 
}

void motor_pulse(int axis, bool cw){
	switch (axis){
		case 0:
		 	//x axis
			if(!(gpio_Read(GPIOC, 9))){
				if(cw){
					gpio_Write(GPIOC, 2, 0); //set motor in cw mode
				}else{
					gpio_Write(GPIOC, 2, 1); //set motor in ccw mode
				}
				//pulse
				delay_us(6);
				gpio_Write(GPIOC, 1, 0); //pulse to 0v
				delay_us(6);
				gpio_Write(GPIOC, 1, 1); //bring back up to 5v
			}
		break;
		
		case 1:
			//y axis
			if(!(gpio_Read(GPIOC, 10))){
				if(cw){
					gpio_Write(GPIOC, 5, 0); //set motor in cw mode
				}else{
					gpio_Write(GPIOC, 5, 1); //set motor in ccw mode
				}
				//pulse
				delay_us(6);
				gpio_Write(GPIOC, 4, 0); //pulse to 0v
				delay_us(6);
				gpio_Write(GPIOC, 4, 1); //bring back up to 5v
			}
		break;
		
		case 2:
			//z axis
				if(cw){
					gpio_Write(GPIOC, 8, 0); //set motor in cw mode
				}else{
					gpio_Write(GPIOC, 8, 1); //set motor in ccw mode
				}
				//pulse
				delay_us(6);
				gpio_Write(GPIOC, 7, 1); //pulse to 0v
				delay_us(6);
				gpio_Write(GPIOC, 7, 0); //bring back up to 5v		
		break;
	}
}

static inline void delay_us(uint32_t us){
    uint32_t cycles = us * (SystemCoreClock / 1000000U);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

void main_timer_function(){
	uint8_t state = 0;
	bool reset = false;
	bool reached_state = true;
	bool x_cw = true;
	bool y_cw = true;
	uint32_t number_of_ticks = 0;
	uint32_t x_steps_to_move = 0;
	uint32_t y_steps_to_move = 0;
	uint32_t x_steps_counter = 0;
	uint32_t y_steps_counter = 0;
	int32_t x_steps_position = 0;
	int32_t y_steps_position = 0;
	static uint32_t ticks_remaining = 0;
	
	//Peek from queues
		xQueuePeekFromISR(xQueueState, &state);
		//Put body of interrupt here
		switch (state){
			case 0:
				//reset machine
				xQueuePeekFromISR(xQueueReset, &reset);
				if(!reset){
					if((gpio_Read(GPIOA, 0)) || (gpio_Read(GPIOA, 1)) || (gpio_Read(GPIOA, 4))){
						if(gpio_Read(GPIOA, 0)){ //x axis limit has not been hit
							motor_pulse(0, true);
						}
						if(gpio_Read(GPIOA, 1)){ //y axis limit has not been hit
							motor_pulse(1, false);
						}
						if(gpio_Read(GPIOA, 4)){ //z axis limit has not been hit
							motor_pulse(2, false);
						}
					}else{
						//machine is in the the home position
						reset = true;
						int32_t x_position = 0;
						int32_t y_position = 0;
						int32_t z_position = 3000;
						xQueueOverwriteFromISR(xQueueReset, &reset, pdFALSE);
						xQueueOverwriteFromISR(xQueueXStepPosition, &x_position, pdFALSE);
						xQueueOverwriteFromISR(xQueueYStepPosition, &y_position, pdFALSE);
						xQueueOverwriteFromISR(xQueueZStepPosition, &z_position, pdFALSE);
					}
				}
				
			break;
			
			case 2:
				//peak from queues
				xQueuePeekFromISR(xQueue_reached_state, &reached_state);
			  xQueuePeekFromISR(xQueue_number_of_ticks, &number_of_ticks);
			
				if((reached_state == false) && (number_of_ticks > 0)){
					
					xQueuePeekFromISR(xQueue_Xsteps_to_move, &x_steps_to_move);
					xQueuePeekFromISR(xQueue_Ysteps_to_move, &y_steps_to_move);
					xQueuePeekFromISR(xQueue_Xsteps_counter, &x_steps_counter);
					xQueuePeekFromISR(xQueue_Ysteps_counter, &y_steps_counter);
					xQueuePeekFromISR(xQueueXStepPosition, &x_steps_position);
					xQueuePeekFromISR(xQueueYStepPosition, &y_steps_position);
					xQueuePeekFromISR(xQueue_x_cw, &x_cw);
					xQueuePeekFromISR(xQueue_y_cw, &y_cw);
					
					//if it is a new round we will update the remaining ticks
					if(ticks_remaining == 0){
						ticks_remaining = number_of_ticks;
					}
					
					//Here we move to next point
				
					x_steps_counter += x_steps_to_move;
					if (x_steps_counter >= number_of_ticks) {
						
						//send a pulse
						motor_pulse(0, x_cw);
						
						if(x_cw == true)
						{
							x_steps_position--;
						}
						else
						{
							x_steps_position++;
						}
						
						x_steps_counter -= number_of_ticks;
					}

					y_steps_counter += y_steps_to_move;
					if (y_steps_counter >= number_of_ticks) {
						
						//send a pulse
						motor_pulse(1, y_cw);
						
						if(y_cw == true)
						{
							y_steps_position++;
						}
						else
						{
							y_steps_position--;
						}
						
						y_steps_counter -= number_of_ticks;
					}
					
					ticks_remaining--;
					
					if(ticks_remaining == 0){
						reached_state = true;
					}
					
					//update queues
					xQueueOverwriteFromISR(xQueue_Xsteps_counter, &x_steps_counter, pdFALSE);
					xQueueOverwriteFromISR(xQueue_Ysteps_counter, &y_steps_counter, pdFALSE);
					xQueueOverwriteFromISR(xQueue_reached_state, &reached_state, pdFALSE);
					xQueueOverwriteFromISR(xQueueXStepPosition, &x_steps_position, pdFALSE);
					xQueueOverwriteFromISR(xQueueYStepPosition, &y_steps_position, pdFALSE);
				}	
					
			break;
			
		}
}
