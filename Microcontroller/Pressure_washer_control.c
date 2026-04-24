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
extern QueueHandle_t xQueue_cutting_complete;
extern uint16_t g_total_tools;
extern uint32_t *g_tool_end_indices;
extern QueueHandle_t xQueue_pressure_washer_on;
extern QueueHandle_t xQueueSpeed;

void Pressure_washer_control(void *argument){
	bool pressure_washer_on = false;
	uint16_t speed = SPEED_1;
	uint8_t state = 0;

	while(1){
		xQueuePeek(xQueue_pressure_washer_on, &pressure_washer_on, portMAX_DELAY);
		xQueuePeek(xQueueSpeed, &speed, portMAX_DELAY);
		xQueuePeek(xQueueState, &state, portMAX_DELAY);

		if((state == 2) || (state == 3)){ //we want to include state 3 here so that when unpasued the pressure washer can get a head start.
			if(pressure_washer_on == true){
				gpio_Write(GPIOA, 5, 1); //turn on pressure washer
				TIM4->ARR = speed;
				TIM4->CCR1 = speed/2;
			} else {
				gpio_Write(GPIOA, 5, 0); //turn off pressure washer
				TIM4->ARR = SPEED_10;
				TIM4->CCR1 = SPEED_10/2;
			}
		} else {
			gpio_Write(GPIOA, 5, 0); //turn off pressure washer
			TIM4->ARR = 7000;
			TIM4->CCR1 = 7000/2;
		}		
		vTaskDelay(1);
	}
}