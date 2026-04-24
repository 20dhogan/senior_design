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
extern Tool_t   *g_tools;
extern QueueHandle_t xQueue_pressure_washer_on; 

static void send_progress_update(uint16_t completed_tools, uint16_t total_tools){
	uint8_t frame[6];
	frame[0] = 0x55;
	frame[1] = 0x07;
	frame[2] = (uint8_t)(completed_tools & 0xFF);
	frame[3] = (uint8_t)((completed_tools >> 8) & 0xFF);
	frame[4] = (uint8_t)(total_tools & 0xFF);
	frame[5] = (uint8_t)((total_tools >> 8) & 0xFF);
	USART_Write(USART3, frame, 6);
}

void Motor_control(void *argument){
	uint8_t state = 0;
	int32_t x_steps_position = 0;
	int32_t y_steps_position = 0;
	int32_t next_x_point = 0;
	int32_t next_y_point = 0;
	uint32_t x_steps_to_move = 0;
	uint32_t y_steps_to_move = 0;
	uint32_t number_of_ticks = 0;
	uint32_t x_steps_counter = 0;
	uint32_t y_steps_counter = 0;
	bool x_cw = true;
	bool y_cw = true;
	bool reached_state = true;
	bool pressure_washer_on = false;
	
	while(1){	
		xQueuePeek(xQueueState, &state, 100);
		
		if(state == 2){
			
			bool abort_cutting = false;

			for(uint16_t tool_i = 0; tool_i < g_total_tools && !abort_cutting; tool_i++){
				uint16_t num_pts = g_tools[tool_i].count;

				for(uint16_t pt_j = 0; pt_j < num_pts && !abort_cutting; pt_j++){

					//Wait until previous segment is complete before planning next one
					while(reached_state == false){
						xQueuePeek(xQueue_reached_state, &reached_state, pdMS_TO_TICKS(1));
						xQueuePeek(xQueueState, &state, 0);
						if(state == 3){
							//Hold while paused; the timer ISR is not advancing in state 3.
							vTaskDelay(pdMS_TO_TICKS(1));
							continue;
						}
						if(state != 2){
							abort_cutting = true;
							break;
						}
					}
					if(abort_cutting){
						break;
					}

					// Arrived at last point of previous tool: pause then report progress
					if(pt_j == 0 && tool_i > 0){
						pressure_washer_on = false;
						xQueueOverwrite(xQueue_pressure_washer_on, &pressure_washer_on);
						vTaskDelay(pdMS_TO_TICKS(MS_DELAY_INBETWEEN_TOOLS));
						send_progress_update(tool_i, g_total_tools);
					}

					// Arrived at first point of current tool: pause before cutting
					if(pt_j == 1){
						pressure_washer_on = true;
						xQueueOverwrite(xQueue_pressure_washer_on, &pressure_washer_on);	
						vTaskDelay(pdMS_TO_TICKS(MS_DELAY_INBETWEEN_TOOLS));
					}
				
					//access tool point
					next_x_point = g_tools[tool_i].points[pt_j].x;
					next_y_point = g_tools[tool_i].points[pt_j].y;
						
					//calculate steps from current position
					if(xQueuePeek(xQueueXStepPosition, &x_steps_position, pdMS_TO_TICKS(100)) != pdPASS) continue;
					if(xQueuePeek(xQueueYStepPosition, &y_steps_position, pdMS_TO_TICKS(100)) != pdPASS) continue;

					if(next_x_point > x_steps_position){
						x_steps_to_move = next_x_point - x_steps_position;
						x_cw = false;
					} else {
						x_steps_to_move = x_steps_position - next_x_point;
						x_cw = true;
					}

					if(next_y_point > y_steps_position){
						y_steps_to_move = next_y_point - y_steps_position;
						y_cw = true;
					} else {
						y_steps_to_move = y_steps_position - next_y_point;
						y_cw = false;
					}

					number_of_ticks = (uint32_t)sqrt((double)x_steps_to_move * x_steps_to_move +
													(double)y_steps_to_move * y_steps_to_move);

					if(number_of_ticks == 0){
						continue;	
					}

					xQueueOverwrite(xQueue_Xsteps_to_move , &x_steps_to_move);
					xQueueOverwrite(xQueue_Ysteps_to_move , &y_steps_to_move);
					xQueueOverwrite(xQueue_number_of_ticks, &number_of_ticks);
					xQueueOverwrite(xQueue_x_cw, &x_cw);
					xQueueOverwrite(xQueue_y_cw, &y_cw);
					x_steps_counter = 0;
					y_steps_counter = 0;
					xQueueOverwrite(xQueue_Xsteps_counter, &x_steps_counter);
					xQueueOverwrite(xQueue_Ysteps_counter, &y_steps_counter);

					reached_state = false;
					xQueueOverwrite(xQueue_reached_state, &reached_state);

					xQueuePeek(xQueueState, &state, 0);
					if(state != 2) {
						abort_cutting = true;
					}
				}
			}

			//Wait for the last segment to finish
			while(reached_state == false){
				xQueuePeek(xQueue_reached_state, &reached_state, pdMS_TO_TICKS(1));
				xQueuePeek(xQueueState, &state, 0);
				if(state == 3){
					vTaskDelay(pdMS_TO_TICKS(1));
					continue;
				}
				if(state != 2) {
					break;
				}
			}
			
			//turn pressure washer off while the maching resets
			pressure_washer_on = false;
			xQueueOverwrite(xQueue_pressure_washer_on, &pressure_washer_on);

			//Pause, send final progress report, and signal complete
			xQueuePeek(xQueueState, &state, 0);
			if(state == 2){
				vTaskDelay(pdMS_TO_TICKS(MS_DELAY_INBETWEEN_TOOLS));
				send_progress_update(g_total_tools, g_total_tools);
				bool cutting_complete = true;
				xQueueOverwrite(xQueue_cutting_complete, &cutting_complete);
			}
			
		} else {
			vTaskDelay(pdMS_TO_TICKS(100));
		}	
	}
}