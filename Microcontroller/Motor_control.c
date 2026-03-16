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
	
	while(1){		
		xQueuePeek(xQueueState, &state, 100);
		
		if((state == 2) || (state == 3)){
			
		size_t number_of_points = g_toolpath.count;
		uint16_t next_tool_to_report = 0;
		uint16_t current_tool_for_delay = 0;
		int32_t last_completed_point = -1;
			
			for(int loop_counter = 0; loop_counter < number_of_points; loop_counter++){ 

				//Wait until previous segment is complete before planning next one
				while(reached_state == false){
					xQueuePeek(xQueue_reached_state, &reached_state, pdMS_TO_TICKS(1));
					xQueuePeek(xQueueState, &state, 0);
					if((state != 2) && (state != 3)){
						break;
					}
				}
				if((state != 2) && (state != 3)){
					break;
				}

				if((loop_counter > 0) && (g_tool_end_indices != NULL)){
					last_completed_point = loop_counter - 1;
					uint32_t prev = (uint32_t)last_completed_point;

					// Pause after arriving at the first point of each tool
					uint32_t tool_first = (current_tool_for_delay == 0) ? 0 :
										 (g_tool_end_indices[current_tool_for_delay - 1] + 1);
					if(prev == tool_first){
						vTaskDelay(pdMS_TO_TICKS(MS_DELAY_INBETWEEN_TOOLS));
					}

					// Pause after the last point of each tool is complete
					if((current_tool_for_delay < g_total_tools) && (prev == g_tool_end_indices[current_tool_for_delay])){
						vTaskDelay(pdMS_TO_TICKS(MS_DELAY_INBETWEEN_TOOLS));
						current_tool_for_delay++;
					}

					while((next_tool_to_report < g_total_tools) &&
							((uint32_t)last_completed_point >= g_tool_end_indices[next_tool_to_report])){
						next_tool_to_report++;
						send_progress_update(next_tool_to_report, g_total_tools);
					}
				}
			
				//access tool points
				next_x_point = g_toolpath.points[loop_counter].x;
				next_y_point = g_toolpath.points[loop_counter].y;
					
				//calculate how many x and y steps need to be taken based on previous points
				if (xQueuePeek(xQueueXStepPosition, &x_steps_position, pdMS_TO_TICKS(100)) != pdPASS) {
					continue;
				}
				if (xQueuePeek(xQueueYStepPosition, &y_steps_position, pdMS_TO_TICKS(100)) != pdPASS) {
					continue;
				}
				//X steps
				if(next_x_point > x_steps_position){
					//this means the x motor needs to turn CCW
					//This means we need to go forward in the x direction
					x_steps_to_move = next_x_point - x_steps_position;
					x_cw = false;
					
				} else {
					//This means we need to go backwards in the x direction
					x_steps_to_move = x_steps_position - next_x_point;
					x_cw = true;
				}
				
				//Y steps
				if(next_y_point > y_steps_position){
					//this means we need to go CW
					//This means we need to go forward in the y direction
					y_steps_to_move = next_y_point - y_steps_position;
					y_cw = true;
					
				} else {
					//This means we need to go backwards in the x direction
					y_steps_to_move = y_steps_position - next_y_point;
					y_cw = false;
				}
				
				//determine the number of interrupts the next move will be spread over		
				number_of_ticks = (uint32_t)sqrt((double)x_steps_to_move * x_steps_to_move + 
				(double)y_steps_to_move * y_steps_to_move); //THIS WILL BE FLOORED
				
				//If the number of ticks are zero then we are already at our target
				//skip to the next point
				if(number_of_ticks == 0){
					continue;  // skip to next point in the loop
				}
						
				//Now that we have reached the state we can update all information in queues
				xQueueOverwrite(xQueue_Xsteps_to_move , &x_steps_to_move);
				xQueueOverwrite(xQueue_Ysteps_to_move , &y_steps_to_move);
				xQueueOverwrite(xQueue_number_of_ticks, &number_of_ticks);
				xQueueOverwrite(xQueue_x_cw, &x_cw);
				xQueueOverwrite(xQueue_y_cw, &y_cw);
				x_steps_counter = 0;
				y_steps_counter = 0;
				xQueueOverwrite(xQueue_Xsteps_counter, &x_steps_counter);
				xQueueOverwrite(xQueue_Ysteps_counter, &y_steps_counter);
				
				
				//update state to not reached
				reached_state = false;
				xQueueOverwrite(xQueue_reached_state, &reached_state);
				
				//if state != 2 or 3 exit loop
				xQueuePeek(xQueueState, &state, 100);
				if((state != 2) && (state != 3)){
					break;
				}
			}

			//Wait for the last segment to finish
			while(reached_state == false){
				xQueuePeek(xQueue_reached_state, &reached_state, pdMS_TO_TICKS(1));
				xQueuePeek(xQueueState, &state, 0);
				if((state != 2) && (state != 3)){
					break;
				}
			}

			if((state == 2) || (state == 3)){
				last_completed_point = (int32_t)number_of_points - 1;
				if((last_completed_point >= 0) && (g_tool_end_indices != NULL)){
					uint32_t last_pt = (uint32_t)last_completed_point;

					// Pause after arriving at the first point of the last tool (handles 1-point last tool)
					uint32_t tool_first = (current_tool_for_delay == 0) ? 0 :
										 (g_tool_end_indices[current_tool_for_delay - 1] + 1);
					if(last_pt == tool_first){
						vTaskDelay(pdMS_TO_TICKS(MS_DELAY_INBETWEEN_TOOLS));
					}

					// Pause after the last point of the last tool
					if((current_tool_for_delay < g_total_tools) && (last_pt == g_tool_end_indices[current_tool_for_delay])){
						vTaskDelay(pdMS_TO_TICKS(MS_DELAY_INBETWEEN_TOOLS));
					}

					while((next_tool_to_report < g_total_tools) &&
							((uint32_t)last_completed_point >= g_tool_end_indices[next_tool_to_report])){
						next_tool_to_report++;
						send_progress_update(next_tool_to_report, g_total_tools);
					}
				}
			}
			
			//Signal cutting complete if all points were processed
			xQueuePeek(xQueueState, &state, 0);
			if((state == 2) || (state == 3)){
				bool cutting_complete = true;
				xQueueOverwrite(xQueue_cutting_complete, &cutting_complete);
			}
		}
	}	
}