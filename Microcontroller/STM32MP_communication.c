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
Toolpath_t g_toolpath = {0};
uint16_t g_total_tools = 0;
uint32_t *g_tool_end_indices = NULL;
					
					
void STM32MP_communication(void *argument){
	
	
	uint8_t state = 0;
	uint8_t msg = 0x00;
	const uint8_t state_command = 0x55;
	const uint8_t done_command = 0x54;
	const uint8_t state0 = 0x00;
	const uint8_t state1 = 0x01;
	const uint8_t state2 = 0x02;
	const uint8_t state3 = 0x03;
	const uint8_t state4 = 0x04;
	const uint8_t reseting = 0x05;
	bool reset = false;
	xQueueOverwrite(xQueueState, &state);
	uint8_t msg2x[2] = {0, 0};

	
	while(1){
		switch (state){
			
			case 0: //idle state
				while(xQueueReceive(xQueueUART, &msg2x[0], portMAX_DELAY) != pdPASS); //first message, expected 0x55
				if(msg2x[0] == 0x55){
					while(xQueueReceive(xQueueUART, &msg2x[1], portMAX_DELAY) != pdPASS); //second message, expected 0x01
					if(msg2x[1] == 0x01){
						//Then linux board is requesting to move to state 1
						//Lets see if the machine has reset yet.
						xQueuePeek(xQueueReset, &reset, 100);
						
						if(reset == true){						
							//Set the state to 1
							state = 1;
							xQueueOverwrite(xQueueState, &state);
							
							//Respond to linux board that the state on the microcontroller is now 1.
							USART_Write(USART3, &state_command, 1); //informing linux board we are making a state update
							USART_Write(USART3, &state1, 1); //state update is 1
						} 
						else {
							USART_Write(USART3, &state_command, 1);
							USART_Write(USART3, &reseting, 1);
						}
					}
				}
			break;
			
			case 1: // load data state
				{
					uint8_t rx;
					uint8_t lsb, msb;
				uint16_t tool_point_count = 0;
				uint16_t total_points = 0;
				uint16_t tool_count = 0;

				/* Free any previous allocation */
				if (g_toolpath.points != NULL) {
					vPortFree(g_toolpath.points);
					g_toolpath.points = NULL;
					g_toolpath.count = 0;
				}
				if (g_tool_end_indices != NULL) {
					vPortFree(g_tool_end_indices);
					g_tool_end_indices = NULL;
				}
				g_total_tools = 0;

				/* Wait for START_TRANSFER = 0xAA */
				do {
					xQueueReceive(xQueueUART, &rx, portMAX_DELAY);
				} while (rx != 0xAA);

				/* Loop over tools until END_TRANSFER = 0xAF */
				while (1) {
					/* Read next marker: expect TOOL_BEGIN (0xAB) or END_TRANSFER (0xAF) */
					xQueueReceive(xQueueUART, &rx, portMAX_DELAY);

					if (rx == 0xAF) {
						break; /* all tools received */
					}

					if (rx != 0xAB) {
						continue; /* unexpected byte, skip */
					}

					/* Read point count for this tool (uint16_t, little-endian) */
					xQueueReceive(xQueueUART, &lsb, portMAX_DELAY);
					xQueueReceive(xQueueUART, &msb, portMAX_DELAY);
					tool_point_count = (uint16_t)lsb | ((uint16_t)msb << 8);

					/* Grow the points array */
					uint16_t new_total = total_points + tool_point_count;
					Point_t *new_buf = pvPortMalloc(new_total * sizeof(Point_t));
					uint16_t new_tool_count = tool_count + 1;
					uint32_t *new_tool_end_indices = pvPortMalloc(new_tool_count * sizeof(uint32_t));

					if ((new_buf == NULL) || (new_tool_end_indices == NULL)) {
						/* Allocation failed - free old and reset to idle */
						if (g_toolpath.points != NULL) {
							vPortFree(g_toolpath.points);
						}
						if (g_tool_end_indices != NULL) {
							vPortFree(g_tool_end_indices);
						}
						if (new_buf != NULL) {
							vPortFree(new_buf);
						}
						if (new_tool_end_indices != NULL) {
							vPortFree(new_tool_end_indices);
						}
						g_toolpath.points = NULL;
						g_toolpath.count = 0;
						g_tool_end_indices = NULL;
						g_total_tools = 0;
						state = 0;
						xQueueOverwrite(xQueueState, &state);
						break;
					}

					/* Copy existing points to new buffer */
					if (total_points > 0 && g_toolpath.points != NULL) {
						memcpy(new_buf, g_toolpath.points, total_points * sizeof(Point_t));
						vPortFree(g_toolpath.points);
					}
					if (tool_count > 0 && g_tool_end_indices != NULL) {
						memcpy(new_tool_end_indices, g_tool_end_indices, tool_count * sizeof(uint32_t));
						vPortFree(g_tool_end_indices);
					}

					g_toolpath.points = new_buf;
					g_tool_end_indices = new_tool_end_indices;

					/* Receive points for this tool */
					for (uint16_t i = 0; i < tool_point_count; i++) {

						uint8_t b[4];

						/* X */
						xQueueReceive(xQueueUART, &b[0], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[1], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[2], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[3], portMAX_DELAY);
						g_toolpath.points[total_points + i].x =
							(int32_t)b[0] |
							((int32_t)b[1] << 8) |
							((int32_t)b[2] << 16) |
							((int32_t)b[3] << 24);

						/* Y */
						xQueueReceive(xQueueUART, &b[0], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[1], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[2], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[3], portMAX_DELAY);
						g_toolpath.points[total_points + i].y =
							(int32_t)b[0] |
							((int32_t)b[1] << 8) |
							((int32_t)b[2] << 16) |
							((int32_t)b[3] << 24);
					}

					total_points = new_total;
					g_tool_end_indices[tool_count] = (total_points > 0) ? (total_points - 1U) : 0U;
					tool_count = new_tool_count;

					/* Wait for TOOL_END = 0xAC */
					do {
						xQueueReceive(xQueueUART, &rx, portMAX_DELAY);
					} while (rx != 0xAC);
				}

				g_toolpath.count = total_points;
				g_total_tools = tool_count;

				if (state == 0) break; /* allocation failed above */
				
				//acknowledge all data has been received
				USART_Write(USART3, &done_command, 1);
				
				//wait for state change command
				while(xQueueReceive(xQueueUART, &msg2x[0], portMAX_DELAY) != pdPASS);
				if(msg2x[0] == 0x55){
					while(xQueueReceive(xQueueUART, &msg2x[1], portMAX_DELAY) != pdPASS);
					if(msg2x[1] == 0x02){
						//Set the state to 2
						state = 2;
						xQueueOverwrite(xQueueState, &state);
						
						//Initialize cutting_complete to false
						bool cutting_complete = false;
						xQueueOverwrite(xQueue_cutting_complete, &cutting_complete);
						
						//Respond to linux board
						USART_Write(USART3, &state_command, 1);
						USART_Write(USART3, &state2, 1);
					}
				}
			}
			break;

			case 2: //cutting state
			{
				bool cutting_complete = false;
				
				//Poll for UART commands and cutting completion
				while (state == 2) {
					//Check if cutting is complete
					xQueuePeek(xQueue_cutting_complete, &cutting_complete, 0);
					if (cutting_complete) {
						//Cutting finished - notify Linux board and go to state 4
						state = 4;
						xQueueOverwrite(xQueueState, &state);
						USART_Write(USART3, &state_command, 1);
						USART_Write(USART3, &state4, 1);
						break;
					}
					
					//Check for UART commands (with short timeout so we keep polling)
					if (xQueueReceive(xQueueUART, &msg2x[0], pdMS_TO_TICKS(50)) == pdPASS) {
						if (msg2x[0] == 0x55) {
							if (xQueueReceive(xQueueUART, &msg2x[1], pdMS_TO_TICKS(100)) == pdPASS) {
								if (msg2x[1] == 0x03) {
									//Pause request
									state = 3;
									xQueueOverwrite(xQueueState, &state);
									USART_Write(USART3, &state_command, 1);
									USART_Write(USART3, &state3, 1);
								} else if (msg2x[1] == 0x04) {
									//Stop request
									state = 4;
									xQueueOverwrite(xQueueState, &state);
									USART_Write(USART3, &state_command, 1);
									USART_Write(USART3, &state4, 1);
								}
							}
						}
					}
				}
			}
			break;
			
			case 3: //pause state
			{
				//Wait for UART commands (resume or stop)
				while (state == 3) {
					if (xQueueReceive(xQueueUART, &msg2x[0], portMAX_DELAY) == pdPASS) {
						if (msg2x[0] == 0x55) {
							if (xQueueReceive(xQueueUART, &msg2x[1], portMAX_DELAY) == pdPASS) {
								if (msg2x[1] == 0x02) {
									//Resume request
									state = 2;
									xQueueOverwrite(xQueueState, &state);
									USART_Write(USART3, &state_command, 1);
									USART_Write(USART3, &state2, 1);
								} else if (msg2x[1] == 0x04) {
									//Stop request
									state = 4;
									xQueueOverwrite(xQueueState, &state);
									USART_Write(USART3, &state_command, 1);
									USART_Write(USART3, &state4, 1);
								}
							}
						}
					}
				}
			}
			break;
			
			case 4: //free memory state
			{
				//Free toolpath memory
				if (g_toolpath.points != NULL) {
					vPortFree(g_toolpath.points);
					g_toolpath.points = NULL;
				}
				if (g_tool_end_indices != NULL) {
					vPortFree(g_tool_end_indices);
					g_tool_end_indices = NULL;
				}
				g_toolpath.count = 0;
				g_total_tools = 0;
				
				//Reset cutting complete flag
				bool complete = false;
				xQueueOverwrite(xQueue_cutting_complete, &complete);
				
				//Reset the machine reset flag so it homes again
				bool reset_val = false;
				xQueueOverwrite(xQueueReset, &reset_val);
				
				//Inform Linux board we're back to idle
				state = 0;
				xQueueOverwrite(xQueueState, &state);
				USART_Write(USART3, &state_command, 1);
				USART_Write(USART3, &state0, 1);
			}
			break;
		}
	}	
}

void USART3_IRQHandler(void)
{
	uint8_t msg = USART_Read(USART3);
	xQueueSendFromISR(xQueueUART, &msg, pdFALSE);
}