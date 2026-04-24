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
extern QueueHandle_t xQueue_pressure_washer_on;
extern QueueHandle_t xQueueSpeed;
Tool_t   *g_tools       = NULL;
uint16_t  g_total_tools = 0;
					
					
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
	const uint8_t speed_command = 0x56;
	const uint8_t transfer_alloc_fail = 0xE1;
	bool reset = false;
	bool pressure_washer_on = false;
	uint8_t nozzle_lsb = 0;
	uint8_t nozzle_msb = 0;
	int32_t requested_z_steps = 0;
	xQueueOverwrite(xQueueState, &state);
	uint8_t msg2x[2] = {0, 0};

	
	while(1){
		switch (state){
			
			case 0: //idle state
				while(xQueueReceive(xQueueUART, &msg2x[0], portMAX_DELAY) != pdPASS); //first message, expected 0x55
				if(msg2x[0] == 0x55){
					while(xQueueReceive(xQueueUART, &msg2x[1], portMAX_DELAY) != pdPASS); //second message, expected 0x01
					if(msg2x[1] == 0x01){
						while(xQueueReceive(xQueueUART, &nozzle_lsb, portMAX_DELAY) != pdPASS);
						while(xQueueReceive(xQueueUART, &nozzle_msb, portMAX_DELAY) != pdPASS);
						requested_z_steps = (int32_t)((uint16_t)nozzle_lsb | ((uint16_t)nozzle_msb << 8));
						//Then linux board is requesting to move to state 1
						//Lets see if the machine has reset yet.
						xQueuePeek(xQueueReset, &reset, 100);
						
						if(reset == true){						
							xQueueOverwrite(xQueueZStepPosition, &requested_z_steps);
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
				uint16_t tool_count = 0;

				/* Free any previous allocation */
					if (g_tools != NULL) {
						for (uint16_t k = 0; k < g_total_tools; k++) {
							if (g_tools[k].points != NULL) vPortFree(g_tools[k].points);
						}
						vPortFree(g_tools);
						g_tools = NULL;
				}
				g_total_tools = 0;

				/* Wait for START_TRANSFER = 0xAA */
				do {
					xQueueReceive(xQueueUART, &rx, portMAX_DELAY);
				} while (rx != 0xAA);

					bool alloc_ok = true;

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

						/* Grow the g_tools array by one entry */
						Tool_t *new_g_tools = pvPortMalloc((tool_count + 1) * sizeof(Tool_t));
						if (new_g_tools == NULL) {
							alloc_ok = false;
							break;
					}
						if (tool_count > 0) {
							memcpy(new_g_tools, g_tools, tool_count * sizeof(Tool_t));
							vPortFree(g_tools);
					}
						g_tools = new_g_tools;
						g_tools[tool_count].points = NULL;
						g_tools[tool_count].count  = 0;

						/* Allocate the points array for this tool */
						g_tools[tool_count].points = pvPortMalloc(tool_point_count * sizeof(Point_t));
						if (g_tools[tool_count].points == NULL) {
							tool_count++;   /* include this entry in cleanup */
							alloc_ok = false;
							break;
		 				}
						g_tools[tool_count].count = tool_point_count;

					/* Receive points for this tool */
					for (uint16_t i = 0; i < tool_point_count; i++) {

						uint8_t b[4];

						/* X */
						xQueueReceive(xQueueUART, &b[0], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[1], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[2], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[3], portMAX_DELAY);
							g_tools[tool_count].points[i].x =
								  (int32_t)b[0]
								| ((int32_t)b[1] << 8)
								| ((int32_t)b[2] << 16)
								| ((int32_t)b[3] << 24);

						/* Y */
						xQueueReceive(xQueueUART, &b[0], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[1], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[2], portMAX_DELAY);
						xQueueReceive(xQueueUART, &b[3], portMAX_DELAY);
							g_tools[tool_count].points[i].y =
								  (int32_t)b[0]
								| ((int32_t)b[1] << 8)
								| ((int32_t)b[2] << 16)
								| ((int32_t)b[3] << 24);
					}

						tool_count++;

					/* Wait for TOOL_END = 0xAC */
					do {
						xQueueReceive(xQueueUART, &rx, portMAX_DELAY);
					} while (rx != 0xAC);
				}

					if (!alloc_ok) {
						if (g_tools != NULL) {
							for (uint16_t k = 0; k < tool_count; k++) {
								if (g_tools[k].points != NULL) vPortFree(g_tools[k].points);
							}
							vPortFree(g_tools);
							g_tools = NULL;
						}
						g_total_tools = 0;
						USART_Write(USART3, &transfer_alloc_fail, 1);
						state = 0;
						xQueueOverwrite(xQueueState, &state);
						break;
					}

					g_total_tools = tool_count;
				
				//acknowledge all data has been received
				USART_Write(USART3, &done_command, 1);
				
				//wait for speed command and state change command
				while (state == 1) {
					while(xQueueReceive(xQueueUART, &msg2x[0], portMAX_DELAY) != pdPASS);
					while(xQueueReceive(xQueueUART, &msg2x[1], portMAX_DELAY) != pdPASS);

					if(msg2x[0] == speed_command){
						if((msg2x[1] >= 1) && (msg2x[1] <= 10)){
							uint16_t timer_period = speed_one_to_ten((int)msg2x[1]);
							xQueueOverwrite(xQueueSpeed, &timer_period);
						}
						USART_Write(USART3, &speed_command, 1);
						USART_Write(USART3, &msg2x[1], 1);
					} else if((msg2x[0] == state_command) && (msg2x[1] == state2)){
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
									//turn pressure washer off. Then change state
									pressure_washer_on = false;
									xQueueOverwrite(xQueue_pressure_washer_on, &pressure_washer_on);
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
									//Now we need to start the pressure washer first
									//Then wait for it to get going and then set the system to state 2.
									pressure_washer_on = true;
									xQueueOverwrite(xQueue_pressure_washer_on, &pressure_washer_on);
									vTaskDelay(pdMS_TO_TICKS(MS_DELAY_INBETWEEN_TOOLS));
									
									//now set the whole system back to the cutting state.
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
				if (g_tools != NULL) {
					for (uint16_t k = 0; k < g_total_tools; k++) {
						if (g_tools[k].points != NULL) vPortFree(g_tools[k].points);
					}
					vPortFree(g_tools);
					g_tools = NULL;
				}
				g_total_tools = 0;
				
				//Reset cutting complete flag
				bool complete = false;
				xQueueOverwrite(xQueue_cutting_complete, &complete);

				//Clear movement planning state so next job starts clean.
				uint32_t zero_u32 = 0;
				bool reached_true = true;
				bool dir_default_true = true;
				xQueueOverwrite(xQueue_Xsteps_to_move, &zero_u32);
				xQueueOverwrite(xQueue_Ysteps_to_move, &zero_u32);
				xQueueOverwrite(xQueue_Xsteps_counter, &zero_u32);
				xQueueOverwrite(xQueue_Ysteps_counter, &zero_u32);
				xQueueOverwrite(xQueue_number_of_ticks, &zero_u32);
				xQueueOverwrite(xQueue_reached_state, &reached_true);
				xQueueOverwrite(xQueue_x_cw, &dir_default_true);
				xQueueOverwrite(xQueue_y_cw, &dir_default_true);

				//Wait for Linux board acknowledgement (foam removed) before reset/idle.
				while (state == 4) {
					while(xQueueReceive(xQueueUART, &msg2x[0], portMAX_DELAY) != pdPASS);
					while(xQueueReceive(xQueueUART, &msg2x[1], portMAX_DELAY) != pdPASS);

					if ((msg2x[0] == state_command) && (msg2x[1] == state0)) {
						//Reset the machine reset flag so it homes again
						bool reset_val = false;
						xQueueOverwrite(xQueueReset, &reset_val);

						//Inform Linux board we're now going back to idle
						state = 0;
						xQueueOverwrite(xQueueState, &state);
						USART_Write(USART3, &state_command, 1);
						USART_Write(USART3, &state0, 1);
					}
				}
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