#include "jet_cutter.h"

volatile QueueHandle_t xQueueState = NULL;
volatile QueueHandle_t xQueueUART = NULL; //uart contents sent from uart to communication task
volatile QueueHandle_t xQueueXStepPosition = NULL;
volatile QueueHandle_t xQueueYStepPosition = NULL;
volatile QueueHandle_t xQueueZStepPosition = NULL;
volatile QueueHandle_t xQueueReset = NULL;
volatile QueueHandle_t xQueue_Xsteps_to_move = NULL;
volatile QueueHandle_t xQueue_Ysteps_to_move = NULL;
volatile QueueHandle_t xQueue_Xsteps_counter = NULL;
volatile QueueHandle_t xQueue_Ysteps_counter = NULL;
volatile QueueHandle_t xQueue_number_of_ticks = NULL;
volatile QueueHandle_t xQueue_reached_state = NULL;
volatile QueueHandle_t xQueue_x_cw = NULL;
volatile QueueHandle_t xQueue_y_cw = NULL;
volatile QueueHandle_t xQueue_cutting_complete = NULL;


int main(void){
	TaskHandle_t xHandleSTM32MP_communication = NULL;
	TaskHandle_t xHandleMotor_control = NULL;
	TaskHandle_t xHandlePressure_washer_control = NULL;
	
	xQueueState = xQueueCreate(1, sizeof(uint8_t));
	xQueueUART = xQueueCreate(1024, sizeof(uint8_t));
	xQueueXStepPosition = xQueueCreate(1, sizeof(int32_t));
	xQueueYStepPosition = xQueueCreate(1, sizeof(int32_t));
	xQueueZStepPosition = xQueueCreate(1, sizeof(int32_t));
	xQueueReset = xQueueCreate(1, sizeof(bool));
	xQueue_Xsteps_to_move = xQueueCreate(1, sizeof(uint32_t));
	xQueue_Ysteps_to_move = xQueueCreate(1, sizeof(uint32_t));
	xQueue_Xsteps_counter = xQueueCreate(1, sizeof(uint32_t));
	xQueue_Ysteps_counter = xQueueCreate(1, sizeof(uint32_t));
	xQueue_number_of_ticks = xQueueCreate(1, sizeof(uint32_t));
	xQueue_reached_state = xQueueCreate(1, sizeof(bool));
    xQueue_x_cw = xQueueCreate(1, sizeof(bool));
	xQueue_y_cw = xQueueCreate(1, sizeof(bool));
	xQueue_cutting_complete = xQueueCreate(1, sizeof(bool));

	uint8_t init_state = 0;
	bool init_reset = false;
	int32_t init_x_pos = 0;
	int32_t init_y_pos = 0;
	int32_t init_z_pos = 3000;
	uint32_t init_u32 = 0;
	bool init_true = true;
	bool init_false = false;

	xQueueOverwrite(xQueueState, &init_state);
	xQueueOverwrite(xQueueReset, &init_reset);
	xQueueOverwrite(xQueueXStepPosition, &init_x_pos);
	xQueueOverwrite(xQueueYStepPosition, &init_y_pos);
	xQueueOverwrite(xQueueZStepPosition, &init_z_pos);
	xQueueOverwrite(xQueue_Xsteps_to_move, &init_u32);
	xQueueOverwrite(xQueue_Ysteps_to_move, &init_u32);
	xQueueOverwrite(xQueue_Xsteps_counter, &init_u32);
	xQueueOverwrite(xQueue_Ysteps_counter, &init_u32);
	xQueueOverwrite(xQueue_number_of_ticks, &init_u32);
	xQueueOverwrite(xQueue_reached_state, &init_true);
	xQueueOverwrite(xQueue_x_cw, &init_true);
	xQueueOverwrite(xQueue_y_cw, &init_true);
	xQueueOverwrite(xQueue_cutting_complete, &init_false);
	
	
	if((xQueueState && xQueueUART) != NULL){
			prvSetupHardware();

			xTaskCreate(
				STM32MP_communication,
				"this task manages the states and all UART communications",
				512,
				NULL,
				1,
				&xHandleSTM32MP_communication
			);
		
		xTaskCreate(
				Motor_control,
				"this task calculates the number of steps and which direction each motor needs to take",
				256,
				NULL,
				1,
				&xHandleMotor_control
			);
			
			xTaskCreate(
				Pressure_washer_control,
				"This task contolls when the pressure washer turns on and off",
				128,
				NULL,
				1,
				&xHandlePressure_washer_control
			);
		
			vTaskStartScheduler();
			
			while(1);
	}
	return 0;
}