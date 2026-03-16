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

void Pressure_washer_control(void *argument){

	while(1){
	
	}
	
}