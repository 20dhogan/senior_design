#ifndef JET_CUTTER_H
#define JET_CUTTER_H
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "stm32l476xx.h"

//#define variables
typedef struct {
    int32_t x;
    int32_t y;
	} Point_t;

typedef struct {
	Point_t  *points;
	uint16_t  count;
} Tool_t;

extern Tool_t    *g_tools;
extern uint16_t   g_total_tools;

#define MAX_X_STEPS 75000
#define MAX_Y_STEPS 75000
#define MAX_Z_STEPS 13000;
#define MS_DELAY_INBETWEEN_TOOLS 2000
#define SPEED_1 102400
#define SPEED_2 51200
#define SPEED_3 25600
#define SPEED_4 12800
#define SPEED_5 6400
#define SPEED_6 3200
#define SPEED_7 1600
#define SPEED_8 800
#define SPEED_9 400
#define SPEED_10 200

//function delorations
//TIMER**************************************
void TIM4_Init(void);
void TIM4_IRQHandler(void);
void delay_init(void);
static inline void delay_us(uint32_t us);
void main_timer_function();
uint16_t speed_one_to_ten(int speed);

//UART***************************************
void USART3_Init(void);
void USART3_IRQHandler(void);
uint8_t USART_Read(USART_TypeDef *USARTx);
void USART_Write(USART_TypeDef * USARTx, uint8_t * buffer, int nBytes);

//TASKS***************************************
void STM32MP_communication(void *argument);
void Motor_control(void *argument);
void Pressure_washer_control(void *argument);

//Helper Functions****************************
void gpio_Write(GPIO_TypeDef *port, uint8_t pin, bool val);
bool gpio_Read(GPIO_TypeDef *port, uint8_t pin);
void general_motors_init(void);
void prvSetupHardware();
void motor_pulse(int axis, bool cw);
void motor_and_sensor_init(void);

#endif