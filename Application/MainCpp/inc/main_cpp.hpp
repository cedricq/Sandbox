#ifndef MAIN_CPP_HPP_
#define MAIN_CPP_HPP_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

extern ADC_HandleTypeDef*  p_hadc1;
extern DMA_HandleTypeDef*  p_hdma_adc1;
extern DAC_HandleTypeDef*  p_hdac;
extern I2C_HandleTypeDef*  p_hi2c1;
extern TIM_HandleTypeDef*  p_htim2;
extern TIM_HandleTypeDef*  p_htim15;
extern UART_HandleTypeDef* p_huart3;
extern DMA_HandleTypeDef*  p_hdma_usart3_tx;

void init_main_cpp();
void loop_main_cpp();
void tick_main_cpp();


void printFloats(float in[], int size);
void printFloatsTelePlot(float in[], char const* names[], int size);

#ifdef __cplusplus
}
#endif

#endif
