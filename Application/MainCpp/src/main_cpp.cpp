#include "main_cpp.hpp"


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

ADC_HandleTypeDef*  p_hadc1;
DMA_HandleTypeDef*  p_hdma_adc1;
DAC_HandleTypeDef*  p_hdac;
I2C_HandleTypeDef*  p_hi2c1;
TIM_HandleTypeDef*  p_htim2;
TIM_HandleTypeDef*  p_htim15;
UART_HandleTypeDef* p_huart3;
DMA_HandleTypeDef*  p_hdma_usart3_tx;

#ifdef __cplusplus
}
#endif


void main_init_cpp()
{

}

void main_cpp()
{

}
