#include "main_cpp.hpp"

#include "print_output.hpp"
#include "measurements.hpp"
#include "commands.hpp"

ADC_HandleTypeDef*  p_hadc1;
DMA_HandleTypeDef*  p_hdma_adc1;
DAC_HandleTypeDef*  p_hdac;
I2C_HandleTypeDef*  p_hi2c1;
TIM_HandleTypeDef*  p_htim2;
TIM_HandleTypeDef*  p_htim15;
UART_HandleTypeDef* p_huart3;
DMA_HandleTypeDef*  p_hdma_usart3_tx;

void init_main_cpp()
{
    init_commands();
    init_print_output(); // !!! Start UART before ADC  !!! ////////
    init_measurements();
}

void loop_main_cpp()
{

}

void tick_main_cpp()
{
    tick_measurements();
    tick_commands();
    tick_print_output();
}
