#include "commands.hpp"

#include "main_cpp.hpp"
#include "main.h"

#include "stm32f3xx_hal.h"


uint32_t target_motor_speed     = 20000;    // 20000 rpm
uint32_t target_motor_qout      = 1000;     // 10 L/min

uint32_t cmd_motor              = 0; //100;  // 10%
uint32_t cmd_peep               = 0; //200;  // 20%
uint32_t cmd_valve              = 0;    // 0%

void UpdatePWM1(uint32_t per1000)
{
    TIM15->CCR1 = (per1000 * TIM15_PERIOD_TICKS) /1000;
}

void UpdatePWM2(uint32_t per1000)
{
    TIM15->CCR2 = (per1000 * TIM15_PERIOD_TICKS) /1000;
}

void UpdatePWM3(uint32_t per1000)
{
    TIM2->CCR1 = (per1000 * TIM2_PERIOD_TICKS) /1000;
}


void init_commands()
{
    UpdatePWM1(0);
    UpdatePWM2(0);
    TIM15->CCER |= TIM_CCER_CC1E;
    TIM15->CCER |= TIM_CCER_CC2E;
    HAL_TIMEx_PWMN_Start(p_htim15, HAL_TIM_ACTIVE_CHANNEL_1);

    UpdatePWM3(0);
    TIM2->CCER |= TIM_CCER_CC1E;
    HAL_TIMEx_PWMN_Start(p_htim2, HAL_TIM_ACTIVE_CHANNEL_1);
}

const int CMD_MOTOR_INSPI     = 2500;
const int CMD_MOTOR_EXPI      = 500;
const int INSPI_TIME          = 3000; //1000;
const int EXPI_TIME           = 3000; //2000;

void tick_commands()
{
    static int time = 0;
    static int time_ini = 0;
    static int time_duration = 0;

    time += 1;

    cmd_peep = 500;
    if ( (time - time_ini) > time_duration )
    {
        if (cmd_motor == CMD_MOTOR_INSPI)
        {
            cmd_motor = 4*CMD_MOTOR_EXPI;
            cmd_valve = 0;
            time_duration = EXPI_TIME;
        }
        else
        {
            cmd_motor = CMD_MOTOR_INSPI;
            cmd_valve = 999;
            time_duration = INSPI_TIME;
        }
        time_ini = time;
    }

    // cmd_motor
    // Calculate new target motor
    // 3.3V - 4095 -> 45'000 rpm
    //int error = target_motor_qout - Measures[MEAS_QOUT];
    //int cmd_target_tmp = (int)cmd_motor + (error / 200);
    //if (cmd_target_tmp < 0) cmd_target_tmp = 0;
    //if (cmd_target_tmp > 4095) cmd_target_tmp = 4095;
    //cmd_motor = (uint32_t)cmd_target_tmp;

    DAC1->DHR12R1 = cmd_motor%4096;

    UpdatePWM1(cmd_valve);
    UpdatePWM2(cmd_peep);

    UpdatePWM3(cmd_valve);

}


