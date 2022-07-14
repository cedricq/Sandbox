#include "Fibre.hpp"
#include "DataAccessor.hpp"

#include "main.h"

#include "stm32f3xx_hal.h"

struct PWM
{
    volatile uint32_t& pwm;
    uint32_t period;
};

const PWM pwm1 = {.pwm = TIM15->CCR1, .period = TIM15_PERIOD_TICKS};
const PWM pwm2 = {.pwm = TIM15->CCR2, .period = TIM15_PERIOD_TICKS};
const PWM pwm3 = {.pwm = TIM2->CCR1,  .period = TIM2_PERIOD_TICKS};

int32_t TargetToDAC(int32_t target, int32_t div)
{
    static const int32_t MAX_RAW_DAC = 4095;
    return target * MAX_RAW_DAC / div;
}

void UpdatePWM(const PWM& pwm, int32_t target, int32_t div)
{
    pwm.pwm = (target * pwm.period) / div;
}

const int CMD_MOTOR_INSPI     = 2500;
const int CMD_MOTOR_EXPI      = 500;
const int INSPI_TIME          = 3000; //1000;
const int EXPI_TIME           = 3000; //2000;

class CommandFibre : public Fibre
{
public:
    CommandFibre(): Fibre("CommandFibre")
    {
        FibreManager& thread = FibreManager::getInstance(THREAD_1MS_ID);
        thread.Add(this);
    }

    virtual void Init()
    {
        UpdatePWM(pwm1, 0, 1);
        UpdatePWM(pwm2, 0, 1);
        TIM15->CCER |= TIM_CCER_CC1E;
        TIM15->CCER |= TIM_CCER_CC2E;
        HAL_TIMEx_PWMN_Start(p_htim15, HAL_TIM_ACTIVE_CHANNEL_1);

        UpdatePWM(pwm3, 0, 1);
        TIM2->CCER |= TIM_CCER_CC1E;
        HAL_TIMEx_PWMN_Start(p_htim2, HAL_TIM_ACTIVE_CHANNEL_1);
    }

    virtual void Run()
    {
        static DataItem main_motor_target(MAIN_MOTOR_TARGET_ID);
        static DataItem peep_motor_target(PEEP_MOTOR_TARGET_ID);
        static DataItem valve_ie_target(VALVE_IE_TARGET_ID);
        static DataItem test_target(TEST_TARGET_ID);

        DAC1->DHR12R1 = TargetToDAC(main_motor_target.get().value, main_motor_target.get().div);
        UpdatePWM(pwm1, valve_ie_target.get().value, valve_ie_target.get().div);
        UpdatePWM(pwm2, peep_motor_target.get().value, peep_motor_target.get().div);
        UpdatePWM(pwm3, test_target.get().value, test_target.get().div);
    }
};

static CommandFibre commandFibre;



