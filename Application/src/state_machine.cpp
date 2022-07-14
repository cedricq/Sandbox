#include "Fibre.hpp"
#include "DataAccessor.hpp"


class StateMachineFibre : public Fibre
{
public:
    StateMachineFibre(): Fibre("StateMachineFibre")
    //: main_motor_target{DataItem(MAIN_MOTOR_TARGET_ID, true)}
        //peep_motor_target(PEEP_MOTOR_TARGET_ID, true),
        //valve_ie_target(VALVE_IE_TARGET_ID, true),
        //test_target(TEST_TARGET_ID, true)
    {
        FibreManager& thread = FibreManager::getInstance(THREAD_1MS_ID);
        thread.Add(this);
    }

    DataItem main_motor_target_ {DataItem(MAIN_MOTOR_TARGET_ID, true)};
    DataItem peep_motor_target_ {DataItem(PEEP_MOTOR_TARGET_ID, true)};
    DataItem valve_ie_target_   {DataItem(VALVE_IE_TARGET_ID, true)};
    DataItem test_target_       {DataItem(TEST_TARGET_ID, true)};

    virtual void Init()
    {

    }

    const int TARGET_MOTOR_INSPI     = 750;
    const int TARGET_MOTOR_EXPI      = 600;
    const int TARGET_MOTOR_PEEP      = 150;
    const int TARGET_ON              = 999;
    const int TARGET_OFF             = 0;
    const int INSPI_TIME             = 2000;
    const int EXPI_TIME              = 3000;

    virtual void Run()
    {
        static DataItem time(TIME_ID);

        static int time_ini = time.get().value;
        static int time_duration = 0;

        peep_motor_target_.set(TARGET_MOTOR_PEEP);
        if ( (time.get().value - time_ini) > time_duration )
        {
            if (time_duration == INSPI_TIME)
            {
                main_motor_target_.set(TARGET_MOTOR_EXPI);
                valve_ie_target_.set(TARGET_OFF);
                time_duration = EXPI_TIME;
            }
            else
            {
                main_motor_target_.set(TARGET_MOTOR_INSPI);
                valve_ie_target_.set(TARGET_ON);
                time_duration = INSPI_TIME;
            }
            time_ini = time.get().value;
        }
    }
};

static StateMachineFibre stateMachineFibre;
