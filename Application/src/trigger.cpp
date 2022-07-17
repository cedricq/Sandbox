#include "trigger.hpp"
#include "breath_machine.hpp"

const Datagram& TriggerI::time_ {DataItem(TIME_ID).get()};

class NoneTrigger : public TriggerI
{
public:
    virtual bool IsTrigger()
    {
        return false;
    }
    virtual void Reset() {};
};
static NoneTrigger noneTrigger_s;


TimeTrigger::TimeTrigger(Datagram& time, int32_t threshold) : time_(time), threshold_(threshold)
{}

void TimeTrigger::Set(int32_t threshold)
{
    threshold_ = threshold;
}

bool TimeTrigger::IsTrigger()
{
    return ( time_.value > threshold_ );
}
void TimeTrigger::Reset() {}


void TriggerManager::SetInspi(TriggerI& trigger)
{
    triggerInspi_ = trigger;
}

void TriggerManager::SetExpi(TriggerI& trigger)
{
    triggerExpi_ = trigger;
}

bool TriggerManager::IsTrigger()
{
    if ( state_.value == EXPI )
    {
        return triggerInspi_.IsTrigger();
    }
    else
    {
        return triggerExpi_.IsTrigger();
    }
}

TriggerManager::TriggerManager():
        triggerInspi_(noneTrigger_s),
        triggerExpi_(noneTrigger_s),
        state_(DataItem(BREATH_STATE_ID).get())
{}

TriggerManager& TriggerManager::GetInstance()
{
    static TriggerManager trg;
    return trg;
}

