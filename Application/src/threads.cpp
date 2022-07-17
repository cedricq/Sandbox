#include "threads.hpp"
#include "Fibre.hpp"

#include "DataAccessor.hpp"

void init_threads()
{
    FibreManager& thread_1ms = FibreManager::getInstance(THREAD_1MS_ID);
    FibreManager& thread_polled = FibreManager::getInstance(THREAD_POLLED_ID);

    thread_polled.Init();
    thread_1ms.Init();
}

void tick_polled()
{
    static FibreManager& thread_polled = FibreManager::getInstance(THREAD_POLLED_ID);

    //static Datagram& time = DataItem(TIME_ID).get();
    //static uint32_t time_ini = time.value;

    //if ( time.value - time_ini > 10 )
    {
        thread_polled.Run();
        //time_ini = time.value;

    }
}

void tick_1ms()
{
    static FibreManager& thread_1ms = FibreManager::getInstance(THREAD_1MS_ID);
    thread_1ms.Run();
}

void tick_10ms()
{
    static FibreManager& thread_10ms = FibreManager::getInstance(THREAD_10MS_ID);
    thread_10ms.Run();
}

