#ifndef ARC_DATAACCESSOR_HPP
#define ARC_DATAACCESSOR_HPP

#include "stm32f3xx_hal.h"

enum DataItemId
{
    // Measurements
    TIME_ID             = 0,
    QOUT_ID             = 1,
    POUT_ID             = 2,
    PPROX_ID            = 3,
    MOTOR_SPEED_ID      = 4,
    MOTOR_CURRENT_ID    = 5,

    // // Targets
    MOTOR_TARGET_ID     = 6,
    ACT_1_TARGET_ID     = 7,
    ACT_2_TARGET_ID     = 8,
    ACT_3_TARGET_ID     = 9,
};

struct Datagram
{
    int32_t value;
    int32_t reset;
    int32_t min;
    int32_t max;
    const char* name;
};

class DataItem
{
public:
    DataItem(DataItemId id);
    DataItem(DataItemId id, bool writable);
    ~DataItem();

    int32_t get();
    void set(int32_t val);
    void reset();


private:
    Datagram&   data_;
    bool        writable_ {false};
};

#endif
