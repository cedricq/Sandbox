#include "DataAccessor.hpp"


Datagram DataItems[] =
{
// Measurements
[TIME_ID]          =    {.value = 0,     .reset = 0,  .min = INT32_MIN,  .max = INT32_MAX,  .div = 1000,    .name = "time"},
[QOUT_ID]          =    {.value = 0,     .reset = 0,  .min = -1000,      .max = 30000,      .div = 100,     .name = "qout"},
[POUT_ID]          =    {.value = 0,     .reset = 0,  .min = -1000,      .max = 10000,      .div = 100,     .name = "pout"},
[PPROX_ID]         =    {.value = 0,     .reset = 0,  .min = -1000,      .max = 10000,      .div = 100,     .name = "prox"},
[MOTOR_SPEED_ID]   =    {.value = 0,     .reset = 0,  .min = 0,          .max = 100000,     .div = 1,       .name = "motor_speed"},
[MOTOR_CURRENT_ID] =    {.value = 0,     .reset = 0,  .min = 0,          .max = 1000,       .div = 100,     .name = "motor_current"},

// // Targets
[MOTOR_TARGET_ID] =     {.value = 0,     .reset = 0,  .min = 0,          .max = 1000,       .div = 10,      .name = "motor_target"},
[ACT_1_TARGET_ID] =     {.value = 0,     .reset = 0,  .min = 0,          .max = 1000,       .div = 10,      .name = "act1_target"},
[ACT_2_TARGET_ID] =     {.value = 0,     .reset = 0,  .min = 0,          .max = 1000,       .div = 10,      .name = "act2_target"},
[ACT_3_TARGET_ID] =     {.value = 0,     .reset = 0,  .min = 0,          .max = 1000,       .div = 10,      .name = "act3_target"}

};

DataItem::DataItem(DataItemId id)
        :   data_(DataItems[id]),
            writable_(false)
{}

DataItem::DataItem(DataItemId id, bool writable)
        :   data_(DataItems[id]),
            writable_(writable)
{}

DataItem::~DataItem()
{}

Datagram& DataItem::get()
{
    return data_;
}


void DataItem::set(int32_t val)
{
    if (writable_)
    {
        if (val >= data_.min and val <= data_.max)
        {
            data_.value = val;
        }
    }
}

void DataItem::reset()
{
    set(data_.reset);
}
