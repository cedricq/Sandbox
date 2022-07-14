#include "print_output.hpp"
#include "Fibre.hpp"
#include "DataAccessor.hpp"
#include "main.h"

#include <stdio.h>
#include <string.h>

#define UART_BUF_LEN 255
unsigned char UART3_rxBuffer[UART_BUF_LEN];


void printMessage(const char* message)
{
    HAL_UART_Transmit(p_huart3, (uint8_t*)message, (uint8_t)(strlen(message)), 100U);
}

void printDatas(DataItemId dataIds[], uint32_t size)
{
    char buffer[512]="";
    for (uint32_t i = 0; i < size; i++)
    {
        char txt[64];
        DataItem item(dataIds[i]);
        Datagram& data = item.get();

        sprintf(txt, "%d.%d ", static_cast<int>(data.value / data.div), static_cast<int>(data.value % data.div));
        strcat(buffer, txt);
    }
    strcat(buffer, "\r\n\0");
    HAL_UART_Transmit_DMA(p_huart3, (uint8_t*)buffer, (uint8_t)(strlen(buffer)));
}

void printTelePlot(DataItemId dataIds[], uint32_t size)
{
    char buffer[512]="";
    for (uint32_t i = 0; i < size; i++)
    {
        char txt[64];
        DataItem item(dataIds[i]);
        Datagram& data = item.get();

        sprintf(txt, ">%s:%d.%d ", data.name, static_cast<int>(data.value / data.div), static_cast<int>(data.value % data.div));
        strcat(buffer, txt);
        strcat(buffer, "\n");
    }
    strcat(buffer, "\0");
    HAL_UART_Transmit_DMA(p_huart3, (uint8_t*)buffer, (uint8_t)(strlen(buffer)));
}


class PrintFibre : public Fibre
{
public:
    PrintFibre(): Fibre("PrintFibre")
    {
        FibreManager& mgr = FibreManager::getInstance(THREAD_1MS_ID);
        mgr.Add(this);
    }

    virtual void Init()
    {
        HAL_UART_Receive_IT(p_huart3, UART3_rxBuffer, 1);
    }

    virtual void Run()
    {
        static DataItem time(TIME_ID);
        static DataItemId datas[] = {TIME_ID, QOUT_ID, POUT_ID, PPROX_ID, MOTOR_SPEED_ID, MOTOR_CURRENT_ID};

        if (time.get().value%10 == 0)
        {
            //printTelePlot(datas,  sizeof(datas)/sizeof(datas[0]));
            printDatas(datas,  sizeof(datas)/sizeof(datas[0]));
        }
    }
};

static PrintFibre printFibre;

