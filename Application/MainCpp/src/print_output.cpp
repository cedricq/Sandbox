#include "print_output.hpp"
#include "main_cpp.hpp"

#include <stdio.h>
#include <string.h>

unsigned char UART3_rxBuffer[UART_BUF_LEN];

void printFloats(float in[], int size)
{
    char buffer[80]="";
    for (int i = 0; i < size; i++)
    {
        char txt[10];
        sprintf(txt, "%.2f ", in[i]);
        strcat(buffer, txt);
    }
    strcat(buffer, "\n");
    HAL_UART_Transmit_DMA(p_huart3, (uint8_t*)buffer, (uint8_t)(strlen(buffer)));
}


void printFloatsTelePlot(float in[], char const* names[], int size)
{
    char buffer[256]="";
    for (int i = 0; i < size; i++)
    {
        char txt[64];
        sprintf(txt, ">%s:%.2f ", names[i], in[i]);
        strcat(buffer, txt);
        strcat(buffer, "\r\n");
    }
    strcat(buffer, "\0");
    HAL_UART_Transmit_DMA(p_huart3, (uint8_t*)buffer, (uint8_t)(strlen(buffer)));
}

void init_print_output()
{
    HAL_UART_Receive_IT(p_huart3, UART3_rxBuffer, 1);
}



