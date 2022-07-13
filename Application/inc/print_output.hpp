#ifndef MAINCPP_INC_PRINT_OUTPUT_HPP_
#define MAINCPP_INC_PRINT_OUTPUT_HPP_

#include "DataAccessor.hpp"


#ifdef __cplusplus
extern "C" {
#endif

#define UART_BUF_LEN 255
extern unsigned char UART3_rxBuffer[UART_BUF_LEN];

void init_print_output();
void tick_print_output();

void printDatas(DataItemId dataIds[], uint32_t size);
void printTelePlot(DataItemId dataIds[], uint32_t size);

#ifdef __cplusplus
}
#endif


#endif /* MAINCPP_INC_PRINT_OUTPUT_HPP_ */
