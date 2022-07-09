#ifndef MAINCPP_INC_PRINT_OUTPUT_HPP_
#define MAINCPP_INC_PRINT_OUTPUT_HPP_


#ifdef __cplusplus
extern "C" {
#endif

#define UART_BUF_LEN 255
extern unsigned char UART3_rxBuffer[UART_BUF_LEN];

void init_print_output();

void printFloats(float in[], int size);
void printFloatsTelePlot(float in[], char const* names[], int size);


#ifdef __cplusplus
}
#endif


#endif /* MAINCPP_INC_PRINT_OUTPUT_HPP_ */
