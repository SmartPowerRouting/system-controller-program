#include "printf.h"
#include <string.h>

int fputc(int ch, FILE* f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFF);
    return ch;
}