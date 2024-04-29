#include "main.h"
#include "gpio.h"

// EB low triggered
uint8_t eb_scan()
{
	return !HAL_GPIO_ReadPin(K_EB_GPIO_Port, K_EB_Pin);
}