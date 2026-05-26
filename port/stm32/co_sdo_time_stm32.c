#include "co_sdo.h"
#include "stm32g4xx_hal.h"

uint32_t co_sdo_get_time_ms(void)
{
    return HAL_GetTick();
}
