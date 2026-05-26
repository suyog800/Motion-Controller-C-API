#ifndef CAN_HAL_STM32_H
#define CAN_HAL_STM32_H


#include "can_hal.h"

#include "stm32g4xx_hal.h"


#ifdef __cplusplus
extern "c"{
#endif

can_hal_status_t can_hal_stm32_init(FDCAN_HandleTypeDef *hfdcan);
can_hal_status_t can_hal_stm32_send(const can_frame_t *frame);
can_hal_status_t can_hal_stm32_recieve(can_frame_t *frame);

extern  can_hal_t can0;
#ifdef __cplusplus

}

#endif

#endif
