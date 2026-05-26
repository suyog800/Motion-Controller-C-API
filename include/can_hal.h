// can_hal.h

#ifndef CAN_HAL_H
#define CAN_HAL_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t id;        // 11-bit CAN ID for standard CANopen
    uint8_t dlc;        // data length, usually 8 for CANopen
    uint8_t data[8];
} can_frame_t;

typedef enum
{
    CAN_HAL_OK = 0,
    CAN_HAL_ERROR,
    CAN_HAL_TIMEOUT,
    CAN_HAL_RX_EMPTY
} can_hal_status_t;

typedef can_hal_status_t (*can_send_fn_t)(const can_frame_t *frame);
typedef can_hal_status_t (*can_receive_fn_t)(can_frame_t *frame);

typedef struct
{
    can_send_fn_t send;
    can_receive_fn_t receive;
} can_hal_t;

#endif
