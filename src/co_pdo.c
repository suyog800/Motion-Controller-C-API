#include "co_pdo.h"
#include <stddef.h>

can_hal_status_t co_pdo_send(
    const can_hal_t *can_if,
    uint32_t cob_id,
    const uint8_t *data,
    uint8_t dlc
)
{
    can_frame_t frame = {0};
    uint8_t i;

    if ((can_if == NULL) || (can_if->send == NULL))
    {
        return CAN_HAL_ERROR;
    }

    if (cob_id > 0x7FFU)
    {
        return CAN_HAL_ERROR;
    }

    if (dlc > 8U)
    {
        return CAN_HAL_ERROR;
    }

    if ((data == NULL) && (dlc > 0U))
    {
        return CAN_HAL_ERROR;
    }

    frame.id = cob_id;
    frame.dlc = dlc;

    for (i = 0U; i < dlc; i++)
    {
        frame.data[i] = data[i];
    }

    return can_if->send(&frame);
}

can_hal_status_t co_pdo_receive(
    const can_hal_t *can_if,
    uint32_t *cob_id,
    uint8_t *data,
    uint8_t *dlc
)
{
    can_frame_t frame = {0};
    can_hal_status_t status;
    uint8_t i;

    if ((can_if == NULL) || (can_if->receive == NULL))
    {
        return CAN_HAL_ERROR;
    }

    if ((cob_id == NULL) || (data == NULL) || (dlc == NULL))
    {
        return CAN_HAL_ERROR;
    }

    status = can_if->receive(&frame);

    if (status != CAN_HAL_OK)
    {
        return status;
    }

    if (frame.dlc > 8U)
    {
        return CAN_HAL_ERROR;
    }

    *cob_id = frame.id;
    *dlc = frame.dlc;

    for (i = 0U; i < frame.dlc; i++)
    {
        data[i] = frame.data[i];
    }

    for (i = frame.dlc; i < 8U; i++)
    {
        data[i] = 0U;
    }

    return CAN_HAL_OK;
}
