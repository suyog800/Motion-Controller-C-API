#ifndef CO_SDO_H
#define CO_SDO_H

#include <stdint.h>
#include "can_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CO_CAN_ID_SDO_TX(node_id)   (0x600U + (uint32_t)(node_id))  /* master to slave */
#define CO_CAN_ID_SDO_RX(node_id)   (0x580U + (uint32_t)(node_id))  /* slave to master */

#define CO_SDO_DLC                  8U
#define CO_SDO_MAX_NODE_ID          127U

typedef enum
{
    CO_SDO_OK = 0,
    CO_SDO_ERROR,
    CO_SDO_TIMEOUT,
    CO_SDO_ABORT
} co_sdo_status_t;

/*
 * The SDO client needs a millisecond timebase for timeout handling.
 *
 * In STM32, implement this function like:
 *
 * uint32_t co_sdo_get_time_ms(void)
 * {
 *     return HAL_GetTick();
 * }
 *
 * A weak fallback is provided in co_sdo.c, but real projects should override it.
 */
uint32_t co_sdo_get_time_ms(void);

co_sdo_status_t co_sdo_write_u8(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint8_t value,
    uint32_t timeout_ms
);

co_sdo_status_t co_sdo_write_u16(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint16_t value,
    uint32_t timeout_ms
);

co_sdo_status_t co_sdo_write_u32(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t value,
    uint32_t timeout_ms
);

co_sdo_status_t co_sdo_read_u32(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t *value,
    uint32_t timeout_ms
);

#ifdef __cplusplus
}
#endif

#endif
