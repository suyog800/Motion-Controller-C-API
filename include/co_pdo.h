#ifndef CO_PDO_H
#define CO_PDO_H

#include <stdint.h>
#include "can_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CO_CAN_ID_RPDO1(node_id)    (0x200U + (uint32_t)(node_id))
#define CO_CAN_ID_RPDO2(node_id)    (0x300U + (uint32_t)(node_id))
#define CO_CAN_ID_RPDO3(node_id)    (0x400U + (uint32_t)(node_id))
#define CO_CAN_ID_RPDO4(node_id)    (0x500U + (uint32_t)(node_id))
#define CO_CAN_ID_RPDO5(node_id)    (0x680U + (uint32_t)(node_id))
#define CO_CAN_ID_RPDO6(node_id)    (0x690U + (uint32_t)(node_id))

#define CO_CAN_ID_TPDO1(node_id)    (0x180U + (uint32_t)(node_id))
#define CO_CAN_ID_TPDO2(node_id)    (0x280U + (uint32_t)(node_id))
#define CO_CAN_ID_TPDO3(node_id)    (0x380U + (uint32_t)(node_id))
#define CO_CAN_ID_TPDO4(node_id)    (0x480U + (uint32_t)(node_id))
//#define CO_CAN_ID_TPDO5(node_id)    (0x580U + (uint32_t)(node_id))
//#define CO_CAN_ID_TPDO6(node_id)    (0x680U + (uint32_t)(node_id))

can_hal_status_t co_pdo_send(
    const can_hal_t *can_if,
    uint32_t cob_id,
    const uint8_t *data,
    uint8_t dlc
);

can_hal_status_t co_pdo_receive(
    const can_hal_t *can_if,
    uint32_t *cob_id,
    uint8_t *data,
    uint8_t *dlc
);

#ifdef __cplusplus
}
#endif

#endif
