#ifndef CO_NMT_H
#define CO_NMT_H

#include <stdint.h>
#include "can_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CO_CAN_ID_NMT              0x000U
#define CO_NMT_DLC                 2U
#define CO_NMT_BROADCAST_NODE_ID   0U
#define CO_NMT_MAX_NODE_ID         127U

typedef enum
{
    CO_NMT_START_REMOTE_NODE      = 0x01,
    CO_NMT_STOP_REMOTE_NODE       = 0x02,
    CO_NMT_ENTER_PRE_OPERATIONAL  = 0x80,
    CO_NMT_RESET_NODE             = 0x81,
    CO_NMT_RESET_COMMUNICATION    = 0x82
} co_nmt_command_t;

can_hal_status_t co_nmt_send(
    const can_hal_t *can_if,
    co_nmt_command_t command,
    uint8_t node_id
);

can_hal_status_t co_nmt_start_node(
    const can_hal_t *can_if,
    uint8_t node_id
);

can_hal_status_t co_nmt_stop_node(
    const can_hal_t *can_if,
    uint8_t node_id
);

can_hal_status_t co_nmt_enter_pre_operational(
    const can_hal_t *can_if,
    uint8_t node_id
);

can_hal_status_t co_nmt_reset_node(
    const can_hal_t *can_if,
    uint8_t node_id
);

can_hal_status_t co_nmt_reset_communication(
    const can_hal_t *can_if,
    uint8_t node_id
);

#ifdef __cplusplus
}
#endif

#endif
