#include "co_nmt.h"

static int co_nmt_is_valid_node_id(uint8_t node_id)
{
    /*
     * CANopen valid node IDs are:
     *
     * 0   = broadcast to all nodes
     * 1-127 = specific node
     */
    return (node_id <= CO_NMT_MAX_NODE_ID);
}

static int co_nmt_is_valid_command(co_nmt_command_t command)
{
    switch (command)
    {
        case CO_NMT_START_REMOTE_NODE:
        case CO_NMT_STOP_REMOTE_NODE:
        case CO_NMT_ENTER_PRE_OPERATIONAL:
        case CO_NMT_RESET_NODE:
        case CO_NMT_RESET_COMMUNICATION:
            return 1;

        default:
            return 0;
    }
}

can_hal_status_t co_nmt_send(
    const can_hal_t *can_if,
    co_nmt_command_t command,
    uint8_t node_id
)
{
    can_frame_t frame = {0};

    if ((can_if == 0) || (can_if->send == 0))
    {
        return CAN_HAL_ERROR;
    }

    if (!co_nmt_is_valid_node_id(node_id))
    {
        return CAN_HAL_ERROR;
    }

    if (!co_nmt_is_valid_command(command))
    {
        return CAN_HAL_ERROR;
    }

    frame.id = CO_CAN_ID_NMT;
    frame.dlc = CO_NMT_DLC;

    frame.data[0] = (uint8_t)command;
    frame.data[1] = node_id;

    return can_if->send(&frame);
}

can_hal_status_t co_nmt_start_node(
    const can_hal_t *can_if,
    uint8_t node_id
)
{
    return co_nmt_send(can_if, CO_NMT_START_REMOTE_NODE, node_id);
}

can_hal_status_t co_nmt_stop_node(
    const can_hal_t *can_if,
    uint8_t node_id
)
{
    return co_nmt_send(can_if, CO_NMT_STOP_REMOTE_NODE, node_id);
}

can_hal_status_t co_nmt_enter_pre_operational(
    const can_hal_t *can_if,
    uint8_t node_id
)
{
    return co_nmt_send(can_if, CO_NMT_ENTER_PRE_OPERATIONAL, node_id);
}

can_hal_status_t co_nmt_reset_node(
    const can_hal_t *can_if,
    uint8_t node_id
)
{
    return co_nmt_send(can_if, CO_NMT_RESET_NODE, node_id);
}

can_hal_status_t co_nmt_reset_communication(
    const can_hal_t *can_if,
    uint8_t node_id
)
{
    return co_nmt_send(can_if, CO_NMT_RESET_COMMUNICATION, node_id);
}
