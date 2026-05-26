#include "co_sdo.h"
#include <stddef.h>

/* SDO command specifiers */
#define CO_SDO_CMD_DOWNLOAD_U32         0x23U
#define CO_SDO_CMD_DOWNLOAD_U16         0x2BU
#define CO_SDO_CMD_DOWNLOAD_U8          0x2FU
#define CO_SDO_CMD_DOWNLOAD_RESPONSE    0x60U

#define CO_SDO_CMD_UPLOAD_REQUEST       0x40U
#define CO_SDO_CMD_UPLOAD_RESPONSE_MASK 0xE0U
#define CO_SDO_CMD_UPLOAD_RESPONSE      0x40U

#define CO_SDO_CMD_ABORT                0x80U

/*
 * Weak default time function.
 *
 * For STM32, create a separate file or add in your STM32 port:
 *
 * uint32_t co_sdo_get_time_ms(void)
 * {
 *     return HAL_GetTick();
 * }
 *
 * This weak function is only a fallback.
 */
#if defined(__GNUC__)
__attribute__((weak))
#endif
uint32_t co_sdo_get_time_ms(void)
{
    return 0U;
}

static int co_sdo_is_valid_node_id(uint8_t node_id)
{
    return ((node_id >= 1U) && (node_id <= CO_SDO_MAX_NODE_ID));
}

static int co_sdo_is_timeout(uint32_t start_ms, uint32_t timeout_ms)
{
    uint32_t now_ms = co_sdo_get_time_ms();

    /*
     * Unsigned subtraction handles wraparound safely.
     */
    return ((uint32_t)(now_ms - start_ms) >= timeout_ms);
}

static void co_sdo_write_index(can_frame_t *frame, uint16_t index, uint8_t subindex)
{
    frame->data[1] = (uint8_t)(index & 0x00FFU);
    frame->data[2] = (uint8_t)((index >> 8) & 0x00FFU);
    frame->data[3] = subindex;
}

static uint32_t co_sdo_read_u32_le(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void co_sdo_write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0x00FFU);
    data[1] = (uint8_t)((value >> 8) & 0x00FFU);
}

static void co_sdo_write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0x000000FFUL);
    data[1] = (uint8_t)((value >> 8) & 0x000000FFUL);
    data[2] = (uint8_t)((value >> 16) & 0x000000FFUL);
    data[3] = (uint8_t)((value >> 24) & 0x000000FFUL);
}

static int co_sdo_response_matches(
    const can_frame_t *frame,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex
)
{
    if (frame->id != CO_CAN_ID_SDO_RX(node_id))
    {
        return 0;
    }

    if (frame->dlc != CO_SDO_DLC)
    {
        return 0;
    }

    if (frame->data[1] != (uint8_t)(index & 0x00FFU))
    {
        return 0;
    }

    if (frame->data[2] != (uint8_t)((index >> 8) & 0x00FFU))
    {
        return 0;
    }

    if (frame->data[3] != subindex)
    {
        return 0;
    }

    return 1;
}

static co_sdo_status_t co_sdo_wait_download_response(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t timeout_ms
)
{
    uint32_t start_ms = co_sdo_get_time_ms();

    while (!co_sdo_is_timeout(start_ms, timeout_ms))
    {
        can_frame_t rx_frame;
        can_hal_status_t rx_status;

        rx_status = can_if->receive(&rx_frame);

        if (rx_status == CAN_HAL_RX_EMPTY)
        {
            continue;
        }

        if (rx_status != CAN_HAL_OK)
        {
            return CO_SDO_ERROR;
        }

        if (!co_sdo_response_matches(&rx_frame, node_id, index, subindex))
        {
//        	this blocking SDO receive loop can consume and lose unrelated CAN frames
            /*
             * Not the SDO response we are waiting for.
             * For a simple blocking implementation, ignore it.
             *
             * Later, when you add a dispatcher, unrelated frames should go
             * to heartbeat, EMCY, PDO, etc.
             */
            continue;
        }

        if (rx_frame.data[0] == CO_SDO_CMD_ABORT)
        {
            return CO_SDO_ABORT;
        }

        if (rx_frame.data[0] != CO_SDO_CMD_DOWNLOAD_RESPONSE)
        {
            return CO_SDO_ERROR;
        }

        return CO_SDO_OK;
    }

    return CO_SDO_TIMEOUT;
}

static co_sdo_status_t co_sdo_download_expedited(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    const uint8_t *data,
    uint8_t size,
    uint8_t command,
    uint32_t timeout_ms
)
{
    can_frame_t tx_frame = {0};

    if ((can_if == NULL) || (can_if->send == NULL) || (can_if->receive == NULL))
    {
        return CO_SDO_ERROR;
    }

    if (!co_sdo_is_valid_node_id(node_id))
    {
        return CO_SDO_ERROR;
    }

    if ((data == NULL) || (size == 0U) || (size > 4U))
    {
        return CO_SDO_ERROR;
    }

    tx_frame.id = CO_CAN_ID_SDO_TX(node_id);
    tx_frame.dlc = CO_SDO_DLC;

    tx_frame.data[0] = command;
    co_sdo_write_index(&tx_frame, index, subindex);

    tx_frame.data[4] = 0U;
    tx_frame.data[5] = 0U;
    tx_frame.data[6] = 0U;
    tx_frame.data[7] = 0U;

    for (uint8_t i = 0U; i < size; i++)
    {
        tx_frame.data[4U + i] = data[i];
    }

    if (can_if->send(&tx_frame) != CAN_HAL_OK)
    {
        return CO_SDO_ERROR;
    }

    return co_sdo_wait_download_response(
        can_if,
        node_id,
        index,
        subindex,
        timeout_ms
    );
}

co_sdo_status_t co_sdo_write_u8(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint8_t value,
    uint32_t timeout_ms
)
{
    uint8_t data[1];

    data[0] = value;

    return co_sdo_download_expedited(
        can_if,
        node_id,
        index,
        subindex,
        data,
        1U,
        CO_SDO_CMD_DOWNLOAD_U8,
        timeout_ms
    );
}

co_sdo_status_t co_sdo_write_u16(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint16_t value,
    uint32_t timeout_ms
)
{
    uint8_t data[2];

    co_sdo_write_u16_le(data, value);

    return co_sdo_download_expedited(
        can_if,
        node_id,
        index,
        subindex,
        data,
        2U,
        CO_SDO_CMD_DOWNLOAD_U16,
        timeout_ms
    );
}

co_sdo_status_t co_sdo_write_u32(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t value,
    uint32_t timeout_ms
)
{
    uint8_t data[4];

    co_sdo_write_u32_le(data, value);

    return co_sdo_download_expedited(
        can_if,
        node_id,
        index,
        subindex,
        data,
        4U,
        CO_SDO_CMD_DOWNLOAD_U32,
        timeout_ms
    );
}

co_sdo_status_t co_sdo_read_u32(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t *value,
    uint32_t timeout_ms
)
{
    can_frame_t tx_frame = {0};
    uint32_t start_ms;

    if ((can_if == NULL) || (can_if->send == NULL) || (can_if->receive == NULL))
    {
        return CO_SDO_ERROR;
    }

    if (value == NULL)
    {
        return CO_SDO_ERROR;
    }

    if (!co_sdo_is_valid_node_id(node_id))
    {
        return CO_SDO_ERROR;
    }

    tx_frame.id = CO_CAN_ID_SDO_TX(node_id);
    tx_frame.dlc = CO_SDO_DLC;

    tx_frame.data[0] = CO_SDO_CMD_UPLOAD_REQUEST;
    co_sdo_write_index(&tx_frame, index, subindex);

    tx_frame.data[4] = 0U;
    tx_frame.data[5] = 0U;
    tx_frame.data[6] = 0U;
    tx_frame.data[7] = 0U;

    if (can_if->send(&tx_frame) != CAN_HAL_OK)
    {
        return CO_SDO_ERROR;
    }

    start_ms = co_sdo_get_time_ms();

    while (!co_sdo_is_timeout(start_ms, timeout_ms))
    {
        can_frame_t rx_frame;
        can_hal_status_t rx_status;

        rx_status = can_if->receive(&rx_frame);

        if (rx_status == CAN_HAL_RX_EMPTY)
        {
            continue;
        }

        if (rx_status != CAN_HAL_OK)
        {
            return CO_SDO_ERROR;
        }

        if (!co_sdo_response_matches(&rx_frame, node_id, index, subindex))
        {
            continue;
        }

        if (rx_frame.data[0] == CO_SDO_CMD_ABORT)
        {
            return CO_SDO_ABORT;
        }

        /*
         * Expedited upload response has command byte:
         *
         * 0x43 = 4-byte value
         * 0x4B = 2-byte value
         * 0x4F = 1-byte value
         *
         * For read_u32, we accept any expedited response up to 4 bytes and
         * return the value zero-extended into uint32_t.
         */
        if ((rx_frame.data[0] & CO_SDO_CMD_UPLOAD_RESPONSE_MASK) != CO_SDO_CMD_UPLOAD_RESPONSE)
        {
            return CO_SDO_ERROR;
        }

        *value = co_sdo_read_u32_le(&rx_frame.data[4]);

        return CO_SDO_OK;
    }

    return CO_SDO_TIMEOUT;
}
