#include "motion_drive_helper.h"
#include "co_sdo.h"
#include <stddef.h>

float motion_drive_u32_to_float(uint32_t raw)
{
    union
    {
        uint32_t u32;
        float f32;
    } converter;

    converter.u32 = raw;
    return converter.f32;
}

uint32_t motion_drive_float_to_u32(float value)
{
    union
    {
        float f32;
        uint32_t u32;
    } converter;

    converter.f32 = value;
    return converter.u32;
}

void motion_drive_pack_u32_le(uint8_t *data, uint8_t offset, uint32_t value)
{
    data[offset] = (uint8_t)(value & 0x000000FFUL);
    data[offset + 1U] = (uint8_t)((value >> 8) & 0x000000FFUL);
    data[offset + 2U] = (uint8_t)((value >> 16) & 0x000000FFUL);
    data[offset + 3U] = (uint8_t)((value >> 24) & 0x000000FFUL);
}

void motion_drive_pack_u16_le(uint8_t *data, uint8_t offset, uint16_t value)
{
    data[offset] = (uint8_t)(value & 0x00FFU);
    data[offset + 1U] = (uint8_t)((value >> 8) & 0x00FFU);
}

motion_drive_status_t motion_drive_read_u32_optional(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t *value,
    uint8_t *valid_flag
)
{
    co_sdo_status_t sdo_status;
    uint32_t raw_value = 0U;

    if ((can_if == NULL) || (value == NULL) || (valid_flag == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    *valid_flag = 0U;

    sdo_status = co_sdo_read_u32(
        can_if,
        node_id,
        index,
        subindex,
        &raw_value,
        MOTION_DRIVE_SDO_TIMEOUT_MS
    );

    if (sdo_status != CO_SDO_OK)
    {
        return MOTION_DRIVE_SDO_ERROR;
    }

    *value = raw_value;
    *valid_flag = 1U;

    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_read_u16_optional(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint16_t *value,
    uint8_t *valid_flag
)
{
    uint32_t raw_value = 0U;
    motion_drive_status_t status;

    if (value == NULL)
    {
        return MOTION_DRIVE_ERROR;
    }

    status = motion_drive_read_u32_optional(
        can_if,
        node_id,
        index,
        subindex,
        &raw_value,
        valid_flag
    );

    if (status != MOTION_DRIVE_OK)
    {
        return status;
    }

    *value = (uint16_t)(raw_value & 0xFFFFU);

    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_read_u8_optional(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint8_t *value,
    uint8_t *valid_flag
)
{
    uint32_t raw_value = 0U;
    motion_drive_status_t status;

    if (value == NULL)
    {
        return MOTION_DRIVE_ERROR;
    }

    status = motion_drive_read_u32_optional(
        can_if,
        node_id,
        index,
        subindex,
        &raw_value,
        valid_flag
    );

    if (status != MOTION_DRIVE_OK)
    {
        return status;
    }

    *value = (uint8_t)(raw_value & 0xFFU);

    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_read_float_optional(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    float *value,
    uint8_t *valid_flag
)
{
    uint32_t raw_value = 0U;
    motion_drive_status_t status;

    if (value == NULL)
    {
        return MOTION_DRIVE_ERROR;
    }

    status = motion_drive_read_u32_optional(
        can_if,
        node_id,
        index,
        subindex,
        &raw_value,
        valid_flag
    );

    if (status != MOTION_DRIVE_OK)
    {
        return status;
    }

    *value = motion_drive_u32_to_float(raw_value);

    return MOTION_DRIVE_OK;
}
