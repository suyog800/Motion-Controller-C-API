#ifndef MOTION_DRIVE_HELPER_H
#define MOTION_DRIVE_HELPER_H

#include <stdint.h>
#include "can_hal.h"
#include "motion_drive.h"

#ifdef __cplusplus
extern "C" {
#endif


float motion_drive_u32_to_float(uint32_t raw);

uint32_t motion_drive_float_to_u32(float value);

void motion_drive_pack_u32_le(uint8_t *data, uint8_t offset, uint32_t value);

void motion_drive_pack_u16_le(uint8_t *data, uint8_t offset, uint16_t value);



motion_drive_status_t motion_drive_read_u8_optional(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint8_t *value,
    uint8_t *valid_flag
);

motion_drive_status_t motion_drive_read_u16_optional(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint16_t *value,
    uint8_t *valid_flag
);

motion_drive_status_t motion_drive_read_u32_optional(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t *value,
    uint8_t *valid_flag
);

motion_drive_status_t motion_drive_read_float_optional(
    const can_hal_t *can_if,
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    float *value,
    uint8_t *valid_flag
);

#ifdef __cplusplus
}
#endif

#endif
