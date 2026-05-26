#include "motion_drive.h"
#include "motion_drive_helper.h"
#include "co_nmt.h"
#include "co_pdo.h"
#include "co_sdo.h"
#include <stddef.h>
#include <string.h>

static int motion_drive_is_valid_node_id(uint8_t node_id)
{
    return ((node_id >= 1U) && (node_id <= 127U));
}

motion_drive_status_t motion_drive_init(
    motion_drive_t *drive,
    const can_hal_t *can_if,
    uint8_t node_id
)
{
    if ((drive == NULL) || (can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if ((can_if->send == NULL) || (can_if->receive == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    memset(drive, 0, sizeof(*drive));

    drive->can_if = can_if;
    drive->node_id = node_id;
    drive->nmt_state = MOTION_DRIVE_NMT_PRE_OPERATIONAL;
    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_refresh_cache(
    motion_drive_t *drive
)
{
    if ((drive == NULL) || (drive->can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    /*
     * Similar to Python try/except/pass.
     * If one SDO read fails, the remaining reads continue.
     */

    (void)motion_drive_read_u16_optional(
        drive->can_if,
        drive->node_id,
        CONTROLWORD_INDEX,
        CONTROLWORD_SUB,
        &drive->controlword_val,
        &drive->controlword_valid
    );

    (void)motion_drive_read_u8_optional(
        drive->can_if,
        drive->node_id,
        MODES_OF_OPERATION_INDEX,
        MODES_OF_OPERATION_SUB,
        &drive->mode_of_operation_val,
        &drive->mode_of_operation_valid
    );

    (void)motion_drive_read_float_optional(
        drive->can_if,
        drive->node_id,
        TARGET_IQ_INDEX,
        TARGET_IQ_SUB,
        &drive->target_iq_val,
        &drive->target_iq_valid
    );

    (void)motion_drive_read_float_optional(
        drive->can_if,
        drive->node_id,
        IQ_KP_INDEX,
        IQ_KP_SUB,
        &drive->iq_kp_val,
        &drive->iq_kp_valid
    );

    (void)motion_drive_read_float_optional(
        drive->can_if,
        drive->node_id,
        IQ_KI_INDEX,
        IQ_KI_SUB,
        &drive->iq_ki_val,
        &drive->iq_ki_valid
    );

    (void)motion_drive_read_u32_optional(
        drive->can_if,
        drive->node_id,
        SPEED_KP_INDEX,
        SPEED_KP_SUB,
        &drive->speed_kp_val,
        &drive->speed_kp_valid
    );

    (void)motion_drive_read_u32_optional(
        drive->can_if,
        drive->node_id,
        SPEED_KI_INDEX,
        SPEED_KI_SUB,
        &drive->speed_ki_val,
        &drive->speed_ki_valid
    );

    (void)motion_drive_read_u32_optional(
        drive->can_if,
        drive->node_id,
        SPEED_KD_INDEX,
        SPEED_KD_SUB,
        &drive->speed_kd_val,
        &drive->speed_kd_valid
    );

    (void)motion_drive_read_u32_optional(
        drive->can_if,
        drive->node_id,
        SPEED_TC_INDEX,
        SPEED_TC_SUB,
        &drive->speed_tc_val,
        &drive->speed_tc_valid
    );

    (void)motion_drive_read_u16_optional(
        drive->can_if,
        drive->node_id,
        POS_KP_INDEX,
        POS_KP_SUB,
        &drive->position_kp_val,
        &drive->position_kp_valid
    );

    (void)motion_drive_read_u16_optional(
        drive->can_if,
        drive->node_id,
        POS_KI_INDEX,
        POS_KI_SUB,
        &drive->position_ki_val,
        &drive->position_ki_valid
    );

    (void)motion_drive_read_u16_optional(
        drive->can_if,
        drive->node_id,
        POS_KD_INDEX,
        POS_KD_SUB,
        &drive->position_kd_val,
        &drive->position_kd_valid
    );

    (void)motion_drive_read_u16_optional(
        drive->can_if,
        drive->node_id,
        POS_TC_INDEX,
        POS_TC_SUB,
        &drive->position_tc_val,
        &drive->position_tc_valid
    );

    (void)motion_drive_read_u32_optional(
        drive->can_if,
        drive->node_id,
        TARGET_POSITION_INDEX,
        TARGET_POSITION_SUB,
        &drive->target_position_val,
        &drive->target_position_valid
    );

    (void)motion_drive_read_u32_optional(
        drive->can_if,
        drive->node_id,
        TARGET_VELOCITY_INDEX,
        TARGET_VELOCITY_SUB,
        &drive->target_velocity_val,
        &drive->target_velocity_valid
    );

    (void)motion_drive_read_u32_optional(
        drive->can_if,
        drive->node_id,
        TARGET_ACCELERATION_INDEX,
        TARGET_ACCELERATION_SUB,
        &drive->target_acceleration_val,
        &drive->target_acceleration_valid
    );

    (void)motion_drive_read_u32_optional(
        drive->can_if,
        drive->node_id,
        TARGET_DECELERATION_INDEX,
        TARGET_DECELERATION_SUB,
        &drive->target_deceleration_val,
        &drive->target_deceleration_valid
    );

    (void)motion_drive_read_u16_optional(
        drive->can_if,
        drive->node_id,
        HEARTBEAT_INDEX,
        HEARTBEAT_SUB,
        &drive->heartbeat_time_ms,
        &drive->heartbeat_valid
    );

    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_set_heartbeat(
    motion_drive_t *drive,
    uint16_t producer_time_ms
)
{
    co_sdo_status_t sdo_status;

    if ((drive == NULL) || (drive->can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    sdo_status = co_sdo_write_u16(
        drive->can_if,
        drive->node_id,
        HEARTBEAT_INDEX,
        HEARTBEAT_SUB,
        producer_time_ms,
        MOTION_DRIVE_SDO_TIMEOUT_MS
    );

    if (sdo_status != CO_SDO_OK)
    {
        return MOTION_DRIVE_SDO_ERROR;
    }

    drive->heartbeat_time_ms = producer_time_ms;
    drive->heartbeat_valid = 1U;

    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_set_iq_gains(
    motion_drive_t *drive,
    float kp,
    float ki
)
{
    uint8_t pdo_data[8] = {0};
    uint32_t kp_raw;
    uint32_t ki_raw;
    can_hal_status_t can_status;

    if ((drive == NULL) || (drive->can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    /*
     * RPDO2 mapping:
     *
     * Byte 0-3: iq_kp, REAL32, object 0x2200
     * Byte 4-7: iq_ki, REAL32, object 0x2201
     *
     * CANopen uses little-endian byte order for multi-byte values.
     */
    kp_raw = motion_drive_float_to_u32(kp);
    ki_raw = motion_drive_float_to_u32(ki);

    motion_drive_pack_u32_le(pdo_data, 0U, kp_raw);
    motion_drive_pack_u32_le(pdo_data, 4U, ki_raw);

    can_status = co_pdo_send(
        drive->can_if,
        CO_CAN_ID_RPDO2(drive->node_id),
        pdo_data,
        8U
    );

    if (can_status != CAN_HAL_OK)
    {
        return MOTION_DRIVE_ERROR;
    }

    /*
     * Update local cached values after successful PDO transmit.
     */
    drive->iq_kp_val = kp;
    drive->iq_ki_val = ki;

    drive->iq_kp_valid = 1U;
    drive->iq_ki_valid = 1U;

    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_set_velocity_gains(
    motion_drive_t *drive,
    uint16_t kp,
    uint16_t ki,
    uint16_t kd,
    uint16_t tc
)
{
    uint8_t pdo_data[8] = {0};
    can_hal_status_t can_status;

    if ((drive == NULL) || (drive->can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    /*
     * RPDO3 mapping:
     *
     * Byte 0-1: speed_kp, uint16_t, object 0x2202
     * Byte 2-3: speed_ki, uint16_t, object 0x2203
     * Byte 4-5: speed_kd, uint16_t, object 0x2204
     * Byte 6-7: speed_Tc, uint16_t, object 0x2205
     *
     * CANopen uses little-endian byte order for multi-byte values.
     */
    motion_drive_pack_u16_le(pdo_data, 0U, kp);
    motion_drive_pack_u16_le(pdo_data, 2U, ki);
    motion_drive_pack_u16_le(pdo_data, 4U, kd);
    motion_drive_pack_u16_le(pdo_data, 6U, tc);

    can_status = co_pdo_send(
        drive->can_if,
        CO_CAN_ID_RPDO3(drive->node_id),
        pdo_data,
        8U
    );

    if (can_status != CAN_HAL_OK)
    {
        return MOTION_DRIVE_ERROR;
    }

    /*
     * Update local cached values after successful PDO transmit.
     */
    drive->speed_kp_val = kp;
    drive->speed_ki_val = ki;
    drive->speed_kd_val = kd;
    drive->speed_tc_val = tc;

    drive->speed_kp_valid = 1U;
    drive->speed_ki_valid = 1U;
    drive->speed_kd_valid = 1U;
    drive->speed_tc_valid = 1U;

    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_set_position_gains(
    motion_drive_t *drive,
    uint16_t kp,
    uint16_t ki,
    uint16_t kd,
    uint16_t tc
)
{
    uint8_t pdo_data[8] = {0};
    can_hal_status_t can_status;

    if ((drive == NULL) || (drive->can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    /*
     * RPDO4 mapping:
     *
     * Byte 0-1: pos_kp, uint16_t, object 0x2206
     * Byte 2-3: pos_ki, uint16_t, object 0x2207
     * Byte 4-5: pos_kd, uint16_t, object 0x2208
     * Byte 6-7: pos_Tc, uint16_t, object 0x2209
     */
    motion_drive_pack_u16_le(pdo_data, 0U, kp);
    motion_drive_pack_u16_le(pdo_data, 2U, ki);
    motion_drive_pack_u16_le(pdo_data, 4U, kd);
    motion_drive_pack_u16_le(pdo_data, 6U, tc);

    can_status = co_pdo_send(
        drive->can_if,
        CO_CAN_ID_RPDO4(drive->node_id),
        pdo_data,
        8U
    );

    if (can_status != CAN_HAL_OK)
    {
        return MOTION_DRIVE_ERROR;
    }

    drive->position_kp_val = kp;
    drive->position_ki_val = ki;
    drive->position_kd_val = kd;
    drive->position_tc_val = tc;

    drive->position_kp_valid = 1U;
    drive->position_ki_valid = 1U;
    drive->position_kd_valid = 1U;
    drive->position_tc_valid = 1U;

    return MOTION_DRIVE_OK;
}


motion_drive_status_t motion_drive_to_operational(
    motion_drive_t *drive
)
{
    can_hal_status_t can_status;

    if ((drive == NULL) || (drive->can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    can_status = co_nmt_start_node(
        drive->can_if,
        drive->node_id
    );

    if (can_status != CAN_HAL_OK)
    {
        return MOTION_DRIVE_ERROR;
    }

    /*
     * This is the requested/commanded state.
     * The real confirmed state should later come from heartbeat monitoring.
     */
    drive->nmt_state = MOTION_DRIVE_NMT_OPERATIONAL;

    return MOTION_DRIVE_OK;
}


motion_drive_status_t motion_drive_set_mode_of_operation(
    motion_drive_t *drive,
    uint8_t mode
)
{
	if ((drive == NULL) || (drive->can_if == NULL))
	{
		return MOTION_DRIVE_ERROR;
	}

	if (!motion_drive_is_valid_node_id(drive->node_id))
	{
		return MOTION_DRIVE_INVALID_NODE;
	}

	if(drive->nmt_state == MOTION_DRIVE_NMT_OPERATIONAL){


		uint8_t pdo_data[8] = {0};
		uint32_t target_iq_raw;
		can_hal_status_t can_status;
		co_sdo_status_t sdo_status;



		/*
		 * RPDO1 mapping:
		 *
		 * Byte 0-1: 0x6040 controlword, uint16_t
		 * Byte 2:   0x6060 modes_of_operation, uint8_t
		 * Byte 3:   dummy/reserved filler byte
		 * Byte 4-7: 0x2030 target_iq, REAL32
		 */
		target_iq_raw = motion_drive_float_to_u32(drive->target_iq_val);

		motion_drive_pack_u16_le(pdo_data, 0U, drive->controlword_val);
		pdo_data[2] = mode;
		pdo_data[3] = 0U;
		motion_drive_pack_u32_le(pdo_data, 4U, target_iq_raw);

		can_status = co_pdo_send(
			drive->can_if,
			CO_CAN_ID_RPDO1(drive->node_id),
			pdo_data,
			8U
		);

		if (can_status == CAN_HAL_OK)
		{
			drive->mode_of_operation_val = mode;
			drive->mode_of_operation_valid = 1U;

			return MOTION_DRIVE_OK;
		}
	}
    /*
     * Fallback: write mode through SDO.
     */
	co_sdo_status_t sdo_status = co_sdo_write_u8(
        drive->can_if,
        drive->node_id,
        MODES_OF_OPERATION_INDEX,
        MODES_OF_OPERATION_SUB,
        mode,
        MOTION_DRIVE_SDO_TIMEOUT_MS
    );

    if (sdo_status != CO_SDO_OK)
    {
        return MOTION_DRIVE_SDO_ERROR;
    }

    drive->mode_of_operation_val = mode;
    drive->mode_of_operation_valid = 1U;

    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_set_target_iq(
    motion_drive_t *drive,
    float iq
)
{
    uint8_t pdo_data[8] = {0};
    uint32_t target_iq_raw;
    can_hal_status_t can_status;

    if ((drive == NULL) || (drive->can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    if (drive->nmt_state != MOTION_DRIVE_NMT_OPERATIONAL)
    {
        return MOTION_DRIVE_ERROR;
    }

    /*
     * RPDO1 mapping:
     *
     * Byte 0-1: 0x6040 controlword, uint16_t
     * Byte 2:   0x6060 modes_of_operation, uint8_t
     * Byte 3:   dummy/reserved filler byte
     * Byte 4-7: 0x2030 target_iq, REAL32
     */
    target_iq_raw = motion_drive_float_to_u32(iq);

    motion_drive_pack_u16_le(pdo_data, 0U, drive->controlword_val);
    pdo_data[2] = drive->mode_of_operation_val;
    pdo_data[3] = 0U;
    motion_drive_pack_u32_le(pdo_data, 4U, target_iq_raw);

    can_status = co_pdo_send(
        drive->can_if,
        CO_CAN_ID_RPDO1(drive->node_id),
        pdo_data,
        8U
    );

    if (can_status != CAN_HAL_OK)
    {
        return MOTION_DRIVE_ERROR;
    }

    drive->target_iq_val = iq;
    drive->target_iq_valid = 1U;

    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_set_target_position_and_velocity(
    motion_drive_t *drive,
    float position,
    float velocity
)
{
    uint8_t pdo_data[8] = {0};
    uint32_t position_raw;
    uint32_t velocity_raw;
    can_hal_status_t can_status;

    if ((drive == NULL) || (drive->can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    if (drive->nmt_state != MOTION_DRIVE_NMT_OPERATIONAL)
    {
        return MOTION_DRIVE_ERROR;
    }

    /*
     * RPDO5 mapping:
     *
     * COB-ID: 0x600 + node_id
     *
     * Byte 0-3: 0x607A target_position, REAL32
     * Byte 4-7: 0x60FF target_velocity, REAL32
     */
    position_raw = motion_drive_float_to_u32(position);
    velocity_raw = motion_drive_float_to_u32(velocity);

    motion_drive_pack_u32_le(pdo_data, 0U, position_raw);
    motion_drive_pack_u32_le(pdo_data, 4U, velocity_raw);

    can_status = co_pdo_send(
        drive->can_if,
        CO_CAN_ID_RPDO5(drive->node_id),
        pdo_data,
        8U
    );

    if (can_status != CAN_HAL_OK)
    {
        return MOTION_DRIVE_ERROR;
    }

    drive->target_position_val = position;
    drive->target_velocity_val = velocity;

    drive->target_position_valid = 1U;
    drive->target_velocity_valid = 1U;

    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_set_velocity(
    motion_drive_t *drive,
    float velocity
)
{
    uint8_t pdo_data[8] = {0};
    uint32_t position_raw;
    uint32_t velocity_raw;
    can_hal_status_t can_status;

    if ((drive == NULL) || (drive->can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    if (drive->nmt_state != MOTION_DRIVE_NMT_OPERATIONAL)
    {
        return MOTION_DRIVE_ERROR;
    }

    /*
     * RPDO5 mapping:
     *
     * COB-ID: 0x600 + node_id
     *
     * Byte 0-3: 0x607A target_position, REAL32
     * Byte 4-7: 0x60FF target_velocity, REAL32
     *
     * This function only changes velocity.
     * Position is taken from the cached drive->target_position_val.
     */
    position_raw = motion_drive_float_to_u32(drive->target_position_val);
    velocity_raw = motion_drive_float_to_u32(velocity);

    motion_drive_pack_u32_le(pdo_data, 0U, position_raw);
    motion_drive_pack_u32_le(pdo_data, 4U, velocity_raw);

    can_status = co_pdo_send(
        drive->can_if,
        CO_CAN_ID_RPDO5(drive->node_id),
        pdo_data,
        8U
    );

    if (can_status != CAN_HAL_OK)
    {
        return MOTION_DRIVE_ERROR;
    }

    drive->target_velocity_val = velocity;
    drive->target_velocity_valid = 1U;

    return MOTION_DRIVE_OK;
}

motion_drive_status_t motion_drive_set_accel_decel(
    motion_drive_t *drive,
    uint32_t accel,
    uint32_t decel
)
{
    uint8_t pdo_data[8] = {0};
    uint32_t accel_raw;
    uint32_t decel_raw;
    can_hal_status_t can_status;
    co_sdo_status_t sdo_status;

    if ((drive == NULL) || (drive->can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    if ((accel < 0.0f) || (decel < 0.0f))
    {
        return MOTION_DRIVE_ERROR;
    }

    /*
     * The caller must pass already-scaled values.
     *
     * Example:
     * Desired acceleration = 0.1 rps^2
     * Drive expects value * 100
     * Caller passes 0.1f * 100.0f = 10.0f
     * This function converts 10.0f to raw uint32_t 10.
     */
    accel_raw = accel;
    decel_raw = decel;

    /*
     * If the node is operational, first try RPDO6.
     *
     * RPDO6 mapping:
     *
     * COB-ID: 0x700 + node_id
     *
     * Byte 0-3: 0x6083 target_acceleration, uint32_t raw
     * Byte 4-7: 0x6084 target_deceleration, uint32_t raw
     */
    if (drive->nmt_state == MOTION_DRIVE_NMT_OPERATIONAL)
    {
        motion_drive_pack_u32_le(pdo_data, 0U, accel_raw);
        motion_drive_pack_u32_le(pdo_data, 4U, decel_raw);

        can_status = co_pdo_send(
            drive->can_if,
            CO_CAN_ID_RPDO6(drive->node_id),
            pdo_data,
            8U
        );

        if (can_status == CAN_HAL_OK)
        {
            return MOTION_DRIVE_OK;
        }
    }

    /*
     * Fallback: write acceleration and deceleration through SDO.
     */
    sdo_status = co_sdo_write_u32(
        drive->can_if,
        drive->node_id,
        TARGET_ACCELERATION_INDEX,
        TARGET_ACCELERATION_SUB,
        accel_raw,
        MOTION_DRIVE_SDO_TIMEOUT_MS
    );

    if (sdo_status != CO_SDO_OK)
    {
        return MOTION_DRIVE_SDO_ERROR;
    }

    sdo_status = co_sdo_write_u32(
        drive->can_if,
        drive->node_id,
        TARGET_DECELERATION_INDEX,
        TARGET_DECELERATION_SUB,
        decel_raw,
        MOTION_DRIVE_SDO_TIMEOUT_MS
    );

    if (sdo_status != CO_SDO_OK)
    {
        return MOTION_DRIVE_SDO_ERROR;
    }

    return MOTION_DRIVE_OK;
}

static uint32_t motion_drive_unpack_u32_le(const uint8_t *data, uint8_t offset)
{
    return ((uint32_t)data[offset]) |
           ((uint32_t)data[offset + 1U] << 8) |
           ((uint32_t)data[offset + 2U] << 16) |
           ((uint32_t)data[offset + 3U] << 24);
}

static float motion_drive_unpack_float_le(const uint8_t *data, uint8_t offset)
{
    union
    {
        uint32_t u32;
        float f32;
    } converter;

    converter.u32 = motion_drive_unpack_u32_le(data, offset);
    return converter.f32;
}

static double motion_drive_unpack_double_le(const uint8_t *data, uint8_t offset)
{
    union
    {
        uint8_t bytes[8];
        double f64;
    } converter;

    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        converter.bytes[i] = data[offset + i];
    }

    return converter.f64;
}

//Example usage from main loop
//can_frame_t rx_frame;
//
//if (can0.receive(&rx_frame) == CAN_HAL_OK)
//{
//    (void)motion_drive_process_tpdo(&drive1, &rx_frame);
//    (void)motion_drive_process_tpdo(&drive2, &rx_frame);
//}
motion_drive_status_t motion_drive_process_tpdo(
    motion_drive_t *drive,
    const can_frame_t *frame
)
{
    if ((drive == NULL) || (frame == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    /*
     * TPDO1: Position feedback
     *
     * COB-ID: 0x180 + node_id
     * Mapping:
     * Byte 0-7: 0x6064 position_measured, REAL64
     */
    if (frame->id == CO_CAN_ID_TPDO1(drive->node_id))
    {
        if (frame->dlc != 8U)
        {
            return MOTION_DRIVE_ERROR;
        }

        drive->position_measured = motion_drive_unpack_double_le(frame->data, 0U);
        drive->position_measured_valid = 1U;
        drive->new_state_available = 1U;

        return MOTION_DRIVE_OK;
    }

    /*
     * TPDO2: Velocity feedback
     *
     * COB-ID: 0x280 + node_id
     * Mapping:
     * Byte 0-3: 0x606C velocity_measured, REAL32
     */
    if (frame->id == CO_CAN_ID_TPDO2(drive->node_id))
    {
//        if (frame->dlc < 4U)
//        {
//            return MOTION_DRIVE_ERROR;
//        }

        drive->velocity_measured = motion_drive_unpack_float_le(frame->data, 0U);
        drive->velocity_measured_valid = 1U;
        drive->new_state_available = 1U;

        return MOTION_DRIVE_OK;
    }

    /*
     * TPDO3: Current feedback
     *
     * COB-ID: 0x380 + node_id
     * Mapping:
     * Byte 0-3: 0x2032 iq_measured, REAL32
     * Byte 4-7: 0x2033 id_measured, REAL32
     */
    if (frame->id == CO_CAN_ID_TPDO3(drive->node_id))
    {
        if (frame->dlc != 8U)
        {
            return MOTION_DRIVE_ERROR;
        }

        drive->iq_measured = motion_drive_unpack_float_le(frame->data, 0U);
        drive->id_measured = motion_drive_unpack_float_le(frame->data, 4U);

        drive->iq_measured_valid = 1U;
        drive->id_measured_valid = 1U;
        drive->new_state_available = 1U;

        return MOTION_DRIVE_OK;
    }

    /*
     * TPDO4: Commanded motion values
     *
     * COB-ID: 0x480 + node_id
     * Mapping:
     * Byte 0-3: 0x6062 position_demand_value, REAL32
     * Byte 4-7: 0x606B velocity_demand_value, REAL32
     */
    if (frame->id == CO_CAN_ID_TPDO4(drive->node_id))
    {
        if (frame->dlc != 8U)
        {
            return MOTION_DRIVE_ERROR;
        }

        drive->position_demand_value = motion_drive_unpack_float_le(frame->data, 0U);
        drive->velocity_demand_value = motion_drive_unpack_float_le(frame->data, 4U);

        drive->position_demand_value_valid = 1U;
        drive->velocity_demand_value_valid = 1U;
        drive->new_state_available = 1U;

        return MOTION_DRIVE_OK;
    }

    /*
     * Frame is not a TPDO for this drive.
     */
    return MOTION_DRIVE_ERROR;
}

// Example usage
//can_frame_t rx_frame;
//
//if (can0.receive(&rx_frame) == CAN_HAL_OK)
//{
//    (void)motion_drive_process_heartbeat(&drive1, &rx_frame);
//    (void)motion_drive_process_tpdo(&drive1, &rx_frame);
//}
motion_drive_status_t motion_drive_process_heartbeat(
    motion_drive_t *drive,
    const can_frame_t *frame
)
{
    uint8_t state;

    if ((drive == NULL) || (frame == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    if (frame->id != CO_CAN_ID_HEARTBEAT(drive->node_id))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (frame->dlc < 1U)
    {
        return MOTION_DRIVE_ERROR;
    }

    state = frame->data[0];

//    drive->heartbeat_raw_state = state;
//    drive->heartbeat_state_valid = 1U;
//    drive->heartbeat_seen = 1U;

    switch (state)
    {
        case CO_NMT_STATE_BOOT_UP:
            drive->nmt_state = MOTION_DRIVE_NMT_INITIALISING;
            break;

        case CO_NMT_STATE_STOPPED:
            drive->nmt_state = MOTION_DRIVE_NMT_STOPPED;
            break;

        case CO_NMT_STATE_OPERATIONAL:
            drive->nmt_state = MOTION_DRIVE_NMT_OPERATIONAL;
            break;

        case CO_NMT_STATE_PRE_OPERATIONAL:
            drive->nmt_state = MOTION_DRIVE_NMT_PRE_OPERATIONAL;
            break;

        default:
            drive->nmt_state = MOTION_DRIVE_NMT_UNKNOWN;
            break;
    }

    return MOTION_DRIVE_OK;
}


// Usage
//motion_drive_set_controlword(&drive1, 0x000F);

motion_drive_status_t motion_drive_set_controlword(
    motion_drive_t *drive,
    uint16_t controlword
)
{
    uint8_t pdo_data[8] = {0};
    uint32_t target_iq_raw;
    can_hal_status_t can_status;
    co_sdo_status_t sdo_status;

    if ((drive == NULL) || (drive->can_if == NULL))
    {
        return MOTION_DRIVE_ERROR;
    }

    if (!motion_drive_is_valid_node_id(drive->node_id))
    {
        return MOTION_DRIVE_INVALID_NODE;
    }

    /*
     * If the node is operational, first try RPDO1.
     *
     * RPDO1 mapping:
     *
     * COB-ID: 0x200 + node_id
     *
     * Byte 0-1: 0x6040 controlword, uint16_t
     * Byte 2:   0x6060 modes_of_operation, uint8_t
     * Byte 3:   dummy/reserved filler byte
     * Byte 4-7: 0x2030 target_iq, REAL32
     */
    if (drive->nmt_state == MOTION_DRIVE_NMT_OPERATIONAL)
    {
        target_iq_raw = motion_drive_float_to_u32(drive->target_iq_val);

        motion_drive_pack_u16_le(pdo_data, 0U, controlword);
        pdo_data[2] = drive->mode_of_operation_val;
        pdo_data[3] = 0U;
        motion_drive_pack_u32_le(pdo_data, 4U, target_iq_raw);

        can_status = co_pdo_send(
            drive->can_if,
            CO_CAN_ID_RPDO1(drive->node_id),
            pdo_data,
            8U
        );

        if (can_status == CAN_HAL_OK)
        {
            drive->controlword_val = controlword;
            drive->controlword_valid = 1U;

            return MOTION_DRIVE_OK;
        }
    }

    /*
     * Fallback: write controlword through SDO.
     */
    sdo_status = co_sdo_write_u16(
        drive->can_if,
        drive->node_id,
        CONTROLWORD_INDEX,
        CONTROLWORD_SUB,
        controlword,
        MOTION_DRIVE_SDO_TIMEOUT_MS
    );

    if (sdo_status != CO_SDO_OK)
    {
        return MOTION_DRIVE_SDO_ERROR;
    }

    drive->controlword_val = controlword;
    drive->controlword_valid = 1U;

    return MOTION_DRIVE_OK;
}
