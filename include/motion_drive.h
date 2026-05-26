#ifndef MOTION_DRIVE_H
#define MOTION_DRIVE_H

#include <stdint.h>
#include "can_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTION_DRIVE_SDO_TIMEOUT_MS     100U

#define CONTROLWORD_INDEX               0x6040U
#define CONTROLWORD_SUB                 0x00U

#define MODES_OF_OPERATION_INDEX        0x6060U
#define MODES_OF_OPERATION_SUB          0x00U

#define IQ_KP_INDEX                     0x2200U
#define IQ_KP_SUB                       0x00U

#define IQ_KI_INDEX                     0x2201U
#define IQ_KI_SUB                       0x00U

#define SPEED_KP_INDEX                  0x2202U
#define SPEED_KP_SUB                    0x00U

#define SPEED_KI_INDEX                  0x2203U
#define SPEED_KI_SUB                    0x00U

#define SPEED_KD_INDEX                  0x2204U
#define SPEED_KD_SUB                    0x00U

#define SPEED_TC_INDEX                  0x2205U
#define SPEED_TC_SUB                    0x00U

#define POS_KP_INDEX                    0x2206U
#define POS_KP_SUB                      0x00U

#define POS_KI_INDEX                    0x2207U
#define POS_KI_SUB                      0x00U

#define POS_KD_INDEX                    0x2208U
#define POS_KD_SUB                      0x00U

#define POS_TC_INDEX                    0x2209U
#define POS_TC_SUB                      0x00U

#define TARGET_IQ_INDEX                 0x2030U
#define TARGET_IQ_SUB                   0x00U

#define TARGET_POSITION_INDEX           0x607AU
#define TARGET_POSITION_SUB             0x00U

#define TARGET_VELOCITY_INDEX           0x60FFU
#define TARGET_VELOCITY_SUB             0x00U

#define TARGET_ACCELERATION_INDEX       0x6083U
#define TARGET_ACCELERATION_SUB         0x00U

#define TARGET_DECELERATION_INDEX       0x6084U
#define TARGET_DECELERATION_SUB         0x00U

#define HEARTBEAT_INDEX                 0x1017U
#define HEARTBEAT_SUB                   0x00U


#define CO_CAN_ID_HEARTBEAT(node_id)    (0x700U + (uint32_t)(node_id))

#define CO_NMT_STATE_BOOT_UP            0x00U
#define CO_NMT_STATE_STOPPED            0x04U
#define CO_NMT_STATE_OPERATIONAL        0x05U
#define CO_NMT_STATE_PRE_OPERATIONAL    0x7FU

typedef enum
{
    MOTION_DRIVE_OK = 0,
    MOTION_DRIVE_ERROR,
    MOTION_DRIVE_INVALID_NODE,
    MOTION_DRIVE_SDO_ERROR
} motion_drive_status_t;


typedef enum
{
    MOTION_DRIVE_NMT_UNKNOWN = 0,
    MOTION_DRIVE_NMT_INITIALISING,
    MOTION_DRIVE_NMT_PRE_OPERATIONAL,
    MOTION_DRIVE_NMT_OPERATIONAL,
    MOTION_DRIVE_NMT_STOPPED
} motion_drive_nmt_state_t;

typedef struct
{
    const can_hal_t *can_if;
    uint8_t node_id;

    motion_drive_nmt_state_t nmt_state;

    uint16_t controlword_val;
    uint8_t mode_of_operation_val;

    float target_iq_val;

    float iq_kp_val;
    float iq_ki_val;

    uint32_t speed_kp_val;
    uint32_t speed_ki_val;
    uint32_t speed_kd_val;
    uint32_t speed_tc_val;

    uint16_t position_kp_val;
    uint16_t position_ki_val;
    uint16_t position_kd_val;
    uint16_t position_tc_val;

    uint32_t target_position_val;
    uint32_t target_velocity_val;
    uint32_t target_acceleration_val;
    uint32_t target_deceleration_val;

    uint16_t heartbeat_time_ms;

    uint8_t controlword_valid;
    uint8_t mode_of_operation_valid;

    uint8_t target_iq_valid;

    uint8_t iq_kp_valid;
    uint8_t iq_ki_valid;

    uint8_t speed_kp_valid;
    uint8_t speed_ki_valid;
    uint8_t speed_kd_valid;
    uint8_t speed_tc_valid;

    uint8_t position_kp_valid;
    uint8_t position_ki_valid;
    uint8_t position_kd_valid;
    uint8_t position_tc_valid;

    uint8_t target_position_valid;
    uint8_t target_velocity_valid;
    uint8_t target_acceleration_valid;
    uint8_t target_deceleration_valid;

    uint8_t heartbeat_valid;

    double position_measured;
    float velocity_measured;

    float iq_measured;
    float id_measured;

    float position_demand_value;
    float velocity_demand_value;

    uint8_t position_measured_valid;
    uint8_t velocity_measured_valid;

    uint8_t iq_measured_valid;
    uint8_t id_measured_valid;

    uint8_t position_demand_value_valid;
    uint8_t velocity_demand_value_valid;

    uint8_t new_state_available;

} motion_drive_t;




motion_drive_status_t motion_drive_init(
    motion_drive_t *drive,
    const can_hal_t *can_if,
    uint8_t node_id
);

motion_drive_status_t motion_drive_to_operational(
    motion_drive_t *drive
);

motion_drive_status_t motion_drive_refresh_cache(
    motion_drive_t *drive
);

motion_drive_status_t motion_drive_set_heartbeat(
    motion_drive_t *drive,
    uint16_t producer_time_ms
);

motion_drive_status_t motion_drive_set_iq_gains(
    motion_drive_t *drive,
    float kp,
    float ki
);

motion_drive_status_t motion_drive_set_velocity_gains(
    motion_drive_t *drive,
    uint16_t kp,
    uint16_t ki,
    uint16_t kd,
    uint16_t tc
);

motion_drive_status_t motion_drive_set_position_gains(
    motion_drive_t *drive,
    uint16_t kp,
    uint16_t ki,
    uint16_t kd,
    uint16_t tc
);

motion_drive_status_t motion_drive_set_mode_of_operation(
    motion_drive_t *drive,
    uint8_t mode
);


motion_drive_status_t motion_drive_set_target_iq(
    motion_drive_t *drive,
    float iq
);

motion_drive_status_t motion_drive_set_target_position_and_velocity(
    motion_drive_t *drive,
    float position,
    float velocity
);

motion_drive_status_t motion_drive_set_velocity(
    motion_drive_t *drive,
    float velocity
);

motion_drive_status_t motion_drive_set_accel_decel(
    motion_drive_t *drive,
    uint32_t accel,
    uint32_t decel
);


motion_drive_status_t motion_drive_process_tpdo(
    motion_drive_t *drive,
    const can_frame_t *frame
);

motion_drive_status_t motion_drive_process_heartbeat(
    motion_drive_t *drive,
    const can_frame_t *frame
);

motion_drive_status_t motion_drive_set_controlword(
    motion_drive_t *drive,
    uint16_t controlword
);


#ifdef __cplusplus
}
#endif

#endif
