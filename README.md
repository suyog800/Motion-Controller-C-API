# Motion CANopen Master C Library

Portable C CANopen master library for an FOC motor controller developed by Medplus India Pvt Ltd. This C library is based on the Python CANopen helper workflow used for the motion controller and follows the same high-level control model: configure/read values through SDO during setup, switch the node to Operational state, then use PDOs for real-time command and feedback.

The current C API is designed around one `motion_drive_t` object per CANopen drive/node. Each drive object stores the CAN interface pointer, node ID, NMT state, cached command values, and real-time feedback values received from TPDOs.

---

## 1. Design goals

The library is designed for embedded firmware and portable CANopen motion control.

Main goals:

1. Keep CANopen logic independent of STM32 HAL, Linux SocketCAN, or any other CAN driver.
2. Use one C object, `motion_drive_t`, to represent one motor drive node.
3. Use SDOs for setup/configuration and PDOs for real-time communication.
4. Allow the same CANopen and motion-control code to be reused on STM32, Linux SocketCAN, ESP32 TWAI, MCP2515, or any other CAN backend by changing only the CAN HAL implementation.
5. Keep the API close to the existing Python API, but make it explicit and deterministic for embedded C.

---

## 2. Project layout

Recommended structure:

```text
motion_canopen_master/
├── include/
│   ├── can_hal.h
│   ├── co_nmt.h
│   ├── co_sdo.h
│   ├── co_pdo.h
│   ├── co_emcy.h
│   ├── co_heartbeat.h
│   ├── co_object_dictionary.h
│   ├── motion_drive.h
│   └── motion_drive_helper.h
│
├── src/
│   ├── co_nmt.c
│   ├── co_sdo.c
│   ├── co_pdo.c
│   ├── co_emcy.c
│   ├── co_heartbeat.c
│   ├── motion_drive.c
│   └── motion_drive_helper.c
│
├── port/
│   ├── stm32/
│   │   ├── can_hal_stm32.c
│   │   ├── can_hal_stm32.h
│   │   └── co_sdo_time_stm32.c
│   │
│   └── socketcan/
│       ├── can_hal_socketcan.c
│       └── can_hal_socketcan.h
│
├── examples/
│   ├── stm32_velocity_control/
│   ├── stm32_position_control/
│   └── linux_socketcan_test/
│
└── README.md
```

File responsibilities:

| File | Responsibility |
|---|---|
| `can_hal.h` | Portable CAN frame and CAN interface definition. No MCU-specific code. |
| `can_hal_stm32.c/.h` | STM32 FDCAN implementation of the portable CAN HAL. |
| `co_nmt.c/.h` | CANopen NMT commands such as Start Remote Node. |
| `co_sdo.c/.h` | Blocking expedited SDO client for setup/configuration. |
| `co_pdo.c/.h` | PDO send/receive primitives and COB-ID definitions. |
| `motion_drive.c/.h` | High-level drive API: set gains, mode, controlword, targets, feedback processing. |
| `motion_drive_helper.c/.h` | Helper wrappers for SDO reads and type conversion. |

---

## 3. Portability model

The key abstraction is `can_hal_t`.

```c
typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

typedef enum
{
    CAN_HAL_OK = 0,
    CAN_HAL_ERROR,
    CAN_HAL_TIMEOUT,
    CAN_HAL_RX_EMPTY
} can_hal_status_t;

typedef can_hal_status_t (*can_send_fn_t)(const can_frame_t *frame);
typedef can_hal_status_t (*can_receive_fn_t)(can_frame_t *frame);

typedef struct
{
    can_send_fn_t send;
    can_receive_fn_t receive;
} can_hal_t;
```

All higher layers call only:

```c
can_if->send(&frame);
can_if->receive(&frame);
```

They do not know whether the backend is STM32 FDCAN, SocketCAN, or any other controller.

### Why this is portable

The upper layers use only standard C types and the `can_hal_t` interface. They do not include STM32 headers such as:

```c
stm32g4xx_hal.h
FDCAN_HandleTypeDef
FDCAN_TxHeaderTypeDef
FDCAN_RxHeaderTypeDef
```

Only the port layer knows about the target platform. For STM32, `can_hal_stm32.c` converts `can_frame_t` into STM32 `FDCAN_TxHeaderTypeDef` and calls `HAL_FDCAN_AddMessageToTxFifoQ()`. For SocketCAN, `can_hal_socketcan.c` would convert `can_frame_t` into Linux `struct can_frame` and call `write()`/`read()`.

The result is:

```text
motion_drive.c
co_sdo.c
co_pdo.c
co_nmt.c
    depend only on can_hal.h

port/stm32/can_hal_stm32.c
    depends on STM32 HAL

port/socketcan/can_hal_socketcan.c
    depends on Linux SocketCAN
```

---

## 4. CANopen communication model

The library uses the following communication strategy.

### 4.1 Setup phase

During pre-operational state, use SDOs to read or configure object dictionary values:

- Controlword: `0x6040:00`
- Modes of operation: `0x6060:00`
- Producer heartbeat time: `0x1017:00`
- Initial gain values
- Initial command values

### 4.2 Operational phase

After sending NMT Start Remote Node, the drive enters Operational state. Real-time commands are sent by PDO:

| PDO | COB-ID | Purpose |
|---|---:|---|
| RPDO1 | `0x200 + node_id` | Controlword, mode, target Iq |
| RPDO2 | `0x300 + node_id` | Current-loop gains |
| RPDO3 | `0x400 + node_id` | Velocity-loop gains |
| RPDO4 | `0x500 + node_id` | Position-loop gains |
| RPDO5 | `0x600 + node_id` | Target position and target velocity |
| RPDO6 | `0x700 + node_id` | Acceleration and deceleration |

Feedback is received through TPDOs:

| PDO | COB-ID | Purpose |
|---|---:|---|
| TPDO1 | `0x180 + node_id` | Position feedback |
| TPDO2 | `0x280 + node_id` | Velocity feedback |
| TPDO3 | `0x380 + node_id` | Current feedback |
| TPDO4 | `0x480 + node_id` | Commanded internal motion values |

> **Important COB-ID warning:** `0x700 + node_id` is the standard CANopen heartbeat COB-ID. If RPDO6 also uses `0x700 + node_id`, it conflicts with heartbeat. Either remap RPDO6 or disable/remap heartbeat in the drive.

---

## 5. Object dictionary constants

The C API uses the following object indexes.

| Object | Subindex | Name | Type / meaning |
|---:|---:|---|---|
| `0x6040` | `0x00` | `controlword` | `uint16_t` |
| `0x6060` | `0x00` | `modes_of_operation` | `uint8_t` |
| `0x2030` | `0x00` | `target_iq` | `float` / REAL32 |
| `0x2200` | `0x00` | `iq_kp` | `float` / REAL32 |
| `0x2201` | `0x00` | `iq_ki` | `float` / REAL32 |
| `0x2202` | `0x00` | `speed_kp` | `uint16_t` |
| `0x2203` | `0x00` | `speed_ki` | `uint16_t` |
| `0x2204` | `0x00` | `speed_kd` | `uint16_t` |
| `0x2205` | `0x00` | `speed_tc` | `uint16_t` |
| `0x2206` | `0x00` | `pos_kp` | `uint16_t` |
| `0x2207` | `0x00` | `pos_ki` | `uint16_t` |
| `0x2208` | `0x00` | `pos_kd` | `uint16_t` |
| `0x2209` | `0x00` | `pos_tc` | `uint16_t` |
| `0x607A` | `0x00` | `target_position` | `float` / REAL32 in this API design |
| `0x60FF` | `0x00` | `target_velocity` | `float` / REAL32 in this API design |
| `0x6083` | `0x00` | `target_acceleration` | raw/scaled `uint32_t` payload |
| `0x6084` | `0x00` | `target_deceleration` | raw/scaled `uint32_t` payload |
| `0x1017` | `0x00` | producer heartbeat time | `uint16_t`, milliseconds |

---

## 6. `motion_drive_t`

`motion_drive_t` represents one CANopen drive/node.

```c
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

    float target_position_val;
    float target_velocity_val;

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
```

### Notes

- `can_if` points to the CAN bus interface used by this drive.
- `node_id` is the CANopen node ID, usually `1` to `127`.
- `nmt_state` stores the last commanded or heartbeat-confirmed NMT state.
- `*_val` fields store cached command/configuration values.
- `*_valid` fields indicate whether the cached value is known.
- Feedback fields are updated by `motion_drive_process_tpdo()`.
- `new_state_available` is set when new TPDO feedback has been decoded.

---

## 7. Status codes

```c
typedef enum
{
    MOTION_DRIVE_OK = 0,
    MOTION_DRIVE_ERROR,
    MOTION_DRIVE_INVALID_NODE,
    MOTION_DRIVE_SDO_ERROR
} motion_drive_status_t;
```

| Status | Meaning |
|---|---|
| `MOTION_DRIVE_OK` | Operation completed successfully. |
| `MOTION_DRIVE_ERROR` | Generic error, invalid pointer, CAN transmit failure, invalid state, or invalid argument. |
| `MOTION_DRIVE_INVALID_NODE` | Node ID is outside valid CANopen node range. |
| `MOTION_DRIVE_SDO_ERROR` | SDO transaction failed, timed out, or returned abort. |

---

## 8. API reference

### 8.1 `motion_drive_init`

```c
motion_drive_status_t motion_drive_init(
    motion_drive_t *drive,
    const can_hal_t *can_if,
    uint8_t node_id
);
```

Initializes one drive object.

Parameters:

| Parameter | Description |
|---|---|
| `drive` | Pointer to the drive object to initialize. |
| `can_if` | Pointer to the portable CAN interface. |
| `node_id` | CANopen node ID of the drive. |

Behavior:

- Clears the drive object.
- Stores `can_if` and `node_id`.
- Initializes NMT state to `MOTION_DRIVE_NMT_UNKNOWN`.

Example:

```c
motion_drive_t drive1;
motion_drive_init(&drive1, &can0, 1U);
```

---

### 8.2 `motion_drive_refresh_cache`

```c
motion_drive_status_t motion_drive_refresh_cache(
    motion_drive_t *drive
);
```

Reads initial object dictionary values through SDO and stores them in the drive object.

Typical values read:

- `0x6040:00` controlword
- `0x6060:00` mode of operation
- `0x2030:00` target Iq
- `0x2200:00`, `0x2201:00` current loop gains
- `0x2202:00` to `0x2205:00` velocity loop gains
- `0x2206:00` to `0x2209:00` position loop gains
- `0x1017:00` heartbeat producer time

This function is similar to the Python implementation where each SDO read is attempted and failures are ignored using `try/except/pass`.

Recommended usage:

```c
motion_drive_refresh_cache(&drive1);
```

Call this during setup/pre-operational state.

---

### 8.3 `motion_drive_to_operational`

```c
motion_drive_status_t motion_drive_to_operational(
    motion_drive_t *drive
);
```

Sends CANopen NMT Start Remote Node to the drive.

CAN frame:

| Field | Value |
|---|---|
| CAN ID | `0x000` |
| DLC | `2` |
| Data[0] | `0x01` Start Remote Node |
| Data[1] | `node_id` |

Behavior:

- Sends NMT command through `co_nmt_start_node()`.
- Sets local `drive->nmt_state` to `MOTION_DRIVE_NMT_OPERATIONAL` if transmission succeeds.

Important: this local state means the command was sent. For confirmed state, process heartbeat using `motion_drive_process_heartbeat()`.

Example:

```c
motion_drive_to_operational(&drive1);
```

---

### 8.4 `motion_drive_set_heartbeat`

```c
motion_drive_status_t motion_drive_set_heartbeat(
    motion_drive_t *drive,
    uint16_t producer_time_ms
);
```

Writes the producer heartbeat time using SDO.

Object:

| Index | Subindex | Type | Description |
|---:|---:|---|---|
| `0x1017` | `0x00` | `uint16_t` | Producer heartbeat time in milliseconds |

Example:

```c
motion_drive_set_heartbeat(&drive1, 1000U);
```

---

### 8.5 `motion_drive_set_controlword`

```c
motion_drive_status_t motion_drive_set_controlword(
    motion_drive_t *drive,
    uint16_t controlword
);
```

Sets the drive controlword.

Behavior:

- If `drive->nmt_state == MOTION_DRIVE_NMT_OPERATIONAL`, first tries RPDO1.
- If PDO transmit fails or the node is not operational, falls back to SDO write to `0x6040:00`.
- Updates `drive->controlword_val` after success.

RPDO1 mapping:

| Byte offset | Object | Type |
|---:|---|---|
| 0 | `0x6040` controlword | `uint16_t` |
| 2 | `0x6060` modes_of_operation | `uint8_t` |
| 3 | dummy filler | `uint8_t` |
| 4 | `0x2030` target_iq | REAL32 |

Example:

```c
motion_drive_set_controlword(&drive1, 0x0006U);  // Shutdown
motion_drive_set_controlword(&drive1, 0x0007U);  // Switch on
motion_drive_set_controlword(&drive1, 0x000FU);  // Enable operation
```

---

### 8.6 `motion_drive_set_mode_of_operation`

```c
motion_drive_status_t motion_drive_set_mode_of_operation(
    motion_drive_t *drive,
    uint8_t mode
);
```

Sets the mode of operation.

Behavior:

- If operational, first tries RPDO1.
- If PDO transmit fails or the node is not operational, falls back to SDO write to `0x6060:00`.
- Updates `drive->mode_of_operation_val` after success.

Example:

```c
motion_drive_set_mode_of_operation(&drive1, 3U);  // Example: profile velocity mode
```

---

### 8.7 `motion_drive_set_target_iq`

```c
motion_drive_status_t motion_drive_set_target_iq(
    motion_drive_t *drive,
    float iq
);
```

Sends target q-axis current using PDO only.

RPDO1 mapping:

| Byte offset | Object | Type |
|---:|---|---|
| 0 | `0x6040` controlword | `uint16_t` |
| 2 | `0x6060` mode | `uint8_t` |
| 3 | dummy filler | `uint8_t` |
| 4 | `0x2030` target_iq | REAL32 |

Behavior:

- Requires `drive->nmt_state == MOTION_DRIVE_NMT_OPERATIONAL`.
- Sends RPDO1 to `0x200 + node_id`.
- Updates `drive->target_iq_val` after successful PDO transmit.

Example:

```c
motion_drive_set_target_iq(&drive1, 1.25f);
```

---

### 8.8 `motion_drive_set_iq_gains`

```c
motion_drive_status_t motion_drive_set_iq_gains(
    motion_drive_t *drive,
    float kp,
    float ki
);
```

Sends current-loop PI gains using RPDO2.

RPDO2:

| Field | Value |
|---|---|
| COB-ID | `0x300 + node_id` |
| DLC | `8` |

Payload:

| Byte offset | Object | Type |
|---:|---|---|
| 0 | `0x2200` iq_kp | REAL32 |
| 4 | `0x2201` iq_ki | REAL32 |

Example:

```c
motion_drive_set_iq_gains(&drive1, 0.25f, 0.01f);
```

---

### 8.9 `motion_drive_set_velocity_gains`

```c
motion_drive_status_t motion_drive_set_velocity_gains(
    motion_drive_t *drive,
    uint16_t kp,
    uint16_t ki,
    uint16_t kd,
    uint16_t tc
);
```

Sends speed/velocity-loop gains using RPDO3.

RPDO3:

| Field | Value |
|---|---|
| COB-ID | `0x400 + node_id` |
| DLC | `8` |

Payload:

| Byte offset | Object | Type |
|---:|---|---|
| 0 | `0x2202` speed_kp | `uint16_t` |
| 2 | `0x2203` speed_ki | `uint16_t` |
| 4 | `0x2204` speed_kd | `uint16_t` |
| 6 | `0x2205` speed_tc | `uint16_t` |

Example:

```c
motion_drive_set_velocity_gains(&drive1, 100U, 20U, 5U, 10U);
```

---

### 8.10 `motion_drive_set_position_gains`

```c
motion_drive_status_t motion_drive_set_position_gains(
    motion_drive_t *drive,
    uint16_t kp,
    uint16_t ki,
    uint16_t kd,
    uint16_t tc
);
```

Sends position-loop gains using RPDO4.

RPDO4:

| Field | Value |
|---|---|
| COB-ID | `0x500 + node_id` |
| DLC | `8` |

Payload:

| Byte offset | Object | Type |
|---:|---|---|
| 0 | `0x2206` pos_kp | `uint16_t` |
| 2 | `0x2207` pos_ki | `uint16_t` |
| 4 | `0x2208` pos_kd | `uint16_t` |
| 6 | `0x2209` pos_tc | `uint16_t` |

Example:

```c
motion_drive_set_position_gains(&drive1, 200U, 10U, 5U, 20U);
```

---

### 8.11 `motion_drive_set_target_position_and_velocity`

```c
motion_drive_status_t motion_drive_set_target_position_and_velocity(
    motion_drive_t *drive,
    float position,
    float velocity
);
```

Sends target position and target velocity using RPDO5.

RPDO5:

| Field | Value |
|---|---|
| COB-ID | `0x600 + node_id` |
| DLC | `8` |

Payload:

| Byte offset | Object | Type |
|---:|---|---|
| 0 | `0x607A` target_position | REAL32 |
| 4 | `0x60FF` target_velocity | REAL32 |

Example:

```c
motion_drive_set_target_position_and_velocity(&drive1, 10.0f, 2.0f);
```

Warning: `0x600 + node_id` is also the standard CANopen SDO request range. If RPDO5 uses this COB-ID, do not run SDO transfers to the same node while using RPDO5 unless the drive’s SDO or PDO COB-IDs are remapped to avoid conflict.

---

### 8.12 `motion_drive_set_velocity`

```c
motion_drive_status_t motion_drive_set_velocity(
    motion_drive_t *drive,
    float velocity
);
```

Sends a new target velocity using RPDO5 while keeping the cached target position.

Payload:

| Byte offset | Value |
|---:|---|
| 0 | cached `drive->target_position_val` |
| 4 | new `velocity` |

Example:

```c
motion_drive_set_velocity(&drive1, 5.0f);
```

Before calling this function, make sure `drive->target_position_val` is initialized by either:

```c
motion_drive_refresh_cache(&drive1);
```

or:

```c
motion_drive_set_target_position_and_velocity(&drive1, position, velocity);
```

---

### 8.13 `motion_drive_set_accel_decel`

Recommended latest API:

```c
motion_drive_status_t motion_drive_set_accel_decel(
    motion_drive_t *drive,
    float accel,
    float decel
);
```

This function treats `accel` and `decel` as already-scaled values. It converts them to `uint32_t` raw values internally.

Example: if the manual says to multiply rps² by `100`, and the desired acceleration is `0.1 rps²`, the caller should pass:

```c
motion_drive_set_accel_decel(&drive1, 0.1f * 100.0f, 0.1f * 100.0f);
```

Internally, the function converts `10.0f` to raw `uint32_t` value `10`.

Behavior:

- If operational, first tries RPDO6.
- If PDO transmit fails or the node is not operational, falls back to SDO writes to `0x6083:00` and `0x6084:00`.
- Does not cache acceleration/deceleration values.

RPDO6:

| Field | Value |
|---|---|
| COB-ID | `0x700 + node_id` |
| DLC | `8` |

Payload:

| Byte offset | Object | Type |
|---:|---|---|
| 0 | `0x6083` target_acceleration | raw `uint32_t` |
| 4 | `0x6084` target_deceleration | raw `uint32_t` |

Warning: `0x700 + node_id` conflicts with standard CANopen heartbeat.

---

### 8.14 `motion_drive_process_tpdo`

```c
motion_drive_status_t motion_drive_process_tpdo(
    motion_drive_t *drive,
    const can_frame_t *frame
);
```

Processes one received CAN frame and updates drive feedback variables if the frame is TPDO1, TPDO2, TPDO3, or TPDO4 for the drive’s node ID.

TPDO mappings:

| TPDO | COB-ID | Payload | Updated variables |
|---|---:|---|---|
| TPDO1 | `0x180 + node_id` | `0x6064` position_measured REAL64 | `position_measured` |
| TPDO2 | `0x280 + node_id` | `0x606C` velocity_measured REAL32 | `velocity_measured` |
| TPDO3 | `0x380 + node_id` | `0x2032` iq_measured REAL32, `0x2033` id_measured REAL32 | `iq_measured`, `id_measured` |
| TPDO4 | `0x480 + node_id` | `0x6062` position_demand_value REAL32, `0x606B` velocity_demand_value REAL32 | `position_demand_value`, `velocity_demand_value` |

Example:

```c
can_frame_t rx_frame;

if (can0.receive(&rx_frame) == CAN_HAL_OK)
{
    (void)motion_drive_process_tpdo(&drive1, &rx_frame);
}
```

When a TPDO is processed successfully, the function sets:

```c
drive->new_state_available = 1U;
```

---

### 8.15 `motion_drive_process_heartbeat`

```c
motion_drive_status_t motion_drive_process_heartbeat(
    motion_drive_t *drive,
    const can_frame_t *frame
);
```

Processes one heartbeat frame and updates `drive->nmt_state`.

Heartbeat frame:

| Field | Value |
|---|---|
| COB-ID | `0x700 + node_id` |
| DLC | `1` |
| Data[0] | NMT state |

NMT state values:

| Raw value | Meaning | Internal state |
|---:|---|---|
| `0x00` | Boot-up | `MOTION_DRIVE_NMT_INITIALISING` |
| `0x04` | Stopped | `MOTION_DRIVE_NMT_STOPPED` |
| `0x05` | Operational | `MOTION_DRIVE_NMT_OPERATIONAL` |
| `0x7F` | Pre-operational | `MOTION_DRIVE_NMT_PRE_OPERATIONAL` |

Example:

```c
can_frame_t rx_frame;

if (can0.receive(&rx_frame) == CAN_HAL_OK)
{
    (void)motion_drive_process_heartbeat(&drive1, &rx_frame);
}
```

---

## 9. STM32 usage example

```c
#include "main.h"
#include "can_hal_stm32.h"
#include "motion_drive.h"

extern FDCAN_HandleTypeDef hfdcan1;

#define DRIVE1_NODE_ID  1U

static motion_drive_t drive1;

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_FDCAN1_Init();

    if (can_hal_stm32_init(&hfdcan1) != CAN_HAL_OK)
    {
        Error_Handler();
    }

    if (motion_drive_init(&drive1, &can0, DRIVE1_NODE_ID) != MOTION_DRIVE_OK)
    {
        Error_Handler();
    }

    (void)motion_drive_refresh_cache(&drive1);

    if (motion_drive_set_heartbeat(&drive1, 1000U) != MOTION_DRIVE_OK)
    {
        Error_Handler();
    }

    if (motion_drive_to_operational(&drive1) != MOTION_DRIVE_OK)
    {
        Error_Handler();
    }

    HAL_Delay(100);

    (void)motion_drive_set_mode_of_operation(&drive1, 3U);
    (void)motion_drive_set_controlword(&drive1, 0x000FU);
    (void)motion_drive_set_iq_gains(&drive1, 0.25f, 0.01f);
    (void)motion_drive_set_velocity_gains(&drive1, 100U, 20U, 5U, 10U);
    (void)motion_drive_set_position_gains(&drive1, 200U, 10U, 5U, 20U);
    (void)motion_drive_set_target_position_and_velocity(&drive1, 10.0f, 2.0f);

    while (1)
    {
        can_frame_t rx_frame;
        can_hal_status_t status;

        do
        {
            status = can0.receive(&rx_frame);

            if (status == CAN_HAL_OK)
            {
                (void)motion_drive_process_heartbeat(&drive1, &rx_frame);
                (void)motion_drive_process_tpdo(&drive1, &rx_frame);
            }
        } while (status == CAN_HAL_OK);

        if (drive1.new_state_available != 0U)
        {
            drive1.new_state_available = 0U;

            if (drive1.position_measured_valid != 0U)
            {
                double pos = drive1.position_measured;
                (void)pos;
            }

            if (drive1.velocity_measured_valid != 0U)
            {
                float vel = drive1.velocity_measured;
                (void)vel;
            }
        }
    }
}
```

---

## 10. Blocking SDO limitation

The current simple SDO client directly calls:

```c
can_if->receive(&rx_frame);
```

while waiting for an SDO response. If a PDO, heartbeat, or EMCY frame arrives during that wait, the blocking SDO function may consume the frame and ignore it because it is not the expected SDO response.

This is acceptable during early bring-up if SDO is used mainly in setup/pre-operational state.

Recommended production architecture:

```text
CAN RX interrupt / polling
    ↓
central RX queue
    ↓
CANopen dispatcher
    ├── SDO response handler
    ├── TPDO handler
    ├── heartbeat handler
    └── EMCY handler
```

In production, the SDO client should wait for an SDO response delivered by the dispatcher instead of directly consuming all CAN RX frames.

---

## 11. Porting to another microcontroller

To port the library to another MCU, only implement the CAN HAL functions for that platform.

### 11.1 Required functions

You need to provide functions matching:

```c
can_hal_status_t your_can_send(const can_frame_t *frame);
can_hal_status_t your_can_receive(can_frame_t *frame);
```

Then create a `can_hal_t` object:

```c
can_hal_t can0 =
{
    .send = your_can_send,
    .receive = your_can_receive
};
```

### 11.2 Example port structure

```text
port/my_mcu/
├── can_hal_my_mcu.c
└── can_hal_my_mcu.h
```

### 11.3 Implementation rules

Your port implementation must:

1. Translate `can_frame_t.id` to the platform CAN identifier field.
2. Translate `can_frame_t.dlc` to the platform DLC encoding.
3. Copy up to 8 data bytes.
4. Return `CAN_HAL_RX_EMPTY` if no receive frame is available.
5. Return `CAN_HAL_OK` only if transmit/receive succeeded.
6. Keep all MCU-specific includes inside the port folder.

### 11.4 Example pseudocode

```c
can_hal_status_t can_hal_my_mcu_send(const can_frame_t *frame)
{
    if (frame == NULL)
    {
        return CAN_HAL_ERROR;
    }

    if (frame->id > 0x7FFU || frame->dlc > 8U)
    {
        return CAN_HAL_ERROR;
    }

    /* Convert can_frame_t to your MCU CAN Tx structure. */
    /* Start transmission. */

    return CAN_HAL_OK;
}

can_hal_status_t can_hal_my_mcu_receive(can_frame_t *frame)
{
    if (frame == NULL)
    {
        return CAN_HAL_ERROR;
    }

    if (/* no RX frame pending */)
    {
        return CAN_HAL_RX_EMPTY;
    }

    /* Read hardware frame. */
    /* Convert hardware RX frame to can_frame_t. */

    return CAN_HAL_OK;
}
```

---

## 12. Porting to Linux SocketCAN

For Linux SocketCAN, implement the same interface using a CAN socket.

Typical backend flow:

1. Open socket with `socket(PF_CAN, SOCK_RAW, CAN_RAW)`.
2. Bind to interface such as `can0`.
3. Convert `can_frame_t` to Linux `struct can_frame`.
4. Use `write()` to transmit.
5. Use non-blocking `read()` to receive.
6. Return `CAN_HAL_RX_EMPTY` when `read()` returns `EAGAIN` or `EWOULDBLOCK`.

Example concept:

```c
can_hal_status_t can_hal_socketcan_send(const can_frame_t *frame)
{
    struct can_frame linux_frame;

    linux_frame.can_id = frame->id;
    linux_frame.can_dlc = frame->dlc;
    memcpy(linux_frame.data, frame->data, frame->dlc);

    if (write(socket_fd, &linux_frame, sizeof(linux_frame)) != sizeof(linux_frame))
    {
        return CAN_HAL_ERROR;
    }

    return CAN_HAL_OK;
}
```

The higher-level code remains unchanged:

```c
motion_drive_init(&drive1, &can0, 1U);
motion_drive_to_operational(&drive1);
motion_drive_set_velocity(&drive1, 5.0f);
```

---

## 13. Recommended build integration for STM32CubeIDE

Add these include paths:

```text
Drivers/MotionController_CANOpen_Master/include
Drivers/MotionController_CANOpen_Master/port/stm32
```

Add these source files to the build:

```text
src/co_nmt.c
src/co_sdo.c
src/co_pdo.c
src/motion_drive.c
src/motion_drive_helper.c
port/stm32/can_hal_stm32.c
port/stm32/co_sdo_time_stm32.c
```

`co_sdo_time_stm32.c` should provide:

```c
#include "co_sdo.h"
#include "stm32g4xx_hal.h"

uint32_t co_sdo_get_time_ms(void)
{
    return HAL_GetTick();
}
```

---

## 14. Recommended usage sequence

A typical startup sequence:

```text
1. Initialize MCU and FDCAN peripheral.
2. Initialize CAN HAL with hfdcan1.
3. Initialize one motion_drive_t per node.
4. Refresh cache through SDO.
5. Configure heartbeat.
6. Send NMT start remote node.
7. Set mode of operation.
8. Set controlword enable sequence.
9. Send real-time PDO commands.
10. Continuously process heartbeat and TPDO feedback frames.
```

Example:

```c
can_hal_stm32_init(&hfdcan1);

motion_drive_init(&drive1, &can0, 1U);
motion_drive_refresh_cache(&drive1);
motion_drive_set_heartbeat(&drive1, 1000U);
motion_drive_to_operational(&drive1);

motion_drive_set_mode_of_operation(&drive1, 3U);
motion_drive_set_controlword(&drive1, 0x000FU);
motion_drive_set_velocity(&drive1, 5.0f);
```

---

## 15. Notes and known issues

1. **RPDO5 COB-ID conflict:** `0x600 + node_id` is normally SDO request. If your drive uses it for RPDO5, avoid simultaneous SDO use or remap one of them.
2. **RPDO6 COB-ID conflict:** `0x700 + node_id` is normally heartbeat. If your drive uses it for RPDO6, heartbeat must be disabled/remapped or RPDO6 must be remapped.
3. **Blocking SDO can consume unrelated frames:** Use SDO mainly in setup, or implement a central dispatcher for production.
4. **REAL32/REAL64 endian assumption:** Packing/unpacking assumes little-endian IEEE-754 format. STM32 is little-endian, so this is valid for the initial STM32 target.
5. **NMT state tracking:** `motion_drive_to_operational()` sets local state after sending the command. Actual confirmed state should come from heartbeat.
6. **Acceleration/deceleration scaling:** The API expects the caller to pass already-scaled values. The function only converts the passed float to raw `uint32_t`.

---

## 16. Summary

This C library gives a small, portable CANopen master layer for the motion controller. The application works with `motion_drive_t` objects and calls motion-level APIs such as:

```c
motion_drive_set_controlword(&drive1, 0x000FU);
motion_drive_set_mode_of_operation(&drive1, 3U);
motion_drive_set_target_iq(&drive1, 1.25f);
motion_drive_set_target_position_and_velocity(&drive1, 10.0f, 2.0f);
motion_drive_process_tpdo(&drive1, &rx_frame);
motion_drive_process_heartbeat(&drive1, &rx_frame);
```

The portability comes from `can_hal_t`: the motion and CANopen layers know only how to send and receive `can_frame_t`. To move from STM32 to another platform, rewrite only the CAN HAL port layer and keep the CANopen/motion API unchanged.
