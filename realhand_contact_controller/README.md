# realhand_contact_controller

`realhand_contact_controller` implements `realhand_contact_controller/ContactGatedController`, a `controller_interface::ControllerInterface` plugin for tactile contact-gated closing. Each finger advances toward a commanded target and holds its current position once the corresponding tactile pad crosses a force threshold. This allows the hand to stop individual fingers at first contact while preserving a standard `ros2_control` position-command interface. Repository-level demos and launch examples are available in the [top-level README](../README.md).

## Control Behavior

A close request specifies one target for each commanded joint. During each controller update, the flexion joints advance toward their targets by `close_step` radians. A finger stops advancing when its tactile force reaches `contact_threshold` and then holds the current command. If a finger instead reaches its requested target without detecting contact, the controller records that condition separately so downstream code can distinguish a contact-limited grip from closure in free space.

The thumb opposition joint is commanded to its target without contact gating. The controller does not enter `HOLD` until the opposition joint and all gated fingers have completed their motions, preventing an early finger stop from interrupting thumb opposition. An open request moves every commanded joint to `open_position` without contact gating.

State and force messages are published through `realtime_tools::RealtimePublisher`. Message storage is allocated during configuration so `update()` does not allocate dynamically.

## Topics

All topics are relative to the controller name.

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `~/close_to` | `sensor_msgs/JointState` | input | Target positions for the commanded joints; starts a contact-gated close |
| `~/open` | `std_msgs/Bool` | input | `true` commands all joints to `open_position` without contact gating |
| `~/contact_state` | `std_msgs/Int32MultiArray` | output | One contact-state code per finger |
| `~/finger_force` | `std_msgs/Float64MultiArray` | output | Summed tactile force per finger |

The contact-state codes distinguish sensing from controller outcome:

| Code | Meaning |
|---|---|
| 0 | No contact detected |
| 1 | Finger stopped on contact during a gated close and is holding |
| 2 | Contact detected while the finger is still moving, or when no gated close is active |
| 3 | Finger reached its commanded target without detecting contact |

## Parameters

Parameters are declared through `generate_parameter_library` and validated during controller configuration. Every entry in `flexion_joints` and the optional `thumb_opposition_joint` must appear in `command_joints`. `finger_names` and `flexion_joints` must have the same length so each gated joint maps to exactly one tactile pad.

| Parameter | Default | Meaning |
|---|---|---|
| `command_joints` | six L6 actuated joints | Position command interfaces claimed by the controller |
| `finger_names` | `[thumb, index, middle, ring, pinky]` | Tactile pads, ordered to match `flexion_joints` |
| `flexion_joints` | five L6 flexion joints | One contact-gated joint per finger |
| `thumb_opposition_joint` | `thumb_cmc_yaw` | Joint driven to its target without contact gating; an empty value disables opposition handling |
| `sensor_prefix` | `tactile_` | Prefix used to construct tactile sensor interface names |
| `contact_threshold` | `300.0` | Summed pad force at or above which contact is detected |
| `close_step` | `0.01` | Maximum position increment in radians per controller update during closing |
| `open_position` | `0.0` | Target position used by an open request |
| `monitor_only` | `false` | If `true`, claim no command interfaces and publish tactile state only |

## Claimed Interfaces

In normal operation, the controller claims `<joint>/position` for each entry in `command_joints` and reads `<sensor_prefix><finger>/force` for each entry in `finger_names`. In `monitor_only` mode it claims no command interfaces and only reads the tactile states.

The controller depends on interface names rather than on `realhand_hardware` directly. Any `ros2_control` hardware component that exports the same position and tactile interfaces can therefore provide the underlying state and commands.

## Tests

`test_contact_gated_controller.cpp` drives the controller through test-owned loaned interfaces without a `controller_manager`. The tests cover stopping on contact, reaching the close target without contact, contact observed before a close begins, ungated opening, thumb-opposition ordering, and monitor-only operation.
