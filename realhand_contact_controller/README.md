# realhand_contact_controller

`controller_interface::ControllerInterface` plugin `realhand_contact_controller/ContactGatedController`. Closes each finger toward a commanded target and freezes the finger the moment its tactile pad crosses a force threshold, so a grasp stops at first touch without a planner in the loop. Overview and quickstart live in the [repository README](../README.md).

## Behavior

A close request loads one target per command joint and starts the fingers moving by `close_step` radians per update. Each finger advances until its pad force reaches `contact_threshold`, at which point the finger stops and holds its current command, or until the finger reaches its target with no contact seen, which the controller reports as a miss so a caller can tell a grip from a close on air. The thumb opposition joint drives to its target without gating, and the hand enters HOLD only when opposition and every finger are done, so a short travel finger cannot freeze opposition partway. An open request moves every joint to `open_position` with no gating. Publishing goes through `realtime_tools::RealtimePublisher` with messages sized once at configure time, so `update()` allocates nothing.

## Topics

Topics live under the controller name.

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `~/close_to` | `sensor_msgs/JointState` | in | Target per command joint, starts a contact gated close |
| `~/open` | `std_msgs/Bool` | in | `true` moves every joint to `open_position` without contact gating |
| `~/contact_state` | `std_msgs/Int32MultiArray` | out | One code per finger, see below |
| `~/finger_force` | `std_msgs/Float64MultiArray` | out | Summed pad force per finger |

| Code | Meaning |
|---|---|
| 0 | no contact |
| 2 | contact sensed, finger still moving or no close in progress |
| 1 | gripped, stopped on contact during a gated close and holding |
| 3 | stopped at the commanded target with no contact seen, closed on air |

## Parameters

Declared through `generate_parameter_library` and validated at configure time. `flexion_joints` and `thumb_opposition_joint` must appear in `command_joints`, and `finger_names` and `flexion_joints` must have the same length.

| Parameter | Default | Meaning |
|---|---|---|
| `command_joints` | the six L6 joints | Position command interfaces the controller claims |
| `finger_names` | `[thumb, index, middle, ring, pinky]` | Pads, in the same order as `flexion_joints` |
| `flexion_joints` | five L6 flexion joints | One per finger, gated by the pad of the same finger |
| `thumb_opposition_joint` | `thumb_cmc_yaw` | Driven to target without gating, empty disables |
| `sensor_prefix` | `tactile_` | Prefix of the sensor names |
| `contact_threshold` | `300.0` | Summed pad force at which a finger counts as touching |
| `close_step` | `0.01` | Radians per update toward the target |
| `open_position` | `0.0` | Target of an open request |
| `monitor_only` | `false` | Claim no command interfaces, publish only |

## Interfaces

Claims `<joint>/position` for every entry of `command_joints` (none in monitor only mode) and reads `<sensor_prefix><finger>/force` for every entry of `finger_names`. The controller shares nothing with the driver beyond the interface names, so any hardware exporting the same names works.

## Tests

`test_contact_gated_controller.cpp` drives the controller with loaned interfaces the test owns, no controller_manager, and covers grip on contact, close on air, contact before close, ungated open, thumb opposition ordering, and monitor only mode.
