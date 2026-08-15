# realhand_hardware

`hardware_interface::SystemInterface` plugin `realhand_hardware/RealHandSystem` for RealHand dexterous hands over Linux SocketCAN. One instance drives one hand. Validated on a right L6. Overview, media, and quickstart live in the [repository README](../README.md).

## Interfaces

Exported interfaces come from the `ros2_control` XML, which the `realhand_l6_ros2_control` macro in `realhand_l6_description` writes for you.

| Interface | Kind | Meaning |
|---|---|---|
| `<joint>/position` | command | Target angle in radians, clamped into `[0, max_rad]` before encoding |
| `<joint>/speed` | command, optional | Raw speed setpoint 0 to 255, sent as one 0x05 frame whenever any joint's value changes |
| `<joint>/torque` | command, optional | Raw torque setpoint 0 to 255, sent as one 0x02 frame whenever any joint's value changes |
| `<joint>/position` | state | Angle in radians, mimic joints synthesized from their parent |
| `<joint>/velocity` | state, optional | Filled with zero, the hand reports no velocity |
| `tactile_<finger>/force` | state, sensor | Sum of the finger's 12 by 6 taxel grid, 0 to 18360 |

In particular, the speed and torque interfaces are seeded with `activation_speed` and `activation_torque` on activation, so a claiming controller reads a defined value, and joints without the interfaces hold the activation value. Both frames go out only on change, as does the position frame, so a held hand costs no bus traffic.

## Parameters

Set inside the `<hardware>` block of the `ros2_control` tag. Every parameter is optional.

| Parameter | Default | Meaning |
|---|---|---|
| `model` | `L6` | Hand model, selects the joint table, mimic ratios, taxel geometry, and CAN ids |
| `can_interface` | `can0` | SocketCAN device |
| `hand_side` | `right` | `right` or `left`, selects the default CAN id (0x27 right, 0x28 left) |
| `can_id` | side default | Explicit CAN id, base 10 or 0x hex |
| `joint_prefix` | `<hand_side>_` | Prefix stripped from URDF joint and sensor names before lookup, bare names always work |
| `enable_tactile` | `true` | Request taxel matrices from the hand |
| `tactile_period_ms` | `50` | Period of the matrix request burst |
| `position_request_decimation` | `4` | Request joint position every N control cycles, each request preempts the tactile stream |
| `activation_speed` | `80` | Speed byte sent on activation, 0 to 255 |
| `activation_torque` | `200` | Torque byte sent on activation, 0 to 255 |
| `taxel_topic` | unset | When set, publish the raw taxel grids as JSON on the named topic from the receiver thread |

## Layout

`hand_model.hpp` holds the per model table (joint names and radian scaling, mimic ratios, taxel geometry, matrix mode byte, CAN ids), `protocol.hpp` the pure frame codec, `realhand_system.hpp` the plugin. The codec and the table have gtest coverage with no bus. The plugin runs end to end against an emulated hand in `realhand_l6_bringup` on a virtual CAN interface.

## Activation sequence

On activation the driver opens the socket, starts the receiver thread, reads the current position, commands the current position back, sets speed and torque, then seeds every state and command interface with the physical position, so the first `write()` holds the hand in place until a controller commands otherwise.
