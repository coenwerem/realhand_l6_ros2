# realhand_l6_bringup

Launch files, controller configuration, RViz contact markers, an emulated hand for a virtual CAN bus, a pad force ramp for the mock, and a read only CAN probe. Overview and quickstart live in the [repository README](../README.md).

## Launch files

| Launch file | What runs | Key arguments |
|---|---|---|
| `mock.launch.py` | `mock_components/GenericSystem`, joint state broadcaster, tactile mock forward controller, contact gated controller, `hand_trajectory_controller` loaded inactive, force ramp, RViz | `use_rviz`, `auto_close`, `close_delay`, `contact_delay`, `stagger`, `mock_ramp` |
| `vcan.launch.py` | the real driver on a virtual CAN interface with `mock_can_feeder` emulating the hand | `can_interface`, `use_rviz`, `auto_close`, `contact_delay`, `ramp`, `record`, `setpoint_controllers` |
| `hardware.launch.py` | the real driver on a real bus | `side`, `can_interface`, `can_id`, `enable_tactile`, `taxel_topic`, `controller`, `setpoint_controllers`, `use_rviz` |

In particular, the `controller` argument of `hardware.launch.py` picks what drives the hand. `contact` runs the contact gated controller, `trajectory` a `joint_trajectory_controller`, `position` a forward position controller, and `monitor` the read only tactile monitor, which commands nothing. `setpoint_controllers:=true` adds forward controllers on the driver's `speed` and `torque` command interfaces next to whichever controller runs.

## Controllers

`config/controllers.yaml` defines every controller the launch files spawn.

| Controller | Type | Claims |
|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | the eleven joint positions, sensors excluded |
| `contact_gated_controller` | ContactGatedController | six position commands, five pad forces |
| `tactile_monitor` | ContactGatedController, `monitor_only` | five pad forces |
| `hand_trajectory_controller` | JointTrajectoryController | six position commands |
| `hand_position_controller` | ForwardCommandController | six position commands |
| `hand_speed_controller` | ForwardCommandController | six speed commands |
| `hand_torque_controller` | ForwardCommandController | six torque commands |
| `tactile_mock_controller` | ForwardCommandController | five fake sensor commands, mock only |

In practice, only one position controller can be active at a time, since all three claim the same position interfaces. Switch with `ros2 control switch_controllers --deactivate contact_gated_controller --activate hand_trajectory_controller`. The speed and torque controllers run next to any position controller.

## Nodes and scripts

| Entry point | Purpose |
|---|---|
| `contact_viz` | MarkerArray of the five pads on their `<finger>_pad` frames, colored by contact code, from any controller named by the `controller` parameter |
| `mock_force_ramp` | Publishes pad forces to `tactile_mock_controller`, one finger after another |
| `mock_can_feeder` | Emulates an L6 on SocketCAN, answers position requests, echoes commands, streams taxel rows, records speed and torque frames with `--record` |
| `can_tactile_probe` | Read only probe, matrix requests only, prints per finger sums and row rates |
| `scripts/setup_vcan.sh` | Brings up `vcan0` |

## Tests

`test_mock_pipeline.py` runs the whole mock stack, presses a pad, closes, checks the latch and miss codes and the joint positions, opens, then swaps in the trajectory controller and drives the hand through a `FollowJointTrajectory` goal. `test_vcan_pipeline.py` runs the real driver against the emulator, checks the decode path, the latch, and that speed setpoints reach the bus, and skips when `vcan0` is absent.
