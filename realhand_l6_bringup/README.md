# realhand_l6_bringup

`realhand_l6_bringup` provides launch files, controller configuration, RViz contact visualization, mock and virtual-CAN support, and hardware-facing diagnostic utilities for the RealHand L6 stack. It is the integration package that connects the robot description, `ros2_control` hardware interface, tactile controller, and standard ROS 2 controllers. Repository-level demos and installation instructions are available in the [top-level README](../README.md).

## Launch Files

| Launch file | Configuration | Key arguments |
|---|---|---|
| `mock.launch.py` | Runs `mock_components/GenericSystem`, joint-state broadcasting, mock tactile commands, `ContactGatedController`, an inactive `hand_trajectory_controller`, `mock_contact_surface`, and RViz | `use_rviz`, `auto_close`, `close_delay`, `mock_ramp`, `use_object` |
| `vcan.launch.py` | Runs the real `RealHandSystem` driver on a virtual CAN interface while `mock_can_feeder` emulates the hand | `can_interface`, `use_rviz`, `auto_close`, `contact_delay`, `ramp`, `record`, `setpoint_controllers` |
| `hardware.launch.py` | Runs `RealHandSystem` on a physical CAN bus with a selectable controller | `side`, `can_interface`, `can_id`, `enable_tactile`, `taxel_topic`, `controller`, `setpoint_controllers`, `use_rviz` |

The `controller` argument of `hardware.launch.py` selects the active position-control path:

- `contact` starts `contact_gated_controller`.
- `trajectory` starts `hand_trajectory_controller`.
- `position` starts `hand_position_controller`.
- `monitor` starts the read-only tactile monitor and claims no position command interfaces.

Setting `setpoint_controllers:=true` additionally starts the forward speed and torque controllers. These controllers claim separate interfaces and can therefore run alongside whichever position controller is active.

## Controllers

`config/controllers.yaml` defines the controllers used by the launch files.

| Controller | Type | Claimed Interfaces |
|---|---|---|
| `joint_state_broadcaster` | `JointStateBroadcaster` | Eleven joint-position states; tactile sensors are excluded |
| `contact_gated_controller` | `ContactGatedController` | Six position commands and five tactile force states |
| `tactile_monitor` | `ContactGatedController` with `monitor_only` | Five tactile force states |
| `hand_trajectory_controller` | `JointTrajectoryController` | Six position commands |
| `hand_position_controller` | `ForwardCommandController` | Six position commands |
| `hand_speed_controller` | `ForwardCommandController` | Six speed commands |
| `hand_torque_controller` | `ForwardCommandController` | Six torque commands |
| `tactile_mock_controller` | `ForwardCommandController` | Five mock sensor commands; mock hardware only |

The mock demo overlays `config/mock_controllers.yaml` on the main controller configuration. For that demonstration, `thumb_cmc_yaw` is used as the contact-gated thumb joint and `thumb_cmc_pitch` is treated as the ungated opposition joint held at zero. This mapping matches the posed cube grasp used by the mock contact-surface model. The physical configuration retains the hardware mapping used by the real hand.

Only one controller that claims the six position interfaces can be active at a time. For example, the contact controller can be replaced by the trajectory controller at runtime with:

```bash
ros2 control switch_controllers \
  --deactivate contact_gated_controller \
  --activate hand_trajectory_controller
```

The speed and torque controllers can remain active because they claim different command interfaces.

## Helper Nodes and Scripts

| Entry point | Purpose |
|---|---|
| `contact_viz` | Publishes a `MarkerArray` for the five tactile pads using their `<finger>_pad` frames and colors the markers from the selected controller's contact-state output |
| `mock_contact_surface` | Emulates object contact by converting each gating-joint angle into a tactile force after a configurable per-finger contact depth; the default depths reproduce the demo cube grasp |
| `mock_can_feeder` | Emulates an L6 on SocketCAN by answering position requests, echoing commands, streaming tactile rows, and optionally recording speed and torque frames |
| `can_tactile_probe` | Sends read-only tactile-matrix requests and reports per-finger force sums and row rates without starting the ROS 2 hardware stack |
| `scripts/setup_vcan.sh` | Creates and brings up the `vcan0` interface used by the virtual-CAN integration path |

The mock and `vcan` paths serve different purposes. `mock.launch.py` replaces the physical hardware layer with `GenericSystem` so controller behavior can be tested directly. `vcan.launch.py` keeps the actual `RealHandSystem` plugin in the loop and replaces only the physical CAN device with an emulator, allowing the SocketCAN and protocol paths to be exercised end to end.

## Tests

`test_mock_pipeline.py` launches the complete mock stack, injects tactile contact, verifies contact-limited and free-space closure states, opens the hand, switches to `hand_trajectory_controller`, and executes a `FollowJointTrajectory` goal.

`test_vcan_pipeline.py` runs `RealHandSystem` against `mock_can_feeder` over `vcan`. It checks the driver decode path, contact-gated behavior, and transmission of speed setpoints. The test skips when a virtual CAN interface is unavailable.
