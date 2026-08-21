# realhand_l6_ros2

`realhand_l6_ros2` provides ROS 2 Jazzy packages for controlling and integrating the RealHand L6 dexterous hand. The repository includes a C++ `ros2_control` hardware interface over Linux SocketCAN, tactile contact control, robot description and bringup, virtual-CAN emulation, and MoveIt 2 configuration. Physical validation on a right L6 covers CAN communication, hardware activation, joint-state feedback, tactile acquisition and contact detection, contact-gated control, and position and trajectory execution through `ros2_control` and `hand_trajectory_controller`. CI exercises the same software interfaces with mock hardware and an emulated hand on `vcan`.

[![ci](https://github.com/coenwerem/realhand_l6_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/coenwerem/realhand_l6_ros2/actions/workflows/ci.yml)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![License](https://img.shields.io/badge/license-Apache--2.0-green)

## Demos

### Physical Tactile Contact Detection

<p align="center">
  <img src="docs/contact_detection_side_by_side.gif" alt="RealHand L6 fingertip contact detection and corresponding per-finger contact markers in RViz during the same hardware test" width="720"/>
</p>

This recording shows a physical right RealHand L6 mounted on an xArm7. The left panel shows fingertip contact during direct interaction with the hand. The right panel shows the corresponding contact state in RViz using markers placed from the modeled tactile-pad geometry. Each marker is attached to its finger-specific contact frame. Green indicates no detected contact and yellow indicates contact. The hand base frame is attached to the robot wrist and is not shown separately.

### Contact-Gated Control

#### Mock Demonstration

<p align="center">
  <img src="docs/mock_contact_gated_close.gif" alt="Contact-gated close on mock hardware, with each finger stopping when its tactile pad reaches the emulated object surface" width="360"/>
</p>

The mock demonstration exercises the same `ContactGatedController` interface without requiring a physical hand. A fixed cube defines the target grasp geometry, while `mock_contact_surface` converts each finger's gating-joint angle into an emulated tactile force once the finger reaches its configured contact depth. The fingers close concurrently and stop independently at contact. The thumb completes its opposition motion through the grasp, and the RViz markers change state as each finger reaches the emulated surface.

#### Physical Validation

The physical right L6 has been operated through the same driver and controller stack used by the repository. Hardware runs supporting the cited manipulation work exercised SocketCAN communication, hardware activation, joint-state feedback, tactile acquisition, contact detection, contact-gated control, and position and trajectory commands through `ros2_control`. The physical contact-detection recording above is taken from this hardware setup; the mock clip provides a reproducible repository-local visualization of the complete contact-gated closing behavior.

### MoveIt 2 Planning and Control

<p align="center">
  <img src="docs/moveit_hand_states.gif" alt="MoveIt 2 planning and executing power, pinch, and open hand states on mock hardware" width="360"/>
</p>

The MoveIt 2 demo shows `move_group` planning and executing three joint-space targets on the mock stack: `power`, `pinch`, and `open`. RViz displays each planned trajectory before execution. The standalone MoveIt configuration sends trajectories through `hand_trajectory_controller`, the same ROS 2 control interface used for trajectory execution on the physical L6.

## Bundled Packages

Each package has its own README with detailed interfaces, parameters, launch arguments, and validation notes.

| Package | Purpose |
|---|---|
| `realhand_hardware` | Implements `realhand_hardware/RealHandSystem`, a `hardware_interface::SystemInterface` plugin that communicates with the RealHand CAN protocol over SocketCAN. It exports joint position command and state interfaces, synthesized state for coupled distal joints, tactile force state interfaces, and optional per-joint speed and torque command interfaces. |
| `realhand_contact_controller` | Implements `realhand_contact_controller/ContactGatedController`. Each finger advances toward a commanded target and holds its current command once the corresponding tactile pad crosses a force threshold. The controller also distinguishes contact-limited closure from reaching the target in free space, supports ungated opening and monitor-only operation, and uses real-time-safe publishing. |
| `realhand_l6_description` | Provides right- and left-hand Xacro descriptions, meshes, tactile-pad frames, a tool-center-point frame, and a `ros2_control` macro that selects either mock hardware or `RealHandSystem`. |
| `realhand_l6_bringup` | Provides launch files and controller configuration for mock hardware, `vcan`, and physical hardware, together with RViz contact visualization, the CAN emulator, the mock contact-surface node, and a read-only tactile probe. |
| `realhand_l6_moveit_config` | Provides an SRDF Xacro macro for arm-hand configurations, predefined hand states, self-collision exclusions, and a standalone MoveIt 2 demo using `hand_trajectory_controller`. |

## Quick Start Without Hardware

The mock stack requires only a ROS 2 Jazzy desktop installation.

```bash
mkdir -p ~/realhand_ws/src && cd ~/realhand_ws
git clone https://github.com/coenwerem/realhand_l6_ros2.git src/realhand_l6_ros2
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch realhand_l6_bringup mock.launch.py
```

RViz starts with the hand model, tactile markers, and the demonstration cube. After a short delay, the mock stack issues a close request. The fingers move toward the commanded grasp and stop independently as their emulated pad forces cross the contact threshold. You can also publish commands directly while the stack is running.

```bash
ros2 topic pub --once /contact_gated_controller/open std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /contact_gated_controller/close_to sensor_msgs/msg/JointState \
  "{name: [thumb_cmc_pitch, thumb_cmc_yaw, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch], position: [0.0, 1.25, 0.75, 1.0, 1.0, 0.78]}"
ros2 topic pub --once /tactile_mock_controller/commands std_msgs/msg/Float64MultiArray "{data: [0, 0, 9000, 0, 0]}"
ros2 topic echo /contact_gated_controller/contact_state
```

## Virtual CAN Bus

The `vcan` path runs the actual `realhand_hardware` plugin against an emulated L6 rather than replacing the driver with mock hardware. This exercises SocketCAN transport, CAN filters, frame encoding and decoding, hardware activation, tactile traffic, and command handling without a physical hand.

```bash
./src/realhand_l6_ros2/realhand_l6_bringup/scripts/setup_vcan.sh
ros2 launch realhand_l6_bringup vcan.launch.py
```

## Physical Hardware

With the hand powered, bring `can0` up at 1 Mbit/s and start with the read-only monitor. The monitor activates the hand, holds the measured joint position, and enables torque while claiming no command interfaces. It publishes tactile force and contact state so the finger mapping can be verified before motion is commanded.

```bash
sudo ip link set can0 up type can bitrate 1000000
ros2 launch realhand_l6_bringup hardware.launch.py controller:=monitor
ros2 topic echo /tactile_monitor/finger_force
```

After verifying the tactile mapping, launch the contact-gated controller on the physical hand.

```bash
ros2 launch realhand_l6_bringup hardware.launch.py side:=right can_interface:=can0
```

## Standard Position and Trajectory Controllers

`RealHandSystem` exports a standard `position` command interface for every actuated joint, so the hand can be driven by standard `ros2_control` controllers as well as `ContactGatedController`. `realhand_l6_bringup/config/controllers.yaml` provides both a `joint_trajectory_controller` for planners and scripted trajectories and a forward position controller for direct setpoints. Select them with `controller:=trajectory` or `controller:=position` in `hardware.launch.py`. Only one controller that claims the position interfaces can be active at a time.

```bash
ros2 launch realhand_l6_bringup hardware.launch.py controller:=trajectory
ros2 action send_goal /hand_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [thumb_cmc_pitch, thumb_cmc_yaw, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch], points: [{positions: [0.3, 0.8, 1.0, 1.0, 1.0, 1.0], time_from_start: {sec: 2}}]}}"
```

Each actuated joint can also expose raw `speed` and `torque` command interfaces. These interfaces use the hand's 0-255 setpoints, initialize from `activation_speed` and `activation_torque`, and transmit a CAN frame when their commanded values change. Setting `setpoint_controllers:=true` starts forward controllers for both interfaces, allowing speed or torque limits to be adjusted while a position controller remains active.

```bash
ros2 topic pub --once /hand_speed_controller/commands std_msgs/msg/Float64MultiArray "{data: [40, 40, 40, 40, 40, 40]}"
ros2 topic pub --once /hand_torque_controller/commands std_msgs/msg/Float64MultiArray "{data: [120, 120, 120, 120, 120, 120]}"
```

For a ROS-independent tactile check, run `ros2 run realhand_l6_bringup can_tactile_probe --interface can0`. The probe sends only tactile-matrix read requests and prints per-finger sums and row rates. Run it with the CAN bus otherwise idle so the measured rate reflects the hand's response rate directly.

## MoveIt 2 Configuration

`realhand_l6_moveit_config` provides the `realhand_l6_srdf` Xacro macro for integration into a larger arm-hand MoveIt configuration. It defines one serial-chain group per finger, the `hand` and `hand_actuated` groups, the `open`, `pinch`, and `power` states, and hand self-collision exclusions. The standalone demo plans and executes trajectories for the `hand` group through `hand_trajectory_controller`.

```bash
ros2 launch realhand_l6_moveit_config demo.launch.py
ros2 launch realhand_l6_moveit_config demo.launch.py hardware:=real can_interface:=can0
```

## Interfaces and Parameters

Detailed package-specific documentation is available in the inner READMEs:

- [realhand_hardware](realhand_hardware/README.md): exported hardware interfaces, CAN behavior, activation sequence, and driver parameters.
- [realhand_contact_controller](realhand_contact_controller/README.md): controller behavior, topics, contact-state codes, and parameters.
- [realhand_l6_description](realhand_l6_description/README.md): URDF/Xacro macros, frames, meshes, and description tests.
- [realhand_l6_bringup](realhand_l6_bringup/README.md): launch files, controller configuration, helper nodes, and integration tests.
- [realhand_l6_moveit_config](realhand_l6_moveit_config/README.md): SRDF macro, planning groups, predefined states, MoveIt configuration, and tests.

## Supported RealHand Models

Physical validation currently covers the right L6 over CAN. According to the vendor SDK, the L6, L7, O6, and L10 use the CAN frame family implemented by `realhand_hardware`: position `0x01`, torque `0x02`, speed `0x05`, and tactile matrices beginning at `0xb1`. Model-specific quantities such as joint names, radian scaling, mimic ratios, taxel geometry, matrix mode byte, and CAN IDs are isolated in the driver model table. Supporting an L7, O6, or L10 therefore requires a model-table entry followed by hardware validation. Only validated model entries are intended to ship as supported configurations.

The L20, L21, L24, L25, and G20 use a different frame layout and require a separate codec. Contributions from users with access to those models are welcome.

The left L6 description mirrors the validated right-hand geometry and matches the vendor left URDF joint by joint. Its joint limits currently use the calibrated right-hand values so the URDF and driver table remain consistent. Those mirrored limits have not been validated on a physical left L6.

## Tests

`colcon test` covers the CAN codec and model table with gtest, the contact-controller state machine with owned loaned interfaces, and both hand descriptions with pytest over generated Xacro. Controller tests cover stopping on contact, reaching a target without contact, ungated opening, thumb-opposition ordering, and monitor-only operation.

Integration coverage uses `launch_testing` for the full mock stack, including a `FollowJointTrajectory` goal; the real hardware plugin against an emulated L6 on `vcan`, including speed-setpoint transmission; and `move_group` on mock hardware planning and executing the `pinch` state. The `vcan` test skips when the interface is unavailable. CI runs the suite on ROS 2 Jazzy for every push and pull request.

## Citation

The driver and controller were developed in support of the hardware experiments associated with the papers below. If `realhand_l6_ros2` supports your work, please cite the relevant paper. [`CITATION.cff`](CITATION.cff) also provides citation metadata for the software itself.

```bibtex
@article{enwerem2026grasp,
  title={Grasp Execution Without a Planner: Configuration-Space Grasp Distance Fields with Certified Safety \& Guaranteed Quality},
  author={Enwerem, Clinton and Baras, John S. and Belta, Calin},
  journal={arXiv preprint arXiv:2608.00600},
  year={2026}
}

@article{enwerem2026firmgrasp,
  title={FIRMGrasp: A Friction-Informed Risk Margin for Robust Grasp Synthesis},
  author={Enwerem, Clinton and Baras, John S. and Belta, Calin},
  journal={arXiv preprint arXiv:2607.25049},
  year={2026}
}

@inproceedings{enwerem2026variational,
  title={Variational Neural Belief Parameterizations for Robust Dexterous Grasping under Multimodal Uncertainty},
  author={Enwerem, Clinton and Kalyanaraman, Shreya and Baras, John S. and Belta, Calin},
  booktitle={Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2026},
  eprint={2604.25897},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  note={To appear}
}
```

## Attribution and License

The repository is licensed under Apache-2.0. The meshes are derived from the vendor's Apache-2.0 [linkerhand-urdf](https://github.com/linker-bot/linkerhand-urdf) release. The vendor SDK is available from [RealHand-Robotics](https://github.com/RealHand-Robotics). The `realhand_hardware` driver is independent of the vendor SDK and implements the CAN protocol directly.
