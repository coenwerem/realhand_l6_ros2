# realhand_l6_ros2

ros2_control driver, tactile contact controller, and robot description for the RealHand L6 dexterous hand. C++ over Linux SocketCAN, ROS 2 Jazzy, tested against a mock hardware stack and an emulated hand on a virtual CAN bus in CI, with the CAN protocol handling and the contact controller validated on a physical L6.

[![ci](https://github.com/coenwerem/realhand_l6_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/coenwerem/realhand_l6_ros2/actions/workflows/ci.yml)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![License](https://img.shields.io/badge/license-Apache--2.0-green)

<p align="center">
  <img src="docs/l6_tactile_contact.gif" alt="RealHand L6 detecting fingertip contact from its taxel pads" width="360"/>
  <img src="docs/rviz_contact_detection.gif" alt="Per finger contact markers in RViz during the same hardware test" width="300"/>
</p>

Specifically, the pair above comes from one hardware session, fingertip contact detection on the real hand through the driver tactile export on the left, and the same signal drawn as pad markers in RViz on the right, yellow on contact.

<p align="center">
  <img src="docs/mock_contact_gated_close.gif" alt="Contact gated close on the mock stack, each finger freezes when its pad reports contact" width="360"/>
</p>

By contrast, the third clip shows the contact gated close on the mock stack, each finger stops where its pad first reports contact and turns red, with the pads pressed in sequence, thumb to pinky, so the latch order shows.

## Packages

| Package | Content |
|---|---|
| `realhand_hardware` | `hardware_interface::SystemInterface` plugin `realhand_hardware/RealHandSystem`. Speaks the RealHand CAN frame family over SocketCAN, position command and state for the actuated joints, synthesized state for the coupled distal joints, and one summed force state interface per tactile pad. Per model constants live in a table selected by a `model` parameter. |
| `realhand_contact_controller` | `controller_interface::ControllerInterface` plugin `realhand_contact_controller/ContactGatedController`. Closes each finger toward a target and freezes the finger the moment its pad crosses a force threshold, reports gripped versus closed on air, ungated open, monitor only mode. Real time safe publishing, parameters through `generate_parameter_library`. |
| `realhand_l6_description` | Xacro macro for the L6, right and left, meshes, `<finger>_pad` frames on each distal link, tool center point, and a `ros2_control` macro switching between `mock_components/GenericSystem` and the real driver. |
| `realhand_l6_bringup` | Launch files for mock, virtual CAN, and hardware, controller configuration, RViz contact markers, an emulated hand for `vcan0`, a pad force ramp for the mock, and a read only CAN probe. |

## Quickstart with no hardware

The mock stack needs nothing beyond a Jazzy desktop install.

```bash
mkdir -p ~/realhand_ws/src && cd ~/realhand_ws
git clone https://github.com/coenwerem/realhand_l6_ros2.git src/realhand_l6_ros2
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch realhand_l6_bringup mock.launch.py
```

Then RViz opens with the hand and the pad markers. After a few seconds a close request goes out and the mock ramp presses the pads one finger at a time, so every finger stops at a different angle. Send your own requests while the mock stack runs.

```bash
ros2 topic pub --once /contact_gated_controller/open std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /contact_gated_controller/close_to sensor_msgs/msg/JointState \
  "{name: [thumb_cmc_pitch, thumb_cmc_yaw, index_mcp_pitch, middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch], position: [0.5, 1.2, 1.4, 1.4, 1.4, 1.4]}"
ros2 topic pub --once /tactile_mock_controller/commands std_msgs/msg/Float64MultiArray "{data: [0, 0, 9000, 0, 0]}"
ros2 topic echo /contact_gated_controller/contact_state
```

## The real driver on a virtual CAN bus

The virtual bus path runs `realhand_hardware` itself, socket, filters, frame decode, and activation sequence, against an emulated hand.

```bash
./src/realhand_l6_ros2/realhand_l6_bringup/scripts/setup_vcan.sh
ros2 launch realhand_l6_bringup vcan.launch.py
```

## Hardware

Bring `can0` up at 1 Mbit/s with the hand powered, then run the read only monitor first. The monitor activates the hand (reads position, holds position, enables torque), claims no command interfaces, and publishes contact and force so you can press each pad and confirm the finger mapping in RViz before anything moves.

```bash
sudo ip link set can0 up type can bitrate 1000000
ros2 launch realhand_l6_bringup hardware.launch.py monitor_only:=true
ros2 topic echo /tactile_monitor/finger_force
```

Next, the full stack replaces the monitor with the contact gated controller.

```bash
ros2 launch realhand_l6_bringup hardware.launch.py side:=right can_interface:=can0
```

In addition, for a check independent of ROS, `ros2 run realhand_l6_bringup can_tactile_probe --interface can0` sends only matrix read requests and prints per finger sums and row rates. Run the probe with nothing else on the bus so the rate you read is the hand's own.

## Driver parameters

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

In practice, exported interfaces come from the `ros2_control` XML. Actuated joints take a `position` command and report `position`, mimic joints report `position`, a `velocity` state interface is filled with zero where declared, and each sensor `tactile_<finger>` reports `force`, the sum of its 12 by 6 taxel grid. The `realhand_l6_description` macro `realhand_l6_ros2_control` writes the whole interface block for you.

## Contact gated controller

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
| 1 | latched on contact during a gated close, gripped |
| 3 | latched at the commanded target with no contact seen, closed on air |

Similarly, the parameters below are all validated at configure time.

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

## Which RealHand models

Validated on a right L6 over CAN. The frame family the driver speaks, position 0x01, torque 0x02, speed 0x05, tactile matrices 0xb1 and up, is shared by the L6, L7, O6, and L10 per the vendor SDK, and everything model specific in the driver is a table entry, joint names and radian scaling, mimic ratios, taxel geometry, matrix mode byte, CAN ids. Adding an L7, O6, or L10 is a table entry plus a check on real hardware, and only validated entries ship. The L20, L21, L24, L25, and G20 use a different frame layout and need a second codec. Contributions with hardware access are welcome.

In particular, the left L6 geometry is the mirror of the validated right hand and matches the vendor left URDF joint by joint. Its joint limits are set to the right hand's calibrated values so the driver table and the URDF agree, and are unvalidated on a left unit.

## Tests

`colcon test` runs gtest on the CAN codec and model table, gtest on the controller state machine with loaned interfaces the test owns (latch on contact, closed on air, ungated open, thumb opposition ordering, monitor only), a pytest over the xacro for both sides and both hardware plugins, a `launch_testing` run of the whole mock stack, and a `launch_testing` run of the real driver on `vcan0`, skipped when the interface is absent. CI runs the same suite on Jazzy for every push.

## Citing

The driver and controller grew out of the hardware experiments in the papers below. If the code helps your work, cite one of them.

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

## Attribution and license

Apache-2.0. Meshes derive from the vendor's Apache-2.0 [linkerhand-urdf](https://github.com/linker-bot/linkerhand-urdf) release. The vendor's own SDK lives at [RealHand-Robotics](https://github.com/RealHand-Robotics). The driver in `realhand_hardware` is independent of the vendor SDK and speaks the CAN protocol directly.
