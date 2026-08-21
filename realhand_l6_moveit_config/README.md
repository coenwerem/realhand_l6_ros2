# realhand_l6_moveit_config

`realhand_l6_moveit_config` provides MoveIt 2 configuration for the RealHand L6. The package includes an SRDF Xacro macro intended for arm-hand systems and a standalone hand demo that mounts the L6 on `world`, starts `ros2_control`, and plans for the hand groups. Repository-level demos and installation instructions are available in the [top-level README](../README.md).

## SRDF Macro

`srdf/realhand_l6_macro.srdf.xacro` defines the `realhand_l6_srdf` macro with parameters `prefix` and `parent_group`.

| Element | Definition |
|---|---|
| `thumb`, `index`, `middle`, `ring`, `pinky` groups | One serial chain per finger from `hand_base_link` to the corresponding distal link |
| `hand` group | Composite group containing the five finger groups |
| `hand_actuated` group | Six actuated joints for direct joint-space targets and servo-style control |
| `open`, `pinch`, `power` group states | Named configurations on the `hand` group, including the coupled mimic-joint values |
| `hand` end effector | Emitted when `parent_group` names an arm planning group |
| Passive joints | The five coupled distal mimic joints |
| Disabled collisions | 65 adjacent or otherwise non-colliding hand-link pairs generated from the L6 Setup Assistant configuration |

The finger groups are serial chains and can use KDL where an individual finger kinematic solver is needed. The composite `hand` and `hand_actuated` groups are primarily intended for joint-space planning and named-state execution.

An arm-hand SRDF can include the macro once per hand:

```xml
<xacro:include filename="$(find realhand_l6_moveit_config)/srdf/realhand_l6_macro.srdf.xacro"/>
<xacro:realhand_l6_srdf prefix="right_" parent_group="arm"/>
```

The parent arm configuration remains responsible for arm-to-hand collision exclusions associated with its specific wrist mount. It should also expose `hand_trajectory_controller` through its `moveit_controllers.yaml`; the standalone configuration in this package provides a reference controller entry.

## Standalone Demo

```bash
ros2 launch realhand_l6_moveit_config demo.launch.py
ros2 launch realhand_l6_moveit_config demo.launch.py hardware:=real can_interface:=can0
```

The demo starts the hand description, `ros2_control`, `joint_state_broadcaster`, `hand_trajectory_controller`, `move_group`, and RViz with the MotionPlanning panel configured for the `hand` group. With mock hardware, the entire planning and execution path can be exercised without a physical hand. With `hardware:=real`, the same trajectory-controller interface is connected to `realhand_hardware/RealHandSystem`.

In RViz, select `open`, `pinch`, or `power` as a named target and use the MotionPlanning panel to plan and execute. Joint-space goals can also be sent programmatically through the MoveIt action interface. For example:

```bash
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup \
  "{request: {group_name: hand, goal_constraints: [{joint_constraints: [{joint_name: index_mcp_pitch, position: 0.8, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}]}]}}"
```

## Configuration Files

| File | Purpose |
|---|---|
| `config/kinematics.yaml` | Configures KDL for the five serial finger chains; composite hand groups use joint-space targets rather than a single chain solver |
| `config/joint_limits.yaml` | Provides velocity and acceleration limits used during trajectory time parameterization |
| `config/moveit_controllers.yaml` | Exposes `hand_trajectory_controller` to the MoveIt simple controller manager |
| `config/ompl_planning.yaml` | Configures RRTConnect for the `hand` and `hand_actuated` planning groups |
| `rviz/moveit.rviz` | RViz configuration with the MotionPlanning panel focused on the hand |

## Tests

`test_srdf.py` renders the SRDF macro with and without a joint prefix, checks every referenced joint and link against the L6 URDF, verifies the mimic-joint values used by the named states, and confirms that the end-effector declaration is emitted only when a parent arm group is provided.

`test_plan_to_pinch.py` launches the standalone configuration on mock hardware, asks `move_group` to plan to the `pinch` state, and verifies execution through `hand_trajectory_controller`.
