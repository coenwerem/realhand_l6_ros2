# realhand_l6_moveit_config

MoveIt configuration for the RealHand L6. The SRDF is a xacro macro meant to be included by an arm and hand configuration, plus a standalone demo mounting the hand on `world` and planning in the hand group. Overview and quickstart live in the [repository README](../README.md).

## SRDF macro

`srdf/realhand_l6_macro.srdf.xacro` defines `realhand_l6_srdf` with params `prefix` and `parent_group`.

| Element | Content |
|---|---|
| groups `thumb`, `index`, `middle`, `ring`, `pinky` | one serial chain each from `hand_base_link` to the distal link, KDL solvable |
| group `hand` | the five finger groups |
| group `hand_actuated` | the six driven joints, for servo control and joint space goals |
| group states `open`, `pinch`, `power` | on `hand`, with the mimic joints at their coupled values |
| end effector `hand` | emitted only when `parent_group` names an arm group |
| passive joints | the five mimic distal joints |
| disable_collisions | 65 hand link pairs, adjacent and never colliding, from the Setup Assistant run on the L6 |

In practice, an arm and hand SRDF includes the macro and calls `realhand_l6_srdf` once per hand.

```xml
<xacro:include filename="$(find realhand_l6_moveit_config)/srdf/realhand_l6_macro.srdf.xacro"/>
<xacro:realhand_l6_srdf prefix="right_" parent_group="arm"/>
```

Then the arm config adds the arm to hand collision pairs for its own mount and lists `hand_trajectory_controller` in its `moveit_controllers.yaml`, copying the entry from `config/moveit_controllers.yaml` here.

## Standalone demo

```bash
ros2 launch realhand_l6_moveit_config demo.launch.py
ros2 launch realhand_l6_moveit_config demo.launch.py hardware:=real can_interface:=can0
```

The demo starts ros2_control (mock or the real driver), the joint state broadcaster, `hand_trajectory_controller`, `move_group`, and RViz with the MotionPlanning panel on the `hand` group. Pick `open`, `pinch`, or `power` as the goal state and plan and execute, or send a goal from code.

```bash
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{request: {group_name: hand, goal_constraints: [{joint_constraints: [{joint_name: index_mcp_pitch, position: 0.8, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1.0}]}]}}"
```

## Config

| File | Content |
|---|---|
| `config/kinematics.yaml` | KDL for the five finger chains, the tree groups take joint goals only |
| `config/joint_limits.yaml` | velocity and acceleration limits for time parameterization |
| `config/moveit_controllers.yaml` | `hand_trajectory_controller` for the simple controller manager |
| `config/ompl_planning.yaml` | RRTConnect for `hand` and `hand_actuated` |
| `rviz/moveit.rviz` | MotionPlanning panel on the hand |

## Tests

`test_srdf.py` renders the macro with and without a prefix, checks every joint and link the macro names against the description's URDF, checks the mimic ratios inside the group states, and checks the end effector appears only with a parent group. `test_plan_to_pinch.py` launches the demo on mock hardware and plans and executes to the pinch state through `move_group`.
