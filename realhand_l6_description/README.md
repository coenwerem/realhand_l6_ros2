# realhand_l6_description

URDF, meshes, `ros2_control` block, and RViz configuration for the RealHand L6, right and left. Overview and quickstart live in the [repository README](../README.md).

## Macros

`urdf/realhand_l6_macro.xacro` defines `realhand_l6` with params `prefix`, `side` (`right` or `left`), `parent`, and an `origin` block. The macro emits the palm and finger links with vendor meshes, six actuated revolute joints, five mimic distal joints, a `tcp` frame 40 mm above the palm, and a `<finger>_pad` frame on each distal link, z out of the pad toward the grip, x along the finger, y across the 12 by 6 taxel window. The left hand is the mirror of the validated right hand, y negated, roll and yaw negated, thumb yaw axis flipped, and matches the vendor left URDF joint by joint. Joint limits are the calibrated right hand values on both sides, so the driver table and the URDF agree, and the left values are unvalidated on a left unit.

Next, `urdf/realhand_l6.ros2_control.xacro` defines `realhand_l6_ros2_control` with params `name`, `prefix`, `side`, `hardware` (`mock` or `real`), `can_interface`, `can_id`, `enable_tactile`, `taxel_topic`, and `setpoints`. `mock` loads `mock_components/GenericSystem` with `mock_sensor_commands`, so tests and demos drive the pad forces through fake command interfaces. `real` loads `realhand_hardware/RealHandSystem` with the matching parameters. `setpoints` (default true) adds `speed` and `torque` command interfaces to every actuated joint.

Finally, `urdf/realhand_l6.urdf.xacro` is the standalone hand on a `world` link with args `prefix`, `side`, `hardware` (`none`, `mock`, `real`), `can_interface`, `can_id`, `enable_tactile`, `taxel_topic`, and `setpoints`. The `use_object` arg (default false) adds a visual-only cube, `object_link`, on a fixed joint at the hand base, posed and sized through `object_size`, `object_xyz`, `object_rpy`, and `object_rgba`. The mock demo renders the cube as the object whose surface `mock_contact_surface` emulates, with the default size and pose taken from the posed grasp scene the demo's contact depths reproduce.

## View the hand

```bash
ros2 launch realhand_l6_description view_hand.launch.py side:=left
```

## Meshes

STL meshes for both sides live under `meshes/l6_right` and `meshes/l6_left`, derived from the vendor's Apache-2.0 [linkerhand-urdf](https://github.com/linker-bot/linkerhand-urdf) release. The palm mesh is decimated to 30k triangles with the bounding box preserved.

## Tests

`test/test_urdf.py` renders the xacro for both sides, with and without a prefix, and both hardware plugins, checks joint types, mimic tags, pad and tcp frames, mesh paths, and the `ros2_control` block, and runs `check_urdf` on the result.
