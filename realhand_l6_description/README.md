# realhand_l6_description

`realhand_l6_description` provides the URDF/Xacro description, meshes, `ros2_control` description, and RViz configuration for right and left RealHand L6 hands. Repository-level demos and launch instructions are available in the [top-level README](../README.md).

## Xacro Macros

### Hand Description

`urdf/realhand_l6_macro.xacro` defines the `realhand_l6` macro with parameters `prefix`, `side` (`right` or `left`), `parent`, and an `origin` block. The macro expands to the palm and finger links, vendor-derived meshes, six actuated revolute joints, five mimic distal joints, a tool-center-point frame, and one tactile-pad frame per finger.

The `tcp` frame is located 40 mm above the palm. Each `<finger>_pad` frame is attached to its distal link and oriented with +z pointing outward from the tactile surface toward the grasp region, +x along the finger, and +y across the 12-by-6 taxel array.

The left-hand model mirrors the physically validated right-hand geometry and matches the vendor left URDF joint by joint. The mirrored model negates the appropriate y coordinates, roll and yaw angles, and thumb-yaw axis direction. Both sides currently use the calibrated right-hand joint limits so the URDF and driver model table remain consistent. Those limits have not been validated on a physical left L6.

### ROS 2 Control Description

`urdf/realhand_l6.ros2_control.xacro` defines `realhand_l6_ros2_control` with parameters `name`, `prefix`, `side`, `hardware` (`mock` or `real`), `can_interface`, `can_id`, `enable_tactile`, `taxel_topic`, and `setpoints`.

With `hardware:=mock`, the macro loads `mock_components/GenericSystem` with `mock_sensor_commands`, allowing tests and demos to drive tactile values through mock command interfaces. With `hardware:=real`, it loads `realhand_hardware/RealHandSystem` and forwards the corresponding CAN and tactile parameters. `setpoints`, enabled by default, adds `speed` and `torque` command interfaces to each actuated joint in addition to position control.

### Standalone Hand Description

`urdf/realhand_l6.urdf.xacro` instantiates the hand on a `world` link for standalone visualization and testing. It accepts `prefix`, `side`, `hardware` (`none`, `mock`, or `real`), `can_interface`, `can_id`, `enable_tactile`, `taxel_topic`, and `setpoints`.

The optional `use_object` argument adds a visual-only fixed cube named `object_link` at the hand base. Its size, pose, and color are configured through `object_size`, `object_xyz`, `object_rpy`, and `object_rgba`. The mock contact-gated demo uses this cube as the visible counterpart to the surface modeled by `mock_contact_surface`; the default pose and contact depths are chosen to reproduce the demonstrated grasp configuration.

## View the Hand

```bash
ros2 launch realhand_l6_description view_hand.launch.py side:=left
```

## Meshes

STL meshes for both sides are stored under `meshes/l6_right` and `meshes/l6_left`. They are derived from the vendor's Apache-2.0 [linkerhand-urdf](https://github.com/linker-bot/linkerhand-urdf) release. The palm mesh is decimated to approximately 30,000 triangles while preserving its bounding box.

## Tests

`test/test_urdf.py` renders the Xacro for both hand sides, with and without joint prefixes, and with both mock and real hardware plugins. The test checks joint types, mimic relationships, tactile-pad and TCP frames, mesh paths, and the generated `ros2_control` block, then validates the resulting URDF with `check_urdf`.
