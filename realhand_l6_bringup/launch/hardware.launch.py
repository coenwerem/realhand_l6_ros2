# Copyright 2026 Clinton Enwerem
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Real hand on a real CAN bus.

The controller argument picks what drives the hand. contact runs the
contact gated controller, trajectory a joint_trajectory_controller for
planners and scripts, position a forward position controller, and monitor
the read only tactile monitor that commands nothing, the first thing to run
on a new hand. setpoint_controllers adds forward controllers on the driver's
speed and torque command interfaces next to whichever controller runs.

  ros2 launch realhand_l6_bringup hardware.launch.py can_interface:=can0
  ros2 launch realhand_l6_bringup hardware.launch.py controller:=monitor
  ros2 launch realhand_l6_bringup hardware.launch.py controller:=trajectory
  ros2 launch realhand_l6_bringup hardware.launch.py setpoint_controllers:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from realhand_l6_bringup import stack

CONTROLLERS = {
    'contact': 'contact_gated_controller',
    'trajectory': 'hand_trajectory_controller',
    'position': 'hand_position_controller',
    'monitor': 'tactile_monitor',
}


def generate_launch_description():
    controller = LaunchConfiguration('controller')
    description = stack.robot_description(
        hardware='real',
        side=LaunchConfiguration('side'),
        can_interface=LaunchConfiguration('can_interface'),
        can_id=LaunchConfiguration('can_id'),
        enable_tactile=LaunchConfiguration('enable_tactile'),
        taxel_topic=LaunchConfiguration('taxel_topic'))
    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=[name, '-c', '/controller_manager'], output='screen',
             condition=IfCondition(PythonExpression(["'", controller, "' == '", key, "'"])))
        for key, name in CONTROLLERS.items()]
    setpoint_spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=[name, '-c', '/controller_manager'], output='screen',
             condition=IfCondition(LaunchConfiguration('setpoint_controllers')))
        for name in ('hand_speed_controller', 'hand_torque_controller')]
    # The contact visualizer follows whichever controller publishes contact.
    viz_controller = PythonExpression([
        "'tactile_monitor' if '", controller, "' == 'monitor' else 'contact_gated_controller'"])
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right', choices=['right', 'left']),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_id', default_value='',
                              description='explicit CAN id, empty selects the side default'),
        DeclareLaunchArgument('enable_tactile', default_value='true'),
        DeclareLaunchArgument('taxel_topic', default_value='',
                              description='publish raw taxel grids as JSON on this topic'),
        DeclareLaunchArgument('controller', default_value='contact',
                              choices=list(CONTROLLERS.keys()),
                              description='which controller drives the hand'),
        DeclareLaunchArgument('setpoint_controllers', default_value='false',
                              description='also spawn the speed and torque forward controllers'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        *stack.control_nodes(description, controllers=('joint_state_broadcaster',)),
        *spawners,
        *setpoint_spawners,
        *stack.viz_nodes(controller=viz_controller),
    ])
