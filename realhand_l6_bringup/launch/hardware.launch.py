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

Default brings up the driver, joint state broadcaster, and the contact gated
controller. With monitor_only:=true the read only tactile_monitor controller
runs instead and nothing commands motion, which is the first thing to run
on a new hand.

  ros2 launch realhand_l6_bringup hardware.launch.py can_interface:=can0
  ros2 launch realhand_l6_bringup hardware.launch.py monitor_only:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from realhand_l6_bringup import stack


def generate_launch_description():
    monitor_only = LaunchConfiguration('monitor_only')
    description = stack.robot_description(
        hardware='real',
        side=LaunchConfiguration('side'),
        can_interface=LaunchConfiguration('can_interface'),
        can_id=LaunchConfiguration('can_id'),
        enable_tactile=LaunchConfiguration('enable_tactile'),
        taxel_topic=LaunchConfiguration('taxel_topic'))
    spawn_contact = Node(
        package='controller_manager', executable='spawner',
        arguments=['contact_gated_controller', '-c', '/controller_manager'],
        output='screen', condition=UnlessCondition(monitor_only))
    spawn_monitor = Node(
        package='controller_manager', executable='spawner',
        arguments=['tactile_monitor', '-c', '/controller_manager'],
        output='screen', condition=IfCondition(monitor_only))
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right', choices=['right', 'left']),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_id', default_value='',
                              description='explicit CAN id, empty selects the side default'),
        DeclareLaunchArgument('enable_tactile', default_value='true'),
        DeclareLaunchArgument('taxel_topic', default_value='',
                              description='publish raw taxel grids as JSON on this topic'),
        DeclareLaunchArgument('monitor_only', default_value='false',
                              description='read only tactile monitor, no motion'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        *stack.control_nodes(description, controllers=('joint_state_broadcaster',)),
        spawn_contact,
        spawn_monitor,
        *stack.viz_nodes(controller=PythonExpression([
            "'tactile_monitor' if '", monitor_only,
            "' == 'true' else 'contact_gated_controller'"])),
    ])
