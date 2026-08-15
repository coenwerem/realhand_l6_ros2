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

"""Full stack on mock hardware, no CAN bus, no sudo.

mock_components/GenericSystem stands in for the hand and mirrors position
commands back as state. A forward command controller owns the fake tactile
command interfaces and mock_force_ramp presses the pads one finger at a
time, so a close request latches finger by finger in RViz.

  ros2 launch realhand_l6_bringup mock.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from realhand_l6_bringup import stack


def generate_launch_description():
    description = stack.robot_description(hardware='mock')
    ramp = Node(
        package='realhand_l6_bringup', executable='mock_force_ramp', output='screen',
        parameters=[{
            'start_delay': LaunchConfiguration('contact_delay'),
            'stagger': LaunchConfiguration('stagger'),
        }],
        condition=IfCondition(LaunchConfiguration('mock_ramp')))
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_close', default_value='true',
                              description='publish one close request after close_delay'),
        DeclareLaunchArgument('close_delay', default_value='3.0'),
        DeclareLaunchArgument('contact_delay', default_value='5.0',
                              description='seconds before the mock pads start pressing'),
        DeclareLaunchArgument('stagger', default_value='0.6',
                              description='seconds between one finger touching and the next'),
        DeclareLaunchArgument('mock_ramp', default_value='true',
                              description='run mock_force_ramp, false leaves the pads to a test'),
        *stack.control_nodes(description, controllers=(
            'joint_state_broadcaster', 'tactile_mock_controller', 'contact_gated_controller')),
        # Loaded inactive so a controller switch can hand the position
        # interfaces to a trajectory client, the path a planner takes.
        Node(package='controller_manager', executable='spawner',
             arguments=['hand_trajectory_controller', '-c', '/controller_manager', '--inactive'],
             output='screen'),
        ramp,
        stack.auto_close(),
        *stack.viz_nodes(),
    ])
