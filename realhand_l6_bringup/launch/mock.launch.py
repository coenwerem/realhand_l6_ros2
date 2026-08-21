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
command interfaces and mock_contact_surface converts finger positions into
pad forces, emulating an object surface at a per finger depth. A close
request sweeps all fingers together and each finger latches where it
meets the virtual object, as on the physical hand. use_object renders
the object as a cube fixed to the hand base, sized and posed so the
default contact angles latch the finger pads on its near face.

  ros2 launch realhand_l6_bringup mock.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from realhand_l6_bringup import stack


def generate_launch_description():
    description = stack.robot_description(
        hardware='mock', use_object=LaunchConfiguration('use_object'))
    surface = Node(
        package='realhand_l6_bringup', executable='mock_contact_surface', output='screen',
        condition=IfCondition(LaunchConfiguration('mock_ramp')))
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_close', default_value='true',
                              description='publish the close request after close_delay'),
        DeclareLaunchArgument('close_delay', default_value='3.0'),
        DeclareLaunchArgument(
            'use_object', default_value='true',
            description='render the virtual object the mock surface emulates'),
        DeclareLaunchArgument(
            'mock_ramp', default_value='true',
            description='run mock_contact_surface, false leaves the pads to a test'),
        *stack.control_nodes(
            description,
            controllers=('joint_state_broadcaster', 'tactile_mock_controller',
                         'contact_gated_controller'),
            extra_params=[stack.mock_controllers_yaml()]),
        # Loaded inactive so a controller switch can hand the position
        # interfaces to a trajectory client, the path a planner takes.
        Node(package='controller_manager', executable='spawner',
             arguments=['hand_trajectory_controller', '-c', '/controller_manager', '--inactive'],
             output='screen'),
        surface,
        stack.auto_close(target=stack.MOCK_CLOSE_TARGET),
        *stack.viz_nodes(),
    ])
