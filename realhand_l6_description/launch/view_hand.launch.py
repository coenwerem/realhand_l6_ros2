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

"""Show the hand in RViz with joint sliders. No hardware, no ros2_control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    side = LaunchConfiguration('side')
    prefix = LaunchConfiguration('prefix')
    description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([
                FindPackageShare('realhand_l6_description'), 'urdf', 'realhand_l6.urdf.xacro']),
            ' side:=', side, ' prefix:=', prefix, ' hardware:=none']),
        value_type=str)
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('realhand_l6_description'), 'rviz', 'view_hand.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right', choices=['right', 'left']),
        DeclareLaunchArgument('prefix', default_value=''),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config]),
    ])
