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

"""MoveIt for the standalone hand, on mock hardware or the real driver."""

# Brings up ros2_control with the joint state broadcaster and the hand
# trajectory controller, move_group with the hand SRDF, and RViz with the
# MotionPlanning panel. Plan and execute between the open, pinch, and power
# states of the hand group, or drive one finger chain to a pose.
#
#   ros2 launch realhand_l6_moveit_config demo.launch.py
#   ros2 launch realhand_l6_moveit_config demo.launch.py hardware:=real can_interface:=can0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    hardware = LaunchConfiguration('hardware').perform(context)
    side = LaunchConfiguration('side').perform(context)
    can_interface = LaunchConfiguration('can_interface').perform(context)
    description_share = get_package_share_directory('realhand_l6_description')
    moveit_share = get_package_share_directory('realhand_l6_moveit_config')

    moveit_config = (
        MoveItConfigsBuilder('realhand_l6', package_name='realhand_l6_moveit_config')
        .robot_description(
            file_path=os.path.join(description_share, 'urdf', 'realhand_l6.urdf.xacro'),
            mappings={'hardware': hardware, 'side': side, 'can_interface': can_interface})
        .robot_description_semantic(
            file_path=os.path.join(moveit_share, 'srdf', 'realhand_l6.srdf.xacro'))
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('realhand_l6_bringup'), 'config', 'controllers.yaml'])
    rviz_config = os.path.join(moveit_share, 'rviz', 'moveit.rviz')

    return [
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[moveit_config.robot_description, controllers_yaml], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[moveit_config.robot_description], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '-c', '/controller_manager'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['hand_trajectory_controller', '-c', '/controller_manager'],
             output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[moveit_config.to_dict()]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config], output='screen',
             parameters=[moveit_config.robot_description,
                         moveit_config.robot_description_semantic,
                         moveit_config.robot_description_kinematics,
                         moveit_config.planning_pipelines,
                         moveit_config.joint_limits],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='mock', choices=['mock', 'real']),
        DeclareLaunchArgument('side', default_value='right', choices=['right', 'left']),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
