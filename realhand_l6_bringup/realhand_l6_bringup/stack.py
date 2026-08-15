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

"""Launch building blocks shared by the mock, vcan, and hardware launch files."""

from launch.actions import ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

JOINTS = ['thumb_cmc_pitch', 'thumb_cmc_yaw', 'index_mcp_pitch', 'middle_mcp_pitch',
          'ring_mcp_pitch', 'pinky_mcp_pitch']
CLOSE_TARGET = [0.5, 1.2, 1.4, 1.4, 1.4, 1.4]


def robot_description(hardware, side='right', can_interface='can0', can_id='',
                      enable_tactile='true', taxel_topic=''):
    """robot_description parameter from the description package xacro."""
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('realhand_l6_description'), 'urdf', 'realhand_l6.urdf.xacro'])
    return {
        'robot_description': ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file,
                ' hardware:=', hardware, ' side:=', side, ' can_interface:=', can_interface,
                ' can_id:=', can_id, ' enable_tactile:=', enable_tactile,
                ' taxel_topic:=', taxel_topic]),
            value_type=str)
    }


def controllers_yaml():
    return PathJoinSubstitution(
        [FindPackageShare('realhand_l6_bringup'), 'config', 'controllers.yaml'])


def control_nodes(description,
                  controllers=('joint_state_broadcaster', 'contact_gated_controller')):
    """ros2_control_node, robot_state_publisher, and one spawner per controller."""
    nodes = [
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[description, controllers_yaml()], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[description], output='screen'),
    ]
    for name in controllers:
        nodes.append(Node(package='controller_manager', executable='spawner',
                          arguments=[name, '-c', '/controller_manager'], output='screen'))
    return nodes


def viz_nodes(condition_arg='use_rviz', controller='contact_gated_controller'):
    """Return RViz with the hand config plus the contact marker node."""
    condition = IfCondition(LaunchConfiguration(condition_arg))
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('realhand_l6_description'), 'rviz', 'view_hand.rviz'])
    return [
        Node(package='realhand_l6_bringup', executable='contact_viz',
             parameters=[{'controller': controller}], output='screen', condition=condition),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             output='screen', condition=condition),
    ]


def auto_close(delay_arg='close_delay', condition_arg='auto_close'):
    """Publish one close request after a delay so a demo runs unattended."""
    names = '[' + ', '.join(JOINTS) + ']'
    positions = '[' + ', '.join(str(v) for v in CLOSE_TARGET) + ']'
    return TimerAction(
        period=LaunchConfiguration(delay_arg),
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/contact_gated_controller/close_to',
                 'sensor_msgs/msg/JointState', f'{{name: {names}, position: {positions}}}'],
            output='screen', condition=IfCondition(LaunchConfiguration(condition_arg)))])
