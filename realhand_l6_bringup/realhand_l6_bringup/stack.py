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

# Close target for the mock demo, the grasp configuration around the
# object cube from joint_state_zeros.yaml in the MuJoCoDex standalone
# scene, plus a small overshoot per gated joint so every finger is still
# moving when its contact angle fires. The contact angles in
# mock_contact_surface sit just under the grasp values, so each finger
# grips as it arrives at the designed configuration. The thumb reaches
# the cube through the opposition swing, so the mock gates the thumb on
# cmc_yaw, see mock_controllers.yaml, and parks cmc_pitch at zero.
MOCK_CLOSE_TARGET = [0.0, 1.25, 0.75, 1.0, 1.0, 0.78]

# Demo object cube in the hand base frame, one source for the description
# xacro and RViz. Size and transform follow the grasp scene in
# realhand_l6_right_standalone.urdf.xacro from MuJoCoDex, which pairs
# the cube with the joint_state_zeros.yaml grasp configuration, using
# the adjusted cube pose from the view_hand.launch.py docstring there.
OBJECT_SIZE = 0.055
OBJECT_XYZ = (0.065, 0.005, 0.08)


def robot_description(hardware, side='right', can_interface='can0', can_id='',
                      enable_tactile='true', taxel_topic='', use_object='false'):
    """robot_description parameter from the description package xacro."""
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('realhand_l6_description'), 'urdf', 'realhand_l6.urdf.xacro'])
    return {
        'robot_description': ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file,
                ' hardware:=', hardware, ' side:=', side, ' can_interface:=', can_interface,
                ' can_id:=', can_id, ' enable_tactile:=', enable_tactile,
                ' taxel_topic:=', taxel_topic, ' use_object:=', use_object,
                ' object_size:=', str(OBJECT_SIZE),
                ' object_xyz:="', ' '.join(str(v) for v in OBJECT_XYZ), '"']),
            value_type=str)
    }


def controllers_yaml():
    return PathJoinSubstitution(
        [FindPackageShare('realhand_l6_bringup'), 'config', 'controllers.yaml'])


def mock_controllers_yaml():
    """Return the mock demo overrides layered over controllers.yaml."""
    return PathJoinSubstitution(
        [FindPackageShare('realhand_l6_bringup'), 'config', 'mock_controllers.yaml'])


def control_nodes(description,
                  controllers=('joint_state_broadcaster', 'contact_gated_controller'),
                  extra_params=()):
    """ros2_control_node, robot_state_publisher, and one spawner per controller.

    extra_params lists parameter files layered over controllers.yaml,
    later files win.
    """
    nodes = [
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[description, controllers_yaml(), *extra_params], output='screen'),
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


def auto_close(delay_arg='close_delay', condition_arg='auto_close', target=None):
    """Publish the close request after a delay so a demo runs unattended.

    The request goes out twice one second apart because a single publish
    from a short lived node can vanish in the publisher teardown race.
    The controller treats the repeated identical request as a no op.
    """
    names = '[' + ', '.join(JOINTS) + ']'
    positions = '[' + ', '.join(str(v) for v in (target or CLOSE_TARGET)) + ']'
    return TimerAction(
        period=LaunchConfiguration(delay_arg),
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--times', '2', '--rate', '1.0', '-w', '1',
                 '/contact_gated_controller/close_to',
                 'sensor_msgs/msg/JointState', f'{{name: {names}, position: {positions}}}'],
            output='screen', condition=IfCondition(LaunchConfiguration(condition_arg)))])
