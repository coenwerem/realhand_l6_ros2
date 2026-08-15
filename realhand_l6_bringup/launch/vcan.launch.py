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

"""Real driver on a virtual CAN bus with an emulated hand.

Runs realhand_hardware against vcan0, where mock_can_feeder answers position
requests and streams taxel rows, so the whole decode path runs with no hand
attached. Bring the bus up first with scripts/setup_vcan.sh.

  ros2 launch realhand_l6_bringup vcan.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

from realhand_l6_bringup import stack


def generate_launch_description():
    can_interface = LaunchConfiguration('can_interface')
    description = stack.robot_description(hardware='real', can_interface=can_interface)
    feeder = ExecuteProcess(
        cmd=['ros2', 'run', 'realhand_l6_bringup', 'mock_can_feeder',
             '--interface', can_interface,
             '--contact-delay', LaunchConfiguration('contact_delay'),
             '--ramp', LaunchConfiguration('ramp')],
        output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='vcan0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_close', default_value='true'),
        DeclareLaunchArgument('close_delay', default_value='4.0'),
        DeclareLaunchArgument('contact_delay', default_value='6.0'),
        DeclareLaunchArgument('ramp', default_value='2.0'),
        # The emulator must answer before the driver activates and reads position.
        feeder,
        TimerAction(period=1.0, actions=stack.control_nodes(description)),
        stack.auto_close(),
        *stack.viz_nodes(),
    ])
