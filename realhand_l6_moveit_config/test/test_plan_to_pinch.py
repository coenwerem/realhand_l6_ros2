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

"""Plan and execute to the pinch state through move_group on mock hardware."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

PINCH = {'thumb_cmc_yaw': 1.248, 'thumb_cmc_pitch': 0.314, 'index_mcp_pitch': 0.816,
         'middle_mcp_pitch': 0.816, 'ring_mcp_pitch': 0.816, 'pinky_mcp_pitch': 0.816}


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('realhand_l6_moveit_config'), 'launch', 'demo.launch.py')
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={'use_rviz': 'false', 'hardware': 'mock'}.items())
    return launch.LaunchDescription([demo, launch_testing.actions.ReadyToTest()])


class Probe(Node):

    def __init__(self):
        super().__init__('moveit_probe')
        self.joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/hand_trajectory_controller/follow_joint_trajectory')

    def _on_js(self, msg):
        self.joint_positions = dict(zip(msg.name, msg.position))

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_to(self, targets, group='hand'):
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        constraints = Constraints()
        for name, value in targets.items():
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=value, tolerance_above=0.01, tolerance_below=0.01,
                weight=1.0))
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=30.0)
        handle = send.result()
        assert handle is not None and handle.accepted, 'move_group rejected the goal'
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=60.0)
        assert result.done(), 'move_group did not finish'
        return result.result().result.error_code.val


class TestPlanToPinch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()

    def test_plan_and_execute_pinch(self):
        p = self.probe
        self.assertTrue(p.client.wait_for_server(timeout_sec=90.0), 'move_group action server')
        # Execution goes through the trajectory controller, which spawns in
        # parallel with move_group, so wait for its action server too.
        self.assertTrue(p.trajectory_client.wait_for_server(timeout_sec=60.0),
                        'hand_trajectory_controller action server')
        p.spin_for(2.0)
        # Give the planning scene monitor its first joint state.
        end = time.time() + 30.0
        while time.time() < end and 'index_mcp_pitch' not in p.joint_positions:
            p.spin_for(0.2)
        code = p.move_to(PINCH)
        self.assertEqual(code, MoveItErrorCodes.SUCCESS)
        p.spin_for(1.0)
        # The goal tolerance is 0.01 rad per joint, so the executed end point
        # sits within 0.01 of the target.
        for name, value in PINCH.items():
            self.assertAlmostEqual(p.joint_positions[name], value, delta=0.02)
        # Mimic joints followed through the mock hardware.
        self.assertAlmostEqual(p.joint_positions['index_dip'], 0.816 * 0.89, delta=0.02)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -6, -15, 130])
