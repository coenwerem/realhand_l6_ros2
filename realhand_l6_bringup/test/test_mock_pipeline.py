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

"""End to end run of the mock stack.

Launches mock.launch.py without RViz or the ramp, presses the index pad
through the forward command controller, requests a close, and checks that
the index finger latches in place while the other fingers reach the target
and report a miss. Then opens and checks every joint returns to zero.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Int32MultiArray

JOINTS = ['thumb_cmc_pitch', 'thumb_cmc_yaw', 'index_mcp_pitch', 'middle_mcp_pitch',
          'ring_mcp_pitch', 'pinky_mcp_pitch']
FULL_FORCE = 18360.0


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('realhand_l6_bringup'), 'launch', 'mock.launch.py')
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={'use_rviz': 'false', 'auto_close': 'false',
                          'mock_ramp': 'false'}.items())
    return launch.LaunchDescription([stack, launch_testing.actions.ReadyToTest()])


class Probe(Node):

    def __init__(self):
        super().__init__('mock_pipeline_probe')
        self.joint_positions = {}
        self.contact = None
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.create_subscription(
            Int32MultiArray, '/contact_gated_controller/contact_state', self._on_contact, 10)
        self.force_pub = self.create_publisher(
            Float64MultiArray, '/tactile_mock_controller/commands', 10)
        self.close_pub = self.create_publisher(
            JointState, '/contact_gated_controller/close_to', 10)
        self.open_pub = self.create_publisher(Bool, '/contact_gated_controller/open', 10)

    def _on_js(self, msg):
        self.joint_positions = dict(zip(msg.name, msg.position))

    def _on_contact(self, msg):
        self.contact = list(msg.data)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, predicate, timeout, what, repeat=None):
        """Spin until predicate holds, calling repeat each loop when given.

        Publishers created by this probe may not be matched with their
        subscribers yet, so requests are re-sent until their effect shows.
        """
        end = time.time() + timeout
        while time.time() < end:
            if repeat is not None:
                repeat()
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise AssertionError(f'timed out waiting for {what}')

    def press(self, forces):
        self.force_pub.publish(Float64MultiArray(data=forces))

    def close_to(self, target):
        msg = JointState()
        msg.name = JOINTS
        msg.position = [target] * len(JOINTS)
        self.close_pub.publish(msg)

    def open_hand(self):
        self.open_pub.publish(Bool(data=True))


class TestMockPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()

    def test_close_latches_on_contact_then_opens(self):
        p = self.probe
        p.wait_until(lambda: p.contact is not None and 'index_mcp_pitch' in p.joint_positions,
                     timeout=30.0, what='controllers to come up')

        # Index pad already pressed when the close request arrives, so the
        # index finger latches at once and stays put.
        p.wait_until(lambda: p.contact[1] == 2, timeout=10.0, what='index contact sensed',
                     repeat=lambda: p.press([0.0, FULL_FORCE, 0.0, 0.0, 0.0]))
        index_before = p.joint_positions['index_mcp_pitch']
        p.wait_until(lambda: p.contact[1] == 1, timeout=10.0, what='index latched',
                     repeat=lambda: p.close_to(1.0))
        p.wait_until(lambda: p.contact[3] == 3, timeout=5.0, what='ring finger miss')
        p.spin_for(0.5)
        self.assertAlmostEqual(p.joint_positions['index_mcp_pitch'], index_before, places=2)
        for j in ('middle_mcp_pitch', 'ring_mcp_pitch', 'pinky_mcp_pitch', 'thumb_cmc_yaw'):
            self.assertAlmostEqual(p.joint_positions[j], 1.0, places=2)
        # Mimic joints follow their parents through the mock.
        self.assertAlmostEqual(p.joint_positions['ring_dip'], 0.89, places=2)

        p.wait_until(lambda: all(abs(p.joint_positions[j]) < 1e-3 for j in JOINTS),
                     timeout=10.0, what='hand to open', repeat=p.open_hand)
        p.spin_for(0.3)
        self.assertEqual(p.contact[1], 2)
        self.assertEqual(p.contact[3], 0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130])
