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

"""End to end run of the real driver on a virtual CAN bus.

Needs vcan0 up (scripts/setup_vcan.sh). Without it the test is skipped, so
the suite passes on machines and runners with no CAN support. With it, the
emulated hand answers realhand_hardware over SocketCAN, contact ramps in on
every finger, and a close request latches all five with the joint state
frozen short of the target.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

VCAN = os.environ.get('REALHAND_TEST_VCAN', 'vcan0')
JOINTS = ['thumb_cmc_pitch', 'thumb_cmc_yaw', 'index_mcp_pitch', 'middle_mcp_pitch',
          'ring_mcp_pitch', 'pinky_mcp_pitch']


def vcan_available():
    return os.path.isdir(f'/sys/class/net/{VCAN}')


@pytest.mark.launch_test
def generate_test_description():
    actions = []
    if vcan_available():
        launch_file = os.path.join(
            get_package_share_directory('realhand_l6_bringup'), 'launch', 'vcan.launch.py')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'can_interface': VCAN, 'use_rviz': 'false',
                              'auto_close': 'false', 'contact_delay': '2.0',
                              'ramp': '0.5'}.items()))
    else:
        # launch_testing needs a live process under test while the skip runs.
        actions.append(ExecuteProcess(cmd=['sleep', '600'], name='vcan_absent'))
    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions)


class Probe(Node):

    def __init__(self):
        super().__init__('vcan_pipeline_probe')
        self.joint_positions = {}
        self.contact = None
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.create_subscription(
            Int32MultiArray, '/contact_gated_controller/contact_state', self._on_contact, 10)
        self.close_pub = self.create_publisher(
            JointState, '/contact_gated_controller/close_to', 10)

    def _on_js(self, msg):
        self.joint_positions = dict(zip(msg.name, msg.position))

    def _on_contact(self, msg):
        self.contact = list(msg.data)

    def wait_until(self, predicate, timeout, what, repeat=None):
        end = time.time() + timeout
        while time.time() < end:
            if repeat is not None:
                repeat()
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise AssertionError(f'timed out waiting for {what}')

    def close_to(self, target):
        msg = JointState()
        msg.name = JOINTS
        msg.position = [target] * len(JOINTS)
        self.close_pub.publish(msg)


class TestVcanPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = Probe()

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()

    def test_driver_decodes_emulated_hand(self):
        if not vcan_available():
            self.skipTest(f'{VCAN} is not up, run scripts/setup_vcan.sh to enable this test')
        p = self.probe
        p.wait_until(lambda: p.contact is not None and 'thumb_cmc_pitch' in p.joint_positions,
                     timeout=30.0, what='driver and controllers to come up')
        # The emulator reports every joint fully open, raw 255, which decodes
        # to zero radians.
        for j in JOINTS:
            self.assertAlmostEqual(p.joint_positions[j], 0.0, places=3)
        # Every pad ramps to full pressure after the emulator's delay.
        p.wait_until(lambda: all(c == 2 for c in p.contact), timeout=10.0,
                     what='all five pads sensed through the CAN decode path')
        # A close with contact already present latches every finger where it
        # stands, and the emulator echoes commands, so positions stay near zero
        # while opposition (ungated) travels to its target.
        p.wait_until(lambda: all(c == 1 for c in p.contact), timeout=10.0,
                     what='all fingers latched', repeat=lambda: p.close_to(1.0))
        p.wait_until(lambda: abs(p.joint_positions['thumb_cmc_yaw'] - 1.0) < 0.02,
                     timeout=10.0, what='thumb opposition to reach target')
        for j in ('index_mcp_pitch', 'middle_mcp_pitch', 'ring_mcp_pitch', 'pinky_mcp_pitch'):
            self.assertLess(p.joint_positions[j], 0.1)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15, 130])
