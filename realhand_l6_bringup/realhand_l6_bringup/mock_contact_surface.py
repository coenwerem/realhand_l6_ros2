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

"""Emulate an object surface for the mock hand's tactile pads.

Subscribes to joint states and publishes one force per pad to the forward
command controller owning the mock tactile command interfaces. Each pad
force rises once the finger's gating joint passes its contact angle, as
if the fingertip pressed into an object at a fixed depth. All fingers
sweep together and each finger latches where it meets the virtual
surface, matching the closing behavior of the physical hand. The default
contact angles sit just under the grasp configuration around the demo
object cube, taken from joint_state_zeros.yaml of the MuJoCoDex
standalone scene, so each finger latches as it arrives at the designed
grasp. The thumb meets the cube through the opposition swing, so its
gating joint is cmc_yaw, matching mock_controllers.yaml.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MockContactSurface(Node):

    def __init__(self):
        super().__init__('mock_contact_surface')
        self.declare_parameter('topic', '/tactile_mock_controller/commands')
        self.declare_parameter('joint_states_topic', '/joint_states')
        # Pad order, thumb to pinky, one gating joint per pad, matching the
        # flexion_joints of the contact gated controller in the mock demo.
        self.declare_parameter('flexion_joints', [
            'thumb_cmc_yaw', 'index_mcp_pitch', 'middle_mcp_pitch',
            'ring_mcp_pitch', 'pinky_mcp_pitch'])
        # Gating joint angle per finger at which the fingertip surface meets
        # the object cube. The finger angles sit 0.015 rad under the grasp
        # configuration in joint_state_zeros.yaml so each finger latches on
        # the designed grasp. The thumb latches earlier, where the thumb
        # link surface first meets the thumb side face of the cube during
        # the opposition swing. The posed yaw of 1.096 lies past the face
        # and would sweep the thumb through the cube's lower corner on the
        # way there. Retune together with the cube in stack.OBJECT_XYZ.
        self.declare_parameter('contact_angles', [0.865, 0.581, 0.846, 0.846, 0.616])
        # Penetration depth over which a pad force rises to full_force.
        self.declare_parameter('contact_width', 0.06)
        self.declare_parameter('full_force', 18360.0)
        self.declare_parameter('rate', 50.0)

        self.joints = [str(j) for j in self.get_parameter('flexion_joints').value]
        self.angles = [float(a) for a in self.get_parameter('contact_angles').value]
        if len(self.angles) != len(self.joints):
            raise ValueError('contact_angles needs one entry per flexion joint.')
        self.width = max(1e-4, float(self.get_parameter('contact_width').value))
        self.full = float(self.get_parameter('full_force').value)
        self.positions = {}
        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self._on_joint_states, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('topic').value, 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)

    def _on_joint_states(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def _tick(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * len(self.joints)
        for i, (joint, angle) in enumerate(zip(self.joints, self.angles)):
            q = self.positions.get(joint)
            if q is None:
                continue
            frac = (q - angle) / self.width
            msg.data[i] = self.full * min(1.0, max(0.0, frac))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MockContactSurface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
