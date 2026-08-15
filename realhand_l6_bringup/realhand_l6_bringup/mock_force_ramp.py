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

"""Ramp fake pad forces into the mock hardware.

Publishes one Float64MultiArray per cycle to the forward command controller
that owns the mock tactile command interfaces, so the contact gated
controller sees a press build up on the chosen fingers. Fingers are hit one
after another so the latching order is visible.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class MockForceRamp(Node):

    def __init__(self):
        super().__init__('mock_force_ramp')
        self.declare_parameter('topic', '/tactile_mock_controller/commands')
        self.declare_parameter('num_fingers', 5)
        self.declare_parameter('fingers', [0, 1, 2, 3, 4])
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('stagger', 0.5)
        self.declare_parameter('ramp', 1.0)
        self.declare_parameter('full_force', 18360.0)
        self.declare_parameter('rate', 50.0)

        self.n = int(self.get_parameter('num_fingers').value)
        self.fingers = [int(f) for f in self.get_parameter('fingers').value]
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.stagger = float(self.get_parameter('stagger').value)
        self.ramp = max(1e-3, float(self.get_parameter('ramp').value))
        self.full = float(self.get_parameter('full_force').value)
        self.pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('topic').value, 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._tick)

    def _tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        msg = Float64MultiArray()
        msg.data = [0.0] * self.n
        for order, f in enumerate(self.fingers):
            start = self.start_delay + order * self.stagger
            frac = min(1.0, max(0.0, (t - start) / self.ramp))
            if 0 <= f < self.n:
                msg.data[f] = self.full * frac
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MockForceRamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
