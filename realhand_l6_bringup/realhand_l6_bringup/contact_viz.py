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

"""Per finger contact markers for RViz.

Draws each tactile sensing window as a thin patch at the <finger>_pad frame
the description publishes, colored by the contact code from the controller,
and a floating label for fingers in contact or gripping.

  green   0  no contact
  yellow  2  contact sensed, finger still moving
  red     1  gripped, stopped on contact and holding
  purple  3  stopped at target with no contact, missed
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

PAD_LENGTH = 0.0144      # 12 taxels at 1.2 mm along the finger, local x
PAD_WIDTH = 0.0096       # 6 taxels at 1.6 mm across the pad, local y
PAD_THICKNESS = 0.0015   # rendered thickness along the pad normal, local z

COLORS = {
    0: (0.20, 0.80, 0.30),
    1: (0.90, 0.20, 0.20),
    2: (0.95, 0.85, 0.10),
    3: (0.60, 0.10, 0.80),
}
LABELS = {0: '', 1: 'contact', 2: 'contact', 3: 'missed'}


class ContactViz(Node):

    def __init__(self):
        super().__init__('contact_viz')
        self.declare_parameter('controller', 'contact_gated_controller')
        self.declare_parameter('finger_names', ['thumb', 'index', 'middle', 'ring', 'pinky'])
        self.declare_parameter('prefix', '')
        self.declare_parameter('text_scale', 0.008)

        controller = self.get_parameter('controller').value
        self.names = list(self.get_parameter('finger_names').value)
        self.prefix = self.get_parameter('prefix').value
        self.text_scale = float(self.get_parameter('text_scale').value)
        self.state = [0] * len(self.names)
        self.force = [0.0] * len(self.names)

        self.create_subscription(
            Int32MultiArray, f'/{controller}/contact_state', self._on_contact, 10)
        self.create_subscription(
            Float64MultiArray, f'/{controller}/finger_force', self._on_force, 10)
        self.pub = self.create_publisher(MarkerArray, '~/markers', 10)
        self.create_timer(0.1, self._publish)

    def _on_contact(self, msg):
        for i in range(min(len(self.names), len(msg.data))):
            self.state[i] = int(msg.data[i])

    def _on_force(self, msg):
        for i in range(min(len(self.names), len(msg.data))):
            self.force[i] = float(msg.data[i])

    def _publish(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, name in enumerate(self.names):
            frame = f'{self.prefix}{name}_pad'
            r, g, b = COLORS.get(self.state[i], COLORS[0])

            pad = Marker()
            pad.header.frame_id = frame
            pad.header.stamp = stamp
            pad.ns = 'pad'
            pad.id = i
            pad.type = Marker.CUBE
            pad.action = Marker.ADD
            pad.frame_locked = True
            pad.pose.orientation.w = 1.0
            pad.scale.x = PAD_LENGTH
            pad.scale.y = PAD_WIDTH
            pad.scale.z = PAD_THICKNESS
            pad.color.r, pad.color.g, pad.color.b, pad.color.a = r, g, b, 0.95
            arr.markers.append(pad)

            # RViz drops a text marker with an empty string as invalid and
            # holds the previous text, so an idle finger gets an explicit
            # DELETE for its label instead of an empty ADD.
            label = Marker()
            label.header.frame_id = frame
            label.header.stamp = stamp
            label.ns = 'labels'
            label.id = 100 + i
            label.type = Marker.TEXT_VIEW_FACING
            text = LABELS.get(self.state[i], '?')
            if text:
                label.action = Marker.ADD
                label.frame_locked = True
                label.pose.position.z = 0.015
                label.pose.orientation.w = 1.0
                label.scale.z = self.text_scale
                label.color.r = label.color.g = label.color.b = label.color.a = 1.0
                label.text = text
            else:
                label.action = Marker.DELETE
            arr.markers.append(label)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = ContactViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
