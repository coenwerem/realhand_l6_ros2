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

"""Emulate a RealHand L6 on a virtual CAN bus.

Answers position requests with the last commanded position and streams
taxel matrix rows for every finger, so realhand_hardware runs its real
decode path with no hand attached. Pad pressure ramps up on the chosen
fingers after a delay, so a commanded close latches finger by finger.

Bring up the bus first with scripts/setup_vcan.sh, then

  ros2 run realhand_l6_bringup mock_can_feeder --interface vcan0
"""

import argparse
import threading
import time

import can

FRAME_POSITION = 0x01
FRAME_MATRIX_BASE = 0xB1
NUM_JOINTS = 6
NUM_FINGERS = 5
TAXEL_ROWS = 12
TAXEL_COLS = 6
OPEN_RAW = 255


class Emulator:

    def __init__(self, bus, can_id, contact_fingers, delay, ramp):
        self.bus = bus
        self.can_id = can_id
        self.contact_fingers = contact_fingers
        self.delay = delay
        self.ramp = max(ramp, 1e-3)
        self.position = [OPEN_RAW] * NUM_JOINTS
        self.lock = threading.Lock()
        self.t0 = time.time()

    def send(self, data):
        self.bus.send(can.Message(arbitration_id=self.can_id, is_extended_id=False, data=data))

    def pressure(self):
        t = time.time() - self.t0 - self.delay
        return min(1.0, max(0.0, t / self.ramp))

    def listen(self):
        # Position command frames update the emulated position, a one byte
        # position request gets the current position back. Matrix requests
        # get twelve rows for that finger.
        while True:
            msg = self.bus.recv(timeout=0.1)
            if msg is None or msg.arbitration_id != self.can_id or len(msg.data) == 0:
                continue
            kind = msg.data[0]
            if kind == FRAME_POSITION:
                if len(msg.data) == 1 + NUM_JOINTS:
                    with self.lock:
                        self.position = list(msg.data[1:1 + NUM_JOINTS])
                elif len(msg.data) == 1:
                    with self.lock:
                        self.send(bytes([FRAME_POSITION] + self.position))
            elif (FRAME_MATRIX_BASE <= kind < FRAME_MATRIX_BASE + NUM_FINGERS
                    and len(msg.data) == 2):
                finger = kind - FRAME_MATRIX_BASE
                value = int(255 * self.pressure()) if finger in self.contact_fingers else 0
                for row in range(TAXEL_ROWS):
                    self.send(bytes([kind, row * 16] + [value] * TAXEL_COLS))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--interface', default='vcan0')
    parser.add_argument('--can-id', type=lambda x: int(x, 0), default=0x27,
                        help='hand CAN id, 0x27 right, 0x28 left')
    parser.add_argument('--fingers', default='0,1,2,3,4',
                        help='comma list of finger indices that make contact, 0 is the thumb')
    parser.add_argument('--contact-delay', type=float, default=3.0)
    parser.add_argument('--ramp', type=float, default=2.0)
    args, _ = parser.parse_known_args()

    fingers = {int(x) for x in args.fingers.split(',') if x.strip()}
    bus = can.interface.Bus(channel=args.interface, interface='socketcan')
    emu = Emulator(bus, args.can_id, fingers, args.contact_delay, args.ramp)
    print(f'emulating L6 on {args.interface} id 0x{args.can_id:02x}, '
          f'contact on fingers {sorted(fingers)} after {args.contact_delay:.1f}s')
    try:
        emu.listen()
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
