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

"""Read only tactile probe for a RealHand on SocketCAN.

Checks the driver's assumptions against the hand itself, with no ROS and no
motion. It sends only matrix read requests (0xb1 to 0xb5 with the mode
byte), never position, torque, or speed, and prints per finger matrix sums
and row rates. Press one pad at a time and read which frame id rises to
confirm the id to finger mapping and the resting baseline.

Run it with nothing else on the bus, so the rate you read is the hand's own.

  ros2 run realhand_l6_bringup can_tactile_probe --interface can0 --can-id 0x27
"""

import argparse
import collections
import time

import can

MATRIX_BASE = 0xB1
FINGERS = ['thumb', 'index', 'middle', 'ring', 'pinky']
ROWS, COLS = 12, 6


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--interface', default='can0')
    parser.add_argument('--can-id', type=lambda x: int(x, 0), default=0x27,
                        help='hand id, 0x27 right, 0x28 left')
    parser.add_argument('--mode-byte', type=lambda x: int(x, 0), default=0xC6,
                        help='matrix request parameter, 0xc6 on the L6')
    parser.add_argument('--period', type=float, default=0.5, help='print interval in seconds')
    parser.add_argument('--csv', default=None, help='optional path for per finger sums')
    args, _ = parser.parse_known_args()

    csv_f = open(args.csv, 'w', buffering=1) if args.csv else None
    if csv_f:
        csv_f.write('elapsed_s,' + ','.join(FINGERS) + '\n')

    bus = can.interface.Bus(channel=args.interface, interface='socketcan')
    reply_ids = {args.can_id, args.can_id + 8}
    ids = [MATRIX_BASE + i for i in range(len(FINGERS))]
    matrices = {i: [[0] * COLS for _ in range(ROWS)] for i in ids}
    rate = collections.Counter()
    t_start = t_last = time.time()
    rotation = 0

    print(f'probing {args.interface} id 0x{args.can_id:02x}, press a pad and watch its sum')
    try:
        while True:
            # Rotate the request order so no finger is always last. The hand's
            # readout is throughput limited and a fixed order starves the tail.
            order = ids[rotation:] + ids[:rotation]
            rotation = (rotation + 1) % len(ids)
            for frame_id in order:
                bus.send(can.Message(arbitration_id=args.can_id, is_extended_id=False,
                                     data=bytes([frame_id, args.mode_byte])))
            t0 = time.time()
            while time.time() - t0 < 0.05:
                msg = bus.recv(timeout=0.02)
                if msg is None or (msg.arbitration_id & 0x7FF) not in reply_ids:
                    continue
                d = msg.data
                if (len(d) >= 2 + COLS and d[0] in matrices and d[1] % 16 == 0
                        and d[1] // 16 < ROWS):
                    matrices[d[0]][d[1] // 16] = list(d[2:2 + COLS])
                    rate[d[0]] += 1

            now = time.time()
            if now - t_last >= args.period:
                dt = now - t_last
                sums = [sum(map(sum, matrices[i])) for i in ids]
                print(f'--- {dt:.2f}s window ---')
                for i, name, total in zip(ids, FINGERS, sums):
                    print(f'  0x{i:02x} {name:6s} sum={total:6d} rows/s={rate[i] / dt:6.1f}')
                if csv_f:
                    csv_f.write(f'{now - t_start:.1f},' + ','.join(map(str, sums)) + '\n')
                rate.clear()
                t_last = now
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()
        if csv_f:
            csv_f.close()


if __name__ == '__main__':
    main()
