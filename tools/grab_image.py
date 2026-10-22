#!/usr/bin/env python3
"""Save one sensor_msgs/Image from a ROS topic as a binary PPM (no extra deps).
usage: python3 tools/grab_image.py <topic> <out.ppm> [timeout_s]"""
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    topic, out = sys.argv[1], sys.argv[2]
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
    rclpy.init()
    node = rclpy.create_node('grab_image')
    got = []
    node.create_subscription(Image, topic, got.append, qos_profile_sensor_data)
    t0 = time.time()
    while not got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not got:
        print(f'no image on {topic}')
        sys.exit(1)
    m = got[-1]
    ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(m.encoding, 3)
    data = bytes(m.data)
    rows = [data[r * m.step: r * m.step + m.width * ch] for r in range(m.height)]
    px = b''.join(rows)
    if ch == 4:
        px = b''.join(px[i:i + 3] for i in range(0, len(px), 4))
    if m.encoding.startswith('bgr'):
        px = b''.join(px[i:i + 3][::-1] for i in range(0, len(px), 3))
    with open(out, 'wb') as f:
        f.write(f'P6 {m.width} {m.height} 255\n'.encode() + px)
    print(f'saved {out} ({m.width}x{m.height} {m.encoding})')


if __name__ == '__main__':
    main()
