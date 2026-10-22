#!/usr/bin/env python3
"""Save every sensor_msgs/Image on a topic as numbered PPM files until killed.
usage: python3 tools/record_frames.py <topic> <out_dir>"""
import os
import sys

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    topic, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('record_frames')
    count = [0]

    def save(m):
        ch = 4 if m.encoding in ('rgba8', 'bgra8') else 3
        data = bytes(m.data)
        px = b''.join(data[r * m.step: r * m.step + m.width * ch] for r in range(m.height))
        if ch == 4:
            px = b''.join(px[i:i + 3] for i in range(0, len(px), 4))
        with open(os.path.join(out, f'f{count[0]:05d}.ppm'), 'wb') as f:
            f.write(f'P6 {m.width} {m.height} 255\n'.encode() + px)
        count[0] += 1

    node.create_subscription(Image, topic, save, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print(f'{topic}: {count[0]} frames -> {out}', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
