#!/usr/bin/env python3
"""Save drone camera frames to PNG for building a fine-tuning dataset (offline utility).

Run once per flight, with a new --run name each time, so train/val can be split by run:
  python3 save_frames.py --run run03_high_north --every 2
  python3 save_frames.py --run run04_no_bear --topic /parrot1/camera/image

Frames go to ~/grizzlyguard_frames/<run>/ (outside the repo; do not commit datasets).
"""
import argparse
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Saver(Node):
    def __init__(self, topic, out, every):
        super().__init__('frame_saver')
        self.out, self.every, self.n, self.saved = out, every, 0, 0
        self.bridge = CvBridge()
        self.create_subscription(Image, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'Saving every {every} frame(s) from {topic} to {out}')

    def cb(self, msg):
        self.n += 1
        if self.n % self.every:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        stamp = f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}'
        cv2.imwrite(str(self.out / f'{self.out.name}_{stamp}.png'), img)
        self.saved += 1
        if self.saved % 25 == 0:
            self.get_logger().info(f'{self.saved} frames saved')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True, help='name for this flight/scene, e.g. run03_low_east')
    ap.add_argument('--topic', default='/parrot1/camera/image')
    ap.add_argument('--every', type=int, default=1, help='keep every Nth frame')
    ap.add_argument('--root', default=str(Path.home() / 'grizzlyguard_frames'))
    a = ap.parse_args()
    out = Path(a.root) / a.run
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Saver(a.topic, out, max(1, a.every))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info(f'Done: {node.saved} frames in {out}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()