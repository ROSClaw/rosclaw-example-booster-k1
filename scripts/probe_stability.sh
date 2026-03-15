#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

FORWARD_SEC="${FORWARD_SEC:-5.0}"
FORWARD_LINEAR_X="${FORWARD_LINEAR_X:-0.12}"
TURN_SEC="${TURN_SEC:-4.0}"
TURN_LINEAR_X="${TURN_LINEAR_X:-0.08}"
TURN_ANGULAR_Z="${TURN_ANGULAR_Z:-0.20}"
SETTLE_SEC="${SETTLE_SEC:-2.0}"
FALL_Z_THRESHOLD="${FALL_Z_THRESHOLD:-0.35}"
MAX_ROLL_THRESHOLD="${MAX_ROLL_THRESHOLD:-0.80}"
MAX_PITCH_THRESHOLD="${MAX_PITCH_THRESHOLD:-0.80}"

container_ros_bash "
python3 - <<'PY' \
    '${FORWARD_SEC}' \
    '${FORWARD_LINEAR_X}' \
    '${TURN_SEC}' \
    '${TURN_LINEAR_X}' \
    '${TURN_ANGULAR_Z}' \
    '${SETTLE_SEC}' \
    '${FALL_Z_THRESHOLD}' \
    '${MAX_ROLL_THRESHOLD}' \
    '${MAX_PITCH_THRESHOLD}'
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class StabilityProbe(Node):
    def __init__(self) -> None:
        super().__init__('rosclaw_k1_stability_probe')
        self.pub = self.create_publisher(Twist, '/k1/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/k1/odom', self._odom_cb, 10)
        self.samples: list[dict[str, float]] = []
        self.last_sample: dict[str, float] | None = None
        self.phase = 'startup'

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(ori.x, ori.y, ori.z, ori.w)
        sample = {
            't': time.time(),
            'phase': self.phase,
            'x': float(pos.x),
            'y': float(pos.y),
            'z': float(pos.z),
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
        }
        self.last_sample = sample
        self.samples.append(sample)

    def publish_for(self, phase: str, duration_sec: float, linear_x: float, angular_z: float) -> None:
        self.phase = phase
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        deadline = time.time() + duration_sec
        while time.time() < deadline:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def publish_stop(self, duration_sec: float) -> None:
        self.publish_for('stop', duration_sec, 0.0, 0.0)


forward_sec = float(sys.argv[1])
forward_linear_x = float(sys.argv[2])
turn_sec = float(sys.argv[3])
turn_linear_x = float(sys.argv[4])
turn_angular_z = float(sys.argv[5])
settle_sec = float(sys.argv[6])
fall_z_threshold = float(sys.argv[7])
max_roll_threshold = float(sys.argv[8])
max_pitch_threshold = float(sys.argv[9])

rclpy.init(args=None)
node = StabilityProbe()
try:
    deadline = time.time() + 10.0
    while node.last_sample is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.last_sample is None:
        raise RuntimeError('timed out waiting for /k1/odom')

    node.publish_stop(1.0)
    start_sample = node.last_sample
    node.publish_for('forward', forward_sec, forward_linear_x, 0.0)
    node.publish_stop(1.0)
    node.publish_for('turn', turn_sec, turn_linear_x, turn_angular_z)
    node.publish_for('settle', settle_sec, 0.0, 0.0)
    end_sample = node.last_sample

    if start_sample is None or end_sample is None:
        raise RuntimeError('missing odom samples')

    min_z = min(sample['z'] for sample in node.samples)
    max_abs_roll = max(abs(sample['roll']) for sample in node.samples)
    max_abs_pitch = max(abs(sample['pitch']) for sample in node.samples)
    breach_sample = next(
        (
            sample
            for sample in node.samples
            if (
                sample['z'] < fall_z_threshold
                or abs(sample['roll']) > max_roll_threshold
                or abs(sample['pitch']) > max_pitch_threshold
            )
        ),
        None,
    )
    planar_displacement = math.dist(
        (start_sample['x'], start_sample['y']),
        (end_sample['x'], end_sample['y']),
    )
    fell = (
        min_z < fall_z_threshold
        or max_abs_roll > max_roll_threshold
        or max_abs_pitch > max_pitch_threshold
    )
    result = {
        'start': start_sample,
        'end': end_sample,
        'planar_displacement_m': planar_displacement,
        'min_z_m': min_z,
        'max_abs_roll_rad': max_abs_roll,
        'max_abs_pitch_rad': max_abs_pitch,
        'fell': fell,
        'first_breach': breach_sample,
        'thresholds': {
            'fall_z_m': fall_z_threshold,
            'max_roll_rad': max_roll_threshold,
            'max_pitch_rad': max_pitch_threshold,
        },
        'profile': {
            'forward_sec': forward_sec,
            'forward_linear_x': forward_linear_x,
            'turn_sec': turn_sec,
            'turn_linear_x': turn_linear_x,
            'turn_angular_z': turn_angular_z,
            'settle_sec': settle_sec,
        },
        'sample_count': len(node.samples),
    }
    print(json.dumps(result, indent=2))
    if fell:
        raise SystemExit('stability probe detected a fall or near-fall condition')
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
"
