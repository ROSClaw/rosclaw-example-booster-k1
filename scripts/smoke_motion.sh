#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

LINEAR_X="${LINEAR_X:-0.18}"
ANGULAR_Z="${ANGULAR_Z:-0.00}"
DURATION_SEC="${DURATION_SEC:-2.0}"
SETTLE_SEC="${SETTLE_SEC:-2.0}"
PUBLISH_RATE="${PUBLISH_RATE:-10}"
MIN_DISTANCE="${MIN_DISTANCE:-0.05}"

container_ros_bash "
read_initial_pose() {
python3 - <<'PY'
import rclpy
from nav_msgs.msg import Odometry

result = {}

def callback(msg):
    result['x'] = msg.pose.pose.position.x
    result['y'] = msg.pose.pose.position.y

rclpy.init(args=None)
node = rclpy.create_node('rosclaw_k1_motion_probe')
subscription = node.create_subscription(Odometry, '/k1/odom', callback, 10)
try:
    deadline_ns = node.get_clock().now().nanoseconds + int(5e9)
    while rclpy.ok() and 'x' not in result:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.get_clock().now().nanoseconds > deadline_ns:
            raise RuntimeError('timed out waiting for /k1/odom')
    print(f\"{result['x']} {result['y']}\")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
}

initial_pose=\"\$(read_initial_pose)\"
echo \"initial pose: \${initial_pose}\"

set +e
timeout ${DURATION_SEC}s ros2 topic pub -r ${PUBLISH_RATE} /k1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: ${LINEAR_X}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${ANGULAR_Z}}}'
status=\$?
set -e
if [[ \${status} -ne 0 && \${status} -ne 124 ]]; then
    exit \${status}
fi

ros2 topic pub --once /k1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
sleep ${SETTLE_SEC}

final_pose=\"\$(read_initial_pose)\"
echo \"final pose: \${final_pose}\"

python3 - <<'PY' \"\${initial_pose}\" \"\${final_pose}\" \"${MIN_DISTANCE}\"
import math
import sys

initial = [float(value) for value in sys.argv[1].split()]
final = [float(value) for value in sys.argv[2].split()]
min_distance = float(sys.argv[3])

distance = math.dist(initial, final)
print(f'planar distance: {distance:.4f} m')
if distance < min_distance:
    raise SystemExit(f'robot did not move enough: {distance:.4f} m < {min_distance:.4f} m')
PY
"
