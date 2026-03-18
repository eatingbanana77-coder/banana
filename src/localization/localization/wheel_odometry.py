"""
wheel_odometry.py — ERP42 Wheel Odometry Node

Subscribes to /erp42_feedback and /imu/data.
Publishes /odom with wall-clock timestamps.

  /erp42_feedback ──┐
                    ├─► WheelOdometryNode ──► /odom (wall clock)
  /imu/data ────────┘
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import Imu
from erp42_msgs.msg import SerialFeedBack
from tf2_ros import TransformBroadcaster


# ── Constants ────────────────────────────────────────────────────────────────
WHEELBASE = 1.04          # ERP42 wheelbase [m]
MAX_DT    = 1.0           # Ignore dt larger than this [s]

COV_POSE_XY   = 0.01      # Position uncertainty [m²]
COV_POSE_YAW  = 0.05      # Heading uncertainty [rad²]
COV_TWIST_VX  = 0.01      # Forward velocity [m²/s²]
COV_TWIST_VY  = 0.001     # Lateral velocity (no-slip) [m²/s²]
COV_TWIST_YAW = 0.05      # Yaw rate [(rad/s)²]
COV_FILL      = 1e-6      # Off-diagonal fill


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        # Parameters
        self.declare_parameter('pub_tf', True)
        self._pub_tf = self.get_parameter('pub_tf').get_parameter_value().bool_value

        # State
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_time = self.get_clock().now()
        self._orientation_ready = False

        # Subscribers
        self.create_subscription(SerialFeedBack, '/erp42_feedback', self._on_feedback, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f'WheelOdometry ready (L={WHEELBASE}m, pub_tf={self._pub_tf})')

    # ── IMU callback (one-time orientation init) ─────────────────────────────

    def _on_imu(self, msg: Imu):
        if not self._orientation_ready:
            _, _, yaw = euler_from_quaternion(msg.orientation)
            self._theta = yaw
            self._orientation_ready = True
            self.get_logger().info(f'IMU heading: {math.degrees(yaw):.1f}°')
            self._publish(0.0, 0.0)

    # ── ERP42 feedback callback ──────────────────────────────────────────────

    def _on_feedback(self, msg: SerialFeedBack):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if not self._orientation_ready or dt <= 0 or dt > MAX_DT:
            return

        v = msg.speed if msg.gear != 0 else -msg.speed
        steer = msg.steer

        # Bicycle kinematic model
        self._x += v * math.cos(self._theta) * dt
        self._y += v * math.sin(self._theta) * dt
        self._theta += (v / WHEELBASE) * math.tan(steer) * dt
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        self._publish(v, steer)

    # ── Odometry publisher ───────────────────────────────────────────────────

    def _publish(self, v: float, steer: float):
        stamp = self.get_clock().now().to_msg()
        q = yaw_to_quaternion(self._theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = q
        odom.pose.covariance = _make_cov_36(COV_POSE_XY, COV_POSE_XY, COV_POSE_YAW)

        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = (v / WHEELBASE) * math.tan(steer) if steer else 0.0
        odom.twist.covariance = _make_cov_36(COV_TWIST_VX, COV_TWIST_VY, COV_TWIST_YAW)

        self._odom_pub.publish(odom)

        if self._pub_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation = q
            self._tf_broadcaster.sendTransform(t)


# ── Utility functions ────────────────────────────────────────────────────────

def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw / 2.0
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def euler_from_quaternion(q) -> tuple:
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _make_cov_36(d0: float, d1: float, d5: float) -> list:
    cov = [COV_FILL] * 36
    cov[0] = d0
    cov[7] = d1
    cov[35] = d5
    return cov


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
