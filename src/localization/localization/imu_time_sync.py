"""
imu_time_sync.py — Re-stamp IMU messages with wall-clock time

The IMU driver publishes with hardware uptime timestamps which differ from
the system wall clock. This node republishes IMU messages with wall-clock
stamps so that the EKF's internal timer and sensor timestamps are in the
same time domain.

  /imu/data (HW uptime stamp) → imu_time_sync → /imu/data/synced (wall clock)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuTimeSyncNode(Node):

    def __init__(self):
        super().__init__('imu_time_sync')
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self._pub = self.create_publisher(Imu, '/imu/data/synced', 10)
        self.get_logger().info('IMU time sync active: /imu/data → /imu/data/synced')

    def _on_imu(self, msg: Imu):
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTimeSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
