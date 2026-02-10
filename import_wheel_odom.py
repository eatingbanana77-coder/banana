import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class WheelOdomRelay(Node):
    def __init__(self):
        super().__init__('wheel_odom_relay')

        self.sub = self.create_subscription(
            Odometry,
            '/wheel/odom',          # wheel odom 입력 토픽
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            Odometry,
            '/odometry/visual',     # RViz용 출력 토픽
            10
        )

    def cb(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "odom"

        odom.child_frame_id = ""   # child_frame_id 사용 안 함
        odom.pose = msg.pose
        odom.twist = msg.twist

        self.pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
