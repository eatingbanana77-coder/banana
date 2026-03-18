import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from pyproj import Transformer

class GpsToOdomNode(Node):
    def __init__(self):
        super().__init__('gps_odom_publisher_node')
        
        # 1. 구독 및 발행 설정
        self.subscription = self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.listener_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/gps_odom', 10)

        # 2. UTM 변환기 (WGS84 -> UTM 52N)
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32652")

        # 3. Map Anchor 설정
        self.anchor_lat = 37.495
        self.anchor_lon = 126.957
        self.anchor_x, self.anchor_y = self.transformer.transform(self.anchor_lat, self.anchor_lon)
        
        self.get_logger().info(f'Map Anchor 설정 완료: x={self.anchor_x}, y={self.anchor_y}')

    def listener_callback(self, msg):
        # 현재 GPS 위치를 UTM으로 변환
        current_x, current_y = self.transformer.transform(msg.latitude, msg.longitude)

        # 4. 상대 좌표 계산 (Current UTM - Anchor UTM)
        relative_x = current_x - self.anchor_x
        relative_y = current_y - self.anchor_y

        # 5. Odometry 메시지 구성
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        # 상대 좌표 입력
        odom.pose.pose.position.x = relative_x
        odom.pose.pose.position.y = relative_y
        odom.pose.pose.position.z = 0.0

        # 발행
        self.odom_publisher.publish(odom)
        self.get_logger().info(f'Relative Pos: x={relative_x:.3f}, y={relative_y:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = GpsToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()