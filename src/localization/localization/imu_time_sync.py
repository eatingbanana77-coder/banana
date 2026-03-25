"""
imu_time_sync.py — IMU 메시지의 타임스탬프를 시스템 시간으로 재설정하는 노드

IMU 드라이버는 보통 하드웨어 내부 시간(uptime) 기준으로 timestamp를 찍는데,
ROS2 시스템 시간과 다르면 EKF에서 시간 불일치 문제가 발생함.

그래서 이 노드는 IMU 메시지를 받아서 timestamp를 현재 ROS 시간으로 바꾼 뒤
다시 publish하여 EKF와 시간 기준을 맞춰줌.

  /imu/data (하드웨어 시간) → imu_time_sync → /imu/data/synced (ROS 시스템 시간)
"""

# ROS2 Python 클라이언트 라이브러리
import rclpy

# ROS2 노드 클래스
from rclpy.node import Node

# IMU 메시지 타입 (가속도, 각속도, orientation 포함)
from sensor_msgs.msg import Imu


# IMU 타임스탬프 동기화 노드 정의
class ImuTimeSyncNode(Node):

    def __init__(self):
        # 부모 Node 초기화 + 노드 이름 설정
        super().__init__('imu_time_sync')

        # /imu/data 토픽 구독
        # IMU 메시지를 받아서 _on_imu 콜백 함수에서 처리
        self.create_subscription(
            Imu,
            '/imu/data',
            self._on_imu,
            10
        )

        # /imu/data/synced 토픽 발행
        # timestamp 수정된 IMU 메시지를 publish
        self._pub = self.create_publisher(
            Imu,
            '/imu/data/synced',
            10
        )

        # 노드 실행 확인용 로그
        self.get_logger().info(
            'IMU time sync active: /imu/data → /imu/data/synced'
        )


    # IMU 메시지 수신 시 실행되는 콜백 함수
    def _on_imu(self, msg: Imu):

        # 핵심: timestamp를 현재 ROS 시간으로 덮어쓰기
        # 기존: IMU 내부 시간 (hardware uptime)
        # 변경: ROS 시스템 시간 (wall clock)
        msg.header.stamp = self.get_clock().now().to_msg()

        # 수정된 메시지를 새로운 토픽으로 publish
        self._pub.publish(msg)


# 메인 함수
def main(args=None):

    # ROS2 초기화
    rclpy.init(args=args)

    # 노드 생성
    node = ImuTimeSyncNode()

    # 노드 실행 (콜백 대기 상태)
    rclpy.spin(node)

    # 종료 시 노드 정리
    node.destroy_node()

    # ROS2 종료
    rclpy.shutdown()


# 이 파일을 직접 실행할 때만 main 함수 실행
if __name__ == '__main__':
    main()
