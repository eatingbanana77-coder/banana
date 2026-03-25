"""
wheel_odometry.py — ERP42 Wheel Odometry Node

/erp42_feedback (속도, 조향각) + /imu/data (초기 방향)
→ 차량 위치(x, y, theta) 추정
→ /odom으로 publish (ROS 시간 기준)

  /erp42_feedback ──┐
                    ├─► WheelOdometryNode ──► /odom
  /imu/data ────────┘
"""

# 수학 연산용
import math

# ROS2 Python 라이브러리
import rclpy
from rclpy.node import Node

# Odometry 메시지 (위치 + 속도)
from nav_msgs.msg import Odometry

# TF 변환 메시지 (좌표계 연결)
from geometry_msgs.msg import TransformStamped, Quaternion

# IMU 메시지 (orientation 사용)
from sensor_msgs.msg import Imu

# ERP42 차량 피드백 (속도, 조향각)
from erp42_msgs.msg import SerialFeedBack

# TF broadcaster (odom → base_link 퍼블리시)
from tf2_ros import TransformBroadcaster


# ── 상수 정의 ─────────────────────────────────────────────

WHEELBASE = 1.04   # 차량 휠베이스 (앞뒤 바퀴 거리) [m]
MAX_DT    = 1.0    # dt가 이 값보다 크면 무시 (센서 끊김 방지)

# 공분산 (불확실도)
COV_POSE_XY   = 0.01   # 위치 오차
COV_POSE_YAW  = 0.05   # 방향 오차
COV_TWIST_VX  = 0.01   # 전진 속도 오차
COV_TWIST_VY  = 0.001  # 횡방향 속도 (거의 없음)
COV_TWIST_YAW = 0.05   # yaw rate 오차
COV_FILL      = 1e-6   # 나머지 값 채우기


# ── 메인 노드 ─────────────────────────────────────────────

class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        # TF publish 여부 파라미터
        self.declare_parameter('pub_tf', True)
        self._pub_tf = self.get_parameter('pub_tf').get_parameter_value().bool_value

        # 상태 변수 (로봇 위치)
        self._x = 0.0       # x 위치
        self._y = 0.0       # y 위치
        self._theta = 0.0   # heading (yaw)

        # 시간 관리
        self._last_time = self.get_clock().now()

        # IMU 초기화 여부
        self._orientation_ready = False

        # ── 구독자 ──

        # 차량 피드백 (속도, 조향각)
        self.create_subscription(
            SerialFeedBack,
            '/erp42_feedback',
            self._on_feedback,
            10
        )

        # IMU (초기 방향 설정용)
        self.create_subscription(
            Imu,
            '/imu/data',
            self._on_imu,
            10
        )

        # ── 퍼블리셔 ──

        # Odometry 출력
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster (odom → base_link)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'WheelOdometry ready (L={WHEELBASE}m, pub_tf={self._pub_tf})'
        )


    # ── IMU 콜백 (초기 방향 1번만 설정) ─────────────────────

    def _on_imu(self, msg: Imu):

        # 아직 방향 초기화 안 됐을 때만 실행
        if not self._orientation_ready:

            # quaternion → yaw 변환
            _, _, yaw = euler_from_quaternion(msg.orientation)

            # 초기 heading 설정
            self._theta = yaw

            # 초기화 완료 플래그
            self._orientation_ready = True

            self.get_logger().info(f'IMU heading: {math.degrees(yaw):.1f}°')

            # 초기 상태 publish
            self._publish(0.0, 0.0)


    # ── ERP42 피드백 콜백 ─────────────────────────────────

    def _on_feedback(self, msg: SerialFeedBack):

        # 현재 시간
        now = self.get_clock().now()

        # dt 계산 (초 단위)
        dt = (now - self._last_time).nanoseconds * 1e-9

        # 시간 갱신
        self._last_time = now

        # 조건 체크
        if not self._orientation_ready or dt <= 0 or dt > MAX_DT:
            return

        # 속도 (기어에 따라 부호 결정)
        v = msg.speed if msg.gear != 0 else -msg.speed

        # 조향각
        steer = msg.steer

        # ── Bicycle 모델 ──
        # x, y, theta 업데이트

        self._x += v * math.cos(self._theta) * dt
        self._y += v * math.sin(self._theta) * dt

        self._theta += (v / WHEELBASE) * math.tan(steer) * dt

        # theta 정규화 (-pi ~ pi)
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        # 결과 publish
        self._publish(v, steer)


    # ── Odometry publish ─────────────────────────────────

    def _publish(self, v: float, steer: float):

        # 현재 시간
        stamp = self.get_clock().now().to_msg()

        # yaw → quaternion 변환
        q = yaw_to_quaternion(self._theta)

        # Odometry 메시지 생성
        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'        # 기준 좌표계
        odom.child_frame_id = 'base_link'    # 로봇

        # 위치
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = q

        # 위치 공분산
        odom.pose.covariance = _make_cov_36(
            COV_POSE_XY,
            COV_POSE_XY,
            COV_POSE_YAW
        )

        # 속도
        odom.twist.twist.linear.x = float(v)

        # yaw rate (각속도)
        odom.twist.twist.angular.z = (
            (v / WHEELBASE) * math.tan(steer) if steer else 0.0
        )

        # 속도 공분산
        odom.twist.covariance = _make_cov_36(
            COV_TWIST_VX,
            COV_TWIST_VY,
            COV_TWIST_YAW
        )

        # Odometry publish
        self._odom_pub.publish(odom)

        # TF publish (옵션)
        if self._pub_tf:
            t = TransformStamped()

            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'

            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation = q

            self._tf_broadcaster.sendTransform(t)


# ── 유틸 함수 ─────────────────────────────────────────────

# yaw → quaternion 변환
def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw / 2.0
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half),
        w=math.cos(half)
    )


# quaternion → euler (roll, pitch, yaw)
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


# 공분산 행렬 생성 (6x6 → 36개)
def _make_cov_36(d0: float, d1: float, d5: float) -> list:
    cov = [COV_FILL] * 36
    cov[0] = d0     # x
    cov[7] = d1     # y
    cov[35] = d5    # yaw
    return cov


# ── 메인 ─────────────────────────────────────────────

def main(args=None):

    rclpy.init(args=args)

    node = WheelOdometryNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
