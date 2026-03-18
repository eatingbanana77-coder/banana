# ROS2 Localization Package (ERP42 기반)

## 1. Package Description

본 패키지는 ROS2 환경에서 자율주행 차량(ERP42)의 위치 추정을 위한 Localization 패키지입니다.

Wheel Encoder 기반 Odometry, IMU 센서, GPS 데이터를 활용하여 EKF(Extended Kalman Filter)를 통해 차량의 위치와 자세를 추정합니다.

주요 기능:
- Wheel Odometry 계산
- IMU 데이터 처리
- GPS 데이터를 UTM 좌표계로 변환
- EKF 기반 센서 융합 (Local / Global)
- rosbag 기반 오프라인 테스트

---

## 2. How to Run

### 2-1. Workspace 빌드
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash

![result](./images/result.gif)

1. Timestamp mismatch
EKF는 센서 간 시간 기반 보간을 수행하기 때문에,
메시지의 header.stamp가 실제 GPS 관측 시간(msg.header.stamp)이어야 합니다.
현재 코드처럼 self.get_clock().now()로 찍으면 EKF 입력 시간 불일치 → TF 계산 중단.

2. Covariance 누락
EKF는 각 센서의 오차 신뢰도를 공분산 행렬로 판단합니다.
0으로 들어가면 수치적으로 특이해져서 필터 update를 생략하는 경우가 있습니다.
(robot_localization은 “zero covariance = ignore measurement” 옵션 동작)

3. Topic 이름 불일치 때문에 EKF가 입력 데이터를 구독하지 못함
