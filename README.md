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
