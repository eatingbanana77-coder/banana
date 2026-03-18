# Localization

ERP42 실외 주행 로봇의 로컬 오도메트리 패키지.  
Wheel encoder + IMU를 EKF로 융합하여 `/odometry/filtered`를 발행합니다.

## 차량 (ERP42 old)

| 항목 | 값 |
|------|-----|
| 구동 | 후륜 구동, 전륜 조향 |
| 크기 (L×W×H) | 1570 × 1180 × 570 mm |
| Wheelbase | 1040 mm |
| Tread | 985 mm |
| 휠 | 13 inch (175/60R13) |
| 최고 속도 | 40 kph (제한 20 kph) |

## 센서 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/erp42_feedback` | `erp42_msgs/SerialFeedBack` | 속도, 조향각, 기어 (타임스탬프 없음) |
| `/imu/data` | `sensor_msgs/Imu` | 9-DOF AHRS IMU (방향, 각속도, 가속도) |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Velodyne 32C 3D LiDAR |
| `/ublox_gps_node/fix` | `sensor_msgs/NavSatFix` | RTK GPS |

## 노드 구성

```
/erp42_feedback ──┐
                  ├─► wheel_odometry ──► /odom ──────────────┐
/imu/data ────────┘                                          ├─► ekf_filter_node ──► /odometry/filtered
                  (sync_imu:=true 시)                        │
/imu/data ──► imu_time_sync ──► /imu/data/synced ───────────┘
```

| 노드 | 역할 |
|------|------|
| `wheel_odometry` | 바퀴 오도메트리 (bicycle kinematic model) + IMU 초기 방향 설정 |
| `ekf_filter_node` | EKF 센서 융합 (`robot_localization`) |
| `imu_time_sync` | IMU 타임스탬프를 wall clock으로 재발행 (선택) |
| `static_transform_publisher` | `base_link → imu_link` TF |

## 실행

```bash
# 빌드
colcon build --packages-select localization --symlink-install

# 실제 로봇
ros2 launch localization local_localization.launch.py

# bag 재생 (IMU 타임스탬프 동기화 필요 시)
ros2 launch localization local_localization.launch.py sync_imu:=true

# bag 재생 (sim time)
ros2 launch localization local_localization.launch.py use_sim_time:=true sync_imu:=true
```

## EKF 설정 (`ekf_local.yaml`)

| 센서 | 사용 상태 | 비고 |
|------|-----------|------|
| `odom0` (`/odom`) | vx, vy=0 | no-slip 제약 |
| `imu0` (`/imu/data`) | yaw, vyaw, ax, ay | `sync_imu:=true` 시 `/imu/data/synced`로 자동 전환 |

## 패키지 구조

```
src/localization/
├── config/ekf_local.yaml
├── launch/local_localization.launch.py
├── localization/
│   ├── __init__.py
│   ├── wheel_odometry.py
│   └── imu_time_sync.py
├── package.xml
├── setup.py
└── setup.cfg
```
