# pipet_hand_mark7_description

현재 ROS 2 Jazzy workspace에서 사용하는 Pipet Hand Mark7 로봇 핸드 URDF
description 패키지입니다.

## 개요

5개의 손가락(엄지, 검지, 중지, 약지, 소지)을 가진 로봇 핸드의 URDF 모델을 제공합니다.

### 링크 구조

| 손가락 | 링크 |
|--------|------|
| Thumb (엄지) | thumb_bottom, thumb_middle, thumb_top |
| Index (검지) | index_bottom, index_top |
| Middle (중지) | middle_bottom, middle_top |
| Ring (약지) | ringer_bottom, ringer_top |
| Pinky (소지) | pinky_bottom, pinky_top |

### 조인트

- 모든 조인트는 `revolute` 타입
- 각 손가락 기저부 조인트: 0 ~ 1.57 rad (90도)
- 손가락 관절 조인트: -1.57 ~ 0 rad 또는 0 ~ 1.57 rad

## 패키지 구조

```
pipet_hand_mark7_description/
├── config/
│   └── urdf.rviz          # RViz 설정 파일
├── launch/
│   └── display.launch.py  # 시각화 런치 파일
├── meshes/                # STL 메시 파일
│   ├── base_link.stl
│   ├── thumb_*.stl
│   ├── index_*.stl
│   ├── middle_*.stl
│   ├── ringer_*.stl
│   └── pinky_*.stl
├── urdf/
│   ├── pipet_hand_mark7.xacro   # 메인 URDF 파일
│   ├── pipet_hand_mark7.gazebo  # Gazebo 설정
│   ├── pipet_hand_mark7.trans   # 변환 정보
│   └── materials.xacro          # 재질 정의
├── CMakeLists.txt
└── package.xml
```

## 의존성

### 빌드 의존성
- ament_cmake

### 실행 의존성
- urdf
- xacro
- robot_state_publisher
- joint_state_publisher
- joint_state_publisher_gui
- rviz2
- gazebo_ros
- ros2_control
- ros2_controllers
- controller_manager

## 설치

### 의존성 설치
```bash
cd <repository>/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 빌드
```bash
cd <repository>/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pipet_hand_mark7_description
source install/setup.bash
```

## 사용법

### RViz에서 모델 시각화
```bash
ros2 launch pipet_hand_mark7_description display.launch.py
```

Joint State Publisher GUI를 통해 각 조인트를 조작할 수 있습니다.

### Launch 파라미터
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| gui | true | Joint State Publisher GUI 활성화 |
