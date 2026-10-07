# 02 · ROS 2 토픽으로 내보내기

작성자: 조연우 · yencho929@snu.ac.kr

01의 계산 함수를 불러와 ROS node로 감쌉니다. `/meroedu/scan`을 구독해 최신 scan에서 위치를 계산하고, `/meroedu/pose`에 PoseStamped, `/meroedu/match_score`에 Float64를 발행합니다. 아래 순서대로 환경을 준비하고 세 터미널에서 실행합니다.

## 환경

실행 확인: Ubuntu 22.04, ROS 2 Humble, 시스템 Python 3.10. ROS 2는 [공식 설치 문서](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)대로 설치합니다. OS와 ROS 배포판의 조합을 맞추세요. Python 가상환경에 `pip install rclpy` 하는 방식은 사용하지 않습니다. 이미 ROS가 설치된 Jetson에 SSH로 접속해서 실습해도 됩니다.

필요 ROS 패키지: `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `ros2topic`. 보통 ROS base 설치에 포함됩니다. Humble에서 빠져 있다면:

```bash
sudo apt install ros-humble-rclpy ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-ros2topic
```

## 세 터미널에서 실행

각 터미널에서 아래를 먼저 실행합니다. `ROS_DOMAIN_ID=42`는 교육 예시이며 세 터미널이 같은 값을 사용합니다. 첫 실습은 같은 컴퓨터에서 진행합니다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
cd ~/meroedu-localization/lessons/02-ros2-topics
```

저장소를 다른 위치에 clone했다면 `cd` 경로를 바꾸세요. 설치한 배포판이 Humble이 아니라면 setup 경로도 해당 배포판으로 바꿉니다. 새 clone은 저장소 루트에서 시작합니다:

```bash
cd ~
git clone https://github.com/merosnurobotics/meroedu-localization.git
```

이미 clone했다면 그 저장소에서 `git pull --ff-only`로 새 강의를 받습니다.

터미널 A — 합성 scan, 1 Hz:

```bash
/usr/bin/python3 demo_scan.py
```

터미널 B — 추정 위치:

```bash
/usr/bin/python3 localization_node.py
```

터미널 C — 결과 확인:

```bash
ros2 topic list
ros2 topic info /meroedu/pose --verbose
ros2 topic echo /meroedu/pose --once
ros2 topic echo /meroedu/match_score --once
ros2 topic hz /meroedu/pose
```

`Ctrl+C`로 각 프로세스를 종료합니다. 고정된 합성 로봇 위치는 (0.63, -0.47), yaw=0.35 rad입니다. 위치가 근처에 오고, `frame_id: map`, 시간 stamp와 quaternion이 채워져야 합니다. score는 거리 잔차[m]이며 낮을수록 이 모델에 잘 맞습니다. 정확도 백분율이나 ground truth 오차가 아닙니다.

| 토픽 | 타입 | 역할 |
| --- | --- | --- |
| `/meroedu/scan` | `sensor_msgs/msg/LaserScan` | 원본 센서 좌표의 각도·거리 |
| `/meroedu/pose` | `geometry_msgs/msg/PoseStamped` | 지도 기준 x/y와 초기 가정 yaw |
| `/meroedu/match_score` | `std_msgs/msg/Float64` | matching 잔차 관찰용 |

## 메시지와 처리 정책

- 위치 m, yaw rad. orientation은 `z=sin(yaw/2)`, `w=cos(yaw/2)`, x=y=0.
- `header.stamp`는 입력 scan의 관측 시각입니다. 완료 시각으로 바꾸지 않습니다.
- `frame_id=map`은 좌표 이름입니다. 방 중심이 원점인 4×4 m 지도에 대해 실제 계산합니다. 이름만 바꾸어 다른 지도에 연결하지 마세요.
- 입력 QoS는 BEST_EFFORT/VOLATILE/KEEP_LAST, depth=1. 최신 scan 하나만 저장하고 0.5초마다 소비합니다. 출력 QoS는 depth=10, 기본 RELIABLE/VOLATILE입니다.
- stamp가 없거나 너무 오래됐거나 미래이면 발행하지 않습니다. 기본 허용 age=1초, 작은 미래 오차 허용=0.1초.
- 유효 측정이 8개 미만이거나 잔차가 기본 0.15 m보다 크면 발행하지 않습니다. 문턱값은 실습 시작값이며 실제 정확도를 보장하지 않습니다.
- 오래된 pose를 타이머로 반복 발행하지 않습니다. 입력이 멈추면 출력도 멈춥니다. 소비자는 마지막 pose의 age도 검사해야 합니다.
- score 토픽에는 stamp가 없으므로 모니터링 용도입니다. 엄밀한 시각 결합이 필요하면 pose와 score를 한 stamped 메시지에 담는 다음 단계가 필요합니다.

## 실제 LiDAR 입력

합성 노드 A를 종료하고 센서 드라이버를 실행합니다. 토픽 이름과 QoS를 확인합니다:

```bash
ros2 topic list -t
ros2 topic info /laser_scan --verbose
ros2 topic echo /laser_scan --once --qos-reliability best_effort
/usr/bin/python3 localization_node.py --ros-args -p scan_topic:=/laser_scan -p known_yaw:=0.0
```

`/laser_scan`은 센서 입력 이름의 예시입니다. 장치가 `/scan`을 내보내면 그 이름을 입력합니다. `known_yaw=0.0`은 지도 +x 방향을 보는 센서를 고정한 경우만 맞습니다. 초기 yaw를 실제 장면에 맞추고, 센서 중심=로봇 중심/정면 일치/같은 방 크기 가정을 먼저 확인합니다. 센서를 회전시켜도 yaw가 자동 추정되지 않습니다. IMU 결합은 01의 심화 discussion에서만 다룹니다.

이 코드는 TF broadcast, Nav2 연결, 모터 제어를 하지 않습니다. RViz에서 Pose를 보고 싶다면 Fixed Frame을 `map`으로 설정하고 Pose display에 `/meroedu/pose`를 지정합니다. LaserScan을 같은 화면에 겹치려면 올바른 시간의 `map → laser` TF가 추가로 필요합니다. 잘못된 정적 TF로 움직이는 센서를 고정하지 마세요.

첫 실습은 로봇 쪽 한 컴퓨터에서 실행합니다. Tailscale에 연결됐다고 ROS DDS discovery가 자동으로 원격 컴퓨터에 전달되지는 않습니다. `ssh jetson`으로 원격 터미널 여러 개를 열어 같은 Jetson에서 확인하면 됩니다.

## 검증

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=169 /usr/bin/python3 test_ros2.py
```

실제 ROS publisher/subscriber로 합성 scan → 위치/점수 수신, frame/방향/위치, 오래된 scan의 출력 차단을 확인했습니다. 실제 LiDAR 연결은 별도 검증이 필요합니다.

## 출처

- [코드 출처·라이선스 기록](../../UPSTREAM.md)
- [ROS publisher/subscriber tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
