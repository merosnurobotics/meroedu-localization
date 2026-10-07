# 원본 출처

Team 14 · MIT · ddonggae 고정 commit `d85758c752e6cd3244e16d9ea4a3d2831da225b4`

- https://github.com/YenCho/ddonggae/blob/d85758c752e6cd3244e16d9ea4a3d2831da225b4/navigation/docs/localization.md
- https://github.com/YenCho/ddonggae/blob/d85758c752e6cd3244e16d9ea4a3d2831da225b4/navigation/ros2/arena_lightweight_control/arena_lightweight_control/map_localization.py

교육용으로 핵심 함수와 실행 예제를 재구성했습니다. 원본 데이터와 하드웨어별 설정은 복사하지 않았습니다.

## 02 / 03

- ROS scan 구독·status 발행: https://github.com/YenCho/ddonggae/blob/d85758c752e6cd3244e16d9ea4a3d2831da225b4/navigation/ros2/arena_lightweight_control/arena_lightweight_control/arena_control_node.py
- Mask 하단 픽셀·depth 중앙값·카메라/로봇/지도 변환: https://github.com/YenCho/ddonggae/blob/d85758c752e6cd3244e16d9ea4a3d2831da225b4/mission/match_runner.py
- Depth pipeline 계약: https://github.com/YenCho/ddonggae/blob/d85758c752e6cd3244e16d9ea4a3d2831da225b4/perception/docs/pipeline.md

02는 원본 String JSON 대신 PoseStamped/Float64 교육 토픽을 사용합니다. 03은 원본 오른쪽/앞쪽 중간 좌표를 ROS 앞쪽/왼쪽/위쪽 축으로 재정리하고, depth 패치에서 mask 밖 배경을 제외합니다. Stitch·grid voting·모델·원본 calibration 값은 포함하지 않습니다.
