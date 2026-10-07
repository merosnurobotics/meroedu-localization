# 01 · LiDAR 사용

LiDAR의 각도·거리 입력과 알려진 사각형 벽의 예상 거리를 비교해 위치 x/y를 찾습니다. 초기 방향 yaw는 알고 있다고 가정합니다. IMU 구현은 포함하지 않습니다.

## 환경과 실행

Python 3.10 이상. 추가 package, ROS, GPU, 센서 불필요.

```bash
python3 demo.py --output output
python3 -m unittest discover -s tests
```

`output/scan.csv`: seed=14인 설명용 합성 측정 48개. `output/result.json`: 정답, 추정, 점수, 위치 오차. 정답은 (.63, -.47, .35 rad), 새로 실행한 추정은 (.6281, -.4688, .35 rad), 위치 오차 약 .00225 m입니다. **이것은 실제 센서 정확도가 아닙니다.**

## 코드 읽는 순서

1. `wall_range`: 광선이 네 벽 중 처음 만나는 거리.
2. `score`: 절대 오차 제한 .35 m, 최악 30% 제외, 최소 8개 입력.
3. `grid_search`: 방향을 고정한 x/y 탐색과 작은 창의 refinement.
4. `demo.py`: 소량 합성 측정 생성과 결과 저장.

실제 LaserScan에서는 `angle_min + i*angle_increment`, `ranges[i]`를 쌍으로 만들고 range_min/max와 finite 조건으로 입력을 걸러줍니다. 이 코드는 장애물이 scan을 가리지 않는 알려진 사각 방만 가정합니다. 일반 환경의 SLAM이나 로봇 구동 stack이 아닙니다.

## 연습과 discussion

- 정답 위치, 잡음, 오염 측정 수를 하나씩 바꿔 결과를 비교합니다.
- 초기 yaw를 10° 틀리게 넣고 위치가 어떻게 바뀌는지 봅니다.
- 정사각형의 90° 대칭을 구별하려면 어떤 추가 정보가 필요할까요?
- 심화: 방향 변화량을 gyro 또는 odometry로 추정하고 지도 관측으로 누적 오차를 보완하는 방법을 토론합니다. 장착 각도·축 보정·IMU filter는 본 강의 범위 밖입니다.

## 검증

합성 예제 실행, 거리/단위, 위치 복원, 대칭 모호성, 불충분한 입력 거부를 검증했습니다. 실제 센서와 로봇은 새로 구동하지 않았습니다. 원본은 [UPSTREAM](../../UPSTREAM.md).
