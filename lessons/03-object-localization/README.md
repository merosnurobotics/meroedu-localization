# 03 · 객체 localization

작성자: 조연우 · yencho929@snu.ac.kr

로봇 위치를 알아내는 것과 객체 위치를 알아내는 것은 다른 문제입니다. 이 실습은 객체 segmentation mask와 같은 픽셀에 aligned된 depth, 카메라 파라미터, 촬영 당시 로봇 pose를 사용해 객체의 관측점을 지도에 올립니다.

한 카메라의 원본 RGB 좌표에 놓인 객체별 mask와 aligned depth를 입력합니다. 하단 대표 픽셀의 깊이로 카메라 기준 관측점을 구하고, 장착 변환과 로봇 pose를 적용합니다. 아래 입력 형식과 코드 읽는 순서에 따라 실행합니다.

## 시작

Python 3.10 이상. 표준 라이브러리만 사용합니다. ROS·GPU·카메라·가중치 없이 계산 흐름을 확인할 수 있습니다.

```bash
git clone https://github.com/merosnurobotics/meroedu-localization.git
cd meroedu-localization/lessons/03-object-localization
python3 demo.py
cat output/result.json
python3 test_geometry.py
```

이미 clone했다면 저장소 루트에서 `git pull --ff-only` 후 강의 폴더로 이동합니다.

합성 예제는 17×13 픽셀 mask/depth를 실행 시 생성합니다. 실제 촬영 데이터가 아닙니다. 배경은 3.5 m, 객체는 1.5 m이며 한 픽셀의 depth=0은 무효값입니다. `output/input.json`은 입력 형식 예시입니다.

```text
pixel_uv:      [11, 10]
depth_m:       1.5
camera_xyz_m:  [0.05625, 0.075, 1.5]
robot_xyz_m:   [1.62948, -0.05625, 0.13933]
map_xyz_m:     [0.55625, 1.12948, 0.13933]
```

이 결과는 보이는 하단 표면의 대표 관측점입니다. 물체의 3D 중심, 정확한 바닥 접점, 실물 ground truth를 뜻하지 않습니다. 깊이 중앙값과 대표 픽셀을 결합하는 방법도 혼합 표면에서는 오차가 생깁니다.

## 데이터 계약

`demo.py --input frame.json`으로 자기 입력을 처리합니다. JSON의 필요한 필드:

| 필드 | 의미 |
| --- | --- |
| `mask` | 원본 RGB 해상도의 H×W 0/1 배열, 객체 하나의 instance mask |
| `depth` | 같은 픽셀 grid로 aligned된 H×W depth 배열 |
| `depth_scale_m` | 저장값 하나당 m, mm 저장이면 0.001; m 저장이면 1.0 |
| `intrinsics` | 그 원본/rectified 이미지의 fx, fy, cx, cy [pixel] |
| `mount` | forward_m, left_m, height_m, pitch_down_rad |
| `robot_pose` | 촬영 시각의 지도 기준 x, y [m], yaw [rad] |
| `label` | 객체 이름 (선택) |

mask와 depth의 크기가 같아도 정렬됐다는 증거가 아닙니다. 반드시 같은 이미지의 같은 픽셀이 같은 광선을 가리켜야 합니다. 보간으로 크기만 바꾸어 depth를 맞추지 않습니다. 이미지를 crop/resize하면 mask 좌표와 intrinsics도 함께 변환해야 합니다. 첫 실습에서는 원본 한 카메라 해상도를 그대로 사용하세요.

## 단계

1. Mask의 가장 아래쪽 두 행에서 평균 u와 최하단 v를 고릅니다. 하단의 대표 관측점을 선택하는 방식입니다. 숨겨진 바닥 접점이 보이는 것처럼 가정하지 않습니다.
2. 주변 작은 패치의 유효 depth 중앙값을 구합니다. 교육 코드는 mask 밖 배경도 제외합니다. 0/NaN/inf/범위 밖 값과 유효 샘플 3개 미만은 거부합니다. mask 내부 경계 픽셀에도 혼합 depth가 생길 수 있어 실제 장면에서 확인해야 합니다.
3. `X=(u-cx)*Z/fx`, `Y=(v-cy)*Z/fy`, `Z=depth_m`으로 카메라 optical 좌표를 구합니다. Z는 광축 깊이이며 직선 거리 `sqrt(X²+Y²+Z²)`와 다릅니다.
4. 카메라 장착 위치와 아래쪽 pitch를 반영해 로봇 좌표로 옮깁니다.
5. 촬영 당시 로봇 yaw로 회전하고 로봇 지도 x/y를 더합니다.

카메라 축은 x=오른쪽, y=아래, z=앞. 로봇 축은 x=앞, y=왼쪽, z=위입니다. 이 코드의 robot 좌표는 `(x_forward, y_left, z_up)`입니다. 카메라 정면이 로봇 정면과 일치하면 카메라의 오른쪽 x는 로봇의 왼쪽 y의 음수 방향입니다.

## 단순화한 가정

- Rectified pinhole 이미지와 대응 intrinsics. 왜곡이 남은 카메라는 보정 후 사용하거나 장치 SDK의 deprojection을 사용합니다.
- 카메라 yaw/roll은 로봇과 정렬, pitch만 존재. 일반 장착에는 전체 calibrated rotation/translation이 필요합니다.
- 로봇이 평평한 바닥에 있고 map z=0과 base 원점 높이가 일치.
- Mask, depth, pose가 같은 관측 시각에 대응. 첫 실제 실험은 멈춘 상태로 진행합니다. 이동 중에는 시간 보간·동기화가 필요합니다.

## 실제 카메라와 연결

이 강의는 모델 학습이나 RealSense 드라이버를 포함하지 않습니다. 객체인식 강의에서 학습한 segmentation 모델로 객체별 mask를 준비합니다. RealSense라면 SDK의 `align`으로 depth를 RGB에 맞추고, 그 aligned 이미지에 대응하는 intrinsics와 장치 depth scale을 읽습니다. raw uint16을 무조건 mm라고 가정하지 마세요.

이미지가 stitch된 경우 원본 카메라와 원본 픽셀로 되돌려야 합니다. 합친 화면에서 검출했다면 사용한 stitch 변환의 역변환으로 원본 카메라와 픽셀을 찾아야 합니다. 첫 실습에서는 이 단계를 피하도록 원본 RGB 하나를 사용합니다. 여러 카메라를 합친 가상 화면의 좌표에 한 카메라의 fx/fy를 바로 대입하면 안 됩니다.

예제 JSON의 `intrinsics`/`mount`/`robot_pose`는 모두 설명용입니다. 그대로 실물에 사용하지 않습니다. 객체마다 관측점을 구하고 label과 함께 저장하면 간단한 object map이 됩니다. 같은 객체의 반복 관측을 합치는 data association이나 경기장 grid snapping은 별도 단계입니다. 정해진 격자를 사용하는 경우에도 객체가 그 격자에 놓인다는 가정과 허용 오차를 먼저 확인합니다.

## 검증

`python3 test_geometry.py`: 광축 좌표/로봇 축, 90° 지도 회전과 평행이동, 카메라 pitch/height, 배경·무효 depth 제외, 입력 크기·intrinsics 검사, 전체 합성 흐름을 확인합니다. 실물 좌표 정확도·카메라 연결·모델 성능은 이 테스트의 검증 범위가 아닙니다.

## 참고 자료

- [코드 출처·라이선스 기록](../../UPSTREAM.md)
- [ROS 좌표 규약 REP 103](https://www.ros.org/reps/rep-0103.html)
