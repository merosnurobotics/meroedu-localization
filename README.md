# MERO 교육 · Localization

시리즈 안에 여러 강의를 순서대로 추가하는 실습 저장소입니다. 각 강의는 독립 실행하고, 필요한 코드와 환경 설정만 담습니다. 데이터·학습 가중치·개인키·원본 프로젝트 로그는 포함하지 않습니다.

| 순서 | 강의 | 실습 폴더 | 설명 자료 |
| --- | --- | --- | --- |
| 01 | LiDAR 사용 | [lessons/01-lidar](lessons/01-lidar/README.md) | [MERO 교육 자료](https://mero-website-one.vercel.app/education/localization/lidar) |
| 02 | ROS 2 토픽으로 내보내기 | [lessons/02-ros2-topics](lessons/02-ros2-topics/README.md) | [MERO 교육 자료](https://mero-website-one.vercel.app/education/localization/ros2-topics) |
| 03 | 객체 localization | [lessons/03-object-localization](lessons/03-object-localization/README.md) | [MERO 교육 자료](https://mero-website-one.vercel.app/education/localization/object-localization) |

## 시작하기

```bash
git clone https://github.com/merosnurobotics/meroedu-localization.git
cd meroedu-localization/lessons/01-lidar
```

강의 폴더의 README를 따라갑니다. 웹사이트 강의와 이 코드 저장소는 모두 공개입니다.

## 새 강의 추가

`lessons/02-주제`, `lessons/03-주제`처럼 별도 폴더에 README·필요 코드·환경 정보를 작성하고 이 목록에 추가합니다. 기존 강의의 환경과 실행 경로를 바꾸지 않습니다. 미래 강의용 빈 폴더는 만들지 않습니다.

Python 3.10 이상과 Git을 사용합니다. 01/03 실습은 Python 표준 라이브러리만 사용합니다. 02에는 ROS 2 환경이 필요합니다. [CONTRIBUTING](CONTRIBUTING.md)
