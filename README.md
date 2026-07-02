# um7_driver

RedShift Labs / CH Robotics **UM7** IMU를 위한 범용·재사용 가능한 **ROS 2 (Jazzy)** 드라이버.
UM7의 바이너리 시리얼 프로토콜을 읽어 표준 `sensor_msgs` 메시지로 발행합니다. **YAML 파라미터만 바꾸면 어떤 ROS 2 로봇에서도** 동작합니다.

- 대상: ROS 2 **Jazzy** (Ubuntu 24.04), Python (`ament_python`, `rclpy`)
- 발행: `imu/data`, `imu/mag`, `imu/temperature` (+ 옵션 health / diagnostics / TF)
- 서비스: `zero_gyros`, `set_mag_reference`
- 프레임·단위 모두 **실제 하드웨어로 검증**

## 빠른 시작

```bash
colcon build --packages-select um7_driver
source install/setup.bash

# config/um7.yaml 의 port 를 /dev/serial/by-id/... 로 설정 후:
ros2 launch um7_driver um7.launch.py

# 드라이버 + RViz (orientation 시각화):
ros2 launch um7_driver display.launch.py
```

자세한 설치·포트 설정은 [docs/getting-started.md](docs/getting-started.md) 참조.

## 문서 (docs/)

필요한 부분만 골라 읽으세요.

| 문서 | 내용 |
|---|---|
| [시작하기](docs/getting-started.md) | 요구 사항, 빌드, 시리얼 포트 설정, 빠른 시작 |
| [설정·토픽·서비스](docs/configuration.md) | 파라미터, 발행 토픽, 서비스 |
| [단위·프레임](docs/units-and-frames.md) | REP-103 단위(accel g→m/s², mag unit-norm), NED→ENU 프레임 규약 |
| [시각화·진단](docs/visualization-and-diagnostics.md) | RViz, `/diagnostics`, `DREG_HEALTH` 비트맵 |
| [문제 해결·테스트](docs/troubleshooting.md) | 자주 겪는 문제, `colcon test` |
| [아키텍처·제한](docs/architecture.md) | 3-파일 구조, 알려진 제한/백로그 |
| 🤖 [AI 컨텍스트 (자기완결형)](docs/ai-context.md) | **이 파일 하나만 보내면** AI가 저장소 전체 파악 — 프로토콜·레지스터맵·변환 공식·검증 수치·기여 규약 |

> **다른 사람/AI에게 저장소를 설명할 때**: 개요만 필요하면 이 README, **전체 컨텍스트가 필요하면 [docs/ai-context.md](docs/ai-context.md) 한 파일**을 보내면 됩니다.

## 아키텍처 (요약)

| 파일 | 역할 |
|---|---|
| `um7_driver/um7_registers.py` | 주소·인코딩·스케일의 단일 진실 공급원 |
| `um7_driver/um7_parser.py` | 바이트→패킷→물리값. **ROS-free**, 단위 테스트 가능 |
| `um7_driver/um7_node.py` | rclpy 노드: 파라미터·단위·프레임·메시지·서비스·TF·진단 |

프로토콜의 근거는 데이터시트(`UM7_Datasheet_v1-8_30.07.2018.pdf`), 개발 규칙·검증 이력은 [`CLAUDE.md`](CLAUDE.md) 참조.
