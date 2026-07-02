# um7_driver

RedShift Labs / CH Robotics **UM7** IMU를 위한 범용·재사용 가능한 **ROS 2 (Jazzy)** 드라이버.
UM7의 바이너리 시리얼 프로토콜을 읽어 표준 `sensor_msgs` 메시지로 발행합니다. **YAML 파라미터만 바꾸면 어떤 ROS 2 로봇에서도** 동작합니다.

- 대상: ROS 2 **Jazzy** (Ubuntu 24.04), Python (`ament_python`, `rclpy`)
- 발행: `imu/data`, `imu/mag`, `imu/temperature` (+ 옵션 health / diagnostics / TF)
- 서비스: `zero_gyros`, `set_mag_reference`
- 프레임·단위 모두 **실제 하드웨어로 검증** (아래 [프레임 규약](#프레임-규약) 참조)

---

## 1. 요구 사항

- ROS 2 Jazzy
- `python3-serial` (pyserial)
- `diagnostic_updater`, `diagnostic_msgs` (진단 토픽)
- (선택, 시각화용) `rviz2`, `rviz_imu_plugin`

```bash
sudo apt install python3-serial ros-jazzy-diagnostic-updater ros-jazzy-rviz-imu-plugin
```

## 2. 빌드

```bash
cd ~/ros2_ws            # 워크스페이스 src/ 아래에 이 패키지를 두고
colcon build --packages-select um7_driver
source install/setup.bash
```

## 3. 시리얼 포트 설정

**반드시 `/dev/serial/by-id/...` 안정 경로를 쓰세요** (`/dev/ttyUSBn`은 재부팅·다른 USB 장치가 있으면 번호가 밀립니다).

```bash
ls -l /dev/serial/by-id/
# 예: usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -> ../../ttyUSB0
```

포트 접근 권한이 없으면 `dialout` 그룹에 추가 후 재로그인:
```bash
sudo usermod -aG dialout $USER
```

## 4. 빠른 시작

`config/um7.yaml`의 `port`를 위 by-id 경로로 설정한 뒤:

```bash
# 드라이버만
ros2 launch um7_driver um7.launch.py

# 포트를 CLI에서 덮어쓰기 (YAML보다 우선)
ros2 launch um7_driver um7.launch.py port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_...-if00-port0

# 드라이버 + RViz (orientation 시각화)
ros2 launch um7_driver display.launch.py
```

확인:
```bash
ros2 topic echo /imu/data --once --qos-profile sensor_data
ros2 topic hz /imu/data          # 이벤트 구동, 대략 수십 Hz
```

## 5. 파라미터 (`config/um7.yaml`)

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `port` | str | `""` | 시리얼 경로. `/dev/serial/by-id/...` 사용 |
| `baud` | int | `115200` | UM7 기본 보드레이트 |
| `frame_id` | str | `"imu_link"` | 메시지 header frame_id |
| `frame_convention` | str | `"enu"` | `"enu"`=REP-103(권장) / `"ned"`=센서 원본 프레임 그대로 |
| `publish_tf` | bool | `false` | `world`→`frame_id` TF 방송(RViz 디버깅용) |
| `publish_health` | bool | `false` | `imu/health`(raw 비트맵) 발행 |
| `publish_diagnostics` | bool | `true` | `/diagnostics` 발행(패킷 레이트, 체크섬 에러, health 비트) |

> **하드코딩 금지 원칙**: port·스케일 등 어떤 값도 코드에 박혀 있지 않습니다. 다른 로봇은 YAML만 고치면 됩니다.

## 6. 발행 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `imu/data` | `sensor_msgs/Imu` | orientation(쿼터니언), angular_velocity(**rad/s**), linear_acceleration(**m/s²**) |
| `imu/mag` | `sensor_msgs/MagneticField` | 자력계 **방향 벡터(unit-norm, 무차원)** — 아래 [단위](#7-단위-rep-103) 주의 |
| `imu/temperature` | `sensor_msgs/Temperature` | 온도(°C) |
| `imu/health` | `std_msgs/UInt32` | (옵션) `DREG_HEALTH` raw 비트맵 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 패킷 레이트·체크섬 에러·health 비트 디코딩 |
| `/tf` | — | (옵션) `world`→`frame_id` |

QoS는 IMU 관례대로 **sensor data (best effort)** 입니다. 구독/에코 시 `--qos-profile sensor_data`.

## 7. 단위 (REP-103)

| 필드 | 소스 | 데이터시트 | 발행 단위 | 변환 |
|---|---|---|---|---|
| angular_velocity | GYRO_PROC | deg/s | rad/s | deg→rad |
| linear_acceleration | ACCEL_PROC | **g** (실측 확정) | m/s² | **×9.80665** |
| orientation | EULER 112~116 | deg | 쿼터니언 | RPY→quat, 프레임 변환 |
| imu/mag | MAG_PROC | **unit-norm** | (무차원 그대로) | 없음 |
| temperature | TEMPERATURE | °C | °C | 없음 |

> ⚠️ **가속도**: 데이터시트 레지스터 설명은 `m/s/s`라 적혀 있지만 **실제 펌웨어 출력은 g**입니다(정지 시 크기 ~1.0 g 실측). 그래서 드라이버가 `9.80665`를 곱합니다.
>
> ⚠️ **자력계**: `sensor_msgs/MagneticField`는 Tesla를 기대하지만 UM7 PROC mag는 **unit-norm(무차원)**입니다. **heading 방향 벡터로만** 유효하고, 물리 단위(Tesla)로 신뢰하면 안 됩니다.

## 8. 서비스

| 서비스 | 타입 | 동작 |
|---|---|---|
| `zero_gyros` | `std_srvs/Trigger` | 자이로 바이어스 0점 보정. **센서를 정지시킨 상태에서** 호출 |
| `set_mag_reference` | `std_srvs/Trigger` | 현재 heading을 자북(0°)으로 설정 |

```bash
ros2 service call /zero_gyros std_srvs/srv/Trigger
ros2 service call /set_mag_reference std_srvs/srv/Trigger
```
응답 `success=true`면 UM7이 `COMMAND_COMPLETE`를 보낸 것입니다.

## 9. 프레임 규약

- **UM7 body frame = NED/FRD** (X-forward, Y-right, Z-down) — 하드웨어로 확인:
  - 평평 시 accel = (0, 0, −1 g)  (표준 NED 비력 규약)
  - 우현(오른쪽) 아래 = +roll, nose-down = −pitch, 위에서 본 시계방향 = +yaw
- 드라이버는 `frame_convention: "enu"`(기본)에서 이를 **ROS ENU/FLU(REP-103)** 로 변환합니다.
  - orientation: `q_enu = Q_NED→ENU · q_ned · Q_FRD→FLU`
  - body 벡터(gyro/accel): `(x, −y, −z)`
- `frame_convention: "ned"`로 두면 센서 원본 프레임 그대로 발행합니다.

> RViz에서 축이 반대로 돌면 대개 부호/프레임 문제이며, **파서가 아니라 노드에서** 고칩니다.

## 10. RViz 시각화

```bash
ros2 launch um7_driver display.launch.py
```
`rviz/um7.rviz`가 Fixed Frame=`world`, `/imu/data` orientation을 **IMU 박스**로 표시합니다. 센서를 움직이면 박스가 따라 회전합니다.

> ⚠️ TF 기반 `Axes`/`TF` 축 표시는 움직일 때 지연·이중·혼자 도는 것처럼 보일 수 있습니다(TF 조회 보간 아티팩트). 발행 데이터 자체는 안정적이므로 **메시지 직결 IMU 박스**를 권장합니다.

## 11. 진단 / Health

`/diagnostics`(기본 켜짐)에 다음이 실립니다:
- `packet_rate_hz`, `packets_total`, `checksum_errors`
- `DREG_HEALTH` 비트 디코딩: gyro/accel/mag 초기화 실패, ACC_N/MG_N(노름 경고), UART overflow, GPS timeout, 위성 수, HDOP
- 레벨: 초기화 실패·overflow → **ERROR**, 노름 경고 → **WARN**, 그 외 **OK**

```bash
ros2 topic echo /diagnostics
```

## 12. 테스트

```bash
colcon test --packages-select um7_driver --event-handlers console_direct+
```
- ROS-free 파서 단위 테스트(데이터시트 검증 Euler 벡터 + 하드웨어 캡처 벡터)
- 프레임 변환 회귀 테스트
- `ament_flake8`, `ament_pep257` 린트

## 13. 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| `device disconnected or multiple access on port` | 다른 프로세스(예: 이전 노드)가 포트 점유. `fuser /dev/ttyUSB0`로 확인 후 종료 |
| 권한 거부(Permission denied) | `dialout` 그룹 추가 후 재로그인 |
| `ros2 topic echo`에 아무것도 안 뜸 | QoS 불일치 → `--qos-profile sensor_data`. 또는 오래된 데몬 → `ros2 daemon stop` |
| RViz 축이 혼자 돌거나 이중으로 보임 | TF 렌더링 아티팩트. 메시지 직결 IMU 박스 사용(위 §10) |
| 재부팅 후 포트 번호 바뀜 | `/dev/serial/by-id/...` 경로 사용 |

## 14. 아키텍처

| 파일 | 역할 |
|---|---|
| `um7_driver/um7_registers.py` | 주소·인코딩·스케일의 단일 진실 공급원 (매직넘버 금지) |
| `um7_driver/um7_parser.py` | 바이트→패킷→물리값. **ROS-free**(rclpy·메시지 import 없음), 단위 테스트 가능 |
| `um7_driver/um7_node.py` | rclpy 노드: 파라미터, 단위 변환, 프레임, 메시지 조립, 서비스, TF, 진단 |

프로토콜의 유일한 근거는 데이터시트(`UM7_Datasheet_v1-8_30.07.2018.pdf`)입니다. 개발 규칙·검증 이력은 [`CLAUDE.md`](CLAUDE.md) 참조.

## 15. 알려진 제한 / 백로그

- orientation은 **Euler 모드(A 모드)** 기반(부팅 기본값). 네이티브 쿼터니언(B 모드, `DREG_QUAT`)은 선택 백로그.
- `imu/mag`는 unit-norm(무차원). Tesla가 필요하면 raw mag 스케일 결정 필요.
- covariance: 미지원 필드는 배열 `[0]=−1`로 표기. UM7이 실제 공분산을 제공하지 않아 값은 0.
