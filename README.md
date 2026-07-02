# um7_driver

RedShift Labs / CH Robotics **UM7** IMU를 위한 범용·재사용 가능한 **ROS 2 (Jazzy)** 드라이버.
UM7의 바이너리 시리얼 프로토콜을 읽어 표준 `sensor_msgs` 메시지로 발행합니다. **YAML 파라미터만 바꾸면 어떤 ROS 2 로봇에서도** 동작합니다.

- 대상: ROS 2 **Jazzy** (Ubuntu 24.04), Python (`ament_python`, `rclpy`)
- 발행: `imu/data`, `imu/mag`, `imu/temperature` (+ 옵션 health / diagnostics / TF)
- 서비스: `zero_gyros`, `set_mag_reference`
- 프레임·단위 모두 **실제 하드웨어로 검증** (아래 [프레임 규약](#9-프레임-규약) 참조)

> 📎 **이 README 하나만 있으면 저장소 전체를 파악할 수 있습니다.** 다른 사람/AI에게 이 저장소를 설명할 때 이 파일만 전달하면 됩니다 — 프로토콜 스펙·레지스터맵·변환 공식·하드웨어 검증 수치·기여 규약을 **[부록 A](#부록-a--저장소-전체-컨텍스트-aillm용)**에 모아 두었습니다.

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

---

# 부록 A — 저장소 전체 컨텍스트 (AI/LLM용)

> 이 부록은 **이 README 하나만으로** 저장소를 이해·수정할 수 있도록, 코드·데이터시트·`CLAUDE.md`에 흩어진 핵심을 압축한 것입니다. (데이터시트 `UM7_Datasheet_v1-8_30.07.2018.pdf`는 저장소에 포함되지 않습니다.)

## A.1 파일 구조
```
um7_driver/
├── package.xml / setup.py / setup.cfg      # ament_python 패키지 메타
├── um7_driver/
│   ├── um7_registers.py   # 주소·인코딩·스케일의 단일 진실 공급원 (매직넘버 금지)
│   ├── um7_parser.py      # ROS-free 스트리밍 파서 (바이트→패킷→물리값)
│   └── um7_node.py        # rclpy 노드 (파라미터·단위·프레임·메시지·서비스·TF·진단)
├── config/um7.yaml        # 파라미터
├── launch/um7.launch.py           # 드라이버 (port:= 오버라이드)
├── launch/display.launch.py       # 드라이버 + RViz
├── rviz/um7.rviz
├── test/                  # 파서·변환·하드웨어벡터 회귀 + flake8/pep257
├── README.md (이 문서) / CLAUDE.md (개발 규칙·검증 이력)
```

## A.2 바이너리 프로토콜 (데이터시트 요약)
- 패킷: `'s' 'n' 'p'` + **PT**(1) + **address**(1) + **data**(4×N) + **checksum**(2, big-endian)
- **PT 비트**: `[7]HasData [6]IsBatch [5:2]BatchLength [1]Hidden [0]CommandFailed`
- data 길이: `HasData=0`→0, `IsBatch=1`→`4×BatchLength`, 그 외→4 (레지스터 1개=4바이트)
- **checksum** = 헤더 포함 앞바이트 전부의 unsigned 16-bit 합 (big-endian 2바이트)
- **바이트 순서: big-endian(MSB first), 워드 스왑 없음.** 32비트 레지스터에 16비트 값 2개면 상위(bits 31:16) 먼저.
- **명령**: `PT=0` + command address 전송 → 성공 시 `COMMAND_COMPLETE`(같은 주소, PT=0), 실패 시 `COMMAND_FAILED`(PT의 CF 비트=1).

## A.3 레지스터 맵 (드라이버가 디코딩하는 것)
| addr | 이름 | 인코딩 | 필드/단위 |
|---|---|---|---|
| 85 (0x55) | DREG_HEALTH | uint32 비트필드 | 상태 비트(§11) |
| 95 (0x5F) | DREG_TEMPERATURE | float32 | °C |
| 97–99 | DREG_GYRO_PROC_X/Y/Z | float32 | deg/s |
| 101–103 | DREG_ACCEL_PROC_X/Y/Z | float32 | **g** (데이터시트는 m/s² 표기지만 실측 g) |
| 105–107 | DREG_MAG_PROC_X/Y/Z | float32 | unit-norm(무차원) |
| 112 (0x70) | DREG_EULER_PHI_THETA | int16 ÷ 91.02222 | roll=bits31:16, pitch=bits15:0 (deg) |
| 113 | DREG_EULER_PSI | int16 ÷ 91.02222 | yaw=bits31:16 (deg) |
| 114 | DREG_EULER_PHI_THETA_DOT | int16 ÷ 16.0 | roll_rate, pitch_rate (deg/s) |
| 115 | DREG_EULER_PSI_DOT | int16 ÷ 16.0 | yaw_rate (deg/s) |
| 116 | DREG_EULER_TIME | float32 | s |
| 109/110 | DREG_QUAT_AB/CD | int16 ÷ 29789.09091 | (B모드 전용, **미사용**) |
| — | 0xAD ZERO_GYROS / 0xB0 SET_MAG_REFERENCE | 명령 | 서비스로 노출 |

기본 브로드캐스트(공장 설정, ~75 pkt/s): HEALTH(85) / raw+temp 배치(86–96) / PROC 배치(97–108) / EULER 배치(112–116) / gyro-bias(137–139). 드라이버는 필요한 레지스터만 디코딩하고 나머지는 무시(파서가 raw 보관).

## A.4 좌표/단위 변환 (node에서만 수행, parser는 원시값)
- 각속도: `deg → rad` (×π/180)
- 가속도: **`×9.80665`** (g → m/s²)
- orientation: `quaternion_from_euler(roll,pitch,yaw)` = **ZYX** 순, `(x,y,z,w)` 반환
- **NED→ENU (frame_convention="enu")**:
  - orientation: `q_enu = Q_NED2ENU ⊗ q_ned ⊗ Q_FRD2FLU`
    - `Q_NED2ENU = (x,y,z,w) = (√0.5, √0.5, 0, 0)` — (1,1,0)축 180° 회전
    - `Q_FRD2FLU = (1, 0, 0, 0)` — x축 180° 회전
  - body 벡터(gyro/accel): `(x, −y, −z)`
  - `frame_convention="ned"`이면 위 변환 생략(센서 원본 프레임 그대로).

## A.5 하드웨어 검증 결과 (ground truth, 2026-07-02)
- **가속도 = g 단위**: 정지 시 벡터 크기 1.02 g (≠ 9.81) → ×9.80665. 라이브 확인 완료.
- **자력계 = unit-norm**: 벡터 크기 ~1.005 (Tesla 아님).
- **body frame = NED/FRD** (X-fwd, Y-right, Z-down):
  - 평평: accel = (0, 0, −1.00 g) (표준 NED 비력식 `(sinθ, −sinφ·cosθ, −cosφ·cosθ)`와 vector error < 0.002)
  - 우현 아래 = **+roll**, nose-down = **−pitch**, 위에서 본 CW = **+yaw**
  - yaw NED→ENU: 물리 CW 회전 → ROS ENU yaw 감소, `ENU_yaw = 90° − NED_yaw`
  - gyro_z 부호가 yaw 증감과 일치 (NED Z-down)
- **안정성**: 정지 시 orientation 최대 점프 0.05°, 쿼터니언 부호 뒤집힘 0회, checksum 에러 0.

## A.6 핵심 설계 결정
- **Euler A 모드**: UM7은 부팅 시 Euler 출력이 기본 → orientation은 Euler(112–116)를 읽어 쿼터니언으로 변환. **부팅 시 설정 write 불필요.** 네이티브 쿼터니언(B 모드)은 Q 비트 write가 필요해 보류.
- **이벤트 구동 발행**: 패킷 수신 시점마다 발행, header stamp = ROS `now()`.
- **견고성**: 시리얼 자동 재연결(backoff 0.5→5 s), SIGINT/ExternalShutdown 깔끔한 종료, 부분 패킷 버퍼링 + 체크섬 실패 시 1바이트 재동기화.

## A.7 기여 규약 (수정 시 지킬 것)
- `um7_parser.py`는 **ROS-free 유지** (rclpy·메시지 import 금지) — 단위 테스트 가능해야 함.
- 프로토콜 매직넘버는 **`um7_registers.py`에만**.
- 프레임/부호 문제는 **parser가 아니라 node에서** 수정.
- 새 디코딩/테스트 벡터는 **데이터시트 또는 물리(하드웨어)로 검증한 뒤에만** 회귀 테스트로 고정.
- 린트(‎`colcon test`가 강제):
  - `ament_flake8` = flake8-import-order **google 스타일**(import/from을 섞어 모듈명 알파벳순), 최대 99열.
  - `ament_pep257` = **멀티라인 docstring 요약은 둘째 줄부터**(D213). `D100–D107`(docstring 누락), `D212` 무시.
- 커밋 시 실제 하드웨어로 검증한 내용은 근거 수치와 함께 기록.
