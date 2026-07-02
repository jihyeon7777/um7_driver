# 저장소 전체 컨텍스트 (AI/LLM용 · 자기완결형)

[← README](../README.md)

> **이 파일 하나만 전달하면** 다른 사람/AI가 저장소를 이해·수정할 수 있습니다.
> 코드·데이터시트·`CLAUDE.md`에 흩어진 핵심을 여기 압축했습니다.
> (데이터시트 `UM7_Datasheet_v1-8_30.07.2018.pdf`는 저장소에 포함되지 않습니다.)

## 개요
RedShift Labs / CH Robotics **UM7** IMU용 범용 **ROS 2 (Jazzy)** 드라이버(Python, `ament_python`).
UM7의 바이너리 시리얼 프로토콜을 읽어 `sensor_msgs/Imu`·`MagneticField`·`Temperature`로 발행. YAML 파라미터만 바꾸면 어떤 로봇에서도 동작.

- 토픽: `imu/data`, `imu/mag`, `imu/temperature` (+ 옵션 `imu/health`, `/diagnostics`, `/tf`)
- 서비스: `zero_gyros`, `set_mag_reference`
- 빌드/테스트: `colcon build --packages-select um7_driver` / `colcon test --packages-select um7_driver`

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
├── README.md / docs/      # 사용자 매뉴얼
└── CLAUDE.md              # 개발 규칙·검증 이력
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
| 85 (0x55) | DREG_HEALTH | uint32 비트필드 | 상태 비트 |
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
- 린트(`colcon test`가 강제):
  - `ament_flake8` = flake8-import-order **google 스타일**(import/from을 섞어 모듈명 알파벳순), 최대 99열.
  - `ament_pep257` = **멀티라인 docstring 요약은 둘째 줄부터**(D213). `D100–D107`(docstring 누락), `D212` 무시.
- 커밋 시 실제 하드웨어로 검증한 내용은 근거 수치와 함께 기록.
