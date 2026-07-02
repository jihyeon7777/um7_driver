# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# UM7 IMU 드라이버 (ROS2)

## 현재 상태 (Current State) — ⚠️ 코드 미작성
- 이 저장소에는 **현재 `CLAUDE.md` 파일 하나뿐이다.** 아래에서 참조하는 소스(`um7_driver/*.py`),
  패키지 메타(`package.xml`, `setup.py`, `setup.cfg`), `config/um7.yaml`, `launch/um7.launch.py`,
  테스트는 **아직 존재하지 않는다.** 즉 이 문서는 구현 스펙이며, `ament_python` 패키지 스캐폴딩부터 만들어야 한다.
- 따라서 아래 **빌드/테스트/실행 명령은 패키지를 만들기 전까지 동작하지 않는다.**
- 데이터시트는 저장소 밖에 있다: **`/home/jh/Downloads/UM7_Datasheet_v1-8_30.07.2018.pdf`**
  (아래 "UM7 프로토콜" 절에서 "이 파일을 직접 보면서 구현" 하라는 그 파일). 저장소에 없으니 이 경로에서 읽을 것.
- git 저장소가 아니다(`git init` 안 됨).

## 목적
RedShift Labs / CH Robotics **UM7** IMU를 위한 **범용·재사용 가능한** ROS2 드라이버.
UM7의 바이너리 시리얼 프로토콜을 읽어 표준 메시지로 발행한다 → 그래서 **파라미터만 바꾸면 어떤 ROS2 로봇에서도** 동작해야 한다.

- 대상: ROS2 **Jazzy** (Ubuntu 24.04), **Python** (`ament_python`, rclpy).

## 아키텍처
| 파일 | 역할 | 규칙 |
|------|------|------|
| `um7_driver/um7_registers.py` | 주소 + 인코딩 + 스케일 | 코드 상의 단일 진실 공급원; 다른 곳엔 매직넘버 금지 |
| `um7_driver/um7_parser.py` | 바이트 → 패킷 → 물리값 | **ROS-free** (rclpy·메시지 import 금지); 단위 테스트 가능 |
| `um7_driver/um7_node.py` | rclpy 노드 | parser 사용; 파라미터, 단위 변환, 메시지 조립, 옵션 TF |

프로토콜/디코딩 로직은 parser에, ROS 관련(REP-103 단위, 메시지 타입, 파라미터, TF)은 node에 둔다.

---

## 개발 워크플로 — 하드웨어 인더루프 (사람 협업)

**개발 내내 UM7이 실제 시리얼 포트에 연결되어 있는 상태로 작업한다.** 단계마다 실제 하드웨어로 검증하는 방식(캡처 → 데이터시트 확인 → 회귀 테스트 고정).

Claude Code는 하드웨어를 물리적으로 만질 수 없다. 그래서 **센서 값을 바꿔서 확인해야 할 때는, 필요한 자세/움직임을 구체적으로 명시해서 사용자에게 조정을 요청**한다. (사람이 조정 → 캡처한 실제 바이트/값을 받아 검증한다.)

조정을 요청할 때는 **왜 그 자세가 필요한지(무엇을 검증·캡처하려는지)**를 함께 말해서, 사용자가 맞는 조작을 하도록 한다. 값이 예상과 다르면 데이터시트를 다시 확인하고, 필요하면 다른 자세로 재요청한다.

요청 예시:
- orientation 반응 확인: "roll을 +90° 근처로 기울여 주세요", "yaw를 천천히 좌우로 돌려 주세요"
- 가속도 단위(m/s² vs g) 확인: "센서를 평평히 두세요"(한 축 ≈ 중력), "뒤집어서 z축이 아래를 향하게 해 주세요"
- gyro/accel/mag 테스트 벡터 캡처: "정지 상태로 두세요", "특정 축을 중심으로 일정 속도로 회전시켜 주세요"
- 프레임/부호 확인: "센서 X축(케이스 표시)을 정북/전방으로 향하게 두세요"

---

## orientation 소스 — **A 모드(Euler 모드) 확정**

UM7은 내부적으로 **항상 EKF로 자세를 추정**하지만, 출력 표현 방식을 고를 수 있다.
`CREG_MISC_SETTINGS`(0x08)의 **Q 비트(bit 1)**가 스위치다:
- Q = 0 → **Euler Angle 모드 (기본값)**
- Q = 1 → **Quaternion 모드**

그리고 **쿼터니언 레지스터(`DREG_QUAT_*`, 109~110)는 quaternion 모드에서만 유효**하다(데이터시트 명시).

### 이 드라이버는 A 모드로 간다
- 센서를 **기본 설정(Euler 모드) 그대로** 켠다 → 부팅 시 설정 write 패킷 불필요.
- orientation은 **Euler 레지스터(112~116)를 읽어 roll/pitch/yaw → 쿼터니언으로 변환**해서 채운다.
- 주의: 짐벌락 근처(pitch ±90°)에서 Euler→쿼터니언 변환 품질만 유의.

### B 모드(네이티브 쿼터니언)는 보류 — 선택 백로그
- `DREG_QUAT`을 직접 쓰면 EKF 내부 표현이라 이론상 orientation 정확도가 더 낫다(Euler를 거쳐 되돌아올 때 생기는 변환 손실·부호 불연속이 없음).
- 단, 구현하려면 **런타임에 write 패킷으로 Q 비트를 1로 세팅**해야 쿼터니언 레지스터가 유효해진다 — 파서에 109번 파싱을 추가하는 것만으로는 부족하다.
- 하드웨어 브링업이 끝나고 정확도가 필요해지면 그때 옵션으로 붙인다.

---

## 파라미터 (config/um7.yaml — 하드코딩 금지)
- `port` (str): 시리얼 경로. **`/dev/serial/by-id/...` 사용**, `/dev/ttyUSBn` 금지
  (재부팅하거나 다른 USB-시리얼 장치가 있으면 번호가 밀린다).
- `baud` (int, 115200): UM7 기본 115200.
- `frame_id` (str, "imu_link")
- `frame_convention` (str, "enu"): UM7 출력 프레임 → ROS ENU(REP-103) 변환.
  ⚠️ **UM7 출력 프레임은 데이터시트에 명시가 없다 → 하드웨어로 먼저 확인**(아래 "프레임 규약" 참조).
- `publish_tf` (bool, false): RViz 디버깅용 `world`→`frame_id` TF 방송.
- **발행 방식**: IMU는 **패킷 수신 시점마다 발행(이벤트 구동)**이 타이밍 보존에 유리.
  고정 주기로 최신 상태를 재발행하는 `publish_rate` 옵션도 둘 수 있으나, **기본은 이벤트 구동 권장**.
- **타임스탬프 정책**: 헤더 stamp를 ROS `now()`로 찍을지, 레지스터의 `*_TIME` 값을 매핑할지 결정.
  기본은 **수신 시점 ROS now**, `*_TIME`(센서 내부 시계)은 진단/동기화용으로 남긴다.

## 발행 토픽 (상대 이름, remap 가능)
- `imu/data` (`sensor_msgs/Imu`): orientation(풀 쿼터니언) + angular_velocity(rad/s) + linear_acceleration(m/s²)
- `imu/mag` (`sensor_msgs/MagneticField`)
- `imu/temperature` (`sensor_msgs/Temperature`)
- (선택) `imu/health` 또는 diagnostics: `DREG_HEALTH` 비트맵 기반

## 빌드 / 테스트 / 실행
```
colcon build --packages-select um7_driver
colcon test  --packages-select um7_driver --event-handlers console_direct+
ros2 launch um7_driver um7.launch.py
```

---

## UM7 프로토콜 — 데이터시트가 유일한 근거
### 프로토콜의 유일한 근거는 datasheet다:
- `UM7_Datasheet_v1-8_30.07.2018.pdf`

이 파일을 직접 보면서 구현할 것.

읽어야 할 섹션:
- Binary Packet Structure (`snp` framing, PT 바이트, address, 2바이트 checksum)
- Register Overview / Data Registers (DREG_* 주소, 레지스터별 인코딩·스케일)

### 바이트 순서 — **big-endian (MSB first)**
- 데이터시트 파서 예제대로, 레지스터는 **MSB부터** 전송·해석한다.
- 32비트 레지스터 안에 16비트 값 두 개가 들어갈 때는 **상위 절반(B3:B2)이 먼저, 하위 절반(B1:B0)이 나중**.
  - 예: `DREG_EULER_PHI_THETA`는 Phi(roll)=bits 31:16, Theta(pitch)=bits 15:0.
- ⚠️ 다른 프로젝트(MDROBOT Modbus)의 **CDAB 워드 스왑 규칙을 여기 적용하지 말 것.**
  UM7은 Modbus가 아니라 CH Robotics 바이너리 프로토콜이고, **워드 스왑이 없다**.

### 레지스터 인코딩은 한 종류가 아니다 (파서에서 구분)
세 가지가 섞여 있다. 데이터시트로 레지스터마다 최종 확인:
- **IEEE float32** (읽으면 바로 물리값): PROC gyro/accel/mag, 모든 `*_TIME`, GPS 위경도/고도/속도/코스 등.
- **16비트 signed int + 스케일 제수**: EULER 각도/각속도, QUAT 성분.
- **16비트 signed int (원시 카운트, 물리 스케일 미정)**: RAW gyro/accel/mag.
- **비트 팩드**: `DREG_HEALTH`(상태 비트), `CREG_COM_SETTINGS` 등.

---

## 단위 변환 — REP-103 (rad, rad/s, m/s²)

가공(PROC) 레지스터 기준으로 정확히 정리. **가속도·자력계는 흔히 착각하는 지점이니 주의.**

| ROS 필드 | 소스 레지스터 | 인코딩 | 데이터시트 단위 | REP-103 목표 | 변환 |
|---|---|---|---|---|---|
| `orientation` | EULER 112~116 | int16 ÷91.02222 | deg | rad→quat | deg→rad 후 RPY→쿼터니언 |
| `angular_velocity` | **GYRO_PROC 97~99** | float32 | deg/s | rad/s | **deg→rad** |
| `linear_acceleration` | ACCEL_PROC 101~103 | float32 | **g** (실측 확정; 데이터시트 "m/s/s" 표기와 불일치) | m/s² | **×9.80665** |
| `imu/mag` | MAG_PROC 105~107 | float32 | **unit-norm(무차원)** | Tesla | **직접 매핑 불가**(아래) |
| `imu/temperature` | TEMPERATURE 95 | float32 | °C | °C | 없음 |

### 핵심 두 가지
1. **가속도는 g다 → ×9.80665 (하드웨어로 확정됨).** 데이터시트 레지스터 설명은 `DREG_ACCEL_PROC`를 `m/s/s`로 적어 놨지만, **실제 펌웨어 출력은 g**다.
   - **실측 근거(2026-07-02)**: 정지 상태에서 accel 벡터 크기 = **1.02 g** (225 샘플 평균, ~9.81 아님). → PROC accel은 g 단위. 노드에서 `GRAVITY=9.80665`를 곱해 REP-103 m/s²로 변환한다(`um7_node.py`).
   - 마찬가지로 mag PROC 벡터 크기도 **~1.005**로 나와 **unit-norm** 확인됨(아래 2번).
2. **자력계는 unit-norm이라 Tesla가 아니다.** `sensor_msgs/MagneticField`는 Tesla를 기대하므로 가공 mag를 그대로 넣으면 단위가 안 맞는다.
   heading용 **정규화 방향 벡터**로는 유효. 물리 단위가 필요하면 raw mag(92~94)를 써야 하나, **raw→µT 스케일은 데이터시트에 명시가 없다** → TODO에서 결정.

### angular_velocity 소스 주의
- body 각속도(= **가공 gyro 97~99**)를 쓴다.
- Euler rate(phi_dot 등, 114~115)는 body 각속도와 **다른 양**이므로 `angular_velocity`에 쓰면 안 된다.

---

## 프레임 규약 — 하드웨어로 NED(FRD) body 확인됨 (2026-07-02)
- **실측 확정**: 평평·수평(케이스 윗면 위)일 때 accel = (x≈0, y≈0, **z≈−1.00 g**), roll/pitch≈0.
  정지 시 비력(specific force)이 (0,0,−g)로 나오는 건 **NED body(FRD: X-forward, Y-right, Z-down)** 규약과 정확히 일치.
  → UM7 body frame = **NED/FRD 확정**.
- 그래서 node의 body→ROS(FLU) 변환 `(x, −y, −z)`가 accel에 대해 옳다: (0,0,−1g)→(0,0,+1g)→×9.80665 = **z≈+9.81**(ROS 기대값). 검증됨.
- **orientation yaw 변환 검증됨(2026-07-02)**: 위에서 봤을 때 시계방향(CW)으로 물리 회전(ΔNED yaw ≈ +104°) →
  node 변환 후 **ROS ENU yaw는 −104°(감소)** 로 정확히 뒤집힘(ENU는 CCW+). flat에서 `ENU_yaw = 90° − NED_yaw`(=78.5°)도 확인.
  → `Q_NED2ENU · q_ned · Q_FRD2FLU` 변환(node)이 옳다.
- ⚠️ **남은 확인**: roll/pitch 부호(실제 tilt 자세)와 **gyro 부호**(연속 회전 시 angular_velocity 방향). 큰 위험은 아님(위 결과와 모델이 일관).
- (참고) CH Robotics 관례상 NED로 추정했었고, 이제 하드웨어로 확인됨. 데이터시트에는 여전히 좌표계 다이어그램 명시가 없다.
- position/velocity 레지스터는 North/East/**Up**(Down 아님)이니 축별 부호를 조심.
- NED(추정) ↔ ENU(ROS): `frame_convention`을 따를 것. RViz에서 축이 반대로 돌면
  거의 항상 이 부호/프레임 문제다 — **parser가 아니라 node에서** 고칠 것.

---

## 파서 검증
- parser는 ROS 없이 단위 테스트 가능해야 한다 (ROS-free 유지).

### 검증된 예제 벡터 (Euler 배치) — 회귀 테스트로 고정 OK
```
73 6E 70 D4 70 FF F5 00 78 EC 4C 00 00 00 07 FF F2 FF F9 00 00 43 43 BC F4 0C 5F
```
- `73 6E 70` = 's''n''p'
- `D4` = PT → Has Data=1, Is Batch=1, **Batch Length=5**, Hidden=0, CF=0
- `70` = 주소 112 (`DREG_EULER_PHI_THETA`)
- 즉 **112~116 Euler 배치(길이 5)** = 각도 + Euler rate + 시간:
  - roll ≈ −0.12°, pitch ≈ +1.32°, yaw ≈ −55.42°
  - roll_rate ≈ +0.44°/s, pitch_rate ≈ −0.88°/s, yaw_rate ≈ −0.44°/s
  - time ≈ 195.73 s (IEEE float)
- **체크섬**: 헤더 포함 앞 25바이트 합 = 3167 = `0x0C5F` → **일치 확인됨**.
- → 이 벡터는 **orientation / Euler-rate 회귀 테스트**로 그대로 고정해도 된다.

### 나머지 메시지는 하드웨어에서 캡처
- 위 예제는 **Euler 배치라 gyro/accel/mag 데이터는 포함하지 않는다.**
- 가공 gyro/accel/mag, temperature, health 테스트 벡터는 **실제 하드웨어에서 직접 캡처**해서
  데이터시트로 검증한 뒤에만 회귀 테스트로 고정할 것.
- 어떤 디코딩도 데이터시트로 확인하기 전엔 옳다고 가정하지 말 것.

---

## DREG_HEALTH 비트맵 (0x55 / 85) — 진단/health 토픽용
| 비트 | 이름 | 의미 |
|---|---|---|
| 31:26 | SATS_USED | 위치 해에 쓴 위성 수 |
| 25:16 | HDOP | ÷10 하면 실제 HDOP |
| 15:10 | SATS_IN_VIEW | 관측 위성 수 |
| 8 | OVF | 전송 과부하 → COM_RATES 레지스터 rate 낮춰라 |
| 5 | MG_N | mag norm이 1.0에서 너무 멀다(캘리브레이션/자기 왜곡) |
| 4 | ACC_N | accel norm이 1G에서 너무 멀다(급가속/진동) |
| 3 | ACCEL | 가속도계 초기화 실패 |
| 2 | GYRO | 자이로 초기화 실패 |
| 1 | MAG | 자력계 초기화 실패 |
| 0 | GPS | 2초 이상 GPS 패킷 미수신 |

---

## TODO 백로그 (열린 작업)
- [ ] `um7_registers.py`의 **모든** 주소·인코딩·스케일을 데이터시트로 확인.
      특히 **인코딩 종류(float32 vs int16+제수 vs raw)**를 레지스터마다 정확히 구분.
- [ ] 가공 gyro(deg→rad) / accel(변환 X — 단 하드웨어로 m/s² vs g 실측 확정) / temperature / health
      디코딩·검증, 테스트 포함.
- [ ] **자력계 단위 결정**: unit-norm(방향 벡터)로 갈지, raw→물리단위로 갈지. MagneticField 채우기 전 확정.
- [ ] `frame_convention`으로 제어하는 프레임→ENU 변환 (**프레임 자체를 하드웨어로 먼저 확인**).
- [ ] Imu/Mag의 covariance 채우기 (REP-145; 미지원 필드는 배열 [0] 원소를 −1로).
- [ ] **이벤트 구동 발행 + 헤더 타임스탬프 정책** 배선.
- [ ] 시리얼 자동 재연결(뽑혔다 꽂히면 backoff 재시도로 복구); 깔끔한 종료.
- [ ] launch의 `port:=` 오버라이드가 YAML보다 우선하게 배선 (OpaqueFunction).
- [ ] `zero_gyros`(0xAD) / `set_mag_reference`(0xB0)를 ROS 서비스로 (UM7 명령 패킷).
- [ ] (선택, 보류) **B 모드**: Q 비트 세팅 write 패킷 + `DREG_QUAT`(÷29789.09091) 직접 사용 경로.
- [ ] (선택) 진단(checksum 에러율, 패킷 레이트)을 diagnostic_updater로.

## 코딩 표준
- 타입 힌트 + docstring. 함수는 작게. parser는 ROS-free 유지.
- 시리얼 I/O는 예외 안전: 끊기면 경고 + 재연결 시도; read 사이에 부분 바이트는 버퍼 보관
  (스트리밍 parser가 부분 패킷을 이미 처리함).
- 인코딩이 섞여 있으니(float32 / int16+제수 / raw) 파서에서 레지스터별로 명확히 분기.

## Definition of Done (완료 정의)
- `ros2 launch` 시 `imu/data`가 발행되고(이벤트 구동), 센서를 돌리면 orientation이 반응
  (RViz에서 `publish_tf:=true`, Fixed Frame = `world`, TF/Axes 추가해서 확인).
- 평평히 뒀을 때 `linear_acceleration`이 한 축 ~9.81 m/s²로 나오는지(중력) 실측 확인.
- `colcon test` 통과 (위 Euler 예제 + 하드웨어로 직접 확인한 테스트 벡터로).
- node에 하드코딩된 port/scale 0개. 다른 로봇은 YAML만 고쳐서 동작.
