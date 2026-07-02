# 단위 · 프레임 규약

[← README](../README.md)

## 단위 (REP-103)

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

## 프레임 규약

- **UM7 body frame = NED/FRD** (X-forward, Y-right, Z-down) — 하드웨어로 확인:
  - 평평 시 accel = (0, 0, −1 g)  (표준 NED 비력 규약)
  - 우현(오른쪽) 아래 = +roll, nose-down = −pitch, 위에서 본 시계방향 = +yaw
- 드라이버는 `frame_convention: "enu"`(기본)에서 이를 **ROS ENU/FLU(REP-103)** 로 변환합니다.
  - orientation: `q_enu = Q_NED→ENU · q_ned · Q_FRD→FLU`
  - body 벡터(gyro/accel): `(x, −y, −z)`
- `frame_convention: "ned"`로 두면 센서 원본 프레임 그대로 발행합니다.

> RViz에서 축이 반대로 돌면 대개 부호/프레임 문제이며, **파서가 아니라 노드에서** 고칩니다.

변환 공식·검증 수치의 자세한 내용은 [AI 컨텍스트 §A.4–A.5](ai-context.md)에 있습니다.
