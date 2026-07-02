# 설정 · 토픽 · 서비스

[← README](../README.md)

## 파라미터 (`config/um7.yaml`)

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

## 발행 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `imu/data` | `sensor_msgs/Imu` | orientation(쿼터니언), angular_velocity(**rad/s**), linear_acceleration(**m/s²**) |
| `imu/mag` | `sensor_msgs/MagneticField` | 자력계 **방향 벡터(unit-norm, 무차원)** — [단위](units-and-frames.md) 주의 |
| `imu/temperature` | `sensor_msgs/Temperature` | 온도(°C) |
| `imu/health` | `std_msgs/UInt32` | (옵션) `DREG_HEALTH` raw 비트맵 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 패킷 레이트·체크섬 에러·health 비트 디코딩 |
| `/tf` | — | (옵션) `world`→`frame_id` |

QoS는 IMU 관례대로 **sensor data (best effort)** 입니다. 구독/에코 시 `--qos-profile sensor_data`.

## 서비스

| 서비스 | 타입 | 동작 |
|---|---|---|
| `zero_gyros` | `std_srvs/Trigger` | 자이로 바이어스 0점 보정. **센서를 정지시킨 상태에서** 호출 |
| `set_mag_reference` | `std_srvs/Trigger` | 현재 heading을 자북(0°)으로 설정 |

```bash
ros2 service call /zero_gyros std_srvs/srv/Trigger
ros2 service call /set_mag_reference std_srvs/srv/Trigger
```
응답 `success=true`면 UM7이 `COMMAND_COMPLETE`를 보낸 것입니다.

관련: [단위·프레임](units-and-frames.md) · [시각화·진단](visualization-and-diagnostics.md)
