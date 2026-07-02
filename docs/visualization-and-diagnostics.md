# 시각화 · 진단

[← README](../README.md)

## RViz 시각화

```bash
ros2 launch um7_driver display.launch.py
```
`rviz/um7.rviz`가 Fixed Frame=`world`, `/imu/data` orientation을 **IMU 박스**로 표시합니다. 센서를 움직이면 박스가 따라 회전합니다.

> ⚠️ TF 기반 `Axes`/`TF` 축 표시는 움직일 때 지연·이중·혼자 도는 것처럼 보일 수 있습니다(TF 조회 보간 아티팩트). 발행 데이터 자체는 안정적이므로 **메시지 직결 IMU 박스**를 권장합니다.

## 진단 / Health

`/diagnostics`(기본 켜짐)에 다음이 실립니다:
- `packet_rate_hz`, `packets_total`, `checksum_errors`
- `DREG_HEALTH` 비트 디코딩: gyro/accel/mag 초기화 실패, ACC_N/MG_N(노름 경고), UART overflow, GPS timeout, 위성 수, HDOP
- 레벨: 초기화 실패·overflow → **ERROR**, 노름 경고 → **WARN**, 그 외 **OK**

```bash
ros2 topic echo /diagnostics
```

### DREG_HEALTH 비트맵 (0x55 / 85)
| 비트 | 이름 | 의미 |
|---|---|---|
| 31:26 | SATS_USED | 위치 해에 쓴 위성 수 |
| 25:16 | HDOP | ÷10 하면 실제 HDOP |
| 15:10 | SATS_IN_VIEW | 관측 위성 수 |
| 8 | OVF | 전송 과부하 → COM_RATES rate 낮춰라 |
| 5 | MG_N | mag norm이 1.0에서 너무 멀다(캘리브레이션/자기 왜곡) |
| 4 | ACC_N | accel norm이 1G에서 너무 멀다(급가속/진동) |
| 3 | ACCEL | 가속도계 초기화 실패 |
| 2 | GYRO | 자이로 초기화 실패 |
| 1 | MAG | 자력계 초기화 실패 |
| 0 | GPS | 2초 이상 GPS 패킷 미수신 |
