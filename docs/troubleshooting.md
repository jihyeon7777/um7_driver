# 문제 해결 · 테스트

[← README](../README.md)

## 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| `device disconnected or multiple access on port` | 다른 프로세스(예: 이전 노드)가 포트 점유. `fuser /dev/ttyUSB0`로 확인 후 종료 |
| 권한 거부(Permission denied) | `dialout` 그룹 추가 후 재로그인 |
| `ros2 topic echo`에 아무것도 안 뜸 | QoS 불일치 → `--qos-profile sensor_data`. 또는 오래된 데몬 → `ros2 daemon stop` |
| RViz 축이 혼자 돌거나 이중으로 보임 | TF 렌더링 아티팩트. 메시지 직결 IMU 박스 사용([시각화](visualization-and-diagnostics.md)) |
| 재부팅 후 포트 번호 바뀜 | `/dev/serial/by-id/...` 경로 사용 |

## 테스트

```bash
colcon test --packages-select um7_driver --event-handlers console_direct+
```
- ROS-free 파서 단위 테스트(데이터시트 검증 Euler 벡터 + 하드웨어 캡처 벡터)
- 프레임 변환 회귀 테스트
- `ament_flake8`, `ament_pep257` 린트
