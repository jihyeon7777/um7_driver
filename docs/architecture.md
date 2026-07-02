# 아키텍처 · 제한

[← README](../README.md)

## 아키텍처

| 파일 | 역할 |
|---|---|
| `um7_driver/um7_registers.py` | 주소·인코딩·스케일의 단일 진실 공급원 (매직넘버 금지) |
| `um7_driver/um7_parser.py` | 바이트→패킷→물리값. **ROS-free**(rclpy·메시지 import 없음), 단위 테스트 가능 |
| `um7_driver/um7_node.py` | rclpy 노드: 파라미터, 단위 변환, 프레임, 메시지 조립, 서비스, TF, 진단 |

프로토콜의 유일한 근거는 데이터시트(`UM7_Datasheet_v1-8_30.07.2018.pdf`)입니다.
프로토콜/레지스터/변환의 상세 스펙은 [AI 컨텍스트](ai-context.md)에, 개발 규칙·검증 이력은 [`CLAUDE.md`](../CLAUDE.md)에 있습니다.

## 알려진 제한 / 백로그

- orientation은 **Euler 모드(A 모드)** 기반(부팅 기본값). 네이티브 쿼터니언(B 모드, `DREG_QUAT`)은 선택 백로그.
- `imu/mag`는 unit-norm(무차원). Tesla가 필요하면 raw mag 스케일 결정 필요.
- covariance: 미지원 필드는 배열 `[0]=−1`로 표기. UM7이 실제 공분산을 제공하지 않아 값은 0.
