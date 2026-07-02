# 시작하기

[← README](../README.md)

## 요구 사항

- ROS 2 Jazzy (Ubuntu 24.04)
- `python3-serial` (pyserial)
- `diagnostic_updater`, `diagnostic_msgs` (진단 토픽)
- (선택, 시각화용) `rviz2`, `rviz_imu_plugin`

```bash
sudo apt install python3-serial ros-jazzy-diagnostic-updater ros-jazzy-rviz-imu-plugin
```

## 빌드

```bash
cd ~/ros2_ws            # 워크스페이스 src/ 아래에 이 패키지를 두고
colcon build --packages-select um7_driver
source install/setup.bash
```

## 시리얼 포트 설정

**반드시 `/dev/serial/by-id/...` 안정 경로를 쓰세요** (`/dev/ttyUSBn`은 재부팅·다른 USB 장치가 있으면 번호가 밀립니다).

```bash
ls -l /dev/serial/by-id/
# 예: usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -> ../../ttyUSB0
```

포트 접근 권한이 없으면 `dialout` 그룹에 추가 후 재로그인:
```bash
sudo usermod -aG dialout $USER
```

## 빠른 시작

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

다음: [설정·토픽·서비스](configuration.md)
