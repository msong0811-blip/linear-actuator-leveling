# 3륜 차량 WASD 주행 및 고속 능동 서스펜션 제어 (2026-11-27)

Dynamixel `XD540-T270-R` 4축(전륜 구동/조향 + 후륜 좌/우 구동) 주행과 리니어 액추에이터 능동 수평유지 제어를 통합한 Mac GUI + Jetson 워커 + Arduino 고속 펌웨어 패키지입니다.

## 디렉터리 구성

| 경로 | 설명 |
|---|---|
| `mac_gui/three_wheel_wasd.py` | Mac Tkinter 통합 제어 GUI (WASD 주행, 실시간 IMU 기울기 게이지, 현재 기울기 입력 기반 캘리브레이션, `5~60 mm/s` 서스펜션 속도 선택, 원클릭 펌웨어 업로드) |
| `jetson/jetson_worker.py` | Jetson 측 Dynamixel `GroupSyncWrite`/`GroupSyncRead`(`2000000 bps`) 주행 워커 및 서스펜션 브리지 |
| `jetson/three_wheel_calibration_20261127.json` | 4모터 ID(`1~4`), 방향, 조향 중심(`3074`), 앞/뒤 바퀴 직경비(`260 mm : 120 mm = 2.167x`) 캘리브레이션 |
| `jetson/suspension/controller.py` | `0.8°` 임계값 기반 정밀/고속 2단 제어기 (비대칭 고속 제동 적용으로 정지·복귀 진동 억제) |
| `jetson/suspension/run.py` | IMU/Arduino 시리얼 루프, 사용자 지정 기울기 기준 보정, `DONE`/`STATUS` 동기화 |
| `jetson/suspension/calibration.py` | 액추에이터 행정(`70 mm`, `-6688..4512` pulses) 경계 검증 및 위치 환산 |
| `jetson/firmware/ActuatorSweep/` | Arduino Uno 펌웨어 `v=10` (`max_hz=9600` = `60 mm/s`, `DIR_SETUP_MS=2`) |
| `jetson/upload_firmware.py` | Jetson에서 `arduino-cli`로 펌웨어 컴파일·업로드하는 헬퍼 |

## 오프라인 단위 검사

```sh
python3 jetson/suspension/controller.py
python3 jetson/suspension/calibration.py
```
