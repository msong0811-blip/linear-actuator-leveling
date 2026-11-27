# 리니어 액추에이터를 이용한 차체 수평 유지

2026년 11월 18일, 6 kg 차체의 더블 위시본 하부 암에 연결한 리니어 액추에이터로 앞뒤 기울기를 보정했다. 수평계로 기준 자세를 확인하고, 그 자세에서 측정한 IMU 롤값과 실시간 롤값의 차이를 차체 앞뒤 기울기 오차로 사용했다.

## 시험 결과

| 측정 항목 | 실측 결과 |
|---|---|
| 수평 맞추기 | 뒤 바퀴 높이를 조정한 뒤 차체 앞쪽이 1.8° 높았다. 액추에이터를 11 mm 수축해 명령 위치를 −10 mm에서 −21 mm로 옮겼고, 사용자가 수평계로 수평을 확인했다. |
| 정지 시험 | −21 mm에서 수평 자세를 41.3초 동안 관찰했다. 추가 보정 0회, 명령 이동량 0 mm로 사용자가 정지했다. |
| 동적 시험 | 새 수평 기준은 IMU 롤 1.53°였다. 제어가 활성화된 137.2초 동안 오차는 −1.41°에서 +2.19° 사이였다. |
| 마지막 10초 | 오차 범위는 −0.19°에서 +0.17°였고 마지막 샘플은 0.00°였다. |

[정지 시험의 위치 조정](evidence/2026-11-18/stationary/level_approach_after_rear_lower.json) · [수평계 확인과 유지 기록](evidence/2026-11-18/stationary/maintenance_state.json) · [동적 시험의 수평 기준](evidence/2026-11-18/dynamic/level_reference.json) · [동적 시험 측정 로그](evidence/2026-11-18/dynamic/trace.csv.gz)

동적 시험 수치는 측정 로그에서 `phase`가 `maintaining`인 13,721개 행을 골라 계산했다. 각 행의 오차는 `roll − 1.53°`이며 마지막 10초는 해당 구간의 마지막 샘플 시각을 기준으로 한다. 사용자가 제어를 일시정지한 뒤에도 센서 기록은 계속됐다. 이후 CP949 문자 인코딩 오류로 프로그램이 종료됐고 보드는 최종 정지 명령을 확인했다. [종료 상태](evidence/2026-11-18/dynamic/state.json)

## 장치와 제어

- Arduino Uno 호환 보드가 스테퍼 드라이버에 이동 펄스와 방향 신호를 보냈다. 시험에서 확인한 명령 환산값은 160펄스/mm였다. 위치 기록은 펄스로 계산한 값이며 기계적 변위의 독립 측정값은 없다. [장치 및 보정 기록](evidence/2026-11-18/setup/actuator_calibration_20261118/calibration.json)
- 동적 제어에서 설정한 사용 행정은 70 mm, 최대 명령 속도는 20 mm/s, 각도 데드밴드는 ±0.2°였다. 제어기는 기울기와 각속도를 이용해 목표 신축 속도를 계산했다. 정지 시험과 동적 시험은 위치 기준을 각각 설정했다. [제어기](src/dynamic/controller.py) · [동적 시험 펌웨어](src/dynamic/ActuatorSweep)
- 원본 호스트 코드는 시험 장치의 USB 식별값과 모터 포트 COM8을 사용한다. 당시 환경은 Windows, Python, pySerial 3.5와 Arduino Uno였다. 제어 코드를 살펴보거나 아래 계산 검사를 하는 데는 장치 연결이 필요 없다.

## 자료 구성

| 경로 | 내용 |
|---|---|
| [src/stationary](src/stationary) | 정지 시험용 호스트, 유지 제어기, 시험 계획, Arduino 스케치 |
| [src/dynamic](src/dynamic) | 동적 제어기, 초기 보정, 호스트, 각도 표시창, Arduino 스케치 |
| [src/three_wheel_teleop](src/three_wheel_teleop) | 3륜 차량 WASD 통합 제어 GUI, Jetson 워커, 고속(`~60 mm/s`)·진동 억제 능동 서스펜션 제어기 및 펌웨어 `v=10` (2026-11-27) |
| [evidence/2026-11-18](evidence/2026-11-18) | 장치 보정 기록, 각 시험의 상태 파일과 전체 IMU·직렬 통신 로그 |
| [SHA256SUMS](SHA256SUMS) | 수록한 원본 35개 파일의 SHA-256. `.gz` 파일은 압축을 푼 내용으로 대조한다. |

아래 두 명령은 Python 표준 라이브러리만 사용해 제어 방향, 이동 경계, 센서 입력, 보정 거리 계산을 검사한다.

```sh
python src/dynamic/controller.py
python src/dynamic/calibration.py
```

