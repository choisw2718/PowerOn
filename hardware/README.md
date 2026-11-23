# PowerOn 차량 하드웨어

부품별 장착 상태와 차량 제어에 쓰는 값을 여기서 관리합니다. 장착 상태는 최신 사용자 설명과 기존 시험 기록을 기준으로 합니다.

| 부품 | 차량 장착 상태 | 확인·준비 사항 |
| --- | --- | --- |
| [body-3.5 차체와 네 바퀴](components/chassis.ko.md) | 전륜 개별 조향, 후륜 개별 구동 | 설계 기준 150 × 200 mm, 완성차 축간거리·돌출 폭·회전 궤적 실측 |
| [Hitec HS-311 서보 2개](components/steering.ko.md) | 장착 확인 | D5/D6의 좌우 대응과 실물 최대 조향각 확인 |
| [좌우 후륜 모터·엔코더](components/propulsion.ko.md) | 좌우 한 세트씩 사용 | [JGA25-370 24 V / 620 RPM 배선과 보정](../vehicle/firmware/dual-rear.ko.md) |
| [L298N 모터 드라이버](components/propulsion.ko.md#l298n-모터-드라이버) | 연결 기록 있음 | 모듈 고정, 점퍼·전원 분기 확인 |
| [Arduino Nano 33 IoT](components/control-board.ko.md) | 사용 확인 · 차체 고정 미확인 | USB·Wi-Fi 응답 확인 기록 |
| [SHARP GP2Y0A21YK0F 거리 센서 3개](components/ir-sensors.ko.md) | **미장착 · 장착 계획 있음** | 왼쪽 A0, 전방 A1, 오른쪽 A2 계획 |
| [SLAMTEC RPLIDAR A1M8-R5](components/lidar.ko.md#rplidar-a1m8-r5) | 차량 장착 미확인 | USB 스캔 수신 확인. 위치와 방향 실측 필요 |
| [Raspberry Pi 4 Model B](components/lidar.ko.md#raspberry-pi-4-model-b) | 차량 장착 미확인 | 라이다 수집과 SSH 수신 시험에서 사용 |
| [배터리·전원 분기](components/power.ko.md) | 차량 탑재 구성 미확인 | 전원장치·배터리·DC-DC 모델 확인 |

## 제어와 시뮬레이션

- [차량 치수·조향·동역학 기준](control-parameters.ko.md)에서 값의 의미, 단위와 측정할 항목을 봅니다.
- [공용 프로필](control_profile.json)이 Python과 Nano 펌웨어의 기준입니다. 각 값의 `note`에 도면 추정, 현재 코드 설정, 임시값과 실측 필요 여부를 적습니다.
- 프로필을 바꾼 뒤 저장소 루트에서 `python -m hardware.generate_firmware_profile`을 실행하면 Arduino용 헤더가 갱신됩니다. `python tests/run_all.py`는 헤더와 프로필의 일치 여부를 검사합니다.

새 실측값을 기록할 때는 측정일, 센서·배터리 등 적재 상태, 노면과 측정 위치를 같이 남깁니다. 이전 결과는 [개발·실물 시험 기록](../docs/development-log.ko.md)에 있습니다.
