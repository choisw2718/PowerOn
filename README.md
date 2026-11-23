# PowerOn

Arduino Nano 33 IoT가 차량의 모터와 조향을 제어합니다. Windows에서 Wi-Fi로 차량을 조종하거나 라이다 화면을 확인하고, Raspberry Pi에서 라이다 벽 추종을 실행할 수 있습니다. 시뮬레이터와 벤치마크는 가상 트랙에서 동작합니다.

## 듀얼 후륜 구동

- **새 구성** · 좌우에 엔코더가 달린 **JGA25-370 (24 V, 620 RPM)** 모터를 한 개씩 사용합니다. 펌웨어는 듀얼 후륜을 기본 선택하며 보정 전에는 구동이 잠겨 있습니다. 기존 중앙 후륜으로 복귀하려면 배선을 원래 구성으로 되돌리고 공용 프로필의 `dual_rear.selected`를 `false`로 설정해 헤더를 재생성하고 다시 빌드합니다.
- **부품과 배선** · 기존 L298N A/B는 모듈과 실제 모터의 정격을 확인한 뒤 벤치 시험에 사용할 수 있습니다. 엔코더 입력, 제한 조건과 절차는 [듀얼 후륜 배선·보정](vehicle/firmware/dual-rear.ko.md)에 있습니다. Li-Po 선정은 보류합니다.
- **속도 제어** · 각 엔코더의 카운트로 후륜 선속도를 추정해 좌우 PWM을 독립 보정합니다. 직선에서는 두 후륜의 목표 선속도를 같게 하고, 코너에서는 조향각과 실측 후륜 윤거로 각 목표 선속도를 계산합니다.
- **검증 순서** · 바퀴를 띄운 상태에서 모터·엔코더 방향과 명령 만료 시 출력 차단을 확인합니다. 이후 저속 직선과 좌·우 원주행에서 속도·회전반경·정지거리를 측정하고 시뮬레이션과 주행 알고리즘에 반영합니다. 새 구성의 배선과 실차 시험은 예정 단계입니다.

## 실행

Windows에서는 [PowerShell 바로가기 모음](launchers/)에서 원하는 `.lnk`를 더블 클릭합니다. `PowerOn PowerShell.lnk`는 저장소 루트에서 PowerShell을 열며, 기능별 바로가기는 해당 기능을 실행한 뒤 PowerShell 창을 유지합니다. `LiDAR SSH Preview.lnk`는 Pi에서 받은 스캔을 10초 동안 사용해 조향과 출력을 미리 계산합니다.

프로젝트 위치를 옮겼거나 바로가기를 새로 만들 때는 저장소 루트에서 PowerShell 7로 `.\launchers\New-Shortcuts.ps1`을 실행합니다.

다음 명령은 저장소 루트에서 실행합니다.

| 작업 | 명령 또는 파일 | 안내 |
| --- | --- | --- |
| Nano 펌웨어 | [PowerOnNano.ino](vehicle/firmware/PowerOnNano/PowerOnNano.ino) | [업로드](vehicle/firmware/upload.ko.md) · [배선과 조향 보정](vehicle/firmware/wiring.ko.md) |
| 수동 조종 | `python -m vehicle` 또는 [control-car.cmd](vehicle/manual_control/control-car.cmd) | [연결과 조종 키](vehicle/manual_control/guide.ko.md) |
| 듀얼 후륜 확인 | `python -m vehicle --rear-status` | [엔코더 조회·벤치 펄스·폐루프 속도 시험](vehicle/firmware/dual-rear.ko.md#설정과-시험-순서) |
| IR 센서 모니터 | `python -m vehicle --sensors` | [설정과 PowerShell 실행](vehicle/sensor_monitor/guide.ko.md) |
| IR 벽 추종 미리보기 | `python -m vehicle --preview-csv vehicle/ir_wall_following/examples/ir_wall_follow_preview.csv` | [IR 벽 추종](vehicle/ir_wall_following/guide.ko.md) |
| 라이다 레이더 | `python -m lidar.radar` 또는 [lidar-radar.cmd](lidar/radar/lidar-radar.cmd) | [RPLIDAR와 Pi 설정](lidar/acquisition/setup.md) · [레이더 사용법](lidar/radar/guide.md) |
| 라이다 자율주행 | `python -m lidar.autopilot` | [미리보기와 Pi 배포](lidar/autopilot/guide.ko.md) · [경로 계획](lidar/autopilot/planning/track-planning.ko.md) |
| 라이다 자율·수동 전환 | 기존 단일 후륜 펌웨어용 `--drive --interactive`. 현재 듀얼 후륜 펌웨어는 Pi의 `LIDAR DRIVE`를 거절하며 계산 전용 실행과 기록을 지원합니다. | [키와 자동 복귀](lidar/autopilot/guide.ko.md#자율주행과-수동조종-전환) |
| 트랙 시뮬레이터 | `python -m simulation.track` 또는 [lidar-simulator.cmd](simulation/track/lidar-simulator.cmd) | [코스 목록](simulation/track/guide.ko.md#코너별-순환-트랙) · [트랙과 센서 설정](simulation/track/guide.ko.md) |
| 벤치마크 | `python -m simulation.benchmark` | [반복 시험과 결과 해석](simulation/benchmark/guide.ko.md) |
| 테스트 | `python tests/run_all.py` | Python 3, 호스트 g++, 라이다 Python 테스트용 pyserial |

개인 Wi-Fi 설정은 펌웨어 스케치의 `wifi_secrets.h`에 저장합니다. 신규 설정은 [공개 예시](vehicle/firmware/PowerOnNano/wifi_secrets.example.h)를 복사해 만듭니다.

## 차량 하드웨어

[부품별 장착 상태](hardware/README.md), [차체 치수와 제어 기준](hardware/control-parameters.ko.md), [시뮬레이션·제어 공용 프로필](hardware/control_profile.json)은 `hardware/`에서 관리합니다.

**현재 차량은 앞바퀴별 조향 서보 2개, 뒷바퀴별 모터·엔코더 2개를 사용합니다.** 차체 설계 기준은 `body-3.5.f3d`의 150 × 200 mm입니다. 네 바퀴 모델에서 좌우 접지 차이, 모터 고착, 엔코더 단절, 벽 반사와 문턱을 시험할 수 있습니다. [가상 트랙](simulation/track/guide.ko.md) · [조건별 벤치마크](simulation/benchmark/guide.ko.md)

## 주행 데이터

노트북에서 수동 조종을 시작하면 프로젝트의 `주행 데이터`에 한국 시각과 고유 번호로 회차가 생깁니다. Pi 자율·수동 통합 조종은 Pi에 먼저 기록합니다. 노트북 조종기는 Pi 연결 중 완료된 기록을 수거합니다. 회차별 원본 스캔, 명령·응답, 코드·설정과 하드웨어 변경 이력의 확인 방법은 [기록과 재생](vehicle/drive_data/guide.ko.md)에 있습니다.

## 개발 기록과 참고 자료

- [개발·실물 검증 기록](docs/development-log.ko.md)
- [Uno USB 시리얼 제어](reference/Uno_Single_Rear_Motor/README.md)
- [Nano Wi-Fi 진단 스케치](reference/springmaru_poweron_nano33_iot/README.md)
