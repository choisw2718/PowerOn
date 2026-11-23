# 라이다 트랙 시뮬레이터

명령줄 예제는 저장소 루트에서 실행합니다.

## 현재 4륜 차량

기본 실행은 `body-3.5` 설계의 판과 네 바퀴를 사용합니다. 앞바퀴는 각 서보의 애커만 목표각·펄스 한계와 실제 응답을 따르고, 뒤에서는 좌우 모터의 접지 이동량과 회전량을 따로 계산합니다. 조향 중 타이어 돌출은 충돌 검사에 포함됩니다. 엔코더 피드백은 정수 틱과 표본 시각에서 계산합니다. `--no-feedback`을 주면 외부 운동 계측이 없는 Pi의 속도 상한과 관측 여유를 적용합니다.

```powershell
python -m simulation.track --scenario loop-narrow --environment indoor-mixed --seed 7
python -m simulation.track --headless --scenario furniture --environment rough-floor --steps 65 --seed 7
python -m simulation.track --headless --scenario blocked-straight --steps 65 --seed 7
```

| 환경 | 달라지는 조건 |
| --- | --- |
| `nominal` | 명목 차량. `--stress`로 스캔·명령 단절과 저접지 구간을 추가 |
| `split-grip`, `weak-returns` | 한쪽 타이어가 미끄러운 바닥을 먼저 밟거나, 특정 벽 구간의 반사율 감소 |
| `rough-floor` | 완만한 경사와 문턱에서의 구동 하중 및 센서 기울기 |
| `indoor-mixed` | 출발 위치·방향, 서보 편차·응답, 좌우 구동력·타이어 둘레, 전압·적재, 바닥과 벽 조건을 시드로 결합 |
| `blocked`, `motor-stall`, `encoder-outage` | 출발 직선에 상자 설치, 4초부터 오른쪽 후륜 잠금, 4초부터 양쪽 엔코더의 호스트 측 샘플 전송 중단을 각각 적용 |

같은 시드는 실내 조건과 라이다 잡음을 재현합니다. 환경과 광선 노이즈는 별개의 난수 생성기를 사용합니다. `--stress`는 추가로 지정된 시각이나 위치에 고장을 겹칩니다. 장애물은 라이다와 충돌 판정에 함께 사용합니다. 약반사 벽에서는 반환 확률과 품질이 낮아져도 차체가 벽에 닿을 수 있습니다.

### 실차와 차이를 만드는 변수

`--lidar a1-cadence`는 보관 중인 2026-11-11 COM19 표준 스캔의 취득 속도를 재현합니다. 호스트가 받은 연속 회전 표식 30개 사이에서 4.500초 동안 29회전·8,975개 표본을 얻어 고정 주기 155 ms와 초당 1,994점을 사용합니다. 유효 거리는 당시 실내 배치에서 6,556개였습니다. 가상 코스에서는 주변 물체와 표면에 따라 유효 거리를 새로 계산합니다. 실제 캡처의 구간별 호스트 수신 간격은 140–172 ms였으며 이 프로필은 그 평균만 재현합니다.

| 항목 | 현재 가상차에 반영한 내용 | 실차에서 확보할 값 |
| --- | --- | --- |
| 차체·센서 배치 | CAD 판 150 × 200 mm, 회전하는 앞 타이어와 링크, 후륜·범퍼를 포함한 외곽. 판 중심은 앞뒤 차축의 중점으로 가정 | 적재 상태 축간거리, 앞 피벗·뒤 바퀴 중심 간격, 범퍼·배선 돌출, 센서의 전후·좌우 위치와 높이 |
| 조향 | 좌우 애커만 목표, 공통 회전반경의 서보 펄스 한계, 채널별 응답과 중립 오차. 타이어 모서리가 도달하는 범위로 경로 검사 | 실제 D5/D6 대응·부호, 중심 명령별 양쪽 바퀴 각도, 좌우 회전반경, 부하별 지연·유격·기계 간섭 |
| 구동·전원 | 뒷바퀴마다 선회 목표속도·가감속·접지·회전량을 계산. 전압·적재는 구동력과 응답의 배율로 근사 | 바퀴별 기동/유지 PWM, 하중별 유효 둘레·속도·제동거리, 배터리 충전·부하 전압, 드라이버 발열과 감속기 손실 |
| 엔코더 | 바퀴의 회전량을 정수 카운트로 바꾸고 샘플 간 변위로 자세·속도·회전율을 추정. 공회전 또는 고착 시 추정 자세와 차체 위치가 달라짐 | 좌우 유효 CPR·부호, 시간 동기, 누락·노이즈, 미끄러운 바닥에서 외부 기준과의 위치 차이 |
| 노면 | 좌우 다른 접지, 가감속·선회의 결합 한계, 경사와 문턱에서의 하중 변화·바닥 반사 | 타이어 재질·마모, 바닥 오염·이음매·단차, 구배·경사 방향, 적재 위치·질량중심에 따른 차륜별 하중 |
| 라이다·가시성 | 회전 스캔, 점별 시각, 각도·거리 편차, 입사각, 가림·약반사, 진동·거리 튐, 수신 지연과 누락 | 회전 주기·각도별 유효율, 높이·방향·장착 흔들림, 벽 색/광택·유리·창가 조명, 낮은 장애물의 가시성 |
| 시간·코스 | 명령 및 센서 지연, 연속 누락, 오래된 스캔, 명령 만료·복귀, 좁은 코너·벽 단절·가구·막힌 길 | 제어 루프와 통신의 지연 분포, 재연결 시간, 실제 정지거리, 출발 자세와 코스 실측, 이동 장애물의 동선 |

기본 추정 프로필에서 직진 충돌 외곽은 폭 223.2 mm, 길이 210 mm입니다. 조향 시에는 앞 타이어의 위치에 따라 외곽을 매번 다시 계산합니다. 보정 모드의 계획기는 전체 조향 범위를 덮는 폭 약 239.5 mm, 라이다에서 앞 125.8 mm·뒤 105 mm를 사용합니다. 후륜 트레드 145 mm, 타이어 한 바퀴 188.5 mm, 유효 360카운트는 임시 가상값입니다. 개별 후륜 실측값이 공용 프로필에 입력되면 해당 값을 우선 사용합니다. [치수와 보정 상태](../../hardware/control-parameters.ko.md)

환경 모델은 저속 평면 운동을 다룹니다. 실제 전류·온도, 개별 타이어의 변형, 차체 피치·롤 진동 전체, 유리의 다중반사, 사람이나 물체의 이동은 별도 실험이 필요한 변수입니다. 모델의 차체 위치와 실제 조향각은 성능·오차 평가에 사용합니다. 기본 4륜 주행기가 받는 운동 관측은 후륜 엔코더 두 개에서 추정한 값입니다.

2026-11-11 USB 수신 자료는 완전한 29회전에 회전당 309~310점, 유효 220~234점이 있었습니다. 가상 `rolling` 기본 설정은 166 ms와 초당 1,950점으로 회전당 약 324점입니다. 실제 CSV의 시각은 같은 수신 묶음에 속한 여러 점이 공유합니다. 이 기록은 패킷, 각도 순서, 무반환과 회전별 분포를 재생하는 데 사용하며, 거리 오차·주행 성능 보정에는 위치나 거리 기준을 함께 수집해야 합니다. [기록 스캔 재생](../../lidar/autopilot/guide.ko.md) · [실제 USB 수집 조건](../../docs/development-log.ko.md#2026-11-11-rplidar-a1m8-r5-usb-거리-데이터-수집-확인)

이전 단일 조향각·속도 모델의 수치는 `--vehicle-model legacy`로 비교할 수 있습니다. 아래 예전 시험의 피드백·랩타임은 당시 모델에서 얻은 값입니다. 실제 계측과 대조할 때는 센서·차량·환경·시드를 함께 기록합니다.

## 가상 트랙에서 시험

Windows에서 [lidar-simulator.cmd](lidar-simulator.cmd)를 더블 클릭합니다. 기본 시나리오는 **대형 복합 순환 트랙**입니다. 약 13 × 7.6 m 안에 12 m 직선, 간격이 짧은 연속 코너, 좌우 전환, 한쪽 벽의 비대칭 변화로 직선부 폭이 0.79~1.28 m가 되는 구간을 배치했습니다. **시작**을 누릅니다. **2×**, **4×**, **최대**로 표시 속도를 높일 수 있습니다.

경로의 색으로 저속, 중속, 고속 구간을 확인합니다. 오른쪽 패널에서 구간별 경과 시간과 평균 속도를 읽습니다. 결승선을 포함한 7개 지점을 순서대로 정상 방향에서 통과해야 1바퀴로 계산합니다. 주행 알고리즘은 라이다 스캔을 사용해 조향과 속도를 구합니다. 통과 지점은 시뮬레이터의 계측에 사용합니다.

**한 스캔**을 누르면 설정한 회전 주기만큼 시간이 진행됩니다. 주행 중 **전방 스캔 끊기**를 선택하면 이후에 측정한 정면 광선의 반사가 사라집니다. 단일 사례를 확인하려면 **소형 사각 순환 트랙** 또는 **긴 직선 → 우회전**을 선택합니다.

### 코너별 순환 트랙

상단 **시나리오**에서 코스를 선택합니다. 각 코스는 결승선을 포함한 일곱 지점을 올바른 방향으로 통과하면 한 바퀴로 계산합니다. 화면 오른쪽에서 코너 진입·탈출 구간의 경과 시간과 속도를 비교합니다.

| 시나리오 | 주요 조건 | 확인 항목 |
| --- | --- | --- |
| `loop-hairpin` 헤어핀 순환 | 중심선 반경 0.46 m, 폭 0.70 m의 180° 코너 두 개 | 직선 끝 감속, 좁은 반경 유지, 반대 방향 직선으로 복귀 |
| `loop-narrow` 협로 순환 | 진입 폭 0.96 m, 90° 코너 폭 0.56 m, 중심선 반경 0.60 m | 폭이 줄어들 때 조향 안정성, 앞범퍼와 옆면 여유 |
| `loop-s` S자 순환 | 길이 방향 5 m 구간, 최대 옆 이동 0.96 m, 폭 0.76 m | 연속 방향 전환에서 조향 지연과 경로 흔들림 |
| `loop-offset` 비대칭 순환 | 양쪽 벽이 번갈아 돌출되고 복귀 직선에 0.22 m 벽 단절 | 비대칭 폭에 대한 중심 복귀, 벽 하나가 끊긴 구간 통과 |

```powershell
python -m simulation.track --scenario loop-s --feedback
python -m simulation.track --headless --scenario loop-hairpin --feedback --laps 1
python -m simulation.benchmark --scenario loop-hairpin loop-narrow loop-s loop-offset --cases clean --seeds 7 23
```

기존 경로 점수와 기본 회전 스캔에서 S자와 비대칭 코스는 시드 7과 23 모두 충돌 없이 완주했습니다. 폭 0.56 m 코너도 두 시드에서 완주했으며 최소 벽 여유는 7.3~7.4 cm로 벤치마크의 10 cm 기준에 미달했습니다. 헤어핀에서 시드 7은 두 번째 180° 코너에서 정지했고 시드 23은 완주했으나 벽 여유가 7.7 cm였습니다. `mixed` 조건은 S자와 비대칭 코스에서 시드 7로 각각 충돌 없이 완주했습니다.

**차량 동역학과 지연**에서 **차체 여유 우선**을 선택하면 코너의 근거리 차체 여유를 더 크게 반영한 경로 점수를 시험합니다. 명령줄에서는 `python -m simulation.track --scenario loop-hairpin --planner clearance --feedback`으로 엽니다. [구간별 평가와 경로 계획 비교](../benchmark/guide.ko.md#구간별-평가와-경로-계획-비교)

짧은 누락과 복구를 한 번에 확인하려면 아래 명령을 실행합니다. `--drop-front-steps`가 없으면 전방 누락이 지속됩니다. `bridged_scans`는 확인된 경로로 이어간 스캔 수입니다. `waiting_scans`는 모터 출력을 끄고 통로를 다시 확인한 스캔 수입니다.

```powershell
python -m simulation.track --headless --scenario loop --feedback --laps 1 --drop-front-at 28 --drop-front-steps 1
python -m simulation.track --headless --scenario straight --feedback --steps 38 --drop-front-at 12 --drop-front-steps 8
```

대형 트랙 직선에 한 스캔 누락을 넣어 대기 없이 37.8 m를 완주했습니다. 코너 부근에 한 스캔 누락을 넣은 별도 시험에서는 복귀 코너에서 두 스캔 동안 출력을 끄고 통로를 확인한 뒤 한 바퀴를 마쳤습니다. 8스캔 누락 시험은 감속 후 자동 복귀했고, 지속 누락은 출력이 꺼진 상태에서 복구 대기 시간이 끝나며 종료되었습니다. 충돌은 없었으며, 누락을 넣은 순환 주행에서는 순간적으로 모델의 횡가속 한계를 초과했습니다.

## 차량 동역학과 지연

기준 차량의 차체 치수·회전 반경·동역학 기본값은 [공용 하드웨어 프로필](../../hardware/control_profile.json)에서 읽습니다. [측정 필요 항목과 임시값](../../hardware/control-parameters.ko.md)을 확인할 수 있습니다.

**차량 동역학과 지연**에서 센서·계산 지연, 명령 지연, 최소 회전반경, 조향 변화율, 가속·감속도를 바꿉니다. 기동 PWM, 상한 PWM에서의 속도, 횡가속 한계도 입력할 수 있습니다. 값을 바꾼 뒤 **적용 후 초기화**를 누릅니다.

**조향·구동 불일치** 프리셋은 조향 배율 0.82, 중립 편차 2°, 조향 유격 0.7°, 구동 배율 1.17과 느린 감속을 적용합니다. **주행 피드백 사용**을 켜면 4륜 모델은 좌우 엔코더의 이동량으로 속도와 회전율을 추정합니다. 피드백을 끄면 회전 스캔을 차량 자세로 보정할 수 없어 Pi의 미계측 속도 상한과 이동 여유를 적용합니다. 창 없이 같은 조건을 비교할 수 있습니다.

```powershell
python -m simulation.track --headless --scenario long-bend --profile mismatch --steps 170 --no-feedback
python -m simulation.track --headless --scenario long-bend --profile mismatch --steps 170 --feedback
python -m simulation.track --headless --scenario long-bend --profile mismatch --throttle-gain 1.5 --steps 170 --feedback
```

피드백을 켠 대형 순환 트랙에서 기본 동역학 2바퀴, 조향·구동 불일치 조건 1바퀴, 회전 250 ms와 초당 1,500점 조건 1바퀴를 완주했습니다. 불일치 조건에서는 이전의 22.2 m 지점 정지를 재현한 뒤 실측 조향 보정을 적용해 38.0 m를 주행했습니다. 해당 폐회로 시험에 충돌과 모델 횡가속 한계 초과가 없었습니다. 별도 구동 배율 1.50 시험에서는 실측 과속을 감지해 정지 절차를 시작했고, 모델 차량은 관성으로 약 0.47 m 더 이동했습니다.

```powershell
python -m simulation.track --headless --scenario loop --feedback --laps 2
python -m simulation.track --headless --scenario loop --feedback --laps 1 --profile mismatch
python -m simulation.track --headless --scenario loop --feedback --laps 1 --scan-ms 250 --point-rate-hz 1500
```

같은 조건에서 **입력값을 주행 계획에도 반영**을 켠 경우와 끈 경우를 비교합니다. 켜면 알고리즘이 입력한 물리 제약을 반영합니다. 끄면 가상 차량은 같은 물리 조건으로 움직이고 알고리즘은 기본 추정값으로 계획합니다. 화면에는 모델 속도, 목표 속도, 모델 조향, 명령 조향, 적용 PWM, 정지 판정 후 이동 거리가 표시됩니다. 횡가속 한계를 초과하면 모델의 회전반경을 넓혀 코너 바깥으로 밀리는 경향을 단순화해 나타냅니다.

### 회전 중에 측정한 점 해석

1. **라이다 측정과 보정**에서 **회전 스캔**을 고릅니다. 기본 회전 주기는 166 ms이고 측정 속도는 초당 1,950점입니다. 한 바퀴에서 약 324점을 받습니다. 각 점의 측정 시각과 각도는 독립적인 측정 간격과 회전 속도로 계산합니다.
2. **움직임 보정**과 **거리·각도 보정**을 켜고 **적용 후 초기화**를 누릅니다. 광선마다 측정 당시의 추정 자세를 사용합니다. 광선의 시작점과 반사점 모두를 회전 종료 시점의 차량 좌표에 맞춥니다. 자세는 별도로 샘플링하고 보간합니다. 거리 오차와 설치 방향 오차도 각각 보정합니다.
3. **움직임 보정**을 끄고 같은 경로를 다시 실행합니다. 0.42 m/s에서 한 바퀴가 166 ms이면 첫 점과 마지막 점 사이에 차량이 약 7 cm 이동합니다. 회전 중에는 차의 방향도 바뀝니다. 화면의 RMS는 해당 스캔에 있는 유효 반사점의 가상 벽 기준 위치 오차입니다. 터미널의 `moving_*_rms`는 주행 속도가 0.05 m/s 이상인 스캔 전체에서 계산합니다.
4. **오차·누락 증가**를 선택하거나 회전 주기와 초당 측정점, 최대 측정 거리를 변경합니다. 모델은 회전 속도 변동, 거리 노이즈, 비스듬하게 맞은 광선에서 증가하는 노이즈와 누락, 거리·장착각 편차, 주행 자세 추정 오차를 포함합니다. 시뮬레이터의 최대 거리 기본값은 A1M8-R5의 흰색 물체 기준인 12 m입니다. 벽에서 반사가 약한 경우를 시험하려면 최대 거리를 낮춥니다. [SLAMTEC A1M8 데이터시트](https://bucket.download.slamtec.com/7fe7e3656e811ab1a645753af40809f05fa7ddcd/LD108_SLAMTEC_rplidar_datasheet_A1M8_v2.4_en.pdf)

유효한 광선은 반사점 직전까지 자유 공간의 근거가 됩니다. 반사점 뒤와 반환값이 없는 방향은 관측이 부족한 곳으로 취급합니다. 벽 거리 계산에는 후보 경로의 각 단면에서 가까운 반사점 여러 개의 중앙값을 사용합니다. 조향과 속도는 확인된 경로 길이, 스캔의 시간 범위, 감속도와 회전반경을 함께 사용합니다. [ROS LaserScan 시간 정의](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg) · [ROS 레이저 좌표 변환 구현](https://github.com/ros-perception/laser_geometry/blob/rolling/src/laser_geometry.cpp)

#### 진동과 거리 튐 시험

**진동·거리 튐**은 주행에 따라 센서 위치와 방향이 흔들리는 현상, 개별 반사점의 거리 튐, 회전 속도의 변동을 넣습니다. 자세는 25 ms 간격으로만 제공됩니다. `--ray-clock phase`는 각도에 비례해 시각을 배정하는 이전 방식이며 `--ray-clock exact`는 가상 측정기의 실제 시각을 비교 기준으로 사용합니다. `--no-spike-guard`는 고립 반사점 검사를 끄고 비교합니다.

```powershell
python -m simulation.track --scenario long-bend --lidar vibration --feedback
python -m simulation.track --headless --scenario long-bend --steps 125 --lidar vibration --feedback
python -m simulation.benchmark --scenario loop-narrow --cases clean --seeds 7 --planners clearance --lidar vibration --scan-ms 250 --point-rate-hz 1500 --json
```

`front_*_range_mm`과 `front_*_bearing_deg`는 회전 종료 시점 기준 전방 ±45° 반사점의 거리와 방향의 RMS 오차입니다. 원본과 보정값은 동일한 유효 반사점으로 비교합니다. `range_spikes`는 가상 거리 튐 횟수, `far_rejected`는 멀게 튄 것으로 판단해 제외한 횟수, `near_return_holds`는 가까운 반사점을 확인하느라 전진 출력을 멈춘 스캔 수입니다. 벤치마크 JSON에서는 `scan_motion` 아래에서 확인합니다. 진동 모델은 2차원 위치와 방향의 흔들림을 시험합니다.

진동 프로필의 긴 직선과 코너 시험에서 전방 거리 RMS 오차는 44.20 mm에서 16.65 mm로, 방향 RMS 오차는 2.24°에서 0.50°로 줄었습니다. 같은 프로필에서 헤어핀과 회전 250 ms의 폭 56 cm 코너는 벽에 닿기 전에 정지했고 미완주로 기록됐습니다.

창 없이 같은 가상 트랙을 실행할 수 있습니다.

```powershell
python -m simulation.track --headless --scenario loop --laps 1
python -m simulation.track --headless --scenario loop-sharp --laps 1
python -m simulation.track --headless --scenario loop --laps 1 --scan-ms 250 --point-rate-hz 1500
python -m simulation.track --headless --scenario loop --laps 1 --lidar harsh
python -m simulation.track --headless --scenario loop --laps 1 --max-range-m 8
python -m simulation.track --headless --scenario long-bend --steps 130
python -m simulation.track --headless --scenario long-bend --steps 130 --no-deskew
python -m simulation.track --headless --scenario long-bend --scan-ms 250 --point-rate-hz 1500
python -m simulation.track --headless --scenario straight --drop-front-at 16
```

대형 트랙 로그의 `checkpoint=4/7`은 4번 지점을 통과했다는 뜻입니다. 최종 줄에서 완주 수, 충돌, 정지 이유를 확인합니다. `loop-sharp`는 짧은 간격 구간의 사선 벽을 직각으로 바꿉니다. `--laps`를 지정한 실행에서 목표 바퀴를 채우지 못하거나 충돌하면 종료 코드가 1입니다.

기본 시드와 움직임 보정으로 대형 트랙에서 2바퀴, 총 75.8 m를 충돌 없이 주행했습니다. 250 ms 회전, 오차·누락 증가, 연속 직각 조건도 각각 1바퀴를 충돌 없이 완주했습니다. 기본 조건의 2바퀴 주행에서는 횡가속도 한계 초과가 기록되었습니다. 모델은 이때 실제 회전 반경을 넓힙니다.

비교하려면 `--lidar ideal`로 순간 스캔을, `--lidar harsh`로 센서 오차·누락이 더 많은 조건을 실행합니다. `--seed` 값으로 노이즈와 누락 패턴을 바꿀 수 있습니다. 이 모델은 고정된 벽과 저속 차량의 운동을 단순화합니다. 실차에서 측정한 값으로 바꾼 다음 트랙 주행을 별도로 검증합니다.

[반복 스트레스 시험](../benchmark/guide.ko.md#반복-스트레스-시험)
