# 라이다 시뮬레이션 벤치마크

저장소 루트에서 명령을 실행합니다.

## 4륜 차량에서 조건 비교

기본값은 독립 앞바퀴 조향과 듀얼 후륜 모터·엔코더를 가진 `four-wheel`, 엔코더 피드백 사용, `clearance` 경로 점수, `scan-coast` 복구 정책입니다. `--scenario`는 코스, `--environment`는 바닥·벽·차량 조건, `--cases`는 시각 또는 위치로 주입하는 고장을 선택합니다. 차량·센서·환경·시드를 고정하고 비교할 항목만 바꿉니다.

```powershell
python -m simulation.benchmark --scenario loop-narrow loop-s --environment nominal --cases clean mixed --seeds 7 23 --planners legacy clearance --jobs 2
python -m simulation.benchmark --scenario loop --environment weak-returns --cases clean --seeds 41 --json
python -m simulation.benchmark --scenario loop --environment encoder-outage --cases clean --seeds 7
python -m simulation.benchmark --scenario loop --environment motor-stall --cases clean --seeds 7
python -m simulation.benchmark --scenario loop --environment blocked --cases clean --seeds 7
```

`--planners legacy clearance`는 같은 차량 모델에서 경로 선택 알고리즘을 비교합니다. `--vehicle-model legacy`는 한 개의 조향각·속도와 과거 제어 흐름으로 주행합니다. `--no-feedback`을 추가하면 차량 운동 측정이 없는 조건을 시험합니다. JSON의 `vehicle_model`, `environment`, `observation`과 좌우 `rear_encoder_counts`를 함께 읽어 실행 조건을 확인합니다. [실내 환경과 실차 측정 변수](../track/guide.ko.md#실차와-차이를-만드는-변수)

### 네 바퀴 모델 검증 결과

같은 4륜 차량에서 두 경로 점수 방식을 비교했습니다. 기본 회전 스캔은 166 ms, 벽 여유 기준은 10 cm입니다. 시드 7·23으로 조정하고 53·89는 이후 검증에 사용했습니다. 표의 통과에는 완주, 벽 여유, 관측 복구와 정체 기준을 모두 적용합니다.

| 코스와 조건 | 기존 방식 | 변경한 방식 | 판정에 영향을 준 관측 |
| --- | --- | --- | --- |
| 폭 56 cm, 정상, 시드 4개 | 완주 0/4 | 완주 4/4, 통과 0/4 | 63.73–64.94초에 완주. 첫 코너 안쪽에서 최소 여유 8.33–8.84 cm |
| 폭 56 cm, 복합 고장, 같은 시드 | 완주 0/4, 최소 7.76–8.63 cm | 완주 0/4, 최소 9.62–11.51 cm | 4회 모두 전진 공간을 다시 확인하지 못해 정지. 3회는 정지까지 10 cm 이상 유지 |
| 헤어핀, 정상 / 복합, 시드 53 | 통과 0/2 | 통과 1/2 | 정상 최소 여유 8.04 → 13.98 cm. 복합에서는 최소 9.84 cm로 정지해 미완주 |
| S자, 정상 / 복합, 시드 53 | 통과 1/2 | 통과 1/2 | 정상 완주. 복합에서는 두 방식 모두 엔코더 추정 과속으로 정지 |
| 비대칭 벽, 정상 / 복합, 시드 53 | 통과 2/2 | 통과 2/2 | 변경 후 최소 여유 22.67 / 20.86 cm |
| 비대칭 벽, 실내 복합 환경과 A1 실측 주기, 시드 89 | 비교 생략 | 통과 2/2 | 정상·복합 고장 조건에서 69.40 / 72.08초, 최소 여유 22.34 / 22.29 cm |

고장 시험은 시드 7·53에서 각각 반복했습니다. 주행 중 명령 단절은 1.080초, 라이다 단절은 0.970초, 엔코더 전송 중단은 0.930초, 오른쪽 뒷바퀴 고착은 0.995초 만에 속도가 0.03 m/s 아래로 떨어졌습니다. 이후 모두 출력 0으로 정지했습니다. 막힌 통로에서는 정지한 차체와 상자 사이에 44.93 cm가 남았습니다. 위 표와 고장 시험에서 충돌은 0건입니다.

엔코더를 제공하지 않은 시드 53의 정상 협로 시험은 충돌 없이 멈추고 미완주했습니다. 이 설정의 명령 단절 시험은 고장을 주입할 때 이미 정지 상태여서 주행 중 정지 검증에 포함하지 않았습니다. 모든 수치는 지정한 가상 지면·센서·구동 설정에서 얻은 결과입니다.

정상 협로에서 최소 여유가 생긴 곳은 첫 코너 진입 후 지도 좌표 x=5.32–5.41 m, y=0.19–0.27 m입니다. 저접지를 더한 시드 7의 정지 지점에서는 가상 차체가 약 59° 돌았을 때 후륜 엔코더는 약 78° 회전한 것으로 추정했습니다. 이 조건에서 계획기는 다음 전진 명령에 필요한 공간을 확인하지 못했습니다. 관측된 협로 코너에만 속도 상한 0.18 m/s를 둔 별도 시험에서도 정상 시드 7의 최소 여유는 8.84 cm였고 복합 조건은 미완주였습니다. 기본 제어기는 충분한 공간이 보일 때만 전진하는 조건을 유지합니다.

```powershell
python -m simulation.benchmark --scenario loop-narrow --cases clean mixed --seeds 7 23 53 89 --planners legacy clearance --jobs 4
python -m simulation.benchmark --scenario loop-hairpin loop-s loop-offset --cases clean mixed --seeds 53 --planners legacy clearance --jobs 3
python -m simulation.benchmark --scenario loop-offset --environment indoor-mixed --lidar a1-cadence --cases clean mixed --seeds 89
python -m simulation.benchmark --scenario loop --cases watchdog sensor-outage --seeds 7 53
```

## 반복 스트레스 시험

```powershell
python -m simulation.benchmark --cases clean sensor transport traction mixed watchdog --seeds 7 23
python -m simulation.benchmark --cases clean mixed --seeds 7 --json
.\simulation\track\lidar-simulator.cmd --feedback --stress mixed --audit-leases
```

| 조건 | 시험 내용 |
| --- | --- |
| `clean` | 기존 회전 스캔, 동역학과 피드백 조건에서 한 바퀴를 확인합니다. |
| `sensor` | 주행 도중 전방 광선이 연속으로 사라지거나 회전 한 바퀴가 통째로 빠집니다. |
| `transport` | 스캔 및 명령 일부에 지연을 넣고 특정 명령을 차량에 전달하지 않습니다. 기준보다 늦게 도착한 스캔은 계획에 쓰지 않습니다. |
| `traction` | 지정된 두 구간에서 접지 한계를 낮춥니다. 가속·감속과 선회에 필요한 접지 요구를 타원으로 합산합니다. |
| `mixed` | 센서 누락, 전달 지연과 저접지 구간을 동시에 적용합니다. |
| `watchdog` | 명령이 850 ms 동안 전달되지 않는 조건에서 모터가 꺼지고 차가 멈추는지 확인합니다. |
| `sensor-outage` | 스캔이 계속 끊겨도 모터 출력을 끄고 제한 시간 안에 차량이 멈추는지 확인합니다. |

`--scenario loop-sharp`를 추가하면 연속 직각 코너가 있는 트랙에서 같은 시험을 실행합니다. `--profile mismatch`는 조향과 구동의 불일치를 추가합니다. `--lidar harsh`는 측정 오차와 누락을 늘립니다. `--scan-ms 250 --point-rate-hz 1500`은 회전 하나에 250 ms가 걸리고 초당 1,500점을 얻는 조건입니다. 고장 일정은 시뮬레이션 시각을 기준으로 하므로 회전 주기를 바꾸면 실제로 영향을 받은 스캔 수가 달라질 수 있습니다.

`--scenario` 뒤에 여러 코스를 넣으면 같은 조건을 코스별로 반복합니다. `traction`과 `mixed`에서는 새 코스의 주요 코너나 방향 전환 구간에 저접지를 적용합니다. `--json` 결과의 `next_checkpoint`는 미완주 시 다음 통과 지점의 이름입니다. [코스 치수와 구성](../track/guide.ko.md#코너별-순환-트랙)

```powershell
python -m simulation.benchmark --scenario loop-hairpin loop-narrow loop-s loop-offset --cases clean mixed --seeds 7 23
```

`gap`은 최대 5 ms 간격의 물리 계산 시점에서 얻은 차체와 벽 사이의 최소 거리입니다. 4륜 모델은 조향 중 타이어 돌출과 범퍼를 감싸는 직사각형으로 측정합니다. `@` 뒤 좌표는 그 순간의 라이다 중심 위치입니다. `wait`는 출력 없이 관측을 기다린 스캔 수입니다. `stops`는 차가 출발한 뒤 속도 0.03 m/s 미만이 0.2초 이상 이어진 횟수입니다. `low_grip`은 차량이 저접지 구간에서 움직인 시간입니다. `grip_limit`은 가감속·선회의 결합 요구가 설정 접지 한계의 105%를 넘은 시간입니다.

`lease_short`는 전진 경로의 유효시간이 다음 스캔까지 필요한 최소치에 부족했던 횟수입니다. 4륜 모델은 Pi와 공용인 명령 판정에서 스캔 나이와 정지 공간을 매번 확인하며, 여유가 부족하면 출력을 0으로 내리고 다시 관측합니다. `worst`는 가장 큰 부족량과 시각·지도 좌표를 표시합니다. 이전 `legacy` 모델은 정상 명령의 거리 예산을 사후 집계합니다. 실제 TCP·보드 타이밍은 실차 로그와 별도로 대조해야 합니다.

## 가상 주행 평가 기준

| 항목 | 초기 합격 기준 |
| --- | --- |
| 충돌과 차체 여유 | 모든 시험에서 충돌 0건, 측정된 최소 차체–벽 거리 0.10 m 이상 |
| 완주와 주행 지속 | `clean`, `sensor`, `transport`, `traction`, `mixed`의 각 실행에서 지정 랩 수 완주, 워치독 만료 0회, 최대 정체 2.0초 이하. `clean`에서는 0.2초 이상 정체 0회 |
| 스캔 도착 중단 복구 | 감지된 일시 중단마다 정상 PWM 적용까지 1.25초 이하. `scan_recovery`는 복귀 횟수와 감지 횟수, `recovery_max`는 가장 긴 복구 시간 |
| 지속 단절 | `watchdog`와 `sensor-outage`에서 예정한 고장 시작부터 속도가 0.03 m/s 미만이 되기까지 2.0초 이하, 이후 모터가 꺼지고 모델이 완전히 정지 |
| 엔코더 단절·모터 고착 | 4륜 피드백 시험에서 4초에 고장을 주입한 뒤 2.0초 이내 속도 0.03 m/s 미만, 최종 출력 0과 정지. `stop_context`에서 정지 위치·시각·사유 확인 |
| 막힌 통로 | `blocked`에서는 실제 출발 후 충돌 없이 정지하고 출력을 0으로 유지. 차체 여유 기준도 적용 |
| 속도 | 두 제어 방식을 같은 정상 조건과 시드로 실행할 때 개선안의 전체 완주 시간이 기존보다 5% 넘게 늘어나면 실패. JSON의 긴 직선과 첫 코너 구간 평균 속도로 감속 패턴 확인 |

고장 시각의 속도는 JSON의 `speed_at_fault_mps`에 남습니다. 이미 0.03 m/s 미만이거나 고장 위치까지 도달하지 못했다면 주행 중 정지 시험은 미충족입니다. 엔코더 단절 및 후륜 고착은 고장 주입 이후 해당 센서 또는 구동 이상을 이유로 정지해야 통과합니다.

`--min-clearance-m`과 `--max-recovery-s`로 가상 시험 기준을 바꿀 수 있습니다. 결과를 비교할 때는 조건, 시드, 기준을 동일하게 둡니다. Pi 명령 여유 부족은 별도 진단 지표로 남기며 실차 적용 전에는 측정한 지연과 정지거리로 다시 평가합니다.

### 구간별 평가와 경로 계획 비교

```powershell
python -X utf8 -m simulation.benchmark --scenario loop-hairpin loop-narrow --cases clean --seeds 7 23 --planners legacy clearance --sector-details
```

- `below_threshold`는 시뮬레이션에서 차체와 벽 사이의 거리가 평가 기준보다 작았던 누적 시간입니다. `--sector-details`는 각 구간의 완주 여부, 시간, 평균 속도, 최소 벽 여유, 기준 미달 시간을 표시합니다. 미완주 구간도 끝까지 기록합니다.
- `stop`은 정지 또는 충돌을 최초로 판정한 시점의 위치, 속도, 실제 조향각과 당시 확인한 전진 길이를 표시합니다. `--json`은 구간별 최소 여유 위치와 정지 원인도 내보냅니다.
- `Planner comparison`은 같은 코스·시드·고장 조건·복구 정책의 실행을 짝지어 완주 수, 벽 여유, 기준 미달 시간의 차이를 보여줍니다. 양쪽이 모두 완주했을 때 전체 시간 차이를 계산합니다. `--json`의 `planner_comparisons`에는 양쪽이 모두 통과한 구간의 차이도 포함됩니다.

`clearance`는 충돌 및 관측 확인을 통과한 후보 중에서 차체와 벽 사이의 여유가 큰 경로를 우선 평가합니다. 후보를 따라 0.08~0.50 m 진행한 차체의 앞범퍼, 옆면, 모서리와 실제 라이다 반환점 사이를 계산합니다. 0.14 m보다 가까운 지점의 평균 부족량과 최대 부족량을 점수에 반영합니다. 0.14 m는 경로 선택 목표이며, 벤치마크 합격 기준은 0.10 m입니다.

과거 단일 조향각·속도 모델의 정상 스캔에서 기존 방식은 헤어핀 시드 7에서 두 번째 코너를 통과하기 전에 정지했고, 시드 23에서 완주했지만 최대 6.12초 동안 벽 여유 기준에 미달했습니다. 당시 개선안은 두 시드 모두 10 cm 이상을 확보하며 완주했습니다. 폭 0.56 m 코너의 최소 여유는 7.3~7.4 cm에서 10.7~10.9 cm로 늘었습니다. 별도 시드 41에도 두 코스 모두 통과했습니다. `mixed`의 좁은 코너는 시드 7에서 개선안도 미완주했습니다. 기준 미달 시간은 5.21초에서 1.82초로 줄었습니다.

`scan-coast`는 스캔 도착 예정 시각에서 40 ms가 지나면 모터 출력 0을 전송하고 같은 조향을 유지합니다. 스캔이 계속 끊기면 복구 대기 동안 출력 0을 갱신합니다. 정지 여유와 명령 유효시간을 확인한 새 스캔 2개로 복귀합니다. 명령 통신이 끊기면 워치독으로 종료하며, 스캔 중단이 2초 지속되면 복구 대기를 종료합니다. 이 정책은 벤치마크와 트랙 화면 모두 기본 선택됩니다.

```powershell
python -m simulation.benchmark --policies baseline scan-coast --cases clean transport mixed --seeds 7 23 --scan-ms 250 --point-rate-hz 1500
.\simulation\track\lidar-simulator.cmd --feedback --stress mixed --scan-ms 250 --point-rate-hz 1500 --coast-on-scan-gap --audit-leases
```

비교 명령은 기존 방식의 실패도 그대로 표시하며, 어느 실행이든 기준을 충족하지 못하면 종료 코드 1을 반환합니다. 개선안만 확인하려면 `--policies scan-coast`를 사용합니다. 여러 바퀴를 돌릴 경우 고장 시각은 첫 출발부터 계산합니다.

250 ms·초당 1,500점의 `mixed`에서 시드 7의 기존 방식은 5.79초에 명령 만료로 정지했습니다. `scan-coast`는 두 차례의 스캔 도착 중단을 복구하고 120.64초에 완주했습니다. 가장 긴 복구는 0.745초였으며 최소 차체–벽 거리는 0.147 m였습니다. 같은 시드의 `transport`에서는 정체 없이 0.495초에 복귀했습니다. `clean`에서는 두 방식 모두 117.67초였습니다.

## 다변인 시험과 후륜 PI

```powershell
python -m simulation.benchmark.campaign --scenario loop-narrow --case mixed --trials 16 --seed 7 --jobs 4
```

한 시험의 같은 코스와 시드, 차체·센서 조건에서 기존 PWM 모델과 후륜 엔코더 PI 모델을 짝지어 실행합니다. 시작 위치·방향, 좌우 모터 편차, 전압 배율, 적재, 바퀴 둘레, 서보 응답, 접지, 가속·관성 감속, 센서·명령 지연, LiDAR 거리·각도 오차와 연속 결측, 엔코더 기반 방향 오차를 변동시킵니다. 기준 LiDAR는 A1 실측 회전 주기 프로필을 사용합니다. 제어기는 기준 감속과 보정값을 사용하며 시험에서 주입된 물리 오차는 관측을 통해서만 드러납니다.

범위는 `campaign.py`의 가정입니다. 모델 최고속도 0.42 m/s, 관성 감속도 0.55 m/s², LiDAR 전방 오프셋 66.62 mm는 [공용 프로필](../../hardware/control_profile.json)의 임시 기준입니다. [11월 20일 Pi USB 검증](../../docs/development-log.ko.md#2026-11-20-pi-연결-및-최신-라이다-주행-코드-미리보기-확인)은 구동 출력 0%인 계산 시험입니다. 후륜 PI에는 별도 가상 이득과 65% 명목 듀티 상한을 사용합니다. 피팅에 필요한 값은 적재 상태의 좌우 바퀴별 PWM–속도 곡선, 버스 전압, 정지거리, 조향 지연·유격, 시간 동기화한 엔코더와 외부 자세입니다.

출력 JSON은 시드, 표본별 원래 변인과 각 실행 결과를 보존합니다. 완주·충돌 수, 최소 차체 여유의 하위값과 중앙값, 완주 시간 중앙값을 함께 읽습니다. 변인별 순위 상관은 다음 실험에서 따로 확인할 변인을 고르는 데 사용합니다. 여러 변인을 함께 바꾼 결과에서 단일 변인의 인과 효과를 단정할 수 없습니다.

협로에서 `--trials 6 --seed 7 --max-seconds 95`로 비교했습니다. `clean`에서 두 모델은 같은 두 시드에서 완주했고 중앙값은 PWM 81.61초, PI 75.23초였습니다. 이 조건의 `clean`은 예약된 고장 없음으로, 21개 변인의 편차는 포함합니다. `mixed`는 모두 충돌 없이 관측 여유가 부족해 정지했습니다. 두 조건의 전체 24회에서 충돌은 없었습니다.

## 목표 기반 RL 속도 실험

```powershell
python -m simulation.benchmark.learn_speed --train-scenario loop-narrow --eval-scenario loop-s --train-seeds 7 23 41 53 --eval-seeds 101 103 --case mixed --max-seconds 100
```

목표는 지정 코스 한 바퀴입니다. 순서대로 통과한 체크포인트와 주행 진행에 보상을 주고, 여유 10 cm 미달 시간과 충돌을 감점합니다. 정책의 관측은 엔코더 속도, 직전 목표, 요청 조향, 라이다가 확인한 전진 거리와 스캔 나이입니다. 액션은 명목 상한의 55%, 75%, 100%입니다. 기존 경로, 곡률·제동 한계, 관측·통신 게이트가 계산한 더 낮은 제한을 적용합니다. 차체의 실제 위치와 충돌은 보상·평가에만 사용합니다.

평가는 학습에 쓰지 않은 시드를 기준 제어와 학습 정책에 동일하게 적용합니다. `terminated`는 완주나 실패로 끝난 경우, `truncated`는 시간 예산이 먼저 끝난 경우입니다. 최소 여유, 완주·충돌, 경과 시간, 정지 이유와 정책 행동 횟수를 함께 확인합니다. `--wheel-pi`를 주면 비교하는 양쪽에 같은 가상 PI를 사용합니다.

첫 복합 고장 시험은 협로 시드 7·23·41·53으로 학습하고 S자 시드 101·103에서 평가했습니다. 100초 안에 기준 제어는 2회 모두 완주했고, RL은 두 번 모두 7개 중 6개 체크포인트에서 시간 예산이 끝났습니다. 네 번 모두 충돌은 없었습니다.

[트랙 시뮬레이터 사용법](../track/guide.ko.md)
