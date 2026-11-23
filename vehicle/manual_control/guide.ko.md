# 차량 Wi-Fi 수동 조종

아래 키보드 구동은 기존 중앙 후륜 모드용입니다. 기본 듀얼 후륜 모드는 [엔코더 조회·속도 시험 명령](../firmware/dual-rear.ko.md#설정과-시험-순서)을 사용합니다.

라이다 자율주행과 수동조종을 한 창에서 전환하려면 [Pi의 통합 조종 모드](../../lidar/autopilot/guide.ko.md#자율주행과-수동조종-전환)를 사용합니다. 아래는 Nano에 직접 연결하는 독립 조종기의 키입니다.

## 조종기 실행

1. Nano USB를 연결하고 PC를 **poweron** Wi-Fi에 연결합니다.
2. 다른 조종기가 실행 중이면 그 창에서 `Q`로 종료합니다. 한 번에 한 조종기만 사용합니다.
3. [control-car.cmd](control-car.cmd)를 더블 클릭합니다. Python 표준 라이브러리만 사용하므로 조종기용 추가 패키지 설치는 필요 없습니다.

PowerShell에서 직접 실행할 수도 있습니다.

```powershell
Set-Location 'C:\Users\user\Arduino\Nano_33_IoT'
py -3 -m vehicle
```

자동 검색이 실패하면 현재 Nano의 IP로 직접 연결합니다.

```powershell
py -3 -m vehicle --host 192.168.50.222
```

`Connected to Nano 33 IoT`와 `READY PowerOn Nano 33 IoT`가 보이면 접속 성공입니다. 접속/종료 시 코드가 모터 정지와 서보 중앙 명령을 적용합니다.

| 키 | 동작 |
|---|---|
| R | 현재 명령 상태 조회; 처음에는 `enabled=0` 확인 |
| N | Wi-Fi / TCP / UDP 상태 조회 |
| P | 내장 LED 켜기/끄기와 `PONG` 응답 |
| 1 / G | 실제 센서 기반 자율주행 시작 요청; 펌웨어의 주행 활성화 설정과 정상 센서 입력 필요 |
| 0 / - | 자율주행 해제 + 모터 정지 + 조향 중앙; 기존 W/S/A/D 수동 제어로 복귀 |
| B | 자율주행 상태와 계산 목표 조회 |
| L | 세 SHARP 센서의 원시 ADC·전압·거리·상태 조회 |
| A / D | 중심 조향각 좌 / 우, 한 번에 2도; 좌우 바퀴 각도는 Nano가 각각 계산 |
| C | 조향 중앙; 현재 모터 명령 유지 |
| W / S | 모터 정/역방향 명목 전압 조정; 정지에서 첫 명령은 ±7 V |
| X / Space | 모터 정지 |
| I | 모터 정지 + 조향 중앙 |
| Q / Esc | 정지 + 중앙을 요청하고 종료 |

먼저 모터 전원이 분리된 상태에서 `R`, `N`, `P`로 통신을 확인합니다. **키를 놓아도 모터 명령이 유지됩니다.** 정지는 `X` 또는 Space입니다. ±7 V는 측정 전압이 아닌 PWM 계산값입니다. 모터 전원/정격/배선 확인 후 바퀴를 띄운 짧은 시험이 필요합니다. Nano 리셋이나 USB 분리 중 정지는 소프트웨어로 보장되지 않으므로 모터 전원 차단 수단을 사용합니다.

자율주행 기능이 있는 스케치에서 `X/Space/I`는 자율주행도 취소하며, 수동 구동·조향 키 역시 수동 제어로 전환합니다. 가상 입력 계산은 `python -m vehicle --preview-csv vehicle/ir_wall_following/examples/ir_wall_follow_preview.csv`로 실행합니다. `AUTO TEST`는 목표를 모터·서보에 적용하지 않지만 기존 접속/종료 시 서보 중앙 명령은 적용됩니다.

`1`은 Nano의 IR 기반 `AUTO START`를 요청하고, `0`과 `-`는 `IDLE`을 보냅니다. 두 전환에서 수동 속도·조향 기억값을 0으로 초기화합니다. 이 조종기에서 IR 자율주행은 센서 장착·보정과 펌웨어 활성화가 필요합니다. `--sensors` 전용 모니터는 센서 조회에 사용합니다.

[펌웨어 업로드](../firmware/upload.ko.md) · [차량 배선](../firmware/wiring.ko.md) · [센서 연속 표시](../sensor_monitor/guide.ko.md#powershell-연속-표시)
