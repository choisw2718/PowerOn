# NUCLEO-F103RB RC카 — 노트북 터미널 제어 가이드

## 1. 빌드

```powershell
powershell -ExecutionPolicy Bypass -File Tools\build_f103_project.ps1
```

`STM32_F103_RC_Car\Build\` 아래 세 파일이 생성된다:

| 파일 | 용도 |
|---|---|
| `STM32_F103_RC_Car.bin` (~16 KB) | **보드 업로드용** — 드래그앤드롭·CLI 플래싱 (500 KB 한도 대비 여유) |
| `STM32_F103_RC_Car.hex` | 대체 플래싱 포맷 |
| `STM32_F103_RC_Car.elf` | 디버깅용 — **보드에 직접 업로드 불가** |

## 2. 플래싱 (둘 중 하나)

**방법 A — 스크립트 (권장):** ST-LINK SWD로 굽고, CLI가 없으면 자동으로 USB 드라이브 복사로 넘어간다.

```powershell
powershell -ExecutionPolicy Bypass -File Tools\flash_f103_project.ps1
```

**방법 B — 드래그앤드롭:** 보드를 USB로 연결하면 뜨는 `NODE_F103RB` 드라이브에 **`.bin` 파일을** 복사한다. (`.elf`를 넣으면 조용히 실패하고 이전 펌웨어가 그대로 남는다 — 기존에 제어가 안 됐던 가장 유력한 원인.)

## 3. 터미널로 제어

### 방법 A — PuTTY / TeraTerm (아무 시리얼 터미널)

장치 관리자에서 `STMicroelectronics STLink Virtual COM Port`의 COM 번호 확인 → 115200 8N1로 접속.

- 입력한 글자는 펌웨어가 **에코**해 주고, 모든 명령에 `OK`/`ERR` 응답이 온다.
- **단일 키 (Enter 불필요):**

| 키 | 동작 |
|---|---|
| `w` / `s` | 속도 ±0.05 m/s |
| `a` / `d` | 조향 ±5° |
| `c` | 조향 중립 |
| `z` | 속도 0 |
| `x` 또는 Space | 모터 정지 (조향 유지) |
| `i` | IDLE: 모터 정지 + 앞바퀴 일자 정렬 |
| `h` | 도움말 |

- **라인 명령 (`@`로 시작, Enter로 전송):** `@DRIVE 0.20 10`, `@SPEED 0.15`, `@STEER -10`, `@CENTER`, `@STOP`, `@IDLE`, `@STATUS`
- `@DRIVE`는 좌·우 후륜을 모두 폐루프 제어하며, 조향각에 맞춰 안쪽/바깥쪽 바퀴 목표 속도를 자동 분배한다. `@MOTOR <좌%> <우%>`는 엔코더 PID를 우회하여 두 후륜을 독립적으로 시험한다.
- 안전 타임아웃: 명령이 1.5초간 없으면 모터가 자동 정지하고 `TIMEOUT STOP` 메시지가 출력된다. 벤치 테스트 시 `@TIMEOUT 5000`(5초)이나 `@TIMEOUT 0`(해제, 주의)으로 조절.

### 방법 B — 파이썬 키보드 컨트롤러 (주행용 권장)

```powershell
python -m pip install pyserial   # 최초 1회
python Tools\keyboard_drive.py   # COM 포트 자동 감지
```

보정 완료 후에는 W/A/S/D로 주행하고, `x`/Space로 모터 정지(조향 유지), `i`로 IDLE(모터 정지 + 앞바퀴 일자 정렬), `q`로 종료한다. 현재 RB35GM 감속비·실측 카운트·PID가 미확정이므로 키보드 도구가 `closed_loop_ready=1`을 확인하기 전까지 W/S 속도 입력을 거부한다.

## 4. 문제가 있을 때

1. `@STATUS` — 반드시 `fw=motor-v7-rb35gm-dual-20260915`, `motor=RB35GM_09TYPE_26P`가 보여야 한다. 현재 보정 전 정상 상태는 `closed_loop_ready=0`, `gear_verified=0`, `encoder_counts_verified=0`이다. `uptime_ms`가 올라가면 펌웨어는 살아 있고, `cmd_count`가 안 늘면 명령이 도달하지 않는 것이다.
2. 바퀴를 지면에서 띄운 상태에서 `@MOTOR 10 0`으로 좌측, `@MOTOR 0 10`으로 우측을 각각 저출력 시험하고 매번 `@MOTOR 0 0`으로 정지한다. 이어서 `@MOTOR 10 10`으로 양쪽 동시 출력을 확인한다. 양수 명령에서 바퀴가 차량 전진 방향의 반대로 돌면 `STM32_F103_RC_Car/Inc/rc_config.h`의 해당 `RC_LEFT_MOTOR_FORWARD_IN1_HIGH` 또는 `RC_RIGHT_MOTOR_FORWARD_IN1_HIGH` 값을 반전한 뒤 다시 빌드한다.
3. `@SERVO 1200 1800` — 서보 펄스 직접 지정. 조향 한계에 따라 좌측 948~1948 µs, 우측 952~1952 µs로 자동 제한된다.
4. `@STATUS`에서 `drive_l=1 drive_r=1 feedback_l=1 feedback_r=1`인지 확인한다. 좌측 바퀴를 손으로 돌렸을 때 `measured_l_rpm`, 우측 바퀴를 돌렸을 때 `measured_r_rpm`이 변해야 한다. 표시 RPM은 보정 전에는 52 counts/motor-rev 작업 가정값이며 실제 휠 속도가 아니다.
5. `uart_errors`·`rx_overflow`가 계속 증가하면 배선/그라운드 노이즈 점검.
6. 정지 상태의 `@STATUS`는 `l_en_pin=1 l_enabled=0 l_ccr=0` 및 오른쪽도 같은 상태여야 한다. `@MOTOR 20 20` 직후에는 각 쪽의 `in1`과 `in2`가 서로 다르고, `en_pin=0`, `enabled=1`, `ccr=80` 근처여야 한다. 이 값들이 맞는데 모터가 돌지 않으면 펌웨어 이후의 로직 전압·드라이버 전원·모터 출력 배선 문제다.

현재 배포 BIN은 `drive_l=1`, `drive_r=1`, `feedback_l=1`, `feedback_r=1`로 엔코더 원시 계수를 활성화하지만, `closed_loop_ready=0`이므로 0이 아닌 W/S/`@DRIVE`/`@SPEED` 명령을 거부한다. `@MOTOR`는 양쪽 모두 최대 10%로 제한된다.
좌측 엔코더 A/B는 PA0/PA1의 TIM2, 우측 엔코더 A/B는 PA8/PA9의 TIM1 quadrature 입력을
사용한다. PA8/PA9와 달리 **PA0/PA1은 5 V-tolerant가 아니므로 좌측 엔코더 신호는 반드시
3.3 V 이하로 변환**해야 한다. 출력이 open-collector라면 각 입력 전압에 맞는 풀업을 사용한다.

## 5. 핀맵 요약

| 기능 | STM32F103RB 핀 / 타이머 |
|---|---|
| UART (ST-LINK VCP) | PA2 / PA3 |
| 좌측 서보 | D15 / PB8 |
| 우측 서보 | D14 / PB9 |
| 좌측 후륜 모터 PWM | PA6 / TIM3_CH1 |
| 우측 후륜 모터 PWM | PA7 / TIM3_CH2 |
| 좌측 후륜 모터 IN1 / IN2 / EN | PB0 / PB12 / PB1 |
| 우측 후륜 모터 IN1 / IN2 / EN | PB10 / PB13 / PB11 |
| 좌측 엔코더 A / B | PA0 / PA1, TIM2 encoder mode |
| 우측 엔코더 A / B | PA8 / PA9, TIM1 encoder mode |

모든 컨트롤러, 모터 드라이버, 엔코더 및 서보 전원은 전기적으로 적절한 지점에서 공통 기준 GND를 사용한다. PA0/PA1에는 5 V 신호를 직접 입력하지 않는다.

## 6. 교체 구동 모터 — RB-35GM 09TYPE

### 6.1 모터 사양

| 항목 | 사양 |
|---|---|
| 모델 | **RB-35GM 09TYPE DC 24V W/EC 26P** |
| 제조사 제품군 | D&J WITH RB-35GM + Encoder |
| 형식 | 브러시드 DC 헬리컬 기어드 모터 |
| 정격 전압 | **DC 24 V** |
| 감속 전 모터 속도 | **6,000 rpm** |
| 모터 출력 | **12.7 W** |
| 제공 감속비 범위 | 약 **1/10~1/3000** |

설치된 모터의 실제 감속비는 모터 라벨의 `1/xx` 표기를 직접 확인해야 한다. 정격 출력축 속도와 토크는 선택된 감속비에 따라 달라지므로, 라벨 확인 전에는 펌웨어 상수를 확정하지 않는다.

### 6.2 엔코더 사양과 임시 계수 가정

- `W/EC`는 엔코더 장착형이다.
- 엔코더는 회전 방향을 판별할 수 있는 2채널 A/B quadrature 방식이다.
- 제조사 표기는 **26 Pulses = 13 Pulses × 2 Channels**이다.
- 필요한 신호는 Encoder VCC, Encoder GND, Encoder A, Encoder B이다.
- `26P` 표기만으로 STM32의 최종 회전당 카운트를 확정하지 않는다.

각 채널이 모터축 1회전당 13주기를 만들고 STM32 타이머 encoder mode가 quadrature의 네 에지를 모두 센다고 가정하면 다음과 같다.

```text
13 × 4 = 52 counts / motor-shaft revolution
expected output-shaft counts/rev ≈ 52 × G
```

여기서 `G`는 실제 감속비다. 이 값은 계산을 위한 **임시 가정**일 뿐이며, 설치 후 모터축 또는 출력축을 알려진 회전수만큼 돌려 실제 타이머 카운트를 측정해야 한다.

### 6.3 6가닥 실제 배선 색상

실제 모터/엔코더 어셈블리의 배선 색상은 아래 여섯 가지이며, 문서와 소스 코드 주석에서 이름을 변경하거나 생략하거나 다른 색으로 대체하지 않는다.

| 순서 | 실제 배선 색상 | 기능 |
|---:|---|---|
| 1 | **보라색 (Purple)** | **TBD — 실물 측정 또는 데이터시트로 확인** |
| 2 | **파란색 (Blue)** | **TBD — 실물 측정 또는 데이터시트로 확인** |
| 3 | **민트색 (Mint)** | **TBD — 실물 측정 또는 데이터시트로 확인** |
| 4 | **갈색 (Brown)** | **TBD — 실물 측정 또는 데이터시트로 확인** |
| 5 | **빨간색 (Red)** | **TBD — 실물 측정 또는 데이터시트로 확인** |
| 6 | **검은색 (Black)** | **TBD — 실물 측정 또는 데이터시트로 확인** |

예상되는 전체 기능은 Motor terminal 1, Motor terminal 2, Encoder VCC, Encoder GND, Encoder A, Encoder B이다. 신뢰할 수 있는 RB35GM 09TYPE 제조사 배선표나 실측으로 확인하기 전에는 색상별 기능을 추측하지 않는다. 미확인 배선을 24 V 또는 STM32 GPIO에 직접 연결하지 않는다.

### 6.4 전기적 제약

- 모터 전원은 24 V이며, 적합한 H-bridge 또는 모터 드라이버를 통해서만 구동한다.
- 모터를 STM32 GPIO에서 직접 구동하거나 전원을 공급하지 않는다.
- STM32F103RB 로직 전압은 3.3 V이다.
- 엔코더를 연결하기 전에 공급 전압과 출력 방식이 3.3 V push-pull, 5 V push-pull, open-collector/open-drain 중 무엇인지 확인한다.
- 엔코더 출력 HIGH 전압이 STM32 입력에 안전한지 확인하고, 필요하면 레벨 시프터나 적절한 풀업을 추가한다.
- 컨트롤러, 모터 드라이버, 엔코더 및 서보의 GND는 전기적으로 적절한 공통 기준을 사용한다.

### 6.5 펌웨어 확정 전 필수 확인

1. 모터 라벨에서 정확한 감속비 `1/xx`를 읽는다.
2. 여섯 가닥 중 모터 단자 두 가닥을 식별한다.
3. 데이터시트 또는 측정으로 Encoder VCC / GND / A / B를 식별한다.
4. 엔코더 공급 전압을 확인한다.
5. 엔코더 출력 HIGH 전압이 STM32 입력에 안전한지 확인한다.
6. 모터축 또는 출력축을 알려진 회전수만큼 돌려 실제 타이머 카운트를 측정한다.
7. 차량 전진 방향을 기준으로 A/B 순서와 카운트 부호를 확인한다.

확인이 끝날 때까지 모든 미검증 배선 지정은 `TBD`로 유지한다.

### 6.6 적용된 펌웨어 상태와 추후 확정 항목

기존 DS4572 W/EC 38P + IG42 구성의 감속비 `4.0`, `38 P/R × quadrature x4 = 152 counts/motor-rev` 값은 제거했다. 현재 `Config/drive_hardware.h`에는 양쪽 RB35GM에 대한 13 periods/channel, quadrature x4, 52 counts/motor-rev 작업 가정과 다음 보정 잠금이 적용되어 있다.

- `DRIVE_GEAR_RATIO=1.0f`: 코드 계산을 위한 비작동 자리표시자. 실제 라벨값으로 교체
- `DRIVE_GEAR_RATIO_VERIFIED=0`: 라벨 확인 후 1로 변경
- `DRIVE_ENCODER_COUNTS_VERIFIED=0`: 좌우 실측 완료 후 1로 변경
- `DRIVE_SPEED_PID_TUNED=0`: 새 모터 PID 조정 후 1로 변경
- `DRIVE_ENCODER_TIMER_EDGE_MULTIPLIER=4.0f`: 실측 결과에 따라 조정
- Arduino `POWERON_PID_KP/KI/KD`와 STM32 `RC_PID_KP/KI/KD`: 현재 0, 실측 조정 필요

세 검증 플래그가 모두 1이 되어 `DRIVE_CLOSED_LOOP_READY=1`이 되기 전에는 속도 기반 주행을 활성화하지 않는다.
