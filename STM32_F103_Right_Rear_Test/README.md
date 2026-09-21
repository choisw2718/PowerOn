# NUCLEO-F103RB 오른쪽 뒷바퀴 단독 테스트

이 폴더는 교체된 **RB-35GM 09TYPE DC 24V W/EC 26P 오른쪽 뒷바퀴 모터와 엔코더만** 짧게 시험하기 위한 독립 펌웨어입니다. 일반 주행 펌웨어와 분리되어 있으며, 부팅만으로 모터가 돌지 않습니다.

## 테스트 펌웨어의 제한

- 오른쪽 모터: `PA7/TIM3_CH2` PWM, `PB10` IN1, `PB13` IN2, `PB11` active-low ENABLE
- 오른쪽 엔코더: `PA8/D7` A, `PA9/D8` B, TIM1
- 왼쪽 모터: `PB1` ENABLE을 HIGH(비활성)로 고정하고, 왼쪽 PWM `PA6` 및 방향 `PB0/PB12`는 출력으로 설정하지 않음
- 조향 서보: `TIM4`와 서보 GPIO를 초기화하지 않음
- 최대 듀티: 10%
- 한 번의 구동 시간: 1초 후 자동 정지
- 모든 구동 전: PWM/ENABLE 정지 상태로 100 ms 대기
- 리셋 또는 부팅 상태: 정지

`test_config.h`는 공용 `rc_config.h`의 우측 핀맵이 현재 배선과 달라진 경우 빌드를 실패시킵니다. 일반 주행 설정에서 왼쪽 모터가 활성화되어 있어도 이 독립 펌웨어는 왼쪽 ENABLE을 별도로 잠급니다.

## 안전 준비

1. **차량을 받침대에 올려 모든 구동 바퀴가 바닥에서 떨어진 상태로 고정합니다.** 사람이 바퀴, 축, 기어에 닿지 않게 합니다.
2. 처음에는 24 V 모터 전원을 끈 상태에서 펌웨어 업로드와 시리얼 연결부터 확인합니다.
3. 모터 드라이버 로직 전원, 엔코더 전원, NUCLEO의 GND를 공통으로 연결합니다. 24 V를 MCU 또는 엔코더 신호선에 연결하면 안 됩니다.
4. 즉시 끌 수 있는 모터 전원 차단 수단을 준비합니다. 예상하지 못한 움직임이 있으면 키보드 명령보다 전원을 먼저 차단합니다.
5. 실제 여섯 선인 **보라색 (Purple), 파란색 (Blue), 민트색 (Mint), 갈색 (Brown), 빨간색 (Red), 검은색 (Black)**의 기능은 모두 `TBD`이다. Motor terminal 1/2와 Encoder VCC/GND/A/B를 데이터시트 또는 측정으로 확인하기 전에는 연결하지 않습니다.
6. `1`(10%)을 선택하고 `f`로 짧게 시험합니다. 이 보정 전 펌웨어에서는 출력을 더 높일 수 없습니다.

## 빌드와 정적 검증

저장소 최상위 폴더에서 실행합니다.

```powershell
powershell -ExecutionPolicy Bypass -File .\STM32_F103_Right_Rear_Test\verify.ps1
```

이 명령은 새로 빌드한 뒤 BIN 안에 다음 안전 표식이 포함되어 있는지 확인합니다.

- 왼쪽 구동 잠금
- 서보 비활성
- 1초 자동 정지
- 최대 40% 제한
- 부팅 시 정지

생성 파일은 다음과 같습니다.

```text
STM32_F103_Right_Rear_Test/Build/STM32_F103_Right_Rear_Test.bin
STM32_F103_Right_Rear_Test/Build/STM32_F103_Right_Rear_Test.hex
STM32_F103_Right_Rear_Test/Build/STM32_F103_Right_Rear_Test.elf
```

## 업로드

다음 스크립트는 항상 재빌드와 검증을 먼저 수행한 후 ST-LINK로 업로드합니다.

```powershell
powershell -ExecutionPolicy Bypass -File .\STM32_F103_Right_Rear_Test\flash.ps1
```

ST-LINK CLI를 찾지 못하면 연결된 `NODE_F103RB` USB 드라이브로 BIN을 복사합니다. 직접 업로드할 때는 ELF가 아니라 위의 `.bin` 파일을 사용합니다.

## 시리얼 테스트

1. `STMicroelectronics STLink Virtual COM Port`를 115200 baud, 8N1로 엽니다.
2. 부팅 배너에서 아래 문구를 확인합니다.

```text
RIGHT_REAR_WHEEL_TEST right-rear-rb35gm-test-v2-20260915
LEFT_DRIVE=LOCKED_OUT; SERVO=NOT_INITIALIZED
AUTO_STOP_MS=1000; MAX_DUTY=10%; BOOT_STATE=STOP
MOTOR=RB35GM_09TYPE_24V_26P; COUNTS_PER_MOTOR_REV=52_ASSUMED; GEAR_RATIO=TBD
```

3. 다음 단일 키 명령을 사용합니다. Enter는 필요 없습니다.

| 키 | 동작 |
|---|---|
| `1` | 듀티 10% 선택. 선택만으로는 구동하지 않음 |
| `f` | 선택한 듀티로 정방향 1초 펄스 |
| `b` | 선택한 듀티로 역방향 1초 펄스 |
| `x` 또는 Space | 즉시 정지 및 예약된 구동 취소 |
| `r` | 오른쪽 엔코더 누적값 리셋 |
| `h` 또는 `?` | 도움말 |

250 ms마다 다음과 비슷한 상태가 출력됩니다.

```text
ms=2250 state=FWD duty=10% total=42 net=18 motor_rpm_assumed=83.1 wheel_rpm=TBD A=1 B=0 Achg=9 Bchg=9
```

- `state`: `STOP`, 안전 대기 `WAIT`, 정방향 `FWD`, 역방향 `REV`
- `duty`: 실제로 인가 중인 PWM 듀티. 정지/대기 중에는 0%
- `total`: 부팅 또는 `r` 이후 오른쪽 엔코더 누적 카운트
- `net`: 최근 250 ms의 방향 카운트
- `motor_rpm_assumed`: `13 periods/channel × quadrature x4 = 52 counts/motor-rev` 작업 가정값
- `wheel_rpm`: 실제 감속비 미확인으로 `TBD`
- `A`, `B`, `Achg`, `Bchg`: 엔코더 두 채널의 현재 상태와 최근 변화 횟수

정방향 명령에서 실제 차량 전진 방향과 반대로 돌면 공용 설정 `STM32_F103_RC_Car/Inc/rc_config.h`의 `RC_RIGHT_MOTOR_FORWARD_IN1_HIGH`를 변경한 뒤 다시 빌드합니다. 엔코더 RPM이 실제값의 2배 또는 4배 차이나면 `Config/drive_hardware.h`의 엔코더 edge 배율과 실측 1회전 카운트를 함께 확인합니다.
