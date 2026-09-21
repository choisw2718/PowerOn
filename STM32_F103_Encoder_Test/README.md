# NUCLEO-F103RB 엔코더 단독 연결 테스트

이 펌웨어는 모터 전원과 모터 드라이버 없이 교체된 **RB-35GM 09TYPE W/EC 26P** 우측 엔코더를 먼저 확인하기 위한 코드입니다.

코드에서 사용하는 것은 다음 주변장치뿐입니다.

- 오른쪽 엔코더 MCU 입력: `PA8/D7`(A), `PA9/D8`(B), TIM1. 실제 배선 색상은 TBD
- 왼쪽 엔코더와 TIM2: 비활성화
- PC 시리얼: ST-LINK Virtual COM Port, USART2, 115200 8N1

`GPIOB`, 모터 PWM용 `TIM3`, 서보용 `TIM4`는 클록조차 켜지 않으므로 이 펌웨어가 모터나 서보 구동 신호를 만들지 않습니다.

## 안전한 배선

1. 차량의 **24 V 모터 전원은 연결하지 않습니다.** 모터 드라이버도 테스트에 필요하지 않습니다.
2. NUCLEO 보드는 ST-LINK USB로 전원을 공급합니다.
3. 엔코더 자체는 신호를 만들기 위한 정격 전원이 필요합니다. 엔코더 데이터시트에 맞는 전원을 공급하고 NUCLEO와 **GND를 공통으로 연결**합니다. 엔코더 전원까지 모두 끊으면 테스트할 수 없습니다.
4. 현재 사용하는 `PA8/PA9`는 STM32F103RB의 5 V-tolerant 입력이므로 0~5 V A/B 신호를 받을 수 있습니다. 5 V를 넘는 신호나 24 V는 절대 연결하지 않습니다.
5. 엔코더 A/B가 open-collector이고 내부 풀업이 없다면 외부 pull-up이 필요합니다. 출력 방식이 불명확하면 오실로스코프나 데이터시트로 확인합니다.

| 실제 선 색상 | 역할 | NUCLEO 연결 |
|---|---|---|
| **보라색 (Purple)** | **TBD** | 확인 전 연결 금지 |
| **파란색 (Blue)** | **TBD** | 확인 전 연결 금지 |
| **민트색 (Mint)** | **TBD** | 확인 전 연결 금지 |
| **갈색 (Brown)** | **TBD** | 확인 전 연결 금지 |
| **빨간색 (Red)** | **TBD** | 확인 전 연결 금지 |
| **검은색 (Black)** | **TBD** | 확인 전 연결 금지 |

여섯 선의 예상 기능은 Motor terminal 1/2와 Encoder VCC/GND/A/B이다. 데이터시트 또는 측정으로 기능과 엔코더 전압을 확인한 뒤 A/B만 PA8/PA9에 연결한다.

## 빌드와 업로드

저장소 최상위 폴더에서 실행합니다.

```powershell
powershell -ExecutionPolicy Bypass -File .\STM32_F103_Encoder_Test\build.ps1
```

생성 파일은 `STM32_F103_Encoder_Test/Build/STM32_F103_Encoder_Test.bin`입니다. 자동 빌드 후 ST-LINK로 업로드하려면 다음을 실행합니다.

```powershell
powershell -ExecutionPolicy Bypass -File .\STM32_F103_Encoder_Test\flash.ps1
```

또는 `.bin` 파일을 NUCLEO의 `NODE_F103RB` USB 드라이브에 복사합니다.

## 확인 방법

1. PC에서 `STMicroelectronics STLink Virtual COM Port`를 115200 baud, 8N1로 엽니다.
2. 모터 전원이 없는 상태에서 우측 바퀴 또는 모터 축을 천천히 손으로 돌립니다.
3. 250 ms마다 다음 형식의 상태가 출력됩니다.

```text
ms=1250 | R total=48 net=12 move=12 A=1 B=0 Achg=6 Bchg=6
```

- `total`: 부팅 또는 리셋 이후 누적 방향 카운트
- `net`: 최근 250 ms의 방향 카운트
- `move`: 최근 250 ms에 감지한 전체 움직임. 정상적으로 한 방향 회전할 때는 대체로 `abs(net)`과 비슷합니다.
- `A`, `B`: 현재 A/B 입력 논리값
- `Achg`, `Bchg`: 최근 250 ms 동안 관찰한 각 입력 변화 횟수
- `r`: 우측 누적값을 0으로 리셋
- `h` 또는 `?`: 간단한 도움말 출력

정상 배선이면 축을 한 방향으로 돌릴 때 `total`이 한 방향으로 계속 변하고 `Achg`와 `Bchg`가 모두 증가합니다. 회전 방향에 따른 부호는 A/B 배선 순서에 따라 반대여도 괜찮습니다.

제조사 표기는 `26 Pulses = 13 Pulses × 2 Channels`이다. 채널당 13주기와 quadrature x4를 가정하면 모터축 1회전당 약 52 count, 실제 감속비가 `G`이면 출력축 1회전당 약 `52 × G` count가 예상된다. 이는 작업 가정일 뿐이며 알려진 회전수로 반드시 실측한다.

`total`이 변하지 않으면 엔코더 전원, 공통 GND, 레벨 변환, 핀 순서를 확인합니다. `Achg` 또는 `Bchg` 한쪽만 계속 0이면 해당 채널 배선을 우선 확인합니다. 손을 대지 않았는데 값이 계속 변하면 입력이 떠 있거나 GND/레벨 변환이 불안정할 가능성이 큽니다.
