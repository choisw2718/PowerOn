# ESP8266 ESP-01 Wi-Fi 모듈 하드웨어 정보

이 문서는 PowerOn 차량에 Wi-Fi 통신 기능을 추가하기 위해 보유 중인 8핀 모듈의 식별 정보, 예상 사양, 전기적 주의사항 및 추후 확인 항목을 정리한다.

현재 모듈은 **ESP8266 ESP-01 V3.0 계열로 추정**하여 취급한다. 정확한 모델과 핀 방향은 실물 PCB 및 칩 표기를 확인한 뒤 확정해야 한다.

## 1. 식별 정보와 판단 상태

실물에서 확인한 표기는 다음과 같다.

```text
MD 8X1642
MD25D80SIG
P13468
```

| 항목 | 현재 판단 | 상태 |
|---|---|---|
| 모듈 제품군 | ESP8266 ESP-01 V3.0 계열 | 추정 |
| 외부 핀 수 | 8핀 | 확인됨 |
| Wi-Fi SoC | ESP8266 | 추정, 칩 표기 확인 필요 |
| 외장 Flash | MD25D80SIG | 실물 표기 확인됨 |
| Flash 용량 | 8 Mbit / 1 MB | 부품 번호 기준 |

`MD25D80SIG`는 Wi-Fi 모듈의 모델명이 아니라 ESP8266이 사용하는 외장 SPI NOR Flash의 부품 번호로 본다.

## 2. 예상 기본 사양

| 항목 | 예상 사양 |
|---|---|
| Wi-Fi SoC | ESP8266 |
| 모듈 제품군 | ESP-01 |
| Wi-Fi 대역 | 2.4 GHz |
| Wi-Fi 표준 | IEEE 802.11 b/g/n |
| 로직 전압 | 3.3 V |
| Arduino 통신 인터페이스 | UART 사용 가능 |
| 외부 핀 | 8개 |

위 사양은 ESP-01 계열이라는 현재 가정을 바탕으로 하며, 실물 확인 결과가 다르면 이 문서와 배선 계획을 함께 수정한다.

## 3. 외장 Flash 메모리

| 항목 | 사양 |
|---|---|
| 부품 번호 | MD25D80SIG |
| 종류 | SPI NOR Flash |
| 밀도 | 8 Mbit |
| 용량 | 1 MB |
| 동작 전압 | 약 3.3 V |
| 인터페이스 | SPI |

이 Flash는 ESP8266의 펌웨어와 관련 데이터를 저장하는 외장 메모리다.

## 4. ESP-01 계열의 일반적인 8핀 신호

Ai-Thinker ESP-01S 제조사 문서의 핀 번호를 기준으로 한다.

| 핀 번호 | 신호 | 역할 |
|---:|---|---|
| 1 | GND | Ground |
| 2 | GPIO2 | 범용 입출력 / 부팅 모드 설정 |
| 3 | GPIO0 | 범용 입출력 / 부팅 모드 설정 |
| 4 | RX / GPIO3 | UART 수신 |
| 5 | TX / GPIO1 | UART 송신 |
| 6 | EN / CH_PD | 칩 활성화 |
| 7 | RST | 리셋 |
| 8 | VCC | 3.3 V 전원 |

> 주의: 표의 신호 구성만으로 실물의 핀 위치를 판단하지 않는다. 전원을 연결하기 전에 PCB 실크스크린, 안테나 방향 및 커넥터를 바라보는 방향을 기준으로 정확한 핀 배열을 다시 확인한다.

## 5. 전압 및 전원 주의사항

ESP8266 계열은 **3.3 V 장치**이고 Arduino Uno R3는 **5 V 로직 장치**다.

- ESP VCC에 5 V를 직접 연결하지 않는다.
- Arduino TX의 5 V 신호를 ESP RX에 직접 입력하지 않는다.
- Arduino TX와 ESP RX 사이에 적절한 5 V → 3.3 V 레벨 변환을 사용한다.
- ESP TX의 3.3 V 신호는 Arduino RX로 전달하는 구성을 고려할 수 있지만, 실제 입력 임계값과 배선 상태를 확인한다.
- Arduino와 ESP의 GND는 공통으로 연결한다.
- Wi-Fi 송신 시의 순간 전류를 감당할 수 있는 안정적인 3.3 V 전원을 사용한다.
- 전원 가까이에 적절한 디커플링 커패시터를 배치하고, 모터 전원 노이즈가 통신 모듈 전원으로 유입되지 않도록 한다.

기본 UART 신호 관계는 다음과 같다.

```text
ESP TX  ──────────────────────→ Arduino RX

Arduino TX
    │
    ▼
5 V → 3.3 V 레벨 변환
    │
    └─────────────────────────→ ESP RX
```

## 6. 프로젝트의 예상 통신 구조

```text
Laptop / Smartphone
        │
        │ 2.4 GHz Wi-Fi
        ▼
ESP8266 ESP-01
        │
        │ UART
        ▼
Arduino Uno R3
```

ESP8266은 `PowerOn-Car` AP와 TCP 서버를 실행하고, 네트워크에서 받은 줄 단위 제어 명령을 UART로 Arduino에 전달한다. 모터·조향 출력과 최종 안전 watchdog은 Arduino가 담당한다.

## 7. Arduino 연결에 필요한 기본 신호

```text
ESP VCC  → 안정적인 3.3 V 전원
ESP GND  → Arduino와 공통 GND

ESP TX   → Arduino RX
ESP RX   ← Arduino TX (5 V → 3.3 V 레벨 변환 경유)

ESP EN   → HIGH / 3.3 V
```

`RST`, `GPIO0`, `GPIO2`의 연결은 정상 부팅, 펌웨어 다운로드 모드 및 현재 설치된 펌웨어에 맞춰 결정한다.

Uno의 `D0/D1`은 USB 업로드와 디버깅용으로 유지한다. ESP UART는 Uno의 A0/A1을 디지털 핀으로 사용한 38400 baud `SoftwareSerial`에 연결한다.

```text
ESP TX / 5번 → Uno A0 / SoftwareSerial RX
ESP RX / 4번 ← Uno A1 / SoftwareSerial TX (레벨 변환 경유)
```

## 8. 펌웨어 구성

ESP8266 내부에서는 별도의 펌웨어가 동작한다. 이 프로젝트에서는 AT firmware 대신 직접 작성한 다음 펌웨어를 사용한다.

```text
Uno_Single_Rear_Motor/WiFi_ESP01/WiFi_ESP01.ino
```

이 펌웨어를 ESP-01에 직접 업로드하면 기존 AT firmware는 덮어써진다. 프로젝트 펌웨어의 구성은 다음과 같다.

- `PowerOn-Car` Wi-Fi AP 생성
- 고정 주소 `192.168.4.1`
- TCP 포트 `5000`에서 조종기 1개 연결
- Uno와 `38400 baud` UART 통신
- TCP 연결 및 해제 시 `IDLE` 명령으로 안전 상태 요청
- TCP 명령과 Uno 응답의 양방향 전달

펌웨어를 업로드하기 전에는 다음 기존 상태를 확인하는 것이 좋다.

- AT firmware 설치 여부 및 버전
- 현재 UART baud rate
- 기존 펌웨어의 동작 상태
- Flash mode 및 메모리 설정

현재 확인된 1 MB Flash 가정을 기준으로 `Generic ESP8266 Module`, `1MB (FS:64KB OTA:~470KB)`, `DOUT` 설정을 사용한다.

## 9. 연결 전 점검 절차

1. 모듈 PCB의 앞면과 뒷면을 촬영하고 안테나 및 8핀 헤더 방향을 기록한다.
2. ESP8266 본체의 정확한 칩 표기와 PCB 실크스크린을 확인한다.
3. 멀티미터와 신뢰할 수 있는 핀맵을 이용해 VCC와 GND 위치를 확인한다.
4. TX, RX, EN, RST, GPIO0 및 GPIO2의 실제 위치를 확인한다.
5. 전류 여유가 충분한 3.3 V 전원과 레벨 변환 회로를 준비한다.
6. 모터 구동부를 정지시킨 상태에서 ESP의 정상 부팅 여부를 확인한다.
7. UART 출력으로 부팅 로그와 현재 baud rate를 확인한다.
8. 필요하면 기존 AT firmware 상태를 기록한 뒤 프로젝트 ESP 펌웨어를 업로드한다.
9. `PowerOn-Car` Wi-Fi 검색, 연결 및 TCP 통신 가능 여부를 시험한다.
10. Arduino 통합 후 장시간 동작 시 전원 전압 강하, 재부팅 및 발열 여부를 확인한다.

## 10. 현재 정보 요약

```text
Module:            ESP8266 ESP-01 V3.0 계열로 추정
External pins:     8
Wi-Fi:             2.4 GHz, IEEE 802.11 b/g/n
Logic voltage:     3.3 V
Flash:             MD25D80SIG, 8 Mbit / 1 MB SPI NOR
Vehicle MCU:       Arduino Uno R3
Project link:      UART 38400 baud on Uno A0/A1
Project firmware:  WiFi_ESP01.ino 구현 및 컴파일 완료, 실물 업로드 필요
Pin orientation:   미확인
```

## 11. 추후 확인할 항목

- [ ] 모듈 PCB 앞면·뒷면 외형 기록
- [ ] 정확한 ESP8266 칩 표기 확인
- [ ] 8핀 실제 방향과 번호 확인
- [ ] VCC / GND 위치 확인
- [ ] TX / RX 위치 확인
- [ ] EN / RST / GPIO0 / GPIO2 연결 조건 확인
- [ ] 필요한 경우 기존 펌웨어 및 버전 기록
- [ ] 프로젝트 ESP 펌웨어 실물 업로드
- [ ] 정상 부팅 여부 확인
- [ ] Wi-Fi 연결 및 통신 시험
- [ ] 안정적인 3.3 V 전원의 요구 전류와 순간 전류 대응 확인
- [x] Uno USB 시리얼과 ESP UART의 공존 방식 결정 — D0/D1 USB, A0/A1 ESP
