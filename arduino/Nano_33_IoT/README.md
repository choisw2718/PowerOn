# Arduino Nano 33 IoT 단일 보드 제어

목표는 **Nano 33 IoT 1개**가 Wi-Fi 명령을 직접 받아 기존 **MAI-2MT-DC V3.0** 드라이버의 채널 A로 **중앙 후륜 모터 1개**를 제어하고, 기존 **Hitec HS-311 앞바퀴 조향 서보 2개**를 구동하는 것입니다. 드라이버 채널 B에는 모터를 연결하지 않습니다.

**Nano용 펌웨어와 핀맵을 작성했습니다. 컴파일·실차 검증은 아직 하지 않았습니다.** USB 시리얼 기반 Uno 버전은 [Uno 안내](../Uno_Single_Rear_Motor/README.md)에 보존했습니다. 아래의 [배선 전 준비와 핀별 배선](WIRING.ko.md)을 먼저 읽으세요.

## 파일과 실행

| 파일 | 용도 |
|---|---|
| [Nano_33_IoT.ino](Nano_33_IoT.ino) | 내장 Wi-Fi AP/TCP 서버와 모터·서보 제어 |
| [WIRING.ko.md](WIRING.ko.md) | 배선 전 준비, D2/D3/D4/D5/D6/D9 핀별 배선, 전원, 최초 시험 |
| [wifi_secrets.example.h](wifi_secrets.example.h) | 개인 Wi-Fi 비밀번호 설정용 예시 |
| [wifi_control.py](wifi_control.py) | Python 표준 라이브러리만 사용하는 노트북 키보드 조종기 |

1. Arduino IDE에 **Arduino SAMD Boards**, **WiFiNINA**, **Servo**를 설치하고, **Arduino Nano 33 IoT**를 선택합니다.
2. `wifi_secrets.example.h`를 같은 폴더의 `wifi_secrets.h`로 복사해 SSID와 8자 이상 개인 비밀번호를 설정한 뒤 스케치를 업로드합니다. 비밀번호 파일은 Git에서 제외됩니다.
3. USB 시리얼 모니터 115200 baud를 열고 Nano의 RESET 버튼을 눌러 `READY AP=... IP=192.168.4.1 PORT=5000`을 확인합니다. 노트북을 해당 Wi-Fi에 연결하고 `python arduino\Nano_33_IoT\wifi_control.py`를 실행합니다.
4. `W/S`는 명목 모터 전압, `A/D`는 조향, `X`/Space는 모터 정지, `C`는 조향 중앙, `I`는 정지와 중앙, `R`은 상태, `Q`/Esc는 종료입니다. TCP 클라이언트는 `192.168.4.1:5000`에 ASCII 명령과 줄바꿈을 보낼 수도 있습니다. 한 번에 조종기 한 대만 받습니다.

`STATUS`, `STEER 5`, `CENTER`, `DRIVE 7 0`, `DRIVE -7 0`, `STOP`, `IDLE`, `MOTOR 20`, `VOLTAGE 6`, `KEEPALIVE`, `TIMEOUT 1500`을 지원합니다. `DRIVE`만 기존 7 V 기동 보정을 적용합니다. `VOLTAGE`는 실측 전압이 아닌 PWM 명목값입니다. 모터 감시는 기본 1.5초이며 설정 범위는 200~5000 ms입니다. 방향을 바꿀 때 50 ms 동안 출력을 끕니다. TCP 연결이 끊어지면 모터가 정지하고 조향은 중앙으로 갑니다.

## 전기적 확인

- Nano GPIO는 3.3 V 로직입니다. [배선 문서](WIRING.ko.md)는 5 V 74AHCT245 버퍼와 별도 트랜지스터를 사용합니다. 드라이버 입력 사양과 실제 전압은 확인해야 합니다. 다른 장치의 5 V 출력을 Nano GPIO에 넣지 않습니다.
- 후륜 모터의 **정확한 모델·정격은 현 단일 모터 문서에 없습니다**. 이전 양쪽 후륜의 24 V RB-35GM 기록을 이 모터에 적용하지 않습니다. 기존 Uno의 12 V 전원 설정과 약 7 V 기동 보정은 실물 확인 후 다시 정합니다.
- 모터·드라이버 로직·서보 전원은 각각 해당 정격에 맞게 외부에서 공급합니다. 서보 2개는 4.8~6.0 V의 충분한 전류 여유가 있는 전원을 사용합니다. 모든 GND는 공통으로 연결하고 모터 전류가 Nano 신호 접지선을 통해 흐르지 않게 합니다.
- `VOLTAGE` 값은 PWM 비율에 대한 명목 값입니다. 엔코더와 전압 피드백이 없으므로 실제 속도나 모터 단자 전압을 일정하게 유지하지 않습니다.

Nano D9의 `analogWrite()` PWM을 사용하므로 Uno 타이머의 20 kHz 설정을 그대로 쓰지 않습니다. 드라이버의 허용 PWM 주파수를 실물 사양과 시험으로 확인하세요. 코드의 핀 번호와 배선 예시는 [핀별 표](WIRING.ko.md#2-핀-번호-한눈에-보기)에 있습니다.

## 완료 기준

- [ ] 실제 Nano에서 Wi-Fi 명령 수신과 상태 응답
- [ ] 바퀴를 띄운 상태에서 후륜 모터 정·역회전/정지와 채널 B 비사용 확인
- [ ] 서보 2개의 중앙, 작은 좌·우 조향 및 기구 한계 확인
- [ ] 연결 해제, 명령 시간 초과, 재부팅 시 모터 정지 확인
- [ ] 3.3 V 신호 호환성, 전원 용량, 공통 GND, 드라이버 발열 확인
