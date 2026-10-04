# Arduino Nano 33 IoT 단일 보드 제어

문서 기준일: **2026-09-30 (한국 시간)**. 현재 메인 작업 폴더는 **`C:\Users\user\Arduino\Nano_33_IoT`**입니다. 이 폴더는 필요한 코드·문서·개인 Wi-Fi 설정을 담은 독립 폴더입니다. 개발 현황과 추가로 필요한 실물 정보는 [개발 현황 문서](DEVELOPMENT.md)에 모았습니다.

목표는 **Nano 33 IoT 1개**가 Wi-Fi 명령을 직접 받아 기존 **MAI-2MT-DC V3.0** 드라이버의 **R 채널로 중앙 후륜 모터 1개**를 제어하고, 기존 **Hitec HS-311 앞바퀴 조향 서보 2개**를 구동하는 것입니다. 드라이버 L 채널에는 모터를 연결하지 않습니다. Nano D9/D2/D4/D3를 각각 드라이버 **6번 R_PWM / 4번 R_IN1 / 7번 R_IN2 / 8번 R_ENABLE**에 직접 연결합니다. D3는 **HIGH=정지, LOW=출력 허용**입니다. 기존 L 배선 그대로는 R 출력이 동작하지 않으며, 실차의 R 채널 연결·모터 정격·3.3 V 직접 신호 호환성은 미확인입니다. **리셋·USB 전원 손실 중 모터 정지는 보장되지 않습니다.** 드라이버의 **10핀 번호**는 [배선 문서](WIRING.ko.md#2-1-모터-드라이버의-실제-10핀-box-커넥터-번호)에 있습니다.

**2026-09-30에 R 채널 펌웨어를 Arduino SAMD 1.8.14로 컴파일해 Nano 33 IoT에 업로드했습니다.** 실제 Nano에서 UDP 자동 검색, TCP `STATUS`·`NETWORK`·`PING`·`IDLE` 왕복, 2초 이상 명령이 없어도 소프트웨어 출력 상태가 유지되는 것과 `STOP` 후 정지를 확인했습니다. 모터를 연결하지 않은 시험이므로 **R 출력 단자 전압·모터 회전·서보의 실제 동작은 아직 검증하지 않았습니다.** 기존 Uno 코드와 사용법은 [이 폴더의 Uno 참고 자료](reference/Uno_Single_Rear_Motor/README.md)에 보존했습니다. 아래의 [배선 전 준비와 핀별 배선](WIRING.ko.md)을 먼저 읽으세요.

**드라이버 전원 LED가 꺼져 있다면** D3 신호보다 [드라이버 10핀 9번 VCC/10번 GND의 로직 5 V 공급과 측정](WIRING.ko.md#먼저-드라이버-전원-led가-꺼져-있다면)을 먼저 확인합니다. Nano USB만으로는 드라이버가 켜지지 않습니다.

**D3를 R_ENABLE에 직접 연결하기 전에 현재 스케치를 다시 업로드하세요.** 이전 NPN용 펌웨어는 D3의 정지/구동 논리가 반대입니다. 업로드와 무동력 상태의 드라이버 8번 전압 확인 전에는 모터 전원을 연결하지 않습니다.

## 파일과 실행

Wi-Fi 조종기에서 `P`를 누르면 Nano의 내장 LED가 켜짐/꺼짐으로 바뀌고 `PONG LED=ON/OFF` 응답이 표시됩니다. `N`을 누르면 `NETWORK wifi=... ip=... rssi_dbm=... tcp_ready=... tcp_state=... udp_ready=... udp_packets=... udp_requests=... udp_replies=... udp_errors=...` 진단 응답이 나옵니다. 모터 전원을 분리한 상태에서도 명령 왕복을 시험할 수 있습니다. 이 기능은 변경된 스케치를 Nano에 다시 업로드하고 Python 조종기도 새로 실행해야 사용할 수 있습니다.

| 파일 | 용도 |
|---|---|
| [Nano_33_IoT.ino](Nano_33_IoT.ino) | 기존 Wi-Fi 접속, TCP 서버와 모터·서보 제어 |
| [WIRING.ko.md](WIRING.ko.md) | 배선 전 준비, Nano D핀과 드라이버 **10핀 번호**, 전원, 최초 시험 |
| [wifi_secrets.example.h](wifi_secrets.example.h) | 개인 Wi-Fi 비밀번호 설정용 예시 |
| [wifi_control.py](wifi_control.py) | Python 표준 라이브러리만 사용하는 노트북 키보드 조종기 |
| [DEVELOPMENT.md](DEVELOPMENT.md) | 현재 개발 방향, 완료·미완료 항목, 필요한 정보와 다음 작업 |
| [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md) | 차량 구성, 이전 버전과 모터 모델 기록 |
| [기존 Uno 참고 자료](reference/Uno_Single_Rear_Motor/README.md) | 기존 Uno 스케치·USB 조종기·배선 안내 |

Arduino IDE에서는 **`C:\Users\user\Arduino\Nano_33_IoT\Nano_33_IoT.ino`**를 엽니다. PowerShell에서는 이 폴더로 이동한 뒤 조종기를 실행합니다.

```powershell
Set-Location 'C:\Users\user\Arduino\Nano_33_IoT'
python wifi_control.py
```

개인 `wifi_secrets.h`도 이 폴더로 복제했으므로 기존 설정을 사용할 수 있습니다. `.gitignore`는 이 파일을 제외하도록 맞췄습니다.

1. Arduino IDE에 **Arduino SAMD Boards**, **WiFiNINA**, **Arduino_SpiNINA**, **Servo**를 설치하고, **Arduino Nano 33 IoT**를 선택합니다. WiFiNINA 2.1.1은 Arduino_SpiNINA 0.0.2도 사용합니다.
2. `wifi_secrets.example.h`를 같은 폴더의 `wifi_secrets.h`로 복사해 **기존 2.4 GHz Wi-Fi 공유기**의 SSID와 비밀번호를 설정한 뒤 `Nano_33_IoT.ino`를 Arduino IDE에서 열어 스케치를 업로드합니다. 비밀번호 파일은 Git에서 제외됩니다. 업로드할 때는 드라이버의 모터 전원을 분리합니다.
3. 노트북도 같은 공유기 네트워크에 연결합니다. USB 시리얼 모니터 115200 baud를 열고 Nano의 RESET 버튼을 눌러 `READY WIFI=... IP=... PORT=5000`을 확인합니다. 자동 검색을 사용하려면 `READY UDP discovery PORT=5001`도 확인합니다. `IP`는 **Nano 주소**이며 노트북 주소와 다릅니다. 공유기가 주소를 다시 배정할 수 있습니다.
4. 기존 Python 조종기가 실행 중이면 그 창에서 `Q`로 종료한 뒤 메인 폴더에서 `python wifi_control.py`를 새로 실행합니다. 조종기는 UDP **5001번**으로 Nano를 자동 검색해 TCP **5000번**에 연결합니다. TCP 연결 직후 읽기 전용 `STATUS`를 먼저 보내 Nano의 접속 처리를 시작하고, `READY` 응답을 받아야 연결 성공으로 표시합니다. 시작할 때 `STATUS`가 두 번 보일 수 있습니다. 자동 검색이 실패해도 TCP가 끊어졌다는 뜻은 아닙니다. 시리얼 모니터의 Nano 주소를 사용해 `python wifi_control.py --host <Nano IP>`로 실행하세요. 이미 다른 조종기가 접속 중이면 새 조종기가 `READY`를 받지 못할 수 있으므로 기존 조종기를 종료합니다.
5. `R`을 눌러 `STATUS channel=R ... enabled=0`을 확인하고, `P`로 LED·명령 왕복을, `N`으로 Wi-Fi/TCP/UDP 상태를 확인합니다. `W/S`는 명목 모터 전압, `A/D`는 조향, `X`/Space는 모터 정지, `C`는 조향 중앙, `I`는 정지와 중앙, `Q`/Esc는 종료입니다. TCP 클라이언트는 Nano의 IP **5000번**에 ASCII 명령과 줄바꿈을 보낼 수도 있습니다. 한 번에 조종기 한 대만 받습니다.

`STATUS`, `NETWORK`, `PING`, `STEER 5`, `CENTER`, `DRIVE 7 0`, `DRIVE -7 0`, `STOP`, `IDLE`, `MOTOR 20`, `VOLTAGE 6`을 지원합니다. `DRIVE`만 기존 7 V 기동 보정을 적용합니다. `VOLTAGE`는 실측 전압이 아닌 PWM 명목값입니다. 구동 명령 후에는 새 명령 없이도 **TCP 연결이 유지되는 동안** 마지막 출력이 계속 적용됩니다. 측정이나 시험이 끝나면 `STOP` 또는 `IDLE`을 보내세요. 방향을 바꿀 때 50 ms 동안 출력을 끕니다. TCP 연결 또는 Wi-Fi가 끊어지면 모터가 정지하고 조향은 중앙으로 갑니다.

기존 Uno의 `TIMEOUT`/`KEEPALIVE` 명령과 1.5초 명령 감시는 이 Nano 버전에 없습니다. 연결 해제 시 정지는 코드에 구현된 동작이며, 실제 모터 정지 결과는 아래 완료 기준에서 별도로 확인합니다.

`NETWORK`의 `tcp_state=1`은 수신 대기, `tcp_state=4`는 연결 중인 상태입니다. `udp_ready=0`이면 자동 검색용 소켓이 열리지 않은 상태입니다. `udp_packets`는 Nano가 받은 모든 검색 포트 패킷 수, `udp_requests`는 올바른 검색 요청 수, `udp_replies`는 응답 전송 성공 수, `udp_errors`는 응답 전송 실패 수입니다. 검색을 시도한 뒤 이 숫자를 비교하면 요청이 Nano까지 왔는지 살펴볼 수 있습니다. 검색 실패 중에도 `--host`로 접속할 수 있다면 TCP 제어 경로는 별도로 확인할 수 있습니다. `STATUS enabled=1`은 펌웨어가 출력을 명령했다는 뜻이며 모터 단자 전압의 실측값은 아닙니다.

## 전기적 확인

- Nano GPIO는 3.3 V 로직입니다. [직결 배선 문서](WIRING.ko.md#4-버퍼-없는-신호-직결과-사전-측정)대로 D3를 포함한 신호선을 직접 연결합니다. 드라이버 HIGH 최저값은 3.2 V인데 SAM D21 출력 HIGH의 부하 조건상 보증 최저값은 그보다 낮으므로, 모터 전원 없이 **드라이버 8번이 정지 시 3.2 V 이상인지와 나머지 입력의 HIGH/LOW**를 측정해야 합니다. 리셋 중 신호는 제어할 수 없으므로 지면 주행에는 사용하지 않습니다. 다른 장치의 5 V 출력을 Nano GPIO에 넣지 않습니다.
- 후륜 모터의 **정확한 모델·정격은 현 단일 모터 문서에 없습니다**. 이전 양쪽 후륜의 24 V RB-35GM 기록을 이 모터에 적용하지 않습니다. 드라이버 매뉴얼의 **모터 전원 5~46 V와 채널당 최대 2 A**는 드라이버 한계입니다. 모터 모델·정격 전압·기동/정지 전류를 확인하기 전에는 모터 전원을 켜지 않습니다. 기존 Uno의 12 V 전원 설정과 약 7 V 기동 보정은 실물 확인 후 다시 정합니다.
- 모터·드라이버 로직·서보 전원과 Nano USB는 [배선 문서의 네 전원 경로](WIRING.ko.md#처음-배선하는-사람을-위한-전원선-연결-순서)로 구분합니다. 서보 2개는 4.8~6.0 V에서 동시 정지 전류 1.6 A 이상에 여유가 있는 전원을 사용합니다. 모든 GND는 공통으로 연결하고 모터 전류가 Nano 신호 접지선을 통해 흐르지 않게 합니다.
- `VOLTAGE` 값은 PWM 비율에 대한 명목 값입니다. 엔코더와 전압 피드백이 없으므로 실제 속도나 모터 단자 전압을 일정하게 유지하지 않습니다.

Nano D9의 `analogWrite()` PWM을 사용하므로 Uno 타이머의 20 kHz 설정을 그대로 쓰지 않습니다. 드라이버의 허용 PWM 주파수를 실물 사양과 시험으로 확인하세요. 코드의 핀 번호와 배선 예시는 [핀별 표](WIRING.ko.md#2-핀-번호-한눈에-보기)에 있습니다.

## Windows에서 한글 스케치북 경로 오류가 날 때

`grpc: error while marshaling: string field contains invalid UTF-8`이 나오고 컴파일 로그의 OneDrive 경로가 깨져 보이면, Arduino IDE의 **파일 → 환경설정 → 스케치북 위치**를 `C:\Users\<사용자이름>\Arduino`처럼 영문 경로로 바꾸고 IDE를 다시 시작합니다. 그 위치의 `Nano_33_IoT` 폴더에 `.ino`와 개인 `wifi_secrets.h`를 함께 둔 뒤 새 위치의 스케치를 엽니다. 기존 OneDrive 파일은 삭제하지 않습니다. `Arduino_SpiNINA.h`가 없다는 오류가 이어지면 라이브러리 관리자에서 **Arduino_SpiNINA**도 설치합니다.

현재 메인 폴더는 **`C:\Users\user\Arduino\Nano_33_IoT`의 독립 폴더**입니다. 이전 junction은 별도 이름으로 보존했고, PowerOn·OneDrive의 기존 파일과 `Nano_33_IoT.before-repo-move-20260928` 백업도 보존했습니다. 앞으로 수정·업로드·조종기 실행은 메인 폴더를 기준으로 합니다.

## 검증 상태와 완료 기준

### 기록된 소프트웨어 시험

- [x] 2026-09-30 R 채널 스케치 컴파일 및 실제 Nano 업로드 (Arduino SAMD 1.8.14)
- [x] 실제 Nano에서 Wi-Fi 명령 수신과 상태 응답
- [x] UDP 자동 검색과 TCP `STATUS`/`NETWORK`/`PING`/`IDLE` 왕복
- [x] 모터 미연결 상태에서 2초 이상 명령 없이 소프트웨어 출력 유지와 `STOP` 후 소프트웨어 정지 상태 확인
- [ ] TCP·Wi-Fi 연결 해제 시 소프트웨어 정지·조향 중앙 처리 결과 기록

### 남은 실물 검증

- [ ] 중앙 후륜 모터 모델·정격·기동/정지 전류 및 드라이버 적합성 확인
- [ ] 바퀴를 띄운 상태에서 후륜 모터 정·역회전/정지와 드라이버 R 채널 사용·L 채널 비활성 확인
- [ ] 서보 2개의 중앙, 작은 좌·우 조향 및 기구 한계 확인
- [ ] 실제 모터의 명령 없는 구동 유지, `STOP`/`IDLE` 및 TCP·Wi-Fi 연결 해제 시 정지 확인
- [ ] 3.3 V 신호 호환성, 전원 용량, 공통 GND, 드라이버 발열 확인

### 리셋·전원 손실의 한계

현재 직결 구성은 **리셋·USB 전원 손실 중 안전 정지를 보장하지 않습니다.** 재부팅 시 정지를 완료 항목으로 약속하지 않고, 모터 전원을 분리한 상태에서만 리셋 중 입력 상태를 측정합니다. 지면 주행이 목표라면 이 구간의 정지 대책이 먼저 필요합니다. 소프트웨어 시험의 완료 표시는 모터 단자 전압이나 실물 정지가 검증됐다는 뜻이 아닙니다.
