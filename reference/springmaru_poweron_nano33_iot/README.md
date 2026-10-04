# springmaru Nano 33 IoT Wi-Fi 진단 참고 자료

자료 수집일: **2026-10-04 (한국 시간)**. 이 날짜는 저장소를 받아 정리한 날짜이며 보드 업로드·실물 시험 완료일이 아닙니다.

원본: [springmaru/poweron-nano33-iot](https://github.com/springmaru/poweron-nano33-iot), `main`의 [f013d8a](https://github.com/springmaru/poweron-nano33-iot/commit/f013d8a60b3232d6363ed5b294f937fa007c08f0). 원본의 4개 추적 파일을 수정 없이 `upstream/`에 보관했습니다. 커밋 전체 값과 각 파일의 SHA-256은 [UPSTREAM.json](UPSTREAM.json)에 있습니다.

## 폴더 구분

| 위치 | 역할 | 연결된 원격 저장소 |
|---|---|---|
| `C:\Users\user\Arduino\Nano_33_IoT` | 현재 차량 개발·업로드·조종기 실행 기준 | `choisw2718/PowerOn` |
| `C:\Users\user\Arduino\external_repos\springmaru\poweron-nano33-iot` | 외부 원본의 독립 Git clone, 이후 pull 위치 | `springmaru/poweron-nano33-iot` |
| 이 폴더의 `upstream/` | PowerOn에 함께 올리는 원본 고정 사본 | 부모 PowerOn에서 관리; 외부 `.git` 없음 |

원본 clone의 파일을 현재 프로젝트 최상위에 덮어쓰지 않습니다. `upstream/` 사본은 외부 pull과 자동 동기화되지 않습니다. 메인 스케치와 조종기, 개인 Wi-Fi 설정은 계속 메인 폴더를 기준으로 사용합니다.

## 어떤 파일을 사용하면 되는가

| 파일 | 용도 |
|---|---|
| [Nano33IoTWifiServer.ino](upstream/Nano33IoTWifiServer/Nano33IoTWifiServer.ino) | NINA·Wi-Fi 스캔과 접속·HTTP 테스트를 수행하는 독립 스케치 |
| [arduino_secrets.example.h](upstream/Nano33IoTWifiServer/arduino_secrets.example.h) | 진단 스케치의 개인 접속 설정용 예시 |
| [원본 README](upstream/README.md) | 원본 macOS/CLI 사용 설명과 결과 판정 |
| [원본 .gitignore](upstream/.gitignore) | 원본 저장소의 개인 설정·빌드 출력 제외 규칙 |

Arduino IDE에서 진단 스케치를 열 때는 외부 clone의 `Nano33IoTWifiServer\Nano33IoTWifiServer.ino`를 사용하면 원본 업데이트와 개인 설정을 그 독립 폴더에 유지할 수 있습니다. 다른 PC에서는 이 참고 자료의 `upstream/Nano33IoTWifiServer` 폴더도 그대로 사용할 수 있습니다. `.ino` 이름과 그 상위 폴더 이름을 함께 유지합니다.

## 현재 코드와의 차이와 활용 후보

| 항목 | 현재 PowerOn 메인 | 외부 진단 스케치 | 활용 방법 |
|---|---|---|---|
| 목적 | R 채널 모터 1개·서보 2개 제어 | 무선 모듈·네트워크 진단 | 통신 문제를 차량 코드와 분리해 확인 |
| 설정 파일 | 필수 `wifi_secrets.h` | 선택 `arduino_secrets.h`; 없으면 스캔 전용 | 진단만 할 때 비밀번호 없이 시작 가능 |
| 네트워크 검색 | UDP 5001로 Nano 발견 | 주변 Wi-Fi SSID·RSSI·암호화 표시 | 공유기 검색용 `scanNetworks()` 참고 |
| 통신 | TCP 5000 ASCII 명령 | TCP 8000 HTTP/1.1 JSON | 브라우저·`curl.exe`로 별도 네트워크 시험 |
| 진단 정보 | `STATUS`·`NETWORK`·`PING` | NINA 버전·MAC·접속 상태 이름·SSID 목록 | 시작 시 펌웨어·MAC 표시를 향후 반영 후보로 보관 |
| LED | `PING`마다 켜짐/꺼짐 변경 | 접속 시 켜짐, `/led/on`·`/led/off`로 설정 | 진단 펌웨어의 LED 동작은 따로 해석 |
| 구동 명령 | `DRIVE`·`MOTOR`·`STEER`·`STOP`·`IDLE` | 없음 | 현재 `wifi_control.py`를 진단 스케치에 연결하지 않음 |

재사용하기 좋은 부분은 `wifiStatusName()`, `encryptionName()`, `printMacAddress()`, 초기 NINA 펌웨어 버전 출력입니다. `scanNetworks()`는 무선 검색에 사용할 수 있지만 반복 스캔을 현재 차량 제어 루프에 바로 넣지 않습니다. HTTP 처리에는 최대 1초 대기 루프가 있고 Wi-Fi 접속에는 20초 타임아웃 설정이 있으므로, 현재 모터·서보 제어와 결합하려면 응답 지연과 연결 해제 처리를 다시 검토해야 합니다. 이번 작업은 참고 자료 정리이며 현재 구동 코드에 이 기능들을 합치지 않았습니다.

## Windows에서 독립 진단하기

진단 스케치는 모터 드라이버의 정지 핀을 설정하지 않습니다. 실행 중인 메인 조종기를 `Q`로 종료하고, 모터·드라이버·서보의 전원과 Nano 신호선을 분리한 뒤 Nano만 USB에 연결합니다. 진단 스케치 업로드는 보드의 메인 차량 펌웨어를 교체합니다.

1. Arduino IDE에서 외부 clone의 `Nano33IoTWifiServer\Nano33IoTWifiServer.ino`를 엽니다.
2. 보드를 **Arduino Nano 33 IoT**, 포트를 연결된 Nano의 포트로 선택합니다. 필요한 라이브러리는 WiFiNINA이며 현재 노트북의 WiFiNINA 2.1.1은 Arduino_SpiNINA 0.0.2를 사용합니다. 보드 패키지는 Arduino SAMD Boards입니다.
3. `arduino_secrets.h`를 만들지 않으면 스캔 전용으로 컴파일됩니다. 업로드 후 시리얼 모니터 **115200 baud**에서 NINA 버전, MAC, 주변 SSID와 RSSI를 확인합니다.
4. 접속·HTTP 시험이 필요하면 아래 명령으로 예시 설정을 복사하고 그 파일의 `WIFI_SSID`·`WIFI_PASSWORD`를 실제 2.4 GHz 네트워크 값으로 편집합니다. 기존 `wifi_secrets.h`는 수정하거나 덮어쓰지 않습니다. 두 설정 파일은 매크로 이름은 같아도 파일명이 다릅니다.

```powershell
Set-Location 'C:\Users\user\Arduino\external_repos\springmaru\poweron-nano33-iot'
if (-not (Test-Path '.\Nano33IoTWifiServer\arduino_secrets.h')) {
    Copy-Item '.\Nano33IoTWifiServer\arduino_secrets.example.h' '.\Nano33IoTWifiServer\arduino_secrets.h'
}
notepad '.\Nano33IoTWifiServer\arduino_secrets.h'
```

5. 설정 후 다시 컴파일·업로드하고, 시리얼에 표시된 IP를 사용해 같은 Wi-Fi의 노트북에서 시험합니다.

```powershell
$nanoIp = Read-Host '시리얼에 표시된 Nano IP를 입력'
curl.exe --connect-timeout 3 --max-time 5 "http://${nanoIp}:8000/ping"
curl.exe --connect-timeout 3 --max-time 5 "http://${nanoIp}:8000/led/on"
curl.exe --connect-timeout 3 --max-time 5 "http://${nanoIp}:8000/led/off"
```

브라우저에서도 `http://<Nano IP>:8000/ping`을 열 수 있습니다. HTTP 주소는 진단 펌웨어를 업로드한 경우에만 사용합니다. 시리얼의 `HTTP test server: ...` 문구는 `server.begin()` 호출 뒤 출력되며, 실제 접속 성공은 `/ping`의 JSON 응답으로 확인합니다. 원본 스케치는 로컬 진단용으로 인증 기능이 없습니다.

6. 시험을 마치면 구동부를 분리한 상태에서 **`C:\Users\user\Arduino\Nano_33_IoT\Nano_33_IoT.ino`**를 다시 업로드합니다. 기존 개인 `wifi_secrets.h`를 사용하고 메인 `READY`·`STATUS`·`NETWORK` 응답을 확인한 뒤, [메인 배선·시험 순서](../../WIRING.ko.md)를 따릅니다.

`arduino_secrets.h`는 외부 원본과 PowerOn 양쪽의 `.gitignore`에서 제외합니다. 실제 접속 정보는 이번 작업에서 새로 복사하거나 생성하지 않았습니다.

## 외부 저장소만 업데이트하기

```powershell
Set-Location 'C:\Users\user\Arduino\external_repos\springmaru\poweron-nano33-iot'
git status --short
git pull --ff-only
git log -1 --oneline
```

로컬 수정 때문에 pull이 실패하면 수정 내용을 확인하고 보존합니다. `reset --hard`나 `clean`으로 정리하지 않습니다. 외부 업데이트를 PowerOn에도 기록할 때는 변경 내용을 검토한 뒤 추적된 원본 파일만 `upstream/`에 복사하고, `UPSTREAM.json`의 커밋·수집일·해시를 함께 갱신합니다. 외부 `.git`, 개인 `arduino_secrets.h`, 빌드 출력은 복사하지 않습니다. 그런 다음 메인 PowerOn 폴더에서 정리 내용을 커밋하고 push합니다.

## 이번 정리의 확인 범위

외부 clone과 `git pull --ff-only`, 원본 커밋 기록, 원본·사본 4개 파일의 해시 일치, 문서의 로컬 링크, 비밀 설정 제외 여부를 확인했습니다.

**2026-10-04 컴파일 검증:** Arduino SAMD 1.8.14, WiFiNINA 2.1.1, Arduino_SpiNINA 0.0.2에서 설정 파일 없는 스캔 전용 모드와 공개 예시 설정을 넣은 HTTP 모드가 모두 컴파일에 성공했습니다. HTTP 모드는 외부 clone의 Git 제외 `build/` 폴더에 만든 별도 시험 사본으로 확인했습니다. 개인 비밀번호는 사용하지 않았습니다.

보드 업로드, HTTP 응답과 실제 무선 검색은 이번 정리에서 수행하지 않았습니다. 메인의 2026-09-30 실물 시험 기록과 완료 표시는 그대로 유지합니다.
