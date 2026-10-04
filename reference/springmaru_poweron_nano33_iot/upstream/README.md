# PowerOn Nano 33 IoT Wi-Fi 서버

Arduino Nano 33 IoT의 NINA 무선 모듈을 진단하고, 같은 로컬 네트워크에서 사용할 수 있는 TCP 8000번 HTTP 테스트 서버를 실행한다.

- 기본 상태: 주변 2.4 GHz Wi-Fi 네트워크 검색
- 자격 증명 추가 시: AP 접속, DHCP 주소, RSSI 및 TCP 8000번 HTTP 서버 확인
- 내장 LED 켜짐: Wi-Fi 접속 성공
- 내장 LED 빠른 점멸: NINA 모듈을 찾지 못함

## 주의사항

- Nano 33 IoT의 GPIO는 3.3 V 전용이며 5 V 입력을 허용하지 않는다.
- 이 테스트에서는 차량, 모터 드라이버 및 서보를 연결하지 않는다.
- Nano 33 IoT는 2.4 GHz Wi-Fi를 사용한다. 5 GHz 전용 SSID는 검색하거나 연결할 수 없다.
- Wi-Fi 비밀번호가 포함된 `arduino_secrets.h`는 Git에 커밋하지 않는다.

## 설치된 CLI 환경

이 Mac에는 Homebrew의 `arduino-cli`를 사용한다. 필요한 구성 요소는 다음과 같다.

```bash
brew install arduino-cli
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:samd
arduino-cli lib install WiFiNINA
```

Arduino IDE를 선호한다면 IDE의 Boards Manager에서 `Arduino SAMD Boards (32-bits ARM Cortex-M0+)`를 설치하고, Library Manager에서 `WiFiNINA`를 설치한다. 보드는 `Arduino Nano 33 IoT`로 선택한다.

## 1. 보드 연결 확인

Nano 33 IoT를 데이터 전송이 가능한 USB 케이블로 연결한다.

```bash
arduino-cli board list
```

macOS에서는 포트가 일반적으로 `/dev/cu.usbmodem...` 형태로 표시된다. 보드가 표시되지 않으면 USB 케이블을 바꾸고, 리셋 버튼을 빠르게 두 번 눌러 부트로더 모드로 진입한 뒤 다시 확인한다.

## 2. 자격 증명 없이 스캔 테스트

프로젝트 루트에서 다음 명령을 실행한다.

```bash
arduino-cli compile \
  --fqbn arduino:samd:nano_33_iot \
  Nano33IoTWifiServer
```

아래 명령의 `<PORT>`를 `board list`에서 확인한 포트로 바꾼다.

```bash
arduino-cli upload \
  --fqbn arduino:samd:nano_33_iot \
  --port <PORT> \
  Nano33IoTWifiServer

arduino-cli monitor \
  --port <PORT> \
  --config baudrate=115200
```

정상이라면 NINA 펌웨어 버전, MAC 주소와 주변 SSID 목록이 출력된다.

## 3. 실제 Wi-Fi 접속 테스트

예제 설정을 복사한다.

```bash
cp Nano33IoTWifiServer/arduino_secrets.example.h \
   Nano33IoTWifiServer/arduino_secrets.h
```

`arduino_secrets.h`에 2.4 GHz SSID와 비밀번호를 입력한 뒤 다시 컴파일하고 업로드한다. 정상 접속되면 시리얼 모니터에 SSID, IP 주소와 RSSI가 표시되고 내장 LED가 켜진다.

개방형 네트워크라면 `WIFI_PASSWORD` 값을 빈 문자열로 둔다.

## 4. 같은 Wi-Fi에서 8000번 포트 통신

Nano와 Mac을 모두 `PowerOn` Wi-Fi에 연결한다. 시리얼 모니터에 표시된 Nano의 IP가 예를 들어 `192.168.0.35`라면 Mac에서 다음 명령을 실행한다.

```bash
curl --connect-timeout 3 http://192.168.0.35:8000/ping
curl --connect-timeout 3 http://192.168.0.35:8000/led/on
curl --connect-timeout 3 http://192.168.0.35:8000/led/off
```

정상 응답 예시는 다음과 같다.

```json
{"ok":true,"device":"Arduino Nano 33 IoT","port":8000,"rssi":-48,"led":true}
```

웹 브라우저에서 `http://<NANO_IP>:8000/ping`을 열어도 된다. 이 서버는 같은 로컬 네트워크에서만 접근하는 진단용이며 인증 기능은 없다. Nano와 Mac이 같은 SSID에 있어도 게스트 네트워크 또는 AP의 client isolation 기능이 켜져 있으면 장치 간 통신이 차단될 수 있다.

## 결과 판정

| 출력 | 의미 |
|---|---|
| `NINA firmware: ...`와 SSID 목록 | 무선 모듈과 SPI 통신 정상 |
| `IP: ...` | AP 인증과 DHCP 정상 |
| `HTTP test server: http://...:8000` | Nano의 8000번 HTTP 서버 시작 완료 |
| `/ping`에서 JSON 응답 | Mac과 Nano 사이 TCP 통신 정상 |
| `WL_NO_MODULE` 오류 및 빠른 LED 점멸 | NINA 펌웨어 또는 보드 인식 점검 필요 |
| 네트워크 0개 | 2.4 GHz AP, 거리, 안테나 주변 금속 구조물 확인 |

접속 상태가 `SSID not found`라면 SSID 철자와 2.4 GHz 활성화를 확인한다. `authentication/connect failed`라면 비밀번호와 WPA2-Personal 보안 설정을 확인한다. WPA3 전용, 기업용 802.1X 로그인 및 웹 로그인 방식의 captive portal은 이 단순 테스트의 대상이 아니다.
