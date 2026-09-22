# ESP-01 Wi-Fi TCP 브리지

이 스케치는 ESP8266 ESP-01을 차량 전용 Wi-Fi AP 및 TCP-UART 브리지로 사용한다.

```text
Laptop
  │ Wi-Fi / TCP 192.168.4.1:5000
  ▼
ESP-01
  │ UART 38400 baud
  ▼
Arduino Uno A0/A1
  │
  ▼
단일 후륜 모터와 조향 서보
```

## 기본 네트워크 설정

| 항목 | 기본값 |
|---|---|
| SSID | `PowerOn-Car` |
| 비밀번호 | `poweron-car` |
| ESP 주소 | `192.168.4.1` |
| TCP 포트 | `5000` |
| ESP↔Uno UART | `38400 baud` |

운용 전에 `WiFi_ESP01.ino`의 `kAccessPointPassword`를 8자 이상의 별도 비밀번호로 변경한다.

## ESP-01 펌웨어 업로드

Arduino IDE의 Boards Manager URL에 다음 주소를 추가한다.

```text
https://arduino.esp8266.com/stable/package_esp8266com_index.json
```

`esp8266 by ESP8266 Community` 코어를 설치한 뒤 다음 설정을 사용한다.

- Board: `Generic ESP8266 Module`
- Flash Size: `1MB (FS:64KB OTA:~470KB)`
- Flash Mode: `DOUT`
- Upload Speed: `115200`

업로드할 때는 USB-TTL 어댑터와 ESP를 다음처럼 연결한다.

> 업로드 중에는 ESP의 4번/5번 UART 선을 Uno A1/A0에서 분리한다. USB-TTL과 Uno가 같은 ESP UART를 동시에 구동하면 신호 충돌이나 역급전이 발생할 수 있다.

| ESP 핀 | 업로드 연결 |
|---:|---|
| 1 GND | USB-TTL 및 외부 3.3 V 전원 GND |
| 2 GPIO2 | 10 kΩ으로 3.3 V pull-up |
| 3 GPIO0 | GND — 다운로드 모드 |
| 4 RXD | 3.3 V USB-TTL TX |
| 5 TXD | USB-TTL RX |
| 6 EN | 10 kΩ으로 3.3 V pull-up |
| 7 RST | 평소 3.3 V pull-up, 업로드 진입 시 잠시 GND |
| 8 VCC | 별도 안정화 3.3 V 전원 |

ESP에 5 V를 공급하지 않는다. USB-TTL의 신호 전압도 3.3 V여야 하며, ESP 전원은 최소 500 mA급 별도 3.3 V 공급원을 사용한다.

업로드가 끝나면 전원을 끄고 GPIO0을 10 kΩ으로 3.3 V에 pull-up한 정상 실행 배선으로 되돌린다. 그 후 RST를 한 번 LOW로 내렸다가 HIGH로 복귀시키거나 전원을 다시 넣는다.

## 동작과 안전 정지

- TCP 연결이 새로 열리면 ESP가 Uno에 `IDLE`을 보내 이전 출력을 정지한다.
- TCP 연결이 끊기면 ESP가 즉시 `IDLE`을 보낸다.
- 노트북은 주행 중 0.5초마다 `KEEPALIVE`를 보낸다.
- ESP 또는 네트워크가 멈춰도 Uno가 1.5초 동안 명령을 받지 못하면 모터를 정지한다.
- ESP는 한 번에 한 명의 TCP 조종기만 사용한다.

## 빌드 명령

```powershell
arduino-cli compile --fqbn "esp8266:esp8266:generic:eesz=1M64" .
```

실제 업로드 포트가 `COM5`라면 다음과 같이 업로드할 수 있다.

```powershell
arduino-cli upload -p COM5 --fqbn "esp8266:esp8266:generic:eesz=1M64" .
```
