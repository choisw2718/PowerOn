# Nano 33 IoT 펌웨어 업로드

SHARP GP2Y0A21YK0F 3개를 사용하는 자율주행 초안을 추가했습니다. A0/A1/A2 입력 드라이버를 구현했고 **2026-11-10 COM6 업로드·실제 ADC 응답을 확인했습니다.** 현재 확인한 Nano IP는 **192.168.50.223**이며 DHCP로 바뀔 수 있습니다. 센서 보정 전 실제 자율주행 출력은 비활성 상태이고 실물 주행은 시험하지 않았습니다. [센서 배선·보정](../sensor_monitor/guide.ko.md), 센서 없는 PC 검사와 Nano 가상 거리 미리보기는 [자율주행 안내](../ir_wall_following/guide.ko.md)를 따릅니다. 아래 연결 목록은 2026-10-08 기록입니다.

## 확인된 연결

2026-10-08 실제 PC와 Nano에서 확인했습니다.

- USB: Arduino Nano 33 IoT, **COM6**.
- PC Wi-Fi와 로컬 설정 SSID: **poweron**.
- Nano UDP 검색: **192.168.50.222**, 제어 TCP **5000**, 검색 UDP **5001**.
- TCP `READY`, `STATUS`, `NETWORK`, `STOP` 응답 성공.
- 실행 중 펌웨어: `driver=L298N channel=A`; 정지 상태 `duty_cmd=0.0% duty_applied=0.0% enabled=0`.
- 사용자는 모터 전원을 껐거나 분리했다고 확인했습니다. 이번 확인에서 모터 구동 명령은 보내지 않았습니다. 출력 전압·회전·실물 정지는 측정하지 않았습니다.

IP는 공유기의 주소 배정에 따라 바뀔 수 있습니다. PC 주소 `192.168.50.66` 대신 **Nano 주소**를 사용합니다.

## 업로드가 필요한 경우

코드를 바꿨을 때는 모터 전원을 분리하고 Arduino IDE에서 [PowerOnNano.ino](PowerOnNano/PowerOnNano.ino)를 엽니다. 보드는 **Arduino Nano 33 IoT**, 포트는 현재 USB 연결에서 감지된 포트를 선택합니다. 과거 시험에서는 **COM6**였습니다. 설치된 보드 코어는 Arduino SAMD 1.8.14이며 라이브러리는 Servo, WiFiNINA, Arduino_SpiNINA를 사용합니다. 개인 `wifi_secrets.h`는 스케치 폴더에 둡니다. 새 환경에서는 [공개 예시](PowerOnNano/wifi_secrets.example.h)를 복사해 개인 설정을 입력합니다.

연결이 다시 실패하면 모터 전원을 분리한 채 시리얼 모니터 **115200 baud**를 열고 Nano를 리셋하여 `READY WIFI=... IP=... PORT=5000`의 최신 IP를 확인합니다.

저장소 루트에서 Arduino CLI로 컴파일할 수 있습니다.

차량 파라미터를 수정했다면 먼저 [공용 프로필](../../hardware/README.md)의 Arduino 헤더를 갱신합니다.

```powershell
arduino-cli compile --fqbn arduino:samd:nano_33_iot vehicle/firmware/PowerOnNano
```

[현재 차량 배선](wiring.ko.md) · [수동 조종](../manual_control/guide.ko.md)
