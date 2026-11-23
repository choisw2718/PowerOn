# Arduino Nano 33 IoT

- **장착 상태** · 차량 제어에 사용 확인. USB 포트, Wi‑Fi 명령 응답과 L298N용 펌웨어 상태를 확인한 기록이 있습니다. 차체 고정 방식은 미확인입니다.
- **보드 구성** · SAMD21, NINA-W102 Wi‑Fi, 3.3 V I/O. 내장 LSM6DS3 IMU는 현재 구동 제어에 연동하지 않습니다. [Arduino 제품 문서](https://docs.arduino.cc/hardware/nano-33-iot/) · [Nano 33 IoT 데이터시트](https://docs.arduino.cc/resources/datasheets/ABX00027-datasheet.pdf)
- **차량에서 하는 일** · Wi‑Fi TCP 명령 수신, L298N PWM과 D5/D6 서보 구동. IR 입력용 A0/A1/A2는 차량에서 3개 모두 비활성으로 설정합니다.
- **피드백** · 좌우 후륜 엔코더가 장착되어 있습니다. 배선, 회전당 카운트, 실제 구동 중 수신 상태를 각각 확인해야 합니다. 앞바퀴의 실제 각도를 직접 측정하는 센서는 확인되지 않았습니다.

[업로드와 이전 연동 결과](../../vehicle/firmware/upload.ko.md)
