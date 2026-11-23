# 라이다와 탑재 컴퓨터

## RPLIDAR A1M8-R5

- **장착 상태** · 차량 고정 미확인. Windows PC와 Raspberry Pi에서 USB 스캔을 읽은 기록이 있습니다.
- **구성** · SLAMTEC 360° 2D 스캔 센서, CP210x USB 시리얼 어댑터. 프로젝트는 표준 스캔과 115200 bps를 사용합니다. [장치 사양](https://www.slamtec.com/en/lidar/a1) · [A1M8 데이터시트](https://bucket.download.slamtec.com/7fe7e3656e811ab1a645753af40809f05fa7ddcd/LD108_SLAMTEC_rplidar_datasheet_A1M8_v2.4_en.pdf)
- **임시 장착값** · 정면 원시 각도 0°, 뒷차축 중심보다 66.62 mm 앞, 거리 오프셋 0 mm. 코드가 차체 좌표로 변환할 때 사용합니다.
- **측정 필요** · 고정 후 라이다 중심 위치·높이·정면각, 차체가 가리는 범위, 기준 물체 거리 편차와 실제 스캔 주기.

## Raspberry Pi 4 Model B

- **장착 상태** · 라이다 수집과 SSH 미리보기에서 사용 확인. 차량 탑재와 배터리 전원 공급은 미확인입니다.
- **역할** · USB에서 스캔을 받아 경로·속도를 계산하고, 명시적으로 주행 모드를 실행할 때 Nano에 TCP 명령을 보냅니다. 보드 전원 사양은 USB-C 5 V / 3 A입니다. [Raspberry Pi 4 사양](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/#specifications)
- **확인 필요** · 차체 고정 위치, 이동 중 USB 접속 유지와 전원 용량, 장착 후 센서 시야.

[RPLIDAR 설치와 USB 시험](../../lidar/acquisition/setup.md) · [Pi 주행 준비](../../lidar/autopilot/guide.ko.md#4-pi에서-주행-준비)
