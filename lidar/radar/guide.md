# RPLIDAR 실시간 레이더

[라이다와 Pi 설정 / Lidar and Pi setup](../acquisition/setup.md)

## 한국어 절차

### 화면에서 작동 확인

1. Windows 파일 탐색기에서 프로젝트 폴더를 엽니다.
2. [lidar-radar.cmd](lidar-radar.cmd)를 더블 클릭합니다.
3. **Pi SSH 로그인** 창에서 Pi 계정 비밀번호를 입력합니다.
4. 주소나 포트를 바꿔야 하면 로그인 창에서 **Cancel**을 누릅니다. **연결**에서 `Raspberry Pi (SSH)`를 선택합니다. **SSH 주소**와 **USB 포트**를 수정합니다. **시작**을 누릅니다.
5. 상태 표시줄에 **수신 중**이 나타나는지 확인합니다.
6. 화면 아래의 **수신**과 **누적 유효값**이 모두 증가하는지 확인합니다.
7. 레이더 화면에 녹색 점이 나타나는지 확인합니다.
8. 점이 희박하면 센서에서 약 1 m 떨어진 곳에 상자를 둡니다. 상자의 옆면이 라이다가 측정하는 높이를 가로지르게 놓습니다. 상자를 옆으로 움직입니다. 화면에서 해당 점의 위치가 변하는지 확인합니다.
9. **정지**를 누릅니다.
10. **정지됨**이 표시되는지 확인합니다. 수신 숫자가 더 이상 증가하지 않는지 확인합니다.
11. 재시작을 시험하려면 **시작**을 누릅니다. Pi 계정 비밀번호를 입력합니다. 녹색 점이 다시 나타나는지 확인합니다. 수신 숫자가 다시 증가하는지 확인합니다.

**통과 기준**은 녹색 점 표시, 유효값 증가, 정지 후 수신 중단, 재시작 후 수신 재개입니다. 화면의 중심은 센서 위치입니다. 화면 위쪽이 센서 기준 0°입니다. 라이다 모터가 도는 것은 전원이 들어왔다는 단서입니다. 거리 데이터는 화면의 유효값으로 확인합니다.

### USB 및 SSH 연결 옵션

RPLIDAR A1M8-R5가 Raspberry Pi에 연결되어 있으면 Windows에서 `lidar-radar.cmd`를 더블 클릭하고 Pi 계정 비밀번호를 입력합니다. 기본 SSH 주소는 `user@andrea.local`이고 Pi의 USB 포트는 `/dev/ttyUSB0`입니다. Pi의 시리얼 수집기는 레이더를 실행할 때 SSH 세션에서 시작됩니다.

- 새 PC에서는 PowerShell에서 `ssh user@andrea.local`로 먼저 접속해 Pi의 SSH 키를 확인합니다. 이름이 검색되지 않으면 레이더의 **SSH 주소**에 `user@<Pi IP>`를 입력하고, 해당 IP의 SSH 키도 먼저 확인합니다.
- Windows USB에 직접 연결했을 때 CP2102 포트가 한 개면 자동 선택합니다. 포트가 비어 있거나 다르면 **PC USB**를 선택하고 `COM19`처럼 입력합니다. 같은 포트를 사용하는 프로그램은 종료한 뒤 **시작**을 누릅니다.
- 녹색 점은 최근 1초 동안의 유효 거리 반환입니다. 각도 0°는 화면 위이고, 증가 방향은 시계 방향입니다. 실제 차량 전방과의 대응은 확인하지 않았습니다.
- 표시 반경은 1/2/4/8/12 m 중 선택합니다. **정지** 또는 창 닫기로 스캔을 중단하고 USB 포트를 해제합니다. 스캔 CSV가 필요하면 별도로 수집합니다.

저장소 루트에서 `python -m lidar.radar --ssh user@andrea.local` 또는 `python -m lidar.radar --port COM19`로도 실행할 수 있습니다.

### 작업 종료

1. 레이더에서 **정지**를 누릅니다.
2. **정지됨**이 표시되면 레이더 창을 닫습니다.
3. Pi 전원을 끌 경우 SSH로 Pi에 다시 로그인합니다.
4. Pi 터미널에서 `sudo poweroff`를 실행합니다.
5. Pi가 종료될 때까지 기다립니다.
6. Pi의 전원 어댑터를 분리합니다.

## English procedure

### Start and check the radar

1. Open the project folder in Windows File Explorer.
2. Double-click `lidar-radar.cmd`.
3. Enter the Pi account password in the **Pi SSH 로그인** window.
4. If you must change the address or port, select **Cancel** in the login window.
5. Select `Raspberry Pi (SSH)` in the **연결** list.
6. Enter the Pi SSH address in the **SSH 주소** field.
7. Enter the Pi USB port in the **USB 포트** field.
8. Select **시작** to start the scan.
9. Make sure that the status shows **수신 중**.
10. Make sure that the **수신** and **누적 유효값** counts increase.
11. Make sure that the display shows green points.
12. If the display shows few points, put a box approximately 1 m from the sensor.
13. Put the side of the box across the scan plane of the sensor.
14. Move the box sideways.
15. Make sure that the position of the related points changes on the display.
16. Select **정지**.
17. Make sure that the status shows **정지됨**.
18. Make sure that the receive count stops increasing.
19. To do a restart test, select **시작**.
20. Enter the Pi account password.
21. Make sure that the green points appear again.
22. Make sure that the receive count increases again.

For a standard start, the launcher starts the scan after login. Steps 4 through 8 apply when the address or USB port changes. The display center represents the sensor position. The top of the display represents 0° relative to the sensor.

### Stop and disconnect

1. Select **정지** in the radar window.
2. When the status shows **정지됨**, close the radar window.
3. If you must remove Pi power, open a new SSH session.
4. Enter `sudo poweroff` in the Pi terminal.
5. Wait for the Pi to shut down.
6. Disconnect the Pi power supply.
