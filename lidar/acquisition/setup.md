# RPLIDAR A1 연결 및 작동 확인 매뉴얼

[한국어 절차](#한국어-절차) · [English procedure](#english-procedure) · [레이더 사용법 / Radar guide](../radar/guide.md)

라이다 스캔으로 차량 조향을 계산하려면 [벽 추종 자율주행 안내](../autopilot/guide.ko.md)를 참조합니다.

이 매뉴얼은 **RPLIDAR A1M8-R5를 Raspberry Pi 4B에 USB로 연결하고 Windows PC에서 실시간 점을 확인**하는 방법을 설명합니다.

영문 절차는 ASD-STE100 Issue 9의 절차 작성 규칙을 적용했습니다. 명령형을 쓰고, 한 문장에 한 동작을 넣고, 절차 문장을 20단어 이하로 작성했습니다. [ASD-STE100 Issue 9](https://www.asd-ste100.org/assets/files/ASD-STE100_ISSUE9.pdf)

## 한국어 절차

### 1. 준비물과 연결 구조

| 준비물 | 확인 사항 |
| --- | --- |
| RPLIDAR A1M8-R5 | 라이다 본체, 기본 연결선, USB 어댑터, USB 데이터 케이블을 준비합니다. |
| Raspberry Pi 4B | Raspberry Pi OS가 설치된 microSD와 5 V / 3 A USB-C 전원 어댑터를 준비합니다. |
| Windows PC | `Nano_33_IoT` 프로젝트 폴더와 Python을 준비합니다. |
| 공유기 | PC와 Pi를 같은 로컬 네트워크에 연결합니다. Pi를 유선으로 연결할 때는 랜선을 준비합니다. |

```text
RPLIDAR A1 → 기본 연결선 → USB 어댑터 → USB 데이터 케이블 → Pi 4B
Pi 4B → Wi-Fi 또는 랜선 → 공유기 → Wi-Fi 또는 랜선 → Windows PC
```

Pi의 USB-C 전원 입력에는 전원 어댑터를 연결합니다. 라이다 USB 데이터 케이블은 Pi의 USB-A 포트에 연결합니다. [Pi 4B 사양](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/#specifications) · [SLAMTEC A1M8 연결 안내](https://bucket.download.slamtec.com/269e60a69933dffb1067a2ee3405f2918168064a/LM108_SLAMTEC_rplidarkit_usermanual_A1M8_v2.1_en.pdf)

### 2. 처음 한 번만 설정

Pi에 Raspberry Pi OS와 SSH 설정이 이미 있으면 3절로 이동합니다. SSH는 Windows에서 Pi의 터미널을 열 수 있는 접속 방식입니다.

1. Windows에서 Raspberry Pi Imager를 엽니다.
2. 기기로 **Raspberry Pi 4**를 선택합니다.
3. 운영체제로 **Raspberry Pi OS**를 선택합니다. 화면 없이 쓰는 경우 **Raspberry Pi OS Lite**를 선택할 수 있습니다.
4. 저장 장치로 사용할 microSD를 선택합니다. **이미지를 쓰면 선택한 카드의 기존 데이터가 지워집니다.**
5. 호스트 이름에 `andrea`를 입력합니다.
6. Pi 사용자 이름에 `user`를 입력합니다.
7. Pi 계정 비밀번호를 설정합니다.
8. Wi-Fi를 사용할 경우 PC의 Wi-Fi 목록에 표시된 이름을 대소문자까지 맞춰 입력합니다.
9. Wi-Fi 비밀번호를 입력합니다.
10. **Remote Access**에서 **Enable SSH**를 켭니다.
11. **Use password authentication**을 선택합니다.
12. 이미지를 씁니다.
13. 쓰기와 확인이 끝나면 microSD를 꺼냅니다.

`andrea`와 `user`는 이 프로젝트 실행 파일의 기본값입니다. 다른 값으로 설정했다면 SSH 명령과 레이더의 **SSH 주소**도 바꿉니다. [Raspberry Pi Imager 설정 안내](https://www.raspberrypi.com/documentation/computers/getting-started.html#step-3-customise-your-operating-system-recommended)

### 3. 장비 연결 및 Pi 접속

1. Pi 전원이 꺼진 상태에서 microSD를 삽입합니다.
2. 라이다 기본 연결선을 USB 어댑터에 연결합니다.
3. USB 데이터 케이블로 USB 어댑터를 Pi의 USB-A 포트에 연결합니다.
4. 유선 연결을 사용할 경우 Pi의 랜 포트와 공유기의 **LAN** 포트를 랜선으로 연결합니다. Wi-Fi를 사용할 경우 2절에서 설정한 네트워크를 사용합니다.
5. PC를 같은 공유기의 네트워크에 연결합니다.
6. Pi의 USB-C 전원 어댑터를 연결합니다.
7. Pi가 부팅할 때까지 기다립니다.
8. **Windows PowerShell**에서 다음 명령을 실행합니다.

   ```powershell
   ssh user@andrea.local
   ```

9. 처음 접속할 때 SSH 키 확인 메시지가 나오면 접속 대상이 해당 Pi인지 확인합니다. 확인 후 `yes`를 입력합니다.
10. **Pi 계정 비밀번호**를 입력합니다. 비밀번호 입력 중에는 화면에 문자가 표시되지 않습니다.
11. `user@andrea:~$`와 비슷한 프롬프트가 나오면 다음 명령을 실행합니다. `andrea`가 나오면 Pi에 로그인한 상태입니다.

    ```bash
    hostname
    ```

빨간 `PWR` LED는 Pi의 전원 상태를 표시합니다. 초록 `ACT` LED는 microSD 접근을 표시합니다. SSH 로그인으로 OS 부팅과 네트워크 접속을 확인합니다. [Pi LED 안내](https://www.raspberrypi.com/documentation/computers/configuration.html#led-behaviour) · [SSH 로그인 안내](https://www.raspberrypi.com/documentation/computers/remote-access.html#connect-to-an-ssh-server)

SSH 키 확인 화면에 `ED25519`가 나오면 Pi에 직접 연결한 터미널에서 `ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub`를 실행해 지문을 대조할 수 있습니다.

### 4. 라이다 USB 포트와 프로그램 준비

1. **Pi에 로그인한 터미널**에서 다음 명령을 실행합니다.

   ```bash
   lsusb
   ls -l /dev/serial/by-id/
   python3 -c "import serial"
   ```

2. `lsusb` 출력에서 `Silicon Labs CP210x` 또는 `10c4:ea60`을 찾습니다.
3. 두 번째 명령의 출력에서 `ttyUSB0`과 같은 포트 이름을 확인합니다. `../../ttyUSB0`이 보이면 레이더에 입력할 경로는 `/dev/ttyUSB0`입니다.
4. 세 번째 명령에서 오류가 나오지 않는지 확인합니다. `No module named 'serial'`이 나오면 Pi 터미널에서 `sudo apt update`를 실행한 뒤 `sudo apt install python3-serial`을 실행합니다. [Debian `python3-serial` 패키지](https://packages.debian.org/trixie/python3-serial)
5. Pi 터미널에서 `exit`를 입력해 Windows PowerShell로 돌아옵니다.
6. 이 PC에서 처음 실행하는 경우 **Windows PowerShell**에서 다음 명령을 실행합니다.

   ```powershell
   python -m pip install pyserial paramiko
   ```

### 5. 증상별 확인

SSH와 PC USB 모드에서 데이터가 끊기면 상태 표시줄에 다음 재연결까지의 대기 시간이 나옵니다. 대기 시간은 2초부터 시작해 최대 15초까지 늘어납니다. 연결이 복구되면 새 스캔부터 표시합니다. **정지**를 누르면 재연결 시도도 멈춥니다.

Windows는 이미 연결한 CP210x 어댑터를 식별할 수 있으면 COM 번호가 바뀐 뒤에도 찾습니다. Pi의 기본 포트는 CP210x 어댑터가 하나일 때 새 `ttyUSB` 번호를 찾아 사용합니다. Pi에 CP210x 어댑터가 여러 개라면 `ls -l /dev/serial/by-id/`에서 해당 장치를 찾아 **USB 포트**에 전체 경로를 입력합니다.

장시간 운용할 때는 Pi를 공유기의 LAN 포트에 연결합니다. Pi에는 5 V / 3 A USB-C 전원을 사용합니다. 수집 중 Windows PC가 절전 모드로 들어가지 않도록 합니다.

| 증상 | 확인 방법 |
| --- | --- |
| `Could not resolve hostname` | 공유기의 연결된 기기 목록에서 Pi의 IP를 찾습니다. 모니터와 키보드가 있다면 Pi 터미널에서 `ip -br -4 addr`를 실행합니다. `eth0`은 유선, `wlan0`은 Wi-Fi입니다. 예를 들어 `192.168.1.42/24`가 보이면 Windows에서 `ssh user@192.168.1.42`를 실행합니다. 같은 주소를 레이더의 **SSH 주소**에 입력합니다. |
| `Connection timed out` 또는 `Connection refused` | Pi 전원과 IP를 확인합니다. PC와 Pi를 같은 공유기에 연결합니다. Pi에서 `sudo raspi-config`를 실행합니다. **Interface Options → SSH → Yes**를 선택합니다. |
| Pi가 Wi-Fi에 연결되지 않음 | Pi 터미널에서 `nmcli device status`를 실행합니다. `wlan0`이 `disconnected`이면 Wi-Fi 이름의 대소문자와 비밀번호를 확인합니다. 초기 확인에는 공유기 LAN 포트와 Pi를 랜선으로 연결할 수 있습니다. |
| `Permission denied`가 SSH 로그인 중에 표시됨 | Pi 사용자 이름과 Pi 계정 비밀번호를 확인합니다. Pi에서 SSH 암호 인증을 허용하는지 확인합니다. |
| CP210x나 `/dev/ttyUSB0`이 보이지 않음 | 레이더를 정지합니다. USB 데이터 케이블을 Pi의 다른 USB 포트에 다시 연결합니다. `lsusb`와 `ls -l /dev/serial/by-id/`를 다시 실행합니다. 포트 번호가 바뀌면 새 경로를 입력합니다. |
| USB 포트 사용 중 또는 포트 권한 오류 | 같은 포트를 사용 중인 수집 프로그램을 닫습니다. 권한 오류가 계속되면 Pi 관리자가 해당 계정의 시리얼 포트 접근 권한을 확인하도록 합니다. |
| 수신 숫자는 증가하지만 녹색 점이 없음 | **누적 유효값**도 증가하는지 확인합니다. **표시 반경**을 늘립니다. 센서가 측정하는 높이에 가까운 상자를 둡니다. |

`nmcli`와 `lsusb`는 Pi에서 실행합니다. Windows PowerShell 프롬프트에 `PS C:\...>`가 보이면 먼저 `ssh`로 Pi에 로그인합니다. [Pi IP 주소 찾기](https://www.raspberrypi.com/documentation/computers/remote-access.html#find-the-ip-address-of-your-raspberry-pi)

## English procedure

### 1. Equipment and connections

| Item | Requirement |
| --- | --- |
| RPLIDAR A1M8-R5 | Sensor, supplied connection cable, USB adapter, and USB data cable. |
| Raspberry Pi 4B | Raspberry Pi OS on a microSD card and a 5 V / 3 A USB-C power supply. |
| Windows PC | Python and the `Nano_33_IoT` project folder. |
| Router | One local network for the PC and the Pi. An Ethernet cable for a wired connection. |

The USB data cable connects the sensor adapter to a USB-A port on the Pi. The power supply connects to the USB-C port. The PC receives scan data from the Pi through the local network.

### 2. Prepare the Pi

If the Pi has Raspberry Pi OS and SSH access, go to section 3. SSH gives the PC access to the terminal on the Pi.

1. Open Raspberry Pi Imager on the PC.
2. Select **Raspberry Pi 4**.
3. Select **Raspberry Pi OS**.
4. Select the microSD card.

   **CAUTION**

   Make sure that the selected card is correct. Imager erases the data on the selected card.

5. Set the hostname to `andrea`.
6. Set the Pi username to `user`.
7. Set a password for the Pi account.
8. For a Wi-Fi connection, enter the Wi-Fi name exactly as it appears on the PC.
9. Enter the Wi-Fi password.
10. Set **Enable SSH** to ON in **Remote Access**.
11. Select **Use password authentication**.
12. Write the OS image to the card.
13. When Imager completes the check, remove the card from the PC.

The launcher uses `user@andrea.local` by default. If you select different values, use them in the SSH command and the radar **SSH 주소** field.

### 3. Connect the equipment and open the Pi terminal

1. With the Pi power off, put the microSD card in the Pi.
2. Connect the supplied sensor cable to the USB adapter.
3. Connect the USB data cable from the adapter to a USB-A port on the Pi.
4. For a wired connection, connect an Ethernet cable from the Pi to a LAN port on the router.
5. Connect the PC to the same local network as the Pi.
6. Connect the USB-C power supply to the Pi.
7. Wait for the Pi to start.
8. Enter this command in **Windows PowerShell**.

   ```powershell
   ssh user@andrea.local
   ```

9. If SSH shows a host-key prompt, make sure that the target is the correct Pi.
10. When the target is correct, enter `yes`.
11. Enter the Pi account password.
12. When the prompt shows `user@andrea:~$`, enter `hostname`.
13. Make sure that the response is `andrea`.

SSH keeps the password characters hidden during entry. The red `PWR` light indicates the power condition. The green `ACT` light indicates microSD activity.

If you can use the Pi local terminal, enter `ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub` to show the ED25519 fingerprint. Compare it with the ED25519 fingerprint in the SSH prompt.

### 4. Check the USB port and prepare the software

1. Enter these commands in the **Pi terminal**.

   ```bash
   lsusb
   ls -l /dev/serial/by-id/
   python3 -c "import serial"
   ```

2. Find `Silicon Labs CP210x` or `10c4:ea60` in the `lsusb` output.
3. Find the USB serial port in the second command output.
4. If the output shows `../../ttyUSB0`, use `/dev/ttyUSB0` in the radar window.
5. Make sure that the third command completes without an error.
6. If the Pi shows `No module named 'serial'`, enter `sudo apt update` in the Pi terminal.
7. If the package is missing, enter `sudo apt install python3-serial` in the Pi terminal.
8. Enter `exit` in the Pi terminal.
9. For first use on this PC, enter this command in **Windows PowerShell**.

   ```powershell
   python -m pip install pyserial paramiko
   ```

### 5. Correct common problems

After a loss of data, the radar shows the time until the next connection attempt. This function works with PC USB and SSH. The radar shows new scan data when the connection returns. Select **정지** to stop the connection attempts.

For long runs, connect the Pi to a LAN port on the router. Use a 5 V / 3 A USB-C power supply for the Pi. Keep the Windows PC out of sleep mode during the scan.

| Indication | Action |
| --- | --- |
| `Could not resolve hostname` | Find the Pi IP address in the router device list. With a monitor on the Pi, enter `ip -br -4 addr` in the Pi terminal. `eth0` shows the Ethernet interface. `wlan0` shows the Wi-Fi interface. If the address is `192.168.1.42/24`, enter `ssh user@192.168.1.42` in Windows PowerShell. Use that address in the radar window. |
| `Connection timed out` or `Connection refused` | Check the Pi power. Check the Pi IP address. Connect the PC and the Pi to the same router. Enter `sudo raspi-config` in the Pi terminal. Select **Interface Options**, then **SSH**, then **Yes**. |
| The Pi does not connect to Wi-Fi | Enter `nmcli device status` in the Pi terminal. If `wlan0` shows `disconnected`, check the Wi-Fi name and password. Use an Ethernet cable for the initial check if necessary. |
| SSH reports `Permission denied` during login | Check the Pi username. Check the Pi account password. Make sure that the Pi permits SSH password authentication. |
| The Pi does not show CP210x or `/dev/ttyUSB0` | Stop the scan. Connect the USB data cable to a different USB port on the Pi. Repeat the Pi USB checks in section 4. If the port number changes, use the new port in the radar window. |
| The USB port is in use or access to the port fails | Close other programs that use the port. If access still fails, ask the Pi administrator to check the account permissions for the serial port. |
| The receive count increases and the display shows no points | Check the **누적 유효값** count. Increase **표시 반경**. Put a box near the sensor in its scan plane. |

## 검증 범위

Pi의 USB 포트에서 거리·각도 데이터를 5초간 수신했습니다. 라이다를 다시 연결한 뒤 Windows 레이더의 SSH 모드에서 녹색 점 표시, 유효값 증가, 정지 후 수신 중단, 재시작 후 수신 재개를 확인했습니다. [Windows 레이더 실수신 기록](../../docs/development-log.ko.md#2026-11-19-windows-레이더에서-raspberry-pi-ssh-실수신-확인) · [Pi USB 실수신 기록](../../docs/development-log.ko.md#2026-11-19-raspberry-pi-4b에서-usb-라이다-수집-확인)
