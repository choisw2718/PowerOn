# PowerOn RC카 프로젝트 개요

문서 기준일: **2026-09-30 (한국 시간)**

현재 개발 목표는 **Arduino Nano 33 IoT 한 대가 기존 공유기의 Wi-Fi 명령을 직접 받아 중앙 후륜 DC 모터 1개와 앞바퀴 조향 서보 2개를 제어하는 RC카**입니다. Nano 스케치는 **M.A.I DC MOTOR DRIVER V3.0 (MAI-2MT-DC)의 R 채널**을 사용합니다. 기존 Uno의 L 채널·USB 시리얼 제어 코드는 참고용으로 보존합니다. 실제 차체의 하네스·모터 정격은 아직 실물로 확인되지 않았습니다.

| 단계 | 보드 | 상태 |
|---|---|---|
| 현재 개발·검증 대상 | Arduino Nano 33 IoT | R 채널 펌웨어 업로드와 무동력 Wi-Fi 명령 시험 기록 있음. 재배선·신호 측정·모터/서보 실물 검증 필요 |
| 기존 참고 버전 | Arduino Uno R3 | L 채널 단일 모터 펌웨어와 USB 시리얼 조종 코드 보존. 실차 검증 상태는 별도 확인 필요 |

## 현재 동작과 검증 범위

- 노트북 Python 조종기가 같은 공유기에서 **UDP 5001번으로 자동 검색**, **TCP 5000번으로 제어**합니다. 전진·후진·조향·정지와 `STATUS`/`NETWORK`/`PING` 진단을 지원합니다.
- Nano는 **TCP 연결이 유지되는 동안 마지막 구동 명령을 유지**합니다. `STOP`/`IDLE` 또는 TCP·Wi-Fi 연결 해제 시 정지하도록 구현됐으며, Uno의 1.5초 명령 감시는 Nano에 적용하지 않습니다.
- **2026-09-30 기존 시험 기록:** Arduino SAMD 1.8.14로 R 채널 스케치를 컴파일·업로드하고, 실제 Nano에서 자동 검색·명령 왕복·2초 이상 소프트웨어 출력 유지·`STOP` 처리를 확인했습니다. 모터를 연결하지 않은 시험이며 모터 출력 전압·회전·서보 동작·연결 해제 시 실물 정지는 미검증입니다.
- 현재 신호 배선은 **버퍼·NPN·저항 없는 3.3 V 직결**입니다. 신호 호환성과 모터 정격은 확인이 필요하고, **리셋·USB 전원 손실 중 안전 정지는 보장되지 않습니다.** 현재 구성은 무동력 확인과 조건을 충족한 벤치 시험 대상으로 유지하며 지면 주행에는 사용하지 않습니다.
- 전압·속도 피드백과 엔코더/PID 제어는 없습니다. `VOLTAGE`는 실측 전압이 아닌 PWM 명목값입니다.

## 현재 사용·전환 자료

```text
Nano_33_IoT/
  Nano_33_IoT.ino             현재 Nano 펌웨어
  wifi_control.py            Wi-Fi 키보드 조종기
  README.md                  Nano 사용 안내
  WIRING.ko.md               상세 핀맵·전원·시험 절차
  DEVELOPMENT.md             개발 현황·필요한 정보
  PROJECT_OVERVIEW.md        차량 구성·이전 모터 모델 기록
  wifi_secrets.h             개인 Wi-Fi 설정 (Git 제외)
  reference/
    Uno_Single_Rear_Motor/   기존 Uno 스케치·USB 조종기·배선 안내
    springmaru_poweron_nano33_iot/
      README.md             외부 Wi-Fi 진단 자료 사용·비교 안내
      UPSTREAM.json         원본 커밋과 파일 해시
      upstream/             원본 4개 파일의 고정 사본 (.git 제외)
```

- [개발 현황·추가로 필요한 정보](DEVELOPMENT.md)
- [Nano 33 IoT 사용 안내](README.md)
- [Nano 배선 전 준비·핀별 배선](WIRING.ko.md)
- [기존 Uno 사용·배선 안내](reference/Uno_Single_Rear_Motor/README.md)
- [springmaru Wi-Fi 진단 자료와 Windows 사용 안내](reference/springmaru_poweron_nano33_iot/README.md)

## 작업 폴더 기준

현재 개발 원본은 **`C:\Users\user\Arduino\Nano_33_IoT`의 독립 폴더**입니다. 필요한 최신 코드·문서·개인 Wi-Fi 설정과 Uno 참고 자료를 모두 이곳으로 복제했습니다. Arduino IDE와 Python 조종기는 이 폴더에서 실행합니다.

기존 PowerOn·OneDrive 폴더와 이전 junction, `Nano_33_IoT.before-repo-move-20260928` 백업은 보존합니다. 현재 업로드·배선 기준은 이 폴더의 Nano 문서와 스케치입니다. 개인 `wifi_secrets.h`는 공개 자료와 Git에서 제외합니다.

외부 `springmaru/poweron-nano33-iot`의 Git 원본은 `C:\Users\user\Arduino\external_repos\springmaru\poweron-nano33-iot`에 있습니다. 외부 저장소의 pull과 현재 PowerOn의 push는 각각 그 저장소 폴더에서 수행합니다. PowerOn의 `reference/springmaru_poweron_nano33_iot/upstream`은 기록된 커밋의 고정 사본이므로 외부 pull에 따라 자동으로 변경되지 않습니다.

## 모터 모델 확인 결과

이전 양쪽 후륜 버전 문서에는 `RB-35GM 09TYPE DC 24V W/EC 26P` **2개**가 적혀 있었습니다. 현재 단일 모터 스케치와 문서에는 **12 V 전원과 약 7 V 기동 기준**만 있으며, 중앙 후륜 모터의 정확한 모델명은 확인되지 않았습니다. 따라서 이전 모터의 모델·정격을 현재 모터에 적용하지 않습니다. 실물 라벨, 정격 전압, 정지 전류를 확인한 뒤 전원과 드라이버 적합성을 판단해야 합니다.

Uno 코드는 기존 차량 참고용으로 유지합니다. Nano 전환 시에는 [코드와 매뉴얼에 맞춘 버퍼 없는 핀맵과 전원 분기 절차](WIRING.ko.md)를 따르고, 모터·드라이버 정격과 3.3 V 직결 신호의 동작을 실물에서 확인해야 합니다. 드라이버 매뉴얼의 **5~46 V/채널당 최대 2 A**는 드라이버 한계이며 현재 모터의 정격을 뜻하지 않습니다.
