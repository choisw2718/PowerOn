# PowerOn RC Car

`PowerOn`은 Arduino Uno R3와 STM32 NUCLEO-F103RB를 이용한 RC카 제어 펌웨어와 노트북용 제어 도구를 모아 둔 프로젝트입니다.

## 프로젝트 구성

| 경로 | 설명 | 문서 |
|---|---|---|
| `STM32_F103_RC_Car/` | NUCLEO-F103RB 기반 메인 주행 펌웨어 | [사용 가이드](STM32_F103_RC_Car/README.md) |
| `STM32_F103_Encoder_Test/` | 우측 엔코더 단독 확인용 펌웨어 | [테스트 가이드](STM32_F103_Encoder_Test/README.md) |
| `STM32_F103_Right_Rear_Test/` | 우측 후륜 모터·엔코더 단독 시험 펌웨어 | [테스트 가이드](STM32_F103_Right_Rear_Test/README.md) |
| `Arduino_Yun_RC_Car/` | Arduino Uno R3 기반 양쪽 후륜 제어 펌웨어 | [사용 가이드](Arduino_Yun_RC_Car/README.md) |
| `Arduino_Yun_RC_Car/Uno_Single_Rear_Motor/` | 단일 중앙 후륜 모터용 Arduino 펌웨어 | [사용 가이드](Arduino_Yun_RC_Car/Uno_Single_Rear_Motor/README.md) |
| `tools/` | 빌드, 플래싱, 시리얼 제어 보조 도구 | 아래 빠른 시작 참고 |

> `Arduino_Yun_RC_Car`는 이전 작업에서 유지된 폴더명입니다. 현재 해당 구현의 대상 보드는 Arduino Yún이 아니라 클래식 Arduino Uno R3입니다.

## 빠른 시작

STM32 메인 펌웨어 빌드:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build_f103_project.ps1
```

STM32 메인 펌웨어 플래싱:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\flash_f103_project.ps1
```

노트북에서 시리얼 제어:

```powershell
python .\tools\keyboard_drive.py
```

각 펌웨어의 배선, 안전 절차, 명령 형식은 해당 폴더의 `README.md`를 먼저 확인하세요.

## 저장소 정책

개인 환경 정보, 실차 보정값, 상세 하드웨어 사양, 생성된 펌웨어 바이너리와 컴파일 산출물은 공개 저장소에서 제외합니다. 관련 로컬 파일은 `.gitignore` 규칙으로 관리하며 삭제하지 않습니다.
