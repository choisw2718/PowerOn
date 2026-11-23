# 작업 기준

- 메인 작업 폴더는 `C:\Users\user\Arduino\Nano_33_IoT`입니다. `vehicle/firmware/PowerOnNano/PowerOnNano.ino`를 개발·업로드 기준으로 사용합니다. PC 조종기는 저장소 루트에서 `python -m vehicle`로 실행합니다.
- 차량은 `vehicle/`, 라이다는 `lidar/`, 시뮬레이션은 `simulation/`의 기능별 코드와 안내를 사용합니다. 전체 테스트는 `python tests/run_all.py`로 실행합니다.
- `README.md`, `docs/development-log.ko.md`와 각 기능의 안내서에 현재 방향, 검증 범위와 배선을 기록합니다. 새 실물 시험을 완료했을 때만 날짜·조건·결과와 완료 표시를 갱신합니다.
- 부품과 장착 상태는 `hardware/`에 기록합니다. 공용 `hardware/control_profile.json`을 변경한 뒤 `python -m hardware.generate_firmware_profile`로 Arduino용 헤더를 갱신합니다.
- `reference/Uno_Single_Rear_Motor`는 기존 Uno 참고 자료입니다. 이전 PowerOn·OneDrive 사본과 과거 백업을 메인 작업 대상으로 삼지 않습니다.
- `vehicle/firmware/PowerOnNano/wifi_secrets.h`는 개인 로컬 설정이며 Git이나 공개 자료에 포함하지 않습니다.

## 사용자 파일 삭제 지침

다량의 파일을 삭제할 때는 다시 한번 검토하는 과정이 필요해.

너가 만든 파일이 아니라면 파일을 삭제하기 전에 보수적으로 검토해.
