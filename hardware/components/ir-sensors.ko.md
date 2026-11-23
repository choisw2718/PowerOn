# SHARP 적외선 거리 센서

**장착 상태 · 3개 모두 미장착, 장착 계획 있음.** 최신 사용자 설명을 기준으로 기록했습니다. 오른쪽 A2에 센서를 연결해 ADC를 읽은 기록은 별도 시험 이력입니다.

| 장착 계획 | 핀 | 현재 상태 |
| --- | --- | --- |
| 왼쪽 측면 | A0 | 미장착 |
| 전방 | A1 | 미장착 |
| 오른쪽 측면 | A2 | 미장착 · 과거 ADC 시험 이력 |

- **모델** · SHARP GP2Y0A21YK0F. 제조사 표기 측정 범위 10–80 cm, 공급 전압 4.5–5.5 V, 비선형 아날로그 출력. [SHARP 데이터시트](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a21yk_e.pdf)
- **설정** · [공용 프로필](../control_profile.json)의 `ir_sensors` 세 항목이 모두 `false`입니다. 펌웨어는 해당 채널의 ADC를 읽지 않고 `not_connected`를 표시합니다.
- **측정 필요** · 센서별 높이와 방향, 차량 중심에서 좌우 오프셋, 벽 재질별 신호, 입력 전압과 거리 보정 곡선. 장착과 배선 확인 후 해당 채널 설정을 갱신합니다.

[센서 배선·시험 안내](../../vehicle/sensor_monitor/guide.ko.md) · [이전 A2 응답 기록](../../docs/development-log.ko.md#2026-11-10-a2-한-개-연결-확인과-미연결-채널-제외)
