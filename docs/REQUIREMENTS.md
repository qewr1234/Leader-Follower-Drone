# 요구도 및 추적성 (2026-09-18)

리더-팔로워 컴패니언 소프트웨어의 요구도를 ID 로 정리하고, 각 요구도가 어느 검사·시나리오·분석으로 검증되는지
표로 묶었습니다. 표의 참조는 `python3 analysis/trace_check.py` 가 실제 코드와 대조하며(`test_fixes.py` 의
`추적성:` 검사로도 돌아감), 깨진 참조가 있으면 실패합니다.

## 표기

| 항목 | 값 |
|---|---|
| ID 접두어 | **FCR** 비행제어 기능 · **EST** 인지/추정 · **SAF** 안전 · **IF** 인터페이스 · **OPS** 운용/유지 |
| 방법 | **A** 분석(선형 모델·시뮬레이션) · **T** 시험(단위/폐루프/SITL) · **I** 검사(코드 확인) · **D** 시연(실기) |
| 검증 근거 토큰 | `UT[접두어]` `test_fixes.py` 검사 이름 접두어 · `CL[접두어]` `test_closed_loop.py` 검사 · `SITL[이름]` `sitl/harness.py` 시나리오 · `AN[키]` `docs/stability_margins.json` 키 · `INSPECT[파일:문자열]` 소스 문자열 · `HW` 개발자 하드웨어 보고 · `—` 없음 |
| 상태 | **검증됨** · **부분**(근거는 있으나 조건 일부만) · **미검증** · **미충족**(검증했더니 요구를 만족하지 않음) |

시험 층은 [VERIFICATION.md](../VERIFICATION.md) 의 4층(순수 함수 → `main.main()` 폐루프 → MAVLink 와이어 →
ArduCopter SITL)이고, 분석 층은 [STABILITY_MARGINS.md](STABILITY_MARGINS.md) 입니다.

## 1. 요구도

### FCR — 비행제어 기능

| ID | 요구도 | 출처 | 방법 | 검증 근거 | 상태 |
|---|---|---|---|---|---|
| FCR-01 | 팔로워는 리더 후방 목표 이격 3.0 m(`TARGET_DISTANCE_M`)를 유지하며, 리더 정지 후 목표 ±0.5 m 로 수렴하고 2.3 m 안쪽으로 접근하지 않는다 | README 제어 법칙 | T | CL[리더 정지 후 최소 접근]; SITL[hover_hold] | 검증됨 |
| FCR-02 | 리더 등속 0.3 m/s 추종 시 정상상태 거리 오차가 이론값 (1−KFF)·v/Kp 와 ±0.05 m 로 일치하고 평형 거리가 깊이창(목표+3 m) 안에 든다 | README 제어 법칙 | A, T | UT[FF: 1축 폐루프]; CL[FOLLOW 중(리더]; SITL[depth_range]; AN[kff_sweep.kff0.8.ss_err_per_mps] | 검증됨 |
| FCR-03 | 속도 명령은 BODY_NED 프레임으로 축별 한계(0.35 / 0.22 / 0.12 m/s, yaw 0.35 rad/s)에 포화되고 yaw_rate 마스크를 항상 유효로 보낸다 | main.py | T, I | UT[C5:]; INSPECT[main.py:MAX_VX = 0.35] | 검증됨 |
| FCR-04 | 속도 setpoint 는 10 Hz(9.5~10.5 Hz) 로 송신한다 | main.py `SETPOINT_PERIOD_SEC` | T | CL[setpoint 송신율] | 검증됨 |
| FCR-05 | 기수는 리더 방위각을 0 으로 유지하고, 명령 yaw_rate 0 에서 기체가 스스로 회전하지 않는다 | VERIFICATION C5 | T, A | UT[C5:]; SITL[hold_heading]; AN[yaw.nominal.pm_deg] | 검증됨 |
| FCR-06 | 명령 평활은 프레임률과 무관하게 같은 시정수를 갖는다 | main.py `smooth_velocity_cmd` | T | UT[평활:] | 검증됨 |
| FCR-07 | 위치 공분산 trace 가 2 / 4 를 넘으면 명령을 0.75 / 0.55 배로 줄인다 | config `controller.uncertainty_slowdown_trace` | A | AN[kff_sweep.scale0.55.gm_db] | 부분 (여유 분석만, 배율 단위 검사 없음) |
| FCR-08 | 리더 절대 속도(자기 속도 + EKF 상대 속도)를 KFF 0.8 로 피드포워드하되 0.1 m/s 데드밴드와 1차 저역통과를 거치고, 인자가 없으면 기존 명령과 같다 | README 제어 법칙 | T | UT[FF:]; SITL[depth_range] | 검증됨 |
| FCR-09 | 바깥 루프는 전 축에서 위상여유 ≥ 45°, 이득여유 ≥ 6 dB, 감도 피크 Ms ≤ 2 를 만족한다 | STABILITY_MARGINS 1·4절 | A | AN[axes.forward.pm_deg]; AN[axes.forward.gm_db]; AN[axes.right.gm_db]; UT[분석 골든:] | **미충족** (GM 4.9 / 4.4 / 5.3 dB, Ms 2.3~2.6. 개선안 7절) |
| FCR-10 | 리더→팔로워 속도 전달 \|Γ(jω)\| 이 모든 주파수에서 1 이하다 (스트링 안정, 다중 기체 체인 전제) | STABILITY_MARGINS 6절 | A | AN[axes.forward.peak]; AN[validation]; UT[분석 골든:] | **미충족** (피크 1.80 @1.15 rad/s, 비선형 체인 1.86) |
| FCR-11 | FC ATTITUDE 의 roll/pitch/yaw 변화량으로 매 프레임 EKF 상대 상태를 역회전해 기체 기울어짐이 리더 이동으로 보이지 않게 하고, 그 보정이 CT 각속도로 새지 않는다 | README 안전 설계 | T | UT[자세보정:]; UT[ego-yaw:] | 검증됨 (실기 미검증) |
| FCR-12 | 비전 거리가 없고 GPS 상대위치만 있으면 이격을 8 m 로 넓힌다 | README 안전 설계 | T | UT[gps-only:] | 검증됨 |
| FCR-13 | 출발·정지·착륙 판단은 리더 절대 속도(ESP32 > 자기+상대 > 상대 폴백) 로 한다 | README 안전 설계 | T | UT[미션:]; CL[리더 출발 후]; CL[추종 중(t=8s)]; SITL[depth_range] | 검증됨 |
| FCR-14 | 피드포워드를 뺀 P+D 기준선은 PM ≥ 45°, GM ≥ 6 dB, Ms ≤ 1.5, 스트링 안정을 만족한다 | STABILITY_MARGINS 1절 | A | AN[kff_sweep.kff0.gm_db]; UT[분석: P+D] | 검증됨 (분석) |
| FCR-15 | 개선안(피드포워드 저역통과 2.0 s, 자기 속도 정합 저역통과 0.3 s)은 GM ≥ 12 dB, Ms ≤ 1.35, \|Γ\| 피크 ≤ 1.15 를 만족한다 | STABILITY_MARGINS 7절 | A | AN[proposed.tau_ff2.0_tau_m0.3.gm_db]; UT[분석: 권고안] | 검증됨 (분석. 코드 미적용) |

### EST — 인지 / 추정

| ID | 요구도 | 출처 | 방법 | 검증 근거 | 상태 |
|---|---|---|---|---|---|
| EST-01 | IMM-EKF(CV+CT) 가 상대 위치·속도를 추정하고 속도 추정의 63 % 응답이 0.5 s 이내이며 램프에 위치 지연이 없다 | imm_ekf.py | A, T | AN[ekf_step.t63_velocity_s]; UT[분석: IMM-EKF] | 검증됨 (0.30 s) |
| EST-02 | 측정은 신뢰도로 R 을 적응하고 카이제곱 게이트(3D 11.34, 2D 9.21)로 거른다 | config `reliability` | T | UT[C4: MAD]; UT[C4: 정상] | 부분 (게이트 임계 자체의 단위 검사 없음) |
| EST-03 | 거리 측정이 끊기면 2 s 코스트 후 소실로 판정하고, bearing-only 는 거리 확보로 치지 않는다 | config `imm.range_coast_max_sec` | T, I | UT[rcoast:]; INSPECT[config.py:"range_coast_max_sec": 2.0] | 검증됨 |
| EST-04 | 트래커는 1프레임 소실 뒤 화면 반대편 검출을 거부하고 max_lost 뒤 새 track_id 로 재초기화한다 | VERIFICATION 트래커 신원 게이트 | T | UT[tracker:] | 검증됨 |
| EST-05 | 깊이 측정은 유효 화소 수·비율·MAD 한계로 걸러 깊이 절벽을 신뢰도 0 으로 만들고, 큰 ROI 서브샘플 오차는 1 cm 미만이다 | VERIFICATION C4 | T | UT[C4:]; UT[measurement:] | 검증됨 |
| EST-06 | 검출을 건너뛴 프레임의 측정은 신뢰도를 0.55 배로 깎는다 | reliability.py | T | UT[skip:] | 검증됨 |
| EST-07 | 검출기는 대상 클래스만 남겨 (conf, area) 순으로 정렬하고 ROI 오프셋을 적용하며 conf/iou/imgsz 를 매 호출 전달한다. 모델에 없는 클래스면 시작 시 경고한다 | detector.py | T | UT[detector:]; UT[검출:] | 검증됨 |
| EST-08 | 스케줄러는 안정 상태에서 `normal_detect_every` 주기로 검출한다 | scheduler.py | T | UT[scheduler:] | 검증됨 |
| EST-09 | EKF 융합 상태는 캐시되고 상태 변경 시 무효화된다 | imm_ekf `get_state` | T | UT[ekf:] | 검증됨 |
| EST-10 | 컬러 센서에 AE priority off·노출 상한 8 ms·역광 보정을 적용하고, AE 측광 ROI 를 추적 bbox 1.5배로 따라간다(≤1 Hz, 10 % 이동, 소실 시 전체 복귀) | README 안전 설계 | T | UT[camera:] | 부분 (pyrealsense2 스텁 검증, 실외 실기 미검증) |
| EST-11 | 카메라 파이프라인은 Jetson 에서 24 FPS 이상이다 | VERIFICATION 하드웨어 | D | HW | 부분 (개발자 보고 24~26 FPS, 로그 없음) |
| EST-12 | 리더 검출률은 80 % 이상이다 | VERIFICATION 하드웨어 | D | HW | **미충족** (약 60 %, 데이터셋 확장 필요) |

### SAF — 안전

| ID | 요구도 | 출처 | 방법 | 검증 근거 | 상태 |
|---|---|---|---|---|---|
| SAF-01 | FC 모드가 GUIDED/OFFBOARD 가 아니면 모드 변경·LAND 명령을 보내지 않는다(조종사 우선) | VERIFICATION C2 | T | SITL[pilot_takeover] | 검증됨 |
| SAF-02 | 리더를 한 번도 획득하지 않은 상태에서는 LAND 하지 않는다 | VERIFICATION C1 | T | UT[C1:]; SITL[boot_no_leader] | 검증됨 |
| SAF-03 | 리더 소실 시 호버(LOST_HOLD) 로 버티다 총 10 s(코스트 2 + 홀드 8)에 LAND 하며, 깊이만 죽고 검출이 살아 있는 경우도 같다 | README 안전 설계 | T, I | UT[소실 후 착륙까지]; CL[4초 소실]; CL[영구 소실]; SITL[depth_loss]; INSPECT[mission_manager.py:lost_hold_sec=8.0] | 검증됨 |
| SAF-04 | 절대 고도 없이 공중에서 착륙 판정을 내지 않고, 착륙 판정은 리더 절대 하강 속도로 한다 | VERIFICATION C3 | T | UT[C3:]; UT[미션: 착륙 판정도]; SITL[air_landing] | 검증됨 |
| SAF-05 | GUIDED 진입 순간 미션·명령·피드포워드 상태를 리셋하고 출발 확인을 다시 요구한다 | README 안전 설계 | T | UT[재개: reset()]; CL[GUIDED 진입 후]; SITL[handover] | 검증됨 |
| SAF-06 | AGL 1.5 m 아래에서는 하강 명령을 차단한다 | main.py `MIN_AGL_M` | I | UT[AGL 바닥]; INSPECT[main.py:MIN_AGL_M = 1.5] | 부분 (상수·분기 존재만, SITL 시나리오 없음) |
| SAF-07 | 기본값은 dry-run(명령 미송신) 이다 | main.py | I | INSPECT[main.py:SEND_MAVLINK_COMMANDS = False] | 검증됨 |
| SAF-08 | PX4 의 3-튜플 mode_mapping 에서도 set_mode 가 예외 없이 동작하고 실패해도 루프가 죽지 않는다 | VERIFICATION C6 | T | UT[C6:]; SITL[px4_setmode] | 검증됨 |
| SAF-09 | 재획득 시 착륙 확인 타이머를 새로 시작한다(가려진 시간 합산 금지) | mission_manager.py | T | UT[타이머:] | 검증됨 |
| SAF-10 | 카메라·FC 링크가 30 회 연속 실패하면 HOLD 후 종료한다 | main.py | I | UT[카메라 연속 실패] | 부분 (상수 존재만) |
| SAF-11 | 잠깐 놓친 뒤 재획득하면 출발 확인 없이 추종을 재개하되 FAILSAFE_LAND 뒤에는 자동 재개하지 않는다 | VERIFICATION 재획득 | T | UT[재개:]; CL[재검출] | 검증됨 |
| SAF-12 | 리더 정지 시 FOLLOW↔LEADER_HOVER 히스테리시스(0.18 / 0.25 m/s) 로 정위치를 유지한다 | VERIFICATION H2 | T | UT[H2:]; CL[리더 정지(11s)]; CL[리더 정지 후 소실]; SITL[hover_hold] | 검증됨 |
| SAF-13 | 공분산 폭주 중에는 LOST_HOLD 이고 회복 첫 프레임에 착륙 명령이 나가지 않는다 | mission_manager.py | T | UT[타이머: 공분산] | 검증됨 |
| SAF-14 | LAND 는 FC 가 GUIDED 안에 있을 때만 2 s 간격으로 재시도하고, 먹으면 FC 가 하강한다 | main.py `LAND_RETRY_SEC` | T | SITL[pilot_takeover]; CL[LAND 이후] | 검증됨 |
| SAF-15 | 위 안전 동작이 폐루프 실비행(실제 공력·바람·프롭워시)에서 유지된다 | VERIFICATION 미검증 | D | — | 미검증 |

### IF — 인터페이스

| ID | 요구도 | 출처 | 방법 | 검증 근거 | 상태 |
|---|---|---|---|---|---|
| IF-01 | FC heartbeat 는 같은 시스템의 autopilot 컴포넌트만 인정하고 GCS/짐벌/INVALID 는 무시하며, pymavlink 가 target_component 0 을 주면 1 로 고정한다 | VERIFICATION 2026-09-18 | T | UT[HB:] | 검증됨 (SITL 실측) |
| IF-02 | FC 연결은 heartbeat 타임아웃을 견디고 필요한 데이터 스트림 3개만 요청한다 | mavlink_io `connect_fc` | T | UT[FC:] | 검증됨 |
| IF-03 | STAT 에 HEARTBEAT / LOCAL_POSITION / ATTITUDE / GLOBAL_POSITION 수신율을 표시한다 | mavlink_io `stream_rates_text` | T | UT[rx:] | 검증됨 |
| IF-04 | ESP32 시리얼의 잘린 라인을 버리지 않고 다음 읽기에서 복원한다 | leader_telemetry.py | T | UT[serial:] | 검증됨 |
| IF-05 | ESP32 장치나 pyserial 이 없어도 비전 단독으로 기동한다 | main.py `open_leader_receiver` | T | UT[ESP32:] | 검증됨 |
| IF-06 | 리더 고도 기준계(AMSL / 타원체)를 팔로워의 같은 기준계와만 뺀다 | leader_telemetry.py | T | UT[alt:]; UT[lla:] | 검증됨 |
| IF-07 | 헤드리스(`MARS_SHOW_WINDOW=0`) 로 동작한다 | main.py | T | UT[헤드리스:] | 검증됨 |
| IF-08 | LOCAL_POSITION_NED 0.4 s, ATTITUDE 0.3 s, GLOBAL_POSITION 0.7 s 이내의 값만 신선한 것으로 쓴다 | main.py `*_MAX_AGE_SEC` | I, T | INSPECT[main.py:LOCAL_POS_MAX_AGE_SEC = 0.40]; SITL[depth_range] | 검증됨 (STAT `fresh=LP1/ATT1` 실측) |
| IF-09 | 배터리 전압 미보고(65535)는 전압으로 쓰지 않는다 | mavlink_io `battery_text` | T | UT[BAT:] | 검증됨 |
| IF-10 | 실험 로그는 평탄화된 행으로 기록된다 | logger.py | T | UT[logger:] | 검증됨 |
| IF-11 | ESP32(ESP-NOW) 송신 펌웨어가 리더 절대 위치·속도를 방송한다 | README 시스템 개요 | D | — | 미검증 (펌웨어 미존재) |

### OPS — 운용 / 유지

| ID | 요구도 | 출처 | 방법 | 검증 근거 | 상태 |
|---|---|---|---|---|---|
| OPS-01 | SITL 하네스 `--all` 은 ArduCopter 로 도는 8개 시나리오를 모두 포함한다 | sitl/README | T | UT[하네스:] | 검증됨 |
| OPS-02 | 모터 테스트 프로토타입 등 죽은 코드가 없다 | VERIFICATION | I | UT[정리:] | 검증됨 |
| OPS-03 | 모든 요구도의 검증 근거가 실제 검사·시나리오·분석 키를 가리킨다 | 이 문서 | T | UT[추적성:] | 검증됨 |
| OPS-04 | `main.main()` 은 가짜 FC·가짜 시계로 결정론 실행되어 예외 없이 끝난다 | test_closed_loop.py | T | CL[main.main()] | 검증됨 |

## 2. 역추적: SITL 시나리오 → 요구도

| 시나리오 | 요구도 | 실패 조건 |
|---|---|---|
| `boot_no_leader` | SAF-02 | 리더를 한 번도 못 봤는데 LAND |
| `pilot_takeover` | SAF-01, SAF-14 | 조종사 LOITER 탈환이 LAND 로 덮어써짐 |
| `air_landing` | SAF-04 | 리더가 보이는데 공중에서 착륙 판정 |
| `depth_range` | FCR-02, FCR-08, FCR-13, IF-08 | 0.3 m/s 리더를 못 따라붙음(후반 거리 > 목표+3 m) |
| `hover_hold` | FCR-01, SAF-12 | 리더 정지 후 목표 거리로 수렴 못 함(오차 > 1.5 m) |
| `hold_heading` | FCR-05 | 명령 yaw_rate 0 인데 기수 25° 이상 자체 회전 |
| `px4_setmode` | SAF-08 | PX4 3-튜플 mode_mapping 에서 set_mode 예외 |
| `depth_loss` | SAF-03 | 깊이만 죽었을 때 10 s 뒤 착륙 안 함 |
| `handover` | SAF-05 | GUIDED 인계 순간 LAND |

## 3. 미충족 · 미검증 · 부분 요구도

| ID | 상태 | 필요한 것 |
|---|---|---|
| FCR-09 | 미충족 | 피드포워드 저역통과 2.0 s + 자기 속도 정합 저역통과 0.3 s + 소프트 데드존 적용(STABILITY_MARGINS 7절). 적용 전 SITL 에 정현파 리더(1.15 rad/s) 시나리오를 추가해 증폭을 재현 |
| FCR-10 | 미충족 | 위와 같음. 3대 이상 체인이면 KFF 0.6 또는 ESP32 선두 절대 속도 방송 |
| EST-12 | 미충족 | 리더 드론 데이터셋 확장, 재학습 |
| FCR-07 | 부분 | 감속 배율 0.75 / 0.55 의 단위 검사 추가 |
| EST-02 | 부분 | 카이제곱 임계 경계값 단위 검사 추가 |
| EST-10 | 부분 | 실외 역광·강한 빛 조건에서 노출 옵션 실기 확인 |
| EST-11 | 부분 | Jetson FPS 로그를 저장소에 남기기 |
| SAF-06 | 부분 | AGL 바닥 차단 SITL 시나리오(저고도에서 하강 명령) |
| SAF-10 | 부분 | 카메라 스톨 30 회 → HOLD → 종료 폐루프 시나리오 |
| SAF-15 | 미검증 | 폐루프 실비행(안전줄·저고도부터) |
| IF-11 | 미검증 | ESP32 펌웨어 작성, 패킷 포맷 고정 |

## 4. 검사 방법

```bash
python3 analysis/trace_check.py     # 표의 참조가 전부 존재하는지 + 요구도에 안 묶인 검사 목록
python3 test_fixes.py               # '추적성:' 검사로 같은 것을 회귀에 포함
```
