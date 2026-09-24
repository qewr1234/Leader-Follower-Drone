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
| FCR-08 | 리더 절대 속도(0.3 s 정합 저역통과한 자기 속도 + EKF 상대 속도)를 KFF 0.8 로 피드포워드하되 0.05 m/s 소프트 데드존(기울기 1)과 2.0 s 저역통과를 거치고, 인자가 없으면 기존 명령과 같다 | README 제어 법칙, STABILITY_MARGINS 7절 | T | UT[FF:]; CL[FOLLOW 중(리더]; SITL[depth_range]; SITL[leader_sine] | 검증됨 (SITL 재실행: depth_range 후반 4.1 m, leader_sine 0.72) |
| FCR-09 | 바깥 루프는 전 축에서 위상여유 ≥ 45°, 이득여유 ≥ 6 dB, 감도 피크 Ms ≤ 2 를 만족한다 | STABILITY_MARGINS 1·4절 | A | AN[axes.forward.pm_deg]; AN[axes.forward.gm_db]; AN[axes.right.gm_db]; UT[분석: 현재 설계]; UT[분석 골든:] | 검증됨 (분석: GM 14.4 / 13.3 / 14.9 dB, Ms ≤ 1.35. 수정 전은 4.9 / 4.4 / 5.3 dB 로 미충족이었음 — 골든 검사가 그 값을 기록) |
| FCR-10 | 리더→팔로워 속도 전달 \|Γ(jω)\| 이 모든 주파수에서 1 이하다 (스트링 안정, 다중 기체 체인 전제) | STABILITY_MARGINS 6절 | A | AN[axes.forward.peak]; AN[validation]; UT[분석: 현재 설계] | 부분 (피크 1.13 @0.27 rad/s — 피드포워드+P 겹침. 수정 전 1.80 @1.15. KFF 0.6 이면 1.02, 체인 운용 전 결정) |
| FCR-11 | FC ATTITUDE 의 roll/pitch/yaw 변화량으로 매 프레임 EKF 상대 상태를 역회전해 기체 기울어짐이 리더 이동으로 보이지 않게 하고, 그 보정이 CT 각속도로 새지 않는다 | README 안전 설계 | T | UT[자세보정:]; UT[ego-yaw:] | 검증됨 (실기 미검증) |
| FCR-12 | 비전 거리가 없고 GPS 상대위치만 있으면 이격을 8 m 로 넓힌다 | README 안전 설계 | T | UT[gps-only:] | 검증됨 |
| FCR-13 | 출발·정지·착륙 판단은 리더 절대 속도(ESP32 > 자기+상대 > 상대 폴백) 로 한다 | README 안전 설계 | T | UT[미션:]; CL[리더 출발 후]; CL[추종 중(t=8s)]; SITL[depth_range] | 검증됨 |
| FCR-14 | 피드포워드를 뺀 P+D 기준선은 PM ≥ 45°, GM ≥ 6 dB, Ms ≤ 1.5, 스트링 안정을 만족한다 | STABILITY_MARGINS 1절 | A | AN[kff_sweep.kff0.gm_db]; UT[분석: P+D] | 검증됨 (분석) |
| FCR-15 | 리더 속도가 0.25 ± 0.05 m/s, 1.15 rad/s 정현파일 때 팔로워 속도 진폭비가 1 이하다 (실제 FC 에서의 스트링 안정성) | STABILITY_MARGINS 6절, sitl/README | T | SITL[leader_sine]; AN[sitl_like.current] | 검증됨 (SITL 실측 0.43 / 0.72, 예측 0.70. 수정 전 코드 대조군 1.95 FAIL — 차등 검증) |
| FCR-16 | 편대 슬롯: 슬롯 미설정이면 위치 오차가 기존 (front−TARGET, right, up) 과 비트 단위로 같고, 리더 heading 기준 / NED 슬롯은 후미 FRU 로 회전되며, 비전 거리 없이 GPS 뿐이면 슬롯 방향을 유지한 채 이격을 `TARGET_DISTANCE_GPS_ONLY_M` 으로 늘린다 | MULTI_FOLLOWER_FOUNDATION 4절 | T | UT[편대: 기본 LOS]; UT[편대: 리더 heading 기준]; UT[편대: 상대 heading 0]; UT[편대: NED]; UT[편대: 비전 거리]; CL[main.main()] | 검증됨 (단위; 폐루프 스트림은 변경 전 `--dump` 와 `--compare` 로 동일 확인) |
| FCR-17 | 리더 상대 heading 은 방송 yaw > 리더 속도 방향(≥ `heading_min_speed_mps`) > 최근값 유지(`heading_hold_sec`) > 없음 순으로 정하고, 없으면 슬롯을 같은 거리의 LOS 후방으로 강등하며 그 사실을 알린다 | formation.py | T | UT[편대: 상대 heading 소스]; UT[편대: 상대 heading 0] | 검증됨 |
| FCR-18 | 선두 속도 방송 토폴로지(`ff_source="broadcast"`)에서 리더→n 단 누적 속도 이득이 단 수에 따라 커지지 않고(3단 ≤ 1.15), 선행기 추종보다 작으며, KFF 1.0 도 안정하다 | MULTI_FOLLOWER_FOUNDATION 3.2·4.3 (Seiler 2004, Zheng 2016) | A | UT[편대: 체인 토폴로지]; UT[편대: 선두 속도 방송이면] | 검증됨 (실제 코드 체인 시뮬 — SITL 다기체 시나리오 없음) |
| FCR-19 | 제어 오차는 FC 의 BODY_NED 해석(yaw 만 회전, z 불변)과 같은 수평 프레임이어야 한다 — 기체 기울기(roll/pitch)가 같은 고도 리더에 상하·좌우 명령을 만들지 않는다 | MULTI_FOLLOWER_FOUNDATION 2.2 A1, FLIGHT_SAFETY_CHECKLIST 3절 | T | UT[수평화:]; CL[안전(tilt, 수평화 ON)]; CL[안전(tilt, 수평화 OFF] | 검증됨 (2026-09-24 기본값 True. 폐루프 tilt 시나리오: OFF 면 pitch −10° 에 vz −0.092, 팔로워가 D·tan10° = 0.52 m 위로 올라가 정착 / ON 이면 \|vz\| < 0.01, Δalt 0.005 m. 실기 지상 기울임 점검 필요) |
| FCR-20 | 선두 방송 토폴로지에서 인접 단 교란 전달 ‖T‖∞ ≤ 1 (L2 스트링 안정)이고 리더→i 단 속도 이득 ‖Γ_i‖∞ 가 i 에 대해 균일 유계이며 꼬리가 ‖P·B_d‖∞ ≤ KFF 로 수렴한다 (정리 1). 선행기 추종은 Γ_1^i 로 발산 | FORMATION_THEORY 2절 | A | UT[이론: 정리 1] | 검증됨 (선형 모델 + 실제 코드 4단 체인 시뮬 ≤ 10 %; 실기 미검증) |
| FCR-21 | 방송이 없을 때 시간간격 정책 D0 + h·v_F 로 스트링 안정을 회복하는 최소 h 가 계산되어 있다 (KFF 0.8: 1.22 s, 추가 이격 0.37 m @0.3 m/s; KFF 1.0: 2.12 s) (정리 2) | FORMATION_THEORY 3절 | A | UT[이론: 정리 2] | 검증됨 (분석; 코드에는 미구현 — 정책 선택은 운용 결정) |
| FCR-22 | 명령 포화·불확실성 감속(0.55/0.75)·추종 이득 전환(섹터 [δ,1], δ>0)에 대해 외루프가 절대안정하다 — 원판 판별법 min Re L(jω) > −1, 현재 설계 여유 0.76 (정리 3). 소프트 데드존은 유계 외란(정상상태 KFF·DB/Kp = 0.18 m) | FORMATION_THEORY 4절 | A | UT[이론: 정리 3]; UT[이론: 대신호] | 부분 (SISO 축별·수치 인증서; 추종/정지(이득 0) 전환은 dwell-time 논거 미완) |

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
| EST-13 | 깊이 측정은 bbox 안쪽 영역의 **가장 가까운 깊이 무리**(0.10 m 빈 히스토그램, 시작 문턱 max(5 %, 30 px)) 의 중앙값이다 — 속이 빈 기체는 안쪽 영역의 절반 이상이 배경이라 단순 중앙값이 배경(8 m) 을 잡아 "멀다" 고 전속 전진했다(χ² 게이트는 5 s 뒤 그 값을 받아들임). 단단한 표적(사람)은 중앙값과 동일 | FLIGHT_SAFETY_CHECKLIST 4절 P1 (DS-KCF, Hannuna·Camplani 2016) | T | UT[안전: nearest-mode 깊이]; CL[main.main()] | 검증됨 (합성 깊이; 실기 드론 표적 미검증) |
| EST-14 | 리더 텔레메트리의 Infinity/NaN/범위 밖 수치(1e999)와 20 m/s 초과 속도 패킷은 버리고, EKF 속도 힌트도 20 m/s 를 넘으면 적용하지 않는다 | leader_telemetry.py | T | UT[안전: parse_leader_json]; UT[안전: 속도 힌트 상한] | 검증됨 |
| EST-15 | RGB-D 측정 R 의 z 분산은 스테레오 오차 σ_z = max(0.25, 0.006·z²) 로 거리에 따라 커진다(8 m 0.38, 10 m 0.6 m) — 원거리 깊이를 과신하지 않는다. bbox 가 프레임 가장자리에 잘리면(`truncated`) 비전 신뢰도 ×0.6 | reliability.py | T | UT[안전: 스테레오 R] | 검증됨 (단위; 골든 무영향 — 리더 ≤ 6.5 m) |
| EST-16 | 트래커 회복(lost_count>0) 은 근접 반경에 더해 bbox 대각선 비 1/1.3~1.3 을 요구한다 — 한 프레임 놓친 사이 다른 거리의 사람이 트랙을 가져가지 못한다. 초기 획득(`_choose_initial` 면적×신뢰도 최대)은 그대로라 **시야에 리더 외 사람이 없어야 한다** | tracker.py, FLIGHT_SAFETY_CHECKLIST 4절 P2 | T | UT[안전: 트래커 크기 게이트]; UT[tracker:] | 검증됨 (운영 규칙 병행) |
| EST-17 | EKF 투영이 화면 밖이면 스케줄러는 전체 프레임(off_image) 이고, 검출기는 폭·높이 < 2 px 퇴화 검출을 버린다 | scheduler.py, detector.py | T | UT[안전: 스케줄러] | 검증됨 |
| EST-18 | ATTITUDE 는 MAV_CMD_SET_MESSAGE_INTERVAL 로 30 Hz 를 요청하고(10 Hz 면 돌풍 pitch ±10°/0.7 Hz 에 0.15~0.39 m 위치·최대 0.58 m/s 가짜 리더 속도), 자세가 잠시 정체돼도 직전 자세를 지워 그 사이 회전을 잃지 않는다 | mavlink_io.py, main.py, FLIGHT_SAFETY_CHECKLIST 4절 P4 | T, I | UT[안전: ATTITUDE 30 Hz]; INSPECT[main.py:자세가 잠시 정체돼도 prev 를 지우지 않는다] | 검증됨 (요청 송신·논리; FC 가 30 Hz 를 실제로 주는지는 STAT `rx[Hz] ATT=` 로 실기 확인) |
| EST-10 | 컬러 센서에 AE priority off·노출 상한 8 ms·역광 보정을 적용하고, AE 측광 ROI 를 추적 bbox 1.5배로 따라간다(≤1 Hz, 10 % 이동, 소실 시 전체 복귀) | README 안전 설계 | T | UT[camera:] | 부분 (pyrealsense2 스텁 검증, 실외 실기 미검증) |
| EST-11 | 카메라 파이프라인은 Jetson 에서 24 FPS 이상이다 | VERIFICATION 하드웨어 | D | HW | 부분 (개발자 보고 24~26 FPS, 로그 없음) |
| EST-12 | 리더 검출률은 80 % 이상이다 | VERIFICATION 하드웨어 | D | HW | **미충족** (약 60 %, 데이터셋 확장 필요) |

### SAF — 안전

| ID | 요구도 | 출처 | 방법 | 검증 근거 | 상태 |
|---|---|---|---|---|---|
| SAF-01 | FC 모드가 GUIDED/OFFBOARD 가 아니면 모드 변경·LAND 명령을 보내지 않는다(조종사 우선) | VERIFICATION C2 | T | SITL[pilot_takeover] | 검증됨 |
| SAF-02 | 리더를 한 번도 획득하지 않은 상태에서는 LAND 하지 않는다 | VERIFICATION C1 | T | UT[C1:]; SITL[boot_no_leader] | 검증됨 |
| SAF-03 | 리더 소실 시 호버(LOST_HOLD) 로 버티다 총 10 s(코스트 2 + 홀드 8)에 FAILSAFE_LAND 로 가며, 깊이만 죽고 검출이 살아 있는 경우도 같다. 그 상태에서 실제 LAND 송신은 `mission.autonomous_land`(기본 False) 일 때만이고, 기본은 0 속도(위치 유지) 유지 + STATUSTEXT | README 안전 설계, FLIGHT_SAFETY_CHECKLIST 5절 F1 | T, I | UT[소실 후 착륙까지]; CL[4초 소실]; CL[영구 소실 25s → 10초 뒤 FAILSAFE_LAND]; CL[정책(autonomous_land=False]; CL[영구 소실 25s → 10초 뒤 LAND 1회]; SITL[depth_loss]; INSPECT[mission_manager.py:lost_hold_sec=8.0] | 검증됨 (SITL depth_loss 는 autonomous_land=True 조건의 기록) |
| SAF-04 | 절대 고도 없이 공중에서 착륙 판정을 내지 않고, 착륙 판정은 리더 절대 하강 속도로 한다 | VERIFICATION C3 | T | UT[C3:]; UT[미션: 착륙 판정도]; SITL[air_landing] | 검증됨 |
| SAF-05 | GUIDED 진입 순간 미션·명령·피드포워드 상태를 리셋하고 출발 확인을 다시 요구한다 | README 안전 설계 | T | UT[재개: reset()]; CL[GUIDED 진입 후]; SITL[handover] | 부분 (인계 직후 LAND 없음은 확인. SITL 의 조종사 LOITER 가 RC 스로틀 없이 하강해 인계가 지상에서 일어났음 — 공중 인계는 미검증, sitl/README 6절) |
| SAF-06 | 고도 2.0 m(`MIN_AGL_M`, EKF 원점 기준) 아래에서는 하강 명령을 차단한다 | main.py `MIN_AGL_M` | I | UT[AGL 바닥]; INSPECT[main.py:MIN_AGL_M = 2.0]; UT[안전: 상수] | 부분 (상수·분기 존재만, SITL 시나리오 없음. z 는 지형이 아니라 EKF 원점 기준 — FLIGHT_SAFETY_CHECKLIST 5절 F6) |
| SAF-07 | 기본값은 dry-run(명령 미송신) 이다 | main.py | I | INSPECT[main.py:SEND_MAVLINK_COMMANDS = False] | 검증됨 |
| SAF-08 | PX4 의 3-튜플 mode_mapping 에서도 set_mode 가 예외 없이 동작하고 실패해도 루프가 죽지 않는다 | VERIFICATION C6 | T | UT[C6:]; SITL[px4_setmode] | 검증됨 |
| SAF-09 | 재획득 시 착륙 확인 타이머를 새로 시작한다(가려진 시간 합산 금지) | mission_manager.py | T | UT[타이머:] | 검증됨 |
| SAF-10 | 카메라·FC 링크가 30 회 연속 실패하면 HOLD 후 종료한다 | main.py | I | UT[카메라 연속 실패] | 부분 (상수 존재만) |
| SAF-11 | 잠깐 놓친 뒤 재획득하면 출발 확인 없이 추종을 재개하되 FAILSAFE_LAND 뒤에는 자동 재개하지 않는다 | VERIFICATION 재획득 | T | UT[재개:]; CL[재검출] | 검증됨 |
| SAF-12 | 리더 정지 시 FOLLOW↔LEADER_HOVER 히스테리시스(0.18 / 0.25 m/s) 로 정위치를 유지한다 | VERIFICATION H2 | T | UT[H2:]; CL[리더 정지(11s)]; CL[리더 정지 후 소실]; SITL[hover_hold] | 검증됨 |
| SAF-13 | 공분산 폭주 중에는 LOST_HOLD 이고 회복 첫 프레임에 착륙 명령이 나가지 않는다 | mission_manager.py | T | UT[타이머: 공분산] | 검증됨 |
| SAF-14 | LAND(autonomous_land=True 일 때)는 결정당 **한 번만**, 착륙 결정 **뒤에** 받은 heartbeat 가 GUIDED 일 때만 보낸다 — 조종사가 직전 1 s 안에 탈환했거나 링크가 죽었으면 보내지 않고, 재시도로 조종사가 되찾은 모드를 덮어쓰지 않는다. 먹으면 FC 가 하강한다 | main.py `land_decision_t`, FLIGHT_SAFETY_CHECKLIST 5절 F2 | T | SITL[pilot_takeover]; CL[LAND 이후]; CL[안전(takeover)]; CL[안전(lost_alt)] | 검증됨 (하네스 HEARTBEAT 1 Hz 탈환 경주) |
| SAF-15 | 위 안전 동작이 폐루프 실비행(실제 공력·바람·프롭워시)에서 유지된다 | VERIFICATION 미검증 | D | — | 미검증 |
| SAF-16 | 편대 슬롯은 비행 전 정적 검사로 슬롯 간 최소 이격(`min_separation_m`), 리더까지 거리의 깊이창 여유(`depth_reserve_m`), slot_id / follower_id 중복을 거른다 | formation.py `validate_formation` | T | UT[편대: 유효성]; UT[편대: config] | 검증됨 (정적 검사만) |
| SAF-17 | 후미 간 동적 충돌 회피와 리더 유실 시 편대 전체의 일관된 행동이 있다 | MULTI_FOLLOWER_FOUNDATION 4.5 | D | — | 미검증 (미구현 — 후미 상태 방송 뒤 반발항) |
| SAF-18 | 비유한(NaN/inf) 값은 어느 단계에서도 속도 setpoint 가 되지 않는다 — `clamp` 자체가 NaN 을 0 으로 자르고(예전엔 +MAX_VX), 제어기가 입력(상대 상태·피드포워드·슬롯 오차)을 검증해 정지하며, `sanitize_cmd` 와 `send_body_velocity` 가 뒤를 받치고, EKF 상태가 비유한이면(predict 직후·융합 직후) 추정기를 리셋한다. 자기 속도 LPF 와 상대 heading 도 NaN 에 고정되지 않는다 | FLIGHT_SAFETY_CHECKLIST 0절 G1 (ArduCopter `sane_vel_or_acc_vector` 는 clamp 뒤라 발동 불가) | T | UT[안전: clamp]; UT[안전: 제어기 입력 검증]; UT[안전: sanitize_cmd]; UT[안전: send_body_velocity]; UT[안전: self_velocity_lpf]; UT[안전: 상대 heading]; CL[안전(nan)] | 검증됨 |
| SAF-19 | FC HEARTBEAT 가 `FC_MODE_MAX_AGE_SEC`(3 s) 보다 오래되면 모드를 모르는 것으로 보아 모드 변경(LAND) 을 보내지 않고, 링크가 돌아오면 GUIDED 진입과 같이 미션을 리셋한다 | main.py `FC_MODE_MAX_AGE_SEC` | T | CL[안전(lost_alt)]; CL[안전(fc_stale): 3 s] | 검증됨 |
| SAF-20 | ATTITUDE / LOCAL_POSITION_NED 가 `FC_STATE_HOLD_AGE_SEC`(1 s) 보다 오래되면 추종 대신 정지(0 속도) 를 보낸다 — 자세 없이 비전만으로 움직이지 않는다 | main.py `FC_STATE_HOLD_AGE_SEC` | T | CL[안전(fc_stale): FC 텔레메트리] | 검증됨 |
| SAF-21 | 고도를 모르면(LOCAL_POSITION_NED 정체) 하강 명령을 막고(SAF-06 확장), GUIDED 인계 고도 + `MAX_CLIMB_ABOVE_ENTRY_M`(5 m) 위에서는 상승 명령을 막는다 | main.py `MAX_CLIMB_ABOVE_ENTRY_M` | T | CL[안전(climb)]; UT[안전: 상수] | 검증됨 (천장; 고도 미상 하강 차단은 SAF-20 의 정지가 먼저 걸림) |
| SAF-22 | 자율 착륙은 정책이다(`mission.autonomous_land`, 기본 False): 기본에서는 FAILSAFE_LAND / CONFIRMED_LANDING 에서도 모드를 바꾸지 않고 0 속도를 10 Hz 로 계속 보내며 GCS 에 STATUSTEXT 로 알린다 — 느린 리더·사람 리더가 앉음·EKF 원점 오차·하늘 배경 깊이 소실이 전부 "리더가 보이는데 엉뚱한 곳에 LAND" 를 만들기 때문 | FLIGHT_SAFETY_CHECKLIST 5절 F1 | T | CL[정책(autonomous_land=False]; UT[안전: 상수] | 검증됨 |
| SAF-23 | 원시 깊이 영상 중앙 50 % 영역에서 40 번째로 가까운 표본(4 배 서브샘플, ≈ 640 px 덩어리)이 `FORWARD_STOP_M`(1.2 m) 보다 가까우면 전진(vx>0) 을 막는다 — 추적·추정과 독립인 충돌 방벽 (ArduPilot AVOID_MARGIN 정지와 같은 역할). 8×8 px 날림 화소는 무시 | main.py `FORWARD_STOP_M` | T | UT[안전: 전방 여유]; UT[안전: 상수] | 검증됨 (단위; 폐루프 골든은 리더 ≥ 2.9 m 라 미발동) |
| SAF-24 | 내부 시계는 `time.monotonic()` — NTP/chrony 의 벽시계 점프가 소실 타이머·dt·신선도 판정을 튀게 하지 않는다(+10 s 점프 = 즉시 FAILSAFE_LAND 였음) | main.py, mavlink_io.py, leader_telemetry.py | I, T | UT[안전: 단조 시계] | 검증됨 |
| SAF-25 | 검출기·스케줄러 예외는 미검출·전체 프레임으로 격리되고, 자세에 inf 가 와도 루프가 죽지 않으며, 조작 키 'l' 은 FC 가 GUIDED 이고 모드를 알 때만, 'v' 끄기는 정지를 먼저 보낸 뒤 동작하고, 종료 시 정지를 3 회 + STATUSTEXT 로 알린다 | FLIGHT_SAFETY_CHECKLIST 5절 F4·F7 | I | INSPECT[main.py:검출기 예외 → 미검출 처리]; INSPECT[main.py:scheduler 예외 → 전체 프레임]; INSPECT[main.py:MARS: companion DOWN, holding]; UT[안전: STATUSTEXT] | 검증됨 (검사) |

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
| IF-12 | 리더 텔레메트리 수신기는 기대 리더 ID(`formation.leader_id`, ArduPilot FOLL_SYSID 역할)와 다른 ID 의 패킷을 버리고 세며, ID 없는 패킷은 호환을 위해 통과시킨다(`require_leader_id` 면 거부) | formation.py, leader_telemetry.py | T | UT[편대: 리더 ID] | 검증됨 |
| IF-13 | 리더 상태 방송 스키마 `LeaderState` 는 MAVLink `FOLLOW_TARGET`(#144) 과 왕복 변환된다 (lat/lon degE7, timestamp ms, vel ENU↔NED, yaw↔attitude_q, est_capabilities) | formation.py | T | UT[편대: LeaderState] | 검증됨 |
| IF-14 | ESP32 상대위치는 팔로워 GLOBAL_POSITION_INT 가 `GPS_MAX_AGE_SEC`(0.7 s) 보다 오래되면 만들지 않는다(`stale_follower_gps`) | MULTI_FOLLOWER_FOUNDATION 2.2 A2 | T | UT[ESP32 GPS 신선도] | 검증됨 |
| IF-15 | 리더 패킷에 yaw 가 없으면 None 으로 구분한다 (0 = 북쪽으로 오해하지 않음) | leader_telemetry.py | T | UT[편대: 패킷에 yaw] | 검증됨 |
| IF-16 | ESP32 상대위치는 팔로워 ATTITUDE yaw 가 `ATTITUDE_MAX_AGE_SEC`(0.3 s) 보다 오래되면 만들지 않는다(`stale_follower_attitude`) — 오래된 yaw 로 회전한 값이 gps 관측으로 EKF 에 들어가지 않게 | leader_telemetry.py | T | UT[안전: 팔로워 자세 신선도] | 검증됨 |
| IF-17 | 컴패니언은 미션 상태 변화(LOST_HOLD/FAILSAFE_LAND/착륙 판정)·GUIDED 진입·FC 상태 정체·전방 정지·LAND 송신·종료를 STATUSTEXT 로 GCS 에 알린다 (ArduPilot 은 대상 없는 메시지를 다른 링크로 중계) | main.py `send_statustext` | T | UT[안전: STATUSTEXT] | 검증됨 (송신; GCS 화면 표시는 실기 확인) |

### OPS — 운용 / 유지

| ID | 요구도 | 출처 | 방법 | 검증 근거 | 상태 |
|---|---|---|---|---|---|
| OPS-01 | SITL 하네스 `--all` 은 ArduCopter 로 도는 9개 시나리오를 모두 포함한다 | sitl/README | T | UT[하네스:] | 검증됨 |
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
| `leader_sine` | FCR-15, FCR-10 | 1.15 rad/s 리더 속도 변동이 팔로워에서 1배 초과로 증폭 |
| (없음 — 4.5 항목 1) | FCR-16~18, SAF-16 | 다기체 SITL 시나리오는 아직 없다. 단위·시뮬 근거만 있다 |

## 3. 미충족 · 미검증 · 부분 요구도

| ID | 상태 | 필요한 것 |
|---|---|---|
| FCR-10 | 부분 | 남은 피크 1.13 은 피드포워드+P 겹침. 3대 이상 체인이면 KFF 0.6(1.02) 또는 ESP32 선두 절대 속도 방송으로 자기 속도 경로 제거 |
| EST-12 | 미충족 | 리더 드론 데이터셋 확장, 재학습 |
| FCR-07 | 부분 | 감속 배율 0.75 / 0.55 의 단위 검사 추가 |
| EST-02 | 부분 | 카이제곱 임계 경계값 단위 검사 추가 |
| EST-10 | 부분 | 실외 역광·강한 빛 조건에서 노출 옵션 실기 확인 |
| EST-11 | 부분 | Jetson FPS 로그를 저장소에 남기기 |
| SAF-06 | 부분 | AGL 바닥 차단 SITL 시나리오(저고도에서 하강 명령). 고도 미상 시 하강 차단·천장은 SAF-21 로 폐루프 검증 |
| EST-13 | 검증됨 (합성) | 실기 드론 표적을 하늘·나무 배경에서 3 / 5 / 8 m 에 두고 `depth_valid_count`·`depth_m` 로그 분포 확인 (FLIGHT_SAFETY_CHECKLIST 3절 B-6) |
| EST-18 | 검증됨 (송신) | 실기 STAT `rx[Hz] ATT=30` 확인. 안 되면 SRn_EXTRA1 로 폴백 |
| IF-17 | 검증됨 (송신) | Mission Planner / QGC 화면에 컴패니언 STATUSTEXT 가 뜨는지 실기 확인 (sysid 255 = GCS 로 기록됨) |
| SAF-10 | 부분 | 카메라 스톨 30 회 → HOLD → 종료 폐루프 시나리오 |
| SAF-15 | 미검증 | 폐루프 실비행(안전줄·저고도부터) |
| SAF-05 | 부분 | 하네스 조종사 링크에 RC override 를 넣어 LOITER 중 고도를 유지하게 고치고 `handover` 재실행 (sitl/README 6절) |
| FCR-02 | 확인 필요 | `depth_range` 를 `--duration 60` 이상으로 재실행해 평형 거리 3.45 m 수렴 확인 |
| IF-11 | 미검증 | ESP32 펌웨어 작성, 패킷 포맷 고정 — 필드 규약은 `formation.LeaderState.to_follow_target()` (leader_id, yaw 추가) |
| FCR-19 | 검증됨 (하네스) | 실기 지상 기울임 점검(FLIGHT_SAFETY_CHECKLIST 3절 B-3) 으로 ATTITUDE 부호 규약 확인 |
| FCR-18 | 검증됨 (시뮬) | SITL 다기체(리더 1 + 후미 2~3, `-I0/-I1/-I2`) V 자 편대 + `leader_sine` 시나리오로 실측 |
| SAF-17 | 미검증 | 후미 상태 방송(`LeaderState` 스키마 재사용) + 이웃 간 최소 이격 반발항 |
| FCR-22 | 부분 | 추종/정지 전환(이득 0 포함)을 스위칭 시스템으로 두고 dwell-time 또는 공통 Lyapunov 논거; 측면·기수 2×2 결합 포함 |
| FCR-10 | 부분 | 단일 후미 \|Γ\| 1.13 은 남지만 다중 후미 체인의 누적 문제는 FCR-20(선두 방송)으로 해소. 방송 없는 운용이면 FCR-21 의 h 적용 |

## 4. 검사 방법

```bash
python3 analysis/trace_check.py     # 표의 참조가 전부 존재하는지 + 요구도에 안 묶인 검사 목록
python3 test_fixes.py               # '추적성:' 검사로 같은 것을 회귀에 포함
```
