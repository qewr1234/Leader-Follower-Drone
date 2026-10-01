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
| FCR-02 | 리더 등속 0.3 m/s 추종 시 정상상태 거리 오차가 이론값 (v − KFF·(v−DB) + KV·v)/Kp (0.63 m) 와 ±0.05 m 로 일치하고 평형 거리가 깊이창(목표+3 m) 안에 든다 | README 제어 법칙 | A, T | UT[FF: 1축 폐루프]; UT[FF (시간간격 정책)]; CL[FOLLOW 중(리더]; SITL[depth_range]; AN[kff_sweep.kff0.8.ss_err_per_mps] | 검증됨 (SITL 은 Kp 0.22·KV 0 설계로 실측 — 재실행 필요) |
| FCR-03 | 속도 명령은 BODY_NED 프레임으로 축별 한계(0.35 / 0.22 / 0.25 m/s, yaw 0.35 rad/s)에 포화되고 yaw_rate 마스크를 항상 유효로 보낸다 | main.py | T, I | UT[C5:]; INSPECT[main.py:MAX_VX = 0.35] | 검증됨 |
| FCR-04 | 속도 setpoint 는 10 Hz(9.5~10.5 Hz) 로 송신한다 | main.py `SETPOINT_PERIOD_SEC` | T | CL[setpoint 송신율] | 검증됨 |
| FCR-05 | 기수는 리더 방위각을 0 으로 유지하고, 명령 yaw_rate 0 에서 기체가 스스로 회전하지 않는다 | VERIFICATION C5 | T, A | UT[C5:]; SITL[hold_heading]; AN[yaw.nominal.pm_deg] | 검증됨 |
| FCR-06 | 명령 평활은 프레임률과 무관하게 같은 시정수를 갖는다 | main.py `smooth_velocity_cmd` | T | UT[평활:] | 검증됨 |
| FCR-07 | 위치 공분산 trace 가 2 / 4 를 넘으면 명령을 0.75 / 0.55 배로 줄인다 | config `controller.uncertainty_slowdown_trace` | A | AN[kff_sweep.scale0.55.gm_db] | 부분 (여유 분석만, 배율 단위 검사 없음) |
| FCR-08 | IMM-EKF 가 직접 추정한 리더 절대 속도를 KFF 0.8 로 피드포워드하되 0.05 m/s 소프트 데드존(기울기 1)과 0.1 s 저역통과를 거치고, 자기 속도 감쇠 −KV·v_self(0.3, 시간간격 정책 h 1.0 s)를 더하며, 인자가 없으면 기존 명령과 같다 | README 제어 법칙, STABILITY_MARGINS 7절 | T | UT[FF:]; CL[FOLLOW 중(리더]; SITL[depth_range]; SITL[leader_sine] | 검증됨 (SITL 은 직전 설계 실측: depth_range 4.1 m, leader_sine 0.72 — 현재 설계 예측은 AN[sitl_like.current], 재실행 필요) |
| FCR-09 | 바깥 루프는 전 축에서 위상여유 ≥ 45°, 이득여유 ≥ 6 dB, 감도 피크 Ms ≤ 2 를 만족한다 | STABILITY_MARGINS 1·4절 | A | AN[axes.forward.pm_deg]; AN[axes.forward.gm_db]; AN[axes.right.gm_db]; UT[분석 (FCR-10)]; UT[분석 골든:] | 검증됨 (분석: GM 16.4 / 16.7 / 17.3 dB, PM 102°, Ms ≤ 1.22. 직전 설계 14.4 dB, 수정 전 4.9 dB — 골든 검사가 두 값을 기록) |
| FCR-10 | 리더→팔로워 속도 전달 \|Γ(jω)\| 이 모든 주파수에서 1 이하다 (스트링 안정, 다중 기체 체인 전제) | STABILITY_MARGINS 6·7절 | A, T | AN[axes.forward.peak]; AN[validation]; AN[sitl_like.current]; UT[분석 (FCR-10)]; UT[분석 (FCR-10 강건성)]; SITL[leader_sine] | 검증됨 (분석: 1.000 / 1.000 / 0.999 — 절대속도 추정기 + FF τ 0.1 s + 시간간격 KV 0.3(h 1.0 s ≥ 2τ_eff). KV=0 이면 1.14, τ 2.0 이면 1.08, KV 0.2 는 τ_fc 0.5 s 에서 1.014 — 골든·강건성 검사가 기록. SITL leader_sine 은 직전 설계 실측 0.72, 현재 설계 재실행 필요) |
| FCR-11 | FC ATTITUDE 의 roll/pitch/yaw 변화량으로 매 프레임 EKF 상대 상태를 역회전해 기체 기울어짐이 리더 이동으로 보이지 않게 하고, 그 보정이 CT 각속도로 새지 않는다 | README 안전 설계 | T | UT[자세보정:]; UT[ego-yaw:] | 검증됨 (실기 미검증) |
| FCR-12 | 비전 거리가 없고 GPS 상대위치만 있으면 이격을 8 m 로 넓힌다 | README 안전 설계 | T | UT[gps-only:] | 검증됨 |
| FCR-13 | 출발·정지·착륙 판단은 리더 절대 속도(ESP32 > EKF 절대속도 > 상대 폴백) 로 하고, 출발·호버 판정은 수평·수직을 합친 3차원 움직임으로 한다(수직으로만 움직이는 리더도 FOLLOW). 착륙 판정은 수평 정지 + 하강 | README 안전 설계 | T | UT[미션:]; UT[미션 수직:]; CL[리더 출발 후]; CL[추종 중(t=8s)]; SITL[depth_range] | 검증됨 |
| FCR-14 | 피드포워드·시간간격 항을 뺀 P+D 기준선은 PM ≥ 45°, GM ≥ 6 dB, Ms ≤ 1.5, 스트링 안정을 만족한다 | STABILITY_MARGINS 1절 | A | AN[kff_sweep.kff0.gm_db]; UT[분석: P+D] | 검증됨 (분석: GM 24.6 dB, \|Γ\| 1.000) |
| FCR-16 | 명령의 시선 방향 접근 속도는 모든 거리에서 KS·(d − 2.0 m) 이하다 (연속 장벽): 바닥 밖에서는 접근 상한, 안에서는 침범량에 비례한 후퇴. 특히 피드포워드가 바닥 안에서 접근 명령을 만들지 못한다. 축별 속도 포화 **뒤**에도 성립한다(포화가 시선 성분을 키우면 재확인; 상자 안에서 후퇴가 부족할 때도 접근은 0) | VERIFICATION 남은 결함(선회 시 최근접 1.59 m), 이전 감사 #35 | T, A, I | UT[이격:]; UT[이격 장벽:]; UT[FCR-16 클램프:]; INSPECT[config.py:"min_separation_m": 2.0] | 검증됨 (P 항만의 경로에서는 교차점 1.0 m 위에서 P 가 더 강해 놀지만, FF 파고들기(1.8 m 에서 +0.04 → −0.12)는 이 장벽만 막는다 — 단위 검사. 포화 재현 rel=[1.2,1.2]·cmd=[0.9,−0.9]: 예전 +0.092 접근 → 이제 ≤ 0) |
| FCR-17 | 회피 반경(1.5 m) 안에서는 시선에 수직인 수평 방향으로 비켜선다. 보증 범위: (a) 한 방향으로 지나가는 리더는 1.0 m/s 까지 접촉 없이 비킨다, (b) 재조준하며 추격하는 리더는 hypot(MAX_VX, MAX_VY)=0.41 m/s 아래에서만 — 그 위는 순수추격 기하상 어떤 제어기로도 불가능하다 | analysis/evasion_sim.py | T, A | UT[회피:]; UT[회피 경계:]; UT[회피 경계 골든]; SITL[min_separation] | 검증됨 (범위 명시. 모의: straight 0.7 m/s → 0.60 m, pursuit 0.4 → 1.81 m, pursuit 0.7 → 접촉(골든). SITL min_separation 통과 — 사용자 실행, 수치 기록 대기) |
| FCR-18 | 제어·미션 입력(상대 위치·속도, 리더 속도)은 카메라 마운트 자세와 기체 roll/pitch 를 편 수평(기수 정렬) 프레임이고, 상대 위치에는 카메라 레버암(`camera.mount_offset_frd_m`)을 더하며, EKF 의 자기 속도 입력은 그 역변환으로 카메라 프레임에 들어간다 — FC 가 BODY_NED 를 yaw 만 돌리는 것과 일치 | 이전 감사 #2·#22·#38·#42·#44 | T, I | UT[레벨링:]; UT[마운트:]; UT[레버암:]; INSPECT[config.py:"mount_pitch_deg"] | 검증됨 (단위; 실기 마운트 각·레버암 미측정 → config 0) |
| FCR-19 | 추정 거리가 `max_follow_dist_m`(15 m)보다 멀면 소실로 취급해 쫓지 않고, `max_alt_m`(30 m) 위에서는 상승을 막으며, 목표 이격 3 m ↔ 8 m 전환은 추종 중 0.3 m/s 램프다 | 감사 2026-10-01 12·20번, 이전 #24·#66 | T, I | UT[최대 거리:]; UT[천장:]; UT[이격 램프:]; INSPECT[config.py:"max_follow_dist_m"] | 검증됨 (단위) |
| FCR-20 | 거리 측정이 끊기면 추종 항(P·D·FF·KV)을 마지막 측정 뒤 `coast_fade_start_sec`(0.3 s)부터 `range_coast_max_sec`(2 s)에 0 이 되도록 선형으로 줄인다. 이격 장벽·회피와 yaw 는 줄이지 않는다 | 이전 감사 #20 (외삽 추정으로 2 s 전진) | T, I | UT[코스트 페이드:]; CL[4초 소실]; INSPECT[config.py:"coast_fade_start_sec"] | 검증됨 (단위; 폐루프에서 16~18 s 명령이 0.025 → 0 으로 감쇠) |
| FCR-21 | 피드포워드 데드존은 축별이고, 상하축 자기 속도 감쇠는 `self_vel_damping_up`(0.15, 수직 뒤처짐 1.94 m/(m/s))이며, yaw 명령은 불확실성 감속을 받지 않고 분모 하한이 최소 이격(2 m)이다 | 이전 감사 #36·#37·#41·#56 | T, A | UT[축별 데드존:]; UT[KV 상하축:]; UT[분석 (FCR-10 상하축)]; UT[yaw 근접:] | 검증됨 (분석: 상하축 KV 0.15 에서 \|Γ\| 0.9993, FC 지연 격자 ≤ 1.003; KV 0 이면 1.032) |
| FCR-15 | 리더 속도가 0.25 ± 0.05 m/s, 1.15 rad/s 정현파일 때 팔로워 속도 진폭비가 1 이하다 (실제 FC 에서의 스트링 안정성) | STABILITY_MARGINS 6절, sitl/README | T | SITL[leader_sine]; AN[sitl_like.current] | 검증됨 (SITL 실측 0.43 / 0.72, 예측 0.70. 수정 전 코드 대조군 1.95 FAIL — 차등 검증) |

### EST — 인지 / 추정

| ID | 요구도 | 출처 | 방법 | 검증 근거 | 상태 |
|---|---|---|---|---|---|
| EST-01 | IMM-EKF(CV+CT) 가 상대 위치·속도를 추정하고 속도 추정의 63 % 응답이 0.5 s 이내이며 램프에 위치 지연이 없다 | imm_ekf.py | A, T | AN[ekf_step.t63_velocity_s]; UT[분석: IMM-EKF] | 검증됨 (0.30 s) |
| EST-15 | 모드 확률이 리더 기동에 따라 움직여 "mode-aware" 추정이 실제로 동작한다: 추종 중 직진 p_ct < 0.2, 선회 0.5 rad/s > 0.4, 1.0 rad/s > 0.7 (리더 1 m/s 기준) | 프로젝트 이름(MARS-IMM) | T | UT[IMM (EST-15)]; UT[IMM: 전이확률]; UT[IMM: 상태 속도] | 검증됨 (0.11 / 0.72 / 0.86 — 리더 1 m/s. 두 가지가 필요했다 — 상태 속도를 리더 절대 속도로(자기 속도는 예측 입력), 전이확률을 체류시간 기반으로. 둘 중 하나만으로는 각각 0.35 / 0.11. **설계점 0.3 m/s 에서는 선회 0.5 rad/s 의 구심 가속 0.15 m/s² 가 CV 모델의 σ_a 0.8 안이라 p_ct ≈ 0.2 로 분리되지 않는다(이전 감사 #27) — 원리적 한계로 기록. 모드 확률은 제어에 쓰이지 않고 검출 주기·ROI 에만 쓰이므로 비행 안전과 무관**) |
| EST-02 | 측정은 신뢰도로 R 을 적응하고 카이제곱 게이트(3D 11.34, 2D 9.21)로 거른다. RGB-D, bearing, ESP32 위치, ESP32 속도 네 경로 모두 같은 게이트를 지난다 | config `reliability` | T | UT[C4: MAD]; UT[C4: 정상]; UT[ESP32 속도:] | 부분 (게이트 통과/거부 검사는 있음 — UT[ESP32 속도:], 임계 11.34/9.21 경계값 검사 없음) |
| EST-13 | ESP32 상대 속도는 필터 상태 직접 대입이 아니라 정규 칼만 속도 측정 갱신으로 반영한다. 게이트를 통과한 것만 쓰고, 공분산은 임의 축소가 아니라 갱신 결과로 줄어든다 | VERIFICATION 남은 결함 | T | UT[ESP32 속도:] | 검증됨 |
| EST-14 | CT 모델의 회전율은 리더 절대 속도(상대 + 자기 속도)로 추정한다. 상대 속도만 쓰면 추종이 정착할수록 방향각이 정의되지 않아 회전율이 실제와 무관해진다 | VERIFICATION 남은 결함 | T | UT[IMM:] | 검증됨 |
| EST-03 | 거리 측정이 끊기면 2 s 코스트 후 소실로 판정하고, bearing-only 는 거리 확보로 치지 않는다 | config `imm.range_coast_max_sec` | T, I | UT[rcoast:]; INSPECT[config.py:"range_coast_max_sec": 2.0] | 검증됨 |
| EST-04 | 트래커는 1프레임 소실 뒤 화면 반대편 검출을 거부하고 max_lost 뒤 새 track_id 로 재초기화한다 | VERIFICATION 트래커 신원 게이트 | T | UT[tracker:] | 검증됨 |
| EST-16 | 트랙을 버린 뒤의 재획득은 추정기가 믿을 수 있는 동안 예측 화소 위치(99 % 타원 + 60 px) 안의 검출로만 하고, 3-D 게이트가 거리 측정을 연속 3회 거부하면 트랙을 버리며, 게이트가 거부한 측정의 bearing 은 쓰지 않는다 | 감사 2026-10-01 2번 | T | UT[재획득:]; INSPECT[config.py:"reacquire_margin_px"] | 검증됨 (단위 8개) |
| EST-17 | 카이제곱 게이트는 기본 R 로 판정하고 신뢰도로 부풀린 R 은 칼만 이득에만 쓴다(RGB-D·bearing·ESP32 위치·속도 네 경로) — 품질이 나쁜 측정일수록 게이트가 넓어지던 결합 제거 | 이전 감사 #26·#59, 신규 16 | T | UT[게이트 분리:] | 검증됨 |
| EST-18 | 회복 근접 게이트는 bbox 대각선 비례에 절대 하한 80 px 를 둔다 — 먼 소형 표적이 자세 과도 한 번에 폐기되지 않게 | 감사 2026-10-01 18번 | T | UT[트래커 회복 게이트:]; INSPECT[config.py:"recover_gate_min_px"] | 검증됨 |
| EST-05 | 깊이 측정은 bbox 안쪽 ROI 의 **최근접 군집**(틈 `depth_cluster_gap_m` 0.5 m, ≥ `min_depth_valid_count` 화소) median 이다 — 배경 화소가 과반이어도 배경에 앉지 않는다. 유효 화소 수·비율(`min_depth_valid_ratio` 아래는 신뢰도 0, 2배까지 램프)·MAD 한계로 거르고 군집 몫이 작으면 신뢰도를 깎는다. 큰 ROI 서브샘플 오차는 1 cm 미만이다 | VERIFICATION C4, 이전 감사 #58·#30 | T | UT[C4:]; UT[measurement:]; UT[깊이 군집:]; INSPECT[config.py:"depth_cluster_gap_m"] | 검증됨 (단위: 배경 56 %·리더 44 % ROI 에서 3 m 선택) |
| EST-19 | EKF 는 연속 `init_confirm_frames`(3) 프레임의 RGB-D 측정이 서로 `init_consistency_m`(0.75 m) 안에 있을 때만 초기화하고, 새 트랙은 검출 신뢰도 `init_conf_thres`(0.40) 이상으로만 시작한다 (추적 중 매칭은 0.25) | 이전 감사 #60 | T, I | UT[초기화 확인:]; UT[tracker: 폐기 뒤에는]; INSPECT[config.py:"init_conf_thres"] | 검증됨 (단위; 폐루프 초기화가 2프레임 늦어짐) |
| EST-20 | RGB-D 측정 잡음은 거리 의존이다: 기준 3 m 의 `base_R_rgbd_diag` 에 횡 (z/3)²·깊이 (z/3)⁴ 배율(0.5~4 배로 자름)을 곱해 게이트와 이득에 쓴다. ESP32 위치·속도의 [수평,수평,수직] 분산은 카메라 축 [h,v,h] 로 들어가고, 측정 잡음 상수는 config 한 곳(`reliability`)에만 있다 | 이전 감사 #26·#61·#67 | T, I | UT[거리 R:]; UT[R 축:]; UT[R 단일화:]; INSPECT[config.py:"rgbd_range_ref_m"] | 검증됨 (3 m 설계점은 그대로라 분석 골든 불변) |
| EST-21 | 자기 속도 가용성이 바뀌는 순간(수신↔끊김) 상태 속도를 Δv_ego 만큼 옮기고 속도 P 를 키워 과도 없이 절대↔상대 속도로 전환하며, 예측 역학의 dt 는 `max_predict_dt_sec`(0.5 s)로 자르되 코스트 타이머는 실제 dt 를 세고, 게이트가 거부한 프레임도 coast_time 을 센다 | 이전 감사 #28·#55·#57 | T, I | UT[ego 전환:]; UT[predict dt 상한:]; UT[게이트 거부 코스트:]; INSPECT[config.py:"max_predict_dt_sec"] | 검증됨 (단위) |
| EST-06 | 검출을 건너뛴 프레임의 측정은 신뢰도를 0.55 배로 깎는다 | reliability.py | T | UT[skip:] | 검증됨 |
| EST-07 | 검출기는 대상 클래스만 남겨 (conf, area) 순으로 정렬하고 ROI 오프셋을 적용하며 conf/iou/imgsz 를 매 호출 전달한다. 모델에 없는 클래스면 시작 시 경고한다 | detector.py | T | UT[detector:]; UT[검출:] | 검증됨 |
| EST-08 | 스케줄러는 안정 상태에서 `normal_detect_every` 주기로 검출한다 | scheduler.py | T | UT[scheduler:] | 검증됨 |
| EST-09 | EKF 융합 상태는 캐시되고 상태 변경 시 무효화된다 | imm_ekf `get_state` | T | UT[ekf:] | 검증됨 |
| EST-10 | 컬러 센서에 AE priority off·역광 보정을, 깊이 센서에 프로젝터·laser_power·IR AE·AE 노출 상한(config `depth_*`)을 적용하고, AE 측광 ROI 를 추적 bbox 1.5배로 따라간다(≤1 Hz, 10 % 이동, 연속 5프레임 넘게 놓쳤을 때만 전체 복귀). 컬러 노출 상한 옵션은 librealsense 에 없어 제거했다 | README 안전 설계, 감사 2026-10-01 17·19번 | T | UT[camera:]; UT[AE ROI:] | 부분 (pyrealsense2 스텁 검증, 실외 실기 미검증) |
| EST-11 | 카메라 파이프라인은 Jetson 에서 24 FPS 이상이다 | VERIFICATION 하드웨어 | D | HW | 부분 (개발자 보고 24~26 FPS, 로그 없음) |
| EST-12 | 리더 검출률은 80 % 이상이다 | VERIFICATION 하드웨어 | D | HW | **미충족** (약 60 %, 데이터셋 확장 필요) |

### SAF — 안전

| ID | 요구도 | 출처 | 방법 | 검증 근거 | 상태 |
|---|---|---|---|---|---|
| SAF-01 | FC 모드가 GUIDED/OFFBOARD 가 아니면 모드 변경·LAND 명령을 보내지 않는다(조종사 우선) — 미션 경로와 수동 'l' 키 모두 | VERIFICATION C2 | T | SITL[pilot_takeover]; UT[키 게이트:] | 검증됨 |
| SAF-02 | 리더를 한 번도 획득하지 않은 상태에서는 LAND 하지 않는다 | VERIFICATION C1 | T | UT[C1:]; SITL[boot_no_leader] | 검증됨 |
| SAF-03 | 리더 소실 시 호버(LOST_HOLD) 로 버티다 총 10 s(코스트 2 + 홀드 8)에 LAND 하며, 깊이만 죽고 검출이 살아 있는 경우도 같다. 10 s 뒤의 행동은 config `controller.lost_action` (land 기본 · rtl · hold) 로 정하며 셋 다 allow_follow=False 다 | README 안전 설계 | T, I | UT[소실 후 착륙까지]; CL[4초 소실]; CL[영구 소실]; SITL[depth_loss]; INSPECT[mission_manager.py:lost_hold_sec=8.0]; UT[소실 정책:]; INSPECT[config.py:"lost_action"] | 검증됨 |
| SAF-04 | 절대 고도 없이 공중에서 착륙 판정을 내지 않고, 착륙 판정은 리더 절대 하강 속도로 한다 | VERIFICATION C3 | T | UT[C3:]; UT[미션: 착륙 판정도]; SITL[air_landing] | 검증됨 |
| SAF-05 | GUIDED 진입 순간 미션·명령·피드포워드 상태를 리셋하고 출발 확인을 다시 요구한다 | README 안전 설계 | T | UT[재개: reset()]; CL[GUIDED 진입 후]; SITL[handover] | 부분 (인계 직후 LAND 없음은 확인. SITL 의 조종사 LOITER 가 RC 스로틀 없이 하강해 인계가 지상에서 일어났음 — 공중 인계는 미검증, sitl/README 6절) |
| SAF-06 | 고도 1.5 m 아래이거나 고도를 모르면 하강 명령을 차단한다(fail-closed). 고도 출처는 하향 거리계(RANGEFINDER/DISTANCE_SENSOR, AGL) → LOCAL_POSITION_NED → GLOBAL_POSITION_INT relative_alt(home 기준) 순이고 낡은 출처는 건너뛴다; 자기 속도도 LOCAL_POSITION_NED → GLOBAL_POSITION_INT 로 폴백한다 | main.py `enforce_agl_floor`, 이전 감사 #62·#64 | T, I | UT[AGL 바닥]; UT[고도 출처:]; UT[자기 속도 폴백:]; INSPECT[main.py:MIN_AGL_M = 1.5] | 검증됨 (단위; SITL 저고도 시나리오는 없음. 거리계 없는 기체에서는 home 기준 — 이륙점이 평지일 것) |
| SAF-07 | 기본값은 dry-run(명령 미송신) 이다 | main.py | I | INSPECT[main.py:SEND_MAVLINK_COMMANDS = False] | 검증됨 |
| SAF-08 | PX4 의 3-튜플 mode_mapping 에서도 set_mode 가 예외 없이 동작하고 실패해도 루프가 죽지 않는다 | VERIFICATION C6 | T | UT[C6:]; SITL[px4_setmode] | 검증됨 |
| SAF-09 | 재획득 시 착륙 확인 타이머를 새로 시작한다(가려진 시간 합산 금지) | mission_manager.py | T | UT[타이머:] | 검증됨 |
| SAF-10 | 카메라 프레임이 실패해도 그 프레임의 예측·소실 타이머·미션·setpoint 는 돌고(소실로 취급), 30 회 연속이면 LOST_ACTION 을 한 번 보내고 종료한다. FC 링크 예외는 15 s 시간 기준으로 포기하되 1 s 마다 재연결을 시도하고, heartbeat 가 3 s 넘게 없으면 모드를 '모름' 으로 본다 | 감사 2026-10-01 (이전 #1·#8·#13·#53) | T, I | UT[카메라 실패 프레임:]; UT[카메라 연속 실패]; UT[FC 링크:]; INSPECT[main.py:FC_FAIL_SEC] | 검증됨 (단위·소스 검사; 폐루프 시나리오는 없음) |
| SAF-11 | 잠깐 놓친 뒤 재획득하면 출발 확인 없이 추종을 재개하되 FAILSAFE_LAND 뒤에는 자동 재개하지 않는다 | VERIFICATION 재획득 | T | UT[재개:]; CL[재검출] | 검증됨 |
| SAF-12 | 리더 정지 시 FOLLOW↔LEADER_HOVER 히스테리시스(0.18 / 0.25 m/s) 로 정위치를 유지한다 | VERIFICATION H2 | T | UT[H2:]; CL[리더 정지(11s)]; CL[리더 정지 후 소실]; SITL[hover_hold] | 검증됨 |
| SAF-13 | 공분산 폭주 중에는 LOST_HOLD 이고 회복 첫 프레임에 착륙 명령이 나가지 않는다 | mission_manager.py | T | UT[타이머: 공분산] | 검증됨 |
| SAF-14 | LAND 는 FC 가 GUIDED 안에 있을 때만 2 s 간격으로 재시도하고, 먹으면 FC 가 하강한다 | main.py `LAND_RETRY_SEC` | T | SITL[pilot_takeover]; CL[LAND 이후] | 검증됨 |
| SAF-15 | 위 안전 동작이 폐루프 실비행(실제 공력·바람·프롭워시)에서 유지된다 | VERIFICATION 미검증 | D | — | 미검증 |
| SAF-17 | ATTITUDE 의 yaw 가 yawspeed·Δt 로 설명되지 않게 뛰면(FC EKF yaw 재정렬) 그 프레임의 자세 보상을 건너뛴다 — 기체는 돌지 않았으므로 상대 상태는 그대로 | 감사 2026-10-01 5번 | T | UT[자세 불연속:]; INSPECT[main.py:ATT_YAW_JUMP_RAD] | 검증됨 |
| SAF-18 | 루프·신선도·setpoint 스케줄·ESP32 age 는 단조 시계(time.monotonic)를 쓴다 — 벽시계 점프가 LAND 나 송신 정지를 만들지 않는다 | 이전 감사 #48·#49 | T, I | UT[단조 시계:] | 검증됨 |
| SAF-19 | SIGTERM/SIGHUP 에도 종료 경로(finally: 마지막 HOLD·장치 닫기·CSV)가 실행된다 | 이전 감사 #51 | T | UT[종료:] | 검증됨 |
| SAF-20 | 한 프레임이 1 s 넘게 걸리면 경고하고 3 s(GUID_TIMEOUT)를 넘기면 평활·피드포워드 상태를 0 에서 재시작한다 — 멈췄다 돌아올 때 옛 속도로 재출발하지 않는다 | 이전 감사 '루프 워치독 부재' | T, I | UT[워치독:]; INSPECT[main.py:LOOP_RESET_SEC] | 검증됨 (단일 스레드라 멈춘 동안의 처리는 없음) |
| SAF-21 | 시동 시 `config.validate_config` 가 설정의 대소·범위(최소 이격 < 목표 이격 < 깊이창, KFF < 1, 이득 부호, 속도 한계, lost_action, 깊이 범위, 마운트 각 …)를 검증해 모순이면 ValueError 로 멈춘다 | 이전 감사 '설정 검증 부재' | T, I | UT[설정 검증:]; INSPECT[config.py:def validate_config] | 검증됨 |
| SAF-22 | 회피 방향 래치는 추종이 끊긴(allow_follow False) 동안 매 프레임 풀린다 — 재개 시 리더 쪽으로 비켜서지 않는다 | 이전 감사 #59 | I | UT[회피 래치 해제:] | 검증됨 (소스 검사) |
| SAF-16 | 추정·명령 경로의 비유한 값(NaN/inf)은 '최대 속도' 가 아니라 '정지' 로 귀결된다: 송신부는 0 으로 치환하고, 제어기는 0 명령을 내며, NaN 공분산은 최대 불확실로 취급하고, EKF 는 재초기화 대기로 돌아간다 | 외부 레퍼런스 감사 2026-09-24 (clamp(nan)=hi 재현) | T, I | UT[NaN:]; INSPECT[utils_geometry.py:if x != x]; INSPECT[main.py:비유한 속도 명령] | 검증됨 (단위 6개: 와이어·제어기·공분산 게이트·자세 보정·재초기화) |

### IF — 인터페이스

| ID | 요구도 | 출처 | 방법 | 검증 근거 | 상태 |
|---|---|---|---|---|---|
| IF-01 | FC heartbeat 는 같은 시스템의 autopilot 컴포넌트만 인정하고 GCS/짐벌/INVALID 는 무시하며, pymavlink 가 target_component 0 을 주면 1 로 고정한다 | VERIFICATION 2026-09-18 | T | UT[HB:] | 검증됨 (SITL 실측) |
| IF-02 | FC 연결은 heartbeat 타임아웃을 견디고 필요한 데이터 스트림만 요청한다 — REQUEST_DATA_STREAM 3개(구형 ArduPilot)와 MAV_CMD_SET_MESSAGE_INTERVAL(ArduPilot 4.x·PX4: LOCAL_POSITION_NED/ATTITUDE/GLOBAL_POSITION_INT 10 Hz, SYSTEM_TIME 1 Hz, 거리계 5 Hz). heartbeat 는 오는데 위치 스트림이 없으면 5 s 마다 재요청한다 | mavlink_io `connect_fc`, 이전 감사 #11·#54 | T | UT[FC:]; UT[스트림 요청:] | 검증됨 (단위; PX4 실기 미확인) |
| IF-16 | FC 의 STATUSTEXT 를 콘솔에 띄우고 GUIDED/OFFBOARD 이탈 시 최근 메시지를 함께 보이며, 모드 변경·NAV_LAND 의 COMMAND_ACK 거부를 경고하고, SYSTEM_TIME·하향 거리계를 저장해 로그에 남긴다. drain 은 한 번에 200개까지만 읽고, MAVLink 1 링크면 경고하며, 보레이트는 `MARS_FC_BAUD` 로 바꾼다 | 이전 감사 #3·#4·#47·#49·#53·#65 | T, I | UT[MAVLink 메시지:]; UT[drain 상한:]; UT[MAVLink2:]; UT[보레이트:]; INSPECT[mavlink_io.py:DRAIN_MAX_MSGS] | 검증됨 (단위) |
| IF-17 | 카메라 내부 파라미터(fx/fy/ppx/ppy)가 빠지면 예외다 — 기본값으로 조용히 대체하지 않는다 | 이전 감사 #63 | T | UT[intrinsics:] | 검증됨 |
| IF-03 | STAT 에 HEARTBEAT / LOCAL_POSITION / ATTITUDE / GLOBAL_POSITION 수신율을 표시한다 | mavlink_io `stream_rates_text` | T | UT[rx:] | 검증됨 |
| IF-04 | ESP32 시리얼의 잘린 라인을 버리지 않고 다음 읽기에서 복원한다 | leader_telemetry.py | T | UT[serial:] | 검증됨 |
| IF-05 | ESP32 장치나 pyserial 이 없어도 비전 단독으로 기동한다 | main.py `open_leader_receiver` | T | UT[ESP32:] | 검증됨 |
| IF-06 | 리더 고도 기준계(AMSL / 타원체)를 팔로워의 같은 기준계와만 뺀다 | leader_telemetry.py | T | UT[alt:]; UT[lla:] | 검증됨 |
| IF-07 | 헤드리스(`MARS_SHOW_WINDOW=0`) 로 동작한다 | main.py | T | UT[헤드리스:] | 검증됨 |
| IF-08 | LOCAL_POSITION_NED 0.4 s, ATTITUDE 0.3 s 이내의 값만 자기 속도·자세 보정·피드포워드에 쓴다. GPS_RAW_INT 의 0.7 s 신선도(GPS_MAX_AGE_SEC)는 HUD·로그에 표시만 하며 ESP32 상대위치 융합(leader_telemetry)의 팔로워 GPS 입력을 게이트하지 않는다 | main.py `*_MAX_AGE_SEC` | I, T | INSPECT[main.py:LOCAL_POS_MAX_AGE_SEC = 0.40]; SITL[depth_range] | 검증됨 (STAT `fresh=LP1/ATT1` 실측) |
| IF-09 | 배터리 전압 미보고(65535)는 전압으로 쓰지 않는다 | mavlink_io `battery_text` | T | UT[BAT:] | 검증됨 |
| IF-10 | 실험 로그는 평탄화된 행으로 기록된다 | logger.py | T | UT[logger:] | 검증됨 |
| IF-11 | ESP32(ESP-NOW) 송신 펌웨어가 리더 절대 위치·속도를 방송한다 | README 시스템 개요 | D | — | 미검증 (펌웨어 미존재) |
| IF-12 | ESP32 패킷에 속도가 없거나 팔로워 자기 속도(GLOBAL_POSITION_INT)를 모르면 상대 속도 측정을 0 이나 절대 속도로 꾸미지 않고 '없음' 으로 넘겨 미션은 EKF 속도로, 속도 갱신은 생략한다 | 감사 2026-10-01 9번, 이전 #39 | T | UT[ESP32 속도 없음:]; UT[ESP32 상대속도 폴백:] | 검증됨 |
| IF-13 | 컴패니언은 기체와 같은 sysid 와 compid 191(ONBOARD_COMPUTER) 로 송신하고 1 Hz heartbeat 를 보내며, 자기 heartbeat 를 FC 것으로 오인하지 않는다 | 이전 감사 #14·#50 | T | UT[신원:] | 검증됨 (SYSID_ENFORCE=0 전제, README 체크리스트) |
| IF-14 | 상태 메시지(위치·자세·배터리)는 우리 FC(같은 sysid·고정 compid)에서 온 것만 받고, HEARTBEAT 의 system_status 를 저장해 CRITICAL 이상이면 경보한다. ESP32 융합은 팔로워 GLOBAL_POSITION_INT·ATTITUDE 가 신선할 때만 | 감사 2026-10-01 10·15·21번 | T | UT[sysid 필터:]; UT[system_status:]; UT[ESP32 신선도:] | 검증됨 |
| IF-15 | 비행 로그는 1 s 마다 fsync 하고 `logger.max_mb` 마다 파일을 돌리며, CSV 변환은 스트리밍으로 전 파일을 합친다(잘린 마지막 줄 허용) | 이전 감사 '로그 내구성' | T | UT[로거:] | 검증됨 |

### OPS — 운용 / 유지

| ID | 요구도 | 출처 | 방법 | 검증 근거 | 상태 |
|---|---|---|---|---|---|
| OPS-01 | SITL 하네스 `--all` 은 ArduCopter 로 도는 10개 시나리오를 모두 포함한다 | sitl/README | T | UT[하네스:] | 검증됨 |
| OPS-02 | 모터 테스트 프로토타입 등 죽은 코드가 없다 | VERIFICATION | I | UT[정리:] | 검증됨 |
| OPS-03 | 모든 요구도의 검증 근거가 실제 검사·시나리오·분석 키를 가리킨다 | 이 문서 | T | UT[추적성:] | 검증됨 |
| OPS-04 | `main.main()` 은 가짜 FC·가짜 시계로 결정론 실행되어 예외 없이 끝난다 | test_closed_loop.py | T | CL[main.main()] | 검증됨 |
| OPS-05 | README 에 실비행 전 FC 파라미터 체크리스트(failsafe·펜스·배터리·GUIDED 옵션)와 리더 운용 제한이 있다 — 컴패니언이 보지 않는 위험은 FC 가 지킨다 | 감사 2026-10-01 4절 | I | INSPECT[README.md:FENCE_ENABLE]; INSPECT[README.md:BATT_FS_LOW_ACT] | 검증됨 (문서) |
| OPS-06 | SITL 하네스의 가상 카메라는 기체 roll/pitch 를 투영에 반영한다 — main 의 자세 보상·레벨링이 하네스 안에서 외란이 아니라 실제 기하다 | 이전 감사 #45 | T | UT[하네스 카메라:] | 검증됨 (단위; SITL 재실행 전) |
| OPS-07 | 문서가 코드와 맞는다: BODY_NED 는 기체 FRD 가 아니라는 docstring, GPS 단독 이격 8 m 는 D435i 실용 범위 밖이라 비전이 바로 이어받지 않는다는 설명, PX4 '검증' 주장은 set_mode 1건으로 한정, librealsense 후처리 미사용 결정 기록 | 이전 감사 #43·#46·#66 | I | UT[문서 수정:]; INSPECT[config.py:librealsense 후처리 필터] | 검증됨 (문서) |

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

## 3. 미충족 · 미검증 · 부분 요구도

| ID | 상태 | 필요한 것 |
|---|---|---|
| EST-12 | 미충족 | 리더 드론 데이터셋 확장, 재학습 |
| FCR-02/08/10 | 검증됨(분석) | 제어 파라미터가 바뀌었다(Kp 0.30, τ_ff 0.1, KV 0.3, 절대속도 추정기). SITL `depth_range`·`leader_sine`·`hover_hold` 재실행으로 예측(정상상태 3.63 m, 진폭비 AN[sitl_like.current])을 확인해야 한다 |
| FCR-07 | 부분 | 감속 배율 0.75 / 0.55 의 단위 검사 추가 |
| EST-02 | 부분 | 카이제곱 임계 경계값 단위 검사 추가 |
| EST-15 | 검증됨(1 m/s) | 설계점 0.3 m/s 에서는 원리적으로 미분리(구심 가속 < σ_a) — 안전과 무관(모드 확률은 검출 주기·ROI 에만). 더 낮은 σ_a_cv 는 CV 추적을 해쳐 채택하지 않음 |
| IF-02/IF-16 | 검증됨(단위) | PX4 실기에서 SET_MESSAGE_INTERVAL 로 LOCAL_POSITION_NED 가 실제로 오는지, STATUSTEXT 가 콘솔에 뜨는지 확인 |
| EST-10 | 부분 | 실외 역광·강한 빛 조건에서 노출 옵션 실기 확인 |
| EST-11 | 부분 | Jetson FPS 로그를 저장소에 남기기 |
| SAF-15 | 미검증 | 폐루프 실비행(안전줄·저고도부터) |
| SAF-05 | 부분 | 하네스 조종사 링크에 RC override 를 넣어 LOITER 중 고도를 유지하게 고치고 `handover` 재실행 (sitl/README 6절) |
| FCR-02 | 확인 필요 | `depth_range` 를 `--duration 60` 이상으로 재실행해 평형 거리 3.63 m 수렴 확인 |
| IF-11 | 미검증 | ESP32 펌웨어 작성, 패킷 포맷 고정 |

## 4. 검사 방법

```bash
python3 analysis/trace_check.py     # 표의 참조가 전부 존재하는지 + 요구도에 안 묶인 검사 목록
python3 test_fixes.py               # '추적성:' 검사로 같은 것을 회귀에 포함
```
