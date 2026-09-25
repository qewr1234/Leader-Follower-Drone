# 실비행 안전 점검서 (2026-09-24)

**한 줄 결론.** 이 저장소의 팔로워는 "명령을 내는 쪽" 이고, 기체를 실제로 지키는 방벽은 세 겹입니다 —
(1) 조종사의 모드 스위치(언제나 이김), (2) ArduCopter 자체의 GUIDED 타임아웃·failsafe·펜스, (3) 컴패니언 코드의 게이트.
아래는 세 겹 각각을 **소스·문서로 확인한 사실**, **오늘 코드에 추가한 방벽과 그 증거**, **지상에서 끝낼 수 있는 점검**,
**비행 단계별 통과/중단 기준** 순서입니다. "기억으로" 쓴 항목은 그렇게 표시했습니다.

돈이 없어 기체가 한 대뿐이라는 전제로, 원칙은 하나입니다: **지상에서 확인할 수 있는 것은 전부 지상에서 확인하고,
공중에서는 한 번에 한 가지만 새로 시험한다.**

---

## 0. 오늘 추가된 방벽 요약

| # | 위험 | 방벽 (코드) | 증거 |
|---|---|---|---|
| G1 | NaN/inf 명령이 `clamp` 를 지나며 **+0.35 m/s 전진** 으로 둔갑 (`max(lo, min(hi, nan))` = hi) | `main.sanitize_cmd` (정지) + `send_body_velocity` 최종 방벽 + EKF 상태 비유한 시 `ImmEkf.reset()` | UT `안전: utils_geometry.clamp` / `안전: sanitize_cmd` / `안전: send_body_velocity`, CL `안전(nan)` |
| G2 | FC HEARTBEAT 가 끊겨도 마지막 모드 문자열 "GUIDED" 가 남아 **조종사가 탈환했는지 모른 채 LAND** 를 보냄 | `FC_MODE_MAX_AGE_SEC` 3 s — 모드 불명이면 LAND 금지, 링크 복구 시 미션 리셋 | CL `안전(lost_alt)`, CL `안전(fc_stale): 3 s` |
| G3 | ATTITUDE/LOCAL_POSITION 이 정체돼도 비전만으로 계속 움직임(자세 보정·수평화·고도 바닥 전부 꺼진 채) | `FC_STATE_HOLD_AGE_SEC` 1 s — 정지 명령 | CL `안전(fc_stale): FC 텔레메트리` |
| G4 | 고도를 모를 때(LOCAL_POSITION 없음) 하강 명령 허용 | 고도 미상 → 하강 차단 (`MIN_AGL_M` 분기 확장) | UT `안전: 상수` |
| G5 | 트래커가 높은 물체를 물거나 리더 고도 오추정 → 끝없이 상승 | `MAX_CLIMB_ABOVE_ENTRY_M` 5 m — 인계 고도 + 5 m 천장 | CL `안전(climb)` |
| G6 | 기체가 기울면(맞바람·가감속) 같은 고도 리더에 상하 명령 — pitch −10° 에 vz −0.092, 팔로워가 D·tan10° = 0.52 m 위로 올라가 정착 | `controller.level_by_attitude` 기본값 **True** 로 전환 | CL `안전(tilt, 수평화 OFF)` (결함 재현) / `안전(tilt, 수평화 ON)` (Δalt 0.005 m) |
| G7 | **G1 의 구멍**: 제어기 입력(피드포워드·상대 상태·슬롯 오차)의 NaN 은 `compute_velocity_cmd_from_estimate` 안의 `clamp` 가 먼저 +0.35 로 바꿔 `sanitize_cmd` 가 볼 수 없었다 (재현: `ff=[nan,0,0] → [0.35, 0, 0, 0]`) | `utils_geometry.clamp` 가 NaN/inf 를 0 으로 자름 + 제어기 입력 검증(정지) + 카메라 뒤(front ≤ 0) 추정이면 정지 + `self_velocity_lpf`·heading 추정기 NaN 고정 방지 | UT `안전: clamp` / `안전: 제어기 입력 검증` / `안전: self_velocity_lpf` / `안전: 상대 heading` |
| G8 | 리더 텔레메트리 JSON 의 `Infinity`/`NaN`/`1e999` 를 `json.loads` 가 받아들여 피드포워드가 NaN 으로 **영구 고정**(inf−inf) → 상한 전진 (`ff_source=broadcast` 설정에서). 단위 오류(cm/s) 속도 힌트가 IMM 을 끌고 가 카메라 관측이 게이트 밖으로 밀려 교정 불가 | 비유한·20 m/s 초과 패킷 폐기, 속도 힌트 20 m/s 상한 | UT `안전: parse_leader_json` / `안전: 속도 힌트 상한` |
| G9 | 속이 빈 드론 기체를 옆에서 보면 bbox 안쪽 55 % 영역의 절반 이상이 배경 → 깊이 **중앙값이 배경(8 m)** → "멀다" → 전속 전진. MAD 는 최대 0.35 라 0.5 문턱을 못 넘고, χ² 게이트는 5.1 s 뒤 그 값을 받아들인다 | 가장 가까운 깊이 무리(0.10 m 빈, 시작 문턱 max(5 %, 30 px)) 의 중앙값. 단단한 표적은 동일 | UT `안전: nearest-mode 깊이` (15~45 % 기체에서 3.0 m; 단순 중앙값은 8 m) |
| G10 | 추적과 무관한 충돌 방벽이 없었다 — 리더가 다가오거나 사람이 끼어들어도 추정에만 의존 | 원시 깊이 중앙 영역의 40 번째 근접 표본 < 1.2 m 면 전진 차단 (`FORWARD_STOP_M`) | UT `안전: 전방 여유` |
| G11 | **자율 LAND 가 양성 상황에서도 엉뚱한 곳에 착륙**: 느린 리더(< 0.25 m/s 는 출발 확인이 안 돼 READY_HOVER 로 깊이창 밖으로 걸어 나감 → 60 s 에 LAND), 사람 리더가 앉음(z<0.65·하강·정지 → 16 s 에 착륙 판정), EKF 원점 1 m 오차(1.6 m 에서 리더가 0.45 m 내려오면 착륙 판정), 하늘 배경 깊이 10 s 소실 | `mission.autonomous_land` 정책, **기본 False** = 0 속도 유지 + STATUSTEXT. 조종사가 착륙한다 | CL `정책(autonomous_land=False` |
| G12 | **조종사 탈환 경주**: heartbeat 는 1 Hz 고정이라 조종사가 LOITER 로 바꾼 뒤 최대 1 s 동안 컴패니언은 GUIDED 로 알고 LAND 를 보내 조종사를 덮어쓴다. 2 s 재시도는 조종사가 잠깐 되찾은 GUIDED 도 덮어썼다 | LAND 는 결정당 1회, **결정 뒤에 받은 heartbeat 가 GUIDED** 일 때만. 재시도 없음(먹지 않으면 FC 가 GUID_TIMEOUT 뒤 위치 유지) | CL `안전(takeover)` (1 Hz heartbeat, 34.6 s 탈환 → LAND 0회) |
| G13 | 벽시계(`time.time()`) 로 dt·소실 타이머를 재서 NTP/chrony 가 +10 s 튀면 그 프레임에 FAILSAFE_LAND | 내부 시계 `time.monotonic()` (벽시계는 MAVLink time_boot_ms 와 로그에만) | UT `안전: 단조 시계` |
| G14 | 검출기(TensorRT)·스케줄러 예외, 자세 inf(`math.cos(inf)`) 가 루프를 죽여 조종사 모르게 컴패니언이 사라짐. 'l' 키가 조종사 모드를 덮어씀. 종료 시 정지 1회가 유실되면 FC 가 마지막 속도를 3 s 유지 | 예외 격리(미검출·전체 프레임), 자세 유한성 검사, 'l' 은 GUIDED·모드 기지일 때만, 'v' 끄기 전 정지, 종료 시 정지 3회 + STATUSTEXT, 미션 상태 변화마다 STATUSTEXT | UT `안전: STATUSTEXT`, INSPECT |
| G15 | ATTITUDE 10 Hz 라 25~30 fps 의 프레임 3개 중 2개가 같은 자세를 보고 계단식으로 튄다(돌풍 pitch ±10°/0.7 Hz: 0.15~0.39 m 위치·최대 0.58 m/s 가짜 리더 속도). 자세가 0.3 s 정체되면 그 사이 회전을 영영 안 보정 | `MAV_CMD_SET_MESSAGE_INTERVAL` 로 ATTITUDE 30 Hz, 정체 뒤 첫 신선한 자세에서 누적 회전 보정 | UT `안전: ATTITUDE 30 Hz` |
| G16 | ESP32 상대위치를 오래된 yaw 로 회전; 원거리 깊이(8~10 m, σ 0.3~0.4 m) 를 σ 0.25 로 과신; bbox 가 프레임 가장자리에 잘리면 중심이 치우침; 한 프레임 놓친 사이 다른 거리의 사람이 트랙을 가져감; `set_region_of_interest` 가 초당 ~140 ms 스톨 가능 | yaw 0.3 s 신선도 게이트, σ_z = max(0.25, 0.006 z²), `truncated` 신뢰도 ×0.6, 회복 시 bbox 크기 비 1.3 게이트, `ae_roi_follow_track` 기본 False + 5 ms 초과 경고 | UT `안전: 팔로워 자세 신선도` / `안전: 스테레오 R` / `안전: 트래커 크기 게이트` |

폐루프 골든 스트림(`test_closed_loop.py --compare`)은 35 s 의 착륙 결정까지 **351 개 setpoint 전부 바이트 단위로 동일**하고,
그 뒤만 정책대로(LAND 대신 0 속도 유지) 달라집니다 — 정상 비행 경로의 명령은 바뀌지 않았고 방벽은 비정상 상황에서만 작동합니다.
단위 169 개 · 폐루프 25 개(시나리오 8 종) · 요구도 86 개(추적 0 오류).

```bash
python3 test_fixes.py
python3 test_closed_loop.py                      # 골든 (기본 시나리오, autonomous_land=False)
python3 test_closed_loop.py --autonomous-land 1  # 예전 정책: 35 s 에 LAND 1회
for s in "tilt --level 0" tilt nan fc_stale "climb --frames 1900" "lost_alt --autonomous-land 1" "takeover --autonomous-land 1"; do python3 test_closed_loop.py --scenario $s; done
```

---

## 1. FC(ArduCopter 4.5) 가 실제로 하는 일 — 소스에서 확인

기체를 지키는 두 번째 겹입니다. 모두 `ArduPilot/ardupilot` 브랜치 `Copter-4.5` 에서 직접 읽었습니다.

| 상황 | ArduCopter 의 동작 | 근거 |
|---|---|---|
| 컴패니언이 **0 속도** setpoint 를 보냄 | 속도 목표를 위치 목표로 적분하고 **위치 안정화까지** 하는 `input_vel_accel_xy` 로 들어간다(`GUID_OPTIONS` 비트 4·5 가 꺼진 기본값). 즉 **Loiter 와 같은 위치 제어기가 바람을 이긴다** — 호버 품질은 FC 의 것이다 | `ArduCopter/mode_guided.cpp` `velaccel_control_run()`, `stabilizing_pos_xy()` |
| 컴패니언이 **죽거나 멈춤** (setpoint 중단) | `GUID_TIMEOUT`(기본 3 s) 뒤 속도·가속도 목표 0, yaw 회전 정지 → 그 자리 호버 | `mode_guided.cpp` `velaccel_control_run()` 의 `tnow - update_time_ms > get_timeout_ms()` |
| GUIDED **진입** 순간 | 속도 0 으로 velaccel 서브모드 시작 (첫 setpoint 전까지 정지) | `mode_guided.cpp` `ModeGuided::init()` |
| setpoint 에 **NaN 또는 \|v\| > 1000** | 거부하고 `mode_guided.init(true)` → 정지 | `ArduCopter/GCS_Mavlink.cpp` `sane_vel_or_acc_vector()`. **단, 우리 `clamp` 가 NaN 을 +0.35 로 먼저 바꾸므로 이 방벽은 절대 발동하지 않는다 → G1** |
| `BODY_NED` 프레임 | 속도 x, y 를 yaw 로만 회전(`rotate_body_frame_to_NE`), z 는 그대로 | 같은 핸들러. 그래서 제어 오차가 수평 프레임이어야 한다 → G6 |
| **모드 스위치** (조종사 탈환) | 다른 모드로 가면 setpoint 를 무시한다(`if (!in_guided_mode()) break;`). 컴패니언은 그 모드에서 LAND 를 보내지 않는다(C2) | 같은 핸들러; `main.py` `fc_accepts_setpoints` |
| 스위치로 GUIDED 를 켰는데 **위치 추정이 없음** | 모드 변경 자체가 거부된다("requires position") → 이전 모드 유지 | `ArduCopter/mode.cpp` `Copter::set_mode()` `requires_GPS() && !position_ok()` |
| **RC 수신 끊김** (GUIDED 중) | `FS_THR_ENABLE` 동작(기본 1 = RTL, GPS 없으면 LAND). `FS_OPTIONS` 비트 2 를 켜면 GUIDED 를 계속한다 — **켜지 말 것** (조종사 없이 컴패니언만 남는다) | `ArduCopter/events.cpp` `failsafe_radio_on_event()`; 위키 `radio-failsafe.rst` |
| **GCS 링크 끊김** | `SYSID_MYGCS` 의 heartbeat 를 한 번이라도 본 뒤에만 감시. 컴패니언은 heartbeat 를 보내지 않으므로 GCS 로 취급되지 않는다 | `events.cpp` `failsafe_gcs_check()` `sysid_myggcs_last_seen_time_ms()`; 위키 `gcs-failsafe.rst` |
| **배터리** | `BATT_FS_LOW_ACT` (위키 권장 2 = RTL, 불가 시 LAND), `BATT_FS_CRT_ACT`. 이미 LAND 중이면 계속 | `events.cpp` `handle_battery_failsafe()`; 위키 `failsafe-battery.rst` |
| **EKF 분산 초과** | `FS_EKF_ACTION` 기본 1 = LAND | `ArduCopter/config.h` `FS_EKF_ACTION_DEFAULT`; `Parameters.cpp` |
| 컴패니언의 **sysid** | `SYSID_ENFORCE` 기본 0 → pymavlink 기본 255 로 보내도 받는다. 1 로 바꾸면 `SYSID_MYGCS` 와 달라 **조용히 무시** 된다 | `Parameters.cpp` `SYSID_ENFORCE`; `GCS_Mavlink.cpp` `sysid_enforce()` |
| **LAND** 모드 | 10 m(`LAND_ALT_LOW`) 아래 `LAND_SPEED` 50 cm/s 로 하강, 착지 감지 후 스로틀 최저면 자동 disarm(`DISARM_DELAY`). 조종사는 롤/피치로 착지점을 옮길 수 있다(`LAND_REPOSITION`=1) | 위키 `land-mode.rst`; `ArduCopter/land_detector.cpp` |
| **펜스** | `AVOID_ENABLE` 이 켜져 있으면 GUIDED 속도 목표를 펜스 앞에서 FC 가 직접 깎는다(`copter.avoid.adjust_velocity`) — 컴패니언이 밀어도 펜스를 넘지 않는다 | `mode_guided.cpp` `velaccel_control_run()` `#if AC_AVOID_ENABLED` |
| **추락 감지** | 2 s 동안 기울기 오차 > 30° 이고 가속 없음 → disarm(`FS_CRASH_CHECK`=1 기본) | 위키 `crash_check.rst` |

**의미.** 컴패니언이 어떤 식으로 죽든(예외·정지·케이블 빠짐) 3 s 안에 기체는 스스로 호버로 돌아가고, 조종사는 스위치 한 번으로
언제든 되찾습니다. 컴패니언이 낼 수 있는 최악은 (a) **잘못된 방향으로 최대 0.35 m/s** 로 밀거나, (b) **엉뚱한 곳에 LAND** 를
보내는 것 둘뿐입니다. 아래 점검은 이 둘을 막는 데 집중합니다.

---

## 2. FC 파라미터 (근접 편대 시험용)

| 파라미터 | 값 | 이유 · 근거 |
|---|---|---|
| `GUID_TIMEOUT` | 3 (기본) | 컴패니언 정지 → 3 s 뒤 호버. 더 짧게(1~2) 두면 카메라 스톨 한 번에 호버·재개가 반복된다. `Parameters.cpp` |
| `GUID_OPTIONS` | 0 (기본) | 비트 4/5 를 켜면 위치 안정화가 꺼져 **바람에 흘러간다**. `mode_guided.cpp` `stabilizing_pos_xy()` |
| `FS_OPTIONS` | 0 | 비트 2("GUIDED 에서 RC failsafe 무시") **절대 켜지 말 것**. `events.cpp` |
| `FS_THR_ENABLE` | 1 (RTL, 기본) 또는 3 (LAND) | RC 끊김. RTL 은 `RTL_ALT`(기본 15 m) 로 올라가 집으로 간다 — 리더 드론이 그 경로·고도에 있으면 3 (LAND) 이 안전. 위키 `radio-failsafe.rst` |
| `RTL_ALT` | 리더 운용 고도 + 5 m 이상 | RTL 이 리더 고도를 가로지르지 않게. `config.h` 기본 1500 cm |
| `FS_GCS_ENABLE` | 0 (기본). 노트북 GCS 를 쓰면 1 | 컴패니언은 GCS 가 아니다. 1 이면 노트북 텔레메트리 끊김에 RTL. 위키 `gcs-failsafe.rst` |
| `SYSID_ENFORCE` | 0 (기본) | 1 이면 컴패니언 setpoint 가 조용히 무시된다 |
| `BATT_LOW_VOLT` / `BATT_LOW_MAH` / `BATT_FS_LOW_ACT` | 셀당 3.5 V / 용량 20 % / 2 (RTL) 또는 1 (LAND) | 위키 `failsafe-battery.rst` 권장. `BATT_FS_CRT_ACT`=1 |
| `FS_EKF_ACTION` | 1 (LAND, 기본) | `config.h` |
| `FENCE_ENABLE` / `FENCE_TYPE` / `FENCE_ALT_MAX` / `FENCE_RADIUS` / `FENCE_ACTION` | 1 / 3 (고도+원) / 30 m / 50~80 m / 1 (RTL 또는 LAND) | GUIDED 속도가 펜스에서 깎인다(`AVOID_ENABLE` 비트 0 켜기). 컴패니언 천장(+5 m) 의 바깥 방벽 |
| `AVOID_ENABLE` | 1 (펜스) | `mode_guided.cpp` `copter.avoid.adjust_velocity` |
| `WP_YAW_BEHAVIOR` | 0 | README 운용 순서 (C5) |
| `LAND_SPEED` | 30~50 cm/s | 위키 `land-mode.rst`. 처음엔 30 |
| `DISARM_DELAY` | 10 (기본) | 착지 후 자동 disarm |
| `FS_CRASH_CHECK` | 1 (기본) | 위키 `crash_check.rst` |
| `ARMING_CHECK` | 1 (전부) | pre-arm 검사를 끄지 말 것 |
| `RCx_OPTION` | 31 (Motor Emergency Stop) 를 남는 스위치에, 비행 모드 스위치에 LOITER·ALT_HOLD·GUIDED | 지상 사고 시 즉시 정지. `libraries/RC_Channel/RC_Channel.h` `MOTOR_ESTOP = 31` |
| `SERIAL2_PROTOCOL` / `SERIAL2_BAUD` | 2 / 921 | 컴패니언은 USB(`/dev/ttyACM0`) 보다 **TELEM2 UART** 권장 — USB 허브 리셋·전원 문제에 강하다. 위키 `raspberry-pi-via-mavlink.rst` (TELEM2, 921600) |

컴패니언 쪽 상수(`main.py`) 첫 비행 값:

| 상수 | 지금 | 첫 비행 권장 | 이유 |
|---|---|---|---|
| `MAX_VX / MAX_VY / MAX_VZ` | 0.35 / 0.22 / 0.12 | **0.15 / 0.08 / 0.05** (README 권장 그대로) | 최악의 오명령이 사람이 걸어서 피할 속도가 되게 |
| `MAX_YAW_RATE` | 0.35 rad/s | 0.2 | 시야 이탈 방지에는 충분 |
| `TARGET_DISTANCE_M` | 3.0 | 3.0~4.0 | 깊이창(10 m) 여유는 C4 대로 |
| `MIN_AGL_M` | 1.5 | 2.0 | 지면 효과·착륙 판정 여유 |
| `MAX_CLIMB_ABOVE_ENTRY_M` | 5.0 | 3.0 | 첫 비행은 고도 변화 자체를 시험하지 않는다 |
| `SEND_MAVLINK_COMMANDS` | False | 지상 점검 전부 통과 뒤에만 True | SAF-07 |

---

## 3. 지상 점검 (프로펠러 제거, 순서대로, 전부 통과해야 다음)

각 항목은 `[STAT]` 한 줄 또는 화면 HUD 로 확인합니다. **B-3 기울임 점검이 오늘 바뀐 기본값(수평화 ON)의 실기 검증**입니다.

| ID | 점검 | 통과 기준 | 실패 시 |
|---|---|---|---|
| B-1 | FC 링크·스트림 | `[STAT] ... fresh=LP1/ATT1/HB1 rx[Hz] HB=1 LP=10 ATT=10 GP=10` 이 10 초 이상 유지 | 포트·baud·`SERIAL2_PROTOCOL` 확인. `fresh` 가 0 이면 컴패니언은 정지 명령만 낸다(G3) |
| B-2 | 모드 반영 | 조종기 스위치로 LOITER↔GUIDED 를 바꿀 때 `[STAT] FC=` 가 1 s 안에 따라오고, GUIDED 진입에 `[SYS] GUIDED 진입 — 미션/명령 리셋` 이 찍힌다 | `is_fc_heartbeat` 컴포넌트 고정 로그(`[FC] autopilot component=1 로 고정`) 확인 |
| B-3 | **기울임 점검** (수평화 부호) | 정지 표적을 3 m 정면 같은 높이에 두고 기체 기수를 손으로 10° 내린다(앞으로 숙임). HUD `rel F/R/U` 의 U 가 **±0.1 m 안에 머물고** `cmd vz` 가 ±0.02 안. `level_by_attitude=False` 로 바꿔 다시 하면 U ≈ +0.5 m, vz ≈ −0.09 가 나와야 한다(부호 규약 확인). 우측으로 10° 기울여도 R 이 그대로 | 부호가 반대면(기울일수록 U 가 커짐) ATTITUDE pitch 부호 확인 — 기수 하향이 음수여야 한다(MAVLink 규약). 해결 전 비행 금지 |
| B-4 | 명령 방향 | 표적을 앞뒤로 1 m 옮길 때 `cmd vx` 부호(멀어지면 +), 좌우로 옮길 때 `vy` 와 `yr` 부호(오른쪽이면 +), 위로 들면 `vz` 음수 | SITL 로 검증된 체인이지만 카메라 장착 방향이 바뀌면 깨진다 |
| B-5 | 표적 신원 | 시야에 사람이 둘 있을 때 어느 쪽을 무는지(`_choose_initial` 은 면적×신뢰도 최대). **비행장에 리더 외 사람이 시야에 없게 운영** | 검출 클래스 `person` 인 동안은 운영으로 막는 수밖에 없다 (4절) |
| B-6 | 깊이 상식 검사 | 표적 2 / 3 / 5 m 에서 HUD `raw=` 와 `est=` 가 줄자와 ±0.15 m. `rD`(깊이 신뢰도) > 0.5. **드론 표적이면** 3 / 5 / 8 m 를 하늘·나무 배경 각각에서 30 s 씩 기록해 로그 `measurement.depth_valid_count`·`depth_m` 분포 확인(4.2). `fwd=` 가 손을 1 m 앞에 대면 1.0 근처로 떨어지고 `[WARN] 전방 … 차단` 이 찍힘 | 하늘 배경·역광에서 `raw=N/A` 가 잦으면 4절의 깊이 소실 시간 참고 |
| B-7 | FPS | `MARS_SHOW_WINDOW=0`, `sudo nvpmodel -m 0 && sudo jetson_clocks` 뒤 `[STAT] FPS=` ≥ 25 를 60 초. `[YOLO] backend=TensorRT device=cuda:0`. `rx[Hz] ATT=30` 확인(30 Hz 요청이 먹었는지). `ae_roi_follow_track` 을 켜 보고 `[CAM] AE ROI 설정 … ms` 경고가 나오면 끈 채로 비행 | `.pt` 폴백·CPU 면 README 배포 절 |
| B-8 | 소실·착륙 로직 | 표적을 가리고 `mission=LOST_HOLD` → 10 s 뒤 `FAILSAFE_LAND` 와 `[FC] set mode: LAND` 가 **CMD=DRY 에서는 나가지 않음** 확인. 이후 CMD ON + 조종기 LOITER 상태에서 같은 시험 → LAND 가 **안 나감**(C2) | |
| B-9 | 헤드리스·종료 | `q` 로 종료 시 `[SYS] shutdown` 뒤 hold 1회. SSH 끊김 시 프로세스가 죽지 않게 `nohup`/`systemd` 또는 `tmux` 로 실행 | SSH 세션이 죽으면 컴패니언도 죽는다 → 3 s 뒤 호버(1절) 이지만 그 뒤 추종은 없다 |
| B-10 | RC failsafe 시험 | 위키 `radio-failsafe.rst` 의 Test #1~#3 (프롭 제거, 송신기 끔) — GCS HUD 에 "Radio Failsafe" 와 설정한 동작 | |
| B-11 | 킬스위치 | `MOTOR_ESTOP` 스위치로 arm 상태에서 모터 정지 확인 | |
| B-12 | 진동·호버 스로틀 | 위키 "First Flight with Copter" 의 진동 측정·호버 스로틀은 **조종사 단독 비행(F-0)** 에서 끝내 둔다 | |
| B-13 | UWB 거리 GT (논문용, 선택) | `[STAT] uwb=` 가 10 Hz 로 갱신되고 정적 교정(EXPERIMENT_PROTOCOL 3절) 의 σ ≤ 0.10 m | GT 없이도 비행은 되지만 논문 데이터는 안 나온다 |

---

## 4. 인지 · FPS · 바람

세 질문("25 FPS 가 나오나", "잘 탐지하나 — 특히 **멀다고 오판**하나", "바람에 흔들리면") 에 대한 답입니다. 수치는 이 저장소의
실제 모듈을 돌린 합성 실험이고, Jetson 하드웨어 수치는 기억(표시)입니다.

### 4.1 FPS — 25 는 Orin 급이면 창 끄고 가능, Xavier NX 급이면 딱 경계

| 항목 | 값 | 근거 |
|---|---|---|
| 인지 제외 파이썬 전체(수신·EKF·스케줄러·측정·미션·제어·평활·로그) | **0.62 ms/프레임** (로그 끔) / **1.14 ms** (로그 켬), x86 2.1 GHz | 폐루프 하네스 1200 프레임 실측 |
| `_depth_stats` 640×480, bbox 400×400 | 0.31 ms (서브샘플) | 마이크로벤치 |
| 화면(그리기 + imshow + waitKey) | Jetson 4~8 ms, 2프레임마다 | 개발자 보고(README) |
| `rs.align` CPU (pip 휠) | 6~10 ms Orin / 10~20 ms Xavier NX (기억) | librealsense 는 `BUILD_WITH_CUDA=true` 로 빌드하면 GPU |
| YOLO11n TRT FP16 416 (전·후처리 포함) | 10~15 / 15~22 ms (기억) | |
| `set_region_of_interest` (AE ROI) | **호출당 ~140 ms 보고** (librealsense #7130, Windows 측정) | 그래서 `ae_roi_follow_track` 기본 False, 5 ms 초과 시 경고 |

루프가 33 ms 를 넘으면 카메라 프레임을 버리고(librealsense 파이프라인은 용량 1 큐라 **항상 최신 프레임** 을 준다 — `src/pipeline/aggregator.cpp`
읽음 — 지연이 쌓이지는 않는다), 40 ms 를 넘으면 25 FPS 아래입니다. 개발자 실측 24~26 FPS 는 Xavier NX 급에 창을 켠 값으로
보이며, **창 끄기(`MARS_SHOW_WINDOW=0`) + `jetson_clocks` + AE ROI 끔** 이면 여유가 생깁니다.

**FPS 가 제어 안정성에 미치는 영향은 작습니다.** 외루프의 지연 여유는 6.3 s (`docs/stability_margins.json` `delay_margin_s`)라
15 FPS 로 떨어져도 불안정해지지 않습니다. FPS 가 낮아서 생기는 문제는 **트래커** 쪽입니다: IoU 0.20 게이트는 3 m 에서 리더가
1 m/s 로 가로지를 때 15 fps 까지 버티고(검출 2프레임마다), 2 m/s 는 25 fps 에서도 놓칩니다. 놓친 한 프레임이 표적 신원을
잃는 창이므로(4.3), FPS 는 안정성이 아니라 **신원 유지** 를 위해 필요합니다.

### 4.2 깊이 — "멀다" 오판이 진짜 위험이었다 (G9)

사람은 단단해서 중앙값이 맞지만, 드론은 속이 비어 있어 bbox 안쪽 55 % 영역의 절반 이상이 배경입니다. 합성 실험(100×100 bbox,
기체 3 m / 지면 8 m, D435 급 잡음):

| 안쪽 영역 중 기체 비율 | 단순 중앙값 | MAD | 가장 가까운 무리 (지금) |
|---|---|---|---|
| 15 % | 8.0 m | 0.1 | **3.0 m** |
| 30 % | 8.0 m | 0.2 | **3.0 m** |
| 45 % | 8.0 m | 0.35 | **3.0 m** |
| 60 % | 3.0 m | 0.35 | 3.0 m |
| 100 % (사람) | 3.0 m | 0.03 | 3.0 m (동일) |

MAD 0.5 문턱은 두 봉우리에서 절대 안 넘고, χ² 게이트는 8 m 를 5.1 s 동안 거부하다가 받아들입니다 — 그 사이 `range_coast` 2 s 로
LOST_HOLD(정지) 가 먼저 걸리지만, 5.1 s 에 8 m 가 받아들여지면 `has_followed` 라 즉시 FOLLOW → 오차 4.8 m → **+0.35 m/s 전속 전진**,
리더는 3 m 앞에 있습니다. 350 급 기체를 옆에서 3 m 에서 보면(fx≈615) bbox ≈ 72×31 px, 안쪽 40×17 px 중 100 mm 몸통이 ≈ 50 % —
정확히 경계입니다. 반대 방향(1.2 m 의 잡동사니에 걸림)은 "가깝다" 쪽 오판이라 후퇴하므로 안전한 쪽입니다.

실기에서 남는 것: 하늘 배경에서 작은 기체의 스테레오 매칭이 드물어 `depth_valid_count` 20 근처를 오가며 1~3 s 씩 끊길 수 있습니다.
그때는 LOST_HOLD(정지) 이고, 10 s 넘으면 FAILSAFE_LAND 지만 **기본 정책은 호버 유지**입니다(G11). 비행 전 지상에서 리더 기체를
3 / 5 / 8 m 에 두고 하늘·나무 배경 각각에서 로그의 `measurement.depth_valid_count` / `depth_m` 분포를 봐 두십시오(3절 B-6).

### 4.3 표적 신원 — 사람 단계에서는 운영으로 막는다

초기 획득은 면적×신뢰도 최대라 3 m 의 리더(0.7)보다 2 m 의 행인(0.5)이 이깁니다. 리더가 보이는 동안은 IoU 0.9 가 항상 이겨
빼앗기지 않지만, **검출을 한 프레임 놓친 순간** 근접 반경(대각선 1.5 배 = 3 m 의 사람에게는 화면 전체) 안의 다른 사람이 트랙을
가져갑니다. 지금은 회복 시 bbox 크기 비 1/1.3~1.3 을 요구해 **다른 거리의 사람** 은 막지만, 같은 거리에 나란히 선 사람은 못 막습니다.
따라서 사람 리더 단계의 규칙은 하나입니다: **카메라 시야에 리더 외 사람이 없을 것.** 관찰자·조종사는 팔로워 뒤에 섭니다.
드론 표적 단계에서는 다른 드론·새가 같은 역할입니다.

### 4.4 바람 · 돌풍

- **호버 품질은 FC 의 것입니다.** 0 속도 setpoint 는 위치 안정화를 포함한 Loiter 와 같은 제어기로 들어갑니다(1절). 컴패니언은
  바람을 이기려 하지 않습니다.
- **기울기 편향(G6).** 맞바람에 기울어 호버하면 카메라도 기웁니다. 수평화 없이는 리더가 D·tan θ 만큼 위/아래로 보여 그만큼
  고도가 밀립니다(10° 에 0.52 m, 5° 에 0.26 m, GPS 단독 8 m 이격이면 1.4 m). 수평화가 기본이므로 남는 것은 ATTITUDE 지연에 의한
  과도 오차뿐입니다.
- **돌풍 응답(G15).** pitch ±10° / 0.7 Hz(최대 44°/s) 의 돌풍 응답을 10 Hz 자세로 보정하면 EKF 에 0.15~0.39 m 위치·최대 0.58 m/s
  의 가짜 리더 속도가 들어가고(출발 확인 문턱 0.25 m/s 를 넘음), 30 Hz 면 0.07 m·0.02 m/s 입니다. 그래서 30 Hz 를 요청합니다.
  카메라 영상이 자세보다 35~50 ms 늦다는 지연 정렬 문제는 남아 있습니다(남은 위험 7절).
- **시야.** 640×480 컬러는 16:9 센서의 4:3 크롭이라 실제 FOV 는 약 55°×43° (fx≈615) 입니다 — README 의 "69°" 는 16:9 규격입니다.
  3 m 의 사람은 세로 348/480 px 라 pitch 6° 만 넘어도 머리·발이 잘립니다(`truncated`, 신뢰도 ×0.6). 표적이 중앙에서 화면 밖으로
  나가는 각은 yaw 27.5° / pitch 21.3° — 30°/s 돌풍이면 0.7~0.9 s 입니다. 시작 로그 `[CAM] fx=` 로 실제 값을 확인하십시오.
- **첫 비행 풍속 상한: 5 m/s** (조종사 판단, 기억이 아니라 위 편향·시야 수치에서 나온 보수값).

---

## 5. 미션 · FC 상호작용 — 검토에서 나온 결정 사항

| # | 발견 | 결정 |
|---|---|---|
| F1 | 자율 LAND 가 양성 상황(느린 리더, 사람이 앉음, EKF 원점 오차, 하늘 배경 깊이 소실)에서 엉뚱한 곳에 착륙한다. 하네스 재현: 리더 0.09~0.21 m/s → 60 s 에 LAND; 사람이 0.12 m/s 로 앉음 → 16 s 에 LAND; 원점 1 m 오차 + 리더 0.45 m 하강 → 14.7 s 에 LAND | **기본 정책 = 호버 유지 + STATUSTEXT** (`mission.autonomous_land=False`). 배터리·EKF failsafe 는 FC 가 맡는다(2절). 자율 LAND 가 필요한 무인 운용은 이 스위치와 아래 F2 규칙으로 켠다 |
| F2 | heartbeat 1 Hz 라 조종사 탈환 뒤 ≤ 1 s 창에 LAND 가 나가 LOITER 를 덮어쓴다. 2 s 재시도는 조종사가 잠깐 되찾은 GUIDED 도 덮어쓴다(더블 플릭). ArduPilot 은 스위치 **변화** 에만 모드를 바꾸므로 조종사는 스위치를 옮겼다 되돌려야 한다 | LAND 는 결정당 1회, 결정 **뒤** 의 GUIDED heartbeat 요구, 재시도 없음. 조종사 절차: 반응 없으면 스위치를 다른 위치로 옮겼다 되돌린다. `PILOT_THR_BHV` 비트 1(=2) 로 스로틀만 올려도 LAND 를 취소할 수 있다(`mode.cpp` `land_run_horizontal_control`) |
| F3 | (HEAD) 하트비트 끊김 뒤 마지막 모드 문자열로 LAND 5회 송신; LOCAL_POSITION 정체 시 1.5 m 바닥을 뚫고 0.51 m 까지 하강 | G2·G3·G4 로 닫힘 (하네스 `fc_stale`, `lost_alt`, `climb`) |
| F4 | 'l' 키가 FC 모드와 무관하게 LAND 를 보냄; 'v' 끄기가 마지막 속도를 3 s 남김; 키는 창에 포커스가 있을 때만 읽힘 | 'l' 은 GUIDED·모드 기지일 때만, 'v' 끄기 전 정지 1회. **비행 중 조작은 조종기 스위치로만** — 키보드는 비상 수단이 아니다 |
| F5 | 거리 코스팅 2 s 동안 예측·피드포워드로 눈 감고 비행(최대 0.35 m/s). 리더가 멈추면 3.60→2.98 m, 후진하면 2.41 m 까지 접근 | 전방 1.2 m 정지(G10) 가 마지막 방벽. 코스팅 중 피드포워드 감쇠는 골든 스트림을 바꾸므로 보류 — 7절 |
| F6 | "AGL" 은 지형이 아니라 EKF 원점(전원 후 **첫 arm** 지점, `AP_Arming.cpp` `resetHeightDatum`) 기준. 착륙 뒤 다른 높이에서 재 arm 하면 z≠0. 경사 5° 에서 20 m 이동 = 1.7 m | `MIN_AGL_M` 2.0 m. 운용: **평지 이륙 지점에서 전원당 한 번 arm**. 하향 거리계(`RNGFND1_*`)를 달면 `DISTANCE_SENSOR` 로 대체 가능(미구현) |
| F7 | 컴패니언 상태를 조종사가 볼 창구가 없고, 종료 시 정지 1회가 유실되면 FC 가 마지막 속도를 GUID_TIMEOUT 까지 유지(최대 1.05 m) | STATUSTEXT(미션 상태 변화·GUIDED 진입·정체·전방 정지·LAND·종료) + 종료 시 정지 3회. `GUID_TIMEOUT` 1.0~1.5 s 권장(2절) — 표류 0.35~0.5 m |
| F8 | 컴패니언 MAVLink 신원이 sysid 255 / comp 0 (GCS 로 위장) | 유지. `SYSID_ENFORCE=0` 이면 문제 없고, 바꾸면(1/191) 실기 확인 없이 링크 신원을 건드리는 셈이라 첫 비행 뒤 검토 |
| F9 | 출발 확인(0.25 m/s × 0.7 s)이 느린 리더를 영영 READY_HOVER 에 둔다 | 기본 정책이 호버 유지라 결과는 양성(리더가 시야를 벗어나면 호버). 리더는 **0.3 m/s 이상으로 분명히 출발** 한다 |
| F10 | GUIDED 에서 지상 arm 시 setpoint 스트림이 있어도 이륙하지 않음(`is_disarmed_or_landed`) — 안전 | `GUID_OPTIONS` 비트 0(송신기 arm 허용) 은 끈 채로. 이륙은 수동, 인계는 공중에서 (README 운용 순서) |

---

## 6. 비행 단계 (한 번에 한 가지만)

모든 단계 공통: 바람 5 m/s 이하(3 절 B-12 뒤 조종사 판단), 리더 외 사람은 카메라 시야 밖, 착륙 가능한 평지, 조종사 손은 모드 스위치에.
**중단 = 스위치를 LOITER 로.** 반응이 없으면 스위치를 다른 위치로 옮겼다가 다시 LOITER(ArduPilot 은 스위치 **변화** 에만 모드를 바꾼다).

| 단계 | 내용 | 통과 기준 | 새로 시험하는 것 |
|---|---|---|---|
| F-0 | 조종사 단독 LOITER 호버 3 분 (컴패니언 OFF) | 위치 유지 ±1 m, 진동 정상, 호버 스로틀 확인 | 기체 자체 |
| F-1 | 컴패니언 ON(CMD ON), **리더 없이** 5 m 에서 LOITER → GUIDED | `mission=WAIT_LEADER`, 기체가 그 자리 호버(FC 위치 안정화). 30 s 뒤 LOITER 로 탈환 → 즉시 반응 | GUIDED 인계·0 속도 호버·탈환 |
| F-2 | F-1 상태에서 **컴패니언 프로세스를 강제 종료** (`kill`) | 3 s 안에 아무 변화 없이 호버 유지(`GUID_TIMEOUT`), LOITER 탈환 | 컴패니언 사망 = 호버 |
| F-3 | 정지 리더(사람) 3~4 m 정면 → GUIDED | `READY_HOVER`, 명령 0, 기체 정지. 리더가 2 m 옆으로 걸어가도 명령 0(출발 확인 전) | 인지만 켠 상태 |
| F-4 | 리더가 0.3 m/s 로 5 m 직진 후 정지 (`MAX_VX` 0.15) | `FOLLOW` → 기체가 따라 이동 → `LEADER_HOVER`, 거리 3~4.5 m 에 정착, 2.3 m 안쪽 접근 없음 | 추종 자체 |
| F-5 | 리더 좌우 이동·U 턴 | yaw 가 리더를 화면 중앙에 유지, 거리 유지 | 측면·기수 축 |
| F-6 | 가림 2 s (리더가 나무 뒤) | `LOST_HOLD` 뒤 재획득 즉시 재개(READY_HOVER 로 안 떨어짐) | 소실·재개 |
| F-7 | 영구 소실 (리더가 시야 밖으로) | 10 s 뒤 `LAND`, 착륙점이 안전한지 **미리 확인한 장소에서만** | 자동 착륙 |
| F-8 | 속도 상한 복원(0.35/0.22/0.12), F-4~F-6 반복 | 동일 | 정격 속도 |
| F-9 | 리더 = 드론(정지 호버) | 검출 클래스 전환 뒤 F-3~F-7 을 다시. 4 절의 깊이 소실 시간 확인 | 드론 표적 |

각 단계 뒤 `logs/*.jsonl` 을 저장하고, `control.body_vx` 최대값·`mission.state` 이력·`reliability.depth` 분포를 보고 다음 단계로 갑니다.

**중단 기준 (즉시 LOITER):** 거리 2 m 이내 접근 · 기체가 리더 방향과 무관하게 움직임 · `fresh=LP0/ATT0` 이 2 s 이상 ·
FPS < 15 · HUD `est=` 가 줄자와 1 m 이상 차이 · 예상 못 한 `set mode: LAND` · 배터리 30 % 이하.

---

## 7. 남은 위험 (지상에서 못 없앤 것) — 우선순위 순

1. **실비행 0회.** 모든 것이 하네스·SITL·소스 근거다. 6절의 단계를 건너뛰지 말 것.
2. **ATTITUDE 부호 규약(수평화)** 은 3절 B-3 지상 기울임 점검으로만 확인된다. 반대면 기울일수록 편향이 2배가 된다.
3. **드론 표적의 깊이 소실 통계** (하늘 배경, 8 m 너머) 를 모른다 — 3절 B-6 지상 측정 뒤 `depth_max_m`·`range_coast_max_sec` 재검토.
4. **컬러 AE 노출 상한 8 ms 가 무효일 가능성.** librealsense 소스에서 `auto_exposure_limit` 은 깊이 센서에만 등록된다 — 시작 로그에
   `[CAM] 컬러 AE 노출 상한` 이 없으면 33 ms 노출로 yaw 0.35 rad/s 에 7 px 블러가 생겨 검출을 한 프레임 놓치는 원인이 된다(4.3 의 창).
   대안: 수동 노출(`enable_auto_exposure 0`, 8 ms, 게인 상향).
5. **영상–자세 지연 정렬.** 30 Hz 자세도 영상보다 새것이라 돌풍 중 0.17 m·0.27 m/s 급 과도 오차는 남는다. `frame.get_timestamp()` 와
   자세 링 버퍼로 보간하는 것이 다음 단계.
6. **같은 거리의 두 사람** 은 트래커가 구분하지 못한다 — 운영 규칙(시야에 리더만).
7. **코스팅 2 s 동안의 맹목 비행** (F5). 피드포워드를 코스팅 중 0.5 s 로 감쇠하는 변경은 골든 스트림이 바뀌므로 첫 비행 로그를 본 뒤 결정.
8. **EKF 원점 기준 고도** (F6) — 평지, 전원당 한 번 arm. 거리계가 없으면 경사지 비행 금지.
9. **STATUSTEXT 가 GCS 화면에 뜨는지** 는 실기 확인 항목(Mission Planner 는 선택 기체 sysid 로 거르고, 컴패니언은 255 라 "GCS:" 로 기록된다).
10. **PX4 경로** 는 C6(set_mode) 외에 이번 검토 범위 밖이다. 이 점검서는 ArduCopter 기준이다.

---

## 8. 출처

- ArduPilot `Copter-4.5`: `ArduCopter/mode_guided.cpp` (`ModeGuided::init`, `velaccel_control_run`, `get_timeout_ms`, `stabilizing_pos_xy`), `ArduCopter/GCS_Mavlink.cpp` (`sane_vel_or_acc_vector`, `SET_POSITION_TARGET_LOCAL_NED` 핸들러), `ArduCopter/events.cpp` (`failsafe_radio_on_event`, `failsafe_gcs_check`, `handle_battery_failsafe`), `ArduCopter/mode.cpp` (`Copter::set_mode`), `ArduCopter/Parameters.cpp`, `ArduCopter/config.h`, `ArduCopter/land_detector.cpp`, `libraries/RC_Channel/RC_Channel.h`.
- ArduPilot 위키(`ArduPilot/ardupilot_wiki` master): `copter/source/docs/radio-failsafe.rst`, `gcs-failsafe.rst`, `failsafe-battery.rst`, `land-mode.rst`, `crash_check.rst`, `ac2_guidedmode.rst`, `flying-arducopter.rst`; `dev/source/docs/raspberry-pi-via-mavlink.rst`.
- 이 저장소: `test_closed_loop.py --scenario ...` (안전 시나리오 6 종), `test_fixes.py` `안전:` 검사, `docs/REQUIREMENTS.md` SAF-18~21, FCR-19.
