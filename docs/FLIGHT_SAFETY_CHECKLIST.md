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

기존 폐루프 골든 스트림(`test_closed_loop.py --compare`)은 **바이트 단위로 동일**합니다 — 정상 비행 경로의 명령은 하나도
바뀌지 않았고, 위 방벽은 비정상 상황에서만 작동합니다. 단위 155 개 · 폐루프 22 개 · 요구도 74 개(추적 0 오류).

```bash
python3 test_fixes.py
python3 test_closed_loop.py                      # 골든 (기본 시나리오)
for s in "tilt --level 0" tilt nan fc_stale "climb --frames 1900" lost_alt; do python3 test_closed_loop.py --scenario $s; done
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
| B-6 | 깊이 상식 검사 | 표적 2 / 3 / 5 m 에서 HUD `raw=` 와 `est=` 가 줄자와 ±0.15 m. `rD`(깊이 신뢰도) > 0.5 | 하늘 배경·역광에서 `raw=N/A` 가 잦으면 4절의 깊이 소실 시간 참고 |
| B-7 | FPS | `MARS_SHOW_WINDOW=0`, `sudo nvpmodel -m 0 && sudo jetson_clocks` 뒤 `[STAT] FPS=` ≥ 25 를 60 초. `[YOLO] backend=TensorRT device=cuda:0` | `.pt` 폴백·CPU 면 README 배포 절 |
| B-8 | 소실·착륙 로직 | 표적을 가리고 `mission=LOST_HOLD` → 10 s 뒤 `FAILSAFE_LAND` 와 `[FC] set mode: LAND` 가 **CMD=DRY 에서는 나가지 않음** 확인. 이후 CMD ON + 조종기 LOITER 상태에서 같은 시험 → LAND 가 **안 나감**(C2) | |
| B-9 | 헤드리스·종료 | `q` 로 종료 시 `[SYS] shutdown` 뒤 hold 1회. SSH 끊김 시 프로세스가 죽지 않게 `nohup`/`systemd` 또는 `tmux` 로 실행 | SSH 세션이 죽으면 컴패니언도 죽는다 → 3 s 뒤 호버(1절) 이지만 그 뒤 추종은 없다 |
| B-10 | RC failsafe 시험 | 위키 `radio-failsafe.rst` 의 Test #1~#3 (프롭 제거, 송신기 끔) — GCS HUD 에 "Radio Failsafe" 와 설정한 동작 | |
| B-11 | 킬스위치 | `MOTOR_ESTOP` 스위치로 arm 상태에서 모터 정지 확인 | |
| B-12 | 진동·호버 스로틀 | 위키 "First Flight with Copter" 의 진동 측정·호버 스로틀은 **조종사 단독 비행(F-0)** 에서 끝내 둔다 | |

---

## 4. 인지·FPS·바람 (검토 결과 반영 — 아래 절은 검토 보고 뒤 채움)

(작성 중)

---

## 5. 미션·FC 상호작용 검토 (검토 보고 뒤 채움)

(작성 중)

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

## 7. 남은 위험 (지상에서 못 없앤 것)

(검토 보고 뒤 채움)

---

## 8. 출처

- ArduPilot `Copter-4.5`: `ArduCopter/mode_guided.cpp` (`ModeGuided::init`, `velaccel_control_run`, `get_timeout_ms`, `stabilizing_pos_xy`), `ArduCopter/GCS_Mavlink.cpp` (`sane_vel_or_acc_vector`, `SET_POSITION_TARGET_LOCAL_NED` 핸들러), `ArduCopter/events.cpp` (`failsafe_radio_on_event`, `failsafe_gcs_check`, `handle_battery_failsafe`), `ArduCopter/mode.cpp` (`Copter::set_mode`), `ArduCopter/Parameters.cpp`, `ArduCopter/config.h`, `ArduCopter/land_detector.cpp`, `libraries/RC_Channel/RC_Channel.h`.
- ArduPilot 위키(`ArduPilot/ardupilot_wiki` master): `copter/source/docs/radio-failsafe.rst`, `gcs-failsafe.rst`, `failsafe-battery.rst`, `land-mode.rst`, `crash_check.rst`, `ac2_guidedmode.rst`, `flying-arducopter.rst`; `dev/source/docs/raspberry-pi-via-mavlink.rst`.
- 이 저장소: `test_closed_loop.py --scenario ...` (안전 시나리오 6 종), `test_fixes.py` `안전:` 검사, `docs/REQUIREMENTS.md` SAF-18~21, FCR-19.
