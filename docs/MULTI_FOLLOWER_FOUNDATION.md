# 정밀 분석 · 제어 판단 · 선두 1 : 후미 N 확장 토대 (2026-09-24)

저장소 전체(인지 → 추정 → 미션 → 제어 → MAVLink)를 코드 단위로 추적해 **유기적으로 동작하는지**, **제어 설계에 문제가
없는지**를 판단했습니다. 판단 기준은 제 생각이 아니라 (1) 스트링 안정성·편대 제어 문헌, (2) ArduPilot / PX4 의 실제
follow 구현 소스, (3) MAVLink 규격입니다(5절). 그 판단 위에 **선두 하나의 조종만으로 후미 여럿을 움직일 수 있는 토대**를
놓았습니다(4절). 전부 구현한 것이 아니라 토대이며, **기존 단일 후미 동작은 비트 단위로 보존**됩니다.

```bash
python3 test_fixes.py                        # 146개 (이번에 17개 추가: '편대:' '수평화:' 'ESP32 GPS 신선도:')
python3 test_closed_loop.py --compare before.json   # 변경 전 스트림과 동일 (setpoint 352개, 모드 전환, 최종 자세)
python3 analysis/trace_check.py              # 요구도 67개, 깨진 참조 0
```

## 0. 결론

| 질문 | 답 | 근거 |
|---|---|---|
| 유기적으로 동작하는가 | **예.** 프레임·단위·주기·상태 리셋이 모듈 경계에서 일관되고, 실제 `main.main()` 을 돌리는 폐루프·SITL 시험이 그 경로를 덮는다 | 2.1 흐름표, 기준선 129+14 통과 |
| 결함이 있는가 | **3건 + 구조 한계 2건.** 중: 기체 기울기 → 제어 오차 프레임 불일치(A1). 하: ESP32 상대위치의 팔로워 GPS 신선도 미검사(A2), ESP32 속도 힌트가 정식 측정 갱신이 아님(A3). 구조: 슬롯 개념 부재(A5), 선행기-전용 정보 토폴로지(A6) | 2.2 |
| 제어는 괜찮은가 | **단일 후미로는 괜찮다.** P+D+피드포워드 외루프는 PX4·ArduPilot follow 와 같은 구조이고 여유(GM 14 dB, PM 84°)는 통상 기준을 넘는다. **다중 후미 체인으로는 원리적으로 부족** — 상수 간격 + 선행기 정보만으로는 어떤 선형 제어기도 스트링 안정을 못 만든다(Seiler 2004). 해법은 선두 속도 방송이며 그것이 토대의 핵심이다 | 3절 |
| 토대는 무엇인가 | `formation.py`(슬롯·리더 상태 방송 스키마·상대 heading), `config.formation`, `main` 훅 4곳, 리더 ID 필터, 체인 토폴로지 시뮬. 기본값은 전부 기존 동작 | 4절 |

## 1. 확인 방법

1. **정적 추적** — 모든 모듈을 읽고 데이터가 어느 프레임·단위·주기로 어디로 가는지 표로 만들었다(2.1).
2. **가설을 수치로** — 의심되는 지점은 실제 코드(`main.compute_velocity_cmd_from_estimate`, `rot_body_to_ned`)를 불러 값을
   냈다. 예: 기울기 편향은 pitch 10° 에서 vz −0.094 m/s 로 재현됐고 단위 검사로 고정했다(`수평화:`).
3. **레퍼런스 소스 대조** — ArduCopter `GCS_MAVLink_Copter.cpp`, `AP_Follow.cpp`, PX4 `mavlink_receiver.cpp`,
   `FlightTaskAutoFollowTarget.hpp`, `TargetEstimator.hpp`, PX4 user guide `follow_me.md` 를 GitHub raw 에서 직접 읽었다.
   MAVLink `FOLLOW_TARGET`·`MAV_FRAME` 정의는 설치된 pymavlink 2.4.50 에서 읽었다.
4. **문헌** — 이 환경은 arXiv·IEEE·ScienceDirect·Semantic Scholar 가 차단돼 원문 PDF 는 못 열었다. 서지사항과 핵심 정리는
   웹 검색 결과 요약으로 교차 확인했고, 5절에 "직접 읽음 / 검색으로 확인" 을 구분해 적었다.
5. **차등 확인** — 변경 전 폐루프 스트림을 저장해 두고(`--dump`), 변경 후 `--compare` 로 setpoint 352개·모드 전환·최종 자세가
   같은지 봤다. 같다.

## 2. 유기적 동작 분석

### 2.1 데이터 흐름 (프레임 · 단위 · 주기)

| 단계 | 모듈 · 함수 | 입력 → 출력 | 프레임 | 주기 | 확인 |
|---|---|---|---|---|---|
| FC 수신 | `mavlink_io.drain_messages` | HEARTBEAT / LOCAL_POSITION_NED / ATTITUDE / GLOBAL_POSITION_INT → `_vehicle_state` (timestamp 부여) | NED, rad, m, cm/s(GLOBAL) | 매 루프, 스트림 10 Hz | ✅ FC 컴포넌트만 heartbeat 인정(IF-01) |
| 카메라 | `camera.D435i.get_frames` | 컬러·정렬 깊이(uint16 × depth_scale) | 카메라 [right, down, forward] | 30 fps | ✅ depth_scale 실제 조회 |
| 검출·추적 | `scheduler` → `detector` → `tracker` | bbox, conf, lost_count, detector_skipped | 픽셀 | 1~2 프레임마다 | ✅ ROI 오프셋 복원, 근접 게이트 |
| 측정 | `measurement.build_rgbd / build_bearing` | bbox 중앙 깊이 median/MAD → 3D 점 또는 bearing | 카메라 | 프레임 | ✅ MAD 게이트(C4) |
| 자세 보상 | `main.ego_rotation_cam` → `ImmEkf.compensate_ego_rotation` | ATTITUDE 변화량 → 상태 회전 | 카메라(기체 고정) | 프레임 | ✅ 수학 확인: p_b2 = R2ᵀR1 p_b1, CT heading 도 같이 옮김 |
| 추정 | `ImmEkf.predict / update_*` | 상대 위치·속도 6 상태, μ(CV/CT), coast 타이머 3종 | 카메라(기체 고정) | 프레임 | ✅ 표준 IMM 혼합·갱신·확률 |
| ESP32 융합 | `leader_telemetry.build_leader_measurement_from_packet` → `main.fuse_esp32` | 리더 LLA·ENU 속도 + 팔로워 GLOBAL_POSITION_INT/yaw → 상대 위치(카메라), 속도 힌트 | **수평(yaw만) 프레임을 카메라 프레임으로 넣음** | 새 패킷마다 | ⚠ A2, A3 |
| 리더 절대 속도 | `main.follower_velocity_fru` + `self_velocity_lpf` + EKF 상대속도 | v_L = v_self(0.3 s LPF) + v_rel | v_self 는 수평(yaw), v_rel 은 카메라(기울어짐) | 프레임 | ⚠ A1 |
| 미션 | `MissionManager.update` | 거리 확보 여부, 절대 속도, 리더 AGL → 상태 8종 · 정책 | FRU | 프레임 | ✅ C1·C3·H2 검증 |
| 제어 | `main.compute_velocity_cmd_from_estimate` → `smooth_velocity_cmd` | (front−D, right, up), v_rel, FF → BODY_NED [vx, vy, vz, yaw_rate] | **입력은 카메라(기울어짐) FRU, 출력은 FC 가 yaw 만 회전** | 프레임, 송신 10 Hz | ⚠ A1 |
| 송신 | `main.send_body_velocity` | SET_POSITION_TARGET_LOCAL_NED, frame 8, mask 1479 | BODY_NED | 10 Hz 위상 고정 | ✅ ArduCopter·PX4 소스 확인(3.1) |

### 2.2 발견 사항

**A1 (중) — 기체 기울기가 제어 오차로 새어 들어간다.**
EKF 상태와 제어 오차 `(front−D, right, up)` 는 기체 고정 카메라 프레임이라 roll/pitch 를 담는다. 그런데 FC 는 BODY_NED 속도를
**yaw 만으로** 회전한다 — ArduCopter `GCS_MAVLink_Copter.cpp`: `vel_ned_ms.xy() = copter.ahrs.body_to_earth2D(vel_ned_ms.xy())`,
z 는 그대로; PX4 `mavlink_receiver.cpp`: `cosf(yaw)·vx − sinf(yaw)·vy`, `velocity[2] = velocity_body_sp(2)`. 두 프레임이 다르다.
실제 코드로 계산하면 pitch −10°(가속 중)에 3 m 전방·같은 고도 리더가 `up = +0.52 m` 로 보여 **vz = −0.094 m/s** 가 나간다
(`MAX_VZ` 0.12 의 78 %). 리더가 움직이지 않았는데 팔로워가 0.5 m 올라갔다가 감속 시 내려온다. 롤도 같은 크기로 좌우에 샌다.
기존의 `ego_rotation_cam` 은 자세 **변화량**만 보상하므로(리더가 화면에서 움직여 보이는 것 방지) 정적 기울기는 남는다.
왜 시험이 못 잡았나: 폐루프 FakeFC 와 SITL 하네스 모두 `roll=pitch=0` 이다.
**조치**: `main.level_fru_by_roll_pitch` 추가 + `config controller.level_by_attitude`(기본 **False** = 기존 동작). 켜면 제어 직전
`rel_fru`·`rel_vel_fru` 의 roll/pitch 를 되돌린다. 단위 검사 `수평화:` 3개. 기본값 전환은 SITL 에 자세 기울기 시나리오(가짜 FC 가
가속도에 비례한 pitch 를 보고)를 넣어 확인한 뒤가 맞다.

**A2 (하) — ESP32 상대위치가 팔로워 GPS 신선도를 보지 않는다.** `gps_fresh` 를 계산만 하고 HUD/로그에만 쓴다.
GLOBAL_POSITION_INT 스트림이 죽으면 마지막 위치로 리더 상대위치를 계속 만든다(팔로워가 움직인 만큼 틀림).
**조치**: `build_leader_measurement_from_packet(follower_gps_max_age_sec=…)` 게이트(`stale_follower_gps`), main 이 `GPS_MAX_AGE_SEC`
(0.7 s) 를 넘긴다. 단위 검사 `ESP32 GPS 신선도:`.

**A3 (하) — ESP32 속도 힌트가 정식 측정 갱신이 아니다.** `apply_leader_velocity_hint_to_imm` 은 두 필터의 속도 상태를 직접 혼합하고
P 를 0.98 배 수축한다. 잔차·이득이 없어 IMM 모드확률이 갱신되지 않고, 힌트는 수평(yaw) 프레임인데 상태는 카메라(기울어짐)
프레임이다(A1 과 같은 종류). 표준은 H = [0 I] 속도 측정 갱신이다(Bar-Shalom·Li·Kirubarajan, *Estimation with Applications to Tracking
and Navigation*, 11장 IMM). 코드 주석도 이를 인정한다. 이번에는 고치지 않았다 — 토대 범위 밖이고 ESP32 펌웨어가 아직 없다.

**A4 (정보) — 깊이 R 이 거리와 무관하다.** 스테레오 깊이 `Z = f·B/disparity`(librealsense `depth-from-stereo.md`) 에서 오차는
`dZ = Z²/(f·B)·d(disp)` 로 **Z² 에 비례**한다(Intel 명세: 2 m 에서 < 2 %). 3 m 에서는 `sigma_z` 0.25 가 넉넉하고 8 m(GPS-only 이격)
에서는 빠듯하다. 신뢰도 팽창이 일부 보완하므로 지금은 문제가 아니지만 `R_z ∝ Z²` 로 바꾸는 편이 맞다.

**A5 (구조) — "슬롯" 이 없다.** 오차가 `(front − 3, right, up)` 이라 후미는 항상 **자기 시선 기준 리더 뒤 3 m** 로 간다. 후미가
둘이면 둘 다 같은 점으로 수렴한다. 리더의 heading 도 모른다(ESP32 패킷의 `yaw` 는 파싱만 하고 안 썼다). 리더를 구분할 ID 도
없다(같은 채널의 아무 패킷이나 리더). → 4절 토대.

**A6 (구조) — 정보 토폴로지가 선행기 전용이다.** 피드포워드의 리더 속도는 `v_self + v_rel(앞 기체)` 로 만든다. 후미 N 대를 체인으로
세우면 각 단이 앞 단만 알게 되고, 이것이 3.2 의 스트링 불안정 문제를 그대로 만든다. 현재 설계의 `|Γ|` 피크 1.13 이 n 단에서
1.13ⁿ 으로 쌓인다는 것은 저장소 문서(STABILITY_MARGINS 7.2)도 알고 있다. → 4절의 `ff_source="broadcast"`.

그 밖에 확인했으나 문제 없던 것: 좌표 체인(픽셀→카메라→FRU→BODY_NED, ENU→FRU)의 부호·순서, `_VEL_YAWRATE_MASK` 1479,
GUIDED 진입 리셋 범위(EKF·트래커는 유지, 미션·명령·FF 리셋 — 의도적), 소실 타이머 3종의 갱신 위치, 10 Hz 위상 고정 송신, LAND
재시도 게이트, 미션 히스테리시스, 데드존·저역통과의 수치. 테스트 커버리지 갭 두 가지는 남는다: 자세 기울기(A1)와 ESP32 경로
(폐루프·SITL 모두 `USE_LEADER_ESP32=False`).

## 3. 제어 판단 (문헌 · 레퍼런스 기준)

### 3.1 무엇을 기준으로 삼았나

| 기준 | 가져온 것 | 확인 |
|---|---|---|
| ArduPilot `AP_Follow.cpp` | 리더 식별 `FOLL_SYSID`, 오프셋 `FOLL_OFS_X/Y/Z` + `FOLL_OFS_TYPE`(0 NED / 1 리더 heading 기준), `FOLL_YAW_BEHAVE`(기본 1 = 리더를 바라봄), `FOLLOW_TARGET` 을 `GLOBAL_POSITION_INT` 보다 우선, 마지막 수신 뒤 `vel·dt + ½·acc·dt²` 외삽, `FOLL_TIMEOUT` 3 s, `FOLL_DIST_MAX` | GitHub raw 직접 읽음 |
| ArduCopter `GCS_MAVLink_Copter.cpp` | BODY_NED / BODY_OFFSET_NED 속도는 `body_to_earth2D` 로 **xy 만** 회전, z 그대로 | 직접 읽음 |
| PX4 `mavlink_receiver.cpp` | BODY_NED 속도는 `cos/sin(yaw)` 로 xy 회전, z 복사. `FOLLOW_TARGET` 은 lat/lon/alt/vel 을 그대로 퍼블리시 | 직접 읽음 |
| PX4 `FlightTaskAutoFollowTarget.hpp`, `TargetEstimator.hpp`, user guide | 추종 거리·각도(`FLW_TGT_DST/FA`, 각도는 리더 heading 기준 시계방향)·높이, 리더 heading 은 속도 방향에서 추정하되 **1.0 m/s 미만 동결**, 2차 기준모델(ω 1.0 rad/s, ζ 0.707) + 속도 피드포워드, 궤도각 저크 제한(4 m/s³), 목표 유실 3 s, 가속도 20 m/s² 포화, "GPS 편차 3~5 m 라 8 m 이상(권장 12 m) 이격" | 직접 읽음 |
| MAVLink `FOLLOW_TARGET`(#144) | timestamp ms, est_capabilities, lat/lon degE7, alt m, vel/acc NED m/s, attitude_q, rates, position_cov, custom_state. `MAV_FRAME_BODY_NED`(8) = 속도에는 BODY_FRD 와 동일 | pymavlink 2.4.50 |
| Seiler·Pant·Hedrick, IEEE TAC 49(10) 1835–1842, 2004 | 이중적분 기체 + **상대 간격만** + 상수 간격 정책이면 **어떤 선형 제어기로도** L₂ 스트링 안정 불가(‖T‖∞ > 1). 선두 정보를 쓰면 해소 | 검색 요약으로 확인 |
| Swaroop·Hedrick, IEEE TAC 41(3) 349–357, 1996; 1999 상수 간격 논문 | 스트링 안정 정의; 상수 간격 정책은 선두 속도 통신이 필요 | 검색 요약 |
| 상수 시간간격(CTH) 정리 | 선행기 정보만으로도 `h ≥ 2τ`(τ = 구동 지연)이면 스트링 안정 — 자율 대안 | 검색 요약 (Swaroop·Rajagopal 2001 리뷰, Rajamani 교재) |
| Ploeg·van de Wouw·Nijmeijer, IEEE TCST 22(2) 786–793, 2014 | CACC: PD + 선행기 가속도 피드포워드(무선), 적분항 없음, L_p 스트링 안정 정의 `‖Γ(jω)‖∞ ≤ 1` | 검색 요약 |
| Zheng·Li·Wang·Cao·Li, IEEE T-ITS 17(1) 14–26, 2016 | 정보 흐름 토폴로지(선행기 / 선두-선행기 / 양방향 …)가 안정성·확장성을 좌우; 선두 정보가 있으면 단 수에 둔감 | 검색 요약 |
| Oh·Park·Ahn, Automatica 53 424–440, 2015 | 편대 제어 분류: 위치 기반 / **변위 기반(공통 방위 필요)** / 거리 기반. 비전 상대위치를 편대 모양으로 쓰려면 방위 정렬(리더 heading)이 필요 | 검색 요약 |
| Mariottini 등, IEEE T-RO 25(6) 1431–1438, 2009 | 비전(bearing) 기반 선두-후미 국소화는 기동에 따라 관측 불가 구간이 있다 — bearing-only 를 거리 확보로 치지 않는 이 코드의 선택(EST-03)과 일치 | 검색 요약 |
| Intel D435 명세, librealsense 문서 | 깊이 오차 < 2 % @ 2 m, `Z = f·B/disp` → 오차 ∝ Z² | 문서 직접 읽음 + 검색 |

### 3.2 항목별 판단

**B1. 외루프 구조 — 적절.** `u = KFF·v_L + Kp·e + Kd·ė` 의 속도 setpoint 외루프는 ArduPilot AP_Follow(위치 P + 리더 속도 FF, `FOLL_POS_P`)
와 PX4 follow_me(기준모델 + 속도 FF)와 같은 구조다. 적분항이 없는 것도 맞다 — CACC 표준 제어기(Ploeg 2014)도 PD + FF 이며 적분은
스트링 안정을 해친다. 정상상태 오차 `(1−KFF)·v/Kp` 는 이 구조의 대가이고 문서가 정확히 알고 있다.

**B2. 여유 — 충분.** GM 14.4 dB, PM 84°, Ms 1.29 는 통상 기준(GM ≥ 6 dB, PM ≥ 30~45°, Ms ≤ 2)을 넘는다. 저장소가 이미 IMM-EKF 를
실측 FRF 로 넣어 계산했고, 이번에 그 모델을 재검토했다(자기 속도 되먹임 항의 유도 `L = P[(Kp E_p + Kd E_v)/s − KFF H_ff (H_m − E_v/s)]`
가 맞다).

**B3. 단일 후미 스트링 게인 |Γ| 1.13 — 수용 가능, 체인에는 부족.** 상수 간격 + 선행기 정보만이면 Seiler 2004 에 따라 원리적으로 `‖Γ‖∞ ≥ 1`
을 피할 수 없다. 저장소 문서는 `KFF 0.6 → 1.02` 를 대안으로 두지만 그것도 1 을 넘는다. **원리적 해법은 둘뿐이다**: (a) 선두 속도 방송
(leader-predecessor 토폴로지), (b) 시간간격 정책 `D = D₀ + h·v`, `h ≥ 2τ`. (a) 가 이 과제에 맞다 — ESP-NOW 는 본래 방송이고 리더
속도가 0.3 m/s 급이라 (b) 의 `h·v` 는 몇십 cm 라 의미가 작다. 4.3 의 체인 시뮬(실제 코드)로 확인했다: ω 0.35 rad/s 에서 선행기
추종은 리더→n 단 누적 이득이 **1.16 → 1.36 → 1.60** 으로 커지고, 선두 방송은 **1.09 → 0.91 → 0.67** 로 단 수에 따라 커지지
않는다. 또 방송이면 자기 속도 되먹임 경로가 없어 **KFF 1.0 도 안정**(순항 잔차 0.23 m = 데드존 몫, 정지 후 0) — vision 소스의
KFF 1.0 은 선형 모델 GM −0.2 dB 로 불안정이었다.

**B4. 프레임 — 결함(A1).** 레퍼런스 구현은 전부 NED 에서 계산하므로 이 문제가 없다. 비전 단독 시스템은 카메라 프레임에서
추정할 수밖에 없지만 **제어 직전에 수평화**해야 FC 의 해석과 맞는다. 옵션으로 넣었다.

**B5. 슬롯 기하 — 부재(A5).** 다중 후미에는 리더 heading 기준 오프셋이 필요하고(AP_Follow `FOLL_OFS_TYPE=1`, PX4 `FLW_TGT_FA`),
그러려면 리더 heading 을 알아야 한다(Oh 2015 의 변위 기반 조건). 이 과제의 리더 속도(0.3 m/s)는 PX4 의 속도-방향 데드존(1.0 m/s)
아래라 **방송 yaw 가 사실상 필수**다 — ESP32 패킷에 이미 있다(`yaw`, 이제 없으면 None 으로 구분).

**B6. 기수와 측면 축의 중복 — 슬롯 도입 시 분리 필요.** 지금은 `right` 오차를 측면 속도와 yaw 가 같이 줄인다. 측면 슬롯에서는 위치는
슬롯 오차로, 기수는 리더 방위로(카메라가 리더를 계속 보도록) 나눠야 한다. AP_Follow `FOLL_YAW_BEHAVE=1`, PX4 follow_me 와 같은 선택이다.
토대의 `compute_velocity_cmd_from_estimate(slot_error=…)` 가 그렇게 한다.

**B7. 안전 — 다중 후미 전용 항목이 비어 있다.** 후미 간 충돌 회피, 슬롯 재배치, 리더 유실 시 편대 전체의 일관된 행동(지금은 각자
10 s 뒤 LAND)은 없다. 토대는 **정적 유효성 검사**(슬롯 간 최소 이격, 깊이창 여유)까지만 둔다(4.5).

## 4. 토대 (구현한 것)

### 4.1 설계 원칙

1. **기존 동작 보존** — 슬롯 미설정이면 `los_slot(TARGET_DISTANCE_M)` 이 되고, 오차 `rel + (−D, 0, 0)` 은 IEEE 754 상
   `(front − D, right, up)` 과 같은 값이다. `--compare` 로 스트림 동일 확인. `ff_source`·`level_by_attitude` 기본값도 기존.
2. **선두 방송 = 토폴로지의 핵심** — 후미 N 대가 같은 `LeaderState` 를 받는다. 스키마는 MAVLink `FOLLOW_TARGET` 과 1:1 이라
   ESP-NOW JSON 이든 MAVLink 든 같은 객체가 된다.
3. **슬롯은 리더 프레임에서** — `FormationSlot(offset, frame)`; frame ∈ `leader`(리더 heading 기준, AP `FOLL_OFS_TYPE=1`) /
   `ned`(AP `=0`) / `los`(기존). 후미는 "리더 + 회전된 오프셋" 가상 점을 추종하고, 기수는 리더를 본다.
4. **heading 을 모르면 우아하게 강등** — 같은 거리의 LOS 후방 슬롯으로(편대 모양은 잃지만 추종·안전 로직은 그대로), 로그·STAT 에 `~LOS` 표시.

### 4.2 무엇이 어디에

| 파일 | 추가 · 변경 | 기본값에서의 동작 |
|---|---|---|
| `formation.py` (신규) | `FormationSlot`, `los_slot`, `slot_error_fru`, `enforce_min_distance`(GPS-only 8 m 규칙의 일반형), `validate_formation`, `slot_from_config`, `RelativeHeadingEstimator`(방송 yaw > 속도 방향 ≥ 0.5 m/s > 2 s 유지 > 없음), `LeaderState`(↔ `FOLLOW_TARGET`) | — |
| `config.py` | `formation.{follower_id, leader_id, slots, min_separation_m, depth_reserve_m, heading_min_speed_mps, heading_hold_sec, ff_source}`, `controller.level_by_attitude` | 슬롯 없음, `ff_source="vision"`, 수평화 꺼짐 |
| `main.py` | `level_fru_by_roll_pitch`; `compute_velocity_cmd_from_estimate(slot_error=)`; 루프에 heading 추정 → 슬롯 오차 → FF 소스 선택; STAT `ff=…(vision) slot=behind_los hdg=none`, HUD `slot=`, 로그 `formation.*`; GUIDED 진입 시 heading 추정기 리셋; ESP32 팔로워 GPS 신선도 게이트 | 명령 비트 동일 |
| `leader_telemetry.py` | 패킷 `leader_id`(`leader_id/id/sysid`), `acc`(ax/ay/az), `yaw` 없으면 None; 수신기 `expected_leader_id`/`require_leader_id` 필터 + `dropped_other_leader`; `follower_gps_max_age_sec` | ID 미지정이면 전부 통과 |
| `analysis/stability_margins.py` | `chain_sim(topology="predecessor"|"leader_broadcast")` — 방송은 선두 절대 속도를 링크 지연 Tm 뒤 받음 | 기존 호출 동일 |
| `test_fixes.py` | `편대:` 13, `수평화:` 3, `ESP32 GPS 신선도:` 1 | — |
| `docs/REQUIREMENTS.md` | FCR-16~19, IF-12~15, SAF-16~17 | — |

### 4.3 수치 근거 (실제 코드 체인 시뮬, `analysis.stability_margins.chain_sim`)

리더 0.30 ± 0.03 m/s 정현파, 후미 3대, FC 1차 지연 0.3 s, 링크 지연 0.1 s, 데드존·포화 밖. 값은 리더 속도 진폭 대비 n 단 속도 진폭.

| ω [rad/s] | 토폴로지 | 1단 | 2단 | 3단 |
|---|---|---|---|---|
| 0.35 (현재 설계 피크 부근) | 선행기 추종 (기존) | 1.16 | 1.36 | 1.60 |
| 0.35 | **선두 속도 방송** | 1.09 | 0.91 | 0.67 |

계단(0 → 0.3 m/s → 0), 후미 2대, **KFF 1.0**, 방송: 최대 오차 0.41 / 0.39 m, 순항 잔차 0.23 / 0.24 m(= 데드존 0.05 / Kp), 정지 후 −0.11 / −0.14 m.
같은 KFF 1.0 을 vision 소스로 쓰면 선형 모델은 GM −0.2 dB(불안정, STABILITY_MARGINS 1절 표).

### 4.4 쓰는 법 — V 자 3대 예

```python
# config.py
"formation": {
    "follower_id": os.environ.get("MARS_FOLLOWER_ID", "F1"),   # 기체마다 MARS_FOLLOWER_ID=F2 …
    "leader_id": "L1",                                          # ESP32 패킷의 leader_id 와 일치해야 채택
    "slots": {
        "F1": {"offset": [-3.0,  0.0, 0.0], "frame": "leader", "slot_id": "tail"},
        "F2": {"offset": [-3.0,  2.5, 0.0], "frame": "leader", "slot_id": "right_wing"},
        "F3": {"offset": [-3.0, -2.5, 0.0], "frame": "leader", "slot_id": "left_wing"},
    },
    "ff_source": "auto",        # 방송이 있으면 선두 절대 속도, 없으면 vision
    ...
}
```

리더 ESP32 는 기존 필드에 `"leader_id": "L1"` 과 `"yaw": <rad, 북 기준 우회전 +>` 를 더해 방송하면 된다(`acc` 는 선택).
비행 전 `formation.validate_formation(slots_from_config(cfg), …)` 가 빈 목록을 돌려주는지 확인한다(슬롯 간 2 m, 리더까지 1.3~7 m).
STAT 의 `slot=right_wing hdg=broadcast ff=+0.24(broadcast)` 가 보이면 슬롯·heading·방송 경로가 다 살아 있는 것이다.
`hdg=none`/`slot=right_wing~LOS` 면 heading 을 못 받아 LOS 로 강등된 상태다.

### 4.5 하지 않은 것 (우선순위 순)

1. **SITL 다기체 시나리오** — ArduCopter SITL 인스턴스 2~3개(`-I0/-I1/-I2`)에 각각 `main.main()` 을 붙여 V 자 편대 + `leader_sine`.
   하네스의 월드 모델(`World`)을 리더 1 + 후미 N 으로 일반화하면 된다. 이것이 없으면 4.3 은 시뮬 근거에 그친다.
2. **자세 기울기 시나리오 + `level_by_attitude` 기본값 전환** — 폐루프 FakeFC 에 가속도 비례 pitch 를 넣어 A1 을 회귀로 잡고 켠다.
3. **후미 간 충돌 회피** — 후미들도 `LeaderState` 와 같은 스키마로 자기 상태를 방송하면(FOLLOW_TARGET 은 원래 그 용도) 이웃 간
   최소 이격 반발항을 넣을 수 있다. 지금은 정적 검사뿐이다.
4. **ESP32 속도 힌트를 정식 측정 갱신으로**(A3) + 카메라 프레임 정합.
5. **ESP32 송신 펌웨어** — `LeaderState.to_follow_target()` 이 필드 규약이다(IF-11 미검증 그대로).
6. **리더 유실 시 편대 정책** — 지금은 각자 10 s 뒤 LAND. 방송이 있으면 "리더 GPS 로 유지" 가 가능(FOLL_TIMEOUT 3 s 같은 기준 필요).
7. **깊이 R ∝ Z²**(A4).

## 5. 참고문헌

직접 읽음: ArduPilot `libraries/AP_Follow/AP_Follow.cpp`, `ArduCopter/GCS_MAVLink_Copter.cpp`; PX4 `src/modules/mavlink/mavlink_receiver.cpp`,
`src/modules/flight_mode_manager/tasks/AutoFollowTarget/FlightTaskAutoFollowTarget.hpp`, 같은 폴더 `follow_target_estimator/TargetEstimator.hpp`;
PX4-user_guide `en/flight_modes_mc/follow_me.md`; IntelRealSense/librealsense `doc/depth-from-stereo.md`; pymavlink 2.4.50 `FOLLOW_TARGET`·`MAV_FRAME`.

검색 요약으로 확인(원문 접근 차단): P. Seiler, A. Pant, K. Hedrick, "Disturbance propagation in vehicle strings," IEEE TAC 49(10):1835–1842, 2004 ·
D. Swaroop, J. K. Hedrick, "String stability of interconnected systems," IEEE TAC 41(3):349–357, 1996 · D. Swaroop, J. K. Hedrick, "Constant spacing
strategies for platooning in automated highway systems," 1999 · D. Swaroop, K. R. Rajagopal, "A review of constant time headway policy for automatic
vehicle following," 2001 · J. Ploeg, N. van de Wouw, H. Nijmeijer, "Lp string stability of cascaded systems: Application to vehicle platooning,"
IEEE TCST 22(2):786–793, 2014 · Y. Zheng, S. E. Li, J. Wang, D. Cao, K. Li, "Stability and scalability of homogeneous vehicular platoon: Study on the
influence of information flow topologies," IEEE T-ITS 17(1):14–26, 2016 · K.-K. Oh, M.-C. Park, H.-S. Ahn, "A survey of multi-agent formation control,"
Automatica 53:424–440, 2015 · G. L. Mariottini 등, "Vision-based localization for leader-follower formation control," IEEE T-RO 25(6):1431–1438, 2009 ·
Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley 2001 (IMM) · Intel RealSense D435 제품 명세
(깊이 정확도 < 2 % @ 2 m).
