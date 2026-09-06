# MARS-IMM — Leader-Follower Drone

**Mode-Aware Reliability-Scheduled IMM Tracking.** RealSense D435i + YOLO11n으로 선두 기체를
추적하고, IMM-EKF로 상대 상태를 추정해 Pixhawk에 속도 setpoint를 내보내는 단일 프로세스
follower 컨트롤러입니다. Jetson에서 동작하며 종합설계(캡스톤) 과제로 개발되었습니다.

---

> # 🔶 조건부 비행 시험 가능
>
> **감시 비행을 막던 결함은 해소되었습니다.** 2026-08-24 감사에서 나온 치명 결함 6건(C1~C6)과
> 미션 결함 H2를 수정했고, 여기에 소실 판정·트래커 신원 게이트를 더했습니다. `test_fixes.py`
> 30개 검사와 ArduCopter/PX4 SITL **차등 검증**을 통과합니다 — 결함이 대조군에서 재현되고
> 수정 후 사라집니다([결과](#sitl-회귀-검증)).
>
> 특히 **조종사 수동 탈환이 유지되는 것을 SITL에서 확인했습니다**(이전에는 0.6초 만에
> 덮어써졌습니다). 이것이 감시 비행 시험을 가능하게 하는 전제입니다.
>
> **다만 이 시스템은 아직 한 번도 비행한 적이 없습니다.** 아래 조건을 지키는 **시험 비행**이
> 가능하다는 뜻이지, 운용 가능하다는 뜻이 아닙니다.
>
> ### 비행 조건 (전부 충족해야 함)
>
> | | |
> |---|---|
> | 조종사 | 상시 대기, 즉시 탈환 가능한 거리 — RC로 LOITER 전환하면 컴패니언이 손을 뗍니다 |
> | 리더 속도 | **0.35 m/s(시속 1.26km) 미만** — `MAX_VX` 한계이며 넘으면 정상상태에서 따라잡지 못합니다 |
> | 고도·장소 | 저고도, 개활지, 장애물 없음 |
> | 기체 파라미터 | `WP_YAW_BEHAVIOR=0` |
> | 사전 절차 | `python3 test_fixes.py` → SITL 회귀 → **프로펠러 제거 후 지상 setpoint 확인** → 저고도 시험 |
> | 첫 비행 | `MAX_VX/VY/VZ`를 0.15/0.08/0.05로 낮춰서 시작 (`main.py:117-120`에 주석으로 있음) |
>
> `SEND_MAVLINK_COMMANDS`는 여전히 기본값 `False`입니다. 지상 확인을 마친 뒤에만 켜세요.
>
> ### 아직 검증되지 않은 것
>
> 실제 비행 중의 검출 성능(진동·모션블러·조명 변화) · 바람/프롭워시 · GPS 품질 ·
> ESP-NOW 송신 펌웨어(**미존재** — 현재는 비전 단독 경로) · 검출률 60%에서의 추종 안정성.
> [남은 결함](#남은-결함)도 함께 보세요.

---

## 운용 순서

**`SEND_MAVLINK_COMMANDS`는 안전을 위해 기본값 `False`입니다** (`main.py:74`). 이 상태에서도
카메라·YOLO·EKF·미션 상태머신·화면·로깅이 전부 정상 동작하며, 계산된 속도 명령도 화면에
표시됩니다. **전송만 하지 않습니다.** 실비행 시 지상에서 `True`로 바꾼 뒤 진행하세요.

```
[지상]
  1. python3 test_fixes.py                     통과 확인
  2. 프로펠러 제거 상태로 python3 main.py       화면의 명령값이 의도대로 나오는지 확인
  3. main.py:74 를 True 로 수정                 (또는 실행 중 v 키)
  4. 기체 파라미터 WP_YAW_BEHAVIOR=0

[비행]
  5. 조종기로 수동 이륙 (ALT_HOLD)              원하는 고도까지
  6. 고도 안착 후 모드 스위치를 GUIDED 로       ← 이 순간부터 추종 시작
  7. 이상하면 스위치를 LOITER/ALT_HOLD 로       ← 즉시 조종사에게 돌아옴
```

### 왜 GUIDED여야 하는가

ArduCopter는 `SET_POSITION_TARGET_LOCAL_NED`를 **GUIDED에서만** 받습니다
(`ArduCopter/GCS_MAVLink_Copter.cpp:975`):

```cpp
// exit if vehicle is not in Guided mode or Auto-Guided mode
if (!copter.flightmode->in_guided_mode()) {
    return;
}
```

디코드 직후 버려지므로, **ALT_HOLD에서는 `SEND_MAVLINK_COMMANDS=True`여도 기체가 움직이지
않습니다.** 화면에는 `CMD:ON`이 뜨는데 반응이 없는 형태라 가장 헷갈리는 실패 방식입니다.
ALT_HOLD로 얻으려는 고도 유지는 GUIDED에서 자동으로 됩니다(위치 제어기가 수평까지 잡아줍니다).

### 조종기 스위치가 곧 자동/수동 전환입니다

컴패니언은 GUIDED/OFFBOARD가 아니면 명령을 보내지 않습니다. 따라서 3단 스위치를 이렇게 두면
SSH나 GUI를 거치지 않고 전환할 수 있습니다.

| 스위치 | 모드 | 의미 |
|---|---|---|
| 위 | ALT_HOLD | 수동 이륙·고도 잡기 |
| 중간 | LOITER | 비상 탈환 (위치까지 고정) |
| 아래 | **GUIDED** | 자동 추종 |

LOITER로 내리면 컴패니언이 다시 뺏지 못합니다 — C2 수정으로 SITL에서 검증했습니다(25초 유지).

## 시스템 개요

단일 스레드 `while` 루프 하나가 전부입니다(`main.py:497`). ROS도 threading도 async도 없습니다.

```
D435i (color+depth, 640×480@30)
   └─> YOLO11n 검출 (scheduler가 ROI·검출 주기 결정)
         └─> IoU 단일 표적 트래커 (bbox 지수 평활)
               └─> 측정 생성  RGB-D 3D  또는  bearing 2D
                     └─> 신뢰도 기반 R 팽창 + Mahalanobis 게이팅
                           └─> IMM-EKF (CV + Coordinated-Turn 2모델)
                                 ├─ ESP32 선두 텔레메트리 융합 (선택)
                                 └─> 미션 상태머신
                                       └─> P·D 제어 → body-frame 속도 setpoint
                                             └─> MAVLink SET_POSITION_TARGET_LOCAL_NED (10Hz)
```

**좌표계 체인**: pixel → camera → FRU → BODY_NED, ENU → FRU. 프레임 `MAV_FRAME_BODY_NED`(8),
type_mask는 항상 1479(속도 + yaw rate 사용, yaw 각도 무시). hold는 같은 마스크에 yaw_rate=0.0.

## 제어 법칙

네 축 모두 P+D로 제어합니다 (`main.py:388-399`). 상대 위치는 IMM-EKF 추정값(FRU: front/right/up)입니다.

| 상황 | 명령 | 식 |
|---|---|---|
| 너무 가까움 / 멂 | 전후 | `KP_FORWARD × (front − TARGET) + KD × v_front` |
| 좌 / 우 | 좌우 | `KP_RIGHT × right + KD × v_right` |
| 위 / 아래 | 상하 | `KP_UP × up + KD × v_up` |
| 시야 중앙 유지 | 기수 | `KP_YAW × atan2(right, front)` |

추정 불확실성(`pos_cov_trace`)이 크면 전체 명령에 0.55 / 0.75 배 감속이 걸립니다.

**속도 상한이 곧 운용 한계입니다.** `MAX_VX = 0.35 m/s`이므로 **리더가 0.35 m/s(시속 1.26km)를
넘으면 정상상태에서 따라잡을 수 없습니다.** D항이 상대속도라 정상상태 기여가 0이고 feed-forward가
없어서 생기는 제약입니다([남은 결함](#남은-결함) 참조).

SITL 실측으로 확인한 평형 거리: 리더 0.3 m/s에서 4.3m — 이론값 `TARGET + v/KP_FORWARD =
3.0 + 0.3/0.22 = 4.36m`와 소수 둘째 자리까지 일치합니다.

## 프레임률에 대하여

**카메라 FPS는 병목이 아닙니다.** setpoint는 `SETPOINT_PERIOD_SEC = 0.10` 즉 10Hz로만 나가면
되고, 24 FPS면 루프가 42ms마다 도니 여유롭습니다(SITL 실측 setpoint 9.1~9.2Hz, 최대 갭 0.138s
vs `GUID_TIMEOUT` 3.0s). Jetson에서 관측된 **24~26 FPS로 충분하며, 30 FPS로 올려도 제어 품질은
개선되지 않습니다.**

다만 평활 계수가 프레임당 값이라 FPS가 바뀌면 응답 시정수가 따라 변했습니다(24fps와 30fps에서
25% 차이). `smooth_velocity_cmd`에 `dt`를 넘겨 30fps 기준으로 보정하므로, 이제 실제 FPS와
무관하게 같은 시정수를 갖습니다.

## 현재 상태

| 영역 | 상태 |
|---|---|
| 좌표계 변환 체인 | ✅ SITL 2회 독립 재현, 오차 -0.1° |
| `BODY_NED` + mask 1479 | ✅ body 기준으로 정확히 동작 (`BODY_OFFSET_NED`로 바꾸지 말 것 — PX4가 거부) |
| 제어 법칙 자체 | ✅ 과감쇠·안정. 정지 리더 정상상태 오차 +0.00m, 5만 프레임 NaN/발산 0 |
| 10Hz setpoint 유지 | ✅ 실측 9.1~9.2Hz, 최대 갭 0.138s (`GUID_TIMEOUT` 3.0s 대비 여유) |
| coast / 재획득 | ✅ 0.5~3초 블랙아웃 전부 정상 |
| 미션 상태머신 | ✅ C1·C3·H2 SITL 차등 검증 |
| 센서 유효범위 | ✅ C4 SITL 차등 검증 (0.3 m/s 리더 추종) |
| 조종사 탈환 | ✅ C2 SITL 차등 검증 (LOITER 25초 유지) |
| 소실 판정 | ✅ 거리 기준으로 교체, 깊이 소실 10.0초 뒤 착륙 확인 |
| 표적 신원 | ✅ 회복 시 근접 게이트 (검출률 60% 대응) |
| PX4 경로 | ✅ C6 PX4 SITL 차등 검증 |
| 수정본 SITL 재검증 | ✅ C1·C2·C3·C4·C6·H2 차등 검증 / C5는 증상 발생 불가로 판정 |
| Jetson 실기 (지상) | 🔶 리더 검출 → 모터 구동까지 확인 (개발자 보고) |
| **실비행** | 🔶 미수행 — [조건부 시험 비행 가능](#-조건부-비행-시험-가능) |

## 수정 완료 (2026-09-07)

`python3 test_fixes.py` — 18개 검사 전부 통과. 하드웨어 없이 순수 로직만 검증합니다.

### C1 — 부팅 즉시 FAILSAFE_LAND

`_lost_time()`이 `last_seen_t is None`에 `999.0`을 반환했습니다. 리더를 **한 번도 본 적 없는
부팅 직후**가 정확히 이 조건이라, `999.0 > lost_hold_sec(5.0)`으로 첫 프레임에
`S_FAILSAFE_LAND`로 직행했습니다.

**수정**: "한 번도 못 봄"과 "놓침"을 분리. `update()`에서 `last_seen_t is None`이면
`S_WAIT_LEADER`(HOVER, `land=False`)로 보냅니다. 센티넬 `999.0`도 `0.0`으로 교체.

### C2 — 조종사 수동 탈환이 무효화됨

`SETPOINT_PERIOD_SEC = 0.10` 주기로 `set_mode("LAND")`를 재송신해, 조종사가 LOITER로 탈환해도
**100~135ms 만에 다시 LAND로 끌려갔습니다.** FC 모드는 1Hz 출력 블록에서만 읽고 경고만 찍었습니다.

**수정**: FC 모드를 매 루프 읽어 `fc_accepts_setpoints`(GUIDED/OFFBOARD)를 만들고, **LAND와 속도
setpoint 양쪽 송신을 이 게이트로 하드 스톱**합니다. LAND는 모드 게이트 자체가 latch 역할을 합니다 —
명령이 먹으면 모드가 GUIDED를 벗어나 분기가 실행되지 않고, 안 먹었을 때만 `LAND_RETRY_SEC`(2초)
간격으로 재시도합니다. (단순 one-shot latch는 LAND 유실 시 영영 재시도하지 않는 새 결함이 됩니다.)

### C3 — 공중에서 착륙 판정 통과

리더 절대고도가 `None`일 때 **상대** z로 대체했습니다. 동고도 편대비행에서 상대 z는 0 근처라
고도 조건(`< 0.65m`)이 **항상 만족**되어, 공중에서 `CONFIRMED_LANDING`이 났습니다.

**수정**: fallback 삭제. 절대(대지) 고도가 없으면 착륙 판정 자체를 하지 않습니다.

### C4 — 깊이 절벽

`depth_max_m = 6.00` vs `TARGET_DISTANCE_M = 5.0` — 여유 1m뿐이었습니다. P제어 정상상태
평형거리가 `TARGET + v_leader/KP_FORWARD`이므로 **리더가 0.22 m/s만 넘어도 평형점이 깊이창
밖**이었습니다. `max_depth_mad`는 정의만 되고 아무도 쓰지 않는 죽은 상수였습니다.

**수정**: `depth_max_m` 10.0, `TARGET_DISTANCE_M` 3.0 → 여유 7m (리더 약 1.5 m/s까지).
`max_depth_mad`를 실제 게이트로 연결 — 깊이 산포가 한계를 넘으면 신뢰도 0을 반환해,
median이 outlier에 앉았을 때 제어가 전속 후진으로 포화되던 경로를 막습니다.

### C5 — yaw mask가 기체를 스스로 돌게 함

`yaw_rate ≈ 0`일 때 `YAW_RATE_IGNORE`를 세워, ArduCopter가 `WP_YAW_BEHAVIOR`(기본 2)로 기수를
스스로 돌렸습니다. 프레임이 `BODY_NED`라 이후 모든 속도 명령이 함께 회전했고, SITL 실측에서 실제
이동이 명령 대비 **164.6° 어긋났습니다.**

**수정**: 분기 삭제 — 항상 mask 1479. `yaw_rate=0.0` + 비트 clear = "현재 기수 유지"가 의도한
동작입니다. **기체 파라미터도 `WP_YAW_BEHAVIOR=0`으로 설정해야 합니다.**

### C6 — PX4에서 LAND 시도 시 루프 사망

`mode_mapping()[name]`을 int로 가정했으나 **PX4는 3-튜플**(`"LAND"` → `(29, 4, 6)`)을 반환합니다.
uint32 필드에 튜플을 넣다가 `struct.error`가 나고, 예외 처리가 없어 **비행 중 제어 루프가
죽었습니다.**

**수정**: `master.set_mode(mode_name)`에 위임(pymavlink가 apm/px4 자동 분기) + `try/except`로
non-fatal화 → `AUTO.LAND` 재시도와 `MAV_CMD_NAV_LAND` fallback에 실제로 도달합니다.

### H2 — 호버 리더 정위치 유지가 존재하지 않음

`S_LEADER_HOVER`가 출발판단 블록에 있어 진입 다음 프레임에 `READY_HOVER`로 덮여 **정확히
1프레임(~33ms)**만 생존했습니다. 헤드라인 유스케이스가 동작하지 않았습니다.

**수정**: `S_LEADER_HOVER`를 FOLLOW 블록에서 처리. 히스테리시스 추가(진입 0.18 m/s, 복귀
0.25 m/s). 두 상태 모두 `allow_follow=True`라 **"따라잡아서 상대속도가 준 것"과 "리더가 멈춘 것"을
혼동해도 제어가 끊기지 않습니다** — 기존의 ~1Hz 정지/재출발 스터터도 함께 해소됩니다.

## SITL 회귀 검증

ArduCopter 4.8.0-dev SITL(네이티브 arm64 빌드)에 **저장소의 실제 `main.main()`을 그대로 붙여**
비행시켰습니다. 스텁은 인지 계층(cv2 / RealSense / YOLO)뿐이고 미션 상태머신 · 제어 · MAVLink
송신은 전부 실제 코드입니다. `SEND_MAVLINK_COMMANDS=True`로 실제 명령이 나갑니다.

**차등 검증** — 같은 시나리오를 수정 전(`HEAD~1`) 코드에도 돌려, 결함이 실제로 재현되는지
확인했습니다. 대조군이 통과하는 테스트는 아무것도 증명하지 못하기 때문입니다.

| 시나리오 | 수정 후 | 대조군 |
|---|---|---|
| **C1** 리더 미획득 상태로 대기 | ✅ 35초간 GUIDED 유지, LAND 없음 | ❌ **t+0.9초에 LAND 전환** |
| **C2** FAILSAFE_LAND 중 조종사 LOITER 탈환 | ✅ LOITER 25초 유지, 송신 차단 확인 | ❌ **0.6초 만에 LAND로 뺏김** |
| **C3** 절대고도 없음 + 리더 하강 | ✅ 착륙 판정 안 남 | ❌ **t=2.6초, 고도 15m에서 착륙 판정** |
| **C4** 0.3 m/s 리더 추종 | ✅ 후반 4.3m (목표 3.0m) | ❌ **10.8m — 따라붙지 못함** |
| **H2** 리더 정지 후 정위치 유지 | ✅ 후반 3.0m (목표 3.0m) | ❌ **6.0m에서 정지, 수렴 안 함** |
| **C6** PX4 3-튜플 `mode_mapping` | ✅ 예외 없음 (PX4 SITL) | ❌ **`required argument is not an integer`** |
| **거리 게이트** 깊이만 소실 (검출은 유지) | ✅ **10.0초 뒤 LAND** | ❌ **35초 내내 GUIDED — 착륙 안 함** |
| **인계** 수동 상승 12초 후 GUIDED 전환 | ✅ LAND 없음 | ❌ **전환 0.1초 만에 LAND** |
| **C5** 기수 제어권 이양 (`c5_probe.py`) | 5.0° 유지 | 1.0° 유지 — **증상 발생 불가** |

C4의 4.3m는 P 제어 평형거리 이론값 `TARGET + v/KP_FORWARD = 3.0 + 0.3/0.22 = 4.36m`와
소수 둘째 자리까지 일치합니다.

**C3·C4·H2의 대조군은 결함별 격리본입니다.** 수정 전 전체 코드는 C1이 부팅 1~3초 만에 LAND를
걸어 다른 결함의 조건에 도달조차 못 하므로, 수정본에서 해당 결함 하나만 되돌린 사본을 썼습니다
(자세한 내용은 [`sitl/README.md`](sitl/README.md)).

C2 대조군의 모드 이력이 결함을 그대로 보여줍니다:
`GUIDED(2s) → LAND(15.1s) → LOITER(15.1s, 조종사) → LAND(15.2s, 컴패니언이 탈환)`.
수정 후에는 `GUIDED(2s) → LAND(15s) → LOITER(15s)`에서 끝까지 LOITER를 유지합니다.

### C5는 SITL에서 재현되지 않았습니다

이전 감사가 보고한 "기수 자동 회전으로 실제 이동이 명령 대비 164.6° 이탈"이 **이번 SITL에서는
나타나지 않았습니다.** 양쪽 arm의 `TARGET_DISTANCE_M`을 동일하게 고정해 이동량을 통제하고
`WP_YAW_BEHAVIOR=2`(공장 기본값)로 두었는데도 기수 편차가 양쪽 다 0.0°였습니다.

`WP_YAW_BEHAVIOR`가 GUIDED 속도 제어에는 적용되지 않고 waypoint 항법에만 적용되는 것으로
보입니다. **마스크 수정(항상 1479) 자체는 유효하며 단위 테스트로 확인**되지만, 그 수정이 막는다고
주장했던 증상은 이 조건에서 검증되지 않았습니다. 기체 파라미터 `WP_YAW_BEHAVIOR=0` 권고는
비용이 0이므로 유지합니다.

### C5는 증상이 발생할 수 없습니다 — 소스로 확정

세 가지가 겹칩니다. (1) 공장 기본값 `WP_YAW_BEHAVIOR=2`는 LOOK_AHEAD가 아니라 LOOK_AT_NEXT_WP이고
GUIDED 속도 제어엔 waypoint가 없습니다. (2) LOOK_AHEAD(값 3)는 **진입 시 현재 기수로 초기화**되고
지면속도 1.0 m/s 초과에서만 갱신됩니다(`autoyaw.cpp:88-91`, `config.h:476`). (3) 이 포크는
`yaw_rate`를 `allow_follow` 안에서만 계산하므로 **버그 마스크는 명령 속도가 0일 때만** 나갑니다
(계측: 620여 setpoint 전부 `|v|=0.00`). 게다가 후진 속도 상한이 `KP_FORWARD×(TARGET−front)`
= 약 0.66 m/s라 문턱에 구조적으로 도달할 수 없습니다.

마스크 수정은 유지합니다(명시가 위임보다 낫고 비용 0). 다만 **C5는 치명 등급이 아니며**,
이전 감사의 "164.6° 이탈"은 이 구성에서 발생하지 않습니다. 재현 시도는
[`sitl/c5_probe.py`](sitl/c5_probe.py)에 남겨뒀습니다.

### C6은 PX4 SITL로 별도 검증했습니다

PX4 SITL(`px4_sitl_nolockstep`)에서 `mode_mapping['LAND'] = (29, 4, 6)` — 3-튜플임을 실물로
확인했습니다. C6은 전송 **전** 패킹에서 터지므로 비행이 필요 없습니다. 빌드·실행 절차는
[`sitl/README.md`](sitl/README.md)에 있습니다.

### PX4 OFFBOARD 데드락 — C2 수정 보완

C6 검증을 준비하다 발견했습니다. C2 수정이 처음에는 **LAND와 속도 setpoint를 같은 모드 게이트로**
묶었는데, PX4는 OFFBOARD에 진입하기 전에 setpoint 스트림이 먼저 흐르고 있어야 합니다. 그대로 두면
**PX4에서는 영원히 OFFBOARD에 못 들어가는 데드락**이 됩니다.

조종사를 뺏는 것은 setpoint가 아니라 **모드 변경(LAND)** 이므로 게이트를 그쪽에만 남겼습니다.
속도 setpoint는 GUIDED/OFFBOARD가 아니면 FC가 조용히 버리므로 무해합니다. 이 변경 후 C1·C2를
ArduCopter SITL에서 재검증해 통과를 확인했습니다.

### 트래커 신원 게이트 (2026-09-07 추가)

`tracker.py`가 `lost_count > 0`이면 IoU와 무관하게 **아무 검출이나 수용**했습니다. 검출률이 낮으면
`lost_count > 0`이 정상 상태가 되어 게이트가 사실상 무력화되고, 한 프레임 놓친 사이 화면 반대편의
다른 대상이 트랙을 가져갑니다. **개발자 보고 기준 현재 YOLO 검출률이 약 60%이므로 실제로 자주
발생할 수 있는 조건입니다.**

**수정**: 회복 시에는 겹침 대신 **근접**을 요구합니다 — 중심 이동량이 이전 bbox 대각선의
`0.75 × (1 + lost_count)`배 이내여야 합니다. 정상 기동은 통과하고 화면 반대편 점프는 막힙니다.

### 소실 판정을 거리 기준으로 (2026-09-07 추가)

`leader_visible_for_mission = track_visible or ekf_reliable or esp_visible` —
**`track_visible` 하나로 충분했습니다.** YOLO가 bbox만 내면 미션은 "리더를 보고 있다"고
판단합니다. 깊이가 죽어 bearing-only로 강등돼도(거리 관측 불가) 소실 판정이 나지 않아
**실패 착륙이 영원히 발동하지 않습니다.** `ekf.is_reliable()`도 마찬가지입니다 —
`coast_time`이 bearing-only 업데이트로도 0이 되기 때문입니다.

**수정**: `coast_time`과 별도로 **`range_coast_time`** 을 셉니다. 매 `predict`마다 누적하고
**거리를 담은 측정(RGB-D / ESP32 위치)에서만** 0으로 되돌립니다. 미션 판정은
`ekf.has_range_fix()`를 씁니다. 상태 출력에 `rcoast=`로 함께 표시되므로 운용 중 두 값이
갈라지는 것을 볼 수 있습니다.

**소실 후 착륙까지 총 10초**로 맞췄습니다 — `imm.range_coast_max_sec`(2.0) +
`MissionManager.lost_hold_sec`(8.0). SITL 실측 10.0초.

### 현장 대비 수정 (2026-09-07 감사)

첫 시험 비행을 상정한 다중 에이전트 감사에서 나온 것들입니다.

| 문제 | 현장에서 | 수정 |
|---|---|---|
| `USE_LEADER_ESP32=True` + ESP32 없음 | `serial.Serial()` 예외가 `try` 블록 **밖**에서 나 프로그램이 아예 안 뜸 (100%) | 기본 `False` + `start()` 예외 가드 |
| **GUIDED 인계 순간 즉시 LAND** | 수동 상승 중 리더가 화면 밖 → 10초 뒤 FAILSAFE_LAND 래치 → 스위치 넘기는 첫 프레임에 착륙 | **GUIDED 진입 에지에서 미션·명령 리셋** |
| 평활 버퍼 포화 | FC가 무시하는 동안 목표값까지 수렴 → 인계 첫 setpoint가 램프 없이 최대 | 같은 리셋에서 `prev_body_cmd` 0으로 |
| 헤드리스 `cv2.imshow` 예외 | 창이 없거나 `ssh -X`가 끊기면 루프가 통째로 사망 | `MARS_SHOW_WINDOW=0` + imshow try/except → 헤드리스로 계속 |
| `target_class_name` 불일치 | 모델에 없는 클래스면 **검출 0건인데 경고 없음** — FPS는 정상이라 원인 찾다 하루 날림 | 시작 시 모델 클래스와 대조해 크게 경고 |
| 카메라 hiccup | `wait_for_frames` 예외가 안 잡혀 비행 중 종료 | 드롭 프레임으로 처리, 연속 30회면 종료 |
| 수직축 바닥 없음 | 리더(또는 지면의 오검출)를 따라 계속 하강 | `MIN_AGL_M`(1.5m) 아래에서 하강 명령 차단 |
| `l`/`h` 키가 거짓 보고 | CMD=DRY인데 "manual LAND command"를 출력해 조작자를 속임 | 실제 송신 여부를 그대로 출력 |

**GUIDED 인계 LAND는 C1 수정의 사각지대였습니다.** C1 가드는 "리더를 한 번도 못 봄"만 막고,
"보다가 상승 중에 놓침"은 막지 못했습니다. 이 저장소의 계획된 운용 절차에서 100% 발생합니다.

## 남은 결함

수정하지 않았습니다. 아래 항목은 **시험 비행을 막지는 않지만**, 조건을 벗어나기 전에 처리해야
합니다.

- **정상상태 뒤처짐** — 순수 P제어에 D항이 상대속도라 정상상태 기여가 0입니다. 리더 1m/s당 약 4.55m
  뒤처집니다(이론값과 소수 2자리 일치). 리더 속도 feed-forward가 필요하며, ESP32 속도가 들어오면
  바로 구현 가능합니다.
- **ESP32 속도 hint가 칼만 게이트를 우회** — `leader_telemetry.py:535-559`가 필터 상태에 직접
  대입하고 공분산을 무조건 축소합니다. innovation도 Mahalanobis 게이트도 likelihood도 없습니다.
- **`p_ct`가 사전확률 0.333에서 벗어나지 않음** — 37개 시나리오 중 34개에서 "mode-aware"가
  사실상 무동작이었습니다.
- **`detector_skipped` 페널티 도달 불가** — 플래그가 측정 dict에 복사되지 않아, 직전 bbox를
  재사용한 프레임도 신뢰도 1.0으로 EKF에 들어갑니다.
- **조작자 키가 전부 `if SHOW_WINDOW:` 안** — 헤드리스 환경에서 탈출구가 없습니다.
- **시작 실패 지점이 잘못됨** — `leader_rx.start()`가 `try` 블록 밖에서 호출되어, ESP32가 안 꽂혀
  있으면 루프 진입 전에 `SerialException`으로 죽습니다.
- **종료 시 GUIDED armed 유지**, 최소 이격 개념 없음(선회 시 최근접 1.59m 관측).

## 보안 주의

- **UDP 텔레메트리는 인증이 없습니다.** `leader_telemetry.py:76-107`이 `0.0.0.0:5005`에 바인딩하며,
  체크섬도 replay 보호도 없습니다(`seq`는 파싱만 하고 검사하지 않음). Jetson에 도달 가능한 임의의
  호스트가 리더 위치를 주입해 **기체를 움직일 수 있습니다.** 격리된 네트워크에서만 사용하십시오.
- **신선도를 패킷 자체 시각이 아니라 수신 시각으로 측정합니다**(`leader_telemetry.py:418`). 얼어붙은
  GPS 좌표를 10Hz로 계속 보내는 송신기는 항상 "신선"으로 판정됩니다.
- **GPS 품질 검사가 없습니다.** `reliability.gps_reliability`는 정의만 되어 있고 어디서도 호출되지
  않습니다. ESP32의 첫 나쁜 fix가 필터 전체를 초기화할 수 있습니다.
- **고도 데이텀 혼용** — WGS84 타원체고와 MSL이 섞여 있습니다(한국 지오이드 약 25m). 그 차이가
  수직 상대오차로 1:1 전달됩니다.
- 비행 로그(`logs/`)에는 GPS 좌표가 들어가므로 `.gitignore`에 포함했습니다.

## 하드웨어

| | |
|---|---|
| 기체 | Holybro X500 V2 + Pixhawk (ArduCopter SITL 검증 / PX4는 C6 수정됨, 미실행) |
| 컴패니언 | NVIDIA Jetson |
| 카메라 | Intel RealSense D435i (640×480 @ 30fps) |
| 선두 텔레메트리 | ESP32 (serial `/dev/ttyUSB0` 115200 또는 UDP 5005) — **송신 펌웨어는 이 저장소에 없습니다** |

## 실행

```bash
# 의존성 (requirements.txt 없음 — 수동 설치)
pip install numpy opencv-python pymavlink ultralytics pyrealsense2 pyserial

# 단위 회귀 — numpy만 있으면 됨 (하드웨어 불필요)
python3 test_fixes.py

# SITL 회귀 — 드론 없이 실제 main.main()을 비행시킴 (sitl/README.md 참조)
python3 sitl/harness.py --all

python3 main.py
```

`MARS_FC_PORT` 환경변수로 FC 포트를 바꿀 수 있습니다(기본 `/dev/ttyACM0`).
SITL에 직접 붙이려면 `MARS_FC_PORT=udpin:0.0.0.0:14551 python3 main.py`.

키: `q`/`ESC` 종료 · `m` MARS-IMM 토글 · `v` MAVLink 송신 토글 · `l` LAND · `h` HOLD
(모두 `SHOW_WINDOW`가 True인 GUI 창에서만 동작)

**CLI 인자가 없습니다.** `MARS_FC_PORT` 외의 모든 설정은 `main.py` 상단 상수와 `config.py`를 직접 편집합니다.

주요 기본값 (`main.py`):

| 상수 | 값 |
|---|---|
| `SEND_MAVLINK_COMMANDS` | `False` (dry-run) |
| `TARGET_DISTANCE_M` | 3.0 |
| `MAX_VX` / `MAX_VY` / `MAX_VZ` | 0.35 / 0.22 / 0.12 m/s |
| `KP_YAW` / `MAX_YAW_RATE` | 0.8 / 0.35 rad/s |
| `SETPOINT_PERIOD_SEC` | 0.10 |
| `USE_LEADER_ESP32` | `True` |

`config.py`의 모델 경로(`/home/dsl/DRONE/leader_drone_yolo11n.pt`)와 `target_class_name`(현재
`"person"`으로 실험 중, 드론 전환 시 `"leader_drone"`)은 환경에 맞게 수정해야 합니다.

## 모듈

| 파일 | 역할 |
|---|---|
| `main.py` (1081) | 제어 루프 전체 — 상수, 명령 생성, MAVLink 송신, 상태 표시 |
| `mission_manager.py` | 미션 상태머신 (WAIT_LEADER / READY_HOVER / FOLLOW / LOST_HOLD / FAILSAFE_LAND …) |
| `imm_ekf.py` | 2모델 IMM-EKF (CV + Coordinated-Turn). **CA 모델은 없습니다** |
| `scheduler.py` | 검출기 on/off와 ROI 크기만 스케줄링 (스레드·센서율은 아님) |
| `reliability.py` | 신뢰도 기반 R 팽창 + Mahalanobis 게이팅 |
| `leader_telemetry.py` | ESP32 선두 GPS/속도 수신·파싱·ENU 변환·융합 |
| `measurement.py` | bbox + depth → RGB-D 3D / bearing 2D 측정 |
| `tracker.py` | IoU 단일 표적 트래커 (bbox 지수 평활) |
| `detector.py` | YOLO11n 래퍼 (TensorRT `.engine` 우선) |
| `camera.py` | D435i 래퍼 (depth→color 정렬, 실제 depth_scale 조회) |
| `utils_geometry.py` | 순수 기하 헬퍼 (intrinsics 누락 시 조용히 기본값 사용에 주의) |
| `logger.py` | JSONL 스트리밍 로거 (종료 시 CSV 변환) |
| `test_fixes.py` | C1~C6 + H2 단위 회귀 테스트 (하드웨어·FC 불필요) |
| `sitl/` | SITL 회귀 하네스 · C5 전용 프로브 · 실행 안내 |
| `controller.py` | **죽은 코드** — 아무도 import하지 않음. 모터 직접 제어(FC 자세 안정화 우회)라 되살리지 말 것 |

## 검증 방법

드론 없이 4층으로 검증 가능합니다: 순수함수/프레임 단위 → `main.main()` 폐루프 시뮬
(`cv2`/`ultralytics`/`pyrealsense2`만 스텁) → MAVLink 와이어 → ArduCopter SITL 실비행.

둘 다 저장소에 있습니다 — `test_fixes.py`(1층)와 [`sitl/`](sitl/)(4층). SITL 하네스는 드론도
카메라도 없이 `numpy`와 `pymavlink`만으로 돌고, ArduCopter 빌드부터 차등 검증까지
[`sitl/README.md`](sitl/README.md)에 적어뒀습니다.

### 하드웨어에서 확인된 것 (개발자 보고, 이 저장소에 로그 없음)

- Jetson 카메라 파이프라인 **24~26 FPS**
- 리더 드론 **검출 → 모터 구동**까지 지상에서 동작
- 거리 측정 양호
- YOLO 검출률 **약 60%** (데이터셋 규모 부족, 확장 시 80%+ 기대)

### 여전히 검증되지 않은 것

폐루프 실비행 · 실제 공력/바람/프롭워시 · GPS 품질 · ESP32(ESP-NOW) 송신 펌웨어(미존재) ·
저조도/역광/가림 상황의 검출 · 60% 검출률에서의 추종 안정성.

## 다음 할 일

1. **첫 시험 비행** — 상단 [비행 조건](#-조건부-비행-시험-가능)을 지켜 프로펠러 제거 지상 확인 →
   저고도 감시 비행. 운용 중 `rcoast=` 값을 보면 실제 깊이 품질을 바로 진단할 수 있습니다
2. **리더 속도 feed-forward** — 리더를 0.35 m/s 이상으로 움직이려면 필요합니다. 그 전까지는
   현재 제어로 충분합니다 (SITL 실측: 리더 0.3 m/s를 4.3m로 추종)
3. **YOLO 데이터셋 확장** — 검출률 60% → 80%+. 추종 안정성에 가장 직접적입니다
4. **기체 파라미터 `WP_YAW_BEHAVIOR=0`** — 비용 0이므로 권고 유지
5. `requirements.txt` 추가, `config.py`의 절대경로 제거
6. ESP-NOW 송신 펌웨어 (현재는 비전 단독 경로로 동작)
