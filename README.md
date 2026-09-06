# MARS-IMM — Leader-Follower Drone

**Mode-Aware Reliability-Scheduled IMM Tracking.** RealSense D435i + YOLO11n으로 선두 기체를
추적하고, IMM-EKF로 상대 상태를 추정해 Pixhawk에 속도 setpoint를 내보내는 단일 프로세스
follower 컨트롤러입니다. Jetson에서 동작하며 종합설계(캡스톤) 과제로 개발되었습니다.

---

> # ⚠️ 실비행 전 SITL 재검증 필요
>
> 2026-08-24 SITL + 오프라인 시뮬 감사에서 발견된 **치명적 결함 6건(C1~C6)과 미션 결함 H2는
> 2026-09-07 수정 완료**되었고, `test_fixes.py`의 18개 검사를 통과합니다.
>
> C1·C2·C3·C4·H2는 **ArduCopter SITL 실비행으로 차등 검증**되었습니다 — 결함이 대조군에서
> 재현되고 수정 후 사라집니다([결과](#sitl-회귀-검증)). C5는 SITL에서 **증상이 재현되지
> 않았습니다**(아래 참조). C6은 PX4 SITL이 필요해 단위 테스트만 거쳤습니다.
>
> **이 시스템은 한 번도 실비행한 적이 없습니다.**
>
> 또한 [남은 결함](#남은-결함)에 정상상태 뒤처짐·표적 바꿔치기·ESP32 융합 게이트 부재 등이
> 그대로 있습니다. `SEND_MAVLINK_COMMANDS`는 여전히 기본값 `False`입니다.
>
> **권장 순서**: `python3 test_fixes.py` → SITL 회귀 → 프로펠러 제거 후 지상 setpoint 확인 →
> 저고도 유선 테스트.

---

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
| PX4 경로 | 🔧 C6 수정 (`master.set_mode()` 위임) |
| 수정본 SITL 재검증 | 🔶 C1·C2·C3·C4·H2 완료 / C5 재현 실패 / C6 미실시 |
| **실비행** | ⛔ **한 번도 수행된 적 없음** |

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
| **C5** 기수 드리프트 (`WP_YAW_BEHAVIOR=2`) | 0.0° (200샘플) | 0.0° — **재현 안 됨** |

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

### 아직 SITL에서 확인하지 않은 것

C6(PX4 3-튜플 `mode_mapping`)만 남았습니다. PX4 SITL이 별도로 필요하며 현재는 단위 테스트만
있습니다.

## 남은 결함

수정하지 않았습니다. 실비행 전에 판단이 필요합니다.

- **정상상태 뒤처짐** — 순수 P제어에 D항이 상대속도라 정상상태 기여가 0입니다. 리더 1m/s당 약 4.55m
  뒤처집니다(이론값과 소수 2자리 일치). 리더 속도 feed-forward가 필요하며, ESP32 속도가 들어오면
  바로 구현 가능합니다.
- **1프레임 드롭에 표적 바꿔치기** — `tracker.py:67`이 `lost_count > 0`이면 IoU와 무관하게 **아무
  검출이나 수용**합니다. 한 프레임 놓친 사이 다른 대상이 트랙을 가져갈 수 있고, 신원 재검증이 없습니다.
- **ESP32 속도 hint가 칼만 게이트를 우회** — `leader_telemetry.py:535-559`가 필터 상태에 직접
  대입하고 공분산을 무조건 축소합니다. innovation도 Mahalanobis 게이트도 likelihood도 없습니다.
- **`is_reliable()`이 거리 관측 없이도 참** — bearing-only 업데이트나 ESP32 업데이트만으로도
  `coast_time`이 0으로 리셋됩니다. 깊이를 잃어도 필터는 "잘 보고 있다"고 믿으므로, **"N초 안 보이면
  착륙" 판정이 발동하지 않을 수 있습니다.**
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
| `sitl/` | ArduCopter SITL 회귀 하네스 + 실행 안내 |
| `controller.py` | **죽은 코드** — 아무도 import하지 않음. 모터 직접 제어(FC 자세 안정화 우회)라 되살리지 말 것 |

## 검증 방법

드론 없이 4층으로 검증 가능합니다: 순수함수/프레임 단위 → `main.main()` 폐루프 시뮬
(`cv2`/`ultralytics`/`pyrealsense2`만 스텁) → MAVLink 와이어 → ArduCopter SITL 실비행.

둘 다 저장소에 있습니다 — `test_fixes.py`(1층)와 [`sitl/`](sitl/)(4층). SITL 하네스는 드론도
카메라도 없이 `numpy`와 `pymavlink`만으로 돌고, ArduCopter 빌드부터 차등 검증까지
[`sitl/README.md`](sitl/README.md)에 적어뒀습니다.

하드웨어 없이 검증 불가한 항목: 실제 RealSense 깊이 품질 · 드론 표적에 대한 YOLO 성능(현재
`person` 대용, 5m에서 X500은 약 50×50px) · ESP32 송신 펌웨어(미존재) · 실제 공력/바람/프롭워시 ·
GPS 품질 · Jetson 실제 FPS.

## 다음 할 일

1. **C6 검증** — PX4 SITL 필요 (나머지는 SITL 차등 검증 완료)
2. **기체 파라미터 `WP_YAW_BEHAVIOR=0`** — 비용 0이므로 권고 유지
3. **리더 속도 feed-forward** — 남은 결함 중 추종 품질에 가장 큰 영향(리더 1m/s당 4.55m 뒤처짐)
4. **`tracker.py` 신원 검증** — 1프레임 드롭 시 IoU 무시하고 아무 검출이나 수용하는 문제
5. `requirements.txt` 추가, `config.py`의 절대경로 제거
