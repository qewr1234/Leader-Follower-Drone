# MARS-IMM — Leader-Follower Drone

**Mode-Aware Reliability-Scheduled IMM Tracking.** RealSense D435i + YOLO11n으로 선두 기체를
추적하고, IMM-EKF로 상대 상태를 추정해 Pixhawk에 속도 setpoint를 내보내는 단일 프로세스
follower 컨트롤러입니다. Jetson에서 동작하며 종합설계(캡스톤) 과제로 개발되었습니다.

---

> # ⚠️ 이 코드로 비행하지 마십시오
>
> 2026-08-24 SITL + 오프라인 시뮬레이션 감사에서 **치명적 결함 6건**이 발견되었고,
> **2026-09-06 재검증 결과 6건 모두 현재 코드에 그대로 남아 있습니다.** 수정된 것은 없습니다.
>
> 판정: **현재 코드로 실비행하면 정상 추종 확률은 사실상 0이며, 위험 사건 확률이 높습니다.**
> 특히 **조종사의 수동 탈환이 약 100~135ms 만에 덮어써집니다**(C2).
>
> 각 항목의 파일·행 번호와 최소 수정안은 [치명적 결함](#치명적-결함-6건--전부-미수정)에 있습니다.
> `SEND_MAVLINK_COMMANDS`는 기본값이 `False`이며, **위 항목을 고치기 전에는 켜지 마십시오.**

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
type_mask 1479(속도+yaw rate) / 3527(hold).

## 현재 상태

| 영역 | 상태 |
|---|---|
| 좌표계 변환 체인 | ✅ SITL 2회 독립 재현, 오차 -0.1° |
| `BODY_NED` + mask 1479/3527 | ✅ body 기준으로 정확히 동작 (`BODY_OFFSET_NED`로 바꾸지 말 것 — PX4가 거부) |
| 제어 법칙 자체 | ✅ 과감쇠·안정. 정지 리더 정상상태 오차 +0.00m, 5만 프레임 NaN/발산 0 |
| 10Hz setpoint 유지 | ✅ 실측 9.1~9.2Hz, 최대 갭 0.138s (`GUID_TIMEOUT` 3.0s 대비 여유) |
| coast / 재획득 | ✅ 0.5~3초 블랙아웃 전부 정상 |
| **미션 상태머신** | ❌ 부팅 즉시 LAND, 공중 착륙 판정, 호버 추종 1프레임 |
| **센서 유효범위** | ❌ 깊이 6m 절벽에 5m 추종 |
| **조종사 탈환** | ❌ 100~135ms 만에 덮어써짐 |
| **PX4 경로** | ❌ 첫 LAND 시도에서 예외로 루프 사망 |
| 실비행 | ⛔ **한 번도 수행된 적 없음** |

## 치명적 결함 6건 — 전부 미수정

2026-09-06 기준 코드에서 직접 확인했습니다.

### C1 — 부팅 즉시 FAILSAFE_LAND

`mission_manager.py:222-225`가 `last_seen_t is None`일 때 `999.0`을 반환합니다.

```python
def _lost_time(self, now):
    if self.last_seen_t is None:
        return 999.0
    return float(now - self.last_seen_t)
```

리더를 **한 번도 본 적 없는 부팅 직후**가 정확히 이 조건입니다. `999.0 > lost_hold_sec(5.0)`이므로
`S_LOST_HOLD`를 건너뛰고 첫 프레임에 `S_FAILSAFE_LAND`로 진입합니다. 명령 송신을 켜는 순간
(= 보통 이륙 직후, 리더 획득 전) 다음 루프에서 LAND가 나갑니다.

**수정**: "한 번도 못 봄"과 "놓침"을 분리. `mission_manager.py:86` 앞에 `last_seen_t is None`이면
`S_WAIT_LEADER`로 보내는 가드 추가.

### C2 — 조종사 수동 탈환이 무효화됨

`main.py:787-792`의 LAND 송신에 **latch도, FC 모드 확인도 없습니다.**

```python
if mission_policy["land"]:
    desired_body_cmd = np.zeros(4, dtype=float)
    if SEND_MAVLINK_COMMANDS and now - last_setpoint_time >= SETPOINT_PERIOD_SEC:
        send_land(master)
        last_setpoint_time = now
```

`SETPOINT_PERIOD_SEC = 0.10`이므로 **100ms마다 재송신**되고, `send_land`의 첫 분기가
`set_mode(master, "LAND")` — 능동적인 모드 변경입니다. 조종사가 LOITER로 탈환해도 약
100~135ms 만에 다시 LAND로 끌려갑니다. FC 모드는 `main.py:523`의 1Hz 출력 블록에서만 읽고
경고만 찍을 뿐, 송신 지점에서는 참조하지 않습니다. `manual_override`는 `main.py:779`에
`False`로 하드코딩되어 있어 `S_MANUAL_OVERRIDE` 상태는 죽은 코드입니다.

**수정**: 모드 읽기를 루프 상단으로 올리고, `fc_mode not in ("GUIDED","OFFBOARD")`이면 모든 송신을
하드 스톱. LAND에 one-shot latch 추가.

### C3 — 공중에서 착륙 판정 통과

`mission_manager.py:214-216`이 리더 절대고도가 `None`일 때 **상대** z로 대체합니다. 동고도
편대비행에서 상대 z는 0 근처이므로 고도 조건(`< 0.65m`)이 **항상 만족**됩니다. 그러면
`CONFIRMED_LANDING`은 하강속도와 수평속도만으로 결정되어, 공중에서 착륙 판정이 납니다.
`LOCAL_POSITION_NED`는 EKF origin 설정 전에는 아예 오지 않으므로 `None`은 흔한 조건입니다.

**수정**: fallback 삭제. `z_for_landing = float(leader_alt) if leader_alt is not None else None`.

### C4 — 깊이 6m 절벽에 5m 추종

`config.py:24` `depth_max_m = 6.00` vs `main.py:103` `TARGET_DISTANCE_M = 5.0` — 여유 1m뿐입니다.
6m를 넘으면 픽셀이 전부 버려지고 `build_rgbd`가 `None`을 반환해 **거리 관측이 불가능한
bearing-only로 조용히 강등**됩니다. `MAX_VX = 0.35`로는 멀어지는 리더를 따라잡지 못합니다.
`max_depth_mad`(`config.py:31`)는 **트리 전체에서 이 한 줄이 유일한 등장** — 정의만 되고 아무도
쓰지 않는 죽은 상수입니다.

**수정**: `depth_max_m`를 10.0으로, `TARGET_DISTANCE_M`을 3.0으로. `max_depth_mad`를 실제 게이트로 연결.

### C5 — yaw mask가 기체를 스스로 돌게 함

`main.py:285-286`이 `yaw_rate ≈ 0`일 때 `YAW_RATE_IGNORE` 비트를 세웁니다. 그러면 ArduCopter가
`WP_YAW_BEHAVIOR`(공장 기본값 2)로 **기수를 스스로 돌립니다.** 프레임이 `BODY_NED`이므로 이후
모든 속도 명령이 기체가 임의로 정한 heading 기준으로 재해석됩니다. SITL 실측에서 실제 이동이
명령 대비 **164.6° 어긋났습니다.** `send_hold()`가 `yaw_rate=0.0`을 그대로 넘기므로 이 분기는
비행 중 확실히 도달합니다.

**수정**: `main.py:285-286` 두 줄 삭제(항상 mask 1479). 기체 파라미터 `WP_YAW_BEHAVIOR=0`도 함께 설정.

### C6 — PX4에서 LAND 시도 시 루프 사망

`main.py:312`가 `mode_mapping()[name]`을 int로 가정합니다. ArduPilot은 int지만 **PX4는 3-튜플**
(`"LAND"` → `(29, 4, 6)`)을 반환합니다. `main.py:308`의 가드는 키 존재만 확인하고 타입은 보지
않으므로, uint32 필드에 튜플을 담다가 `struct.error`가 발생합니다. `main.py:496`~`1048` 사이에
`except KeyboardInterrupt` 외에는 예외 처리가 없어 **비행 중 제어 루프가 죽습니다.**
`AUTO.LAND` 재시도와 `MAV_CMD_NAV_LAND` fallback에는 도달조차 못 합니다.

**수정**: `master.set_mode(mode_name)` 사용(pymavlink가 PX4/ArduPilot을 자동 분기). `set_mode`를
non-fatal로 만들어 fallback이 실제로 동작하게 할 것.

## 그 밖의 주요 결함

- **호버 리더 정위치 유지가 존재하지 않음** — `mission_manager.py:134`의 `S_LEADER_HOVER`가 잘못된
  분기에 있어 **정확히 1프레임(~33ms)** 생존 후 `READY_HOVER`로 붕괴합니다. 헤드라인 유스케이스가
  동작하지 않습니다.
- **추종 중 ~1Hz 스터터** — 따라잡아서 상대속도가 줄면 상태머신이 "리더가 멈췄다"로 오판해 FOLLOW를
  빠져나가고, 멈추면 다시 상대속도가 올라 재진입합니다. "리더가 정지"와 "우리가 따라잡음"을 구분하지
  못합니다. 리더 속도 0.25~0.35 m/s 구간에서 발생합니다.
- **정상상태 뒤처짐** — 순수 P제어에 D항이 상대속도라 정상상태 기여가 0입니다. 리더 1m/s당 약 4.55m
  뒤처집니다(이론값과 소수 2자리 일치). 리더 속도 feed-forward가 필요합니다.
- **1프레임 드롭에 표적 바꿔치기** — `tracker.py:67`이 `lost_count > 0`이면 IoU와 무관하게 **아무
  검출이나 수용**합니다. 한 프레임 놓친 사이 화면 반대편의 다른 대상이 트랙을 가져갈 수 있고,
  이후 신원 재검증이 없습니다.
- **ESP32 속도 hint가 칼만 게이트를 우회** — `leader_telemetry.py:535-559`가 필터 상태에 직접
  대입하고 공분산을 무조건 축소합니다. innovation도, Mahalanobis 게이트도, likelihood도 없어
  모드 확률이 이를 보지 못합니다. 잘못된 값이 무저항으로 들어옵니다.
- **`p_ct`가 사전확률 0.333에서 벗어나지 않음** — 37개 시나리오 중 34개에서 "mode-aware"가
  사실상 무동작이었습니다.
- **`detector_skipped` 페널티 도달 불가** — 플래그가 측정 dict에 복사되지 않아, 직전 bbox를
  재사용한 프레임도 신뢰도 1.0으로 EKF에 들어갑니다.
- **조작자 키가 전부 `if SHOW_WINDOW:` 안** — `ssh -X` 헤드리스 환경에서 탈출구가 없습니다.
- **시작 실패 지점이 잘못됨** — `leader_rx.start()`가 `main.py:466`, 즉 `try` 블록(`main.py:494`)
  **밖**에서 호출됩니다. ESP32가 안 꽂혀 있으면 루프 진입 전에 `SerialException`으로 죽습니다.
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
| 기체 | Holybro X500 V2 + Pixhawk (ArduCopter 검증 / PX4는 C6로 미동작) |
| 컴패니언 | NVIDIA Jetson |
| 카메라 | Intel RealSense D435i (640×480 @ 30fps) |
| 선두 텔레메트리 | ESP32 (serial `/dev/ttyUSB0` 115200 또는 UDP 5005) — **송신 펌웨어는 이 저장소에 없습니다** |

## 실행

```bash
# 의존성 (requirements.txt 없음 — 수동 설치)
pip install numpy opencv-python pymavlink ultralytics pyrealsense2 pyserial

python3 main.py
```

키: `q`/`ESC` 종료 · `m` MARS-IMM 토글 · `v` MAVLink 송신 토글 · `l` LAND · `h` HOLD
(모두 `SHOW_WINDOW`가 True인 GUI 창에서만 동작)

**CLI 인자도 환경변수도 없습니다.** 모든 설정은 `main.py` 상단 상수와 `config.py`를 직접 편집합니다.

주요 기본값 (`main.py`):

| 상수 | 값 |
|---|---|
| `SEND_MAVLINK_COMMANDS` | `False` (dry-run) |
| `TARGET_DISTANCE_M` | 5.0 |
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
| `controller.py` | **죽은 코드** — 아무도 import하지 않음. 모터 직접 제어(FC 자세 안정화 우회)라 되살리지 말 것 |

## 검증 방법

드론 없이 4층으로 검증 가능함이 확인되었습니다: 순수함수/프레임 단위 → `main.main()` 폐루프
시뮬(`cv2`/`ultralytics`/`pyrealsense2`만 스텁, 37 시나리오 5만 프레임) → MAVLink 와이어 →
ArduCopter SITL 실비행. 다만 **하네스 코드는 이 저장소에 포함되어 있지 않습니다.**

하드웨어 없이 검증 불가한 항목: 실제 RealSense 깊이 품질 · 드론 표적에 대한 YOLO 성능(현재
`person` 대용, 5m에서 X500은 약 50×50px) · ESP32 송신 펌웨어(미존재) · 실제 공력/바람/프롭워시 ·
GPS 품질 · Jetson 실제 FPS.

## 다음 할 일

1. **C1~C6 수정** — 이것 없이는 어떤 실비행도 안 됩니다
2. `mavlink_io.py:15`의 포트를 `os.environ.get("MARS_FC_PORT", "/dev/ttyACM0")`로 (3줄).
   현재 하드코딩 때문에 **FC 코드가 실제 펌웨어에 붙어 실행된 적이 한 번도 없습니다.**
   고치면 `MARS_FC_PORT=udpin:0.0.0.0:14550 python main.py`로 SITL 상시 회귀가 가능해집니다
3. 기체 파라미터 `WP_YAW_BEHAVIOR=0` 설정 (C5와 세트)
4. `requirements.txt` 추가, `config.py`의 절대경로 제거
