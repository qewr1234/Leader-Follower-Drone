# MARS-IMM — Leader-Follower Drone

RealSense D435i로 선두 드론을 보고, YOLO11n으로 찾고, IMM-EKF로 상대 위치를 추정해
Pixhawk에 속도 명령을 보내는 **팔로워 컨트롤러**입니다. Jetson에서 단일 프로세스로 돕니다.
종합설계(캡스톤) 과제.

```
D435i ─▶ YOLO11n ─▶ 트래커 ─▶ IMM-EKF ─▶ 미션 상태머신 ─▶ P·D 제어 ─▶ MAVLink 10Hz
```

> **상태: 조건부 시험 비행 가능**
> 알려진 치명 결함은 모두 수정되었고 ArduCopter/PX4 SITL 차등 검증을 통과합니다.
> 조종사 상시 대기 · 리더 0.35 m/s 미만 · 저고도 개활지 조건에서의 **시험 비행**이
> 가능하다는 뜻이며, 운용 가능하다는 뜻은 아닙니다.
> 자세한 조건은 [운용 순서](#운용-순서), 검증 근거는 [VERIFICATION.md](VERIFICATION.md).

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
없어서 생기는 제약입니다([남은 결함](VERIFICATION.md#남은-결함) 참조).

SITL 실측으로 확인한 평형 거리: 리더 0.3 m/s에서 4.3m — 이론값 `TARGET + v/KP_FORWARD =
3.0 + 0.3/0.22 = 4.36m`와 소수 둘째 자리까지 일치합니다.

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

## 알아둘 것

### 프레임률은 병목이 아닙니다
**카메라 FPS는 병목이 아닙니다.** setpoint는 `SETPOINT_PERIOD_SEC = 0.10` 즉 10Hz로만 나가면
되고, 24 FPS면 루프가 42ms마다 도니 여유롭습니다(SITL 실측 setpoint 9.1~9.2Hz, 최대 갭 0.138s
vs `GUID_TIMEOUT` 3.0s). Jetson에서 관측된 **24~26 FPS로 충분하며, 30 FPS로 올려도 제어 품질은
개선되지 않습니다.**

다만 평활 계수가 프레임당 값이라 FPS가 바뀌면 응답 시정수가 따라 변했습니다(24fps와 30fps에서
25% 차이). `smooth_velocity_cmd`에 `dt`를 넘겨 30fps 기준으로 보정하므로, 이제 실제 FPS와
무관하게 같은 시정수를 갖습니다.

### 보안
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

### 한계
- **리더가 0.35 m/s를 넘으면 따라잡지 못합니다** (`MAX_VX`). D항이 상대속도라 정상상태
  기여가 0이고 feed-forward가 없습니다.
- ESP-NOW 송신 펌웨어가 아직 없어 **비전 단독 경로**로 동작합니다.
- 실제 비행 중의 검출 성능(진동·모션블러·조명), 바람/프롭워시, GPS 품질은 미검증입니다.
- 남은 결함 전체 목록은 [VERIFICATION.md](VERIFICATION.md#남은-결함).

## 라이선스 · 문서

- [VERIFICATION.md](VERIFICATION.md) — 결함 감사·수정·SITL 차등 검증 이력
- [sitl/README.md](sitl/README.md) — SITL 회귀 하네스 실행법
