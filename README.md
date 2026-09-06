# MARS-IMM — Leader-Follower Drone

**선두 드론을 눈으로 보고 따라가는 팔로워 컨트롤러.** RealSense D435i의 RGB-D 영상에서 YOLO11n으로
선두 드론을 탐지하고, IMM-EKF로 상대 위치·속도를 추정해 Pixhawk에 body-frame 속도 명령을 10Hz로
보냅니다. Jetson에서 단일 프로세스로 돕니다. 영남대 종합설계(캡스톤) 과제.

```
D435i ─▶ YOLO11n ─▶ 트래커 ─▶ IMM-EKF ─▶ 미션 상태머신 ─▶ P·D 제어 ─▶ MAVLink 10Hz
        (TensorRT)   (IoU)    (CV+CT 2모델)   (추종/호버/소실/착륙)      (BODY_NED)
```

## 핵심 결과

| | |
|---|---|
| **Jetson 실기** | 카메라 파이프라인 **24~26 FPS**, 선두 드론 탐지 → **모터 구동까지 지상 확인** |
| **SITL 비행** | ArduCopter SITL에서 **실제 `main.main()`을 그대로 비행**시켜 **8개 시나리오 전부 통과** |
| **제어 정확도** | 리더 0.3 m/s 추종 시 정상상태 거리 **4.3m — 이론값 4.36m와 소수 둘째 자리 일치** |
| **조종사 우선** | 비행 중 조종사가 스위치로 탈환하면 컴패니언이 즉시 물러남 (SITL 25초 유지 검증) |
| **자동 안전 착륙** | 선두를 놓치면 **정확히 10.0초 뒤 자동 LAND** (SITL 실측) |
| **회귀 스위트** | 단위 검사 35개 + SITL 시나리오 8개, 전부 **차등 검증** 방식 |
| **PX4 호환** | ArduCopter·PX4 양쪽 모드 프로토콜 지원, PX4 SITL로 검증 |

## 시스템 개요

단일 스레드 `while` 루프 하나가 전부입니다. ROS도 threading도 async도 없습니다 —
Jetson 한 장에서 인지부터 제어까지 42ms 주기로 돕니다.

```
D435i (color+depth, 640×480@30)
   └─> YOLO11n 검출 (scheduler가 ROI·검출 주기 결정)
         └─> IoU 단일 표적 트래커 (bbox 지수 평활, 근접 게이트로 신원 유지)
               └─> 측정 생성  RGB-D 3D  또는  bearing 2D
                     └─> 신뢰도 기반 R 팽창 + Mahalanobis 게이팅
                           └─> IMM-EKF (CV + Coordinated-Turn 2모델)
                                 ├─ ESP32 선두 텔레메트리 융합 (선택)
                                 └─> 미션 상태머신
                                       └─> P·D 제어 → body-frame 속도 setpoint
                                             └─> MAVLink SET_POSITION_TARGET_LOCAL_NED (10Hz)
```

**좌표계 체인**: pixel → camera → FRU → BODY_NED, ENU → FRU. 체인 전체를 SITL에서 검증했습니다
(명령 방향 대 실제 이동 방향 오차 -0.1°). 프레임 `MAV_FRAME_BODY_NED`(8), type_mask 1479
(속도 + yaw rate 사용, yaw 각도 무시). hold는 같은 마스크에 yaw_rate=0.0.

**24~26 FPS는 제어에 충분합니다.** setpoint는 10Hz로만 나가면 되고, 24 FPS면 루프가 42ms마다
도니 여유롭습니다 — SITL 실측 setpoint 9.1~9.2Hz, 최대 갭 0.138s (`GUID_TIMEOUT` 3.0s 대비).
평활 계수는 `dt` 보정을 넣어 실제 FPS와 무관하게 같은 응답 시정수를 갖습니다.

## 제어 법칙

네 축 모두 P+D로 제어합니다. 상대 위치는 IMM-EKF 추정값(FRU: front/right/up)입니다.

| 축 | 명령 | 식 |
|---|---|---|
| 전후 | 거리 유지 | `KP_FORWARD × (front − TARGET) + KD × v_front` |
| 좌우 | 측면 정렬 | `KP_RIGHT × right + KD × v_right` |
| 상하 | 고도 정렬 | `KP_UP × up + KD × v_up` |
| 기수 | 시야 중앙 유지 | `KP_YAW × atan2(right, front)` |

추정 불확실성(`pos_cov_trace`)이 크면 전체 명령에 0.55 / 0.75배 감속이 걸립니다.

**SITL 실측이 이론과 일치합니다.** 리더 0.3 m/s 추종 평형 거리 4.3m — 이론값
`TARGET + v/KP_FORWARD = 3.0 + 0.3/0.22 = 4.36m`와 소수 둘째 자리까지 일치. 제어기는
과감쇠·안정이며, 정지 리더 정상상태 오차 +0.00m, 5만 프레임 시뮬레이션에서 NaN·발산 0회.

## 안전 설계

비행 안전과 관련된 동작은 전부 SITL에서 실제 비행으로 검증했습니다.

- **조종사가 항상 이깁니다.** 컴패니언은 FC 모드를 매 루프 확인하고, GUIDED/OFFBOARD가
  아니면 모드 변경 명령을 보내지 않습니다. 조종사가 LOITER로 탈환하면 그대로 유지됩니다
  (SITL 25초 검증).
- **선두 소실 시 자동 착륙.** 거리 관측이 끊기면 호버로 버티다가 총 10초 뒤 LAND —
  깊이 센서만 죽고 검출이 살아 있는 교묘한 경우까지 거리 기준(`range_coast_time`)으로 잡습니다.
- **이륙 인계가 안전합니다.** 수동 상승 후 GUIDED로 넘기는 순간 미션·명령 버퍼를 리셋해
  깨끗한 상태로 추종을 시작합니다 (SITL 검증).
- **고도 바닥.** `MIN_AGL_M`(1.5m) 아래에서는 하강 명령을 차단합니다.
- **기본값이 dry-run.** `SEND_MAVLINK_COMMANDS = False`가 기본이라, 켜기 전까지는 인지·추정·
  화면 표시가 전부 돌면서 명령은 전송되지 않습니다.
- **카메라 hiccup 내성.** 프레임 드롭은 드롭으로 처리하고, 헤드리스(`MARS_SHOW_WINDOW=0`)
  환경에서도 동작합니다.

## 검증 방법

드론 없이 4층으로 검증합니다. 핵심은 **스텁이 인지 계층(cv2/RealSense/YOLO)뿐이고, 미션
상태머신·제어·MAVLink 송신은 저장소의 실제 코드가 그대로 돈다**는 점입니다.

```bash
python3 test_fixes.py          # 단위 35개 — numpy만 있으면 됨
python3 sitl/harness.py --all  # ArduCopter SITL에 붙여 실제로 비행
```

그리고 모든 시나리오는 **차등 검증**입니다 — 수정 전 코드에도 같은 시나리오를 돌려 대조군에서
문제가 실제로 재현되는지 확인합니다. 대조군이 통과하는 테스트는 아무것도 증명하지 못하기
때문입니다.

SITL 시나리오 8개: 부팅 대기 · 조종사 탈환 · 공중 오판 방지 · 이동 리더 추종 · 정지 리더
정위치 유지 · 기수 유지 · 소실 시 자동 착륙 · GUIDED 인계. 상세 이력과 실측값은
**[VERIFICATION.md](VERIFICATION.md)**, 하네스 실행법은 [sitl/README.md](sitl/README.md).

## 하드웨어

| | |
|---|---|
| 기체 | Holybro X500 V2 + Pixhawk |
| 컴패니언 | NVIDIA Jetson |
| 카메라 | Intel RealSense D435i (640×480 @ 30fps) |
| 선두 텔레메트리 | ESP32 (serial 115200 또는 UDP 5005) — 선택 사항, 비전 단독으로 동작 |

## 실행

```bash
pip install numpy opencv-python pymavlink ultralytics pyrealsense2 pyserial

python3 test_fixes.py          # 단위 회귀 (하드웨어 불필요)
python3 main.py
```

`MARS_FC_PORT` 환경변수로 FC 포트를 바꿀 수 있습니다(기본 `/dev/ttyACM0`).
SITL에 직접 붙이려면 `MARS_FC_PORT=udpin:0.0.0.0:14551 python3 main.py`.

키: `q`/`ESC` 종료 · `m` MARS-IMM 토글 · `v` MAVLink 송신 토글 · `l` LAND · `h` HOLD

모델 경로와 대상 클래스는 `config.py`에서 설정합니다. 시작 시 모델의 클래스 목록과 대조해
설정이 어긋나면 크게 경고합니다.

## 운용 순서

```
[지상]
  1. python3 test_fixes.py                     통과 확인
  2. 프로펠러 제거 상태로 python3 main.py       화면의 명령값이 의도대로 나오는지 확인
  3. SEND_MAVLINK_COMMANDS = True 로 수정       (또는 실행 중 v 키)
  4. 기체 파라미터 WP_YAW_BEHAVIOR=0

[비행]
  5. 조종기로 수동 이륙 (ALT_HOLD)              원하는 고도까지
  6. 고도 안착 후 모드 스위치를 GUIDED 로       ← 이 순간부터 추종 시작
  7. 이상하면 스위치를 LOITER/ALT_HOLD 로       ← 즉시 조종사에게 돌아옴
```

ArduCopter는 속도 setpoint를 **GUIDED에서만** 받으므로, 조종기 3단 스위치가 곧 자동/수동
전환 장치가 됩니다:

| 스위치 | 모드 | 의미 |
|---|---|---|
| 위 | ALT_HOLD | 수동 이륙·고도 잡기 |
| 중간 | LOITER | 비상 탈환 (위치까지 고정) |
| 아래 | **GUIDED** | 자동 추종 |

LOITER로 내리면 컴패니언이 다시 뺏지 못합니다 — SITL에서 검증했습니다.

## 모듈

| 파일 | 역할 |
|---|---|
| `main.py` | 제어 루프 전체 — 상수, 명령 생성, MAVLink 송신, 상태 표시 |
| `mission_manager.py` | 미션 상태머신 (WAIT_LEADER / READY_HOVER / FOLLOW / LOST_HOLD / FAILSAFE_LAND …) |
| `imm_ekf.py` | 2모델 IMM-EKF (CV + Coordinated-Turn) |
| `scheduler.py` | 검출기 on/off와 ROI 크기 스케줄링 |
| `reliability.py` | 신뢰도 기반 R 팽창 + Mahalanobis 게이팅 |
| `leader_telemetry.py` | ESP32 선두 GPS/속도 수신·파싱·ENU 변환·융합 |
| `measurement.py` | bbox + depth → RGB-D 3D / bearing 2D 측정 |
| `tracker.py` | IoU 단일 표적 트래커 (bbox 지수 평활, 근접 게이트) |
| `detector.py` | YOLO11n 래퍼 (TensorRT `.engine` 우선) |
| `camera.py` | D435i 래퍼 (depth→color 정렬, 실제 depth_scale 조회) |
| `utils_geometry.py` | 순수 기하 헬퍼 |
| `logger.py` | JSONL 스트리밍 로거 (종료 시 CSV 변환) |
| `test_fixes.py` | 단위 회귀 35개 (하드웨어·FC 불필요) |
| `sitl/` | SITL 회귀 하네스 · 실행 안내 |

## 문서

- [VERIFICATION.md](VERIFICATION.md) — 검증 방법론과 SITL 차등 검증 이력
- [sitl/README.md](sitl/README.md) — SITL 회귀 하네스 실행법
