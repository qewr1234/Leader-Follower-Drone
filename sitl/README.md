# SITL 회귀

저장소의 **실제 `main.main()`을** ArduCopter SITL에 붙여 비행시킵니다. 스텁은 인지 계층
(cv2 / RealSense / YOLO)뿐이고 미션 상태머신 · 제어 · MAVLink 송신은 전부 실제 코드입니다.
`SEND_MAVLINK_COMMANDS=True`로 돌기 때문에 SITL 기체가 실제로 움직입니다.

드론도 카메라도 필요 없습니다. 필요한 건 `numpy`와 `pymavlink`뿐입니다.

## 1. ArduCopter SITL 빌드

Apple Silicon 기준 클론 몇 분 + 빌드 40초 정도입니다. Docker 불필요.

```bash
git clone --depth 1 --recurse-submodules --shallow-submodules -j8 \
    https://github.com/ArduPilot/ardupilot.git
cd ardupilot
pip install pymavlink empy==3.3.4 pexpect future
./waf configure --board sitl
./waf copter -j8          # build/sitl/bin/arducopter
```

## 2. SITL 실행

포트를 **두 개** 엽니다 — 하나는 `main.py`(컴패니언), 하나는 하네스의 감시/조종사 링크입니다.

```bash
./build/sitl/bin/arducopter -I0 --model + --speedup 1 \
    --defaults Tools/autotest/default_params/copter.parm \
    --home 35.83,128.75,50,0 \
    --serial0 udpclient:127.0.0.1:14551 \
    --serial1 udpclient:127.0.0.1:14552
```

첫 하트비트까지 30초 남짓 걸립니다. 로그에 `Waiting for internal clock bits`가 떠 있어도
정상이며, 곧 하트비트가 나옵니다.

## 2-b. Windows (WSL1, 빌드 없이)

가상화를 못 켜는 PC 는 WSL1 로도 됩니다. ArduPilot 을 빌드하지 않고 배포된 리눅스 SITL 바이너리를 씁니다
(WSL1 에서 빌드는 30~60분, wxpython 컴파일이 특히 오래 걸리며 하네스에는 필요 없습니다).

```bash
# Ubuntu (WSL1) 안에서
mkdir -p ~/sitl && cd ~/sitl
wget https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter
wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/default_params/copter.parm
chmod +x arducopter
./arducopter -I0 --model + --speedup 1 --defaults copter.parm --home 35.83,128.75,50,0 \
    --serial0 udpclient:127.0.0.1:14551 --serial1 udpclient:127.0.0.1:14552
```

저장소는 `/mnt/c/...` 보다 WSL 홈(`~`)에 clone 하는 편이 빠릅니다. 하네스는 `numpy`, `pymavlink` 만 있으면 됩니다.

**콘솔 주의.** 기본 Ubuntu 콘솔은 QuickEdit 모드라 창을 클릭하거나 텍스트를 드래그하면 출력이 막히고
`print` 에서 프로세스 전체가 멈춥니다. 하네스 로그에 `FPS=0.2` 가 찍히거나 `main 이 15초 안에 종료되지 않음`
경고가 나오면 그것입니다. **Windows Terminal** 에서 Ubuntu 탭을 열어 돌리고, 실행 중에는 창을 건드리지 마세요.

## 3. 회귀 실행

```bash
python3 sitl/harness.py --all      # ArduCopter 시나리오 9개. px4_setmode는 PX4 엔드포인트 필요
```

하네스가 GUIDED 진입 → ARM → 이륙까지 알아서 하고, 시나리오마다 다시 이륙시킵니다. 직전 시나리오가
LAND 로 끝났으면(`depth_loss` 등) 착지·시동 해제를 기다린 뒤 이륙합니다 — ArduCopter 는 공중에서 보낸
GUIDED 이륙 명령을 거부합니다.

각 시나리오는 `프레임 N개 · PASS/FAIL` 로 끝납니다. **프레임 수가 1000개 안팎이 아니면 그 PASS/FAIL 은
믿지 마세요** — main 이 돌지 않은 채 시나리오만 흘러간 것입니다.

| 시나리오 | 검증 대상 | 실패 조건 |
|---|---|---|
| `boot_no_leader` | **C1** | 리더를 한 번도 못 봤는데 LAND로 전환 |
| `pilot_takeover` | **C2** | 조종사 LOITER 탈환이 LAND로 덮어써짐 |
| `air_landing` | **C3** | 리더가 보이는데 공중에서 착륙 판정 → LAND |
| `depth_range` | **C4** | 0.3 m/s 리더를 따라붙지 못함 (후반 거리 > 목표+3m) |
| `hover_hold` | **H2** | 리더 정지 후 목표거리로 수렴 못 함 (오차 > 1.5m) |
| `hold_heading` | **C5** | 명령 yaw_rate=0인데 기수가 25° 이상 자체 회전 |
| `px4_setmode` | **C6** | PX4의 3-튜플 `mode_mapping`에서 `set_mode`가 예외 |
| `depth_loss` | 거리 게이트 | 깊이만 죽었을 때(검출은 유지) 10초 뒤 착륙하지 않음 |
| `handover` | GUIDED 인계 | 수동 상승 후 GUIDED로 넘기는 순간 LAND가 나감 |
| `leader_sine` | 스트링 안정성 | 리더 0.25 ± 0.05 m/s 정현파(1.15 rad/s)에 대한 팔로워 속도 진폭비 > 1 (증폭) |

월드는 **폐루프**입니다 — 팔로워가 실제로 움직이면 리더까지의 거리가 변합니다. 팔로워 위치와
기수는 SITL의 `LOCAL_POSITION_NED` / `ATTITUDE`로 갱신되며, 그래야 정위치 유지(H2)나 거리
추종(C4)이 관측 가능해집니다.

`air_landing`은 `LOCAL_POSITION_NED`를 `main`의 시야에서만 가려 `leader_alt_est=None`
조건(EKF origin 설정 전에 실제로 발생)을 재현합니다. 로직을 건드리는 게 아니라 메시지가 없는
환경을 만드는 것입니다.

## 4. 차등 검증 — 이게 핵심입니다

**대조군이 통과하는 테스트는 아무것도 증명하지 못합니다.** 수정 전 코드에서 결함이 실제로
재현되는지 반드시 함께 확인하세요.

```bash
# 9863543 = C1~C6 수정 커밋. 그 부모가 수정 전 코드다.
git worktree add --detach /tmp/mars_before 9863543^
python3 sitl/harness.py --all --repo /tmp/mars_before
git worktree remove --force /tmp/mars_before
```

### 2026-09-07 실측

| 시나리오 | 수정 후 | 대조군 |
|---|---|---|
| `boot_no_leader` (C1) | PASS — 35초 GUIDED 유지 | **FAIL — t+0.9초에 LAND** |
| `pilot_takeover` (C2) | PASS — LOITER 25초 유지 | **FAIL — 0.6초 만에 LAND로 뺏김** |
| `air_landing` (C3) | PASS | **FAIL — t=2.6초, 고도 15m에서 착륙 판정** |
| `depth_range` (C4) | PASS — 후반 4.3m (목표 3.0m) | **FAIL — 10.8m (목표 5.0m)** |
| `hover_hold` (H2) | PASS — 후반 3.0m (목표 3.0m) | **FAIL — 6.0m에서 정지, 수렴 안 함** |
| `hold_heading` (C5) | 기수 편차 0.0° | 0.0° — **재현 안 됨** |
| `px4_setmode` (C6) | PASS — 예외 없음 | **FAIL — `required argument is not an integer`** |
| `depth_loss` (거리 게이트) | PASS — 깊이 소실 10.0초 뒤 LAND | **FAIL — 35초 내내 GUIDED, 착륙 안 함** |
| `handover` (GUIDED 인계) | PASS — 인계 후 LAND 없음 | **FAIL — 인계 0.1초 만에 LAND** |

C4 수정 후의 4.3m는 P 제어 평형거리 이론값 `TARGET + v/KP_FORWARD = 3.0 + 0.3/0.22 = 4.36m`와
소수 둘째 자리까지 일치합니다.

### 2026-09-18 실측 — Windows/WSL1, 배포 바이너리, 피드포워드 포함

pymavlink 2.4.49, `arducopter` stable 배포 바이너리(빌드 없음), 시나리오당 프레임 985~1019개.

| 시나리오 | 결과 | 관측 |
|---|---|---|
| `boot_no_leader` | PASS | 35초 GUIDED 유지, LAND 없음 |
| `pilot_takeover` | PASS | 18.0s LAND → 조종사 LOITER, 재 LAND 없음 |
| `air_landing` | PASS | 공중 착륙 판정 없음. 체인 실행에서 리더가 수직 FOV 를 벗어나 소실 failsafe 가 난 것은 preflight 상승 정착 대기로 해결 |
| `depth_range` | PASS | 후반 3.9m (피드포워드 전 4.4m). `vL=0.28 ff=+0.28 fresh=LP1/ATT1`, FOLLOW 유지. 3.3m 까지 못 간 것은 MAX_VX 0.35 − 리더 0.3 = 0.05 m/s 의 추격 여유 때문 (아래). 피드포워드 안정성 수정(82b4f74) 뒤 재실행에서는 4.1m — 소프트 데드존이 빼는 0.05 m/s 만큼(+0.18 m 예측) |
| `hover_hold` | PASS | 후반 3.0m |
| `hold_heading` | PASS | 기수 편차 0.0° |
| `depth_loss` | PASS | 깊이 소실 → LAND 10.0초 (2026-09-24 부터 `mission.autonomous_land=True` 일 때의 동작. 기본 False 는 FAILSAFE_LAND 에서 호버 유지) |
| `handover` | PASS | GUIDED 인계 후 LAND 없음 |
| `leader_sine` | PASS | 진폭비 0.72 (수정 전 대조군 1.95 FAIL). 아래 절 |

이 실행이 잡아낸 저장소 결함 2건과 하네스 결함 4건은 커밋 이력(cc427b5, e217a1f, 90fb807, e22ebee)에 있습니다.
가장 큰 것은 pymavlink 2.4.4x 가 `target_component` 를 0 으로 두는데 HEARTBEAT 필터가 컴포넌트 일치를
요구해 FC 모드가 영원히 `?` 로 남던 회귀 — 단위 검사는 가짜 master 의 컴포넌트를 1 로 두고 있어 못 잡았고
SITL 만 잡았습니다.

**`depth_range` 3.9m 해석.** 명령 = 0.8·v_L + 0.22·e 가 e > 0.6m 에서 0.35 m/s 상한에 걸린다. 리더가 0.3 m/s
라 추격 여유가 0.05 m/s 뿐이어서 FOLLOW 확정 시점의 2.1m 격차를 좁히는 데 30초가 걸리고, 후반 21~35초 평균은
3.8m 로 계산된다(관측 3.9m). 시나리오가 20초 더 길면 (1−KFF)·v/Kp = 0.27m 로 수렴한다. 운용상 의미:
`MAX_VX` 0.35 는 첫 비행용 보수값이고 리더가 그보다 빠르면 어떤 제어기로도 못 따라간다 — 지상 확인 뒤 올릴 것.

### 대조군은 결함별로 격리해야 합니다

**수정 전 전체 코드(초기 커밋)는 C3·C4·H2의 대조군으로 쓸 수 없습니다.** C1이 부팅 1~3초 만에
LAND를 걸어버려 다른 결함의 조건에 도달조차 못 하기 때문입니다. 위 표의 C3/C4/H2 대조군은
**수정본에서 해당 결함 하나만 되돌린** 사본입니다:

| 대조군 | 되돌린 것 |
|---|---|
| C3 | `mission_manager._extract_motion`의 상대 z fallback 복원 |
| C4 | `depth_max_m` 6.0, `TARGET_DISTANCE_M` 5.0 |
| H2 | `S_LEADER_HOVER`를 출발판단 블록으로 되돌림 |
| C6 | `set_mode`를 `set_mode_send(mode_mapping[name])`로 되돌림 |
| 거리 게이트 | `leader_visible_for_mission`을 `track_visible or ekf_reliable or esp_visible`로 되돌림 |
| 인계 | GUIDED 진입 에지의 미션·명령 리셋 블록 제거 |

C2 대조군 모드 이력이 결함을 그대로 보여줍니다:
`GUIDED(2s) → LAND(15.1s) → LOITER(15.1s, 조종사) → LAND(15.2s, 컴패니언이 되뺏음)`

**C5의 증상은 재현되지 않으며, 그 이유가 소스 수준에서 확정되었습니다.** 아래 참조.

## 유용한 옵션

```bash
--scenario NAME        # 하나만 실행
--duration 60          # 시나리오 길이 [s]
--alt 20               # 이륙 고도 [m]
--leader-front 6.0     # 리더 전방 거리 [m]
--force-target 5.0     # TARGET_DISTANCE_M 강제 (두 arm의 이동량 통제용)
--wp-yaw-behavior 2    # 기체 파라미터. C5 재현 시도 시 공장 기본값
--fc-port / --pilot-port
```

## C5 — 증상이 발생할 수 없는 이유 (`c5_probe.py`)

`sitl/c5_probe.py`가 C5의 본질만 직접 잽니다: **type_mask가 기수 제어권을 FC에 넘기는가.**
이륙 → 기수를 90°로 정렬 → 30초간 정지 setpoint를 지정한 마스크로 송신 → 기수 변화 측정.

```bash
python3 sitl/c5_probe.py 1479 3 30    # 수정 후 마스크
python3 sitl/c5_probe.py 3527 3 30    # 수정 전 마스크
```

실측 (ArduCopter 4.8.0-dev, `WP_YAW_BEHAVIOR=3`):

| 마스크 | 기수 최대 편차 |
|---|---|
| 1479 (수정 후) | 5.0° |
| 3527 (수정 전) | 1.0° |

**둘 다 기수를 유지합니다.** 이유는 세 가지이며 전부 소스로 확인됩니다:

1. **`WP_YAW_BEHAVIOR=2`(공장 기본값)는 LOOK_AHEAD가 아닙니다.** `autoyaw.cpp`의
   `default_mode()`에서 2는 `LOOK_AT_NEXT_WP_EXCEPT_RTL` → `LOOK_AT_NEXT_WP`이고,
   GUIDED 속도 제어에는 waypoint가 없습니다. LOOK_AHEAD는 값 3입니다.
2. **LOOK_AHEAD는 진입 시 현재 기수로 초기화됩니다** — `autoyaw.cpp:88-91`의
   `_look_ahead_yaw_rad = copter.ahrs.get_yaw_rad()`. 그리고 `look_ahead_yaw_rad()`는
   지면속도가 `YAW_LOOK_AHEAD_MIN_SPEED_MS`(=1 m/s)를 넘을 때만 갱신됩니다
   (`autoyaw.cpp:22`, `config.h:476`). 정지 상태면 진입 시점의 기수를 영원히 유지합니다.
3. **이 포크는 버그 마스크를 명령 속도가 0일 때만 내보냅니다.** `yaw_rate`는 `allow_follow`
   경로 안에서만 계산되므로, `yaw_rate == 0`이면 병진 명령도 0입니다. 계측 결과 마스크 3527이
   나간 620여 개 setpoint 전부에서 `|v| = 0.00`이었고, 1.0 m/s를 넘은 적은 한 번도 없습니다.
   즉 **버그 마스크가 필요로 하는 조건과 look-ahead가 필요로 하는 조건이 상호 배타적입니다.**

또한 이 기체의 후진 속도 상한은 `KP_FORWARD × (TARGET − front)`이므로 기본 게인
0.22에서는 약 0.66 m/s이며, look-ahead 문턱 1.0 m/s에 **구조적으로 도달할 수 없습니다.**

**결론**: 마스크 수정(항상 1479)은 그대로 둡니다 — 기수 유지를 FC 파라미터에 위임하는 대신
명시하는 것이 옳고 비용이 0입니다. 다만 이전 감사가 보고한 "164.6° 이탈"은 이 구성에서
발생할 수 없으며, C5는 **치명 등급이 아닙니다.**

## PX4 SITL (C6 전용)

C6은 **전송 전 패킹**에서 터집니다 — `set_mode_send`의 uint32 필드에 튜플을 넣는 순간입니다.
따라서 비행도 EKF도 필요 없고, PX4로 식별되는 MAVLink 엔드포인트만 있으면 재현됩니다.

```bash
git clone --depth 1 --recurse-submodules --shallow-submodules -j8 \
    https://github.com/PX4/PX4-Autopilot.git && cd PX4-Autopilot
brew install cmake ninja
pip install empy==3.3.4 toml jinja2 pyros-genmsg packaging kconfiglib jsonschema

# macOS에서 Gazebo 메시지 모듈이 protobuf 버전 충돌로 빌드에 실패한다.
# C6에는 시뮬레이터가 필요 없으므로 꺼도 된다.
sed -i.bak 's/^\(CONFIG_MODULES_SIMULATION_GZ_[A-Z]*\)=y/\1=n/' \
    boards/px4/sitl/default.px4board

# lockstep 빌드는 시뮬레이터(TCP 4560)를 기다리며 멈춘다 → nolockstep 변형을 쓴다
# (nolockstep.px4board는 default 위에 얹히는 델타라 위 설정을 그대로 물려받는다)
make px4_sitl_nolockstep

cd build/px4_sitl_nolockstep && mkdir -p rootfs_run && cd rootfs_run
PX4_SIM_MODEL=none_iris ../bin/px4 -d ../etc -s ../etc/init.d-posix/rcS
```

센서가 없어 arm은 안 되지만(“Preflight Fail: No valid data from Accel 0” 등) 하트비트는
정상적으로 나오며, 그것으로 충분합니다.

```bash
python3 sitl/harness.py --scenario px4_setmode --fc-port udpin:0.0.0.0:14540
```

실측: `autopilot=MAV_AUTOPILOT_PX4`, `mode_mapping['LAND'] = (29, 4, 6)` (튜플 확인).
수정 후는 예외 없이 통과하고, `set_mode`만 되돌린 대조군은 `set_mode`와 `send_land` 양쪽에서
`required argument is not an integer`로 실패합니다 — 실제 비행이었다면 제어 루프가 죽는 지점입니다.

## 하네스가 스텁하는 것

`cv2`(전부 no-op) · `pyrealsense2` · `ultralytics` · `serial`, 그리고 `camera.D435i`와
`detector.YoloDetector`를 합성 리더를 만드는 가짜로 교체합니다. 깊이 영상은 리더 bbox 안만
실제 거리, 나머지는 15m 배경(depth_max 10m 밖)입니다. **그 외에는 저장소 코드가 그대로 돕니다.**

## leader_sine — 스트링 안정성 (2026-09-18 추가·실측)

[docs/STABILITY_MARGINS.md](../docs/STABILITY_MARGINS.md) 의 선형 모델이 처음의 피드포워드 구현에서 1.15 rad/s 공진
(리더 속도 변동 1.8배 증폭)을 예측했는데, 등속·계단 시나리오는 그 주파수를 자극하지 않아 8개가 전부 통과했습니다.
`leader_sine` 은 리더를 `0.25 + 0.05·sin(1.15 t)` m/s 로 전진시키고, 정착 20 s 뒤 6주기 동안 팔로워의
`LOCAL_POSITION_NED.vx` 를 최소제곱으로 맞춰 **속도 진폭비**를 냅니다. 1 을 넘으면 FAIL. 시나리오 길이는 자동으로
약 53 s 가 됩니다.

```bash
python3 sitl/harness.py --scenario leader_sine                       # 현재 코드 (선형 예측 0.70)
git worktree add --detach /tmp/mars_before e4b4cd7                   # 수정 전 (FF 저역통과 0.7s, 정합 없음, 램프 데드밴드)
python3 sitl/harness.py --scenario leader_sine --repo /tmp/mars_before   # 대조군 (비선형 시뮬 예측 2.3 → FAIL 이어야 함)
git worktree remove --force /tmp/mars_before
```

로그의 `진폭비 x.xx (선형 예측: 수정 전 1.8, 현재 0.67)` 줄이 결과입니다.

| 저장소 | 진폭비 | 판정 | 관측 |
|---|---|---|---|
| 현재 (82b4f74) | **0.43** (단독 실행) / **0.72** (`--all` 안에서) | PASS | 팔로워 평균 0.27 m/s, 거리 평균 4.28 / 3.37 m, 거리 진폭 0.03 / 0.06 m. 미션 FOLLOW 유지 |
| 수정 전 e4b4cd7 | **1.95** | **FAIL** | 속도 진폭 0.097 m/s, 거리 진폭 0.11 m. 추정 리더 속도가 0.15~0.42 로 흔들려 미션이 FOLLOW↔LEADER_HOVER 를 5.5 s 주기로 오갔음 |
| 현재 (CSV 실행 `20260919-090834_current`) | **0.70** | PASS | 속도 진폭 0.035 m/s. CSV 와 그림이 저장소에 있음 |
| 수정 전 (CSV 실행 `20260919-091549_mars_before`) | **2.15** | **FAIL** | 속도 진폭 0.107 m/s. 진폭뿐 아니라 위상이 반대로 돌아 리더 가속 구간에 팔로워가 감속 |

2026-09-18 WSL1 실측(ArduCopter 4.5 SITL). 선형 모델 예측(현재 0.67, 수정 전 1.8)과 비선형 체인 시뮬 예측(0.70, 2.3) 사이에
들어옵니다. 현재 코드의 두 값 차이(0.43 / 0.72)는 단독 실행 중 콘솔 정지(FPS=0.1 한 번)가 정착 구간에 걸린 영향으로 보이며
둘 다 기준(1.0) 아래입니다. 수정 전 대조군에서는 증폭 자체보다 **미션 상태가 진동한 것**이 더 위험한 관측입니다 —
LEADER_HOVER 도 allow_follow 라 제어는 끊기지 않았지만, 실기에서는 착륙 후보 판정까지 흔들 수 있습니다.

## 5. 결과 CSV 와 실측 그림 (2026-09-19 추가)

하네스는 시나리오마다 `LOCAL_POSITION_NED` 10 Hz 로 시계열을 CSV 로 남깁니다 (`--no-csv` 로 끌 수 있음).

```
sitl/results/<실행시각>_<태그>/
  boot_no_leader.csv … leader_sine.csv   # t_s, front_m, agl_m, in_fov, fol_vn_mps, fol_n/e/d, leader_n/e/d, fc_mode, heading_deg
  summary.csv                            # scenario, PASS/FAIL, 프레임 수, 핵심 수치, 실패 사유
```

태그는 이 저장소면 `current`, `--repo` 로 다른 체크아웃을 돌리면 그 폴더 이름(예: `mars_before`)입니다. 첫 줄의 `#` 메타에
저장소 경로·시나리오 상수(정현파 ω·진폭·시작 시각)가 있어 그림 스크립트가 같은 최소제곱을 다시 계산합니다.

```bash
python3 sitl/harness.py --all                                             # → sitl/results/<시각>_current/
git worktree add --detach /tmp/mars_before e4b4cd7
python3 sitl/harness.py --scenario leader_sine --repo /tmp/mars_before   # → sitl/results/<시각>_mars_before/
git worktree remove --force /tmp/mars_before
git add sitl/results && git commit -m "SITL 결과 CSV" && git push         # 그림은 아래로

python3 analysis/sitl_figures.py sitl/results/<시각>_current --before sitl/results/<시각>_mars_before
#  → docs/images/sitl_regression.png (8개 시나리오 실측), docs/images/sitl_leader_sine.png (리더 vs 팔로워 속도, 진폭비)
```

## 6. 하네스 한계 — 조종사 역할이 고도를 유지하지 못한다 (2026-09-19 CSV 로 발견)

`pilot_takeover` 와 `handover` 는 `pilot.set_mode("LOITER")` 로 조종사 탈환을 흉내냅니다. 그런데 RC 입력이 없는
SITL 에서 LOITER 는 스로틀 채널을 최소로 읽어 기체가 지면까지 내려갑니다 (`agl_m` 이 0 으로 떨어지는 것을 CSV 에서
확인). 판정 자체는 유효합니다 — 우리 코드가 LAND 를 다시 보내지 않는 것, 인계 직후 LAND 가 없는 것을 봅니다. 다만
`handover` 가 의도한 "수동으로 12 초 상승한 뒤 공중에서 인계" 는 실제로는 지상에서의 인계였습니다.

고치려면 조종사 링크가 LOITER 중 RC override 로 스로틀을 중립 이상으로 보내야 합니다
(`pilot.mav.rc_channels_override_send`). 고친 뒤에는 `handover` 의 인계 고도가 15 m 근처로 유지되는지 CSV 의
`agl_m` 으로 확인하면 됩니다.
