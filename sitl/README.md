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

## 3. 회귀 실행

```bash
python3 sitl/harness.py --all
```

하네스가 GUIDED 진입 → ARM → 이륙까지 알아서 하고, 시나리오마다 다시 이륙시킵니다.

| 시나리오 | 검증 대상 | 실패 조건 |
|---|---|---|
| `boot_no_leader` | **C1** | 리더를 한 번도 못 봤는데 LAND로 전환 |
| `pilot_takeover` | **C2** | 조종사 LOITER 탈환이 LAND로 덮어써짐 |
| `air_landing` | **C3** | 리더가 보이는데 공중에서 착륙 판정 → LAND |
| `depth_range` | **C4** | 0.3 m/s 리더를 따라붙지 못함 (후반 거리 > 목표+3m) |
| `hover_hold` | **H2** | 리더 정지 후 목표거리로 수렴 못 함 (오차 > 1.5m) |
| `hold_heading` | **C5** | 명령 yaw_rate=0인데 기수가 25° 이상 자체 회전 |
| `px4_setmode` | **C6** | PX4의 3-튜플 `mode_mapping`에서 `set_mode`가 예외 |

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

C4 수정 후의 4.3m는 P 제어 평형거리 이론값 `TARGET + v/KP_FORWARD = 3.0 + 0.3/0.22 = 4.36m`와
소수 둘째 자리까지 일치합니다.

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

C2 대조군 모드 이력이 결함을 그대로 보여줍니다:
`GUIDED(2s) → LAND(15.1s) → LOITER(15.1s, 조종사) → LAND(15.2s, 컴패니언이 되뺏음)`

**C5는 이 하네스로 재현되지 않았습니다.** `--force-target`으로 두 arm의 이동량을 통제하고
`--wp-yaw-behavior 2`(공장 기본값)로 두어도 양쪽 다 0.0°였습니다. `WP_YAW_BEHAVIOR`가 GUIDED
속도 제어에는 적용되지 않고 waypoint 항법에만 적용되는 것으로 보입니다. 마스크 수정 자체는
`test_fixes.py`로 확인되지만, 그것이 막는다던 증상은 미검증 상태입니다.

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
실제 거리, 나머지는 9.5m 배경입니다. **그 외에는 저장소 코드가 그대로 돕니다.**
