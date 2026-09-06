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
| `hold_heading` | **C5** | 명령 yaw_rate=0인데 기수가 25° 이상 자체 회전 |

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

| 시나리오 | 수정 후 | 수정 전 (대조군) |
|---|---|---|
| `boot_no_leader` (C1) | PASS — 35초 GUIDED 유지 (947프레임) | **FAIL — t+0.9초에 LAND** |
| `pilot_takeover` (C2) | PASS — LOITER 25초 유지 (1073프레임) | **FAIL — 0.1초 만에 LAND로 뺏김** |
| `hold_heading` (C5) | 기수 편차 0.0° | 0.0° — **재현 안 됨** |

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

## 아직 커버하지 않는 것

C3(공중 착륙 판정) · C4(깊이 범위) · H2(호버 정위치 유지)는 `test_fixes.py`의 단위 테스트만
있습니다. C6(PX4 3-튜플 mode_mapping)은 PX4 SITL이 따로 필요합니다.

## 하네스가 스텁하는 것

`cv2`(전부 no-op) · `pyrealsense2` · `ultralytics` · `serial`, 그리고 `camera.D435i`와
`detector.YoloDetector`를 합성 리더를 만드는 가짜로 교체합니다. 깊이 영상은 리더 bbox 안만
실제 거리, 나머지는 9.5m 배경입니다. **그 외에는 저장소 코드가 그대로 돕니다.**
