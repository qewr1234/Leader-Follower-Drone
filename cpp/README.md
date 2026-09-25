# cpp/ — MARS-IMM C++ 코어 (`mars_core`)

파이썬 루프·인지(YOLO·RealSense)는 그대로 두고 **추정·제어 코어만 C++** 로 옮기는 자리입니다. 목적은 속도가 아니라
(파이썬 코어는 프레임당 0.6~1.1 ms) 이식성(ROS2 노드·FC 내부)과 결정론, 그리고 **파이썬 오라클 대비 차등 검증된 포팅** 이라는 이력입니다.

지금 들어 있는 것 (의존성 0 — 고정 크기 행렬은 `include/mars_core/mat.hpp`):

| 모듈 | 파이썬 원본 | C++ | 파이썬에서 보이는 이름 |
|---|---|---|---|
| IMM-EKF | `imm_ekf.ImmEkf` | `src/imm_ekf.cpp` | `mars_core.ImmEkf` (같은 메서드·반환) |
| 제어 법칙·필터·방벽 | `main.compute_velocity_cmd_from_estimate` / `smooth_velocity_cmd` / `leader_velocity_ff` / `self_velocity_lpf` / `level_fru_by_roll_pitch` / `sanitize_cmd` / `utils_geometry.clamp` | `src/control.cpp` | `mars_core.compute_velocity_cmd(gains, …)` 등 — 이득·한계는 `mars_core.ControlGains` 로 매 호출 전달 (`main.cpp_control_gains()` 가 모듈 상수에서 만든다) |
| 미션 상태머신 | `mission_manager.MissionManager` | `src/mission.cpp` | `mars_core.MissionManager` (같은 생성 인자·`update()` 반환 `(state, policy)`·속성) |

`MARS_CORE=cpp` 이면 main 이 셋을 모두 C++ 로 바꿔 끼운다 (`main.ImmEkf`, `main.MissionManager`, 제어 함수 다섯 개).
루프 골격·인지·MAVLink 는 파이썬 그대로다. 남는 것은 인지(YOLO·RealSense)뿐이고 그것은 이미 C/CUDA 라이브러리 안에서 돈다.

```bash
pip install pybind11                 # 헤더만 쓴다
./cpp/build.sh                       # cmake + gtest(googletest 를 FetchContent 로 받음) + 파이썬 차등 검증
MARS_CORE=cpp python3 main.py        # C++ 코어로 실행 (기본은 파이썬)
MARS_CORE=cpp python3 test_closed_loop.py --compare <golden.json>   # 골든 스트림과 비교
```

## 검증 세 겹

1. **gtest** (`tests/`): `test_imm_ekf.cpp` — 역행렬·행렬식, 초기화, 예측의 공분산 증가·코스트, 갱신의 공분산 감소, bearing 갱신, 순수 yaw 회전, 혁신, 속도 힌트·리셋, 20 000 스텝 코스팅 유한성. `test_control.cpp` — NaN/inf → 정지, front ≤ 0 → 정지, 축별 포화, 불확실성 감속 밴드 경계, 슬롯 오차, 피드포워드 이득, 평활의 dt 등가, 소프트 데드존·20 m/s 상한·오염된 prev, 자기 속도 LPF, 수평화의 노름 보존, sanitize. `test_mission.cpp` — C1(본 적 없음 ≠ 소실), 출발 확인 0.7 s 와 타이머 취소, FOLLOW↔LEADER_HOVER 히스테리시스, 소실 8 s → FAILSAFE_LAND, 재획득 즉시 재개 / 확인 필요, 착륙 후보 1.8 s → 확인, C3(고도 없으면 착륙 판정 없음), 가림이 타이머를 끊음, 공분산 > 8 → HOLD, 속도 소스 우선순위, NaN 속도는 아무것도 바꾸지 않음, reset.
2. **단위 차등** (`test_fixes.py` `C++ 코어:`): 같은 난수 입력을 파이썬과 C++ 에 넣어 비교 — EKF 600 스텝 × 3 회를 1e-9 (실측 1.2e-14), 제어 법칙 4000 조합(NaN/inf·front≤0·슬롯·밴드 경계·데드존·음수 dt)을 1e-12 (실측 2.7e-15), 미션 6 만 스텝 사건열(가림·속도 소스 3종·NaN·고도·공분산·reset)에서 상태·정책·타이머 완전 일치 + 8 개 상태 전부 통과.
3. **폐루프 차등** (`test_closed_loop.py --core cpp --compare`): 실제 `main.main()` 을 C++ 코어(EKF+제어+미션)로 1200 프레임 돌려 setpoint 스트림이 파이썬 골든과 같은지. 나머지 시나리오(`tilt`, `nan`, `fc_stale`, `climb --frames 1900`, `lost_alt --autonomous-land 1`, `takeover --autonomous-land 1`, `sine`)도 `--core cpp` 로 통과한다.

## 설계 규칙

- 연산 순서를 파이썬(numpy)과 같게 둔다 — `F·P·Fᵀ + Q`, Joseph 형, 혼합 합의 순서. 그래야 차등 검증이 1e-12 급으로 닫힌다.
- `-ffast-math` 금지, `-O2`, `-ffp-contract=off` — GCC 는 gnu++17 에서 `a*b+c` 를 FMA 로 묶는데(aarch64/Jetson 은 기본으로 나온다) numpy 는 안 묶으므로 끄지 않으면 1e-16 급 차이가 쌓인다.
- 제어 함수는 상태가 없다(이득은 인자). 파이썬 쪽 래퍼가 모듈 상수를 그때그때 `ControlGains` 로 실어 보내므로 분석 스크립트가 `main.KP_FORWARD` 를 바꿔 가며 스윕해도 두 코어가 같게 움직인다.
- 미션 상태는 C++ 안에 있고 파이썬에는 문자열(`"FOLLOW"` 등)로 보인다. 하네스의 `RecordingMission` 은 `main.MissionManager` 를 상속하므로 두 코어 모두 감싼다.
- 외부 의존성 0 (Eigen 없음). 나중에 ChibiOS/MCU 로 옮길 때 그대로 간다.
- 파이썬 API 와 같은 이름·반환 형태. 파이썬 쪽에서 필터 내부를 만지던 코드(속도 힌트)는 `apply_velocity_hint` 메서드로 대체하고, 검사용으로 `get_filter_x/set_filter_x/...` 를 둔다.
