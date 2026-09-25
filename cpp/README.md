# cpp/ — MARS-IMM C++ 코어 (`mars_core`)

파이썬 루프·인지(YOLO·RealSense)는 그대로 두고 **추정·제어 코어만 C++** 로 옮기는 자리입니다. 목적은 속도가 아니라
(파이썬 코어는 프레임당 0.6~1.1 ms) 이식성(ROS2 노드·FC 내부)과 결정론, 그리고 **파이썬 오라클 대비 차등 검증된 포팅** 이라는 이력입니다.

지금 들어 있는 것: `ImmEkf` (imm_ekf.py 의 1:1 이식, 의존성 0 — 6×6 고정 행렬은 `include/mars_core/mat.hpp`).
다음 차례: 제어 법칙(`compute_velocity_cmd_from_estimate` + 방벽) → 미션 상태머신.

```bash
pip install pybind11                 # 헤더만 쓴다
./cpp/build.sh                       # cmake + gtest(googletest 를 FetchContent 로 받음) + 파이썬 차등 검증
MARS_CORE=cpp python3 main.py        # C++ 코어로 실행 (기본은 파이썬)
MARS_CORE=cpp python3 test_closed_loop.py --compare <golden.json>   # 골든 스트림과 비교
```

## 검증 세 겹

1. **gtest** (`tests/test_imm_ekf.cpp`): 역행렬·행렬식, 초기화, 예측의 공분산 증가·코스트, 갱신의 공분산 감소, bearing 갱신, 순수 yaw 회전의 정확성, 혁신, 속도 힌트·리셋, 20 000 스텝 코스팅 유한성.
2. **단위 차등** (`test_fixes.py` `C++ 코어:`): 같은 난수 입력열(예측·위치·bearing·자세 회전·힌트·리셋)을 파이썬과 C++ 에 넣어 상태·공분산·모드 확률·혁신을 1e-9 로 비교.
3. **폐루프 차등** (`test_closed_loop.py --core cpp --compare`): 실제 `main.main()` 을 C++ 코어로 1200 프레임 돌려 setpoint 스트림이 파이썬 골든과 같은지.

## 설계 규칙

- 연산 순서를 파이썬(numpy)과 같게 둔다 — `F·P·Fᵀ + Q`, Joseph 형, 혼합 합의 순서. 그래야 차등 검증이 1e-12 급으로 닫힌다.
- `-ffast-math` 금지, `-O2`.
- 외부 의존성 0 (Eigen 없음). 나중에 ChibiOS/MCU 로 옮길 때 그대로 간다.
- 파이썬 API 와 같은 이름·반환 형태. 파이썬 쪽에서 필터 내부를 만지던 코드(속도 힌트)는 `apply_velocity_hint` 메서드로 대체하고, 검사용으로 `get_filter_x/set_filter_x/...` 를 둔다.
