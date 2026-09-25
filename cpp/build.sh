#!/usr/bin/env bash
# cpp/build.sh — C++ 코어 빌드 + gtest + 파이썬 차등 검증. 저장소 루트에서 실행.
#   ./cpp/build.sh            # 빌드·gtest·차등 검증 전부
#   ./cpp/build.sh --no-tests # 모듈만
#   ./cpp/build.sh --all-scenarios # + 폐루프 시나리오 전부를 C++ 코어로
set -euo pipefail
cd "$(dirname "$0")/.."
TESTS=ON
[[ "${1:-}" == "--no-tests" ]] && TESTS=OFF
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release -DMARS_BUILD_TESTS=${TESTS} > /dev/null
cmake --build cpp/build -j"$(nproc)"
if [[ "${TESTS}" == "ON" ]]; then
  (cd cpp/build && ctest --output-on-failure)
  echo "== 파이썬 오라클 대비 차등 검증 =="
  python3 -c "import mars_core; print('mars_core', mars_core.__version__)"
  python3 test_fixes.py | grep -E "C\+\+ 코어|FAILED|모든 검사 통과"
  MARS_CORE=cpp python3 test_closed_loop.py | tail -1
  if [[ "${1:-}" == "--all-scenarios" ]]; then
    for s in "tilt --level 0" tilt nan fc_stale "climb --frames 1900" "lost_alt --autonomous-land 1" "takeover --autonomous-land 1" sine; do
      echo "== $s"; python3 test_closed_loop.py --scenario $s --core cpp | tail -1
    done
  fi
fi
