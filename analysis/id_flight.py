#!/usr/bin/env python3
"""
id_flight.py — FC 속도루프 식별용 개루프 계단 명령 (비행 중, GUIDED)

선두 없이 팔로워만 GUIDED 에 넣고 BODY_NED 속도 계단(기본 ±0.3 m/s, 4 s)을 보내며 LOCAL_POSITION_NED 속도를 기록한다.
그 로그를 analysis/identify_plant.py 가 K·τ·L(1차 + 지연) 로 맞춘다 — docs/STABILITY_MARGINS.md 의 가정 τ_fc = 0.3 s 를 실측으로 바꾼다.

    python3 analysis/id_flight.py --axis forward              # dry-run: 명령을 찍기만 한다
    python3 analysis/id_flight.py --axis forward --send       # 실제 송신. 조종사가 GUIDED 로 넘긴 뒤에만 명령이 나간다

안전 (docs/FLIGHT_SAFETY_CHECKLIST.md 와 같은 규칙):
  - 진폭 상한 forward 0.5 / right 0.3 / up 0.15 m/s (인자로 못 넘김), 총 시간 상한 120 s
  - 매 주기(10 Hz) FC 모드 GUIDED·heartbeat ≤ 3 s·자세/위치 ≤ 1 s·고도 ≥ --min-alt·상승 ≤ --max-climb 를 확인, 하나라도 깨지면 정지 3회 + 종료
  - 조종사는 언제든 스위치로 탈환한다 (그 순간 FC 가 setpoint 를 버린다)
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

AMP_MAX = {"forward": 0.5, "right": 0.3, "up": 0.15}
AXIS_INDEX = {"forward": 0, "right": 1, "up": 2}
TOTAL_MAX_SEC = 120.0
PERIOD = 0.1


def step_sequence(axis, amp, step_sec=4.0, hold_sec=4.0, repeat=2):
    """[(지속 시간, [f, r, u] FRU 명령), ...]. 정지 → +amp → 정지 → −amp → 정지 을 repeat 회. 진폭은 축별 상한으로 자른다."""
    amp = min(abs(float(amp)), AMP_MAX[axis])
    i = AXIS_INDEX[axis]
    seq = [(hold_sec, [0.0, 0.0, 0.0])]
    for _ in range(int(repeat)):
        for sign in (+1.0, -1.0):
            cmd = [0.0, 0.0, 0.0]
            cmd[i] = sign * amp
            seq.append((step_sec, cmd))
            seq.append((hold_sec, [0.0, 0.0, 0.0]))
    total = sum(d for d, _ in seq)
    if total > TOTAL_MAX_SEC:
        raise ValueError(f"총 {total:.0f} s > 상한 {TOTAL_MAX_SEC:.0f} s — repeat/step 을 줄일 것")
    return seq


def fru_to_body_ned(cmd):
    return [float(cmd[0]), float(cmd[1]), -float(cmd[2])]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--axis", default="forward", choices=list(AMP_MAX))
    ap.add_argument("--amp", type=float, default=0.3)
    ap.add_argument("--step-sec", type=float, default=4.0)
    ap.add_argument("--hold-sec", type=float, default=4.0)
    ap.add_argument("--repeat", type=int, default=2)
    ap.add_argument("--send", action="store_true", help="실제로 setpoint 를 보낸다 (없으면 dry-run)")
    ap.add_argument("--min-alt", type=float, default=3.0, help="이 고도(EKF 원점 기준) 아래면 중단")
    ap.add_argument("--max-climb", type=float, default=2.0, help="시작 고도보다 이만큼 위면 중단")
    ap.add_argument("--out", default=None, help="JSONL 경로 (기본 logs/id_<axis>_<시각>.jsonl)")
    args = ap.parse_args()

    from mavlink_io import connect_fc, drain_messages, get_vehicle_state
    import main as m

    seq = step_sequence(args.axis, args.amp, args.step_sec, args.hold_sec, args.repeat)
    out = args.out or os.path.join("logs", f"id_{args.axis}_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    print(f"[ID] axis={args.axis} amp={min(abs(args.amp), AMP_MAX[args.axis]):.2f} 총 {sum(d for d, _ in seq):.0f} s "
          f"{'SEND' if args.send else 'DRY-RUN'} → {out}")
    print("[ID] 순서:", " ".join(f"{d:.0f}s@{c[AXIS_INDEX[args.axis]]:+.2f}" for d, c in seq))

    master = connect_fc()
    f = open(out, "w")
    alt0 = None
    t0 = time.monotonic()
    seg_i, seg_t0 = 0, None
    last_send = 0.0
    reason = "done"

    def stop(why):
        nonlocal reason
        reason = why
        print(f"[ID] 중단: {why}")
        if args.send:
            m.send_statustext(master, f"MARS ID: stop ({why})", m._SEV_CRITICAL)
            for _ in range(3):
                m.send_hold(master)
                time.sleep(0.05)

    try:
        while True:
            now = time.monotonic()
            drain_messages(master)
            vs = get_vehicle_state()
            mode_msg = vs.get("mode", {})
            mode = mode_msg.get("name", "?")
            hb_ok = m.is_fresh(mode_msg, now, m.FC_MODE_MAX_AGE_SEC)
            st_ok = m.is_fresh(vs.get("attitude", {}), now, m.FC_STATE_HOLD_AGE_SEC) and m.is_fresh(vs.get("local_position", {}), now, m.FC_STATE_HOLD_AGE_SEC)
            alt = m.get_follower_altitude_m(vs)
            v_fru = m.follower_velocity_fru(vs)

            if args.send:
                if not (hb_ok and mode in ("GUIDED", "OFFBOARD")):
                    if seg_t0 is not None:
                        stop(f"FC mode={mode} hb_ok={hb_ok}")
                        break
                    time.sleep(PERIOD)
                    continue                     # 조종사가 GUIDED 로 넘길 때까지 대기 (명령 없음)
                if not st_ok or alt is None:
                    stop("FC state stale"); break
                if alt < args.min_alt:
                    stop(f"alt {alt:.1f} < {args.min_alt}"); break
                if alt0 is None:
                    alt0 = alt
                if alt - alt0 > args.max_climb:
                    stop(f"climb {alt - alt0:.1f} > {args.max_climb}"); break

            if seg_t0 is None:
                seg_t0 = now
                print(f"[ID] 시작 (alt0={alt0 if alt0 is not None else float('nan'):.1f} m)")
            if now - seg_t0 >= seq[seg_i][0]:
                seg_i += 1
                seg_t0 = now
                if seg_i >= len(seq):
                    break
                print(f"[ID] 구간 {seg_i}/{len(seq) - 1}: {seq[seg_i][1]} for {seq[seg_i][0]:.0f} s")
            cmd = seq[seg_i][1]
            if now - last_send >= PERIOD:
                last_send = now
                if args.send:
                    m.send_body_velocity(master, *fru_to_body_ned(cmd), yaw_rate=0.0)
                row = {"t_mono": now, "t_wall": time.time(), "t_rel": now - t0, "seg": seg_i, "axis": args.axis, "send": bool(args.send),
                       "cmd_f": cmd[0], "cmd_r": cmd[1], "cmd_u": cmd[2], "mode": mode, "alt": alt,
                       "v_ned": [vs.get("local_position", {}).get(k) for k in ("vx", "vy", "vz")],
                       "yaw": vs.get("attitude", {}).get("yaw"),
                       "v_f": None if v_fru is None else float(v_fru[0]), "v_r": None if v_fru is None else float(v_fru[1]),
                       "v_u": None if v_fru is None else float(v_fru[2])}
                f.write(json.dumps(row) + "\n")
            time.sleep(0.01)
    except KeyboardInterrupt:
        stop("KeyboardInterrupt")
    finally:
        if args.send and reason == "done":
            for _ in range(3):
                m.send_hold(master)
                time.sleep(0.05)
        f.close()
        try:
            master.close()
        except Exception:
            pass
    print(f"[ID] 끝 ({reason}). 다음: python3 analysis/identify_plant.py --log {out} --margins")


if __name__ == "__main__":
    main()
