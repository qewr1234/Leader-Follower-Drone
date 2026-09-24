#!/usr/bin/env python3
"""
sine_gain.py — 리더 정현파 시험 로그에서 리더→팔로워 속도 이득 |Γ(jω)| 과 위상을 뽑는다 (스트링 안정성의 실측점)

팔로워 속도 v_F: LOCAL_POSITION_NED 를 yaw 로 돌린 전진 성분.
리더 속도 v_L 의 출처(--leader):
  esp32 : 텔레메트리 절대 속도(ENU → 후미 FRU 전진 성분)
  uwb   : 거리 GT r(t) 의 페이저로. r = x_L − x_F 이므로 V_L = jω·R + V_F (수치 미분 없이 페이저 합성)
  ekf   : EKF 상대속도 v_front + v_F
  auto  : esp32 → uwb → ekf 순으로 있는 것
각 신호에 y = A sin ωt + B cos ωt + C + D t 를 최소제곱으로 맞춰 진폭·위상을 얻고 |Γ| = |V_F|/|V_L|, 위상 = ∠V_F − ∠V_L.
--predict 면 analysis/stability_margins.py 의 선형 모델 Γ(jω) 와 비교한다 (SITL leader_sine: 예측 0.70, 실측 0.72).

    python3 analysis/sine_gain.py --log logs/mars_imm_*.jsonl --omega 1.15 --t0 15 --t1 40 --leader uwb --predict
    python3 analysis/sine_gain.py --selftest
"""

import argparse
import cmath
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from analysis.logtools import Log, follower_vel_fru, enu_to_fru, fit_sinusoid, dominant_omega  # noqa: E402


def leader_phasors(log, omega, t, mask, v_f, source):
    """(리더 속도 페이저 V_L, 설명 문자열, 적합 dict)."""
    if source == "esp32":
        avail = log.col_bool("leader_esp32.available") & mask
        if avail.sum() < 8:
            return None, "esp32 없음", None
        v_enu = log.vec("leader_esp32.leader_vel_enu", 3)
        yaw = log.col("vehicle_state.attitude.yaw")
        vl = enu_to_fru(v_enu, yaw)[:, 0]
        fit = fit_sinusoid(t[avail], vl[avail], omega)
        return fit["phasor"], "esp32 절대 속도", fit["amp_std"]
    if source == "uwb":
        avail = log.col_bool("uwb.available") & mask
        if avail.sum() < 8:
            return None, "uwb 없음", None
        r = log.col("uwb.range_center_m")
        fr = fit_sinusoid(t[avail], r[avail], omega)
        ff = fit_sinusoid(t[mask], v_f[mask], omega)
        # 거리 진폭은 속도 진폭/ω 라 높은 ω 에서 UWB 잡음에 묻힌다 (0.05 m/s·1.15 rad/s → 0.025 m). 불확실성을 같이 준다.
        return 1j * omega * fr["phasor"] + ff["phasor"], f"uwb 거리 페이저 (R={fr['amp']:.3f}±{fr['amp_std']:.3f} m)", omega * fr["amp_std"]
    if source == "ekf":
        vrel = log.col("relative_fru.v_front")
        ok = np.isfinite(vrel) & mask
        fit = fit_sinusoid(t[ok], vrel[ok] + v_f[ok], omega)
        return fit["phasor"], "EKF 상대속도 + 자기 속도", fit["amp_std"]
    raise ValueError(source)


def analyze(log, omega=None, t0=None, t1=None, leader="auto"):
    t = log.t
    v_f = follower_vel_fru(log)[:, 0]
    mask = np.isfinite(v_f)
    if t0 is not None:
        mask &= t >= t0
    if t1 is not None:
        mask &= t <= t1
    if mask.sum() < 16:
        raise ValueError("창 안의 표본이 부족")
    if omega is None:
        omega = dominant_omega(t[mask], v_f[mask])
    ff = fit_sinusoid(t[mask], v_f[mask], omega)
    sources = ["esp32", "uwb", "ekf"] if leader == "auto" else [leader]
    for src in sources:
        VL, desc, amp_l_std = leader_phasors(log, omega, t, mask, v_f, src)
        if VL is not None:
            break
    else:
        raise ValueError("리더 속도 출처를 못 찾음")
    gain = abs(ff["phasor"]) / max(abs(VL), 1e-9)
    phase = math.degrees(cmath.phase(ff["phasor"] / VL)) if abs(VL) > 1e-9 else float("nan")
    # 이득 불확실성 (1차 전파, 독립 가정)
    gain_std = gain * math.sqrt((ff["amp_std"] / max(ff["amp"], 1e-9)) ** 2 + (float(amp_l_std) / max(abs(VL), 1e-9)) ** 2)
    return {"omega": float(omega), "t0": float(t[mask].min()), "t1": float(t[mask].max()), "n": int(mask.sum()),
            "leader_source": src, "leader_desc": desc, "amp_leader": float(abs(VL)), "amp_leader_std": float(amp_l_std),
            "amp_follower": float(ff["amp"]), "amp_follower_std": float(ff["amp_std"]), "gain": float(gain), "gain_std": float(gain_std),
            "phase_deg": float(phase), "resid_std_follower": float(ff["resid_std"]), "offset_follower": float(ff["offset"])}


def predicted_gain(omega, axis="forward"):
    import analysis.stability_margins as sm
    frf = sm.EkfFrf(sm.load_frf())
    gains = {"forward": (sm.main.KP_FORWARD, sm.main.KD_FORWARD), "right": (sm.main.KP_RIGHT, sm.main.KD_RIGHT), "up": (sm.main.KP_UP, sm.main.KD_UP)}
    p = sm.Params(kp=gains[axis][0], kd=gains[axis][1])
    G = sm.leader_to_follower(p, np.array([float(omega)]), frf)[0]
    return {"gain": float(abs(G)), "phase_deg": float(math.degrees(cmath.phase(G)))}


def selftest():
    """알려진 이득 0.7·위상 −35° 의 팔로워 응답을 합성해 세 출처 모두에서 되찾는다."""
    rng = np.random.default_rng(3)
    omega, T = 1.15, 40.0
    t = np.arange(0, T, 1 / 30)
    v_l = 0.25 + 0.05 * np.sin(omega * t)
    G = 0.7 * cmath.exp(-1j * math.radians(35))
    v_f = 0.25 + (0.05 * (G * np.exp(1j * omega * t))).imag + rng.normal(0, 0.01, t.size)
    x_l = np.cumsum(v_l) / 30; x_f = np.cumsum(v_f) / 30
    r = 3.0 + x_l - x_f + rng.normal(0, 0.03, t.size)
    rows = []
    for i in range(t.size):
        rows.append({"t_mono": float(t[i]), "vehicle_state.local_position.vx": float(v_f[i]), "vehicle_state.local_position.vy": 0.0,
                     "vehicle_state.local_position.vz": 0.0, "vehicle_state.attitude.yaw": 0.0,
                     "leader_esp32.available": True, "leader_esp32.leader_vel_enu": f"0;{v_l[i]};0",
                     "uwb.available": True, "uwb.range_center_m": float(r[i]), "relative_fru.v_front": float(v_l[i] - v_f[i])})
    log = Log(rows)
    ok = True
    for src in ("esp32", "uwb", "ekf"):
        res = analyze(log, omega=None, t0=5, leader=src)
        tol = 0.03 if src != "uwb" else 0.06        # uwb: 거리 진폭 0.025 m 대 잡음 0.03 m → 이득 σ ≈ 0.03
        good = abs(res["gain"] - 0.7) < tol and abs(res["phase_deg"] + 35) < 5 and abs(res["omega"] - omega) < 0.05 and abs(res["gain"] - 0.7) < 2.5 * res["gain_std"] + 0.02
        ok &= good
        print(f"[selftest] {src:5s} ω={res['omega']:.3f} gain={res['gain']:.3f}±{res['gain_std']:.3f} phase={res['phase_deg']:+.1f}° {'OK' if good else 'FAIL'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log")
    ap.add_argument("--omega", default="auto", help="rad/s 또는 auto (팔로워 속도의 주기도 최대)")
    ap.add_argument("--t0", type=float, default=None); ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--leader", default="auto", choices=["auto", "esp32", "uwb", "ekf"])
    ap.add_argument("--predict", action="store_true", help="선형 모델 Γ(jω) 와 비교")
    ap.add_argument("--json", default=None)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(0 if selftest() else 1)
    if not args.log:
        ap.error("--log 필요")
    omega = None if args.omega == "auto" else float(args.omega)
    res = analyze(Log.load(args.log), omega, args.t0, args.t1, args.leader)
    print(f"ω={res['omega']:.3f} rad/s  창 {res['t0']:.1f}~{res['t1']:.1f} s (n={res['n']})  리더={res['leader_desc']}")
    print(f"리더 진폭 {res['amp_leader']:.4f} m/s  팔로워 진폭 {res['amp_follower']:.4f} ± {res['amp_follower_std']:.4f} m/s")
    print(f"|Γ| = {res['gain']:.3f} ± {res['gain_std']:.3f}   위상 {res['phase_deg']:+.1f}°   잔차 σ {res['resid_std_follower']:.4f} m/s")
    if args.predict:
        pr = predicted_gain(res["omega"])
        res["predicted"] = pr
        print(f"선형 모델 예측 |Γ| = {pr['gain']:.3f} (위상 {pr['phase_deg']:+.1f}°)  →  실측/예측 = {res['gain'] / pr['gain']:.2f}")
    if args.json:
        Path(args.json).write_text(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
