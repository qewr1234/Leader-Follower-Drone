#!/usr/bin/env python3
"""
identify_plant.py — FC 속도루프를 1차 + 지연 (FOPDT) 로 식별: v(s) = K e^{-sL} / (τ s + 1) · u(s)

입력: analysis/id_flight.py 로그(--log) 또는 main.py 로그(--main-log; 명령 control.body_v*, 속도 LOCAL_POSITION_NED 를 yaw 로 회전).
방법: 시뮬레이션 오차 최소화 — (τ, L) 격자마다 K=1 로 시뮬레이션한 응답에 대한 최소제곱 K 를 닫힌 형태로 구하고 RMSE 최소를
고른 뒤 좌표 하강으로 정밀화. 적합도는 NRMSE(%) = 100·(1 − ‖y−ŷ‖/‖y−ȳ‖) (MATLAB compare 와 같은 정의).
--margins 를 주면 식별값으로 docs/STABILITY_MARGINS.md 의 외루프 여유(PM·GM·Ms·|Γ| 피크)를 다시 계산한다
(τ_fc ← τ, 지연 Td ← 0.10 + L 로 보수적으로).

    python3 analysis/identify_plant.py --log logs/id_forward_*.jsonl --margins
    python3 analysis/identify_plant.py --selftest
"""

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from analysis.logtools import Log, follower_vel_fru  # noqa: E402

AXES = ("forward", "right", "up")


def simulate_fopdt(t, u, K, tau, L, y0=0.0):
    """가변 dt 에서 정확한 1차 이산화 + 입력 지연(선형 보간)."""
    t = np.asarray(t, dtype=float); u = np.asarray(u, dtype=float)
    ud = np.interp(t - L, t, u, left=u[0])
    y = np.empty_like(u)
    y[0] = y0
    for k in range(1, len(t)):
        dt = max(t[k] - t[k - 1], 1e-6)
        a = 1.0 - math.exp(-dt / max(tau, 1e-6))
        y[k] = y[k - 1] + a * (K * ud[k - 1] - y[k - 1])
    return y


def _rmse_for(t, u, y, tau, L):
    y1 = simulate_fopdt(t, u, 1.0, tau, L, y0=0.0)
    den = float(y1 @ y1)
    K = float(y1 @ y) / den if den > 1e-12 else 0.0
    r = y - K * y1
    return math.sqrt(float(r @ r) / len(y)), K


def fit_fopdt(t, u, y, tau_range=(0.05, 2.0), L_range=(0.0, 0.6)):
    """반환 dict(K, tau, L, rmse, fit_pct, n)."""
    t = np.asarray(t, dtype=float); u = np.asarray(u, dtype=float); y = np.asarray(y, dtype=float)
    ok = np.isfinite(t) & np.isfinite(u) & np.isfinite(y)
    t, u, y = t[ok], u[ok], y[ok]
    if t.size < 20:
        raise ValueError("표본 부족")
    t = t - t[0]
    y = y - y[0]                      # 초기 속도 오프셋 제거 (호버 잔류 속도)
    taus = np.geomspace(tau_range[0], tau_range[1], 40)
    Ls = np.arange(L_range[0], L_range[1] + 1e-9, 0.02)
    best = (float("inf"), None, None, None)
    for tau in taus:
        for L in Ls:
            e, K = _rmse_for(t, u, y, tau, L)
            if e < best[0]:
                best = (e, K, float(tau), float(L))
    e, K, tau, L = best
    # 좌표 하강 정밀화
    for _ in range(3):
        for dtau in (0.9, 0.95, 1.05, 1.1):
            e2, K2 = _rmse_for(t, u, y, tau * dtau, L)
            if e2 < e:
                e, K, tau = e2, K2, tau * dtau
        for dL in (-0.01, -0.005, 0.005, 0.01):
            if L + dL < 0:
                continue
            e2, K2 = _rmse_for(t, u, y, tau, L + dL)
            if e2 < e:
                e, K, L = e2, K2, L + dL
    yhat = K * simulate_fopdt(t, u, 1.0, tau, L)
    fit = 100.0 * (1.0 - np.linalg.norm(y - yhat) / max(np.linalg.norm(y - y.mean()), 1e-9))
    return {"K": float(K), "tau": float(tau), "L": float(L), "rmse": float(e), "fit_pct": float(fit), "n": int(t.size),
            "t63_s": float(L + tau)}


def load_id_log(path):
    log = Log.load(path)
    t = log.col("t_mono")
    u = np.stack([log.col("cmd_f"), log.col("cmd_r"), log.col("cmd_u")], axis=1)
    y = np.stack([log.col("v_f"), log.col("v_r"), log.col("v_u")], axis=1)
    return t - np.nanmin(t), u, y


def load_main_log(path):
    log = Log.load(path)
    t = log.t
    u = np.stack([log.col("control.body_vx"), log.col("control.body_vy"), -log.col("control.body_vz")], axis=1)
    y = follower_vel_fru(log)
    return t, u, y


def identify(t, u, y, axes=AXES):
    res = {}
    for i, name in enumerate(AXES):
        if name not in axes:
            continue
        if np.nanstd(u[:, i]) < 1e-3:
            res[name] = {"skipped": "명령 변화 없음"}
            continue
        res[name] = fit_fopdt(t, u[:, i], y[:, i])
    return res


def margins_with(tau_fc, L, axis="forward"):
    """식별값으로 외루프 여유 재계산 (analysis/stability_margins.py 모델)."""
    import analysis.stability_margins as sm
    frf = sm.EkfFrf(sm.load_frf())
    gains = {"forward": (sm.main.KP_FORWARD, sm.main.KD_FORWARD), "right": (sm.main.KP_RIGHT, sm.main.KD_RIGHT), "up": (sm.main.KP_UP, sm.main.KD_UP)}
    kp, kd = gains[axis]
    out = {}
    for label, p in (("assumed", sm.Params(kp=kp, kd=kd)), ("identified", sm.Params(kp=kp, kd=kd, tau_fc=float(tau_fc), Td=0.10 + float(L)))):
        r = sm.margins(sm.open_loop(p, sm.W, frf), sm.W)
        r.update(sm.string_stability(p, sm.W, frf))
        out[label] = {k: float(r[k]) for k in ("pm_deg", "gm_db", "Ms", "peak", "w_peak") if k in r}
        out[label].update({"tau_fc": p.tau_fc, "Td": p.Td})
    return out


def selftest():
    rng = np.random.default_rng(1)
    t = np.arange(0, 40.0, 0.1)
    u = np.zeros_like(t)
    for a, b, v in ((4, 8, 0.3), (12, 16, -0.3), (20, 24, 0.3), (28, 32, -0.3)):
        u[(t >= a) & (t < b)] = v
    truth = dict(K=0.97, tau=0.35, L=0.12)
    y = simulate_fopdt(t, u, **truth) + rng.normal(0, 0.02, t.size)
    r = fit_fopdt(t, u, y)
    ok = abs(r["K"] - truth["K"]) < 0.05 and abs(r["tau"] - truth["tau"]) < 0.06 and abs(r["L"] - truth["L"]) < 0.04 and r["fit_pct"] > 85
    print(f"[selftest] truth={truth} fit={ {k: round(r[k], 3) for k in ('K', 'tau', 'L', 'fit_pct')} } {'OK' if ok else 'FAIL'}")
    return ok, r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", action="append", default=[], help="id_flight.py JSONL (여러 개 가능, 축별로 이어 붙임)")
    ap.add_argument("--main-log", action="append", default=[], help="main.py JSONL")
    ap.add_argument("--axes", default="forward,right,up")
    ap.add_argument("--margins", action="store_true", help="식별값으로 외루프 여유 재계산")
    ap.add_argument("--out", default=None, help="결과 JSON 경로 (예: docs/plant_id.json)")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        ok, _ = selftest()
        sys.exit(0 if ok else 1)
    if not args.log and not args.main_log:
        ap.error("--log 또는 --main-log 필요")
    results = {"files": args.log + args.main_log, "axes": {}}
    axes = tuple(a.strip() for a in args.axes.split(","))
    ts, us, ys = [], [], []
    for p in args.log:
        t, u, y = load_id_log(p); ts.append(t + (ts[-1][-1] + 5.0 if ts else 0.0)); us.append(u); ys.append(y)
    for p in args.main_log:
        t, u, y = load_main_log(p); ts.append(t + (ts[-1][-1] + 5.0 if ts else 0.0)); us.append(u); ys.append(y)
    t, u, y = np.concatenate(ts), np.concatenate(us), np.concatenate(ys)
    results["axes"] = identify(t, u, y, axes)
    for name, r in results["axes"].items():
        if "skipped" in r:
            print(f"{name:8s} 건너뜀: {r['skipped']}")
        else:
            print(f"{name:8s} K={r['K']:.3f} tau={r['tau']:.3f} s L={r['L']:.3f} s (63 % 응답 {r['t63_s']:.2f} s) fit={r['fit_pct']:.1f} % n={r['n']}")
    if args.margins and "forward" in results["axes"] and "tau" in results["axes"]["forward"]:
        f = results["axes"]["forward"]
        results["margins"] = margins_with(f["tau"], f["L"], "forward")
        for label, r in results["margins"].items():
            print(f"여유[{label}] tau_fc={r['tau_fc']:.2f} Td={r['Td']:.2f}: PM={r.get('pm_deg', float('nan')):.0f}° GM={r.get('gm_db', float('nan')):.1f} dB "
                  f"Ms={r.get('Ms', float('nan')):.2f} |Γ|peak={r.get('peak', float('nan')):.2f}")
    if args.out:
        Path(args.out).write_text(json.dumps(results, indent=1, ensure_ascii=False))
        print(f"저장: {args.out}")


if __name__ == "__main__":
    main()
