#!/usr/bin/env python3
"""
nees_nis.py — IMM-EKF 일관성: NIS(혁신 정규화 제곱) 와 UWB 거리 GT 로 잰 1차원 NEES

NIS: 로그의 reliability.gate_d2(RGB-D 3차원 / bearing 2차원) 와 leader_esp32.esp_gate_d2(3차원). 받아들인 갱신의 d² 평균이
차원(dim) 근처여야 한다. N 개 평균의 95 % 구간은 [χ²(0.025; N·dim)/N, χ²(0.975; N·dim)/N]. 거부율도 같이 낸다.
NEES(거리): r̂ = |x̂[:3]| (ekf.x), σ_r̂² = uᵀ P_pos u (ekf.P_pos, u = x̂/|x̂|), r_gt = uwb.range_center_m.
  ε = (r̂ − r_gt)² / (σ_r̂² + σ_uwb²), 기대 1. 잔차 r̂ − r_gt 의 바이어스·표준편차가 논문의 거리 정확도다.
분석 창 밖(초기화 직후·코스팅) 을 빼려면 --t0/--t1, --coast-max 로 거른다.

    python3 analysis/nees_nis.py --log logs/mars_imm_*.jsonl --uwb-sigma 0.10 --t0 5
    python3 analysis/nees_nis.py --selftest
"""

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from analysis.logtools import Log, chi2_ppf  # noqa: E402

NIS_SOURCES = {
    "rgbd": ("reliability.gate_d2", "reliability.update_used", {"rgbd"}, {"gate_reject_rgbd", "gate_reject_all"}, 3),
    "bearing": ("reliability.gate_d2", "reliability.update_used", {"bearing", "bearing_after_rgbd_gate"}, {"gate_reject_bearing"}, 2),
    "esp_gps": ("leader_esp32.esp_gate_d2", "leader_esp32.esp_update_used", {"esp_gps"}, {"gate_reject_esp_gps"}, 3),
}


def nis_summary(d2, dim):
    d2 = np.asarray(d2, dtype=float)
    d2 = d2[np.isfinite(d2)]
    n = int(d2.size)
    if n == 0:
        return {"n": 0}
    mean = float(d2.mean())
    lo, hi = chi2_ppf(0.025, n * dim) / n, chi2_ppf(0.975, n * dim) / n
    s_lo, s_hi = chi2_ppf(0.025, dim), chi2_ppf(0.975, dim)
    return {"n": n, "dim": dim, "mean": mean, "expected": float(dim), "mean_ci95": [float(lo), float(hi)],
            "consistent": bool(lo <= mean <= hi), "frac_in_single95": float(np.mean((d2 >= s_lo) & (d2 <= s_hi))),
            "ratio": mean / dim}


def nis_all(log, mask):
    out = {}
    for name, (d2_key, used_key, accept, reject, dim) in NIS_SOURCES.items():
        d2 = log.col(d2_key)
        used = np.array(log.col_str(used_key))
        acc = np.isin(used, list(accept)) & mask
        rej = np.isin(used, list(reject)) & mask
        s = nis_summary(d2[acc], dim)
        s["rejected"] = int(rej.sum())
        s["reject_rate"] = float(rej.sum() / max(acc.sum() + rej.sum(), 1))
        out[name] = s
    return out


def range_nees(log, mask, uwb_sigma=0.10, uwb_max_age=0.3):
    avail = log.col_bool("uwb.available") & mask & (log.col("uwb.age") <= uwb_max_age)
    x = log.vec("ekf.x", 6)
    P6 = log.vec("ekf.P_pos", 6)
    r_gt = log.col("uwb.range_center_m")
    eps, resid, sig = [], [], []
    for i in np.flatnonzero(avail):
        p = x[i, :3]
        n = float(np.linalg.norm(p))
        if not (np.all(np.isfinite(p)) and n > 1e-6 and np.all(np.isfinite(P6[i]))):
            continue
        u = p / n
        P = np.array([[P6[i, 0], P6[i, 1], P6[i, 2]], [P6[i, 1], P6[i, 3], P6[i, 4]], [P6[i, 2], P6[i, 4], P6[i, 5]]])
        s2 = float(u @ P @ u) + uwb_sigma ** 2
        e = n - float(r_gt[i])
        eps.append(e * e / max(s2, 1e-9)); resid.append(e); sig.append(math.sqrt(s2))
    if not eps:
        return {"n": 0}
    eps = np.asarray(eps); resid = np.asarray(resid)
    n = int(eps.size)
    lo, hi = chi2_ppf(0.025, n) / n, chi2_ppf(0.975, n) / n
    return {"n": n, "mean": float(eps.mean()), "expected": 1.0, "mean_ci95": [float(lo), float(hi)],
            "consistent": bool(lo <= eps.mean() <= hi), "resid_bias_m": float(resid.mean()), "resid_std_m": float(resid.std()),
            "resid_rms_m": float(np.sqrt(np.mean(resid ** 2))), "sigma_pred_mean_m": float(np.mean(sig)),
            "frac_in_single95": float(np.mean(eps <= chi2_ppf(0.95, 1)))}


def analyze(log, t0=None, t1=None, coast_max=0.5, uwb_sigma=0.10):
    t = log.t
    mask = np.ones(log.n, dtype=bool)
    if t0 is not None:
        mask &= t >= t0
    if t1 is not None:
        mask &= t <= t1
    coast = log.col("ekf.coast_time")
    mask &= ~(np.isfinite(coast) & (coast > coast_max))
    return {"window": [float(t[mask].min()) if mask.any() else None, float(t[mask].max()) if mask.any() else None],
            "nis": nis_all(log, mask), "range_nees": range_nees(log, mask, uwb_sigma=uwb_sigma)}


def selftest():
    rng = np.random.default_rng(5)
    rows_ok, rows_bad = [], []
    for i in range(1500):
        # 일관: d² ~ χ²(3); 거리 r̂ − r_gt ~ N(0, σ) 이고 P 가 σ² 을 맞게 말함
        d2 = float(np.sum(rng.normal(size=3) ** 2))
        sig = 0.08
        e_ok = rng.normal(0, math.sqrt(sig ** 2 + 0.1 ** 2))
        e_bad = rng.normal(0, 3 * math.sqrt(sig ** 2 + 0.1 ** 2))
        base = {"t_mono": i / 30, "reliability.gate_d2": d2, "reliability.update_used": "rgbd", "ekf.coast_time": 0.0,
                "uwb.available": True, "uwb.age": 0.05, "ekf.P_pos": f"{sig**2};0;0;{sig**2};0;{sig**2}"}
        rows_ok.append(dict(base, **{"ekf.x": f"0;0;{3.0 + e_ok};0;0;0", "uwb.range_center_m": 3.0}))
        rows_bad.append(dict(base, **{"ekf.x": f"0;0;{3.0 + e_bad};0;0;0", "uwb.range_center_m": 3.0, "reliability.gate_d2": 3 * d2}))
    a, b = analyze(Log(rows_ok)), analyze(Log(rows_bad))
    ok = a["nis"]["rgbd"]["consistent"] and a["range_nees"]["consistent"] and not b["nis"]["rgbd"]["consistent"] and not b["range_nees"]["consistent"]
    print(f"[selftest] 일관 데이터: NIS mean={a['nis']['rgbd']['mean']:.2f} (기대 3) NEES mean={a['range_nees']['mean']:.2f} (기대 1) → "
          f"{a['nis']['rgbd']['consistent']}/{a['range_nees']['consistent']}; 과신 데이터: NIS {b['nis']['rgbd']['mean']:.2f} NEES {b['range_nees']['mean']:.2f} → "
          f"{b['nis']['rgbd']['consistent']}/{b['range_nees']['consistent']}  {'OK' if ok else 'FAIL'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log")
    ap.add_argument("--t0", type=float, default=None); ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--coast-max", type=float, default=0.5, help="coast_time 이 이보다 크면 제외")
    ap.add_argument("--uwb-sigma", type=float, default=0.10, help="UWB 거리 σ [m] (정적 교정에서)")
    ap.add_argument("--json", default=None)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(0 if selftest() else 1)
    if not args.log:
        ap.error("--log 필요")
    res = analyze(Log.load(args.log), args.t0, args.t1, args.coast_max, args.uwb_sigma)
    print(f"창 {res['window']}")
    for name, s in res["nis"].items():
        if s.get("n", 0) == 0:
            print(f"NIS {name:8s} 갱신 없음"); continue
        print(f"NIS {name:8s} n={s['n']:5d} mean={s['mean']:.2f} (기대 {s['expected']:.0f}, 95 % CI {s['mean_ci95'][0]:.2f}~{s['mean_ci95'][1]:.2f}) "
              f"{'일관' if s['consistent'] else '불일치'}  단일표본 95 % 안 {100 * s['frac_in_single95']:.0f} %  거부율 {100 * s['reject_rate']:.1f} %")
    r = res["range_nees"]
    if r.get("n", 0):
        print(f"거리 NEES n={r['n']} mean={r['mean']:.2f} (기대 1, CI {r['mean_ci95'][0]:.2f}~{r['mean_ci95'][1]:.2f}) {'일관' if r['consistent'] else '불일치'} | "
              f"잔차 bias {r['resid_bias_m']:+.3f} m σ {r['resid_std_m']:.3f} m RMS {r['resid_rms_m']:.3f} m | 예측 σ 평균 {r['sigma_pred_mean_m']:.3f} m")
    else:
        print("거리 NEES: UWB 행 없음")
    if args.json:
        Path(args.json).write_text(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
