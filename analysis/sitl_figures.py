"""
analysis/sitl_figures.py — sitl/harness.py 가 남긴 CSV(sitl/results/<run>/) 로 실측 그림을 만든다

    python3 analysis/sitl_figures.py sitl/results/20260919-101500_current
    python3 analysis/sitl_figures.py sitl/results/20260919-101500_current --before sitl/results/20260919-103000_mars_before

  docs/images/sitl_regression.png   ArduCopter 시나리오 8개의 실측 시계열 (거리 / 고도+FC 모드 / 기수) + PASS/FAIL
  docs/images/sitl_leader_sine.png  leader_sine: 리더 속도 vs 팔로워 속도, 진폭비 (--before 가 있으면 수정 전과 나란히)

CSV 형식은 harness.write_csv 참고 ('#' 메타 줄 + 헤더 + 10Hz 행). 색은 dataviz 기준 팔레트.
"""

import argparse
import csv
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMG_DIR = os.path.join(_ROOT, "docs", "images")
C = dict(blue="#2a78d6", orange="#eb6834", aqua="#1baf7a", yellow="#eda100", red="#e34948",
         gray="#9a9a96", surface="#fcfcfb", ink="#0b0b0b", ink2="#52514e", grid="#e6e6e3")
MODE_TINT = {"LOITER": (C["yellow"], 0.18), "LAND": (C["red"], 0.12), "ALT_HOLD": (C["yellow"], 0.18), "RTL": (C["red"], 0.12)}
ORDER = ["boot_no_leader", "pilot_takeover", "air_landing", "depth_range", "hover_hold", "hold_heading", "depth_loss", "handover"]

plt.rcParams.update({"font.size": 9, "axes.edgecolor": C["grid"], "axes.labelcolor": C["ink2"], "xtick.color": C["ink2"],
                     "ytick.color": C["ink2"], "text.color": C["ink"], "axes.titlecolor": C["ink"], "figure.facecolor": C["surface"],
                     "axes.facecolor": C["surface"], "savefig.facecolor": C["surface"], "legend.frameon": False})


def load_run(run_dir):
    """{scenario: {"meta": {...}, "rows": {col: np.array}}} + summary {scenario: (result, note, why)}"""
    out = {}
    for path in sorted(glob.glob(os.path.join(run_dir, "*.csv"))):
        name = os.path.splitext(os.path.basename(path))[0]
        if name == "summary":
            continue
        meta, lines = {}, []
        with open(path, encoding="utf-8") as f:
            for line in f:
                if line.startswith("#"):
                    for tok in line[1:].strip().split(" "):
                        if "=" in tok:
                            k, v = tok.split("=", 1); meta[k] = v
                else:
                    lines.append(line)
        rd = list(csv.DictReader(lines))
        cols = {}
        if rd:
            for k in rd[0].keys():
                vals = [r[k] for r in rd]
                if k == "fc_mode":
                    cols[k] = np.array(vals, dtype=object)
                else:
                    cols[k] = np.array([float(v) if v not in ("", None) else np.nan for v in vals])
        out[name] = {"meta": meta, "rows": cols}
    summary = {}
    sp = os.path.join(run_dir, "summary.csv")
    if os.path.exists(sp):
        with open(sp, encoding="utf-8") as f:
            for r in csv.DictReader(f):
                summary[r["scenario"]] = (r["result"], r.get("note", ""), r.get("why", ""))
    return out, summary


def _style(ax):
    ax.grid(True, color=C["grid"], lw=0.7)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)


def _mode_shading(ax, t, modes, ymax_text):
    """FC 모드 구간 배경 + 전환 표시. GUIDED 는 색 없음."""
    if len(t) == 0:
        return
    change = [0] + [i for i in range(1, len(modes)) if modes[i] != modes[i - 1]] + [len(modes)]
    for a, b in zip(change[:-1], change[1:]):
        m = str(modes[a])
        if m in MODE_TINT:
            col, al = MODE_TINT[m]
            ax.axvspan(t[a], t[min(b, len(t) - 1)], color=col, alpha=al, lw=0)
    for j, i in enumerate(change[1:-1]):
        ax.axvline(t[i], color=C["ink2"], lw=0.8, ls=(0, (3, 2)))
        ax.text(t[i] + 0.3, ymax_text - (0.0 if j % 2 == 0 else 3.4), f"{modes[i]}\n{t[i]:.0f} s",
                fontsize=7.5, color=C["ink2"], va="top")


def fig_regression(run, summary, out_name="sitl_regression.png"):
    fig, axes = plt.subplots(2, 4, figsize=(13, 6.2), dpi=150)
    for ax, name in zip(axes.flat, ORDER):
        _style(ax)
        res = summary.get(name, ("", "", ""))
        tag = f"{res[0]}" if res[0] else "no data"
        ax.set_title(f"{name}  [{tag}]", loc="left", fontsize=9.5, color=C["ink"] if res[0] != "FAIL" else C["red"])
        if name not in run or len(run[name]["rows"].get("t_s", [])) == 0:
            ax.text(0.5, 0.5, "no CSV", transform=ax.transAxes, ha="center", color=C["ink2"]); continue
        r = run[name]["rows"]; t = r["t_s"]
        if name in ("depth_range", "hover_hold"):
            target = float(run[name]["meta"].get("target", 3.0))
            ax.plot(t, r["front_m"], color=C["blue"], lw=1.8)
            ax.axhline(target, color=C["ink2"], lw=0.9, ls=(0, (4, 3)))
            ax.text(t[0] + 0.5, target + 0.08, f"target {target:.1f} m", fontsize=7.5, color=C["ink2"])
            late = r["front_m"][t > t[-1] * 0.6]
            if late.size:
                ax.text(t[-1], float(np.nanmean(late)) + 0.12, f"late mean {np.nanmean(late):.1f} m", ha="right", fontsize=7.5)
            if name == "depth_range":
                # 정상상태 오차 = (v − KFF·(v − DB))/Kp. v 0.3, KFF 0.8, DB 0.05, Kp 0.22 → 0.45 m
                ss = target + 0.45
                ax.axhline(ss, color=C["aqua"], lw=1.0, ls=(0, (2, 2)))
                ax.text(t[0] + 0.5, ss + 0.08, f"theory {ss:.2f} m", fontsize=7.5, color=C["aqua"])
                tail = r["front_m"][t > t[-1] - 3.0]
                if tail.size and float(np.nanmean(tail)) - ss > 0.15:
                    ax.text(t[0] + 0.5, float(np.nanmin(r["front_m"])) - 0.45,
                            f"still converging at end of run ({np.nanmean(tail):.2f} m at {t[-1]:.0f} s)",
                            fontsize=7.5, color=C["ink2"])
            ax.set_ylabel("distance to leader [m]")
            ax.set_ylim(min(2.0, float(np.nanmin(r["front_m"])) - 0.3), float(np.nanmax(r["front_m"])) + 0.8)
        elif name == "hold_heading":
            h = r["heading_deg"]; ok = ~np.isnan(h)
            if ok.any():
                dev = ((h[ok] - h[ok][0] + 180) % 360) - 180
                ax.plot(t[ok], dev, color=C["blue"], lw=1.8)
                ax.text(t[-1], float(np.max(np.abs(dev))) + 1, f"max |drift| {np.max(np.abs(dev)):.1f}°", ha="right", fontsize=7.5)
            ax.axhline(0, color=C["ink2"], lw=0.9, ls=(0, (4, 3)))
            ax.set_ylabel("heading drift [deg]"); ax.set_ylim(-30, 30)
        else:
            agl = r["agl_m"]
            ax.plot(t, agl, color=C["blue"], lw=1.8)
            ymax = float(np.nanmax(agl)) + 3.0
            ax.set_ylim(0, ymax + 1.5)
            _mode_shading(ax, t, r["fc_mode"], ymax + 1.2)
            ax.set_ylabel("follower altitude AGL [m]")
            if name in ("pilot_takeover", "handover") and float(np.nanmin(agl)) < 1.0:
                ax.text(t[-1], 2.4, "descends under SITL LOITER\n(no RC throttle in the harness)", ha="right", fontsize=7.5,
                        color=C["ink2"], va="bottom")
            if name == "air_landing":
                ax.plot(t, agl + (r["leader_d"] - r["fol_d"]) * -1.0, color=C["orange"], lw=1.4)   # 리더 고도 = 팔로워 AGL + (up)
                ax.text(t[-1], float(np.nanmin(agl + (r["leader_d"] - r["fol_d"]) * -1.0)) - 1.0, "leader (descends)", ha="right",
                        fontsize=7.5, color=C["ink2"])
        if res[1]:
            ax.text(0.01, 0.02, res[1], transform=ax.transAxes, fontsize=7.5, color=C["ink2"], va="bottom")
        ax.set_xlabel("t [s]")
    meta = next(iter(run.values()))["meta"] if run else {}
    fig.suptitle(f"ArduCopter SITL regression — run {meta.get('run', '?')} ({meta.get('tag', '?')}), 10 Hz LOCAL_POSITION_NED. "
                 "Yellow = pilot LOITER, red = LAND", x=0.01, ha="left", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out = os.path.join(IMG_DIR, out_name); fig.savefig(out); plt.close(fig)
    return out


def _sine_panel(ax, entry, label):
    r = entry["rows"]; m = entry["meta"]; t = r["t_s"]
    w = float(m.get("sine_w", 1.15)); amp = float(m.get("sine_amp", 0.05)); t0 = float(m.get("sine_t0", 0.0) or 0.0)
    settle = float(m.get("sine_settle", 20.0))
    # 리더 속도: leader_n 의 미분 (하네스는 0.05s 마다 적분하므로 10Hz 샘플에서는 계단이 조금 남는다 → 0.5s 이동평균)
    vl = np.gradient(r["leader_n"], t)
    k = max(1, int(round(0.5 / max(np.median(np.diff(t)), 1e-3))))
    vl_s = np.convolve(vl, np.ones(k) / k, mode="same")
    vl_s[: k // 2 + 1] = np.nan; vl_s[-(k // 2 + 1):] = np.nan          # 이동평균 가장자리 제거
    ax.plot(t, vl_s, color=C["orange"], lw=1.6, label="leader velocity (north)")
    ax.plot(t, r["fol_vn_mps"], color=C["blue"], lw=1.8, label="follower velocity (north)")
    tf = t0 + settle
    ax.axvspan(tf, t[-1], color=C["blue"], alpha=0.05, lw=0)
    sel = t >= tf
    ratio = None
    if sel.sum() > 50:
        ts = t[sel] - t0; M = np.column_stack([np.cos(w * ts), np.sin(w * ts), np.ones_like(ts)])
        (a, b, c), *_ = np.linalg.lstsq(M, r["fol_vn_mps"][sel], rcond=None)
        ratio = float(np.hypot(a, b)) / amp
        ax.text(tf + 0.5, -0.03, f"fit window → follower amplitude {np.hypot(a, b):.3f} m/s / leader {amp} = ×{ratio:.2f}",
                fontsize=8.5, color=C["ink"], va="bottom")
    ax.set_ylim(-0.05, 0.6); ax.set_ylabel("velocity [m/s]"); _style(ax)
    ax.set_title(label + (f"  — amplitude ratio ×{ratio:.2f} ({'PASS' if ratio <= 1 else 'FAIL'})" if ratio is not None else ""),
                 loc="left", fontsize=10, color=C["ink"] if (ratio is None or ratio <= 1) else C["red"])
    return ratio


def fig_leader_sine(run, before_run=None, out_name="sitl_leader_sine.png"):
    panels = []
    if before_run and "leader_sine" in before_run:
        panels.append((before_run["leader_sine"], f"before fix ({before_run['leader_sine']['meta'].get('tag', 'before')}): FF LPF 0.7 s, no self-velocity matching"))
    if "leader_sine" in run:
        panels.append((run["leader_sine"], f"current ({run['leader_sine']['meta'].get('tag', 'current')}): FF LPF 2.0 s + self-velocity LPF 0.3 s + soft deadzone"))
    if not panels:
        return None
    fig, axes = plt.subplots(len(panels), 1, figsize=(10, 3.4 * len(panels)), dpi=150, sharex=True, squeeze=False)
    for ax, (entry, label) in zip(axes[:, 0], panels):
        _sine_panel(ax, entry, label)
    axes[-1, 0].set_xlabel("t [s]")
    axes[0, 0].legend(loc="upper right", fontsize=8.5, ncol=2)
    m = panels[-1][0]["meta"]
    fig.suptitle(f"leader_sine in ArduCopter SITL: leader {m.get('sine_mean', '0.25')} ± {m.get('sine_amp', '0.05')} m/s at "
                 f"{m.get('sine_w', '1.15')} rad/s — does the follower amplify it?", x=0.01, ha="left", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out = os.path.join(IMG_DIR, out_name); fig.savefig(out); plt.close(fig)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", help="sitl/results/<run>_<tag>/ (현재 코드 실행)")
    ap.add_argument("--before", help="수정 전 코드 실행 폴더 (leader_sine 대조군)")
    a = ap.parse_args()
    run, summary = load_run(a.run_dir)
    before = load_run(a.before)[0] if a.before else None
    print(fig_regression(run, summary))
    print(fig_leader_sine(run, before))
