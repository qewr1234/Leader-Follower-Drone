"""
analysis/readme_figures.py — README 용 그림 2장 (matplotlib)

    python3 test_closed_loop.py --dump /tmp/cl.json      # 그림 2 의 데이터 (프레임별 거리·상태·setpoint)
    python3 analysis/readme_figures.py --closed-loop /tmp/cl.json

  docs/images/readme_string_stability.png  선형 모델의 리더→팔로워 속도 이득 (수정 전/현재/P+D) + ArduCopter SITL 실측점
  docs/images/readme_closed_loop.png       test_closed_loop 의 40초 폐루프: 리더 거리, 전진 속도 명령, 미션 상태 타임라인

색은 dataviz 기준 팔레트(검증 완료: blue #2a78d6, orange #eb6834, aqua #1baf7a, yellow #eda100, red #e34948).
그림 안 글자는 영어(컨테이너에 한글 폰트 없음), 설명은 README 캡션에 한글로.
"""

import argparse
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
from analysis import stability_margins as sm  # noqa: E402

IMG_DIR = os.path.join(_ROOT, "docs", "images")
C = dict(blue="#2a78d6", orange="#eb6834", aqua="#1baf7a", yellow="#eda100", red="#e34948",
         gray="#9a9a96", surface="#fcfcfb", ink="#0b0b0b", ink2="#52514e", grid="#e6e6e3")
SITL = {"before": [1.95], "current": [0.43, 0.72], "omega": 1.15}      # 2026-09-18 WSL1 ArduCopter 실측 (sitl/README.md)

plt.rcParams.update({"font.size": 10, "axes.edgecolor": C["grid"], "axes.labelcolor": C["ink2"], "xtick.color": C["ink2"],
                     "ytick.color": C["ink2"], "text.color": C["ink"], "axes.titlecolor": C["ink"], "figure.facecolor": C["surface"],
                     "axes.facecolor": C["surface"], "savefig.facecolor": C["surface"], "legend.frameon": False})


def _style(ax):
    ax.grid(True, color=C["grid"], lw=0.8)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)


def fig_string_stability():
    frf = sm.EkfFrf(sm.load_frf())
    base = sm.Params()
    W = np.logspace(-2, np.log10(20.0), 1500)
    curves = [("P+D only (no feedforward)", sm.Params(kff=0.0), C["gray"]),
              ("before fix: FF LPF 0.7 s, no self-velocity matching", base.copy(**sm.BEFORE), C["orange"]),
              ("current: FF LPF 2.0 s + self-velocity LPF 0.3 s + soft deadzone", base, C["blue"])]
    fig, ax = plt.subplots(figsize=(9.2, 5.4), dpi=150)
    peaks = {}
    for label, p, col in curves:
        g = np.abs(sm.leader_to_follower(p, W, frf))
        ax.plot(W, g, color=col, lw=2.0, label=label, solid_capstyle="round")
        i = int(np.argmax(g)); peaks[label] = (W[i], g[i])
    ax.axhline(1.0, color=C["ink2"], lw=1.0, ls=(0, (4, 3)))
    ax.text(0.0115, 1.03, "string-stability limit |Γ| = 1", color=C["ink2"], fontsize=9, va="bottom")
    ax.axvline(SITL["omega"], color=C["grid"], lw=1.2)
    ax.text(SITL["omega"] * 1.06, 2.33, "SITL test: ω = 1.15 rad/s (T = 5.5 s)", color=C["ink2"], fontsize=8.5, va="top")
    # SITL 실측점 (흰 테두리 2px)
    for key, col in (("before", C["orange"]), ("current", C["blue"])):
        for v in SITL[key]:
            ax.plot(SITL["omega"], v, "o", ms=9, mfc=col, mec="white", mew=2, zorder=5)
    ax.annotate("ArduCopter SITL, before fix: ×1.95 (FAIL)", (SITL["omega"], 1.95), xytext=(1.6, 1.95), fontsize=9,
                color=C["ink"], va="center", arrowprops=dict(arrowstyle="-", color=C["ink2"], lw=0.8))
    ax.annotate("SITL, current code: ×0.72 / ×0.43 (PASS)", (SITL["omega"], 0.72), xytext=(1.6, 0.62), fontsize=9,
                color=C["ink"], va="center", arrowprops=dict(arrowstyle="-", color=C["ink2"], lw=0.8))
    # 곡선 직접 라벨 (피크 근처)
    wb, gb = peaks[curves[1][0]]
    ax.annotate(f"before fix: peak {gb:.2f} at {wb:.2f} rad/s\n(gain margin 4.9 dB)", (wb, gb), xytext=(0.12, 1.58), fontsize=9,
                color=C["ink"], arrowprops=dict(arrowstyle="-", color=C["ink2"], lw=0.8))
    wc, gc = peaks[curves[2][0]]
    ax.annotate(f"current: peak {gc:.2f} at {wc:.2f} rad/s\n(gain margin 14.4 dB)", (wc, gc), xytext=(0.028, 1.32), fontsize=9,
                color=C["ink"], arrowprops=dict(arrowstyle="-", color=C["ink2"], lw=0.8))
    ax.text(0.0115, 0.86, "P+D only: no amplification,\nbut 1.36 m lag at 0.3 m/s", color=C["ink2"], fontsize=8.5, va="top")
    ax.set_xscale("log"); ax.set_xlim(0.01, 20); ax.set_ylim(0, 2.36)
    ax.set_xlabel("ω [rad/s]"); ax.set_ylabel("|Γ(jω)| = |v_follower / v_leader|")
    ax.set_title("Leader→follower velocity gain: linear model vs ArduCopter SITL (2026-09-18)", loc="left", fontsize=11)
    ax.legend(loc="upper left", fontsize=8.5, bbox_to_anchor=(0.0, -0.13), ncol=1)
    _style(ax)
    fig.tight_layout()
    out = os.path.join(IMG_DIR, "readme_string_stability.png"); fig.savefig(out, bbox_inches="tight"); plt.close(fig)
    return out


STATE_COLOR = {"READY_HOVER": C["yellow"], "FOLLOW": C["blue"], "LEADER_HOVER": C["aqua"], "LOST_HOLD": C["orange"],
               "FAILSAFE_LAND": C["red"], "WAIT_LEADER": C["gray"], "LANDING_CANDIDATE": C["yellow"], "CONFIRMED_LANDING": C["red"]}


def fig_closed_loop(dump_path):
    d = json.load(open(dump_path))
    t = np.array([r[0] for r in d["dist"]]); dist = np.array([r[1] for r in d["dist"]]); vis = np.array([r[2] for r in d["dist"]], bool)
    sp = np.array([[r[1], r[2]] for r in d["setpoints"]])       # (t, vx)
    states = d["states"]; T = t[-1]
    land_t = [c[1] for c in d["mode_calls"] if c[2] == "LAND"]
    fig, axes = plt.subplots(3, 1, figsize=(9.2, 6.6), dpi=150, sharex=True, gridspec_kw={"height_ratios": [3.0, 1.7, 0.9], "hspace": 0.12})
    a0, a1, a2 = axes
    # 배경 구간: 리더 이동(3~11s) / 리더 안 보임
    for ax in (a0, a1):
        ax.axvspan(3.0, 11.0, color=C["blue"], alpha=0.06, lw=0)
        inv = np.where(~vis)[0]
        if inv.size:
            edges = np.split(inv, np.where(np.diff(inv) > 1)[0] + 1)
            for e in edges:
                ax.axvspan(t[e[0]], t[e[-1]], color=C["gray"], alpha=0.16, lw=0)
    a0.text(7.0, 5.55, "leader moves 0.3 m/s", ha="center", color=C["ink2"], fontsize=8.5)
    a0.text(18.0, 5.55, "leader hidden 4 s", ha="center", color=C["ink2"], fontsize=8.5)
    a0.text(29.5, 5.55, "leader lost for good", ha="center", color=C["ink2"], fontsize=8.5)
    # 거리
    a0.plot(t, dist, color=C["blue"], lw=2.0, label="distance to leader")
    a0.axhline(3.0, color=C["ink2"], lw=1.0, ls=(0, (4, 3)))
    a0.text(0.4, 3.08, "target 3.0 m", color=C["ink2"], fontsize=8.5, va="bottom")
    i_pk = int(np.argmax(dist[(t > 3) & (t < 12)])) + int(np.searchsorted(t, 3))
    a0.annotate(f"peak lag {dist[i_pk]:.2f} m (P+D+FF, leader 0.3 m/s)", (t[i_pk], dist[i_pk]), xytext=(t[i_pk] + 1.5, dist[i_pk] + 0.9),
                fontsize=8.5, arrowprops=dict(arrowstyle="-", color=C["ink2"], lw=0.8))
    a0.set_ylim(2.0, 6.0); a0.set_ylabel("distance [m]"); _style(a0)
    a0.set_title("Closed-loop run of the real main loop — fake camera & fake FC, 30 fps, 40 s (test_closed_loop.py)", loc="left", fontsize=11)
    # 속도 명령
    a1.plot(sp[:, 0], sp[:, 1], color=C["orange"], lw=2.0, label="forward velocity setpoint (BODY_NED vx)")
    a1.axhline(0.35, color=C["grid"], lw=1.0); a1.text(0.4, 0.355, "MAX_VX 0.35 m/s", color=C["ink2"], fontsize=8, va="bottom")
    a1.set_ylim(-0.1, 0.45); a1.set_ylabel("vx cmd [m/s]"); _style(a1)
    a1.text(0.4, -0.08, "setpoints stop when the mission says HOLD or LAND", color=C["ink2"], fontsize=8, va="bottom")
    # 미션 상태 스트립
    segs = [(states[i][1], states[i + 1][1] if i + 1 < len(states) else T, states[i][2]) for i in range(len(states))]
    for s0, s1, st in segs:
        a2.barh(0, s1 - s0, left=s0, height=0.8, color=STATE_COLOR.get(st, C["gray"]), edgecolor="white", lw=2)
        if s1 - s0 >= 1.6:
            a2.text((s0 + s1) / 2, 0, st.replace("_", "\n") if s1 - s0 < 5 else st, ha="center", va="center", fontsize=7.5, color="white" if st != "READY_HOVER" else C["ink"])
    a2.set_yticks([]); a2.set_ylabel("mission", rotation=0, ha="right", va="center"); a2.set_xlim(0, T); a2.set_xlabel("t [s]")
    for sp_ in ("top", "right", "left"):
        a2.spines[sp_].set_visible(False)
    # LAND 시각
    for lt in land_t:
        for ax in axes:
            ax.axvline(lt, color=C["red"], lw=1.2, ls=(0, (3, 2)))
        a0.text(lt + 0.3, 4.9, f"LAND sent\nat {lt:.0f} s\n(10 s after\nloss)", ha="left", va="top", color=C["red"], fontsize=8.5)
    fig.align_ylabels(axes)
    out = os.path.join(IMG_DIR, "readme_closed_loop.png"); fig.savefig(out, bbox_inches="tight"); plt.close(fig)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--closed-loop", help="test_closed_loop.py --dump 결과 JSON (없으면 그림 2 생략)")
    a = ap.parse_args()
    print(fig_string_stability())
    if a.closed_loop:
        print(fig_closed_loop(a.closed_loop))
