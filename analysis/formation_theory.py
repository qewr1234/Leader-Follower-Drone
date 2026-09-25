"""
analysis/formation_theory.py — 편대 이론 (이론 우선 트랙)

    python3 analysis/formation_theory.py            # 표 출력 + docs/formation_theory.json

analysis/stability_margins.py 의 외루프 선형 모델(실측 IMM-EKF FRF 포함) 위에 세 가지를 정리 형태로 세우고 실제 코드
체인 시뮬(chain_sim)로 수치 검증한다. 문서: docs/FORMATION_THEORY.md.

  정리 1 (선두 방송 토폴로지의 스트링 안정성)
      후미 i 의 명령 u_i = B_d·v_L + A·(v_{i-1} − v_i),  v_i = P·u_i          (B_d = KFF·H_ff·e^{-sTm}: 방송 지연 Tm)
      ⇒ v_i = T·v_{i-1} + G·v_L,   T = PA/(1+PA),  G = P·B_d/(1+PA)
      ⇒ Γ_i := v_i / v_L = P·B_d + T^i·(1 − P·B_d)
      따라서 |Γ_i| ≤ |P B_d| + |T|^i·|1 − P B_d|. ‖T‖∞ ≤ 1 이면(P+D 루프, 실측 0.999) 리더→i 단 이득은 i 에 대해 균일 유계이고
      꼬리는 피드포워드 단독 응답 P·B_d (‖·‖∞ ≤ KFF) 로 수렴한다. 선행기 간 교란 전달은 T 이므로 ‖T‖∞ ≤ 1 이 곧
      Ploeg(2014) 의미의 L2 스트링 안정성이다.  대조: 선행기 추종(vision FF) 은 Γ_i = Γ_1^i 로 ‖Γ_1‖∞ > 1 이면 기하급수 발산.

  정리 2 (시간간격 정책 — 방송이 없을 때의 자율 대안)
      목표 간격 D = D0 + h·v_F 로 두면 자기 속도 항 −Kp·h·H_m·v_F 가 루프에 더해져
      Γ_h = P(A + BC) / (1 + P(F + Kp·h·H_m)),  F = A − B(H_m − C).   ‖Γ_h‖∞ ≤ 1 이 되는 최소 h 를 이분법으로 구한다.

  정리 3 (포화·이득 전환에 대한 절대안정성 — 원판 판별법)
      명령 포화 sat(·), 불확실성 감속(0.55/0.75/1), 추종/정지 전환은 모두 명령 채널의 섹터 [δ, 1] 이득이다(δ > 0).
      루프 전달 L = P·F 에 대해 나이퀴스트 선도가 원판 D(δ,1) (실축 −1/δ … −1) 밖에 있으면 절대안정 (Khalil, Nonlinear
      Systems 3판 7.1절 원판 판별법). min_ω Re L(jω) > −1 이면 모든 δ ∈ (0, 1] 에 대해 성립한다.
      소프트 데드존 φ(v) = v − sat_DB(v) 의 비선형 부분 sat_DB 는 |·| ≤ DB 로 유계라 되먹임 비선형이 아니라 유계 외란으로
      다루며, 정상상태 기여는 KFF·DB/Kp (0.18 m).

가정·한계: SISO 축별 모델(측면·기수 결합 제외), FC 1차 지연 τ_fc 는 가정(0.3 s), 격자 위 수치 평가(해석적 증명이 아닌 수치
인증서), 방송 지연은 Tm 로 고정, 후미 간 충돌 회피 미포함.
"""

import json
import math
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from analysis import stability_margins as sm  # noqa: E402  (main 스텁 임포트 포함)

JSON_PATH = os.path.join(_ROOT, "docs", "formation_theory.json")
W = sm.W


# ---------------------------------------------------------------- 정리 1: 체인 전달함수
def predecessor_disturbance_transfer(p, w, frf):
    """T = PA/(1+PA): 선행기 속도 → 후미 속도 (선두 방송 토폴로지에서 인접 단 교란 전달)."""
    P, A, B, C, Hm = sm._pieces(p, w, frf)
    return P * A / (1.0 + P * A)


def broadcast_tail(p, w, frf):
    """P·B_d: 방송 피드포워드 단독 응답 (i → ∞ 극한)."""
    P, A, B, C, Hm = sm._pieces(p, w, frf)
    return P * B * np.exp(-1j * w * p.Tm)


def chain_transfer(p, w, frf, n, topology):
    """리더 속도 → i 단 속도 Γ_i (i = 1..n). topology: 'predecessor' | 'leader_broadcast'."""
    if topology == "predecessor":
        g1 = sm.leader_to_follower(p, w, frf)
        return [g1 ** i for i in range(1, n + 1)]
    if topology == "leader_broadcast":
        T = predecessor_disturbance_transfer(p, w, frf)
        PB = broadcast_tail(p, w, frf)
        return [PB + (T ** i) * (1.0 - PB) for i in range(1, n + 1)]
    raise ValueError(topology)


def chain_peaks(p, w, frf, n, topology):
    return [float(np.max(np.abs(g))) for g in chain_transfer(p, w, frf, n, topology)]


def chain_gains_at(p, w, frf, n, topology, omega):
    k = int(np.argmin(np.abs(w - omega)))
    return [float(abs(g[k])) for g in chain_transfer(p, w, frf, n, topology)]


# ---------------------------------------------------------------- 정리 2: 시간간격 정책
def headway_gamma(p, w, frf, h):
    P, A, B, C, Hm = sm._pieces(p, w, frf)
    F = A - B * (Hm - C)
    return P * (A + B * C) / (1.0 + P * (F + p.kp * float(h) * Hm))


def headway_open_loop(p, w, frf, h):
    P, A, B, C, Hm = sm._pieces(p, w, frf)
    return sm.open_loop(p, w, frf) + P * p.kp * float(h) * Hm


def headway_min(p, w, frf, tol=1e-3, h_max=5.0):
    """‖Γ_h‖∞ ≤ 1 이 되는 최소 h [s] (이분법). 0 이면 이미 스트링 안정, inf 면 h_max 안에 없음."""
    peak = lambda h: float(np.max(np.abs(headway_gamma(p, w, frf, h))))
    if peak(0.0) <= 1.0:
        return 0.0
    if peak(h_max) > 1.0:
        return float("inf")
    lo, hi = 0.0, h_max
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if peak(mid) <= 1.0:
            hi = mid
        else:
            lo = mid
    return hi


# ---------------------------------------------------------------- 정리 3: 원판 판별법 인증서
def circle_certificate(p, w, frf):
    """명령 채널 섹터 [δ,1] 비선형(포화·이득 감속·모드 전환)에 대한 원판 판별법. 조건: min Re L(jω) > −1."""
    L = sm.open_loop(p, w, frf)
    re = np.real(L)
    k = int(np.argmin(re))
    return {"min_re_L": float(re[k]), "w_at": float(w[k]), "disk_margin": float(1.0 + re[k]),
            "certified": bool(re[k] > -1.0)}


def circle_certificate_mimo(p, w, frf, w_min=0.05):
    """(보조) 포화 + 소프트 데드존을 둘 다 섹터 [0,1] 되먹임으로 둔 2×2 원판 조건 λ_min(Z+Z^H), Z = I − G.
    ω→0 에서 적분기 때문에 S→0 이라 λ_min→0 (구조적 퇴화)이므로 ω ≥ w_min 에서만 평가한다. 본문의 인증서는 정리 3 (SISO)."""
    P, A, B, C, Hm = sm._pieces(p, w, frf)
    F = A - B * (Hm - C)
    L = P * F
    G = np.empty((len(w), 2, 2), dtype=complex)
    G[:, 0, 0] = L / (1 + L)
    G[:, 0, 1] = -B / (1 + L)
    G[:, 1, 0] = -(Hm - C) * P / (1 + L)
    G[:, 1, 1] = -(Hm - C) * P * B / (1 + L)
    Z = np.eye(2)[None, :, :] - G
    H = Z + np.conj(np.transpose(Z, (0, 2, 1)))
    lam = np.array([float(np.min(np.linalg.eigvalsh(h))) for h in H])
    m = w >= w_min
    k = int(np.argmin(np.where(m, lam, np.inf)))
    return {"lambda_min": float(lam[k]), "w_at": float(w[k]), "w_min": float(w_min)}


# ---------------------------------------------------------------- 수치 검증 (실제 코드 체인 시뮬)
def validate_chain(frf, omegas=(0.35, 1.15), n=4, amp=0.03, T=60.0, t_from=24.0):
    """선형 Γ_i 예측 대 비선형 체인 시뮬(실제 ImmEkf·제어 함수) 의 리더→i 단 진폭비."""
    p = sm.Params()
    out = []
    for topo in ("predecessor", "leader_broadcast"):
        for om in omegas:
            hist = sm.chain_sim(n_followers=n, v_leader=0.30, T=T, profile="sine", omega=om, amp=amp, topology=topo)
            amps = sm.chain_sine_amplitudes(hist, om, t_from=t_from)["amplitudes"]
            nonlin = [a / amps[0] for a in amps[1:]]
            lin = chain_gains_at(p, W, frf, n, topo, om)
            out.append({"topology": topo, "omega": om, "linear": lin, "nonlinear": nonlin,
                        "max_rel_err": float(max(abs(a - b) / b for a, b in zip(nonlin, lin)))})
    return out


def large_signal(p=None):
    """포화가 상시 활성인 대신호 조건에서 실제 코드가 유계·수렴하는가 (정리 3 의 수치 확인).
    (a) 초기 간격 오차 +4 m 계단 추종: 최대 명령 0.35 로 오래 포화 → 언더슈트 없이 수렴.
    (b) 리더 0.3 ± 0.5 m/s 정현파(명령 포화 상시): 전반부 대 후반부 진폭이 커지지 않음."""
    p = p or sm.Params()
    a = sm.chain_sim(n_followers=1, v_leader=0.3, T=60.0, profile="step", p=p, initial_err=4.0)
    err = a["err"][0]; t = a["t"]
    step = {"initial_err_m": 4.0, "min_spacing_m": float(3.0 + np.min(err)), "final_err_m": float(err[-1]),
            "err_at_25s_m": float(err[int(np.argmin(np.abs(t - 25.0)))])}
    b = sm.chain_sim(n_followers=1, v_leader=0.3, T=80.0, profile="sine", omega=0.6, amp=0.5, p=p)
    e = b["err"][0]; tb = b["t"]
    first = float(np.max(np.abs(e[(tb >= 20) & (tb < 50)]))); second = float(np.max(np.abs(e[tb >= 50])))
    sine = {"amp": 0.5, "omega": 0.6, "max_abs_err_first_half_m": first, "max_abs_err_second_half_m": second,
            "bounded": bool(second <= first * 1.05 + 1e-6)}
    return {"step": step, "sine": sine}


# ---------------------------------------------------------------- 전체
def compute_all(frf, n=6):
    p = sm.Params()
    T = predecessor_disturbance_transfer(p, W, frf)
    PB = broadcast_tail(p, W, frf)
    res = {
        "theorem1": {
            "T_inf_norm": float(np.max(np.abs(T))),
            "PBd_inf_norm": float(np.max(np.abs(PB))),
            "uniform_bound": float(np.max(np.abs(PB) + np.abs(1 - PB))),
            "peaks_predecessor": chain_peaks(p, W, frf, n, "predecessor"),
            "peaks_broadcast": chain_peaks(p, W, frf, n, "leader_broadcast"),
            "at_0.35_predecessor": chain_gains_at(p, W, frf, n, "predecessor", 0.35),
            "at_0.35_broadcast": chain_gains_at(p, W, frf, n, "leader_broadcast", 0.35),
            "peaks_broadcast_kff1.0": chain_peaks(sm.Params(kff=1.0), W, frf, n, "leader_broadcast"),
            "peaks_broadcast_kp0.6": chain_peaks(sm.Params(kp=0.6, kd=0.05 * 0.6 / 0.22), W, frf, n, "leader_broadcast"),
        },
        "theorem2": {},
        "theorem3": {},
        "validation": validate_chain(frf),
        "large_signal": large_signal(),
    }
    for kff in (0.8, 1.0):
        pk = sm.Params(kff=kff)
        h = headway_min(pk, W, frf)
        entry = {"h_min_s": h, "extra_spacing_at_0.3mps_m": h * 0.3}
        if math.isfinite(h):
            r = sm.margins(headway_open_loop(pk, W, frf, h), W)
            entry.update(pm_deg=r["pm_deg"], gm_db=r["gm_db"], Ms=r["Ms"],
                         peak=float(np.max(np.abs(headway_gamma(pk, W, frf, h)))))
        res["theorem2"][f"kff{kff}"] = entry
    for name, pp in (("current", sm.Params()), ("before_2026-09-18", sm.Params(**sm.BEFORE)),
                     ("kp0.4", sm.Params(kp=0.4, kd=0.05 * 0.4 / 0.22)), ("kp0.6", sm.Params(kp=0.6, kd=0.05 * 0.6 / 0.22)),
                     ("kp0.8", sm.Params(kp=0.8, kd=0.05 * 0.8 / 0.22))):
        c = circle_certificate(pp, W, frf)
        c["mimo_supplementary"] = circle_certificate_mimo(pp, W, frf)
        res["theorem3"][name] = c
    return res


def print_tables(res):
    t1 = res["theorem1"]
    print("=== 정리 1: 리더→i 단 |Γ_i| 피크 (i = 1..6) ===")
    print("  선행기 추종 :", ", ".join(f"{v:.3f}" for v in t1["peaks_predecessor"]))
    print("  선두 방송   :", ", ".join(f"{v:.3f}" for v in t1["peaks_broadcast"]),
          f"   (‖T‖∞={t1['T_inf_norm']:.4f}, 꼬리 ‖P·B_d‖∞={t1['PBd_inf_norm']:.3f})")
    print("  선두 방송 KFF 1.0:", ", ".join(f"{v:.3f}" for v in t1["peaks_broadcast_kff1.0"]))
    print("  선두 방송 Kp 0.6 :", ", ".join(f"{v:.3f}" for v in t1["peaks_broadcast_kp0.6"]))
    print("=== 정리 2: 시간간격 정책 최소 h ===")
    for k, v in res["theorem2"].items():
        print(f"  {k}: h_min={v['h_min_s']:.2f} s, 추가 이격 {v['extra_spacing_at_0.3mps_m']:.2f} m @0.3 m/s, "
              + (f"PM {v['pm_deg']:.0f}° GM {v['gm_db']:.1f} dB Ms {v['Ms']:.2f}" if 'pm_deg' in v else "(없음)"))
    print("=== 정리 3: 원판 판별법 (섹터 [δ,1] 명령 비선형) ===")
    for k, v in res["theorem3"].items():
        print(f"  {k:20s}: min Re L = {v['min_re_L']:+.3f} @ {v['w_at']:.2f} rad/s → 여유 {v['disk_margin']:.3f} "
              f"{'인증' if v['certified'] else '실패'} | 보조 2×2 λ_min(ω≥{v['mimo_supplementary']['w_min']}) = {v['mimo_supplementary']['lambda_min']:.3f}")
    print("=== 검증: 선형 Γ_i 대 비선형 체인 시뮬 (리더→i 단 진폭비) ===")
    for v in res["validation"]:
        print(f"  {v['topology']:17s} ω={v['omega']}: 선형 {[round(x, 3) for x in v['linear']]} / 비선형 {[round(x, 3) for x in v['nonlinear']]} "
              f"(최대 상대오차 {100 * v['max_rel_err']:.1f} %)")
    ls = res["large_signal"]
    print("=== 대신호 (포화 상시) ===")
    print(f"  계단 +4 m: 최소 간격 {ls['step']['min_spacing_m']:.2f} m, 25 s 오차 {ls['step']['err_at_25s_m']:+.2f} m, 최종 {ls['step']['final_err_m']:+.2f} m")
    print(f"  정현파 0.3±0.5: 전반 {ls['sine']['max_abs_err_first_half_m']:.2f} m → 후반 {ls['sine']['max_abs_err_second_half_m']:.2f} m, 유계 {ls['sine']['bounded']}")


def main():
    frf = sm.EkfFrf(sm.load_frf())
    res = compute_all(frf)
    print_tables(res)
    with open(JSON_PATH, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print(f"[json] {JSON_PATH}")


if __name__ == "__main__":
    main()
