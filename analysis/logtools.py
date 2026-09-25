"""
logtools.py — main.py 가 남기는 JSONL 로그(평탄화된 행)를 논문용 분석 스크립트가 같이 쓰는 도구.

키 예: "time"(벽시계), "t_mono", "dt", "control.body_vx", "vehicle_state.local_position.vx", "vehicle_state.attitude.yaw",
"relative_fru.front", "relative_fru.v_front", "ekf.x"(';' 로 이은 6개), "ekf.P_pos"(';' 6개, 상삼각 xx xy xz yy yz zz),
"reliability.gate_d2", "reliability.update_used", "leader_esp32.esp_gate_d2", "leader_esp32.leader_vel_enu"(';' 3개),
"uwb.available", "uwb.range_center_m", "uwb.age".
scipy 없이 돈다 (카이제곱 분위수는 불완전감마 CDF 의 이분법 — 정확).
"""

import json
import math
import os

import numpy as np


class Log:
    def __init__(self, rows):
        self.rows = rows
        self.n = len(rows)

    @classmethod
    def load(cls, path):
        rows = []
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    rows.append(json.loads(line))
        return cls(rows)

    def has(self, key):
        return any(key in r for r in self.rows[: min(50, self.n)]) or any(key in r for r in self.rows)

    def col(self, key, default=np.nan):
        out = np.full(self.n, default, dtype=float)
        for i, r in enumerate(self.rows):
            v = r.get(key, None)
            if v is None or isinstance(v, bool) and key.endswith("available"):
                out[i] = float(v) if isinstance(v, bool) else default
                continue
            try:
                out[i] = float(v)
            except (TypeError, ValueError):
                out[i] = default
        return out

    def col_bool(self, key):
        return np.array([bool(r.get(key, False)) for r in self.rows])

    def col_str(self, key, default=""):
        return [str(r.get(key, default)) for r in self.rows]

    def vec(self, key, n):
        """';' 로 이은 벡터 열 → (N, n) 배열 (없으면 NaN)."""
        out = np.full((self.n, n), np.nan)
        for i, r in enumerate(self.rows):
            v = r.get(key, None)
            if v is None:
                continue
            try:
                parts = [float(x) for x in str(v).split(";")]
                out[i, : min(n, len(parts))] = parts[:n]
            except ValueError:
                pass
        return out

    @property
    def t(self):
        """단조 시각(t_mono) 을 우선, 없으면 벽시계. 첫 행 기준 0 부터."""
        key = "t_mono" if self.has("t_mono") else "time"
        tt = self.col(key)
        return tt - np.nanmin(tt)


def follower_vel_fru(log):
    """LOCAL_POSITION_NED 속도(NED) 를 기체 yaw 로 돌린 (N, 3) [front, right, up]. main.follower_velocity_fru 와 같은 식."""
    vx, vy, vz = (log.col(f"vehicle_state.local_position.v{a}") for a in "xyz")
    yaw = log.col("vehicle_state.attitude.yaw")
    c, s = np.cos(yaw), np.sin(yaw)
    return np.stack([vx * c + vy * s, -vx * s + vy * c, -vz], axis=1)


def enu_to_fru(v_enu, yaw):
    """leader_telemetry.enu_to_body_fru 의 벡터화."""
    e, n, u = v_enu[:, 0], v_enu[:, 1], v_enu[:, 2]
    return np.stack([e * np.sin(yaw) + n * np.cos(yaw), e * np.cos(yaw) - n * np.sin(yaw), u], axis=1)


def rel_los_unit(log):
    rel = np.stack([log.col("relative_fru.front"), log.col("relative_fru.right"), log.col("relative_fru.up")], axis=1)
    n = np.linalg.norm(rel, axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        return rel / n[:, None], n


# ---------------------------------------------------------------- 통계
def norm_ppf(p):
    """표준정규 분위수 (이분법, |오차| < 1e-9)."""
    if not 0.0 < p < 1.0:
        raise ValueError(p)
    lo, hi = -40.0, 40.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if 0.5 * (1.0 + math.erf(mid / math.sqrt(2.0))) < p:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _gammainc_lower(a, x):
    """정규화 하부 불완전감마 P(a, x) (Numerical Recipes gammp: 급수 / Lentz 연분수). 1e-12 정밀도."""
    if x <= 0.0:
        return 0.0
    gln = math.lgamma(a)
    if x < a + 1.0:
        ap, total, delta = a, 1.0 / a, 1.0 / a
        for _ in range(500):
            ap += 1.0
            delta *= x / ap
            total += delta
            if abs(delta) < abs(total) * 1e-14:
                break
        return total * math.exp(-x + a * math.log(x) - gln)
    tiny = 1e-300
    b = x + 1.0 - a
    c = 1.0 / tiny
    d = 1.0 / b
    h = d
    for i in range(1, 500):
        an = -i * (i - a)
        b += 2.0
        d = an * d + b
        d = tiny if abs(d) < tiny else d
        c = b + an / c
        c = tiny if abs(c) < tiny else c
        d = 1.0 / d
        de = d * c
        h *= de
        if abs(de - 1.0) < 1e-14:
            break
    return 1.0 - math.exp(-x + a * math.log(x) - gln) * h


def chi2_cdf(x, df):
    return _gammainc_lower(0.5 * float(df), 0.5 * float(x))


def chi2_ppf(p, df):
    """카이제곱 분위수 — CDF(불완전감마) 이분법, scipy 없이 정확 (상대오차 < 1e-8)."""
    if not 0.0 < p < 1.0:
        raise ValueError(p)
    df = float(df)
    lo, hi = 0.0, max(10.0 * df + 100.0, 50.0)
    while chi2_cdf(hi, df) < p:
        hi *= 2.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if chi2_cdf(mid, df) < p:
            lo = mid
        else:
            hi = mid
        if hi - lo < 1e-10 * max(hi, 1.0):
            break
    return 0.5 * (lo + hi)


def fit_sinusoid(t, y, omega, trend=True):
    """y ≈ A sin(ωt) + B cos(ωt) + C (+ D t). 반환 dict(amp, phase[rad, y = amp·sin(ωt + phase)], offset, slope, resid_std,
    amp_std, phasor(complex: amp·e^{jφ})). NaN 행은 뺀다."""
    t = np.asarray(t, dtype=float); y = np.asarray(y, dtype=float)
    ok = np.isfinite(t) & np.isfinite(y)
    t, y = t[ok], y[ok]
    if t.size < 8:
        raise ValueError("표본이 너무 적다")
    cols = [np.sin(omega * t), np.cos(omega * t), np.ones_like(t)]
    if trend:
        cols.append(t - t.mean())
    X = np.stack(cols, axis=1)
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ coef
    dof = max(t.size - X.shape[1], 1)
    s2 = float(resid @ resid) / dof
    cov = s2 * np.linalg.pinv(X.T @ X)
    A, B = float(coef[0]), float(coef[1])
    amp = math.hypot(A, B)
    phase = math.atan2(B, A)
    # amp 의 표준편차: 1차 전파
    g = np.array([A / amp, B / amp]) if amp > 0 else np.array([1.0, 0.0])
    amp_std = float(math.sqrt(max(g @ cov[:2, :2] @ g, 0.0)))
    return {"amp": amp, "phase": phase, "offset": float(coef[2]), "slope": float(coef[3]) if trend else 0.0,
            "resid_std": math.sqrt(s2), "amp_std": amp_std, "phasor": amp * complex(math.cos(phase), math.sin(phase)),
            "n": int(t.size)}


def dominant_omega(t, y, w_min=0.05, w_max=6.0, n=4096):
    """정현파 시험에서 ω 를 모를 때: 선형 보간·해닝 창·16 배 zero-padding 주기도의 최대를 잡은 뒤, 그 근처에서 사인 적합 잔차를
    황금분할로 최소화해 정밀화한다 (창 길이 T 의 FFT 해상도 2π/T 에 갇히지 않는다)."""
    t = np.asarray(t, dtype=float); y = np.asarray(y, dtype=float)
    ok = np.isfinite(t) & np.isfinite(y)
    t, y = t[ok], y[ok]
    tu = np.linspace(t.min(), t.max(), n)
    yu = np.interp(tu, t, y) - np.mean(y)
    dt = tu[1] - tu[0]
    nfft = 16 * n
    freqs = np.fft.rfftfreq(nfft, dt) * 2 * np.pi
    power = np.abs(np.fft.rfft(yu * np.hanning(n), n=nfft)) ** 2
    m = (freqs >= w_min) & (freqs <= w_max)
    w0 = float(freqs[m][np.argmax(power[m])])
    bin_w = 2 * np.pi / (t.max() - t.min())

    def resid(w):
        return fit_sinusoid(t, y, w)["resid_std"]
    a, b = max(w_min, w0 - bin_w), min(w_max, w0 + bin_w)
    gr = (math.sqrt(5) - 1) / 2
    c, d = b - gr * (b - a), a + gr * (b - a)
    fc, fd = resid(c), resid(d)
    for _ in range(40):
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - gr * (b - a); fc = resid(c)
        else:
            a, c, fc = c, d, fd
            d = a + gr * (b - a); fd = resid(d)
    return float(0.5 * (a + b))


def step_segments(t, u, min_len_sec=1.0, tol=1e-6):
    """u 가 일정한 구간 [(i0, i1, value), ...] (i1 은 포함 안 함). 식별 로그의 계단 구간 찾기."""
    t = np.asarray(t, dtype=float); u = np.asarray(u, dtype=float)
    segs, i0 = [], 0
    for i in range(1, len(u) + 1):
        if i == len(u) or abs(u[i] - u[i0]) > tol:
            if t[i - 1] - t[i0] >= min_len_sec:
                segs.append((i0, i, float(u[i0])))
            i0 = i
    return segs


def ensure_dir(path):
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
