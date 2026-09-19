"""
analysis/stability_margins.py — 바깥 루프(Jetson 속도 명령) 선형 모델의 안정성 여유·스트링 안정성 계산

    python3 analysis/stability_margins.py            # 캐시된 EKF 주파수응답으로 계산 + 표 출력
    python3 analysis/stability_margins.py --plots    # docs/images/stability_*.png 생성
    python3 analysis/stability_margins.py --identify # IMM-EKF 주파수응답을 다시 실측 (약 20초)

결과 문서: docs/STABILITY_MARGINS.md.  숫자는 이 스크립트가 쓰는 docs/stability_margins.json 에서 나온다.

모델 (전후축, 라플라스 영역. 좌우·상하는 이득만 다름)
  플랜트  P(s) = H_s(s)·e^{-s·Td}·H_fc(s)          명령 → 팔로워 속도 v_F.  H_fc = 1/(τ_fc s+1) FC 속도루프,
                                                    H_s = 1/(τ_s s+1) main.smooth_velocity_cmd, Td 검출+ZOH 지연
  상대거리 d = x_L − x_F,  v_F = s·x_F
  EKF     d̂ = E_p(s)·d,  v̂_rel = E_v(s)·d          E_p, E_v 는 선형화하지 않고 imm_ekf.ImmEkf 에 정현파를 넣어 실측
  제어기  u = KFF·H_ff·(H_m·v_F + v̂_rel) + Kp·(d̂ − D*) + Kd·v̂_rel      (main.compute_velocity_cmd_from_estimate,
                                                    main.leader_velocity_ff.  H_ff = 1/(τ_ff s+1), H_m = e^{-s·Tm} FC 속도 수신 지연)
개루프 (플랜트 입력에서 절단, x_L = 0 → d = −v_F/s):
  L(s) = P(s)·[ (Kp·E_p + Kd·E_v)/s − KFF·H_ff·(H_m − E_v/s) ]
  두 번째 항이 "자기 속도 양성 되먹임" — v_L 추정에 든 자기 속도가 EKF 상대속도로 상쇄되기까지의 지연이 루프 이득을 깎는다.
리더 속도 → 팔로워 속도 (스트링 안정성):
  Γ(s) = P·(A + B·C) / (1 + P·(A − B·(H_m − C))),   A = (Kp·E_p + Kd·E_v)/s,  B = KFF·H_ff,  C = E_v/s
  체인 n 단 뒤의 속도 변동 = Γ^n.  |Γ(jω)| ≤ 1 (모든 ω) 이면 뒤로 갈수록 진동이 커지지 않는다.
"""

import argparse
import json
import math
import os
import sys
import types

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

JSON_PATH = os.path.join(_ROOT, "docs", "stability_margins.json")
IMG_DIR = os.path.join(_ROOT, "docs", "images")


# ---------------------------------------------------------------- main.py 임포트용 스텁 (test_fixes.py 와 동일)
def _stub(name, **attrs):
    if name in sys.modules:
        return sys.modules[name]
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class _FakeMavlinkConsts:
    POSITION_TARGET_TYPEMASK_X_IGNORE = 1
    POSITION_TARGET_TYPEMASK_Y_IGNORE = 2
    POSITION_TARGET_TYPEMASK_Z_IGNORE = 4
    POSITION_TARGET_TYPEMASK_VX_IGNORE = 8
    POSITION_TARGET_TYPEMASK_VY_IGNORE = 16
    POSITION_TARGET_TYPEMASK_VZ_IGNORE = 32
    POSITION_TARGET_TYPEMASK_AX_IGNORE = 64
    POSITION_TARGET_TYPEMASK_AY_IGNORE = 128
    POSITION_TARGET_TYPEMASK_AZ_IGNORE = 256
    POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1024
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 2048
    MAV_FRAME_BODY_NED = 8
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_TYPE_GCS = 6
    MAV_CMD_NAV_LAND = 21
    MAV_DATA_STREAM_ALL = 0
    MAV_DATA_STREAM_EXTENDED_STATUS = 2
    MAV_DATA_STREAM_POSITION = 6
    MAV_DATA_STREAM_EXTRA1 = 10


def _import_main():
    for _name in ("cv2", "pyrealsense2", "serial"):
        _stub(_name)
    _stub("ultralytics", YOLO=object)
    if "pymavlink.mavutil" not in sys.modules:
        _stub("pymavlink")
        _stub("pymavlink.mavutil", mavlink=_FakeMavlinkConsts, mavutil=None)
        sys.modules["pymavlink"].mavutil = sys.modules["pymavlink.mavutil"]
        sys.modules["pymavlink.mavutil"].mode_string_v10 = lambda msg: msg.mode_name
    import main  # noqa: E402
    return main


main = _import_main()
from imm_ekf import ImmEkf, LEGACY_TUNING  # noqa: E402

FPS = 30.0
SETPOINT_HZ = 10.0
SMOOTH_ALPHA = 0.28
TAU_SMOOTH = -(1.0 / FPS) / math.log(1.0 - SMOOTH_ALPHA)        # 0.1015 s: smooth_velocity_cmd 의 등가 시정수


# ---------------------------------------------------------------- 파라미터
# 세 설계. 추정기까지 포함한다 — 2026-09-19 에 추정기가 리더 절대 속도 상태 + 자기 속도 예측 입력으로 바뀌었다.
#   BEFORE : 2026-09-18 수정 전. FF 저역통과 0.7s, 자기 속도 정합 없음, 0.10 램프 데드밴드, 상대속도 추정기.
#   PREV   : 2026-09-18 수정. FF 2.0s + 자기 속도 정합 0.3s + 소프트 데드존, 상대속도 추정기 (SITL leader_sine 0.72 가 이 설계).
#   현재    : PREV 의 FF 파라미터 + 절대속도 추정기(자기 속도가 예측 입력). 정합 저역통과가 필요 없어졌다.
#   전후 이득도 다르다: BEFORE/PREV 는 Kp 0.22, 현재는 0.30 (+ 자기 속도 감쇠 KV 0.2).
LEGACY_FWD = (0.22, 0.05)
BEFORE = dict(tau_ff=0.7, tau_m=0.0, legacy_ff=True, deadband=0.10, ego_input=False, legacy_ekf=True, kv=0.0)
PREV = dict(tau_ff=2.0, tau_m=0.3, ego_input=False, legacy_ekf=True, kv=0.0)


def legacy_axes():
    return {"forward": LEGACY_FWD, "right": (main.KP_RIGHT, main.KD_RIGHT), "up": (main.KP_UP, main.KD_UP)}


def current_axes():
    return {"forward": (main.KP_FORWARD, main.KD_FORWARD), "right": (main.KP_RIGHT, main.KD_RIGHT), "up": (main.KP_UP, main.KD_UP)}


class Params:
    """선형 모델 파라미터. 기본값 = 현재 코드 + FC/지연 가정 (docs/STABILITY_MARGINS.md 의 '가정' 절)."""

    def __init__(self, **kw):
        self.kp = main.KP_FORWARD
        self.kd = main.KD_FORWARD
        self.kff = main.KFF_LEADER_VEL
        self.kv = main.KV_SELF      # 자기 속도 감쇠 (시간간격 정책 h = kv/kp). PREV/BEFORE 0
        self.tau_ff = main.FF_TAU_SEC
        self.tau_fc = 0.30      # ArduCopter 속도루프 등가 1차 시정수 (PSC_VELXY_P=2 → 0.5s 에 가속→자세 지연 포함, 보수적으로 0.3~0.5)
        self.tau_s = TAU_SMOOTH
        self.Td = 0.10          # 검출·추론 ~40ms + 10Hz ZOH 평균 50ms
        self.Tm = 0.10          # LOCAL_POSITION_NED 10Hz 수신 지연 (자기 속도)
        self.tau_m = 0.0        # [ego_input=False 전용] 자기 속도 정합 저역통과 (예전 main.self_velocity_lpf). PREV 0.3, BEFORE 0
        self.ego_input = True   # 추정기가 자기 속도를 예측 입력으로 받는다 (리더 절대 속도 상태). False = 예전 상대속도 추정기 + v_self + v_rel
        self.legacy_ekf = False  # FRF/체인에 예전 추정기 튜닝(프레임당 고정 전이행렬) 을 쓴다 — BEFORE/PREV 골든 보존용
        self.legacy_ff = False  # 체인 시뮬레이션에서 수정 전 leader_velocity_ff(램프 데드밴드) 를 쓴다
        self.scale = 1.0        # 불확실성 감속 배율 (1.0 / 0.75 / 0.55)
        self.deadband = None    # 체인 시뮬레이션에서 main.FF_DEADBAND_MPS 를 덮어쓸 값 (None = 코드 값)
        for k, v in kw.items():
            if not hasattr(self, k):
                raise KeyError(k)
            setattr(self, k, v)

    def copy(self, **kw):
        p = Params(**{k: getattr(self, k) for k in vars(self)})
        for k, v in kw.items():
            setattr(p, k, v)
        return p


# ---------------------------------------------------------------- IMM-EKF 주파수응답 실측
def _make_ekf(legacy=False):
    return ImmEkf(**LEGACY_TUNING) if legacy else ImmEkf()


def _run_ekf_sine(omega, fps=FPS, legacy=False):
    """카메라 z(전방) 축에 d(t) = 3 + a·sin(ωt) 를 넣고 정착 후 d̂, v̂ 의 복소 이득을 최소제곱으로 뽑는다.
    자기 속도 입력 0 이므로 상태 v 는 상대 속도와 같다 — 리더 운동에 대한 추정기 응답 E_p, E_v 가 나온다."""
    dt = 1.0 / fps
    period = 2 * math.pi / omega
    t_settle = max(3 * period, 10.0)
    t_fit = max(3 * period, 10.0)
    a = min(0.5, 2.0 / omega**2)               # 최대 가속 2 m/s² 로 제한 (실제 기동 범위)
    ekf = _make_ekf(legacy)
    ekf.init(np.array([0.0, 0.0, 3.0]))
    n_settle, n_fit = int(t_settle / dt), int(t_fit / dt)
    ts = np.empty(n_fit); dp = np.empty(n_fit); dv = np.empty(n_fit)
    for k in range(n_settle + n_fit):
        t = (k + 1) * dt
        ekf.predict(dt)
        ekf.update_position3d(np.array([0.0, 0.0, 3.0 + a * math.sin(omega * t)]))
        if k >= n_settle:
            x, _ = ekf.get_state()
            i = k - n_settle
            ts[i], dp[i], dv[i] = t, x[2] - 3.0, x[5]
    M = np.column_stack([np.cos(omega * ts), np.sin(omega * ts), np.ones_like(ts)])
    (ap, bp, _), *_ = np.linalg.lstsq(M, dp, rcond=None)
    (av, bv, _), *_ = np.linalg.lstsq(M, dv, rcond=None)
    # 입력 a·sin(ωt) 에 대해 출력 = b·sin + a'·cos = |G|a·sin(ωt+φ) → G = (b + j·a')/a
    return complex(bp, ap) / a, complex(bv, av) / a


def ekf_velocity_step_lag(v=0.3, fps=FPS, T=8.0):
    """등속 출발(속도 계단)에 대한 v̂ 의 63.2% 도달 시간과 정상상태 위치 지연. 직관용 (설계 계산은 FRF 를 쓴다)."""
    dt = 1.0 / fps
    ekf = ImmEkf()
    ekf.init(np.array([0.0, 0.0, 3.0]))
    t63 = None
    for k in range(int(T / dt)):
        t = (k + 1) * dt
        ekf.predict(dt)
        ekf.update_position3d(np.array([0.0, 0.0, 3.0 + v * t]))
        x, _ = ekf.get_state()
        if t63 is None and x[5] >= 0.632 * v:
            t63 = t
    x, _ = ekf.get_state()
    return {"t63_velocity_s": t63, "pos_lag_m_at_end": float(3.0 + v * T - x[2]), "vel_est_at_end": float(x[5])}


def identify_ekf_frf(omegas=None, legacy=False):
    if omegas is None:
        omegas = np.logspace(math.log10(0.03), math.log10(30.0), 28)
    Ep, Ev = [], []
    for w in omegas:
        gp, gv = _run_ekf_sine(float(w), legacy=legacy)
        Ep.append(gp); Ev.append(gv)
    return {"omega": [float(w) for w in omegas],
            "Ep_re": [g.real for g in Ep], "Ep_im": [g.imag for g in Ep],
            "Ev_re": [g.real for g in Ev], "Ev_im": [g.imag for g in Ev]}


class EkfFrf:
    """실측 FRF 를 임의 ω 로 보간. 측정 범위 밖은 이상 추정기(E_p=1, E_v=jω) 쪽으로 붙인다."""

    @staticmethod
    def pair(frf_data):
        """load_frf() 결과 → {'current': EkfFrf, 'legacy': EkfFrf}. 전달함수들이 Params.legacy_ekf 로 고른다."""
        return {"current": EkfFrf(frf_data["current"]), "legacy": EkfFrf(frf_data["legacy"])}

    def __init__(self, frf):
        self.w = np.asarray(frf["omega"], dtype=float)
        Ep = np.asarray(frf["Ep_re"]) + 1j * np.asarray(frf["Ep_im"])
        Ev = np.asarray(frf["Ev_re"]) + 1j * np.asarray(frf["Ev_im"])
        self._lw = np.log(self.w)
        self._Ep_mag, self._Ep_ph = np.log(np.abs(Ep)), np.unwrap(np.angle(Ep))
        # E_v 는 jω 로 나눈 뒤(저주파 1) 보간해야 매끄럽다
        Evn = Ev / (1j * self.w)
        self._Evn_mag, self._Evn_ph = np.log(np.abs(Evn)), np.unwrap(np.angle(Evn))

    def __call__(self, w):
        w = np.asarray(w, dtype=float)
        lw = np.log(w)
        Ep = np.exp(np.interp(lw, self._lw, self._Ep_mag) + 1j * np.interp(lw, self._lw, self._Ep_ph))
        Evn = np.exp(np.interp(lw, self._lw, self._Evn_mag) + 1j * np.interp(lw, self._lw, self._Evn_ph))
        lo = w < self.w[0]
        Ep = np.where(lo, 1.0 + 0j, Ep)
        Evn = np.where(lo, 1.0 + 0j, Evn)
        return Ep, Evn * (1j * w)


# ---------------------------------------------------------------- 전달함수
def _pieces(p, w, frf):
    f = (frf["legacy" if p.legacy_ekf else "current"] if isinstance(frf, dict) else frf)
    s = 1j * w
    P = (1.0 / (p.tau_s * s + 1)) * np.exp(-s * p.Td) * (1.0 / (p.tau_fc * s + 1))
    Ep, Ev = f(w)
    A = p.scale * (p.kp * Ep + p.kd * Ev) / s
    B = p.scale * p.kff / (p.tau_ff * s + 1)
    C = Ev / s
    Dm = np.exp(-s * p.Tm)                     # 자기 속도 수신 지연
    Hm = Dm / (p.tau_m * s + 1)
    return P, A, B, C, Hm, Dm, Ep, Ev


def _self_terms(p, w, frf):
    """자기 속도 v_F → 명령 u 의 전달 (부호는 음성 되먹임이 양). (전체, 피드포워드가 만드는 부분) 을 돌려준다.

    ego_input=True (현재): 추정기가 자기 속도를 예측 입력으로 받으므로 p̂ = E_p·p_L − [E_p(1−D_m)+D_m]·p_F,
    v̂_L = E_v·[p_L − (1−D_m)·p_F], v̂_rel = v̂_L − D_m·v_F. 자기 위치·속도 경로가 지연 없이 되먹임되고(1−D_m 만큼만
    추정기를 거친다), 피드포워드 경로는 v_F 가 오르면 v̂_L 이 내려가는 **음성** 되먹임이 된다.
    ego_input=False (PREV/BEFORE): v̂_L = H_m·v_F + E_v·(p_L − p_F) 라 H_m − E_v/s 만큼 양성 되먹임.
    """
    P, A, B, C, Hm, Dm, Ep, Ev = _pieces(p, w, frf)
    s = 1j * w
    kv_term = p.scale * p.kv * Dm                    # −KV·v_self (FC 수신 지연만, 음성 되먹임)
    if p.ego_input:
        Gp = Ep * (1 - Dm) + Dm
        Gv = Ev * (1 - Dm) / s + Dm
        Gff = Ev * (1 - Dm) / s
        ff_self = B * Gff
        return p.scale * (p.kp * Gp / s + p.kd * Gv) + ff_self + kv_term, ff_self
    return A - B * (Hm - C) + kv_term, -B * (Hm - C)


def open_loop(p, w, frf):
    P = _pieces(p, w, frf)[0]
    return P * _self_terms(p, w, frf)[0]


def self_feedback_path(p, w, frf):
    """피드포워드를 통한 자기 속도 경로의 단독 이득 |P·(FF 자기 항)|. PREV/BEFORE 에서는 양성 되먹임(main.leader_velocity_ff
    주석의 '루프 이득'), 현재 설계에서는 음성 되먹임이라 크기만 의미가 있다."""
    P = _pieces(p, w, frf)[0]
    return P * _self_terms(p, w, frf)[1]


def leader_to_follower(p, w, frf):
    P, A, B, C, *_ = _pieces(p, w, frf)
    return P * (A + B * C) / (1 + open_loop(p, w, frf))


def sensitivity(p, w, frf):
    return 1.0 / (1.0 + open_loop(p, w, frf))


def yaw_open_loop(w, kp_yaw=None, front=None, tau_yaw=0.20, Td=0.10, Tatt=0.10, tau_s=TAU_SMOOTH):
    """기수 루프: cmd = KP_YAW·atan2(right, front) ≈ KP_YAW·right/front, right ≈ −front·ψ (ego-yaw 보정은 ATTITUDE 수신 지연 Tatt 뒤 즉시).
    L_ψ = KP_YAW·H_s·e^{-s(Td+Tatt)}·H_fcψ / s.  front 는 소거된다."""
    kp_yaw = main.KP_YAW if kp_yaw is None else kp_yaw
    s = 1j * w
    return kp_yaw * (1.0 / (tau_s * s + 1)) * np.exp(-s * (Td + Tatt)) * (1.0 / (tau_yaw * s + 1)) / s


# ---------------------------------------------------------------- 여유 계산
def margins(L, w):
    """이득 교차(|L|=1) 중 최소 위상여유, 위상 교차(∠L=−180°) 중 최소 이득여유. 교차가 없으면 None."""
    mag = np.abs(L)
    ph = np.degrees(np.unwrap(np.angle(L)))
    out = {"pm_deg": None, "w_gc": None, "gm_db": None, "w_pc": None, "delay_margin_s": None,
           "Ms": float(np.max(np.abs(1.0 / (1.0 + L)))), "stable": None}
    # 이득 교차
    lm = np.log(mag)
    idx = np.where(np.sign(lm[:-1]) != np.sign(lm[1:]))[0]
    pms = []
    for i in idx:
        f = lm[i] / (lm[i] - lm[i + 1])
        wg = math.exp(math.log(w[i]) + f * (math.log(w[i + 1]) - math.log(w[i])))
        pg = ph[i] + f * (ph[i + 1] - ph[i])
        pm = ((pg + 180.0 + 180.0) % 360.0) - 180.0
        pms.append((pm, wg))
    if pms:
        pm, wg = min(pms, key=lambda t: t[0])
        out.update(pm_deg=float(pm), w_gc=float(wg), delay_margin_s=float(math.radians(pm) / wg) if pm > 0 else 0.0)
    # 위상 교차 (−180 − 360k)
    gms = []
    for k in range(0, 4):
        target = -180.0 - 360.0 * k
        d = ph - target
        j = np.where(np.sign(d[:-1]) != np.sign(d[1:]))[0]
        for i in j:
            f = d[i] / (d[i] - d[i + 1])
            wp = math.exp(math.log(w[i]) + f * (math.log(w[i + 1]) - math.log(w[i])))
            mp = math.exp(lm[i] + f * (lm[i + 1] - lm[i]))
            gms.append((-20.0 * math.log10(mp), wp))
    if gms:
        gm, wp = min(gms, key=lambda t: t[0])
        out.update(gm_db=float(gm), w_pc=float(wp))
    # 개루프에 우반면 극이 없으므로(적분기 1개 + 안정 지연/지연요소) 단순 나이퀴스트: 교차 여유가 양수면 안정
    out["stable"] = bool((out["pm_deg"] is None or out["pm_deg"] > 0) and (out["gm_db"] is None or out["gm_db"] > 0))
    return out


def string_stability(p, w, frf):
    G = np.abs(leader_to_follower(p, w, frf))
    i = int(np.argmax(G))
    return {"peak": float(G[i]), "w_peak": float(w[i]), "string_stable": bool(G[i] <= 1.0 + 1e-6),
            "dc_gain": float(G[0])}


# ---------------------------------------------------------------- 비선형 체인 시뮬레이션 (실제 main.* 함수 + 실제 ImmEkf)
def _legacy_leader_velocity_ff(prev_ff, leader_vel_fru, dt):
    """2026-09-18 수정 전 main.leader_velocity_ff: 데드밴드 DB 위로 2·DB 까지 선형 램프 (국소 기울기 최대 3)."""
    target = np.zeros(3)
    if leader_vel_fru is not None:
        v = np.asarray(leader_vel_fru, dtype=float)
        speed = float(np.linalg.norm(v))
        if speed > main.FF_DEADBAND_MPS:
            target = v * min(max((speed - main.FF_DEADBAND_MPS) / main.FF_DEADBAND_MPS, 0.0), 1.0)
    prev = np.asarray(prev_ff, dtype=float)
    a = 1.0 - math.exp(-max(float(dt), 0.0) / max(main.FF_TAU_SEC, 1e-3))
    return prev + a * (target - prev)


class _Follower:
    def __init__(self, x0, p):
        self.x = float(x0); self.v = 0.0
        self.ekf = _make_ekf(p.legacy_ekf); self.ekf.init(np.array([0.0, 0.0, 3.0]))
        self.ff = np.zeros(3); self.cmd = np.zeros(4)
        self.sent = 0.0                                  # FC 가 현재 들고 있는 setpoint
        self.delay = [0.0] * max(1, int(round(p.Td * FPS)))     # 검출·전송 지연 버퍼
        self.vmeas = [0.0] * max(1, int(round(p.Tm * FPS)))     # 자기 속도 수신 지연 버퍼
        self.v_self_f = 0.0                                      # [제안] 정합 저역통과 상태
        self.p = p


def chain_sim(n_followers=4, v_leader=0.3, T=50.0, p=None, profile="step", omega=None, amp=None):
    """리더 + n 팔로워 체인. 각 팔로워는 앞 기체와의 거리만 카메라 z 축 측정으로 받는다(잡음 없음).
    profile: 'step' = 2s 뒤 1s 램프로 v_leader, 30s 에 정지 / 'sine' = v_leader + amp·sin ωt (amp 기본 = v_leader)."""
    p = p or Params()
    saved = (main.KFF_LEADER_VEL, main.FF_TAU_SEC, main.FF_DEADBAND_MPS, main.KP_FORWARD, main.KD_FORWARD, main.KV_SELF)
    main.KFF_LEADER_VEL, main.FF_TAU_SEC = p.kff, p.tau_ff
    main.KP_FORWARD, main.KD_FORWARD, main.KV_SELF = p.kp, p.kd, p.kv
    if p.deadband is not None:
        main.FF_DEADBAND_MPS = p.deadband
    dt = 1.0 / FPS
    send_every = int(round(FPS / SETPOINT_HZ))
    xs = [0.0]
    fols = [_Follower(-3.0 * (i + 1), p) for i in range(n_followers)]
    hist = {"t": [], "leader_v": [], "err": [[] for _ in fols], "v": [[] for _ in fols]}
    xl, vl = 0.0, 0.0
    try:
        for k in range(int(T / dt)):
            t = k * dt
            if profile == "step":
                if t < 2.0: vl = 0.0
                elif t < 3.0: vl = v_leader * (t - 2.0)
                elif t < 30.0: vl = v_leader
                elif t < 31.0: vl = v_leader * (31.0 - t)
                else: vl = 0.0
            else:
                a_ = v_leader if amp is None else amp
                vl = (v_leader + a_ * math.sin(omega * t)) if t >= 2.0 else 0.0
            xl += vl * dt
            x_front, v_front = xl, vl
            for i, f in enumerate(fols):
                d = x_front - f.x
                f.vmeas.append(f.v); v_self = f.vmeas.pop(0)              # FC 자기 속도 (Tm 지연)
                if p.ego_input:
                    f.ekf.set_ego_velocity_cam([0.0, 0.0, v_self])       # 예측 입력 (카메라 z = 전방)
                f.ekf.predict(dt)
                f.ekf.update_position3d(np.array([0.0, 0.0, d]))
                xe, _ = f.ekf.get_state()
                rel = main.camera_xyz_to_fru(xe[:3])
                if p.ego_input:
                    relv = main.camera_xyz_to_fru(f.ekf.relative_velocity())
                    v_lead_est = main.camera_xyz_to_fru(f.ekf.leader_velocity())[0]
                else:
                    relv = main.camera_xyz_to_fru(xe[3:6])
                    if p.tau_m > 0:
                        f.v_self_f += (v_self - f.v_self_f) * (1.0 - math.exp(-dt / p.tau_m)); v_self = f.v_self_f
                    v_lead_est = v_self + relv[0]
                ff_fn = _legacy_leader_velocity_ff if p.legacy_ff else main.leader_velocity_ff
                f.ff = ff_fn(f.ff, [v_lead_est, 0.0, 0.0], dt)
                u = main.compute_velocity_cmd_from_estimate(rel, relv, 1.0, None, f.ff,
                                                            v_self_fru=[v_self, 0.0, 0.0] if p.ego_input else None)
                f.cmd = main.smooth_velocity_cmd(f.cmd, u, alpha=SMOOTH_ALPHA, dt=dt)
                if k % send_every == 0:
                    f.delay.append(float(f.cmd[0]))
                    f.sent = f.delay.pop(0)
                f.v += (f.sent - f.v) * (dt / p.tau_fc)
                f.x += f.v * dt
                hist["err"][i].append(d - main.TARGET_DISTANCE_M); hist["v"][i].append(f.v)
                x_front, v_front = f.x, f.v
            hist["t"].append(t); hist["leader_v"].append(vl)
    finally:
        (main.KFF_LEADER_VEL, main.FF_TAU_SEC, main.FF_DEADBAND_MPS,
         main.KP_FORWARD, main.KD_FORWARD, main.KV_SELF) = saved
    for key in ("t", "leader_v"):
        hist[key] = np.asarray(hist[key])
    hist["err"] = [np.asarray(e) for e in hist["err"]]; hist["v"] = [np.asarray(v) for v in hist["v"]]
    return hist


def chain_summary(hist, t_from=0.0):
    m = hist["t"] >= t_from
    return {"max_abs_err_m": [float(np.max(np.abs(e[m]))) for e in hist["err"]],
            "min_spacing_m": [float(main.TARGET_DISTANCE_M + np.min(e[m])) for e in hist["err"]],
            "final_err_m": [float(e[-1]) for e in hist["err"]]}


def chain_sine_amplitudes(hist, omega, t_from=20.0):
    """정현파 리더에 대한 각 단 속도 진폭 (최소제곱) → 인접 단 비율이 |Γ(jω)| 의 비선형 실측치."""
    m = hist["t"] >= t_from
    t = hist["t"][m]
    M = np.column_stack([np.cos(omega * t), np.sin(omega * t), np.ones_like(t)])

    def amp(y):
        (a, b, _), *_ = np.linalg.lstsq(M, y[m], rcond=None)
        return math.hypot(a, b)
    amps = [amp(hist["leader_v"])] + [amp(v) for v in hist["v"]]
    ratios = [amps[i + 1] / amps[i] if amps[i] > 1e-9 else float("nan") for i in range(len(amps) - 1)]
    return {"amplitudes": amps, "stage_ratios": ratios}


# ---------------------------------------------------------------- 전체 계산
W = np.logspace(-2, math.log10(30.0), 3000)


def load_frf(identify=False):
    """{'current': 현재 추정기 FRF, 'legacy': 예전 추정기 FRF}. JSON 에 있으면 재사용, 없으면 실측(각 ~40s)."""
    data = {}
    if not identify and os.path.exists(JSON_PATH):
        with open(JSON_PATH) as f:
            data = json.load(f)
    return {"current": data.get("ekf_frf") or identify_ekf_frf(),
            "legacy": data.get("ekf_frf_legacy") or identify_ekf_frf(legacy=True)}


def compute_all(frf_data, chain=True):
    frf = EkfFrf.pair(frf_data)
    base = Params()
    res = {"params": {k: getattr(base, k) for k in vars(base)},
           "gains": {"forward": (main.KP_FORWARD, main.KD_FORWARD), "right": (main.KP_RIGHT, main.KD_RIGHT),
                     "up": (main.KP_UP, main.KD_UP), "yaw": main.KP_YAW,
                     "limits": (main.MAX_VX, main.MAX_VY, main.MAX_VZ, main.MAX_YAW_RATE)},
           "ekf_step": ekf_velocity_step_lag(), "axes": {}, "kff_sweep": {}, "robustness": [], "yaw": {}}
    # 축별 (KFF = 설계값)
    for name, (kp, kd) in current_axes().items():
        p = base.copy(kp=kp, kd=kd)
        r = margins(open_loop(p, W, frf), W)
        r.update(string_stability(p, W, frf))
        r["self_fb_peak"] = float(np.max(np.abs(self_feedback_path(p, W, frf))))
        res["axes"][name] = r
    # 수정 전(BEFORE) / 직전(PREV) 설계 (축별)
    for key, design in (("before", BEFORE), ("prev", PREV)):
        res[key] = {}
        for name, (kp, kd) in legacy_axes().items():
            p = base.copy(kp=kp, kd=kd, **design)
            r = margins(open_loop(p, W, frf), W); r.update(string_stability(p, W, frf))
            r["self_fb_peak"] = float(np.max(np.abs(self_feedback_path(p, W, frf))))
            res[key][name] = r
    # 전후축 KFF 스윕 (P+D 만 / 현재 / 직전 / 수정 전 / KFF=1 / KFF=1 & 저역통과 없음)
    for label, kw in (("kff0", dict(kff=0.0, kv=0.0)), ("kff0.8", dict()), ("prev", dict(kp=LEGACY_FWD[0], kd=LEGACY_FWD[1], **PREV)),
                      ("before", dict(kp=LEGACY_FWD[0], kd=LEGACY_FWD[1], **BEFORE)),
                      ("kff1.0", dict(kff=1.0)), ("kff1.0_nolpf", dict(kff=1.0, tau_ff=1e-3)), ("kv0", dict(kv=0.0)),
                      ("kv0_tau2.0", dict(kv=0.0, tau_ff=2.0))):
        p = base.copy(**kw)
        r = margins(open_loop(p, W, frf), W)
        r.update(string_stability(p, W, frf))
        r["self_fb_peak"] = float(np.max(np.abs(self_feedback_path(p, W, frf))))
        r["ss_err_per_mps"] = (1.0 - p.kff) / p.kp
        res["kff_sweep"][label] = r
    # [2026-09-18 의 결정 기록] 상대속도 추정기에서 FF 저역통과 τ_ff × 자기 속도 정합 저역통과 τ_m (KFF 설계값)
    res["proposed"] = {}
    for tau_ff in (0.7, 1.0, 1.5, 2.0, 3.0):
        for tau_m in (0.0, 0.3, 0.5):
            p = base.copy(tau_ff=tau_ff, tau_m=tau_m, ego_input=False, legacy_ekf=True, kv=0.0, kp=LEGACY_FWD[0], kd=LEGACY_FWD[1])
            r = margins(open_loop(p, W, frf), W); r.update(string_stability(p, W, frf))
            r["self_fb_peak"] = float(np.max(np.abs(self_feedback_path(p, W, frf))))
            res["proposed"][f"tau_ff{tau_ff}_tau_m{tau_m}"] = r
    # 현재 추정기(절대 속도)에서 KFF × Kd (τ_ff 코드값) — FCR-10 의 |Γ| 피크와 정상상태 오차의 교환표
    res["recommended_grid"] = {}
    for kff in (0.6, 0.7, 0.8, 0.9, 1.0):
        for kd in (0.05, 0.10, 0.15):
            p = base.copy(kff=kff, kd=kd)
            r = margins(open_loop(p, W, frf), W); r.update(string_stability(p, W, frf))
            r["ss_err_per_mps"] = (1.0 - kff) / p.kp
            res["recommended_grid"][f"kff{kff}_kd{kd}"] = r
    # 현재 추정기에서 τ_ff × KV 스윕 (KFF 코드값, Kp 코드값) — FF 저역통과와 시간간격 정책의 스트링 피크
    res["ego_tau_ff"] = {}
    for tau_ff in (0.1, 0.2, 0.3, 0.5, 1.0, 2.0):
        for kv in (0.0, 0.1, 0.2, 0.3):
            p = base.copy(tau_ff=tau_ff, kv=kv)
            r = margins(open_loop(p, W, frf), W); r.update(string_stability(p, W, frf))
            r["self_fb_peak"] = float(np.max(np.abs(self_feedback_path(p, W, frf))))
            r["ss_err_per_mps"] = (1.0 - p.kff + kv) / p.kp
            res["ego_tau_ff"][f"tau_ff{tau_ff}_kv{kv}"] = r
    # Kp 스윕 (KV·τ_ff 코드값): 시간간격 정책이 늘린 이격을 Kp 로 되사는 여유
    res["kp_sweep"] = {}
    for kp in (0.22, 0.26, 0.30, 0.35):
        p = base.copy(kp=kp)
        r = margins(open_loop(p, W, frf), W); r.update(string_stability(p, W, frf))
        r["ss_err_per_mps"] = (1.0 - p.kff + p.kv) / kp
        res["kp_sweep"][f"kp{kp}"] = r
    res["recommended_params"] = {"tau_ff": base.tau_ff, "kff": base.kff, "kp": base.kp, "kd": base.kd, "kv": base.kv, "ego_input": True}
    # 데드밴드: 수정 전 램프 target = v·(|v|−DB)/DB (DB<|v|<2DB) 의 국소 기울기 최대 3, 현재 소프트 데드존은 1
    res["deadband_slope"] = {"before_ramp_max": 3.0, "current_soft": 1.0}
    # 불확실성 감속
    for sc in (0.75, 0.55):
        p = base.copy(scale=sc)
        r = margins(open_loop(p, W, frf), W); r.update(string_stability(p, W, frf))
        res["kff_sweep"][f"scale{sc}"] = r
    # 강건성: FC 시정수 × 지연
    for tau_fc in (0.2, 0.3, 0.5, 0.8, 1.2):
        for Td in (0.1, 0.2, 0.3, 0.5):
            p = base.copy(tau_fc=tau_fc, Td=Td)
            r = margins(open_loop(p, W, frf), W); g = string_stability(p, W, frf)
            res["robustness"].append({"tau_fc": tau_fc, "Td": Td, "pm_deg": r["pm_deg"], "gm_db": r["gm_db"],
                                      "Ms": r["Ms"], "gamma_peak": g["peak"]})
    # 기수
    ry = margins(yaw_open_loop(W), W)
    ry_slow = margins(yaw_open_loop(W, tau_yaw=0.5, Td=0.3), W)
    res["yaw"] = {"nominal": ry, "slow_fc_long_delay": ry_slow}
    # 이산화 여유
    res["sampling"] = {"setpoint_hz": SETPOINT_HZ, "nyquist_rad_s": math.pi * SETPOINT_HZ,
                       "w_gc_over_nyquist": res["axes"]["forward"]["w_gc"] / (math.pi * SETPOINT_HZ)}
    if chain:
        pb = base.copy(kp=LEGACY_FWD[0], kd=LEGACY_FWD[1], **BEFORE)
        pp = base.copy(kp=LEGACY_FWD[0], kd=LEGACY_FWD[1], **PREV)
        w_sine = res["before"]["forward"]["w_peak"]          # 수정 전 설계의 |Γ| 피크 주파수 (≈1.15) 에서 비교
        h = chain_sim(p=base)
        res["chain_step"] = chain_summary(h, t_from=0.0)
        h0 = chain_sim(p=base.copy(kff=0.0, kv=0.0))
        res["chain_step_kff0"] = chain_summary(h0, t_from=0.0)
        hb = chain_sim(p=pb)
        res["chain_step_before"] = chain_summary(hb, t_from=0.0)
        hs = chain_sim(p=base, profile="sine", omega=w_sine, v_leader=0.15, T=80.0)
        res["chain_sine"] = dict(omega=w_sine, **chain_sine_amplitudes(hs, w_sine, t_from=30.0))
        hs0 = chain_sim(p=base.copy(kff=0.0, kv=0.0), profile="sine", omega=w_sine, v_leader=0.15, T=80.0)
        res["chain_sine_kff0"] = dict(omega=w_sine, **chain_sine_amplitudes(hs0, w_sine, t_from=30.0))
        hsb = chain_sim(p=pb, profile="sine", omega=w_sine, v_leader=0.15, T=80.0)
        res["chain_sine_before"] = dict(omega=w_sine, **chain_sine_amplitudes(hsb, w_sine, t_from=30.0))
        gr = res["axes"]["forward"]
        hsr = chain_sim(p=base, profile="sine", omega=gr["w_peak"], v_leader=0.15, T=120.0)
        res["chain_sine_at_own_peak"] = dict(omega=gr["w_peak"], **chain_sine_amplitudes(hsr, gr["w_peak"], t_from=40.0))
        # SITL leader_sine 시나리오와 같은 조건 (리더 0.25 ± 0.05, 1.15 rad/s), 1단. SITL 실측 0.72 는 PREV 설계.
        res["sitl_like"] = {}
        for label, pv in (("current", base), ("prev", pp), ("before", pb)):
            hv = chain_sim(n_followers=1, p=pv, profile="sine", omega=1.15, v_leader=0.25, amp=0.05, T=80.0)
            res["sitl_like"][label] = chain_sine_amplitudes(hv, 1.15, t_from=30.0)["stage_ratios"][0]
        # 선형 모델 검증: 데드밴드·포화 밖의 작은 진폭 (리더 0.30 ± 0.02 m/s), 2단
        res["validation"] = []
        for label, pv in (("current", base), ("kff0", base.copy(kff=0.0, kv=0.0)), ("prev", pp), ("before", pb)):
            for wv in (1.15, 0.35):
                hv = chain_sim(n_followers=2, p=pv, profile="sine", omega=wv, v_leader=0.30, amp=0.02, T=80.0)
                av = chain_sine_amplitudes(hv, wv, t_from=30.0)
                res["validation"].append({"case": label, "omega": wv,
                                          "linear": float(abs(leader_to_follower(pv, np.array([wv]), frf)[0])),
                                          "nonlinear_stage1": av["stage_ratios"][0], "nonlinear_stage2": av["stage_ratios"][1]})
        # 데드밴드 구간(리더 0.15 ± 0.03 m/s) 의 비선형 증폭: 수정 전 램프 vs 현재 소프트 데드존
        res["deadband_lowspeed_ratios"] = {}
        for label, pv in (("current", base), ("before", pb)):
            hd = chain_sim(n_followers=2, p=pv, profile="sine", omega=1.15, v_leader=0.15, amp=0.03, T=80.0)
            res["deadband_lowspeed_ratios"][label] = chain_sine_amplitudes(hd, 1.15, t_from=30.0)["stage_ratios"]
        res["_hist"] = {"step": h, "step_kff0": h0, "step_before": hb}
    return res


# ---------------------------------------------------------------- 출력
def _fmt(v, nd=1, unit=""):
    return "—" if v is None else f"{v:.{nd}f}{unit}"


def print_tables(res):
    print("\n[축별 여유 — KFF 설계값, τ_fc=%.2fs, Td=%.2fs]" % (res["params"]["tau_fc"], res["params"]["Td"]))
    print("| 축 | Kp / Kd | ω_gc [rad/s] | PM [°] | GM [dB] | 지연 여유 [s] | Ms | |Γ| 피크 (ω) |")
    print("|---|---|---|---|---|---|---|---|")
    for name in ("forward", "right", "up"):
        r = res["axes"][name]; kp, kd = res["gains"][name]
        print(f"| {name} | {kp} / {kd} | {_fmt(r['w_gc'],3)} | {_fmt(r['pm_deg'])} | {_fmt(r['gm_db'])} | "
              f"{_fmt(r['delay_margin_s'],2)} | {_fmt(r['Ms'],2)} | {_fmt(r['peak'],3)} ({_fmt(r['w_peak'],2)}) |")
    print("\n[전후축 KFF 스윕]")
    print("| 경우 | PM [°] | GM [dB] | Ms | 양성되먹임 경로 피크 | |Γ| 피크 | 스트링 안정 | 정상상태 오차 [m per m/s] |")
    print("|---|---|---|---|---|---|---|---|")
    for label, r in res["kff_sweep"].items():
        print(f"| {label} | {_fmt(r['pm_deg'])} | {_fmt(r['gm_db'])} | {_fmt(r['Ms'],2)} | {_fmt(r.get('self_fb_peak'),2)} | "
              f"{_fmt(r['peak'],3)} | {'예' if r['string_stable'] else '아니오'} | {_fmt(r.get('ss_err_per_mps'),2)} |")
    print("\n[τ_ff × τ_m 스윕 (수정 전 = 0.7/0, 현재 = 2.0/0.3) → GM / Ms / |Γ|피크(ω) / 양성되먹임 피크]")
    tms = (0.0, 0.3, 0.5)
    print("| τ_ff \\ τ_m | " + " | ".join(f"{tm}s" for tm in tms) + " |")
    print("|---|" + "---|" * len(tms))
    for tff in (0.7, 1.0, 1.5, 2.0, 3.0):
        row = []
        for tm in tms:
            r = res["proposed"][f"tau_ff{tff}_tau_m{tm}"]
            row.append(f"{_fmt(r['gm_db'])}dB / {_fmt(r['Ms'],2)} / {_fmt(r['peak'],2)}({_fmt(r['w_peak'],2)}) / {_fmt(r['self_fb_peak'],2)}")
        print(f"| {tff}s | " + " | ".join(row) + " |")
    print("\n[현재 추정기 τ_ff × KV (KFF·Kp 코드값): |Γ|피크(ω) / GM / 정상상태 오차]")
    kvs = (0.0, 0.1, 0.2, 0.3)
    print("| τ_ff \\ KV | " + " | ".join(str(k) for k in kvs) + " |"); print("|---|" + "---|" * len(kvs))
    for tau_ff in (0.1, 0.2, 0.3, 0.5, 1.0, 2.0):
        row = [res["ego_tau_ff"][f"tau_ff{tau_ff}_kv{kv}"] for kv in kvs]
        print(f"| {tau_ff}s | " + " | ".join(f"{_fmt(r['peak'],3)}({_fmt(r['w_peak'],2)}) / {_fmt(r['gm_db'])}dB / {_fmt(r['ss_err_per_mps'],2)}" for r in row) + " |")
    print("[Kp 스윕 (KV·τ_ff 코드값)] " + "; ".join(f"Kp {k[2:]}: |Γ| {_fmt(r['peak'],3)} GM {_fmt(r['gm_db'])}dB PM {_fmt(r['pm_deg'],0)}° e_ss {_fmt(r['ss_err_per_mps'],2)}" for k, r in res["kp_sweep"].items()))
    print("\n[현재 추정기 KFF × Kd 교환표 (τ_ff 코드값): PM / GM / Ms / |Γ|피크 / 정상상태 오차]")
    print("| KFF \\ Kd | 0.05 | 0.10 | 0.15 |")
    print("|---|---|---|---|")
    for kff in (0.6, 0.7, 0.8, 0.9, 1.0):
        row = []
        for kd in (0.05, 0.10, 0.15):
            r = res["recommended_grid"][f"kff{kff}_kd{kd}"]
            row.append(f"{_fmt(r['pm_deg'],0)}° / {_fmt(r['gm_db'])}dB / {_fmt(r['Ms'],2)} / {_fmt(r['peak'],3)} / {_fmt(r['ss_err_per_mps'],2)}")
        print(f"| {kff} | " + " | ".join(row) + " |")
    print("\n[강건성: FC 시정수 × 지연 → PM / GM / |Γ|피크]")
    tds = sorted({r["Td"] for r in res["robustness"]}); tfs = sorted({r["tau_fc"] for r in res["robustness"]})
    print("| τ_fc \\ Td | " + " | ".join(f"{td:.1f}s" for td in tds) + " |")
    print("|---|" + "---|" * len(tds))
    for tf in tfs:
        row = []
        for td in tds:
            r = next(x for x in res["robustness"] if x["tau_fc"] == tf and x["Td"] == td)
            row.append(f"{_fmt(r['pm_deg'],0)}° / {_fmt(r['gm_db'],0)}dB / {_fmt(r['gamma_peak'],2)}")
        print(f"| {tf:.1f}s | " + " | ".join(row) + " |")
    y = res["yaw"]["nominal"]; ys = res["yaw"]["slow_fc_long_delay"]
    print(f"\n[기수] nominal PM {_fmt(y['pm_deg'])}° GM {_fmt(y['gm_db'])}dB ω_gc {_fmt(y['w_gc'],2)} | "
          f"τ_yaw 0.5/Td 0.3: PM {_fmt(ys['pm_deg'])}° GM {_fmt(ys['gm_db'])}dB")
    e = res["ekf_step"]
    print(f"[EKF 계단] v̂ 63% 도달 {_fmt(e['t63_velocity_s'],2)}s, 8s 후 위치 지연 {_fmt(e['pos_lag_m_at_end'],3)}m")
    print(f"[이산화] ω_gc/ω_Nyquist = {res['sampling']['w_gc_over_nyquist']:.4f}")
    if "chain_step" in res:
        c, c0 = res["chain_step"], res["chain_step_kff0"]
        print("\n[체인 4단 계단(0.3 m/s)] max|e| KFF=0.8: " + ", ".join(f"{v:.2f}" for v in c["max_abs_err_m"]) +
              " | KFF=0: " + ", ".join(f"{v:.2f}" for v in c0["max_abs_err_m"]))
        print("  최소 간격 KFF=0.8: " + ", ".join(f"{v:.2f}" for v in c["min_spacing_m"]))
        cm = res["chain_step_before"]
        print("  [수정 전] max|e|: " + ", ".join(f"{v:.2f}" for v in cm["max_abs_err_m"]) +
              " 최소 간격: " + ", ".join(f"{v:.2f}" for v in cm["min_spacing_m"]))
        s, s0, sm = res["chain_sine"], res["chain_sine_kff0"], res["chain_sine_before"]
        print(f"[체인 정현파 ω={s['omega']:.2f}, 리더 0.15±0.15] 단별 진폭비 현재: " + ", ".join(f"{v:.3f}" for v in s["stage_ratios"]) +
              " | KFF=0: " + ", ".join(f"{v:.3f}" for v in s0["stage_ratios"]) +
              " | 수정 전: " + ", ".join(f"{v:.3f}" for v in sm["stage_ratios"]))
        sr = res["chain_sine_at_own_peak"]
        print(f"[현재 설계 자체 피크 ω={sr['omega']:.2f}] 단별 진폭비: " + ", ".join(f"{v:.3f}" for v in sr["stage_ratios"]))
        print(f"[SITL leader_sine 조건 (0.25±0.05, 1.15)] 1단 진폭비 현재 {res['sitl_like']['current']:.3f} / 직전(SITL 실측 0.72) {res['sitl_like']['prev']:.3f} / 수정 전 {res['sitl_like']['before']:.3f}")
        print("[선형 모델 검증: 리더 0.30±0.02 m/s] " + "; ".join(
            f"{v['case']}@{v['omega']}: 선형 {v['linear']:.3f} / 비선형 {v['nonlinear_stage1']:.3f}" for v in res["validation"]))
        d = res["deadband_lowspeed_ratios"]
        print("[데드밴드 구간 리더 0.15±0.03 @1.15] 단별 진폭비 현재(소프트): " + ", ".join(f"{v:.3f}" for v in d["current"]) +
              " | 수정 전(램프): " + ", ".join(f"{v:.3f}" for v in d["before"]))


def make_plots(res, frf_data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    frf = EkfFrf.pair(frf_data); base = Params()
    os.makedirs(IMG_DIR, exist_ok=True)
    # 1) 전후축 보드 (KFF 0 / 0.8 / 1.0)
    fig, ax = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    for label, kw, c in (("P+D only (KFF=0)", dict(kff=0.0, kv=0.0), "gray"),
                         ("before 2026-09-18 (FF LPF 0.7s, relative-velocity EKF)", dict(kp=LEGACY_FWD[0], kd=LEGACY_FWD[1], **BEFORE), "tab:red"),
                         ("2026-09-18 fix (FF LPF 2.0s + self-vel LPF 0.3s)", dict(kp=LEGACY_FWD[0], kd=LEGACY_FWD[1], **PREV), "tab:orange"),
                         ("current (ego-input EKF, FF LPF 0.1s, time-gap KV 0.2, Kp 0.30)", {}, "tab:green")):
        L = open_loop(base.copy(**kw), W, frf)
        ax[0].semilogx(W, 20 * np.log10(np.abs(L)), color=c, label=label)
        ax[1].semilogx(W, np.degrees(np.unwrap(np.angle(L))), color=c)
    r = res["axes"]["forward"]
    ax[0].axhline(0, color="k", lw=0.6); ax[1].axhline(-180, color="k", lw=0.6)
    if r["w_gc"]:
        ax[0].axvline(r["w_gc"], color="tab:green", ls=":", lw=0.8); ax[1].axvline(r["w_gc"], color="tab:green", ls=":", lw=0.8)
        ax[1].annotate(f"PM {r['pm_deg']:.0f}° @ {r['w_gc']:.2f} rad/s", (r["w_gc"], -180 + r["pm_deg"]), textcoords="offset points", xytext=(8, 6))
    if r["w_pc"]:
        ax[0].axvline(r["w_pc"], color="tab:green", ls="--", lw=0.8)
        ax[0].annotate(f"GM {r['gm_db']:.1f} dB @ {r['w_pc']:.1f} rad/s", (r["w_pc"], -r["gm_db"]), textcoords="offset points", xytext=(-140, 8))
    ax[0].set_ylabel("|L| [dB]"); ax[1].set_ylabel("∠L [deg]"); ax[1].set_xlabel("ω [rad/s]")
    ax[0].set_title(f"Forward-axis open loop L(jω)  (τ_fc={base.tau_fc}s, Td={base.Td}s, EKF FRF measured)")
    ax[0].legend(fontsize=8); ax[0].grid(True, which="both", alpha=0.3); ax[1].grid(True, which="both", alpha=0.3)
    ax[1].set_ylim(-400, 0)
    fig.tight_layout(); fig.savefig(os.path.join(IMG_DIR, "stability_bode_forward.png"), dpi=130); plt.close(fig)
    # 2) 스트링 안정성
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for label, kw, c in (("KFF=0", dict(kff=0.0), "gray"), ("before 2026-09-18 (KFF 0.8, FF LPF 0.7s)", dict(**BEFORE), "tab:red"),
                         ("before, KFF=1.0", dict(kff=1.0, tau_ff=0.7, tau_m=0.0), "tab:orange"),
                         ("KFF=1.0, no LPF", dict(kff=1.0, tau_ff=1e-3, tau_m=0.0), "tab:purple"),
                         ("current (KFF 0.8, FF LPF 2.0s + self-vel LPF 0.3s)", {}, "tab:green")):
        G = np.abs(leader_to_follower(base.copy(**kw), W, frf))
        ax.semilogx(W, G, color=c, label=f"{label}  peak {G.max():.3f}")
    ax.axhline(1.0, color="k", lw=0.8, ls="--")
    ax.set_xlabel("ω [rad/s]"); ax.set_ylabel("|Γ(jω)| = |v_F / v_L|"); ax.set_ylim(0, 2.0)
    ax.set_title("String stability: leader→follower velocity gain (chain of n = Γⁿ)")
    ax.legend(fontsize=8); ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(IMG_DIR, "stability_string.png"), dpi=130); plt.close(fig)
    # 3) 체인 계단 응답
    if "_hist" in res:
        fig, ax = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
        for j, (key, title) in enumerate((("step", "current (KFF 0.8, FF LPF 2.0s + self-velocity LPF 0.3s)"), ("step_kff0", "KFF=0 (P+D only)"),
                                          ("step_before", "before 2026-09-18 (FF LPF 0.7s, no self-velocity LPF)"))):
            h = res["_hist"][key]
            for i, e in enumerate(h["err"]):
                ax[j].plot(h["t"], e, label=f"follower {i+1}")
            ax[j].plot(h["t"], h["leader_v"] * 3.0, "k:", lw=0.8, label="leader v ×3 [m/s]")
            ax[j].set_ylabel("spacing error [m]"); ax[j].set_title(f"4-follower chain, leader 0.3 m/s step — {title}")
            ax[j].grid(alpha=0.3); ax[j].legend(fontsize=7, ncol=3)
        ax[-1].set_xlabel("t [s]")
        fig.tight_layout(); fig.savefig(os.path.join(IMG_DIR, "stability_chain_step.png"), dpi=130); plt.close(fig)
    # 4) 강건성
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.8))
    tds = sorted({r["Td"] for r in res["robustness"]}); tfs = sorted({r["tau_fc"] for r in res["robustness"]})
    for td in tds:
        pms = [next(x["pm_deg"] for x in res["robustness"] if x["tau_fc"] == tf and x["Td"] == td) for tf in tfs]
        gps = [next(x["gamma_peak"] for x in res["robustness"] if x["tau_fc"] == tf and x["Td"] == td) for tf in tfs]
        ax[0].plot(tfs, pms, "o-", label=f"Td={td:.1f}s"); ax[1].plot(tfs, gps, "o-", label=f"Td={td:.1f}s")
    ax[0].axhline(45, color="r", ls="--", lw=0.8); ax[0].set_xlabel("τ_fc [s]"); ax[0].set_ylabel("PM [deg]"); ax[0].set_title("Phase margin vs FC lag / delay")
    ax[1].axhline(1.0, color="r", ls="--", lw=0.8); ax[1].set_xlabel("τ_fc [s]"); ax[1].set_ylabel("|Γ| peak"); ax[1].set_title("String-stability peak")
    for a in ax:
        a.grid(alpha=0.3); a.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(IMG_DIR, "stability_robustness.png"), dpi=130); plt.close(fig)
    # 5) EKF FRF
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.8))
    fd = frf_data["current"]
    w = np.asarray(fd["omega"]); Ep = np.asarray(fd["Ep_re"]) + 1j * np.asarray(fd["Ep_im"])
    Ev = np.asarray(fd["Ev_re"]) + 1j * np.asarray(fd["Ev_im"])
    ax[0].semilogx(w, 20 * np.log10(np.abs(Ep)), "o-", label="|d̂/d|"); ax[0].semilogx(w, 20 * np.log10(np.abs(Ev / (1j * w))), "s-", label="|v̂/(jω d)|")
    ax[0].set_ylabel("[dB]"); ax[0].set_title("IMM-EKF measured FRF (magnitude)"); ax[0].legend(fontsize=8); ax[0].grid(True, which="both", alpha=0.3)
    ax[1].semilogx(w, np.degrees(np.unwrap(np.angle(Ep))), "o-", label="∠ d̂/d"); ax[1].semilogx(w, np.degrees(np.unwrap(np.angle(Ev / (1j * w)))), "s-", label="∠ v̂/(jω d)")
    ax[1].set_ylabel("[deg]"); ax[1].set_title("phase (lag)"); ax[1].legend(fontsize=8); ax[1].grid(True, which="both", alpha=0.3)
    for a in ax:
        a.set_xlabel("ω [rad/s]")
    fig.tight_layout(); fig.savefig(os.path.join(IMG_DIR, "stability_ekf_frf.png"), dpi=130); plt.close(fig)


def _jsonable(res):
    out = {k: v for k, v in res.items() if not k.startswith("_")}
    return json.loads(json.dumps(out, default=lambda o: float(o) if isinstance(o, (np.floating, np.integer)) else str(o)))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--identify", action="store_true", help="IMM-EKF 주파수응답 재실측")
    ap.add_argument("--plots", action="store_true", help="docs/images/stability_*.png 생성")
    ap.add_argument("--no-chain", action="store_true")
    args = ap.parse_args()
    frf_data = load_frf(identify=args.identify)
    res = compute_all(frf_data, chain=not args.no_chain)
    print_tables(res)
    if args.plots:
        make_plots(res, frf_data)
        print(f"[plots] {IMG_DIR}/stability_*.png")
    payload = _jsonable(res); payload["ekf_frf"] = frf_data["current"]; payload["ekf_frf_legacy"] = frf_data["legacy"]
    os.makedirs(os.path.dirname(JSON_PATH), exist_ok=True)
    with open(JSON_PATH, "w") as f:
        json.dump(payload, f, indent=1, ensure_ascii=False)
    print(f"[json] {JSON_PATH}")
