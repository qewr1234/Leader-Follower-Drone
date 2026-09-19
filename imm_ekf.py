"""
imm_ekf.py — IMM-EKF 상태 추정기 (CV + Coordinated-Turn 2모델), adaptive R / bearing update

상태벡터 x = [px, py, pz, vx, vy, vz], 카메라 좌표계 (x=right, y=down, z=forward).
  p = 리더의 **상대** 위치 (측정이 상대라서),  v = 리더의 **절대** 속도 (카메라 프레임 좌표로 표현).
팔로워 자기 속도 v_ego 는 예측의 알려진 입력이다: p' = p + (v − v_ego)·dt.

왜 절대 속도인가 — 상대 속도를 상태로 두면 팔로워가 잘 따라갈수록 리더의 기동이 상대상태에서 상쇄되어
CV 와 CT 의 예측이 같아지고 모드 확률이 움직이지 않는다(2026-09-19 측정: 선회율 0.35~1.0 rad/s 에서 p_ct
0.325~0.329). 절대 속도는 팔로워가 무엇을 하든 리더가 돌면 돈다. 자기 속도가 없으면(v_ego=0) 예전의 상대
필터와 정확히 같다.

모드 전이는 프레임률과 무관한 체류 시간(MODE_SOJOURN_SEC)으로 준다. 프레임마다 [[0.95,0.05],[0.10,0.90]] 을
곱하던 예전 방식은 30 fps 에서 모드 기억이 0.3~0.6 s 뿐이라 프레임당 우도비가 아무리 쌓여도 정상분포(1/3)로
끌려 돌아갔다 — 절대 속도로 바꿔도 p_ct 가 0.35 를 못 넘었던 이유다.
"""

import numpy as np

from config import CONFIG


# 프로세스 잡음 [m/s²]. CV 는 작게 — CV 가 선회를 잡음으로 흡수하면 CT 와 구분되지 않는다. (config imm.sigma_a_*)
SIGMA_A_CV = float(CONFIG["imm"].get("sigma_a_cv", 0.8))
SIGMA_A_CT = float(CONFIG["imm"].get("sigma_a_ct", 1.2))
# 모드 체류 시간 [s] (CV, CT). 프레임당 전이확률 = dt/체류시간. 정상분포 p_ct = τ_ct/(τ_cv+τ_ct) = 1/3 을 유지한다.
MODE_SOJOURN_SEC = tuple(CONFIG["imm"].get("mode_sojourn_sec", (10.0, 5.0)))
# 2026-09-19 이전 추정기 (분석의 '수정 전' 대조군이 이 값으로 FRF 를 잰다 — SITL 실측과 대조된 골든이라 보존)
LEGACY_TUNING = dict(sigma_a_cv=0.8, sigma_a_ct=1.2, mode_sojourn_sec=None)

SIGMA_XY = CONFIG["imm"]["sigma_xy"]
SIGMA_Z = CONFIG["imm"]["sigma_z"]
MAX_COAST_SEC = CONFIG["imm"]["max_coast_sec"]
# 거리 정보를 담은 측정이 끊긴 뒤 "아직 위치를 안다"고 볼 수 있는 시간.
RANGE_COAST_MAX_SEC = CONFIG["imm"].get("range_coast_max_sec", 2.0)
# omega 를 추정할 최소 리더 속도 [m/s]. 이 아래면 진행 방향각이 잡음이라 기존 omega 를 감쇠시킨다.
OMEGA_MIN_SPEED = CONFIG["imm"].get("omega_min_speed_mps", 0.15)

TRANS_PROB_LEGACY = np.array([[0.95, 0.05], [0.10, 0.90]], dtype=float)   # 프레임당 고정 (예전)
MU0 = np.array([0.7, 0.3], dtype=float)


def transition_matrix(dt, sojourn=MODE_SOJOURN_SEC):
    """체류 시간 기반 프레임당 전이행렬. sojourn=None 이면 예전의 프레임당 고정 행렬."""
    if sojourn is None:
        return TRANS_PROB_LEGACY
    p01 = min(max(float(dt) / max(float(sojourn[0]), 1e-3), 0.0), 0.5)
    p10 = min(max(float(dt) / max(float(sojourn[1]), 1e-3), 0.0), 0.5)
    return np.array([[1.0 - p01, p01], [p10, 1.0 - p10]], dtype=float)

P0_POS = 1.0
P0_VEL = 2.0

H_POS = np.hstack([np.eye(3), np.zeros((3, 3))])
H_VEL = np.hstack([np.zeros((3, 3)), np.eye(3)])
R_DEFAULT = np.diag([SIGMA_XY**2, SIGMA_XY**2, SIGMA_Z**2])


def _make_P0():
    return np.diag([P0_POS] * 3 + [P0_VEL] * 3).astype(float)


def _F_cv(dt):
    F = np.eye(6)
    F[0, 3] = F[1, 4] = F[2, 5] = dt
    return F


def _Q(dt, sigma_a):
    """등가속 백색잡음 모델의 이산 process noise. 세 축 동일, 축 간 독립."""
    q = sigma_a**2
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt3 * dt
    block = np.array([[dt4 / 4, dt3 / 2], [dt3 / 2, dt2]]) * q
    Q = np.zeros((6, 6))
    for i in range(3):
        Q[np.ix_([i, i + 3], [i, i + 3])] += block
    return Q


def _F_ct(dt, omega):
    """표준 coordinated-turn (카메라 x-z 수평면). 위치는 회전하는 속도의 적분, 속도만 omega로 회전."""
    if abs(omega) < 1e-5:
        return _F_cv(dt)
    s = np.sin(omega * dt)
    c = np.cos(omega * dt)
    so = s / omega
    co = (1 - c) / omega
    F = np.eye(6)
    F[0, 3] = so
    F[0, 5] = -co
    F[2, 3] = co
    F[2, 5] = so
    F[3, 3] = c
    F[3, 5] = -s
    F[5, 3] = s
    F[5, 5] = c
    F[1, 4] = dt
    return F


def _bearing_hH(x):
    """bearing 관측 h(x) = [px/pz, py/pz] 와 그 야코비안. pz가 0에 붙으면 None."""
    px, py, pz = x[0], x[1], x[2]
    if pz <= 1e-4:
        return None, None
    h = np.array([px / pz, py / pz], dtype=float)
    H = np.zeros((2, 6), dtype=float)
    H[0, 0] = 1.0 / pz
    H[0, 2] = -px / (pz * pz)
    H[1, 1] = 1.0 / pz
    H[1, 2] = -py / (pz * pz)
    return h, H


def _wrap(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))


class _SingleEKF:
    def __init__(self, model_id, sigma_a=None):
        self.model_id = model_id
        self.sigma_a = float(sigma_a) if sigma_a is not None else (SIGMA_A_CV if model_id == 0 else SIGMA_A_CT)
        self.x = np.zeros(6)
        self.P = _make_P0()
        self._omega = 0.0
        self._prev_heading = None
        self._dt_since_update = 0.0
        # 팔로워 자신의 속도(카메라 프레임) — 예측의 알려진 입력. 상태 v 는 리더 절대 속도라 여기 더할 필요가 없다.
        self.ego_vel = np.zeros(3)

    def reset(self, x, P):
        self.x = x.copy()
        self.P = P.copy()

    def predict(self, dt):
        self._dt_since_update += float(dt)
        if self.model_id == 0:
            F, Q = _F_cv(dt), _Q(dt, self.sigma_a)
        else:
            F, Q = _F_ct(dt, self._omega), _Q(dt, self.sigma_a)
        self.x = F @ self.x
        self.x[:3] -= self.ego_vel * dt        # 상대 위치는 (리더 절대 속도 − 자기 속도) 로 진행한다
        self.P = F @ self.P @ F.T + Q

    def update_position3d(self, z, R=None):
        R = R_DEFAULT if R is None else R
        return self._update(z, H_POS @ self.x, H_POS, R, dim=3)

    def update_velocity3d(self, z_rel, R):
        """상대 속도 측정. 상태는 절대 속도이므로 h(x) = v − v_ego (선형, 오프셋)."""
        return self._update(z_rel, H_VEL @ self.x - self.ego_vel, H_VEL, R, dim=3)

    def update_bearing2d(self, z, R):
        h, H = _bearing_hH(self.x)
        if h is None:
            return 1e-300
        return self._update(z, h, H, R, dim=2)

    def _update(self, z, h, H, R, dim):
        y = z - h
        S = H @ self.P @ H.T + R
        try:
            inv_S = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return 1e-300

        K = self.P @ H.T @ inv_S
        self.x = self.x + K @ y
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        return self._gaussian_likelihood(y, S, inv_S, dim)

    def leader_velocity(self):
        """리더의 절대 속도(카메라 프레임) — 상태 그대로."""
        return self.x[3:6].copy()

    def estimate_omega(self, v_ref):
        """회전율 omega = 리더 진행 방향각의 시간 변화율. 저속에서는 heading 이 잡음이라 추정하지 않는다.

        v_ref 는 **CV 필터의** 절대 속도다 (ImmEkf 가 넘긴다). 자기(CT) 속도로 재면 안 된다 — CT 예측이 속도를
        omega 만큼 돌리고 그 회전을 다시 방향각 변화율로 읽어 omega 가 스스로 커진다(선회 0.5 rad/s 에서 −1.29 로
        발산, 2026-09-19 측정). CV 속도는 측정 보정으로만 돌아가므로 지연은 있어도 되먹임이 없다(−0.53).
        절대 속도로 잰다. 상대 속도로 재면 추종이 정착할수록 0 으로 수렴해 방향각이 잡음이 되고 omega 가
        영원히 0 이라 CT 모델이 CV 와 같아진다.
        """
        v = np.asarray(v_ref, dtype=float)
        vx, vz = v[0], v[2]
        if np.hypot(vx, vz) > OMEGA_MIN_SPEED:
            heading = np.arctan2(vz, vx)
            if self._prev_heading is not None and self._dt_since_update > 1e-3:
                d_heading = _wrap(heading - self._prev_heading)
                omega_raw = np.clip(d_heading / self._dt_since_update, -1.5, 1.5)
                self._omega = 0.7 * self._omega + 0.3 * omega_raw
            self._prev_heading = heading
        else:
            self._prev_heading = None
            self._omega *= 0.9
        self._dt_since_update = 0.0

    @staticmethod
    def _gaussian_likelihood(y, S, inv_S, dim):
        det_S = np.linalg.det(S)
        if det_S <= 0:
            return 1e-300
        exp_term = max(float(-0.5 * y @ inv_S @ y), -500)
        likelihood = np.exp(exp_term) / np.sqrt((2 * np.pi) ** dim * det_S)
        return max(float(likelihood), 1e-300)


class ImmEkf:
    N_MODELS = 2

    def __init__(self, sigma_a_cv=None, sigma_a_ct=None, mode_sojourn_sec=MODE_SOJOURN_SEC):
        """튜닝 인자는 분석용(analysis/stability_margins.py 가 LEGACY_TUNING 으로 예전 추정기를 만든다). 운용은 기본값."""
        self.filters = [_SingleEKF(0, sigma_a_cv), _SingleEKF(1, sigma_a_ct)]
        self.mode_sojourn_sec = mode_sojourn_sec
        self.mu = MU0.copy()
        self.initialized = False
        self.coast_time = 0.0
        # "거리를 마지막으로 관측한 뒤 흐른 시간". bearing-only 업데이트는 방위만 주고 거리는
        # 관측하지 못하는데 coast_time은 거기서도 0이 되므로, 미션의 소실 판정은 이 값을 쓴다.
        self.range_coast_time = 0.0
        # 그중 카메라 깊이(RGB-D)로 거리를 본 뒤 흐른 시간. ESP32 GPS 상대위치도 거리를 담지만
        # 오차가 m 단위라, 그것만으로 3m 추종을 계속하면 안 된다 — 제어기가 이격 거리를 정할 때 쓴다.
        self.vision_range_coast_time = 0.0
        self._fused = None      # get_state() 캐시. 상태가 바뀌는 곳마다 mark_dirty().

    def mark_dirty(self):
        """filters[i].x / P / mu 를 바깥에서 직접 고쳤으면 호출 (leader_telemetry 의 속도 힌트)."""
        self._fused = None

    def init(self, z, source="rgbd"):
        # 초기 절대 속도 = 자기 속도 (상대 속도 0 가정 — 예전과 같은 가정이다)
        v0 = self.filters[0].ego_vel
        x0 = np.array([z[0], z[1], z[2], v0[0], v0[1], v0[2]], dtype=float)
        P0 = _make_P0()
        for f in self.filters:
            f.reset(x0, P0)
        self.mu = MU0.copy()
        self.initialized = True
        self.coast_time = 0.0
        self.range_coast_time = 0.0
        # GPS로 초기화했으면 비전 거리는 아직 본 적이 없다.
        self.vision_range_coast_time = 0.0 if source == "rgbd" else float("inf")
        self._fused = None

    def predict(self, dt):
        if not self.initialized:
            return
        dt = max(float(dt), 1e-4)
        # 매 프레임 누적하고, 거리 측정이 들어올 때만 0으로 되돌린다.
        self.range_coast_time += dt
        self.vision_range_coast_time += dt

        Pi = transition_matrix(dt, self.mode_sojourn_sec)
        mu_pred = Pi.T @ self.mu
        mu_pred = mu_pred / max(mu_pred.sum(), 1e-300)

        mixed = []
        for j in range(self.N_MODELS):
            c_ij = Pi[:, j] * self.mu
            c_ij = c_ij / max(c_ij.sum(), 1e-300)
            x_j = sum(c_ij[i] * self.filters[i].x for i in range(self.N_MODELS))
            P_j = np.zeros((6, 6))
            for i in range(self.N_MODELS):
                dx = self.filters[i].x - x_j
                P_j += c_ij[i] * (self.filters[i].P + np.outer(dx, dx))
            mixed.append((x_j, P_j))

        for f, (x_j, P_j) in zip(self.filters, mixed):
            f.reset(x_j, P_j)
            f.predict(dt)

        self.mu = mu_pred
        self._fused = None

    def update_position3d(self, z, R=None, source="rgbd"):
        """source: "rgbd"(카메라 깊이) 또는 "gps"(ESP32 상대위치)."""
        z = np.asarray(z, dtype=float)
        if not self.initialized:
            self.init(z, source=source)
            return
        self.coast_time = 0.0
        self.range_coast_time = 0.0     # 거리를 담은 유일한 측정 경로
        if source == "rgbd":
            self.vision_range_coast_time = 0.0
        self._update_mode_probs([f.update_position3d(z, R) for f in self.filters])
        self._refresh_omega()

    def _refresh_omega(self):
        self.filters[1].estimate_omega(self.filters[0].leader_velocity())

    def update_velocity3d(self, z, R):
        """상대 속도 측정(ESP32 등)을 정규 칼만 업데이트로 반영.

        coast 타이머는 건드리지 않는다 — 속도 측정은 '거리를 안다'도 '지금 보인다'도 보장하지 않는다.
        소실 판정은 위치를 담은 측정(RGB-D / GPS 상대위치)만 되돌린다.
        """
        if not self.initialized:
            return
        z = np.asarray(z, dtype=float)
        self._update_mode_probs([f.update_velocity3d(z, R) for f in self.filters])
        self._refresh_omega()

    def set_ego_velocity_cam(self, v_cam):
        """팔로워 자기 속도(카메라 프레임)를 주입 — predict 의 입력이자 상대 속도 환산의 기준. None 이면 0.

        None(자기 속도 미수신)이면 필터는 팔로워가 정지한 것으로 보고 상태 v 는 사실상 상대 속도가 된다.
        수신이 돌아오면 다음 갱신들에서 절대 속도로 되돌아온다(속도 P0 가 아니라 갱신으로 — 수 프레임 과도).
        """
        v = np.zeros(3) if v_cam is None else np.asarray(v_cam, dtype=float)[:3]
        if not np.all(np.isfinite(v)):
            v = np.zeros(3)
        for f in self.filters:
            f.ego_vel = v.copy()

    def update_bearing2d(self, z, R):
        if not self.initialized:
            return
        self.coast_time = 0.0
        z = np.asarray(z, dtype=float)
        self._update_mode_probs([f.update_bearing2d(z, R) for f in self.filters])
        self._refresh_omega()

    def _update_mode_probs(self, likelihoods):
        mu = self.mu * np.asarray(likelihoods, dtype=float)
        total = mu.sum()
        if total < 1e-300 or not np.isfinite(total):
            self.mu = np.ones(self.N_MODELS) / self.N_MODELS
        else:
            self.mu = mu / total
        self._fused = None

    def innovation_position3d(self, z, R=None):
        if not self.initialized:
            return np.zeros(3), np.eye(3) * 999
        x, P = self.get_state()
        R = R_DEFAULT if R is None else R
        return z - H_POS @ x, H_POS @ P @ H_POS.T + R

    def innovation_velocity3d(self, z_rel, R):
        if not self.initialized:
            return np.zeros(3), np.eye(3) * 999
        x, P = self.get_state()
        return np.asarray(z_rel, dtype=float) - (H_VEL @ x - self.ego_velocity_cam()), H_VEL @ P @ H_VEL.T + R

    def ego_velocity_cam(self):
        return self.filters[0].ego_vel.copy()

    def leader_velocity(self):
        """융합 리더 절대 속도 (카메라 프레임)."""
        return self.get_state()[0][3:6].copy()

    def relative_velocity(self):
        """융합 상대 속도 (카메라 프레임) = 절대 속도 − 자기 속도. 제어의 D 항과 미션 폴백이 쓴다."""
        return self.get_state()[0][3:6] - self.filters[0].ego_vel

    def innovation_bearing2d(self, z, R):
        if not self.initialized:
            return np.zeros(2), np.eye(2) * 999
        x, P = self.get_state()
        h, H = _bearing_hH(x)
        if h is None:
            return np.array([999.0, 999.0]), np.eye(2) * 999
        return z - h, H @ P @ H.T + R

    def on_lost(self, dt):
        self.coast_time += float(dt)

    def compensate_ego_rotation(self, T):
        """팔로워 자세(roll/pitch/yaw)가 바뀌어 카메라 프레임이 돌았을 때 상대상태를 역회전.

        T: 3x3, 이전 카메라 프레임 좌표 → 현재 카메라 프레임 좌표. 위치·속도에 같은 회전을 적용하고
        P 도 함께 돌린다(ω×r 항은 무시 — 프레임 간격 33ms 에서 mm 수준). CT 필터의 직전 진행 방향각도
        같이 옮긴다 — 안 옮기면 팔로워 자신의 회전이 omega 로 샌다.
        """
        if not self.initialized:
            return
        T = np.asarray(T, dtype=float)
        if np.abs(T - np.eye(3)).max() < 1e-9:
            return
        G = np.zeros((6, 6))
        G[:3, :3] = T
        G[3:, 3:] = T
        cv, ct = self.filters[0], self.filters[1]
        v0 = cv.leader_velocity()                    # CT 의 직전 방향각은 CV 속도 기준이다 (estimate_omega)
        h0 = np.arctan2(v0[2], v0[0]) if ct._prev_heading is not None else None
        for f in self.filters:
            f.x = G @ f.x
            f.P = G @ f.P @ G.T
            f.ego_vel = T @ f.ego_vel
        if h0 is not None:
            # 진행 방향각(x-z 평면 투영)이 이 회전으로 얼마나 변했는지를 직전 방향각에도 더한다.
            # 순수 yaw 면 정확히 dpsi. roll/pitch 로 y 성분이 섞이면 투영각 변화가 dpsi 와 달라
            # 단위벡터를 돌리는 방식으로는 omega 가 샌다.
            v1 = cv.leader_velocity()
            ct._prev_heading = _wrap(ct._prev_heading + _wrap(np.arctan2(v1[2], v1[0]) - h0))
        self._fused = None

    def compensate_ego_yaw(self, dpsi):
        """yaw 만 바뀐 경우(우회전 +). 타겟은 카메라 프레임에서 왼쪽으로: [x'; z'] = [[c,-s],[s,c]] @ [x; z]."""
        if abs(float(dpsi)) < 1e-6:
            return
        c, s = np.cos(dpsi), np.sin(dpsi)
        self.compensate_ego_rotation([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])

    def get_state(self):
        if not self.initialized:
            return np.zeros(6), np.eye(6) * 999
        if self._fused is None:
            x = sum(self.mu[i] * self.filters[i].x for i in range(self.N_MODELS))
            P = np.zeros((6, 6))
            for i in range(self.N_MODELS):
                dx = self.filters[i].x - x
                P += self.mu[i] * (self.filters[i].P + np.outer(dx, dx))
            self._fused = (x, P)
        return self._fused

    def get_state_dict(self):
        x, P = self.get_state()
        return {
            "x": x,
            "P": P,
            "mode_probs": self.mu.copy(),
            "initialized": self.initialized,
            "coast_time": self.coast_time,
            "range_coast_time": self.range_coast_time,
            "vision_range_coast_time": self.vision_range_coast_time,
        }

    def get_model_probs(self):
        return self.mu.copy()

    def is_reliable(self):
        return self.initialized and (self.coast_time <= MAX_COAST_SEC)

    def has_range_fix(self, max_age=None):
        """거리를 아직 안다고 볼 수 있는가 (RGB-D 또는 ESP32 GPS). 미션의 소실 판정용."""
        limit = RANGE_COAST_MAX_SEC if max_age is None else float(max_age)
        return self.initialized and (self.range_coast_time <= limit)

    def has_vision_range_fix(self, max_age=None):
        """카메라 깊이로 거리를 본 지 오래되지 않았는가. 추종 이격 거리를 정할 때 쓴다 —
        GPS 링크가 살아 있으면 착륙할 이유는 없지만, GPS만으로 3m 이격은 GPS 오차보다 작다."""
        limit = RANGE_COAST_MAX_SEC if max_age is None else float(max_age)
        return self.initialized and (self.vision_range_coast_time <= limit)
