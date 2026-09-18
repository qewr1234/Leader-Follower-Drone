"""
imm_ekf.py — IMM-EKF 상태 추정기 (CV + Coordinated-Turn 2모델), adaptive R / bearing update

상태벡터 x = [px, py, pz, vx, vy, vz], 카메라 좌표계 (x=right, y=down, z=forward).
"""

import numpy as np

from config import CONFIG


SIGMA_A_CV = 0.8
SIGMA_A_CT = 1.2

SIGMA_XY = CONFIG["imm"]["sigma_xy"]
SIGMA_Z = CONFIG["imm"]["sigma_z"]
MAX_COAST_SEC = CONFIG["imm"]["max_coast_sec"]
# 거리 정보를 담은 측정이 끊긴 뒤 "아직 위치를 안다"고 볼 수 있는 시간.
RANGE_COAST_MAX_SEC = CONFIG["imm"].get("range_coast_max_sec", 2.0)

TRANS_PROB = np.array([[0.95, 0.05], [0.10, 0.90]], dtype=float)
MU0 = np.array([0.7, 0.3], dtype=float)

P0_POS = 1.0
P0_VEL = 2.0

H_POS = np.hstack([np.eye(3), np.zeros((3, 3))])
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
    def __init__(self, model_id):
        self.model_id = model_id
        self.x = np.zeros(6)
        self.P = _make_P0()
        self._omega = 0.0
        self._prev_heading = None
        self._dt_since_update = 0.0

    def reset(self, x, P):
        self.x = x.copy()
        self.P = P.copy()

    def predict(self, dt):
        self._dt_since_update += float(dt)
        if self.model_id == 0:
            F, Q = _F_cv(dt), _Q(dt, SIGMA_A_CV)
        else:
            F, Q = _F_ct(dt, self._omega), _Q(dt, SIGMA_A_CT)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update_position3d(self, z, R=None):
        R = R_DEFAULT if R is None else R
        return self._update(z, H_POS @ self.x, H_POS, R, dim=3)

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

        if self.model_id == 1:
            self._estimate_omega()

        return self._gaussian_likelihood(y, S, inv_S, dim)

    def _estimate_omega(self):
        """회전율 omega = 진행 방향각의 시간 변화율. 저속에서는 heading이 노이즈라 추정하지 않는다."""
        vx, vz = self.x[3], self.x[5]
        if np.hypot(vx, vz) > 0.3:
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

    def __init__(self):
        self.filters = [_SingleEKF(i) for i in range(self.N_MODELS)]
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
        x0 = np.array([z[0], z[1], z[2], 0.0, 0.0, 0.0], dtype=float)
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

        Pi = TRANS_PROB
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

    def update_bearing2d(self, z, R):
        if not self.initialized:
            return
        self.coast_time = 0.0
        z = np.asarray(z, dtype=float)
        self._update_mode_probs([f.update_bearing2d(z, R) for f in self.filters])

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

    def compensate_ego_yaw(self, dpsi):
        """후미 기체가 dpsi(rad, 우회전 +)만큼 yaw했을 때 카메라 프레임 기준 상대상태를 역회전.

        yaw 우회전 시 타겟은 카메라 프레임에서 왼쪽으로 이동:
            [x'; z'] = [[cos, -sin], [sin, cos]] @ [x; z]   (x=right, z=forward)
        """
        if not self.initialized or abs(float(dpsi)) < 1e-6:
            return
        c, s = np.cos(dpsi), np.sin(dpsi)
        G = np.eye(6)
        G[0, 0] = G[2, 2] = G[3, 3] = G[5, 5] = c
        G[0, 2] = G[3, 5] = -s
        G[2, 0] = G[5, 3] = s
        for f in self.filters:
            f.x = G @ f.x
            f.P = G @ f.P @ G.T
            # CT 필터의 회전율은 진행 방향각의 변화로 추정한다. 속도를 dpsi 만큼 돌렸으니
            # 직전 방향각도 같이 돌려야 팔로워 자신의 yaw rate 가 omega 로 새지 않는다.
            if f._prev_heading is not None:
                f._prev_heading = _wrap(f._prev_heading + dpsi)
        self._fused = None

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
