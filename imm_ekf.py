"""
imm_ekf.py — IMM-EKF 상태 추정기, adaptive R / bearing update 지원

상태벡터: x = [px, py, pz, vx, vy, vz]
카메라 좌표계 기준.
"""

import numpy as np

from config import CONFIG
from utils_geometry import pixel_to_camera as pixel_to_3d


SIGMA_A_CV = 0.8
SIGMA_A_CT = 1.2
SIGMA_W_CT = 0.3

SIGMA_XY = CONFIG["imm"]["sigma_xy"]
SIGMA_Z = CONFIG["imm"]["sigma_z"]
MAX_COAST_SEC = CONFIG["imm"]["max_coast_sec"]
# 거리 정보를 담은 측정이 끊긴 뒤 "아직 위치를 안다"고 볼 수 있는 시간.
RANGE_COAST_MAX_SEC = CONFIG["imm"].get("range_coast_max_sec", 2.0)

TRANS_PROB = np.array(
    [
        [0.95, 0.05],
        [0.10, 0.90],
    ],
    dtype=float,
)

P0_POS = 1.0
P0_VEL = 2.0


def _make_P0():
    P = np.zeros((6, 6))
    P[0, 0] = P[1, 1] = P[2, 2] = P0_POS
    P[3, 3] = P[4, 4] = P[5, 5] = P0_VEL
    return P


def _make_R():
    return np.diag([SIGMA_XY**2, SIGMA_XY**2, SIGMA_Z**2])


def _F_cv(dt):
    F = np.eye(6)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt
    return F


def _Q_cv(dt):
    q = SIGMA_A_CV**2
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt3 * dt

    block = np.array(
        [
            [dt4 / 4, dt3 / 2],
            [dt3 / 2, dt2],
        ]
    ) * q

    Q = np.zeros((6, 6))
    for i in range(3):
        Q[np.ix_([i, i + 3], [i, i + 3])] += block

    return Q

def _F_ct(dt, omega):
    if abs(omega) < 1e-5:
        return _F_cv(dt)

    s = np.sin(omega * dt)
    c = np.cos(omega * dt)
    so = s / omega
    co = (1 - c) / omega

    # 표준 coordinated-turn 모델 (카메라 x-z 수평면, x=right, z=forward).
    # 위치는 회전하는 속도의 적분이고, 속도만 omega로 회전한다.
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

def _Q_ct(dt):
    q = SIGMA_A_CT**2
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt3 * dt

    block = np.array(
        [
            [dt4 / 4, dt3 / 2],
            [dt3 / 2, dt2],
        ]
    ) * q

    Q = np.zeros((6, 6))
    for i in range(3):
        Q[np.ix_([i, i + 3], [i, i + 3])] += block

    return Q

class _SingleEKF:
    H_POS = np.array(
        [
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ],
        dtype=float,
    )

    R_DEFAULT = _make_R()

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
            F = _F_cv(dt)
            Q = _Q_cv(dt)
        else:
            F = _F_ct(dt, self._omega)
            Q = _Q_ct(dt)

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update_position3d(self, z, R=None):
        H = self.H_POS
        R = self.R_DEFAULT if R is None else R
        return self._linear_update(z, H, R, likelihood_dim=3)

    def update_bearing2d(self, z, R):
        px, py, pz = self.x[0], self.x[1], self.x[2]

        if pz <= 1e-4:
            return 1e-300

        h = np.array([px / pz, py / pz], dtype=float)

        H = np.zeros((2, 6), dtype=float)
        H[0, 0] = 1.0 / pz
        H[0, 2] = -px / (pz * pz)
        H[1, 1] = 1.0 / pz
        H[1, 2] = -py / (pz * pz)

        return self._nonlinear_update(z, h, H, R, likelihood_dim=2)

    def innovation_position3d(self, z, R=None):
        H = self.H_POS
        R = self.R_DEFAULT if R is None else R

        y = z - H @ self.x
        S = H @ self.P @ H.T + R

        return y, S

    def innovation_bearing2d(self, z, R):
        px, py, pz = self.x[0], self.x[1], self.x[2]

        if pz <= 1e-4:
            return np.array([999.0, 999.0]), np.eye(2) * 999.0

        h = np.array([px / pz, py / pz], dtype=float)

        H = np.zeros((2, 6), dtype=float)
        H[0, 0] = 1.0 / pz
        H[0, 2] = -px / (pz * pz)
        H[1, 1] = 1.0 / pz
        H[1, 2] = -py / (pz * pz)

        y = z - h
        S = H @ self.P @ H.T + R

        return y, S

    def _linear_update(self, z, H, R, likelihood_dim):
        h = H @ self.x
        return self._nonlinear_update(z, h, H, R, likelihood_dim)

    def _nonlinear_update(self, z, h, H, R, likelihood_dim):
        y = z - h
        S = H @ self.P @ H.T + R

        try:
            K = self.P @ H.T @ np.linalg.inv(S)
            self.x = self.x + K @ y

            I = np.eye(6)
            self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T

        except np.linalg.LinAlgError:
            return 1e-300

        if self.model_id == 1:
            # 회전율 omega는 진행 방향각(heading)의 시간 변화율로 추정한다.
            vx, vz = self.x[3], self.x[5]
            speed = np.sqrt(vx**2 + vz**2)

            if speed > 0.3:
                heading = np.arctan2(vz, vx)
                if self._prev_heading is not None and self._dt_since_update > 1e-3:
                    d_heading = np.arctan2(
                        np.sin(heading - self._prev_heading),
                        np.cos(heading - self._prev_heading),
                    )
                    omega_raw = np.clip(d_heading / self._dt_since_update, -1.5, 1.5)
                    self._omega = 0.7 * self._omega + 0.3 * omega_raw
                self._prev_heading = heading
            else:
                # 저속에서는 heading이 노이즈라 추정 신뢰 불가
                self._prev_heading = None
                self._omega *= 0.9

            self._dt_since_update = 0.0

        return self._gaussian_likelihood(y, S, likelihood_dim)

    @staticmethod
    def _gaussian_likelihood(y, S, dim):
        try:
            det_S = np.linalg.det(S)
            if det_S <= 0:
                return 1e-300

            inv_S = np.linalg.inv(S)
            exp_term = float(-0.5 * y @ inv_S @ y)
            exp_term = max(exp_term, -500)

            likelihood = np.exp(exp_term) / np.sqrt((2 * np.pi) ** dim * det_S)
            return max(float(likelihood), 1e-300)

        except np.linalg.LinAlgError:
            return 1e-300


class ImmEkf:
    N_MODELS = 2

    def __init__(self):
        self.filters = [_SingleEKF(i) for i in range(self.N_MODELS)]
        self.mu = np.array([0.7, 0.3], dtype=float)
        self.initialized = False
        self.coast_time = 0.0
        # coast_time과 별도로 "거리를 마지막으로 관측한 뒤 흐른 시간"을 센다.
        # bearing-only 업데이트는 방위만 주고 거리는 관측하지 못하는데,
        # coast_time은 거기서도 0이 되므로 깊이가 죽어도 필터가 "잘 보고 있다"고
        # 믿게 된다. 미션의 소실 판정은 이 값을 써야 한다.
        self.range_coast_time = 0.0

    def init(self, z):
        x0 = np.array([z[0], z[1], z[2], 0.0, 0.0, 0.0], dtype=float)
        P0 = _make_P0()

        for f in self.filters:
            f.reset(x0, P0)

        self.mu = np.array([0.7, 0.3], dtype=float)
        self.initialized = True
        self.coast_time = 0.0
        self.range_coast_time = 0.0

    def predict(self, dt):
        if not self.initialized:
            return

        dt = max(float(dt), 1e-4)
        # 매 프레임 누적하고, 거리 측정이 들어올 때만 0으로 되돌린다.
        self.range_coast_time += dt

        Pi = TRANS_PROB
        mu_pred = Pi.T @ self.mu
        mu_pred = mu_pred / max(mu_pred.sum(), 1e-300)

        x_mix = []
        P_mix = []

        for j in range(self.N_MODELS):
            c_ij = Pi[:, j] * self.mu
            c_ij = c_ij / max(c_ij.sum(), 1e-300)

            x_j = sum(c_ij[i] * self.filters[i].x for i in range(self.N_MODELS))

            P_j = np.zeros((6, 6))
            for i in range(self.N_MODELS):
                dx = self.filters[i].x - x_j
                P_j += c_ij[i] * (self.filters[i].P + np.outer(dx, dx))

            x_mix.append(x_j)
            P_mix.append(P_j)

        for j, f in enumerate(self.filters):
            f.reset(x_mix[j], P_mix[j])
            f.predict(dt)

        self.mu = mu_pred

    def update(self, z, R=None):
        return self.update_position3d(z, R)

    def update_position3d(self, z, R=None):
        z = np.asarray(z, dtype=float)

        if not self.initialized:
            self.init(z)
            return

        self.coast_time = 0.0
        self.range_coast_time = 0.0     # 거리를 담은 유일한 측정 경로

        likelihoods = np.array(
            [f.update_position3d(z, R) for f in self.filters]
        )
        self._update_mode_probs(likelihoods)

    def update_bearing2d(self, z, R):
        if not self.initialized:
            return

        self.coast_time = 0.0
        z = np.asarray(z, dtype=float)

        likelihoods = np.array(
            [f.update_bearing2d(z, R) for f in self.filters]
        )
        self._update_mode_probs(likelihoods)

    def _update_mode_probs(self, likelihoods):
        self.mu = self.mu * likelihoods
        total = self.mu.sum()

        if total < 1e-300 or not np.isfinite(total):
            self.mu = np.ones(self.N_MODELS) / self.N_MODELS
        else:
            self.mu = self.mu / total

    def innovation_position3d(self, z, R=None):
        if not self.initialized:
            return np.zeros(3), np.eye(3) * 999

        x, P = self.get_state()
        H = _SingleEKF.H_POS
        R = _SingleEKF.R_DEFAULT if R is None else R

        y = z - H @ x
        S = H @ P @ H.T + R

        return y, S

    def innovation_bearing2d(self, z, R):
        if not self.initialized:
            return np.zeros(2), np.eye(2) * 999

        x, P = self.get_state()
        px, py, pz = x[0], x[1], x[2]

        if pz <= 1e-4:
            return np.array([999.0, 999.0]), np.eye(2) * 999

        h = np.array([px / pz, py / pz])

        H = np.zeros((2, 6))
        H[0, 0] = 1.0 / pz
        H[0, 2] = -px / (pz * pz)
        H[1, 1] = 1.0 / pz
        H[1, 2] = -py / (pz * pz)

        y = z - h
        S = H @ P @ H.T + R

        return y, S

    def on_lost(self, dt):
        self.coast_time += float(dt)

    def compensate_ego_yaw(self, dpsi):
        """
        후미 기체가 dpsi(rad, 우회전 +)만큼 yaw했을 때
        카메라 프레임 기준 상대상태를 반대로 회전시켜 보정한다.

        yaw 우회전 시 타겟은 카메라 프레임에서 왼쪽으로 이동:
            [x'; z'] = [[cos, -sin], [sin, cos]] @ [x; z]  (x=right, z=forward)
        """
        if not self.initialized or abs(float(dpsi)) < 1e-6:
            return

        c = np.cos(dpsi)
        s = np.sin(dpsi)

        G = np.eye(6)
        G[0, 0] = c
        G[0, 2] = -s
        G[2, 0] = s
        G[2, 2] = c
        G[3, 3] = c
        G[3, 5] = -s
        G[5, 3] = s
        G[5, 5] = c

        for f in self.filters:
            f.x = G @ f.x
            f.P = G @ f.P @ G.T

    def get_state(self):
        if not self.initialized:
            return np.zeros(6), np.eye(6) * 999

        x_fused = sum(self.mu[i] * self.filters[i].x for i in range(self.N_MODELS))

        P_fused = np.zeros((6, 6))
        for i in range(self.N_MODELS):
            dx = self.filters[i].x - x_fused
            P_fused += self.mu[i] * (self.filters[i].P + np.outer(dx, dx))

        return x_fused, P_fused

    def get_state_dict(self):
        x, P = self.get_state()

        return {
            "x": x,
            "P": P,
            "mode_probs": self.mu.copy(),
            "initialized": self.initialized,
            "coast_time": self.coast_time,
            "range_coast_time": self.range_coast_time,
        }

    def get_position(self):
        x, _ = self.get_state()
        return x[:3]

    def get_velocity(self):
        x, _ = self.get_state()
        return x[3:]

    def get_model_probs(self):
        return self.mu.copy()

    def is_coasting(self):
        return self.coast_time > 0.0

    def is_reliable(self):
        return self.initialized and (self.coast_time <= MAX_COAST_SEC)

    def has_range_fix(self, max_age=None):
        """거리를 아직 안다고 볼 수 있는가.

        is_reliable()과 다르다: bearing-only 업데이트는 coast_time을 0으로
        되돌리지만 거리는 관측하지 못한다. 미션의 소실 판정에는 이 쪽을 쓴다.
        """
        limit = RANGE_COAST_MAX_SEC if max_age is None else float(max_age)
        return self.initialized and (self.range_coast_time <= limit)


def get_intrinsics_from_camera(pipeline_profile):
    stream = pipeline_profile.get_stream(__import__("pyrealsense2").stream.color)
    intr = stream.as_video_stream_profile().get_intrinsics()

    return {
        "fx": intr.fx,
        "fy": intr.fy,
        "ppx": intr.ppx,
        "ppy": intr.ppy,
    }
