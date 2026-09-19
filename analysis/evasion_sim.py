#!/usr/bin/env python3
"""근접 회피 폐루프 모의 — 실제 main.* 제어 함수와 실제 ImmEkf 로 '어떤 접근을 피할 수 있는가' 를 잰다.

    python3 analysis/evasion_sim.py            # 표 출력

리더는 SETTLE 초 동안 0.3 m/s 로 멀어져 추종을 정착시킨 뒤 팔로워 쪽으로 다가온다.
  straight : 그 순간의 시선 방향으로 곧장 달린다 (조종 실수·관성 — 리더가 팔로워를 '지나간다')
  pursuit  : 매 스텝 팔로워를 다시 겨눈다 (순수추격 — 적대적, 팔로워가 어디로 가든 따라온다)

순수추격은 추격자가 회피자보다 빠르면 개활지에서 반드시 포획한다. 팔로워가 기수를 리더에 고정한 채 낼 수 있는
최대 속력은 hypot(MAX_VX, MAX_VY) 이고, 그 위의 접근 속도는 어떤 제어기로도 피할 수 없다 — 이 스크립트는
그 경계를 실측하고 test_fixes.py 의 `회피 경계:` 검사가 고정한다 (요구도 FCR-17).

가정: 수평면 2D, FC 속도루프 1차 지연 TAU_FC, 10 Hz setpoint ZOH, 카메라 측정 잡음 없음, 미션 상태머신 없음
(제어기는 항상 켜져 있다 — 회피 자체의 능력만 본다).
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

from analysis.stability_margins import main  # noqa: E402  (cv2/pyrealsense2/pymavlink 스텁을 설치한 뒤의 main)
from imm_ekf import ImmEkf  # noqa: E402

FPS = 30.0
SETPOINT_HZ = 10.0
TAU_FC = 0.30       # ArduCopter 속도루프 등가 1차 지연 (docs/STABILITY_MARGINS.md 가정과 같다)
TAU_YAW = 0.20
SETTLE = 10.0       # 리더가 멀어지며 추종을 정착시키는 시간
CONTACT_M = 0.15    # 이 아래면 접촉으로 본다 (sitl/harness.py SEP_HARD_FLOOR_M 과 같다)


def escape_speed_bound():
    """기수를 리더에 고정한 팔로워가 낼 수 있는 최대 속력 = 후퇴(MAX_VX) ⊕ 측면(MAX_VY). 순수추격 회피의 이론 상한."""
    return math.hypot(main.MAX_VX, main.MAX_VY)


def simulate(approach_speed, mode="straight", T=30.0, start_dist=4.5, fps=FPS):
    """approach_speed [m/s] 로 다가오는 리더에 대한 최소 거리와 회피 응답.

    반환 dict: min_dist, contact, max_cmd_lat(회피 반경 안에서 '명령한' 측면 속도), max_body_lat(기체 측면 속력),
              t_min(최소 거리 시각), lost(리더가 카메라 뒤로 넘어감).
    """
    dt = 1.0 / fps
    send_every = int(round(fps / SETPOINT_HZ))
    # 월드: 팔로워 (x, y, ψ), 리더 (x, y). 팔로워 기수 ψ=0 이 +x.
    fx, fy, psi = 0.0, 0.0, 0.0
    lx, ly = start_dist, 0.0
    v_body = np.zeros(2)          # 팔로워 실제 속도 (body: front, right)
    yaw_rate = 0.0
    sent = np.zeros(4)            # FC 가 들고 있는 setpoint (vx, vy, vz, yaw_rate)
    cmd = np.zeros(4)
    ff = np.zeros(3)
    ekf = ImmEkf()
    main.reset_evade_side()
    charge_dir = None
    out = {"min_dist": float("inf"), "t_min": None, "max_cmd_lat": 0.0, "max_body_lat": 0.0, "lost": False}

    for k in range(int(T / dt)):
        t = k * dt
        # ---- 리더
        if t < SETTLE:
            lvx, lvy = 0.3, 0.0
        else:
            if mode == "pursuit" or charge_dir is None:
                dx, dy = fx - lx, fy - ly
                h = math.hypot(dx, dy)
                d_ = (dx / h, dy / h) if h > 1e-6 else (-1.0, 0.0)
                if charge_dir is None:
                    charge_dir = d_
                if mode == "pursuit":
                    charge_dir = d_
            lvx, lvy = approach_speed * charge_dir[0], approach_speed * charge_dir[1]
        lx += lvx * dt
        ly += lvy * dt

        # ---- 인지: 상대 위치를 body(FRU) → 카메라 좌표로
        c, s = math.cos(psi), math.sin(psi)
        dxw, dyw = lx - fx, ly - fy
        front = dxw * c + dyw * s
        right = -dxw * s + dyw * c
        dist = math.hypot(front, right)
        if dist < out["min_dist"]:
            out["min_dist"], out["t_min"] = dist, t
        if front <= 0.05:
            out["lost"] = True
            break
        v_self_fru = np.array([v_body[0], v_body[1], 0.0])
        if ekf.initialized:
            ekf.compensate_ego_yaw(yaw_rate * dt)
            ekf.set_ego_velocity_cam(main.fru_to_camera_xyz(v_self_fru))
            ekf.predict(dt)
        ekf.update_position3d(main.fru_to_camera_xyz([front, right, 0.0]))
        x_est, P_est = ekf.get_state()
        rel_fru = main.camera_xyz_to_fru(x_est[:3])
        rel_vel_fru = main.camera_xyz_to_fru(ekf.relative_velocity()) if hasattr(ekf, "relative_velocity") \
            else main.camera_xyz_to_fru(x_est[3:6])
        v_leader_fru = main.camera_xyz_to_fru(ekf.leader_velocity()) if hasattr(ekf, "leader_velocity") \
            else v_self_fru + rel_vel_fru

        # ---- 제어 (실제 main 함수)
        ff = main.leader_velocity_ff(ff, v_leader_fru, dt)
        u = main.compute_velocity_cmd_from_estimate(rel_fru, rel_vel_fru, float(np.trace(P_est[:3, :3])), None, ff,
                                                    v_self_fru=v_self_fru)
        cmd = main.smooth_velocity_cmd(cmd, u, dt=dt)
        if k % send_every == 0:
            sent = cmd.copy()
        if dist < main.EVADE_RADIUS_M:
            out["max_cmd_lat"] = max(out["max_cmd_lat"], abs(float(sent[1])))
            out["max_body_lat"] = max(out["max_body_lat"], abs(float(v_body[1])))

        # ---- 기체: body 속도 1차 지연, yaw 1차 지연, 월드 적분
        v_body += (sent[:2] - v_body) * (dt / TAU_FC)
        yaw_rate += (float(sent[3]) - yaw_rate) * (dt / TAU_YAW)
        fx += (v_body[0] * c - v_body[1] * s) * dt
        fy += (v_body[0] * s + v_body[1] * c) * dt
        psi += yaw_rate * dt

    out["contact"] = bool(out["min_dist"] < CONTACT_M)
    return out


def table(speeds=(0.3, 0.4, 0.5, 0.6, 0.7, 1.0)):
    rows = []
    for mode in ("straight", "pursuit"):
        for v in speeds:
            r = simulate(v, mode=mode)
            rows.append((mode, v, r))
    return rows


if __name__ == "__main__":
    print(f"MAX_VX {main.MAX_VX} MAX_VY {main.MAX_VY} → 순수추격 회피 상한 hypot = {escape_speed_bound():.2f} m/s")
    print("| 접근 | 속도 [m/s] | 최소 거리 [m] | 접촉 | 명령 측면 [m/s] | 기체 측면 [m/s] | 시야 뒤로 |")
    print("|---|---|---|---|---|---|---|")
    for mode, v, r in table():
        print(f"| {mode} | {v:.1f} | {r['min_dist']:.2f} | {'예' if r['contact'] else '아니오'} | "
              f"{r['max_cmd_lat']:.2f} | {r['max_body_lat']:.2f} | {'예' if r['lost'] else ''} |")
