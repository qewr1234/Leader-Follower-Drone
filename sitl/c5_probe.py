#!/usr/bin/env python3
"""C5의 본질만 직접 잰다: type_mask가 기수 제어권을 FC에 넘기는가.

절차: 이륙 → 기수를 90°로 돌림 → 30초간 "정지" setpoint 송신 → 기수 변화 측정.
  mask 1479 (수정 후, YAW_RATE_IGNORE clear + yaw_rate=0) → "현재 기수 유지"
  mask 3527 (수정 전, YAW_RATE_IGNORE set)               → FC의 WP_YAW_BEHAVIOR가 결정

WP_YAW_BEHAVIOR=3(LOOK_AHEAD)이면 _look_ahead_yaw_rad의 초기값 0(정북)으로 끌려가야 한다.
"""
import sys
import time

from pymavlink import mavutil

MASK = int(sys.argv[1]) if len(sys.argv) > 1 else 1479
WPY = int(sys.argv[2]) if len(sys.argv) > 2 else 3
HOLD_SEC = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
TARGET_HDG = 90.0

m = mavutil.mavlink_connection("udpin:0.0.0.0:14552")
m.wait_heartbeat(timeout=60)
for sid in (mavutil.mavlink.MAV_DATA_STREAM_ALL,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1):
    m.mav.request_data_stream_send(m.target_system, m.target_component, sid, 10, 1)

m.mav.param_set_send(m.target_system, m.target_component, b"WP_YAW_BEHAVIOR",
                     WPY, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
time.sleep(1)
print(f"WP_YAW_BEHAVIOR={WPY}, mask={MASK}")

m.set_mode("GUIDED")
time.sleep(2)
t0 = time.time()
while time.time() - t0 < 120:
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    a = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if a and a.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and a.result == 0:
        break
    time.sleep(3)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 15)
t0 = time.time()
while time.time() - t0 < 90:
    g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if g and g.relative_alt / 1000.0 >= 13.5:
        break
print("이륙 완료")

# 기수를 90°로
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                        TARGET_HDG, 30, 1, 0, 0, 0, 0)
t0 = time.time()
hdg = None
while time.time() - t0 < 30:
    v = m.recv_match(type="VFR_HUD", blocking=True, timeout=3)
    if v:
        hdg = float(v.heading)
        if abs((hdg - TARGET_HDG + 180) % 360 - 180) < 5:
            break
print(f"기수 정렬: {hdg:.0f}°")
start_hdg = hdg

# 정지 setpoint를 지정한 마스크로 계속 송신
print(f"{HOLD_SEC:.0f}초간 정지 setpoint 송신 (mask={MASK}) ...")
t0 = time.time()
last = 0.0
worst = 0.0
while time.time() - t0 < HOLD_SEC:
    now = time.time()
    if now - last >= 0.1:
        m.mav.set_position_target_local_ned_send(
            int(now * 1000) & 0xFFFFFFFF, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED, MASK,
            0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0.0, 0.0)
        last = now
    v = m.recv_match(type="VFR_HUD", blocking=False)
    if v:
        d = abs((float(v.heading) - start_hdg + 180) % 360 - 180)
        worst = max(worst, d)

print(f"결과: 시작 {start_hdg:.0f}° → 최대 편차 {worst:.1f}°")
print("VERDICT:", "기수 유지 (FC에 제어권 안 넘어감)" if worst < 25 else
      f"기수 제어권이 FC로 넘어감 ({worst:.1f}° 회전)")
