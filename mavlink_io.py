"""
mavlink_io.py — Pixhawk MAVLink 통신

- FC 모드/arm 상태, 배터리, GPS/attitude/local position 수집
- 모터 테스트(MAV_CMD_DO_MOTOR_TEST) 계열은 velocity setpoint 제어로 넘어가면서 제거했다.
"""

import os
import time
from pymavlink import mavutil

last_battery_pct = None
last_battery_voltage = None

# SITL 회귀용: MARS_FC_PORT=udpin:0.0.0.0:14550 python3 main.py
SERIAL_PORT = os.environ.get("MARS_FC_PORT", "/dev/ttyACM0")
SERIAL_BAUD = 115200

_vehicle_state = {
    "gps": {},
    "global_position": {},
    "local_position": {},
    "attitude": {},
    "mode": {},
    "timestamp": 0.0,
}


def connect_fc():
    print("[FC] connecting...")
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=SERIAL_BAUD)
    master.wait_heartbeat()
    print(f"[FC] connected  sys={master.target_system}  comp={master.target_component}")
    request_data_streams(master)
    return master


def request_data_streams(master, rate_hz=10):
    # ArduPilot/PX4 공통적으로 일부 stream rate 요청
    streams = [
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
    ]
    for stream_id in streams:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            stream_id,
            rate_hz,
            1,
        )


def drain_messages(master):
    global last_battery_pct, last_battery_voltage, _vehicle_state
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        mt = msg.get_type()
        now = time.time()
        _vehicle_state["timestamp"] = now

        if mt == "HEARTBEAT":
            # FC 본체(autopilot 컴포넌트)의 heartbeat만 사용한다. 시스템 ID만 맞추면
            # 같은 기체의 다른 컴포넌트(짐벌, 카메라, mavlink-router 등)의 heartbeat가
            # 섞여 들어와 모드 문자열이 왕복하고, main의 GUIDED 진입 에지가 매번 발동해
            # 미션이 계속 리셋된다.
            try:
                if (
                    msg.get_srcSystem() == master.target_system
                    and msg.get_srcComponent() == master.target_component
                    and msg.type != mavutil.mavlink.MAV_TYPE_GCS
                ):
                    _vehicle_state["mode"] = {
                        "name": mavutil.mode_string_v10(msg),
                        "armed": bool(
                            msg.base_mode
                            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        ),
                        "timestamp": now,
                    }
            except Exception:
                pass

        elif mt == "BATTERY_STATUS":
            try:
                if msg.battery_remaining >= 0:
                    last_battery_pct = int(msg.battery_remaining)
            except Exception:
                pass
            try:
                v0 = msg.voltages[0]
                if v0 not in (None, 65535):
                    last_battery_voltage = float(v0) / 1000.0
            except Exception:
                pass

        elif mt == "SYS_STATUS":
            try:
                if msg.battery_remaining >= 0:
                    last_battery_pct = int(msg.battery_remaining)
            except Exception:
                pass
            try:
                # 규격상 65535(UINT16_MAX)는 "전압 미보고"다. > 0만 보면 65.5V로 표시된다.
                if 0 < msg.voltage_battery < 65535:
                    last_battery_voltage = float(msg.voltage_battery) / 1000.0
            except Exception:
                pass

        elif mt == "GPS_RAW_INT":
            _vehicle_state["gps"] = {
                "fix_type": getattr(msg, "fix_type", None),
                "lat": getattr(msg, "lat", None),
                "lon": getattr(msg, "lon", None),
                "alt": getattr(msg, "alt", None),
                # MAVLink2 확장 필드. 리더 텔레메트리가 타원체고를 보낼 때 같은 기준으로 뺀다.
                "alt_ellipsoid": getattr(msg, "alt_ellipsoid", None),
                "eph": getattr(msg, "eph", None),
                "epv": getattr(msg, "epv", None),
                "vel": getattr(msg, "vel", None),
                "cog": getattr(msg, "cog", None),
                "satellites_visible": getattr(msg, "satellites_visible", None),
                "h_acc": getattr(msg, "h_acc", None),
                "v_acc": getattr(msg, "v_acc", None),
                "timestamp": now,
            }

        elif mt == "GLOBAL_POSITION_INT":
            _vehicle_state["global_position"] = {
                "lat": getattr(msg, "lat", None),
                "lon": getattr(msg, "lon", None),
                "alt": getattr(msg, "alt", None),
                "relative_alt": getattr(msg, "relative_alt", None),
                "vx": getattr(msg, "vx", None),
                "vy": getattr(msg, "vy", None),
                "vz": getattr(msg, "vz", None),
                "hdg": getattr(msg, "hdg", None),
                "timestamp": now,
            }

        elif mt == "LOCAL_POSITION_NED":
            _vehicle_state["local_position"] = {
                "x": getattr(msg, "x", None),
                "y": getattr(msg, "y", None),
                "z": getattr(msg, "z", None),
                "vx": getattr(msg, "vx", None),
                "vy": getattr(msg, "vy", None),
                "vz": getattr(msg, "vz", None),
                "timestamp": now,
            }

        elif mt == "ATTITUDE":
            _vehicle_state["attitude"] = {
                "roll": getattr(msg, "roll", None),
                "pitch": getattr(msg, "pitch", None),
                "yaw": getattr(msg, "yaw", None),
                "rollspeed": getattr(msg, "rollspeed", None),
                "pitchspeed": getattr(msg, "pitchspeed", None),
                "yawspeed": getattr(msg, "yawspeed", None),
                "timestamp": now,
            }


def get_vehicle_state():
    return {
        "gps": dict(_vehicle_state.get("gps", {})),
        "global_position": dict(_vehicle_state.get("global_position", {})),
        "local_position": dict(_vehicle_state.get("local_position", {})),
        "attitude": dict(_vehicle_state.get("attitude", {})),
        "mode": dict(_vehicle_state.get("mode", {})),
        "timestamp": _vehicle_state.get("timestamp", 0.0),
    }


def battery_text():
    if last_battery_pct is None:
        return "BAT=N/A"
    if last_battery_voltage is None:
        return f"BAT={last_battery_pct}%"
    return f"BAT={last_battery_pct}%  {last_battery_voltage:.2f}V"

