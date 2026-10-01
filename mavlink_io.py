"""
mavlink_io.py — Pixhawk MAVLink 통신

FC 모드/arm 상태, 배터리, GPS/attitude/local position 을 모아 get_vehicle_state() 로 준다.
속도 setpoint / 모드 변경 송신은 main.py 에 있다.
"""

import os
import time
from pymavlink import mavutil

last_battery_pct = None
last_battery_voltage = None

# SITL 회귀용: MARS_FC_PORT=udpin:0.0.0.0:14550 python3 main.py
SERIAL_PORT = os.environ.get("MARS_FC_PORT", "/dev/ttyACM0")
SERIAL_BAUD = 115200

# 메시지 타입 → (저장 키, 필드 목록). 필드가 없으면 None 으로 들어간다.
_STATE_FIELDS = {
    # alt_ellipsoid 는 MAVLink2 확장 필드 — 리더 텔레메트리가 타원체고를 보낼 때 같은 기준으로 뺀다.
    "GPS_RAW_INT": ("gps", ("fix_type", "lat", "lon", "alt", "alt_ellipsoid", "eph", "epv", "vel", "cog",
                            "satellites_visible", "h_acc", "v_acc")),
    "GLOBAL_POSITION_INT": ("global_position", ("lat", "lon", "alt", "relative_alt", "vx", "vy", "vz", "hdg")),
    "LOCAL_POSITION_NED": ("local_position", ("x", "y", "z", "vx", "vy", "vz")),
    "ATTITUDE": ("attitude", ("roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed")),
}

# 기체가 아닌 heartbeat 발신자(규격 고정값. 테스트 스텁에 없을 수 있어 기본값을 둔다).
_NON_VEHICLE_TYPES = frozenset(getattr(mavutil.mavlink, n, d) for n, d in (
    ("MAV_TYPE_GCS", 6), ("MAV_TYPE_ONBOARD_CONTROLLER", 18), ("MAV_TYPE_GIMBAL", 26), ("MAV_TYPE_ADSB", 27)))
_AUTOPILOT_INVALID = getattr(mavutil.mavlink, "MAV_AUTOPILOT_INVALID", 8)
_COMP_GIMBAL = getattr(mavutil.mavlink, "MAV_COMP_ID_GIMBAL", 154)


def is_fc_heartbeat(master, msg):
    """FC 본체(autopilot)의 heartbeat 인가.

    pymavlink 2.4.4x 는 heartbeat 로 target_system 만 잠그고 target_component 는 0 으로 둔다. 컴포넌트 ID
    일치를 요구하면 FC heartbeat 를 전부 버려 모드가 '?' 로 남고 setpoint 송신이 막힌다 (SITL 에서 재현).
    그래서 같은 시스템 + 기체형(GCS/짐벌/ADSB/온보드 아님) + autopilot 유효로 판별하고, 처음 받아들인
    컴포넌트를 master.target_component 에 고정해 이후 다른 컴포넌트(카메라·라우터)가 섞이지 않게 한다."""
    if msg.get_srcSystem() != master.target_system:
        return False
    comp = msg.get_srcComponent()
    if master.target_component not in (0, comp) or comp == _COMP_GIMBAL:
        return False
    if getattr(msg, "type", None) in _NON_VEHICLE_TYPES or getattr(msg, "autopilot", None) == _AUTOPILOT_INVALID:
        return False
    if master.target_component == 0:
        master.target_component = comp
        print(f"[FC] autopilot component={comp} 로 고정")
    return True


# 메시지 수신율 (STAT 진단용). 스트림 요청이 안 먹으면 fresh 게이트가 닫혀 피드포워드·자세보정이 조용히 꺼진다.
_RATE_TYPES = {"HEARTBEAT": "HB", "LOCAL_POSITION_NED": "LP", "ATTITUDE": "ATT", "GLOBAL_POSITION_INT": "GP"}
_rate_counts = {}
_rate_t0 = None


def stream_rates_text(now=None):
    """마지막 호출 이후의 타입별 수신율 문자열. 1초마다 STAT 에서 부른다."""
    global _rate_t0
    now = time.monotonic() if now is None else float(now)
    if _rate_t0 is None:
        _rate_t0 = now
        return "rx[Hz] -"
    dt = max(now - _rate_t0, 1e-6)
    text = " ".join(f"{short}={_rate_counts.get(mt, 0) / dt:.0f}" for mt, short in _RATE_TYPES.items())
    _rate_counts.clear()
    _rate_t0 = now
    return f"rx[Hz] {text}"


_vehicle_state = {
    "gps": {},
    "global_position": {},
    "local_position": {},
    "attitude": {},
    "mode": {},
}


# 컴패니언의 MAVLink 신원. pymavlink 기본값(sysid 255 / compid 0)은 GCS 를 사칭하고 compid 0 은 규격상 송신원으로 쓸 수 없다.
# 기체와 같은 sysid + MAV_COMP_ID_ONBOARD_COMPUTER(191) 가 관례(mavros, MAVSDK). sysid 는 FC 의 heartbeat 를 받은 뒤 그 값으로 맞춘다.
# ArduPilot 의 SYSID_ENFORCE 는 0(기본) 이어야 한다 — 1 이면 MAV_GCS_SYSID(255) 가 아닌 발신자의 명령을 버린다 (README 체크리스트).
COMPANION_COMPONENT_ID = getattr(mavutil.mavlink, "MAV_COMP_ID_ONBOARD_COMPUTER", 191)
_MAV_TYPE_ONBOARD = getattr(mavutil.mavlink, "MAV_TYPE_ONBOARD_CONTROLLER", 18)
_MAV_STATE_ACTIVE = getattr(mavutil.mavlink, "MAV_STATE_ACTIVE", 4)
HEARTBEAT_MAX_AGE_SEC = 3.0     # FC heartbeat(1 Hz)가 이보다 오래 안 오면 모드를 '모름' 으로 — 캐시된 GUIDED 를 믿지 않는다


def adopt_identity(master):
    """FC heartbeat 를 받은 뒤: 송신 신원을 (기체 sysid, compid 191) 로. 테스트 스텁처럼 mav 객체가 없으면 건너뛴다."""
    mav = getattr(master, "mav", None)
    if mav is None:
        return
    try:
        mav.srcSystem = int(master.target_system)
        mav.srcComponent = int(COMPANION_COMPONENT_ID)
    except Exception as exc:
        print(f"[FC] 송신 신원 설정 실패: {type(exc).__name__}: {exc}")


def send_heartbeat(master):
    """컴패니언 heartbeat(ONBOARD_CONTROLLER, autopilot INVALID). 1 Hz 로 보낸다. 라우터·GCS 가 컴패니언 생사를 보고,
    FC 가 '이 compid 가 살아 있다' 를 안다. 스텁에 heartbeat_send 가 없으면 조용히 생략."""
    fn = getattr(getattr(master, "mav", None), "heartbeat_send", None)
    if fn is None:
        return False
    try:
        fn(_MAV_TYPE_ONBOARD, _AUTOPILOT_INVALID, 0, 0, _MAV_STATE_ACTIVE)
        return True
    except Exception as exc:
        print(f"[FC] heartbeat 송신 실패: {type(exc).__name__}: {exc}")
        return False


def connect_fc(heartbeat_timeout=10, give_up_after=None):
    """FC 에 연결하고 첫 heartbeat 를 기다린다. give_up_after(초)가 None 이면 영원히(10초마다 알림), 아니면 그 시간 뒤 None."""
    print("[FC] connecting...")
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=SERIAL_BAUD)
    # wait_heartbeat() 는 timeout 없이 영원히 막힌다. 포기하진 않되 알려서 포트/전원 문제를 침묵 속에 묻지 않는다.
    waited = 0
    while master.wait_heartbeat(timeout=heartbeat_timeout) is None:
        waited += heartbeat_timeout
        if give_up_after is not None and waited >= give_up_after:
            try:
                master.close()
            except Exception:
                pass
            return None
        print(f"[FC] heartbeat 대기 중 ({waited}s) — 포트/전원 확인")
    print(f"[FC] connected  sys={master.target_system}  comp={master.target_component}")
    adopt_identity(master)
    request_data_streams(master)
    return master


def reconnect_fc(old_master):
    """FC 링크 예외가 이어질 때 한 번 시도하는 재연결. 성공하면 새 master, 실패하면 None (호출자가 old 를 계속 쓰며 다시 시도)."""
    try:
        if old_master is not None and hasattr(old_master, "close"):
            old_master.close()
    except Exception:
        pass
    try:
        return connect_fc(heartbeat_timeout=1, give_up_after=1)
    except Exception as exc:
        print(f"[FC] 재연결 실패: {type(exc).__name__}: {exc}")
        return None


def request_data_streams(master, rate_hz=10):
    """쓰는 스트림만 요청한다 (ArduPilot). ALL 을 요청하면 RAW_SENS/EXTRA2 까지 매 프레임 파싱하게 된다.
    PX4 는 이 메시지를 무시하고 포트 프로파일 기본 rate 로 보낸다."""
    for stream_id in (mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS):
        master.mav.request_data_stream_send(master.target_system, master.target_component, stream_id, rate_hz, 1)


def _set_battery(pct, mv):
    global last_battery_pct, last_battery_voltage
    if pct is not None and pct >= 0:
        last_battery_pct = int(pct)
    # 규격상 65535(UINT16_MAX)는 '전압 미보고'다.
    if mv is not None and 0 < mv < 65535:
        last_battery_voltage = float(mv) / 1000.0


def drain_messages(master):
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        mt = msg.get_type()
        now = time.monotonic()      # main 의 루프 시계(단조)와 같은 시계 — is_fresh 비교용
        if mt in _RATE_TYPES:
            _rate_counts[mt] = _rate_counts.get(mt, 0) + 1

        if mt == "HEARTBEAT":
            # FC 본체(autopilot 컴포넌트)의 heartbeat 만 쓴다. 시스템 ID 만 맞추면 같은 기체의 다른
            # 컴포넌트(짐벌, 카메라, mavlink-router)의 heartbeat 가 섞여 모드 문자열이 왕복하고,
            # main 의 GUIDED 진입 에지가 매번 발동해 미션이 계속 리셋된다.
            try:
                if is_fc_heartbeat(master, msg):
                    _vehicle_state["mode"] = {
                        "name": mavutil.mode_string_v10(msg),
                        "armed": bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                        "timestamp": now,
                    }
            except Exception:
                pass
        elif mt == "BATTERY_STATUS":
            try:
                _set_battery(msg.battery_remaining, msg.voltages[0])
            except Exception:
                pass
        elif mt == "SYS_STATUS":
            try:
                _set_battery(msg.battery_remaining, msg.voltage_battery)
            except Exception:
                pass
        elif mt in _STATE_FIELDS:
            key, fields = _STATE_FIELDS[mt]
            d = {f: getattr(msg, f, None) for f in fields}
            d["timestamp"] = now
            _vehicle_state[key] = d


def get_vehicle_state():
    return {k: (dict(v) if isinstance(v, dict) else v) for k, v in _vehicle_state.items()}


def battery_text():
    if last_battery_pct is None:
        return "BAT=N/A"
    if last_battery_voltage is None:
        return f"BAT={last_battery_pct}%"
    return f"BAT={last_battery_pct}%  {last_battery_voltage:.2f}V"
