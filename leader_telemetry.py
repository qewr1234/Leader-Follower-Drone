"""
leader_telemetry.py — ESP32 leader telemetry receiver + relative measurement builder

선두 ESP32에서 아래 값이 온다고 가정한다.

필수 필드:
- lat, lon, alt
- vx, vy, vz
- roll, pitch, yaw
- timestamp

지원 입력 형식:
1) Serial JSON line
   {"timestamp":123.45,"lat":35.123,"lon":128.123,"alt":12.3,
    "vx":0.2,"vy":0.0,"vz":0.1,"roll":0.01,"pitch":-0.02,"yaw":1.57}

2) UDP JSON packet
   위와 동일

좌표계 가정:
- leader lat/lon/alt: WGS84, degree, meter
- leader velocity 기본값: ENU 기준 [east, north, up] m/s
- MAVLink follower attitude yaw: rad, North 기준 clockwise
- MARS-IMM / camera state:
    x = [X_right, Y_down, Z_forward, VX_right, VY_down, VZ_forward]
- MissionManager / controller:
    FRU = [front, right, up]
"""

import json
import math
import socket
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any

import numpy as np


# ============================================================
# 데이터 구조
# ============================================================

@dataclass
class LeaderPacket:
    timestamp: float
    lat: float
    lon: float
    alt: float
    vx: float
    vy: float
    vz: float
    roll: float
    pitch: float
    yaw: float
    rx_time: float
    seq: int = -1
    raw: Optional[Dict[str, Any]] = None


# ============================================================
# ESP32 수신기
# ============================================================

class LeaderTelemetryReceiver:
    """
    ESP32 telemetry 수신기.

    kind="serial":
        LeaderTelemetryReceiver(kind="serial", port="/dev/ttyUSB0", baud=115200)

    kind="udp":
        LeaderTelemetryReceiver(kind="udp", udp_ip="0.0.0.0", udp_port=5005)
    """

    def __init__(
        self,
        kind="serial",
        port="/dev/ttyUSB0",
        baud=115200,
        udp_ip="0.0.0.0",
        udp_port=5005,
        timeout=0.001,
    ):
        self.kind = kind
        self.port = port
        self.baud = baud
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.timeout = timeout

        self.ser = None
        self.sock = None
        self.latest_packet: Optional[LeaderPacket] = None
        self.last_error = None

    def start(self):
        if self.kind == "serial":
            import serial
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            print(f"[LEADER] serial opened: {self.port} @ {self.baud}")

        elif self.kind == "udp":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.udp_ip, self.udp_port))
            self.sock.setblocking(False)
            print(f"[LEADER] UDP listening: {self.udp_ip}:{self.udp_port}")

        else:
            raise ValueError(f"unknown leader telemetry kind: {self.kind}")

    def close(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass

        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass

    def read_latest(self) -> Optional[LeaderPacket]:
        """
        버퍼에 쌓인 패킷을 최대한 비우고 가장 최신 패킷만 반환.
        """
        if self.kind == "serial":
            self._drain_serial()
        elif self.kind == "udp":
            self._drain_udp()

        return self.latest_packet

    def _drain_serial(self):
        if self.ser is None:
            return

        while True:
            try:
                line = self.ser.readline()
                if not line:
                    break

                text = line.decode("utf-8", errors="ignore").strip()
                if not text:
                    continue

                pkt = parse_leader_json(text)
                if pkt is not None:
                    self.latest_packet = pkt

            except Exception as exc:
                self.last_error = str(exc)
                break

    def _drain_udp(self):
        if self.sock is None:
            return

        while True:
            try:
                data, _addr = self.sock.recvfrom(4096)
                if not data:
                    break

                text = data.decode("utf-8", errors="ignore").strip()
                pkt = parse_leader_json(text)
                if pkt is not None:
                    self.latest_packet = pkt

            except BlockingIOError:
                break

            except Exception as exc:
                self.last_error = str(exc)
                break


# ============================================================
# JSON 파싱
# ============================================================

def _get_any(d, names, default=None):
    for name in names:
        if name in d:
            return d[name]
    return default


def parse_leader_json(text: str) -> Optional[LeaderPacket]:
    try:
        d = json.loads(text)
        now = time.time()

        # alias 지원
        timestamp = float(_get_any(d, ["timestamp", "time", "t", "ts"], now))

        lat = float(_get_any(d, ["lat", "latitude"]))
        lon = float(_get_any(d, ["lon", "lng", "longitude"]))
        alt = float(_get_any(d, ["alt", "altitude", "alt_m"]))

        vx = float(_get_any(d, ["vx", "vel_x", "v_east"], 0.0))
        vy = float(_get_any(d, ["vy", "vel_y", "v_north"], 0.0))
        vz = float(_get_any(d, ["vz", "vel_z", "v_up"], 0.0))

        roll = float(_get_any(d, ["roll", "r"], 0.0))
        pitch = float(_get_any(d, ["pitch", "p"], 0.0))
        yaw = float(_get_any(d, ["yaw", "y"], 0.0))

        seq = int(_get_any(d, ["seq", "packet_seq"], -1))

        return LeaderPacket(
            timestamp=timestamp,
            lat=lat,
            lon=lon,
            alt=alt,
            vx=vx,
            vy=vy,
            vz=vz,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            rx_time=now,
            seq=seq,
            raw=d,
        )

    except Exception:
        return None


# ============================================================
# 좌표 변환
# ============================================================

def normalize_lat_lon_alt_from_mavlink(gps_or_global: Dict[str, Any]):
    """
    MAVLink GPS_RAW_INT / GLOBAL_POSITION_INT 값을 degree/meter로 변환.

    GPS_RAW_INT:
        lat, lon: deg * 1e7
        alt: mm

    GLOBAL_POSITION_INT:
        lat, lon: deg * 1e7
        alt: mm
    """
    if not gps_or_global:
        return None

    lat = gps_or_global.get("lat", None)
    lon = gps_or_global.get("lon", None)
    alt = gps_or_global.get("alt", None)

    if lat is None or lon is None or alt is None:
        return None

    lat = float(lat)
    lon = float(lon)
    alt = float(alt)

    # MAVLink integer scale
    if abs(lat) > 1000:
        lat *= 1e-7
    if abs(lon) > 1000:
        lon *= 1e-7
    if abs(alt) > 1000:
        alt *= 1e-3

    return lat, lon, alt


def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """
    짧은 거리용 WGS84 근사 ENU 변환.
    드론 추종 수십 m 수준에서는 충분히 실용적.
    """
    R = 6378137.0

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    d_lat = lat_rad - lat0_rad
    d_lon = lon_rad - lon0_rad

    east = d_lon * math.cos(lat0_rad) * R
    north = d_lat * R
    up = alt - alt0

    return np.array([east, north, up], dtype=float)


def get_follower_yaw(vehicle_state: Dict[str, Any]):
    """
    MAVLink ATTITUDE yaw는 보통 rad.
    없으면 GLOBAL_POSITION_INT hdg를 사용.
    hdg는 cdeg일 수 있음.
    """
    att = vehicle_state.get("attitude", {})
    yaw = att.get("yaw", None)

    if yaw is not None:
        return float(yaw)

    gp = vehicle_state.get("global_position", {})
    hdg = gp.get("hdg", None)

    if hdg is not None and int(hdg) != 65535:
        # MAVLink hdg: centi-degree
        return math.radians(float(hdg) / 100.0)

    return None


def enu_to_body_fru(vec_enu, follower_yaw_rad):
    """
    ENU vector [east, north, up]를 후미 body 기준 FRU로 변환.

    follower_yaw_rad:
        North 기준 clockwise yaw.
        yaw=0이면 forward=N, right=E.
        yaw=90deg이면 forward=E, right=S.

    반환:
        [front, right, up]
    """
    east, north, up = map(float, vec_enu[:3])
    psi = float(follower_yaw_rad)

    # heading unit in ENU
    # forward = [sin(psi), cos(psi)]
    # right   = [cos(psi), -sin(psi)]
    front = east * math.sin(psi) + north * math.cos(psi)
    right = east * math.cos(psi) - north * math.sin(psi)

    return np.array([front, right, up], dtype=float)


def fru_to_camera_xyz(vec_fru):
    """
    FRU [front, right, up] → camera XYZ [right, down, forward]
    """
    front, right, up = map(float, vec_fru[:3])
    return np.array([right, -up, front], dtype=float)


def camera_xyz_to_fru(vec_cam):
    """
    camera XYZ [right, down, forward] → FRU [front, right, up]
    """
    right, down, front = map(float, vec_cam[:3])
    return np.array([front, right, -down], dtype=float)


# ============================================================
# Telemetry → MARS-IMM measurement
# ============================================================

def _follower_vel_enu_from_mavlink(vehicle_state: Dict[str, Any]):
    """
    follower GLOBAL_POSITION_INT 속도를 ENU m/s로 변환.

    GLOBAL_POSITION_INT:
        vx = north [cm/s], vy = east [cm/s], vz = down [cm/s]
    """
    gp = vehicle_state.get("global_position", {})
    vx_n = gp.get("vx", None)
    vy_e = gp.get("vy", None)
    vz_d = gp.get("vz", None)

    if vx_n is None or vy_e is None or vz_d is None:
        return None

    try:
        return np.array(
            [float(vy_e), float(vx_n), -float(vz_d)],
            dtype=float,
        ) / 100.0
    except Exception:
        return None


def build_leader_measurement_from_packet(
    packet: Optional[LeaderPacket],
    follower_vehicle_state: Dict[str, Any],
    now: Optional[float] = None,
    max_age_sec=0.7,
    leader_velocity_frame="ENU",
):
    """
    ESP32 leader packet과 follower Pixhawk state를 이용해
    MARS-IMM에 넣을 상대 위치/속도 measurement를 만든다.

    반환 dict:
        available
        fresh
        age
        rel_enu
        rel_fru
        rel_cam
        rel_vel_enu
        rel_vel_fru
        rel_vel_cam
        leader_alt
        leader_vz_up
        leader_hspeed
        roll/pitch/yaw
    """
    now = time.time() if now is None else float(now)

    if packet is None:
        return {"available": False, "reason": "no_leader_packet"}

    age = now - float(packet.rx_time)
    fresh = age <= max_age_sec

    if not fresh:
        return {
            "available": False,
            "fresh": False,
            "age": age,
            "reason": "stale_leader_packet",
        }

    # follower GPS는 GPS_RAW_INT보다 GLOBAL_POSITION_INT를 우선 사용
    follower_lla = normalize_lat_lon_alt_from_mavlink(
        follower_vehicle_state.get("global_position", {})
    )

    if follower_lla is None:
        follower_lla = normalize_lat_lon_alt_from_mavlink(
            follower_vehicle_state.get("gps", {})
        )

    if follower_lla is None:
        return {
            "available": False,
            "fresh": True,
            "age": age,
            "reason": "no_follower_gps",
        }

    follower_yaw = get_follower_yaw(follower_vehicle_state)
    if follower_yaw is None:
        return {
            "available": False,
            "fresh": True,
            "age": age,
            "reason": "no_follower_yaw",
        }

    f_lat, f_lon, f_alt = follower_lla

    rel_enu = lla_to_enu(
        lat=packet.lat,
        lon=packet.lon,
        alt=packet.alt,
        lat0=f_lat,
        lon0=f_lon,
        alt0=f_alt,
    )

    rel_fru = enu_to_body_fru(rel_enu, follower_yaw)
    rel_cam = fru_to_camera_xyz(rel_fru)

    # leader velocity
    if leader_velocity_frame.upper() == "ENU":
        leader_vel_enu = np.array([packet.vx, packet.vy, packet.vz], dtype=float)

    elif leader_velocity_frame.upper() == "NED":
        # NED [north, east, down] → ENU [east, north, up]
        n, e, d = packet.vx, packet.vy, packet.vz
        leader_vel_enu = np.array([e, n, -d], dtype=float)

    else:
        raise ValueError("leader_velocity_frame must be 'ENU' or 'NED'")

    # 상대속도 = 선두 속도 - 후미 속도.
    # IMM-EKF 상태가 "상대" 위치/속도이므로, 힌트도 상대속도여야
    # 추종 중(둘 다 이동)에 후미 자신의 속도만큼 편향되지 않는다.
    follower_vel_enu = _follower_vel_enu_from_mavlink(follower_vehicle_state)

    if follower_vel_enu is not None:
        rel_vel_enu = leader_vel_enu - follower_vel_enu
    else:
        # follower 속도를 모르면 기존처럼 leader 속도로 fallback
        rel_vel_enu = leader_vel_enu

    rel_vel_fru = enu_to_body_fru(rel_vel_enu, follower_yaw)
    rel_vel_cam = fru_to_camera_xyz(rel_vel_fru)

    # 미션(출발/호버/착륙) 판단용은 선두의 "절대" 속도
    leader_hspeed = float(np.linalg.norm(leader_vel_enu[:2]))
    leader_vz_up = float(leader_vel_enu[2])

    return {
        "available": True,
        "fresh": True,
        "age": float(age),
        "reason": "ok",

        "rel_enu": rel_enu,
        "rel_fru": rel_fru,
        "rel_cam": rel_cam,

        "leader_vel_enu": leader_vel_enu,
        "follower_vel_enu": follower_vel_enu,

        "rel_vel_enu": rel_vel_enu,
        "rel_vel_fru": rel_vel_fru,
        "rel_vel_cam": rel_vel_cam,

        "leader_alt": float(packet.alt),
        "leader_vz_up": leader_vz_up,
        "leader_hspeed": leader_hspeed,

        "roll": float(packet.roll),
        "pitch": float(packet.pitch),
        "yaw": float(packet.yaw),
        "timestamp": float(packet.timestamp),
        "rx_time": float(packet.rx_time),
        "seq": int(packet.seq),
        "raw": packet.raw or {},
    }


# ============================================================
# IMM-EKF velocity hint
# ============================================================

def apply_leader_velocity_hint_to_imm(ekf, rel_vel_cam, alpha=0.12, shrink_vel_cov=0.96):
    """
    ESP32 leader velocity를 IMM-EKF의 velocity state에 약하게 반영.

    깔끔한 방법은 IMM-EKF에 velocity measurement update를 추가하는 것이지만,
    현재 구조를 크게 깨지 않기 위해 weak hint 방식으로 적용한다.

    rel_vel_cam:
        camera state velocity [VX_right, VY_down, VZ_forward]
    """
    if ekf is None or not getattr(ekf, "initialized", False):
        return False

    rel_vel_cam = np.asarray(rel_vel_cam, dtype=float)
    if rel_vel_cam.size < 3 or not np.all(np.isfinite(rel_vel_cam[:3])):
        return False

    try:
        for f in ekf.filters:
            f.x[3:6] = (1.0 - alpha) * f.x[3:6] + alpha * rel_vel_cam[:3]
            f.P[3:6, 3:6] *= float(shrink_vel_cov)
        return True

    except Exception:
        return False
