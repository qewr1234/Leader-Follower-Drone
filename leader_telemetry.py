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
- leader lat/lon: WGS84 degree
- leader alt: meter. 기준계는 필드 이름으로 정한다 —
    "alt_msl" / "alt_amsl"              → 해발(AMSL)
    "alt_ellipsoid" / "alt_hae" / "alt_wgs84" → WGS84 타원체고
    "alt" / "altitude" / "alt_m"         → 수신기의 default_alt_frame (기본 AMSL)
  팔로워 쪽은 같은 기준으로 뺀다: AMSL이면 GLOBAL_POSITION_INT.alt, 타원체고면
  GPS_RAW_INT.alt_ellipsoid. 기준이 다르면 국내 지오이드 차이(~25m)가 상대 고도로 들어간다.
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
    yaw: Optional[float]          # rad, 북 기준 우회전 +. 필드가 없으면 None (0 으로 두면 '북쪽' 으로 오해된다)
    rx_time: float
    seq: int = -1
    alt_frame: str = "AMSL"       # "AMSL" | "ELLIPSOID"
    raw: Optional[Dict[str, Any]] = None
    leader_id: str = ""           # 편대: 어느 리더의 패킷인가 (ArduPilot FOLL_SYSID 역할). 없으면 ""
    acc: Optional[tuple] = None   # 편대: 선두 가속도 (vx/vy/vz 와 같은 프레임). FOLLOW_TARGET.acc 대응. 없으면 None
    uwb_range: Optional[float] = None   # 선두 UWB 가 잰 선두↔후미 거리 [m] (GT 전용, 제어 미사용). 없거나 비유한·0 이하면 None


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
        default_alt_frame="AMSL",
        expected_leader_id=None,
        require_leader_id=False,
    ):
        self.kind = kind
        self.port = port
        self.baud = baud
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.timeout = timeout
        self.default_alt_frame = default_alt_frame

        # 편대: 기대 리더 ID. 주면 다른 ID 의 패킷은 버린다(같은 채널에 리더가 둘일 때). require_leader_id 면 ID 없는
        # 패킷도 버린다 — 기본은 호환을 위해 통과.
        self.expected_leader_id = str(expected_leader_id) if expected_leader_id else None
        self.require_leader_id = bool(require_leader_id)
        self.dropped_other_leader = 0
        self.ser = None
        self.sock = None
        self._rx_buf = b""
        self._last_err = None     # 같은 오류가 매 프레임 반복될 때 1회만 출력하기 위해
        self.latest_packet: Optional[LeaderPacket] = None

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

    def _warn_once(self, msg):
        """read_latest() 는 매 프레임 불리므로, 같은 오류(예: USB 빠짐)는 처음 한 번만 찍는다."""
        if msg != self._last_err:
            print(f"[LEADER] {msg}")
            self._last_err = msg

    def _accept(self, pkt: Optional[LeaderPacket]) -> bool:
        """파싱된 패킷을 latest_packet 으로 채택할지. 리더 ID 필터."""
        if pkt is None:
            return False
        if self.expected_leader_id is not None:
            if pkt.leader_id:
                if pkt.leader_id != self.expected_leader_id:
                    self.dropped_other_leader += 1
                    return False
            elif self.require_leader_id:
                self.dropped_other_leader += 1
                return False
        self.latest_packet = pkt
        return True

    def read_latest(self) -> Optional[LeaderPacket]:
        """
        버퍼에 쌓인 패킷을 최대한 비우고 가장 최신 패킷만 반환.
        """
        if self.kind == "serial":
            self._drain_serial()
        elif self.kind == "udp":
            self._drain_udp()

        return self.latest_packet

    # 줄바꿈 없이 이만큼 쌓이면 쓰레기로 보고 앞부분을 버린다 (115200 baud로 수 초 분량).
    _RX_BUF_LIMIT = 65536

    def _drain_serial(self):
        """버퍼에 도착한 바이트를 전부 읽어 완성된 줄만 파싱한다.

        readline()에 1ms timeout을 걸면 전송 도중(115200 baud에서 150바이트 한 줄이
        13ms) 읽기가 시작될 때 잘린 앞토막이 돌아와 JSON 파싱에 실패하고, 뒤토막도
        다음 호출에서 따로 잘려 그 패킷을 통째로 잃는다. 바이트를 모아 두고 '\n'이
        보일 때만 자르면 어느 시점에 읽어도 손실이 없다.
        """
        if self.ser is None:
            return

        try:
            n = int(getattr(self.ser, "in_waiting", 0) or 0)
            chunk = self.ser.read(n if n > 0 else 1)
        except Exception as exc:
            self._warn_once(f"serial read error: {exc}")
            return

        if not chunk:
            return
        self._rx_buf += chunk

        while b"\n" in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split(b"\n", 1)
            text = line.decode("utf-8", errors="ignore").strip()
            if not text:
                continue
            self._accept(parse_leader_json(text, default_alt_frame=self.default_alt_frame))

        if len(self._rx_buf) > self._RX_BUF_LIMIT:
            self._rx_buf = self._rx_buf[-4096:]

    def _drain_udp(self):
        if self.sock is None:
            return

        while True:
            try:
                data, _addr = self.sock.recvfrom(4096)
                if not data:
                    break

                text = data.decode("utf-8", errors="ignore").strip()
                self._accept(parse_leader_json(text, default_alt_frame=self.default_alt_frame))

            except BlockingIOError:
                break

            except Exception as exc:
                self._warn_once(f"UDP recv error: {exc}")
                break


# ============================================================
# JSON 파싱
# ============================================================

def _get_any(d, names, default=None):
    for name in names:
        if name in d:
            return d[name]
    return default


# 고도 필드 후보 (우선순위 순). 프레임 None 은 수신기의 default_alt_frame 을 쓴다는 뜻.
_ALT_FIELDS = (
    (("alt_msl", "alt_amsl"), "AMSL"),
    (("alt_ellipsoid", "alt_hae", "alt_wgs84"), "ELLIPSOID"),
    (("alt", "altitude", "alt_m"), None),
)


def _reject_json_constant(name):
    # json.loads 는 기본으로 NaN / Infinity / -Infinity 를 float 로 받아들인다. 속도에 inf 가 들어오면 피드포워드가
    # NaN 으로 고정돼(inf − inf) 상한 전진 명령이 된다 (FLIGHT_SAFETY_CHECKLIST G1). 패킷 자체를 버린다.
    raise ValueError(f"non-finite JSON constant {name}")


# 리더 속도 상한(m/s). 이 위는 단위 오류(cm/s, mm/s)나 쓰레기 값이다 — 패킷을 버린다.
LEADER_SPEED_MAX_MPS = 20.0


def parse_leader_json(text: str, default_alt_frame: str = "AMSL") -> Optional[LeaderPacket]:
    try:
        d = json.loads(text, parse_constant=_reject_json_constant)
        now = time.monotonic()

        # alias 지원
        timestamp = float(_get_any(d, ["timestamp", "time", "t", "ts"], now))

        lat = float(_get_any(d, ["lat", "latitude"]))
        lon = float(_get_any(d, ["lon", "lng", "longitude"]))

        # 고도는 기준계를 함께 정한다. 이름이 명시된 필드가 우선이고,
        # 그냥 "alt"면 수신기 설정(default_alt_frame)을 따른다.
        alt = alt_frame = None
        for keys, frame in _ALT_FIELDS:
            alt = _get_any(d, keys)
            if alt is not None:
                alt_frame = frame or str(default_alt_frame).upper()
                break
        alt = float(alt)
        if alt_frame not in ("AMSL", "ELLIPSOID"):
            raise ValueError(f"unknown alt frame {alt_frame}")

        vx = float(_get_any(d, ["vx", "vel_x", "v_east"], 0.0))
        vy = float(_get_any(d, ["vy", "vel_y", "v_north"], 0.0))
        vz = float(_get_any(d, ["vz", "vel_z", "v_up"], 0.0))

        roll = float(_get_any(d, ["roll", "r"], 0.0))
        pitch = float(_get_any(d, ["pitch", "p"], 0.0))
        _yaw = _get_any(d, ["yaw", "y", "heading"], None)
        yaw = None if _yaw is None else float(_yaw)

        # 1e999 같은 리터럴은 json 이 inf 로 만든다 — 수치 필드는 전부 유한해야 하고 속도는 상한 안이어야 한다.
        _nums = [timestamp, lat, lon, alt, vx, vy, vz, roll, pitch] + ([] if yaw is None else [yaw])
        if not all(math.isfinite(v) for v in _nums) or math.sqrt(vx * vx + vy * vy + vz * vz) > LEADER_SPEED_MAX_MPS:
            return None

        seq = int(_get_any(d, ["seq", "packet_seq"], -1))
        leader_id = _get_any(d, ["leader_id", "id", "sysid", "system_id"], "")
        leader_id = "" if leader_id is None else str(leader_id)
        _acc = [_get_any(d, keys, None) for keys in (("ax", "acc_x"), ("ay", "acc_y"), ("az", "acc_z"))]
        acc = None if any(a is None for a in _acc) else tuple(float(a) for a in _acc)
        _uwb = _get_any(d, ["uwb_range", "uwb", "range_uwb", "uwb_m"], None)
        try:
            uwb_range = None if _uwb is None else float(_uwb)
        except (TypeError, ValueError):
            uwb_range = None
        if uwb_range is not None and not (math.isfinite(uwb_range) and uwb_range > 0.0):
            uwb_range = None          # 거리 필드가 나빠도 패킷은 살린다 (GT 는 부가 정보)

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
            alt_frame=alt_frame,
            raw=d,
            leader_id=leader_id,
            acc=acc,
            uwb_range=uwb_range,
        )

    except Exception:
        return None


# ============================================================
# 좌표 변환
# ============================================================

def normalize_lat_lon_alt_from_mavlink(gps_or_global: Dict[str, Any]):
    """
    MAVLink GPS_RAW_INT / GLOBAL_POSITION_INT dict를 degree/meter로 변환.

    두 메시지 모두 lat, lon은 deg * 1e7, alt는 mm 이므로 항상 그 배율로 나눈다.
    (크기로 단위를 추측하지 않는다 — 해발 1m 미만 이륙지에서 alt=800mm 가
    800m 로 남는 문제가 있었다.)
    """
    if not gps_or_global:
        return None

    lat = gps_or_global.get("lat", None)
    lon = gps_or_global.get("lon", None)
    alt = gps_or_global.get("alt", None)

    if lat is None or lon is None or alt is None:
        return None

    return float(lat) * 1e-7, float(lon) * 1e-7, float(alt) * 1e-3


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


def _follower_alt_ellipsoid_m(vehicle_state: Dict[str, Any]):
    """GPS_RAW_INT.alt_ellipsoid (mm, MAVLink2 확장) → m. 없거나 0이면 None."""
    v = vehicle_state.get("gps", {}).get("alt_ellipsoid", None)
    if v is None:
        return None
    try:
        v = float(v)
    except Exception:
        return None
    if v == 0.0:
        return None
    return v * 1e-3


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


def _unavailable(reason, age, fresh=True):
    return {"available": False, "fresh": fresh, "age": age, "reason": reason}


def build_leader_measurement_from_packet(
    packet: Optional[LeaderPacket],
    follower_vehicle_state: Dict[str, Any],
    now: Optional[float] = None,
    max_age_sec=0.7,
    leader_velocity_frame="ENU",
    follower_gps_max_age_sec=None,
    follower_attitude_max_age_sec=None,
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
    now = time.monotonic() if now is None else float(now)

    if packet is None:
        return {"available": False, "reason": "no_leader_packet"}

    age = now - float(packet.rx_time)
    if age > max_age_sec:
        return _unavailable("stale_leader_packet", age, fresh=False)

    # follower GPS는 GPS_RAW_INT보다 GLOBAL_POSITION_INT를 우선 사용
    follower_src = follower_vehicle_state.get("global_position", {})
    follower_lla = normalize_lat_lon_alt_from_mavlink(follower_src)

    if follower_lla is None:
        follower_src = follower_vehicle_state.get("gps", {})
        follower_lla = normalize_lat_lon_alt_from_mavlink(follower_src)

    if follower_lla is None:
        return _unavailable("no_follower_gps", age)

    # 팔로워 위치가 오래됐으면 상대위치도 그만큼 틀린다(팔로워가 그 사이 움직인 만큼). 스트림이 죽었을 때 조용히
    # 틀린 값을 쓰지 않도록 게이트. None 이면 검사하지 않는다(기존 동작).
    if follower_gps_max_age_sec is not None:
        _ts = follower_src.get("timestamp", None)
        if _ts is not None and (now - float(_ts)) > float(follower_gps_max_age_sec):
            return _unavailable("stale_follower_gps", age)

    # 리더가 타원체고를 보내면 팔로워도 타원체고(GPS_RAW_INT.alt_ellipsoid)로 뺀다.
    # 해발과 타원체고를 섞으면 지오이드 차이가 그대로 상대 고도가 된다.
    if packet.alt_frame == "ELLIPSOID":
        f_alt_ell = _follower_alt_ellipsoid_m(follower_vehicle_state)
        if f_alt_ell is None:
            return _unavailable("no_follower_ellipsoid_alt", age)
        follower_lla = (follower_lla[0], follower_lla[1], f_alt_ell)

    # 상대위치는 팔로워 yaw 로 회전한다 — yaw 가 오래됐으면(ATTITUDE 정체) 회전이 틀리고 그 값이 EKF 에 gps 관측으로
    # 들어간다. 자세가 있으면 자세 시각으로, 없으면(hdg 폴백) 위 GPS 신선도 게이트가 이미 걸러 준다.
    if follower_attitude_max_age_sec is not None:
        _att = follower_vehicle_state.get("attitude", {})
        _att_ts = _att.get("timestamp", None)
        if _att.get("yaw") is not None and _att_ts is not None and (now - float(_att_ts)) > float(follower_attitude_max_age_sec):
            return _unavailable("stale_follower_attitude", age)

    follower_yaw = get_follower_yaw(follower_vehicle_state)
    if follower_yaw is None:
        return _unavailable("no_follower_yaw", age)

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
        "leader_alt_frame": packet.alt_frame,
        "leader_vz_up": leader_vz_up,
        "leader_hspeed": leader_hspeed,

        "roll": float(packet.roll),
        "pitch": float(packet.pitch),
        "yaw": (None if packet.yaw is None else float(packet.yaw)),   # 편대: 리더 heading (없으면 None)
        "leader_id": str(getattr(packet, "leader_id", "") or ""),
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
    # 크기 상한: 단위 오류(cm/s)나 쓰레기 값이 힌트로 들어오면 IMM 속도가 끌려가 카메라 관측이 게이트 밖으로 밀리고
    # (교정 불가) 팔로워가 MAX_VX 로 전진한다. 리더 속도 상한 + 자기 속도 여유.
    if float(np.linalg.norm(rel_vel_cam[:3])) > LEADER_SPEED_MAX_MPS:
        return False

    hint = getattr(ekf, "apply_velocity_hint", None)
    if callable(hint):          # C++ 코어(mars_core.ImmEkf): 필터 내부를 밖에서 만지지 않고 메서드로
        try:
            return bool(hint(rel_vel_cam[:3], float(alpha), float(shrink_vel_cov)))
        except Exception:
            return False

    try:
        for f in ekf.filters:
            f.x[3:6] = (1.0 - alpha) * f.x[3:6] + alpha * rel_vel_cam[:3]
            f.P[3:6, 3:6] *= float(shrink_vel_cov)
        ekf.mark_dirty()        # get_state() 캐시 무효화 — 상태를 직접 고쳤다
        return True

    except Exception:
        return False
