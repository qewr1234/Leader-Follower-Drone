"""
leader_telemetry.py — ESP32 leader telemetry receiver + relative measurement builder

선두 ESP32에서 아래 값이 온다고 가정한다.

필수: lat, lon, alt (또는 alt_msl/alt_amsl = AMSL, alt_ellipsoid/alt_hae/alt_wgs84 = 타원체고; 그냥 alt/altitude/alt_m 이면
수신기 default_alt_frame). 셋 중 하나라도 없으면 패킷은 버려진다(parse_leader_json → None).
선택(없으면 0.0): vx, vy, vz(ENU m/s), roll, pitch, yaw. timestamp 없으면 수신 시각, seq 없으면 -1.
주의: vx/vy/vz 가 빠지면 리더 속도 0 으로 처리돼 (1) 상대속도 측정이 −자기속도 로 EKF 에 들어가고
(2) 미션이 ESP32 절대속도 0 을 최우선으로 읽어 선두를 "정지" 로 판정한다. 실제 운용 펌웨어는 vx/vy/vz 를 반드시 보낼 것.

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
    yaw: float
    rx_time: float
    seq: int = -1
    alt_frame: str = "AMSL"       # "AMSL" | "ELLIPSOID"
    has_velocity: bool = True     # vx/vy/vz 가 패킷에 있었는가. False 면 vx/vy/vz 는 0 이고 속도는 쓰면 안 된다.


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
    ):
        self.kind = kind
        self.port = port
        self.baud = baud
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.timeout = timeout
        self.default_alt_frame = default_alt_frame

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
            pkt = parse_leader_json(text, default_alt_frame=self.default_alt_frame)
            if pkt is not None:
                self.latest_packet = pkt

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
                pkt = parse_leader_json(text, default_alt_frame=self.default_alt_frame)
                if pkt is not None:
                    self.latest_packet = pkt

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


def parse_leader_json(text: str, default_alt_frame: str = "AMSL") -> Optional[LeaderPacket]:
    try:
        d = json.loads(text)
        now = time.monotonic()      # rx_time 은 main 의 루프 시계(단조)와 같은 시계여야 age 계산이 맞는다

        # alias 지원. timestamp 는 리더 쪽 시각(로그용)이라 없으면 벽시계.
        timestamp = float(_get_any(d, ["timestamp", "time", "t", "ts"], time.time()))

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

        # 속도 세 키가 모두 없으면 '속도 없음' 으로 표시한다. 0 으로 채우면 미션이 ESP32 속도를 최우선으로 읽어
        # 리더를 '정지' 로 판정해 FOLLOW 에 들어가지 못한다(2026-10-01 감사 9번).
        v_raw = [_get_any(d, k) for k in (["vx", "vel_x", "v_east"], ["vy", "vel_y", "v_north"], ["vz", "vel_z", "v_up"])]
        has_velocity = any(v is not None for v in v_raw)
        vx, vy, vz = (float(v) if v is not None else 0.0 for v in v_raw)

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
            alt_frame=alt_frame,
            has_velocity=has_velocity,
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
    """FRU [front, right, up] → camera XYZ [right, down, forward]. main.camera_xyz_to_fru 의 역변환."""
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


def _unavailable(reason, age):
    return {"available": False, "age": age, "reason": reason}


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
        return _unavailable("stale_leader_packet", age)

    # follower GPS는 GPS_RAW_INT보다 GLOBAL_POSITION_INT를 우선 사용
    follower_lla = normalize_lat_lon_alt_from_mavlink(
        follower_vehicle_state.get("global_position", {})
    )

    if follower_lla is None:
        follower_lla = normalize_lat_lon_alt_from_mavlink(
            follower_vehicle_state.get("gps", {})
        )

    if follower_lla is None:
        return _unavailable("no_follower_gps", age)

    # 리더가 타원체고를 보내면 팔로워도 타원체고(GPS_RAW_INT.alt_ellipsoid)로 뺀다.
    # 해발과 타원체고를 섞으면 지오이드 차이가 그대로 상대 고도가 된다.
    if packet.alt_frame == "ELLIPSOID":
        f_alt_ell = _follower_alt_ellipsoid_m(follower_vehicle_state)
        if f_alt_ell is None:
            return _unavailable("no_follower_ellipsoid_alt", age)
        follower_lla = (follower_lla[0], follower_lla[1], f_alt_ell)

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

    # leader velocity. 패킷에 속도가 없으면 속도 관련 값은 전부 None — main 은 미션을 EKF 속도로 돌리고 속도 측정 갱신을 건너뛴다.
    leader_vel_enu = rel_vel_enu = rel_vel_fru = rel_vel_cam = None
    leader_hspeed = leader_vz_up = None
    follower_vel_enu = _follower_vel_enu_from_mavlink(follower_vehicle_state)
    if getattr(packet, "has_velocity", True):
        if leader_velocity_frame.upper() == "ENU":
            leader_vel_enu = np.array([packet.vx, packet.vy, packet.vz], dtype=float)

        elif leader_velocity_frame.upper() == "NED":
            # NED [north, east, down] → ENU [east, north, up]
            n, e, d = packet.vx, packet.vy, packet.vz
            leader_vel_enu = np.array([e, n, -d], dtype=float)

        else:
            raise ValueError("leader_velocity_frame must be 'ENU' or 'NED'")

        # 상대속도 = 선두 속도 − 후미 속도. EKF 상태 속도는 리더 절대 속도지만 속도 측정의 관측 모델이
        # h(x) = v − v_ego (ImmEkf.update_velocity3d)라 측정은 상대속도로 넘긴다(힌트가 아니라 게이트 있는 정규 측정).
        # 후미 속도를 모르면 아래 폴백에서 리더 절대 속도가 그대로 들어가 자기 속도만큼 편향된다.
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
        "yaw": float(packet.yaw),
        "timestamp": float(packet.timestamp),
        "rx_time": float(packet.rx_time),
        "seq": int(packet.seq),
    }


# ============================================================
# IMM-EKF velocity hint
# ============================================================

def apply_leader_velocity_update_to_imm(ekf, rel_vel_cam, rel, r_vel):
    """ESP32 상대 속도를 IMM-EKF 의 정규 속도 측정 갱신으로 반영한다. (gate_ok, d2) 반환.

    예전 구현(apply_leader_velocity_hint_to_imm)은 필터 상태에 직접 대입하고 공분산을 무조건 줄였다.
    혁신도 마할라노비스 게이트도 우도도 없어서, 잘못된 패킷 하나가 추정을 오염시키면서 동시에
    "더 확신한다"고 공분산까지 줄이는 구조였다. 지금은 다른 측정과 같은 경로를 탄다 —
    R 은 신뢰도로 적응하고, 게이트를 통과한 것만 반영하며, 우도가 모드 확률에 들어간다.

    rel_vel_cam: 카메라 프레임 상대 속도 [VX_right, VY_down, VZ_forward]
    rel:         ReliabilityEstimator
    r_vel:       이 측정의 신뢰도 (0~1). 패킷 신선도 기반.
    """
    if ekf is None or not getattr(ekf, "initialized", False):
        return False, None

    z = np.asarray(rel_vel_cam, dtype=float)
    if z.size < 3 or not np.all(np.isfinite(z[:3])):
        return False, None
    z = z[:3]

    try:
        R = rel.make_R_velocity(r_vel)
        gate_ok, d2 = rel.gate_velocity3d(ekf, z, R)
        if gate_ok:
            ekf.update_velocity3d(z, R)
        return bool(gate_ok), d2
    except Exception:
        return False, None
