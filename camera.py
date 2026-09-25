"""
camera.py — Intel RealSense D435i RGB-D 래퍼 (pyrealsense2)

컬러(BGR8)·깊이(Z16) 스트림을 켜고, 깊이를 컬러 시점에 정렬한 프레임 쌍을
get_frames() 로 돌려준다. main.py 는 .intrinsics / .depth_scale 을 읽어
MeasurementBuilder 에 넘긴다.

IMU(accel/gyro) 스트림은 켜지 않는다 — 읽는 곳이 없고, 켜 두면 IMU 전용
frameset 이 wait_for_frames() 에 섞여 들어와 컬러/깊이 없는 프레임이 생긴다.

rs.align 은 매 프레임 640x480 전체를 재투영한다. pip 휠의 librealsense 는 CPU 구현이라
Jetson 에서 프레임당 수 ms 를 쓴다 — BUILD_WITH_CUDA=ON 으로 직접 빌드하면 GPU 로 간다.
"""

import time

import numpy as np

from config import CONFIG

try:
    import pyrealsense2 as rs
except ImportError as exc:
    rs = None
    _IMPORT_ERROR = exc

# AE 측광 ROI 갱신 최소 간격(초)과, 다시 보낼 만한 중심 이동(프레임 폭 대비). set_region_of_interest 는
# USB 제어 전송이라 매 프레임 부르지 않는다.
AE_ROI_MIN_INTERVAL_SEC = 1.0
AE_ROI_MOVE_FRAC = 0.10
AE_ROI_SCALE = 1.5
AE_ROI_MIN_PX = 32


def _rs_option(name):
    return getattr(getattr(rs, "option", None), name, None)


def _set_option(sensor, name, value):
    """지원하면 범위로 잘라 설정하고 True. 옵션이 없거나(구버전 librealsense) 실패하면 로그만 남기고 False."""
    opt = _rs_option(name)
    try:
        if opt is None or not sensor.supports(opt):
            print(f"[CAM] 옵션 {name} 미지원 — 건너뜀")
            return False
        rng = sensor.get_option_range(opt)
        v = min(max(float(value), float(rng.min)), float(rng.max))
        sensor.set_option(opt, v)
        return True
    except Exception as exc:
        print(f"[CAM] 옵션 {name}={value} 설정 실패: {type(exc).__name__}: {exc}")
        return False


def _exposure_unit_us(sensor):
    """컬러 센서 노출 옵션의 단위(µs). D4xx RGB 는 범위 1~10000 인 100µs 단위, 그 외는 µs 로 본다."""
    opt = _rs_option("exposure")
    try:
        if opt is not None and sensor.supports(opt) and float(sensor.get_option_range(opt).max) <= 10000:
            return 100.0
    except Exception:
        pass
    return 1.0


def apply_color_exposure_options(sensor, cfg):
    """실외용 컬러 AE 설정. 30fps 고정(AE priority off), 노출 상한, 역광 보정. 항목별로 독립 적용."""
    _set_option(sensor, "enable_auto_exposure", 1)
    _set_option(sensor, "auto_exposure_priority", 1 if cfg.get("color_auto_exposure_priority", False) else 0)
    _set_option(sensor, "backlight_compensation", 1 if cfg.get("color_backlight_compensation", True) else 0)
    max_us = float(cfg.get("color_exposure_max_us", 0) or 0)
    if max_us > 0:
        _set_option(sensor, "auto_exposure_limit_toggle", 1)          # 신버전은 토글이 있어야 limit 가 먹는다
        if _set_option(sensor, "auto_exposure_limit", max_us / _exposure_unit_us(sensor)):
            print(f"[CAM] 컬러 AE 노출 상한 {max_us:.0f}µs")


def roi_from_bbox(bbox, width, height, scale=AE_ROI_SCALE, min_px=AE_ROI_MIN_PX):
    """bbox 를 scale 배로 키워 프레임 안으로 자른 (x1, y1, x2, y2). None 이면 전체 프레임."""
    if bbox is None:
        return (0, 0, width - 1, height - 1)
    x1, y1, x2, y2 = bbox
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    hw = max((x2 - x1) * scale / 2.0, min_px / 2.0)
    hh = max((y2 - y1) * scale / 2.0, min_px / 2.0)
    rx1, ry1 = int(max(0, cx - hw)), int(max(0, cy - hh))
    rx2, ry2 = int(min(width - 1, cx + hw)), int(min(height - 1, cy + hh))
    if rx2 - rx1 < min_px:
        rx1, rx2 = max(0, min(rx1, width - 1 - min_px)), min(width - 1, max(rx2, rx1 + min_px))
    if ry2 - ry1 < min_px:
        ry1, ry2 = max(0, min(ry1, height - 1 - min_px)), min(height - 1, max(ry2, ry1 + min_px))
    return (rx1, ry1, rx2, ry2)


class D435i:
    # wait_for_frames 기본 5000ms 는 USB hiccup 한 번에 제어 루프를 5초 세운다.
    # 500ms(15프레임)면 정상 지터는 넘기고, 진짜 스톨은 예외 → main 이 드롭으로 처리한다.
    def __init__(self, width=640, height=480, fps=30, timeout_ms=500):
        if rs is None:
            raise ImportError("pyrealsense2가 설치되어 있지 않습니다. `pip install pyrealsense2` 확인") from _IMPORT_ERROR

        self.width = width
        self.height = height
        self.fps = fps
        self.timeout_ms = int(timeout_ms)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = None
        self.profile = None
        self.depth_scale = 0.001
        self.intrinsics = None
        self._color_sensor = None
        self._ae_roi_last = None          # 마지막으로 보낸 (x1,y1,x2,y2)
        self._ae_roi_last_t = -1e9

    def start(self):
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)

        self.depth_scale = float(self.profile.get_device().first_depth_sensor().get_depth_scale())
        intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intrinsics = {"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy}
        print(f"[CAM] D435i started. depth_scale={self.depth_scale:.6f}")

        self._color_sensor = self._find_color_sensor()
        if self._color_sensor is None:
            print("[CAM] 컬러 센서를 찾지 못해 노출 옵션을 건너뜁니다")
        else:
            apply_color_exposure_options(self._color_sensor, CONFIG["camera"])

    def _find_color_sensor(self):
        try:
            for s in self.profile.get_device().query_sensors():
                is_color = getattr(s, "is_color_sensor", None)
                if (is_color() if is_color else False) or "RGB" in str(s.get_info(rs.camera_info.name)):
                    return s
        except Exception as exc:
            print(f"[CAM] 센서 열거 실패: {type(exc).__name__}: {exc}")
        return None

    def set_exposure_roi(self, bbox, now):
        """AE 측광 영역을 추적 bbox(1.5배) 로, bbox 가 None 이면 전체 프레임으로.
        1초에 한 번, 그리고 bbox→bbox 는 중심이 프레임의 10% 이상 움직였을 때만 보낸다. 실패는 1초 뒤 재시도."""
        if self._color_sensor is None:
            return False
        roi = roi_from_bbox(bbox, self.width, self.height)
        if roi == self._ae_roi_last or now - self._ae_roi_last_t < AE_ROI_MIN_INTERVAL_SEC:
            return False
        full = roi_from_bbox(None, self.width, self.height)
        if bbox is not None and self._ae_roi_last not in (None, full):
            dx = abs((roi[0] + roi[2]) - (self._ae_roi_last[0] + self._ae_roi_last[2])) / 2.0
            dy = abs((roi[1] + roi[3]) - (self._ae_roi_last[1] + self._ae_roi_last[3])) / 2.0
            if dx < AE_ROI_MOVE_FRAC * self.width and dy < AE_ROI_MOVE_FRAC * self.height:
                return False
        self._ae_roi_last_t = now
        t0 = time.perf_counter()
        try:
            r = rs.region_of_interest()
            r.min_x, r.min_y, r.max_x, r.max_y = roi
            self._color_sensor.as_roi_sensor().set_region_of_interest(r)
        except Exception as exc:
            print(f"[CAM] AE ROI {roi} 설정 실패: {type(exc).__name__}: {exc}")
            return False
        # hwmon USB 왕복이라 느릴 수 있다 (librealsense #7130: ~140 ms 보고). 제어 루프 안에서 5 ms 를 넘으면 알린다 —
        # 그러면 config camera.ae_roi_follow_track 을 끄거나 워커 스레드로 옮길 것.
        cost_ms = (time.perf_counter() - t0) * 1e3
        if cost_ms > 5.0:
            print(f"[CAM] AE ROI 설정 {cost_ms:.0f} ms — 제어 루프 스톨. ae_roi_follow_track 끄기를 권장")
        self._ae_roi_last = roi
        return True

    def stop(self):
        self.pipeline.stop()

    def get_frames(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=self.timeout_ms)
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        return np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data())
