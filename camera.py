"""
camera.py — Intel RealSense D435i RGB-D + IMU 래퍼

기존 업로드에는 D435i 클래스 구현이 없어서, pyrealsense2 기반으로 새로 구성.
"""

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError as exc:
    rs = None
    _IMPORT_ERROR = exc


class D435i:
    def __init__(self, width=640, height=480, fps=30):
        if rs is None:
            raise ImportError("pyrealsense2가 설치되어 있지 않습니다. `pip install pyrealsense2` 확인") from _IMPORT_ERROR

        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = None
        self.profile = None
        self.depth_scale = 0.001
        self.accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.intrinsics = None

    def start(self):
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        try:
            self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f)
            self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f)
        except Exception:
            pass

        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())

        color_stream = self.profile.get_stream(rs.stream.color)
        intr = color_stream.as_video_stream_profile().get_intrinsics()
        self.intrinsics = {"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy}
        print(f"[CAM] D435i started. depth_scale={self.depth_scale:.6f}")

    def stop(self):
        self.pipeline.stop()

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()

        # IMU stream이 섞여 들어올 수 있으므로 motion frame 먼저 처리
        for f in frames:
            try:
                if f.is_motion_frame():
                    motion = f.as_motion_frame()
                    data = motion.get_motion_data()
                    profile = f.get_profile()
                    stream_type = profile.stream_type()
                    if stream_type == rs.stream.accel:
                        self.accel = (data.x, data.y, data.z)
                    elif stream_type == rs.stream.gyro:
                        self.gyro = (data.x, data.y, data.z)
            except Exception:
                pass

        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        return color_image, depth_image

    def get_depth_m(self, depth_image, x1, y1, x2, y2):
        # 호환성용. 실제로는 measurement.py에서 depth quality까지 계산 권장.
        if depth_image is None:
            return None
        H, W = depth_image.shape
        x1 = max(0, int(x1)); y1 = max(0, int(y1))
        x2 = min(W, int(x2)); y2 = min(H, int(y2))
        if x2 <= x1 or y2 <= y1:
            return None
        roi = depth_image[y1:y2, x1:x2]
        valid = roi[(roi > 0)]
        if valid.size < 10:
            return None
        return float(np.median(valid)) * self.depth_scale
