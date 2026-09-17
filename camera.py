"""
camera.py — Intel RealSense D435i RGB-D 래퍼 (pyrealsense2)

컬러(BGR8)·깊이(Z16) 스트림을 켜고, 깊이를 컬러 시점에 정렬한 프레임 쌍을
get_frames() 로 돌려준다. main.py 는 .intrinsics / .depth_scale 을 읽어
MeasurementBuilder 에 넘긴다.

IMU(accel/gyro) 스트림은 켜지 않는다 — 읽는 곳이 없고, 켜 두면 IMU 전용
frameset 이 wait_for_frames() 에 섞여 들어와 컬러/깊이 없는 프레임이 생기고
메인 루프가 dt 를 잃는다.
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
        self.intrinsics = None

    def start(self):
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

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
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        return color_image, depth_image
