import pyrealsense2 as rs
import numpy as np
import cv2


class box_video_data:
    def __init__(self):
        self.dev = None
        self.intrinsics = None
        self.intrinsics2 = ""
        self.color_images = None
        self.depth_images = None


class d435:
    def __init__(self):
        self.pipe = None
        self.video_data = None
        self.initialize()

    def initialize(self):
        ctx = rs.context()
        devices = ctx.query_devices()
        dev = devices[0]
        count = len(devices)
        self.video_data = [box_video_data() for _ in range(count)]
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(dev.get_info(rs.camera_info.serial_number))
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
        cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        profile = self.pipe.start(cfg)
        intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.video_data[0].dev = dev
        self.video_data[0].intrinsics = intrinsics
        intrinsics_str = self.rsintrinsics2string(intrinsics)
        self.video_data[0].intrinsics2 = intrinsics_str

        for _ in range(30):
            self.pipe.wait_for_frames()  # 放弃几帧以进行自动曝光

    def rsintrinsics2string(self, intrinsics):
        intrinsics_str = ""
        intrinsics_str += f"{intrinsics.width}/"
        intrinsics_str += f"{intrinsics.height}/"
        intrinsics_str += f"{intrinsics.fx}/"
        intrinsics_str += f"{intrinsics.fy}/"
        intrinsics_str += f"{intrinsics.ppx}/"
        intrinsics_str += f"{intrinsics.ppy}/"
        intrinsics_str += f"{intrinsics.coeffs[0]}/"
        intrinsics_str += f"{intrinsics.coeffs[1]}/"
        intrinsics_str += f"{intrinsics.coeffs[2]}/"
        intrinsics_str += f"{intrinsics.coeffs[3]}/"
        intrinsics_str += f"{intrinsics.coeffs[4]}/"
        return intrinsics_str

    def string2rsintrinsics(self, intrinsics_str):
        intrinsics = rs.intrinsics()
        tokens = intrinsics_str.split('/')
        intrinsics.width = int(tokens[0])
        intrinsics.height = int(tokens[1])
        intrinsics.fx = float(tokens[2])
        intrinsics.fy = float(tokens[3])
        intrinsics.ppx = float(tokens[4])
        intrinsics.ppy = float(tokens[5])
        intrinsics.coeffs[0] = float(tokens[6])
        intrinsics.coeffs[1] = float(tokens[7])
        intrinsics.coeffs[2] = float(tokens[8])
        intrinsics.coeffs[3] = float(tokens[9])
        intrinsics.coeffs[4] = float(tokens[10])
        return intrinsics

    def get_d435_image(self):

        align_to_color = rs.align(rs.stream.color)

        frameset = self.pipe.wait_for_frames()  # Drop several frames for auto-exposure
        frameset = align_to_color.process(frameset)  # Align depth frame to color frame

        depth_frame = frameset.get_depth_frame()  # Get depth frame
        color_frame = frameset.get_color_frame()  # Get color frame

        color_image = np.asanyarray(color_frame.get_data())
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        depth_image = np.asanyarray(depth_frame.get_data())
        
        # color_image = cv2.flip(color_image, -1)
        # depth_image = cv2.flip(depth_image, -1)

        self.video_data[0].color_images = color_image
        self.video_data[0].depth_images = depth_image

        return self.video_data[0]

    def close_d435(self):
        self.pipe.stop()
