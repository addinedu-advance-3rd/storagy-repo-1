import numpy as np
import cv2

class RGBDepthAligner:
    def __init__(self, rgb_resolution=(1280, 720), depth_resolution=(640, 480)):
        """ RGB → Depth 정렬을 위한 초기 설정 """
        self.rgb_width, self.rgb_height = rgb_resolution
        self.depth_width, self.depth_height = depth_resolution

        # ✅ RGB → Depth 해상도 변환 비율
        self.scale_x = self.depth_width / self.rgb_width
        self.scale_y = self.depth_height / self.rgb_height

        # ✅ 예제 Intrinsics (제조사 제공값 필요)
        self.rgb_intrinsics = np.array([
            [900, 0, 640],  # fx, 0, cx
            [0, 900, 360],  # 0, fy, cy
            [0, 0, 1]       # 0, 0, 1
        ])

        self.depth_intrinsics = np.array([
            [700, 0, 320],  # fx, 0, cx
            [0, 700, 240],  # 0, fy, cy
            [0, 0, 1]       # 0, 0, 1
        ])

        # ✅ Extrinsic Matrix (RGB → Depth 변환 행렬, 직접 캘리브레이션 가능)
        self.R = np.eye(3)  # 회전 행렬
        self.T = np.array([[0.05], [0], [0]])  # 이동 행렬 (예제값)

    def transform_rgb_to_depth(self, cx, cy):
        """ RGB에서 얻은 좌표(cx, cy)를 Depth 좌표계로 변환 """
        rgb_point = np.array([[cx], [cy], [1]])
        depth_point = self.depth_intrinsics @ (self.R @ np.linalg.inv(self.rgb_intrinsics) @ rgb_point + self.T)

        cx_depth, cy_depth = int(depth_point[0, 0]), int(depth_point[1, 0])

        # ✅ 유효한 범위로 클리핑
        cx_depth = max(0, min(self.depth_width - 1, cx_depth))
        cy_depth = max(0, min(self.depth_height - 1, cy_depth))

        return cx_depth, cy_depth
