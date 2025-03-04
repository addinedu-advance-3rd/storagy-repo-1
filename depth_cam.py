import cv2
import numpy as np

class DepthCamera:
    def __init__(self, url="http://192.168.0.6:5000/depth_feed"):
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            raise Exception("❌ Depth 카메라를 열 수 없습니다.")

    def get_depth_frame(self):
        """ Depth 프레임 가져오기 """
        ret, depth_frame = self.cap.read()
        if not ret:
            return None
        return cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)  # 깊이 맵 변환

    def get_distance(self, depth_frame, cx, cy, roi_size=5):
        """ ROI 평균값 사용하여 거리값 반환 """
        h, w = depth_frame.shape
        cx, cy = int(cx), int(cy)

        x1, x2 = max(0, cx - roi_size), min(w, cx + roi_size)
        y1, y2 = max(0, cy - roi_size), min(h, cy + roi_size)

        roi = depth_frame[y1:y2, x1:x2]
        depth_value = np.median(roi)  # 중앙값 활용하여 노이즈 제거

        return depth_value if depth_value > 0 else None  # 유효한 값만 반환

    def release(self):
        self.cap.release()
