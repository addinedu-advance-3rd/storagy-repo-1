import cv2
import torch
from ultralytics import YOLO
from collections import defaultdict

# ✅ YOLO 모델 로드
model = YOLO("runs/detect/fod_ver1/weights/best.pt")

# ✅ 특정 클래스만 감지 (Bolt, Nut, Nail)
target_classes = [1, 5, 6]  # Sheet(4) 없음

class RGBCamera:
    def __init__(self, url="http://192.168.0.6:5000/video_feed"):
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            raise Exception("❌ RGB 카메라를 열 수 없습니다.")

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def detect_fod(self, frame):
        """ YOLO로 FOD 검출 """
        results = model(frame, conf=0.2, classes=target_classes)
        fod_objects = []

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # 중심점
                fod_objects.append((cx, cy, x1, y1, x2, y2))

        return fod_objects  # 검출된 FOD 좌표 리스트 반환

    def release(self):
        self.cap.release()
