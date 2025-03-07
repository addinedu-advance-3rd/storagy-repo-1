import cv2
import torch
import logging
import numpy as np
from ultralytics import YOLO
from collections import defaultdict
from math import atan2, degrees

# ✅ YOLO의 불필요한 로그 메시지 제거
logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

class FODDetector:
    def __init__(self, model_path="runs/detect/fod_ver1/weights/best.pt", camera_url="http://192.168.0.6:5000/video_feed"):
        """ 객체 초기화 """
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path).to(self.device)

        # ✅ Homography 행렬 (실제 값으로 교체 필요)
        self.homography_matrix = np.array([
            [-1.33501421e-01,  8.38672822e-03,  4.06410103e+01],
            [-3.71847207e-03,  4.26407866e-02, -7.44009040e+01],
            [4.94951733e-05, -4.08950562e-03,  1.00000000e+00]
        ])

        # ✅ 감지된 객체 저장 (프레임 누락 방지)
        self.detected_objects = defaultdict(lambda: {"count": 0, "bbox": None, "distance": None, "angle": None})
        self.max_missed_frames = 10  # 감지를 유지할 프레임 수
        self.max_y = 480  # 프레임 바닥 좌표

        # ✅ 웹캠 설정
        self.cap = cv2.VideoCapture(camera_url)
        if not self.cap.isOpened():
            raise Exception("❌ 웹캠을 열 수 없습니다.")

    def estimate_distance(self, cx, cy):
        """ 픽셀 좌표 (cx, cy)를 실세계 거리로 변환 """
        pixel_point = np.array([[[cx, cy]]], dtype=np.float32)
        real_point = cv2.perspectiveTransform(pixel_point, self.homography_matrix)[0][0]
        real_x, real_y = real_point[0], real_point[1]
        return (real_x ** 2 + real_y ** 2) ** 0.5  # 유클리드 거리 계산

    def estimate_real_angle(self, cx, cy):
        """ 실제 거리 기반 실각도 계산 """
        pixel_point = np.array([[[cx, cy]]], dtype=np.float32)
        real_point = cv2.perspectiveTransform(pixel_point, self.homography_matrix)[0][0]
        real_x, real_y = real_point[0], real_point[1]  # 실세계 좌표

        camera_x, camera_y = 0, 0  # 카메라의 실제 위치 (필요하면 높이 추가 가능)
        dx_real = real_x - camera_x  # 가로 거리 차이
        dy_real = real_y - camera_y  # 세로 거리 차이

        return degrees(atan2(dx_real, dy_real))  # 실각도 계산

    def detect_objects(self, frame):
        """ YOLO를 이용한 객체 감지 및 정보 저장 """
        results = self.model(frame, conf=0.2)
        current_objects = set()

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # 중심 좌표 계산

                # ✅ y 좌표 필터 적용 (바닥에 있는 객체만 감지)
                if cy > 250:  
                    distance = self.estimate_distance(cx, cy)  # 실세계 거리 계산
                    angle = self.estimate_real_angle(cx, cy)  # 실각도 계산

                    label = f"FOD {distance:.2f} cm, Angle {angle:.2f}°"
                    print(f"FOD {distance:.2f} cm | Angle {angle:.2f}°")

                    # ✅ 감지된 객체 저장
                    current_objects.add(label)
                    self.detected_objects[label]["count"] = self.max_missed_frames
                    self.detected_objects[label]["bbox"] = (x1, y1, x2, y2)
                    self.detected_objects[label]["distance"] = distance
                    self.detected_objects[label]["angle"] = angle

        return current_objects

    def draw_detections(self, frame):
        """ 감지된 객체에 바운딩 박스 및 정보 표시 """
        for obj, data in list(self.detected_objects.items()):
            if data["count"] > 0:
                x1, y1, x2, y2 = data["bbox"]
                distance = data["distance"]
                angle = data["angle"]

                # ✅ 바운딩 박스 및 정보 표시
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"D: {distance:.2f} cm", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Angle: {angle:.2f}°", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                self.detected_objects[obj]["count"] -= 1  # 유지 시간 감소

    def run(self):
        """ 웹캠 실행 및 객체 감지 루프 """
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                print("❌ 웹캠에서 프레임을 읽을 수 없습니다.")
                break

            # ✅ 객체 감지
            current_objects = self.detect_objects(frame)

            # ✅ 탐지가 끊긴 객체도 일정 시간 유지
            self.draw_detections(frame)

            # ✅ 화면 출력
            cv2.imshow("FOD Detection - YOLOv8", frame)

            # ✅ 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

# ✅ 실행
if __name__ == "__main__":
    detector = FODDetector()
    detector.run()
