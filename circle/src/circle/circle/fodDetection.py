import cv2
import torch
import logging
import numpy as np
import multiprocessing
from ultralytics import YOLO
from math import atan2, degrees

# ✅ YOLO의 불필요한 로그 메시지 제거
logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

class FODDetector:
    def __init__(self, shared_data, model_path="/home/addinedu/venv/FOD/runs/detect/fod_ver1/weights/best.pt", camera_url="http://192.168.0.6:5000/video_feed"):
        """ 객체 초기화 """
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path).to(self.device)
        self.shared_data = shared_data  # 공유 데이터 딕셔너리
        self.frame_count = 0  # ✅ 프레임 카운터 추가

        # ✅ Homography 행렬 (실제 값으로 교체 필요)
        self.homography_matrix = np.array([
            [-1.33501421e-01,  8.38672822e-03,  4.06410103e+01],
            [-3.71847207e-03,  4.26407866e-02, -7.44009040e+01],
            [4.94951733e-05, -4.08950562e-03,  1.00000000e+00]
        ])

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

        camera_x, camera_y = 0, 0  # 카메라의 실제 위치
        dx_real = real_x - camera_x  # 가로 거리 차이
        dy_real = real_y - camera_y  # 세로 거리 차이

        return degrees(atan2(dx_real, dy_real))  # 실각도 계산

    def detect_objects(self, frame):
        """ YOLO를 이용한 객체 감지 및 정보 저장 """
        results = self.model(frame, conf=0.5)

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # 중심 좌표 계산

                # ✅ y 좌표 필터 적용 (바닥에 있는 객체만 감지)
                if cy > 300:  
                    distance = self.estimate_distance(cx, cy)  # 실세계 거리 계산
                    angle = self.estimate_real_angle(cx, cy)  # 실각도 계산

                    # ✅ 감지된 객체 정보를 공유 메모리에 즉시 저장
                    self.shared_data["distance"] = distance
                    self.shared_data["angle"] = angle

                    print(f"🟢 FOD 감지됨: 거리 {distance:.2f} cm, 각도 {angle:.2f}°")

                    # ✅ 바운딩 박스 및 정보 표시
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"D: {distance:.2f} cm", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"Angle: {angle:.2f}°", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def run(self):
        print("🟢 FOD 감지 시작...")
        """ 웹캠 실행 및 객체 감지 루프 """
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                print("❌ 웹캠에서 프레임을 읽을 수 없습니다.")
                break

            self.frame_count += 1  # ✅ 프레임 카운트 증가

            # ✅ 45 프레임마다 한 번 감지 실행
            if self.frame_count % 45 == 0:
                print(f"🔍 {self.frame_count} 프레임 - FOD 감지 실행")
                self.detect_objects(frame)
            #else:
                #print(f"⏳ {self.frame_count} 프레임 - 감지 건너뜀")

            # ✅ 화면 출력
            cv2.imshow("FOD Detection - YOLOv8", frame)

            # ✅ 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

def start_fod_detector(shared_data):
    detector = FODDetector(shared_data)
    detector.run()

# ✅ 실행
if __name__ == "__main__":
    manager = multiprocessing.Manager()
    shared_data = manager.dict()  # ✅ 공유 딕셔너리 생성
    shared_data["distance"] = 0.0
    shared_data["angle"] = 0.0

    # ✅ FODDetector 프로세스 실행 (백그라운드 실행)
    fod_process = multiprocessing.Process(target=start_fod_detector, args=(shared_data,))
    fod_process.start()
