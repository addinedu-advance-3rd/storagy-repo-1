import cv2
import torch
import logging
import time
from datetime import datetime
from ultralytics import YOLO

# 🔹 YOLO 로그 메시지를 끄기 위한 설정
logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

class ObjectDetect:
    def __init__(self, latest_worker, cam_index=2, model_path="yolov8n.pt", lost_frame_count=45, detected_frame_count=45):
        """
        객체 감지를 수행하는 클래스
        - latest_worker: 최근 감지된 사용자를 공유하는 변수
        - cam_index: 사용할 카메라 인덱스
        - model_path: YOLO 모델 경로
        - lost_frame_count: 객체가 사라졌다고 판단할 연속 감지 실패 프레임 수
        - detected_frame_count: 감지 또는 사라짐 상태를 유지해야 하는 최소 프레임 수
        """
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = YOLO(model_path).to(self.device)

        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.latest_worker = latest_worker  # FaceDetect의 감지 결과 공유
        self.detected_objects = {"Spanner": lost_frame_count, "Hammer": lost_frame_count, "Driver": lost_frame_count}  # 감지 상태 프레임 카운트
        self.last_user = "Unknown User"
        self.previous_state = {"Spanner": "Missing", "Hammer": "Missing", "Driver": "Missing"}  # 이전 상태 저장
        self.confirmed_state = {"Spanner": "Missing", "Hammer": "Missing", "Driver": "Missing"}  # 최소 프레임 유지된 확정 상태
        self.state_count = {"Spanner": 0, "Hammer": 0, "Driver": 0}  # 감지 연속 프레임 카운트
        self.rental_times = {"Spanner": None, "Hammer": None, "Driver": None}  # 대여 시간 저장
        self.return_times = {"Spanner": None, "Hammer": None, "Driver": None}  # 반납 시간 저장

        # 🔹 감지할 클래스 지정 (spanner: 67, hammer: 39, driver: 64)
        self.target_classes = {67: "Spanner", 39: "Hammer", 64: "Driver"}
        self.lost_frame_count = lost_frame_count
        self.detected_frame_count = detected_frame_count
    
    def update_detection_status(self, detected_now):
        """ 객체 감지 상태를 업데이트하고, 중복 이벤트 발생을 방지하는 함수 """
        for obj in self.detected_objects:
            if detected_now[obj]:
                self.detected_objects[obj] = min(self.lost_frame_count, self.detected_objects[obj] + 1)  # 감지되면 증가 (최대 lost_frame_count 유지)
                self.state_count[obj] += 1  # 감지된 프레임 카운트 증가
            else:
                self.detected_objects[obj] = max(0, self.detected_objects[obj] - 1)  # 감지되지 않으면 감소
                self.state_count[obj] = max(0, self.state_count[obj] - 1)  # 감지 연속 프레임 감소

            current_state = "Detected" if self.state_count[obj] >= self.detected_frame_count else "Missing"

            # 🔹 최소 감지 프레임을 충족해야 상태 변경 확정
            if self.detected_objects[obj] == self.lost_frame_count:
                self.confirmed_state[obj] = "Detected"
                if self.rental_times[obj]:  # 대여 상태였다면 반납 시간 기록
                    self.return_times[obj] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            elif self.detected_objects[obj] == 0:
                self.confirmed_state[obj] = "Missing"
                self.rental_times[obj] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')  # 사라진 순간 대여 시간 기록
                self.return_times[obj] = None  # 아직 반납되지 않음

            # 🔹 상태 변화 확인 후 이벤트 발생 (같은 상태에서는 중복 실행 X)
            if self.previous_state[obj] != self.confirmed_state[obj]:
                prev_user = self.last_user
                self.last_user = self.latest_worker.value if self.latest_worker.value != "No Match" else "Unknown User"
                if self.confirmed_state[obj] == "Missing":
                    print(f"🚨 {obj} 사라짐 → 가져간 사용자: {self.last_user} (이전: {prev_user}) | 대여 시간: {self.rental_times[obj]}")
                else:
                    print(f"✅ {obj} 감지됨 → {self.last_user} 반납 처리 (이전: {prev_user}) | 반납 시간: {self.return_times[obj]}")
            
            self.previous_state[obj] = self.confirmed_state[obj]  # 이전 상태 업데이트

    def detect_objects(self):
        if not self.cap.isOpened():
            print("Error: 객체 감지 카메라를 열 수 없습니다.")
            return

        print("YOLO 객체 검출 시작...")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Error: 카메라 프레임을 읽을 수 없습니다.")
                break

            results = self.model(frame, conf=0.1, verbose=False)
            detected_now = {key: False for key in self.detected_objects}  # 현재 프레임에서 감지된 객체

            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()
                class_ids = result.boxes.cls.cpu().numpy()

                for box, class_id in zip(boxes, class_ids):
                    if int(class_id) in self.target_classes:
                        label = self.target_classes[int(class_id)]
                        x1, y1, x2, y2 = map(int, box)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        detected_now[label] = True

            # 감지 상태 업데이트 및 중복 이벤트 방지 처리
            self.update_detection_status(detected_now)

            cv2.imshow("YOLO Object Detection", frame)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
