import cv2
import torch
import logging
from ultralytics import YOLO

# 🔹 YOLO 로그 메시지를 끄기 위한 설정
logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

class ObjectDetect:
    def __init__(self, latest_worker, cam_index=2, model_path="yolov8n.pt"):
        # CUDA 사용 여부 확인
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if torch.cuda.is_available():
            print(f"Using GPU: {torch.cuda.get_device_name(0)}")
        else:
            print("Using CPU")
        self.model = YOLO(model_path).to(self.device)

        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.latest_worker = latest_worker  # FaceDetect의 감지 결과를 공유
        self.phone_detected = False  
        self.last_user = "Unknown User"  

        # 🔹 감지할 클래스 지정 (cell phone: 67, bottle: 39, watch(대체: clock): 74)
        self.target_classes = {67: "Cellphone", 39: "Bottle", 64: "Mouse"}

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

            # 🔹 YOLO 감지 실행 (verbose=False로 설정해 불필요한 로그 제거)
            results = self.model(frame, conf=0.1, verbose=False)

            phone_now_detected = False

            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()
                class_ids = result.boxes.cls.cpu().numpy()

                for box, class_id in zip(boxes, class_ids):
                    if int(class_id) in self.target_classes:  # 🔹 설정한 3개 클래스만 감지
                        x1, y1, x2, y2 = map(int, box)
                        label = self.target_classes[int(class_id)]
                        color = (255, 0, 0) if label == "Cellphone" else (0, 255, 0)  # 핸드폰은 파란색, 나머지는 초록색

                        # 핸드폰 감지 여부 확인
                        if label == "Cellphone":
                            phone_now_detected = True

                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            # 🔹 핸드폰이 사라짐
            if self.phone_detected and not phone_now_detected:
                prev_user = self.last_user  # 🔹 이전 사용자 정보 저장
                self.last_user = self.latest_worker.value if self.latest_worker.value != "No Match" else "Unknown User"

                print(f"🚨 핸드폰 사라짐 → 가져간 사용자: {self.last_user} (이전: {prev_user})")

            # 🔹 핸드폰이 다시 감지됨 (반납 처리)
            if not self.phone_detected and phone_now_detected:
                prev_user = self.last_user
                self.last_user = self.latest_worker.value if self.latest_worker.value != "No Match" else "Unknown User"

                print(f"📱 핸드폰 감지됨 → {self.last_user} 반납 처리 (이전: {prev_user})")

            self.phone_detected = phone_now_detected

            cv2.imshow("YOLO Object Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
