import cv2
import torch
import logging
import time
from datetime import datetime
from ultralytics import YOLO
import json
import numpy as np

from db.flask_db.app import db
from db.flask_db.app.models import Log, Tool

# 🔹 YOLO 로그 메시지를 끄기 위한 설정
logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

class ObjectDetect:
    def __init__(self, latest_worker, cam_index=2, model_path="/home/addinedu/venv/facedetection/tools_train/runs/segment/tools_training/weights/best.pt"
    , lost_frame_count=45, detected_frame_count=45):
        """
        객체 감지를 수행하는 클래스
        - latest_worker: 최근 감지된 사용자를 공유하는 변수
        - cam_index: 사용할 카메라 인덱스
        - model_path: YOLO 모델 경로 (Instance Segmentation)
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

        # JSON 경로 설정
        self.tools_json_path = "db/tools.json"
        self.log_json_path = "db/log.json"

        # 🔹 감지할 클래스 지정 (spanner: 67, hammer: 39, driver: 64)
        self.target_classes = {0: "Driver", 1: "Hammer", 2: "Spanner"}  # 모델 내 클래스 인덱스 사용
        self.lost_frame_count = lost_frame_count
        self.detected_frame_count = detected_frame_count

#################데이터 저장 및 로드 #####################
        #Tool을 json으로 init, 처음에만
        self.tools_json_path = "db/tools.json"

    # Tool
    def is_Tool_empty():
        return db.session.query(Tool).count() == 0

    def Tool_init(self):
        if self.is_Tool_empty():
            tools = self.load_json(self.tools_json_path)
            for tool in tools:
                new_tool = Tool(name=tool['name'], avail=tool['avail'])
                db.session.add(new_tool)
            db.session.commit()

    def update_Tool(tool_name, avail):
        """ Tool의 avail 상태 업데이트 """
        tool = Tool.query.filter_by(name=tool_name).first()
        # id가 더 바람직
        if tool:
            tool.avail = avail
            db.session.commit()

    # Log
    def create_log(tool_name, user_name, rental_time):
        """ 대여 """
        tool = Tool.query.filter_by(name=tool_name).first()
        new_log = Log(tool_id=tool.id, user_name=user_name, rental_time=rental_time)
        db.session.add(new_log)
        db.session.commit()

    def fix_log(tool_name, return_time):
        """ 반납 """
        tool = Tool.query.filter_by(name=tool_name).first()
        log = Log.query.filter(Log.tool_id == tool.id, Log.return_date == None).first()
        if log :
            log.return_time = return_time
            db.session.commit()
            print(tool_name, '반납 완료')

################DB랑 합칠 때 포맷만 json-> db로 변경###########

    def update_detection_status(self, detected_now):
        """ 객체 감지 상태를 업데이트하고, 중복 이벤트 발생을 방지하는 함수 """
        for obj in self.detected_objects:
            if detected_now[obj]:
                self.detected_objects[obj] = min(self.lost_frame_count, self.detected_objects[obj] + 1)  # 감지되면 증가
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
                if self.rental_times[obj] is None:  # ✅ 처음 사라질 때만 기록
                    self.rental_times[obj] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                self.return_times[obj] = None  # 아직 반납되지 않음

            # 🔹 상태 변화 확인 후 이벤트 발생 (같은 상태에서는 중복 실행 X)
            if self.previous_state[obj] != self.confirmed_state[obj]:
                prev_user = self.last_user
                self.last_user = self.latest_worker.value if self.latest_worker.value != "No Match" else "Unknown User"

                # 대여 발생
                if self.confirmed_state[obj] == "Missing":
                    self.update_Tool(obj, False)
                    self.create_log(self.last_user, self.rental_times[obj])
                    print(f"🚨 {obj} 사라짐 → 가져간 사용자: {self.last_user} (이전: {prev_user}) | 대여 시간: {self.rental_times[obj]}")

                # 반납 발생
                else:
                    self.return_times[obj] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    self.update_Tool(obj, True)
                    self.fix_log(obj, self.return_times[obj])
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

            results = self.model(frame, conf=0.5)
            detected_now = {key: False for key in self.detected_objects}

            for result in results:
                if result.masks is not None:
                    for mask, cls in zip(result.masks.xy, result.boxes.cls):
                        mask = np.array(mask, dtype=np.int32)
                        class_id = int(cls)
                        
                        # ✅ 거미줄 문제 해결: 너무 작은 폴리곤 무시
                        if len(mask) < 10:
                            continue

                        # ✅ 거미줄 문제 해결: 연결된 점들의 거리 체크 (너무 길면 제외)
                        distances = np.linalg.norm(mask - np.roll(mask, shift=1, axis=0), axis=1)
                        if np.any(distances > 200):
                            continue

                        # ✅ 외곽선(폴리곤) 그리기 (초록색)
                        cv2.polylines(frame, [mask], isClosed=True, color=(0, 255, 0), thickness=2)

                        # ✅ 객체 중심 계산 후 클래스명 표시
                        M = cv2.moments(mask)
                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            label = self.target_classes[class_id]
                            cv2.putText(frame, label, (cX, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                            detected_now[label] = True

            # 감지 상태 업데이트 및 중복 이벤트 방지 처리
            self.update_detection_status(detected_now)

            cv2.imshow("YOLO Instance Segmentation - Improved Polygon", frame)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
