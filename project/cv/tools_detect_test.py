import cv2
import torch
import logging
import time
from datetime import datetime
from ultralytics import YOLO
import numpy as np
import requests
import json
import os

from flask import current_app
from flask_socketio import emit

# 🔹 YOLO 로그 메시지를 끄기 위한 설정
logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

class ObjectDetect:
    def __init__(self, latest_worker
                #  , cam_index=3
                 , cam_index=0 #test
                 , model_path="/home/addinedu/dev_ws/storagy-repo-1/project/cv/tools_train/runs/segment/tools_training/weights/best.pt"
    , lost_frame_count=60, detected_frame_count=60
    , tools_status=None, event_queue=None):
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
        self.app = current_app # 잘 가져오는가?
        from app import db, socketio
        from app.models import Tool, Log
        self.db = db
        self.socketio = socketio
        self.Tool = Tool
        self.Log = Log
        # 구조개선필요
        self.tools_status = tools_status
        self.event_queue = event_queue

    # Tool
    def update_Tool(self, tool_name, avail):
        """도구 상태를 데이터베이스에 업데이트하는 함수"""
        try:
            from app.models import Tool
            from app import db
            import traceback
            
            print(f"[DB] 도구 '{tool_name}' 상태 업데이트 시도: {avail}")
            
            # 도구 이름으로 도구 객체 찾기
            tool = Tool.query.filter_by(name=tool_name).first()
            
            if tool:
                # 현재 상태 확인
                current_status = tool.avail
                print(f"[DB] 도구 '{tool_name}' 현재 상태: {current_status}, 변경할 상태: {avail}")
                
                # 상태가 다를 때만 업데이트
                if current_status != avail:
                    # 상태 변경 및 저장
                    tool.avail = avail
                    db.session.commit()
                    print(f"[DB] 도구 '{tool_name}' 상태 업데이트 성공: {avail}")
                    
                    # 직접 SQL 쿼리로 확인
                    from sqlalchemy import text
                    result = db.session.execute(text(f"SELECT avail FROM tool WHERE name = '{tool_name}'")).fetchone()
                    print(f"[DB] SQL 확인 결과: {result}")
                    
                    return True
                else:
                    print(f"[DB] 도구 '{tool_name}' 상태가 이미 {avail}입니다. 업데이트 불필요.")
                    return False
            else:
                print(f"[DB] 도구 '{tool_name}'을 찾을 수 없음")
                return False
        except Exception as e:
            print(f"[DB] 도구 상태 업데이트 중 오류: {e}")
            import traceback
            print(traceback.format_exc())
            return False

    # Log
    def create_log(self, tool_name, user_name, rental_date):
        """ 대여 """
        tool = self.Tool.query.filter_by(name=tool_name).first()
        if tool is None :
            print('응 나는 도구를 찾지 못했어')
        else :
            print('도구 찾았다')
            new_log = self.Log(tool_id=tool.id, user_name=user_name, rental_date=rental_date)
            self.db.session.add(new_log)
            self.db.session.commit()
            #self.socketio.emit('log-update')

    def fix_log(self, tool_name, return_date):
        """ 반납 """
        tool = self.Tool.query.filter_by(name=tool_name).first()
        log = self.Log.query.filter(self.Log.tool_id == tool.id, self.Log.return_date == None).first()
        if log :
            log.return_date = return_date
            self.db.session.commit()
            #self.socketio.emit('log-update')
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
                    self.return_times[obj] = datetime.now()
            elif self.detected_objects[obj] == 0:
                self.confirmed_state[obj] = "Missing"
                if self.rental_times[obj] is None:  # ✅ 처음 사라질 때만 기록
                    self.rental_times[obj] = datetime.now()
                self.return_times[obj] = None  # 아직 반납되지 않음

            # 🔹 상태 변화 확인 후 이벤트 발생 (같은 상태에서는 중복 실행 X)
            if self.previous_state[obj] != self.confirmed_state[obj]:
                prev_user = self.last_user
                self.last_user = self.latest_worker.value if self.latest_worker.value != "No Match" else "Unknown User"

                # 대여 발생
                if self.confirmed_state[obj] == "Missing":
                    try:
                        if self.app and hasattr(self.app, 'app_context'):
                            with self.app.app_context():
                                self.update_Tool(obj, False)
                                self.create_log(obj, self.last_user, self.rental_times[obj])
                                # 로그 업데이트 이벤트 추가 (명확한 데이터 구조)
                                if self.socketio:
                                    self.socketio.emit('log-update', {
                                        'tool_name': obj,
                                        'action': 'rental',
                                        'user': self.last_user,
                                        'timestamp': str(self.rental_times[obj])
                                    }, namespace='/')
                                
                                # 이벤트 큐를 통한 업데이트
                                if hasattr(self, 'event_queue') and self.event_queue:
                                    self.event_queue.put({
                                        'type': 'tool-update',
                                        'payload': {
                                            'tool_name': obj,
                                            'avail': False,
                                            'user': self.last_user,
                                            'timestamp': str(self.rental_times[obj]),
                                            'force_update': True
                                        }
                                    })
                                
                                # 공유 딕셔너리 직접 업데이트
                                if hasattr(self, 'tools_status') and self.tools_status is not None:
                                    self.tools_status[obj] = False
                                    print(f"[DEBUG] 공유 딕셔너리 직접 업데이트: {obj} -> False")

                                # HTTP 요청으로 상태 업데이트 (소켓 백업)
                                try:
                                    # Flask 서버 URL (환경에 맞게 수정 필요)
                                    url = "http://localhost:5000/api/tool/update"
                                    
                                    # 데이터 준비
                                    data = {
                                        "tool_name": obj,
                                        "avail": False,
                                        "user": self.last_user,
                                        "timestamp": str(self.rental_times[obj]),
                                        "api_key": "your_secret_key"  # 보안을 위한 키
                                    }
                                    
                                    # POST 요청 전송
                                    response = requests.post(url, json=data, timeout=3)
                                    print(f"[HTTP] 도구 상태 업데이트 요청 결과: {response.status_code} - {response.text}")
                                except Exception as e:
                                    print(f"[HTTP] 도구 상태 업데이트 요청 실패: {e}")
                    except Exception as e:
                        print(f"🚨 대여 처리 중 오류: {e}")
                    
                    print(f"🚨 {obj} 사라짐 → 가져간 사용자: {self.last_user} (이전: {prev_user}) | 대여 시간: {self.rental_times[obj]}")
                    # 이벤트 큐를 통해 이벤트 전달 (딕셔너리 형태로)
                    if hasattr(self, 'event_queue') and self.event_queue:
                        self.event_queue.put({
                            'type': 'console-log',
                            'payload': {
                                'message': f"🚨 {obj} 사라짐 → 가져간 사용자: {self.last_user} (이전: {prev_user}) | 대여 시간: {self.rental_times[obj]}",
                                'level': 'warning'
                            }
                        })
                    
                    # 강제 업데이트 추가
                    if hasattr(self, 'tools_status') and self.tools_status is not None:
                        self.tools_status[obj] = False

                # 반납 발생
                else:
                    try:
                        if self.app and hasattr(self.app, 'app_context'):
                            with self.app.app_context():
                                self.return_times[obj] = datetime.now()
                                self.update_Tool(obj, True)
                                self.fix_log(obj, self.return_times[obj])
                                # 로그 업데이트 이벤트 추가 (명확한 데이터 구조)
                                if self.socketio:
                                    self.socketio.emit('log-update', {
                                        'tool_name': obj,
                                        'action': 'return',
                                        'user': self.last_user,
                                        'timestamp': str(self.return_times[obj])
                                    }, namespace='/')
                    except Exception as e:
                        print(f"✅ 반납 처리 중 오류: {e}")
                    
                    print(f"✅ {obj} 감지됨 → {self.last_user} 반납 처리 (이전: {prev_user}) | 반납 시간: {self.return_times[obj]}")
                    # 이벤트 큐를 통해 이벤트 전달 (딕셔너리 형태로)
                    if hasattr(self, 'event_queue') and self.event_queue:
                        self.event_queue.put({
                            'type': 'console-log',
                            'payload': {
                                'message': f"✅ {obj} 감지됨 → {self.last_user} 반납 처리 (이전: {prev_user}) | 반납 시간: {self.return_times[obj]}",
                                'level': 'success'
                            }
                        })
                    
                    # 강제 업데이트 추가
                    if hasattr(self, 'tools_status') and self.tools_status is not None:
                        self.tools_status[obj] = True

            # 상태 파일 업데이트
            self.update_status_file()
            
            self.previous_state[obj] = self.confirmed_state[obj]  # 이전 상태 업데이트

    def detect_objects(self):
        """객체 감지 메인 함수"""
        # 초기화 플래그
        initialized = False
        
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

            # 첫 번째 프레임에서 도구 상태 초기화
            if not initialized:
                self.initialize_tool_status()
                initialized = True

            cv2.imshow("YOLO Instance Segmentation - Improved Polygon", frame)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def initialize_tool_status(self):
        """도구 상태를 초기화하는 함수"""
        from datetime import datetime
        
        try:
            # 현재 감지 상태에 따라 도구 상태 초기화
            for obj in self.detected_objects:
                is_detected = self.confirmed_state[obj] == "Detected"
                
                # 공유 메모리 업데이트
                if hasattr(self, 'tools_status') and self.tools_status is not None:
                    self.tools_status[obj] = is_detected
                    print(f"[INIT] 공유 메모리 도구 상태 초기화: {obj} -> {is_detected}")
                
                # 데이터베이스 업데이트
                try:
                    if self.app and hasattr(self.app, 'app_context'):
                        with self.app.app_context():
                            self.update_Tool(obj, is_detected)
                    else:
                        # HTTP 요청으로 상태 업데이트
                        try:
                            import requests
                            
                            # Flask 서버 URL
                            url = "http://localhost:5000/api/tool/update"
                            
                            # 데이터 준비
                            data = {
                                "tool_name": obj,
                                "avail": is_detected,
                                "user": "System",
                                "timestamp": str(datetime.now()),
                                "api_key": "your_secret_key"
                            }
                            
                            # POST 요청 전송
                            response = requests.post(url, json=data, timeout=3)
                            print(f"[INIT] HTTP 도구 상태 초기화 요청 결과: {response.status_code} - {response.text}")
                        except Exception as e:
                            print(f"[INIT] HTTP 도구 상태 초기화 요청 실패: {e}")
                except Exception as e:
                    print(f"[INIT] 도구 상태 초기화 중 오류: {e}")
            
            # 상태 파일 업데이트
            self.update_status_file()
            
            print("[INIT] 모든 도구 상태 초기화 완료")
            return True
        except Exception as e:
            print(f"[INIT] 도구 상태 초기화 중 오류: {e}")
            return False

    def update_status_file(self):
        """도구 상태를 파일로 저장하는 함수"""
        try:
            import json
            import os
            
            # 상태 정보 구성
            status_data = {}
            for obj in self.detected_objects:
                status_data[obj] = self.confirmed_state[obj] == "Detected"
            
            # 상태 파일 경로
            status_file = os.path.join(os.path.dirname(__file__), 'tool_status.json')
            
            # 파일에 저장
            with open(status_file, 'w') as f:
                json.dump(status_data, f)
            
            print(f"[FILE] 도구 상태 파일 업데이트 완료: {status_data}")
        except Exception as e:
            print(f"[FILE] 도구 상태 파일 업데이트 실패: {e}")
