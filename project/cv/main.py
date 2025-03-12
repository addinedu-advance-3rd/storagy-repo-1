import multiprocessing
from .face_detect import FaceDetect
from .tools_detect_test import ObjectDetect  # 파일명 확인 필요
import time
import threading
from app import create_app
from datetime import datetime
import logging

class MainManager:
    def __init__(self, socketio=None):
        self.manager = multiprocessing.Manager()
        self.latest_worker = self.manager.Value("s", "No Match")  # 공유 메모리 생성 (초기값: "No Match")
        self.face_process = None
        self.object_process = None
        self.socketio = socketio
        self.monitor_thread = None
        self.running = True
        self.tools_status = self.manager.dict()  # 공구 상태를 저장할 공유 딕셔너리
        
        # 초기 도구 상태 설정
        self.tools_status["Hammer"] = True  # True = 사용 가능
        self.tools_status["Driver"] = True
        self.tools_status["Spanner"] = True
        
        # 이벤트 큐 추가 - 프로세스 간 통신용
        self.event_queue = self.manager.Queue()
        
        # Flask 앱 객체 저장 (소켓 이벤트 발생 시 필요)
        if socketio:
            self.flask_app = socketio.server.environ.get('flask.app')
            print(f"[DEBUG] Flask 앱 객체: {self.flask_app}")
        else:
            self.flask_app = None

    def run_face_detection(self):
        face_detector = FaceDetect(self.latest_worker)
        face_detector.recognize_face()

    def run_object_detection(self):
        """객체 인식 프로세스를 실행합니다."""
        logging.info("객체 인식 프로세스 시작됨")
        
        # 원래 순서대로 매개변수 전달
        object_detector = ObjectDetect(self.latest_worker, 
                                      cam_index=2,  # 카메라 인덱스 명시적 지정
                                      event_queue=self.event_queue, 
                                      tools_status=self.tools_status)
        object_detector.detect_objects()  # 원래 메서드 호출

    def monitor_latest_worker(self):
        last_debug_time = time.time()
        last_status_broadcast = time.time()
        
        while self.running:
            current_time = time.time()
            
            # 30초마다 디버그 메시지 출력 (상태 확인용)
            if current_time - last_debug_time > 30:
                print(f"[DEBUG] 모니터링 스레드 실행 중... 현재 도구 상태: {dict(self.tools_status)}")
                last_debug_time = current_time
            
            # 5초마다 모든 클라이언트에 현재 상태 브로드캐스트 (연결 유지 및 상태 동기화)
            if current_time - last_status_broadcast > 5:  # 15초에서 5초로 변경
                print("[DEBUG] 주기적 상태 브로드캐스트 실행...")
                try:
                    if self.socketio:
                        for tool_name, avail in self.tools_status.items():
                            # 도구 정보 가져오기 (대여자 정보 포함)
                            tool_info = {
                                'tool_name': tool_name,
                                'avail': avail,
                                'timestamp': str(datetime.now())
                            }
                            
                            # 대여자 정보 추가 (도구가 대여 중인 경우)
                            if not avail:
                                tool_info['current_user'] = self.latest_worker.value if self.latest_worker.value != "No Match" else "Unknown User"
                            else:
                                tool_info['current_user'] = None
                            
                            try:
                                # 앱 컨텍스트 없이 직접 emit 호출
                                self.socketio.emit('tool-update', tool_info, namespace='/')
                                print(f"[DEBUG] 주기적 상태 브로드캐스트: {tool_name} -> {avail} (대여자: {tool_info.get('current_user', 'None')})")
                            except Exception as e:
                                print(f"[ERROR] 주기적 상태 브로드캐스트 중 오류: {e}")
                except Exception as e:
                    print(f"[ERROR] 주기적 상태 브로드캐스트 실패: {e}")
                
                last_status_broadcast = current_time
            
            # 이벤트 큐 처리 - 우선순위 높게 처리
            try:
                while not self.event_queue.empty():
                    tool_name = None
                    avail = None
                    try:
                        event_data = self.event_queue.get_nowait()
                        
                        # 문자열인 경우 (이전 코드와의 호환성)
                        if isinstance(event_data, str):
                            print(f"[INFO] 문자열 이벤트 수신: {event_data}")
                            # 웹 콘솔에 로그 표시
                            if self.socketio:
                                self.socketio.emit('console-log', {'message': event_data}, namespace='/')
                            continue
                        
                        # 딕셔너리인 경우 (새로운 형식)
                        event_type = event_data.get('type')
                        event_payload = event_data.get('payload')
                        
                        if self.socketio and event_type and event_payload:
                            print(f"[DEBUG] 이벤트 발생: {event_type} - 페이로드: {event_payload}")
                            
                            # 도구 상태 업데이트 이벤트인 경우 공유 딕셔너리 업데이트
                            if event_type == 'tool-update' and 'tool_name' in event_payload and 'avail' in event_payload:
                                tool_name = event_payload['tool_name']
                                avail = event_payload['avail']
                                
                                # 로그 출력 추가
                                print(f"[DEBUG] 도구 상태 업데이트 이벤트 발생: {tool_name} -> {avail}")
                                
                                # 공유 딕셔너리 업데이트
                                self.tools_status[tool_name] = avail
                                
                                # 소켓 이벤트 발생 전 추가 로그
                                print(f"[DEBUG] 소켓 이벤트 발생 직전: {event_type} - {event_payload}")
                                
                                # 소켓 이벤트 발생 (여러 번 시도)
                                for attempt in range(3):
                                    try:
                                        self.socketio.emit('tool-update', event_payload, namespace='/', broadcast=True)
                                        print(f"[DEBUG] 소켓 이벤트 발생 완료: {event_type} (시도 #{attempt+1})")
                                        break
                                    except Exception as e:
                                        print(f"[ERROR] 소켓 이벤트 발생 실패 (시도 #{attempt+1}): {e}")
                                        time.sleep(0.2)
                            else:
                                print(f"[DEBUG] 도구 상태 변경 없음: {tool_name} (이미 {'사용 가능' if avail else '대여 중'})")
                            
                            # 실제 소켓 이벤트 발생 - 즉시 처리
                            try:
                                # 앱 컨텍스트 없이 직접 emit 호출
                                self.socketio.emit(event_type, event_payload, namespace='/')
                                print(f"[DEBUG] 이벤트 전송 완료: {event_type}")
                            except Exception as e:
                                print(f"[ERROR] 소켓 이벤트 발생 중 오류: {e}")
                                # 재시도
                                try:
                                    time.sleep(0.1)
                                    self.socketio.emit(event_type, event_payload, namespace='/')
                                    print(f"[DEBUG] 이벤트 재전송 성공: {event_type}")
                                except Exception as e2:
                                    print(f"[ERROR] 이벤트 재전송 실패: {e2}")
                    except Exception as e:
                        print(f"[ERROR] 이벤트 처리 중 오류: {e}")
                        import traceback
                        print(traceback.format_exc())
            except Exception as e:
                print(f"[ERROR] 이벤트 큐 처리 중 오류: {e}")
            
            # CPU 사용량 감소를 위한 짧은 대기
            time.sleep(0.1)

    def start_processes(self):
        print("[INFO] Starting FaceDetect and ObjectDetect processes...")
        
        # 모니터링 스레드 먼저 시작
        self.running = True
        self.monitor_thread = threading.Thread(target=self.monitor_latest_worker)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        # 프로세스 시작 - 각 프로세스에 대해 별도의 예외 처리
        try:
            self.face_process = multiprocessing.Process(target=self.run_face_detection)
            self.face_process.start()
            print("[INFO] 얼굴 인식 프로세스 시작됨")
        except Exception as e:
            print(f"[ERROR] 얼굴 인식 프로세스 시작 실패: {e}")
            import traceback
            print(traceback.format_exc())
        
        try:
            self.object_process = multiprocessing.Process(target=self.run_object_detection)
            self.object_process.start()
            print("[INFO] 객체 인식 프로세스 시작됨")
        except Exception as e:
            print(f"[ERROR] 객체 인식 프로세스 시작 실패: {e}")
            import traceback
            print(traceback.format_exc())

    def stop_processes(self):
        print("[INFO] Stopping processes...")
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
            
        if self.face_process and self.face_process.is_alive():
            self.face_process.terminate()
            self.face_process.join()
        if self.object_process and self.object_process.is_alive():
            self.object_process.terminate()
            self.object_process.join()
        print("[INFO] Processes stopped.")

    def is_running(self):
        """프로세스가 실행 중인지 확인하는 메서드"""
        return self.running and (
            (self.face_process and self.face_process.is_alive()) or 
            (self.object_process and self.object_process.is_alive())
        )

    def get_tools_status(self):
        """현재 도구 상태를 반환하는 메서드"""
        return dict(self.tools_status)

    def process_event_queue(self):
        """이벤트 큐에서 이벤트를 처리하는 메서드"""
        print("[INFO] 이벤트 처리 스레드 시작")
        while self.running:
            try:
                if not self.event_queue.empty():
                    event = self.event_queue.get(block=False)
                    print(f"[DEBUG] 이벤트 처리: {event['type']}")
                    
                    if event['type'] == 'tool-update':
                        if self.socketio:
                            self.socketio.emit('tool-update', event['payload'])
                            print(f"[DEBUG] 소켓 이벤트 전송: tool-update - {event['payload'].get('tool_name')}")
                        else:
                            print("[WARNING] socketio 객체가 없어 이벤트를 전송할 수 없습니다")
                else:
                    # 큐가 비어있으면 잠시 대기
                    time.sleep(0.1)
            except Exception as e:
                print(f"[ERROR] 이벤트 처리 중 오류: {e}")
                import traceback
                print(traceback.format_exc())
                time.sleep(1)  # 오류 발생 시 잠시 대기

if __name__ == "__main__":
    manager = MainManager()
    try:
        manager.start_processes()
        manager.face_process.join()
        manager.object_process.join()
    except KeyboardInterrupt:
        manager.stop_processes()