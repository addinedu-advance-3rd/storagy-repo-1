import multiprocessing
from .face_detect import FaceDetect
from .tools_detect import ObjectDetect
import time
import threading
import cv2

class MainManager:
    def __init__(self, socketio=None):
        self.manager = multiprocessing.Manager()
        self.latest_worker = self.manager.Value("s", "No Match")  # 공유 메모리 생성 (초기값: "No Match")
        self.face_process = None
        self.object_process = None
        self.socketio = socketio
        self.monitor_thread = None
        self.running = False
        self.tools_status = self.manager.dict()  # 공구 상태를 저장할 공유 딕셔너리 추가

    def run_face_detection(self):
        face_detector = FaceDetect(self.latest_worker)
        face_detector.recognize_face()

    def run_object_detection(self):
        object_detector = ObjectDetect(self.latest_worker, tools_status=self.tools_status)
        object_detector.detect_objects()

    def monitor_latest_worker(self):
        last_value = self.latest_worker.value
        last_tools_status = {}
        
        while self.running:
            current_value = self.latest_worker.value
            current_tools_status = dict(self.tools_status)
            
            # 작업자 변경 감지
            if current_value != last_value:
                if self.socketio:
                    self.socketio.emit('worker_update', {'worker': current_value})
                last_value = current_value
                
            # 공구 상태 변경 감지
            if current_tools_status != last_tools_status:
                if self.socketio:
                    self.socketio.emit('tools_update', {'tools': current_tools_status})
                last_tools_status = current_tools_status
                
            time.sleep(0.1)  # 100ms마다 확인

    def start_processes(self):
        print("[INFO] Starting FaceDetect and ObjectDetect processes...")
        self.face_process = multiprocessing.Process(target=self.run_face_detection)
        self.object_process = multiprocessing.Process(target=self.run_object_detection)
        self.face_process.start()
        self.object_process.start()
        
        # 모니터링 스레드 시작
        self.running = True
        self.monitor_thread = threading.Thread(target=self.monitor_latest_worker)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def stop_processes(self):
        print("[INFO] Stopping processes...")
        self.running = False
        
        # 모니터링 스레드 종료
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1.0)
        
        # 얼굴 인식 프로세스 종료
        if hasattr(self, 'face_process') and self.face_process and self.face_process.is_alive():
            try:
                self.face_process.terminate()
                self.face_process.join(timeout=1.0)
            except Exception as e:
                print(f"[WARNING] Face process termination error: {e}")
        
        # 객체 감지 프로세스 종료
        if hasattr(self, 'object_process') and self.object_process and self.object_process.is_alive():
            try:
                self.object_process.terminate()
                self.object_process.join(timeout=1.0)
            except Exception as e:
                print(f"[WARNING] Object process termination error: {e}")
        
        print("[INFO] Processes stopped.")

    def is_running(self):
        """프로세스가 실행 중인지 확인하는 메서드"""
        return self.running and (
            (self.face_process and self.face_process.is_alive()) or 
            (self.object_process and self.object_process.is_alive())
        )

    def init_camera(self, camera_index):
        try:
            cap = cv2.VideoCapture(camera_index)
            if not cap.isOpened():
                print(f"카메라 {camera_index}를 열 수 없습니다. 다른 인덱스 시도...")
                # 다른 카메라 인덱스 시도
                for i in range(10):  # 0부터 9까지 시도
                    if i != camera_index:
                        cap = cv2.VideoCapture(i)
                        if cap.isOpened():
                            print(f"대체 카메라 {i}를 사용합니다.")
                            return cap
                # 여전히 실패한 경우
                print("사용 가능한 카메라를 찾을 수 없습니다.")
                return None
            return cap
        except Exception as e:
            print(f"카메라 초기화 중 오류: {e}")
            return None

if __name__ == "__main__":
    manager = MainManager()
    try:
        manager.start_processes()
        manager.face_process.join()
        manager.object_process.join()
    except KeyboardInterrupt:
        manager.stop_processes()