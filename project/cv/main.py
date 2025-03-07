import multiprocessing
from .face_detect import FaceDetect
from .tools_detect_test import ObjectDetect

class MainManager:
    def __init__(self):
        self.manager = multiprocessing.Manager()
        self.latest_worker = self.manager.Value("s", "No Match")  # 공유 메모리 생성 (초기값: "No Match")
        self.stop_event = multiprocessing.Event()
        self.face_process = None
        self.object_process = None
        # 공유 자원에 대한 동기화를 위한 Lock
        self.lock = self.manager.Lock()

    def run_face_detection(self):
        face_detector = FaceDetect(self.latest_worker, self.lock)
        face_detector.recognize_face(self.stop_event)

    def run_object_detection(self):
        object_detector = ObjectDetect(self.latest_worker, self.lock)
        object_detector.detect_objects(self.stop_event)

    def start_processes(self):
        print("[INFO] Starting FaceDetect and ObjectDetect processes...")
        self.face_process = multiprocessing.Process(target=self.run_face_detection)
        self.object_process = multiprocessing.Process(target=self.run_object_detection)
        self.face_process.start()
        self.object_process.start()

    def stop_processes(self):
        print("[INFO] Stopping processes...")
        self.stop_event.set()
        print("[INFO] Processes stopped.")

'''
if __name__ == "__main__":
    manager = MainManager()
    try:
        manager.start_processes()
        manager.face_process.join()
        manager.object_process.join()
    except KeyboardInterrupt:
        manager.stop_processes()
'''