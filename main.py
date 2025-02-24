import multiprocessing
from face_detect import FaceDetect
from tools_detect_test import ObjectDetect

class MainManager:
    def __init__(self):
        self.manager = multiprocessing.Manager()
        self.latest_worker = self.manager.Value("s", "No Match")  # 공유 메모리 생성 (초기값: "No Match")
        self.face_process = None
        self.object_process = None

    def run_face_detection(self):
        face_detector = FaceDetect(self.latest_worker)
        face_detector.recognize_face()

    def run_object_detection(self):
        object_detector = ObjectDetect(self.latest_worker)
        object_detector.detect_objects()

    def start_processes(self):
        print("[INFO] Starting FaceDetect and ObjectDetect processes...")
        self.face_process = multiprocessing.Process(target=self.run_face_detection)
        self.object_process = multiprocessing.Process(target=self.run_object_detection)
        
        self.face_process.start()
        self.object_process.start()

    def stop_processes(self):
        print("[INFO] Stopping processes...")
        if self.face_process and self.face_process.is_alive():
            self.face_process.terminate()
            self.face_process.join()
        if self.object_process and self.object_process.is_alive():
            self.object_process.terminate()
            self.object_process.join()
        print("[INFO] Processes stopped.")

if __name__ == "__main__":
    manager = MainManager()
    try:
        manager.start_processes()
        manager.face_process.join()
        manager.object_process.join()
    except KeyboardInterrupt:
        manager.stop_processes()