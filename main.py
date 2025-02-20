import multiprocessing
from face_detect import FaceDetect
from tools_detect import ObjectDetect

def run_face_detection(latest_worker):
    face_detector = FaceDetect(latest_worker)
    face_detector.recognize_face()

def run_object_detection(latest_worker):
    object_detector = ObjectDetect(latest_worker)
    object_detector.detect_objects()

if __name__ == "__main__":
    manager = multiprocessing.Manager()
    latest_worker = manager.Value("s", "No Match")  # 공유 메모리 생성 (초기값: "No Match")

    # 멀티프로세싱을 사용하여 두 개의 기능을 동시에 실행
    face_process = multiprocessing.Process(target=run_face_detection, args=(latest_worker,))
    object_process = multiprocessing.Process(target=run_object_detection, args=(latest_worker,))

    face_process.start()
    object_process.start()

    face_process.join()
    object_process.join()
