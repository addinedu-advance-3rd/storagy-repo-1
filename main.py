import cv2
import threading
from rgb_cam import RGBCamera
from depth_cam import DepthCamera
from align_rgb_to_depth import RGBDepthAligner  # ✅ 정렬 모듈 추가

class FODDetector:
    def __init__(self):
        self.rgb_camera = RGBCamera()
        self.depth_camera = DepthCamera()
        self.aligner = RGBDepthAligner()  # ✅ 정렬 클래스 추가
        self.running = True

    def process_stream(self):
        while self.running:
            frame = self.rgb_camera.get_frame()
            if frame is None:
                continue

            fod_objects = self.rgb_camera.detect_fod(frame)
            depth_frame = self.depth_camera.get_depth_frame()
            if depth_frame is None:
                continue

            for (cx, cy, x1, y1, x2, y2) in fod_objects:
                cx_depth, cy_depth = self.aligner.transform_rgb_to_depth(cx, cy)
                distance = self.depth_camera.get_distance(depth_frame, cx_depth, cy_depth)

                if distance is not None:
                    text = f"{distance:.2f} mm"
                    cv2.putText(frame, text, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, "FOD", (x1, y1 - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("FOD Detection & Distance", frame)
            cv2.imshow("Depth View", depth_frame)  # ✅ Depth도 표시

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

    def start(self):
        self.thread = threading.Thread(target=self.process_stream)
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()
        self.rgb_camera.release()
        self.depth_camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    fod_detector = FODDetector()
    try:
        fod_detector.start()
        fod_detector.thread.join()
    except KeyboardInterrupt:
        fod_detector.stop()
