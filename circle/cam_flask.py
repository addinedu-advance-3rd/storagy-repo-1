import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time
from flask import Flask, Response, render_template
app = Flask(__name__)
# :white_check_mark: ROS2 이미지 토픽을 구독할 전역 변수
global_frame = None
global_depth_frame = None
frame_lock = threading.Lock()
class CameraSubscriber(Node):
    """ROS2 카메라 토픽을 구독하여 global_frame에 최신 프레임 저장"""
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # :white_check_mark: 카메라 이미지 토픽을 구독
            self.image_callback,
            10)


        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',  # :white_check_mark: 카메라 이미지 토픽을 구독
            self.depth_image_callback,
            10)

        self.get_logger().info(":white_check_mark: ROS2 카메라 구독 시작")
    def image_callback(self, msg):
        """ROS2 Image 메시지를 OpenCV 프레임으로 변환 후 저장"""
        global global_frame
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # ROS2 → OpenCV 변환
            _, buffer = cv2.imencode('.jpg', frame)  # JPEG 변환
            with frame_lock:
                global_frame = buffer.tobytes()  # 프레임 저장
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")
    def depth_image_callback(self, msg):
        """ROS2 Depth Image 메시지를 OpenCV 프레임으로 변환 후 저장"""
        global global_depth_frame
        try:
            # ✅ Depth 이미지는 16-bit unsigned int (16UC1) 또는 32-bit float (32FC1)로 변환해야 함
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")  
            
            # 16-bit 이미지를 8-bit로 변환 (보이는 이미지로 만들기 위해 정규화)
            frame_normalized = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
            frame_8bit = frame_normalized.astype('uint8')

            # ✅ 컬러맵 적용 (Heatmap 스타일)
            frame_colormap = cv2.applyColorMap(frame_8bit, cv2.COLORMAP_JET)

            # JPEG 변환
            _, buffer = cv2.imencode('.jpg', frame_colormap)  
            with frame_lock:
                global_depth_frame = buffer.tobytes()  # 프레임 저장
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")

def gen_frames():
    """Flask에서 global_frame을 HTTP 스트리밍"""
    while True:
        with frame_lock:
            frame = global_frame
        if frame is None:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.001)  # CPU 부담을 줄이기 위한 대기 시간
def gen_depth_frames():
    """Flask에서 global_depth_frame을 HTTP 스트리밍"""
    while True:
        with frame_lock:
            frame = global_depth_frame
        if frame is None:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.001)  # CPU 부담을 줄이기 위한 대기 시간
@app.route('/')
def index():
    """웹 브라우저에서 기본 페이지 렌더링"""
    return render_template('index.html')
@app.route('/video_feed')
def video_feed():
    """Flask에서 비디오 스트리밍"""
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/depth_feed')
def depth_feed():
    """Flask에서 디퍼드 스트리밍"""
    return Response(gen_depth_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
def flask_thread():
    """Flask 서버를 별도의 스레드에서 실행"""
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
def main(args=None):
    """ROS2 노드 실행 및 Flask 웹 서버 시작"""
    rclpy.init(args=args)
    node = CameraSubscriber()
    # Flask를 별도 스레드에서 실행
    flask_threading = threading.Thread(target=flask_thread)
    flask_threading.daemon = True
    flask_threading.start()
    try:
        rclpy.spin(node)  # ROS2 노드 실행
    except KeyboardInterrupt:
        node.get_logger().info("노드 종료 요청됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
