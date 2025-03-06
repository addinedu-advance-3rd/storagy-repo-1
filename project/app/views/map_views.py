from flask import Blueprint, render_template, request, Response, current_app
import requests
import json
import os
from flask_socketio import emit
from app import socketio
import threading
import io
from PIL import Image
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

bp = Blueprint('map', __name__, url_prefix='/map')

# 전역 변수
map_image_data = None
robot_position = {'x': 0, 'y': 0, 'theta': 0}
ros_initialized = False
ros_node = None

# ROS 2 노드 클래스
class MapNode(Node):
    def __init__(self):
        super().__init__('web_map_node')
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        self.get_logger().info('Map node initialized')

    def map_callback(self, msg):
        global map_image_data
        self.get_logger().info('Map received')
        
        # OccupancyGrid를 이미지로 변환
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape(height, width)
        
        # -1(unknown)은 회색, 0(free)은 흰색, 100(occupied)은 검은색으로 변환
        rgb_data = np.zeros((height, width, 3), dtype=np.uint8)
        rgb_data[data == -1] = [128, 128, 128]  # 회색
        rgb_data[data == 0] = [255, 255, 255]   # 흰색
        rgb_data[data == 100] = [0, 0, 0]       # 검은색
        
        # PIL Image로 변환
        pil_image = Image.fromarray(rgb_data)
        
        # 바이트 스트림으로 변환
        img_byte_arr = io.BytesIO()
        pil_image.save(img_byte_arr, format='PNG')
        img_byte_arr.seek(0)
        
        # 전역 변수 업데이트
        map_image_data = img_byte_arr.getvalue()
        
        # 웹소켓으로 지도 업데이트 알림
        socketio.emit('map_updated', namespace='/map')

    def pose_callback(self, msg):
        global robot_position
        pose = msg.pose.pose
        
        # 로봇 위치 업데이트
        robot_position = {
            'x': pose.position.x,
            'y': pose.position.y,
            'theta': 2 * np.arctan2(pose.orientation.z, pose.orientation.w)
        }
        
        # 웹소켓으로 위치 정보 전송
        forward_robot_pose(robot_position)

# 기본 지도 이미지 생성 (검은색 배경에 격자무늬)
def create_default_map_image(width=800, height=600):
    # 검은색 배경 생성
    image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # 격자 간격
    grid_size = 50
    grid_color = (50, 50, 50)  # 어두운 회색
    
    # 수평선 그리기
    for y in range(0, height, grid_size):
        image[y, :] = grid_color
    
    # 수직선 그리기
    for x in range(0, width, grid_size):
        image[:, x] = grid_color
    
    # 중앙에 좌표축 표시 (빨간색 X축, 녹색 Y축)
    center_x, center_y = width // 2, height // 2
    
    # X축 (빨간색)
    image[center_y, center_x:center_x + 100] = (0, 0, 255)
    
    # Y축 (녹색)
    image[center_y - 100:center_y, center_x] = (0, 255, 0)
    
    # PIL Image로 변환
    pil_image = Image.fromarray(image)
    
    # 바이트 스트림으로 변환
    img_byte_arr = io.BytesIO()
    pil_image.save(img_byte_arr, format='PNG')
    img_byte_arr.seek(0)
    
    return img_byte_arr.getvalue()

# ROS 2 노드 실행 함수
def run_ros_node():
    global ros_initialized, ros_node
    try:
        if not ros_initialized:
            rclpy.init()
            ros_initialized = True
        
        ros_node = MapNode()
        rclpy.spin(ros_node)
    except Exception as e:
        print(f"ROS 노드 실행 중 오류 발생: {str(e)}")
    finally:
        if ros_node is not None:
            ros_node.destroy_node()

# 애플리케이션 시작 시 ROS 노드 시작
def start_ros_node():
    global map_image_data
    # 기본 지도 이미지 생성
    map_image_data = create_default_map_image()
    
    # ROS 노드 스레드 시작
    ros_thread = threading.Thread(target=run_ros_node, daemon=True)
    ros_thread.start()

# 애플리케이션 종료 시 ROS 종료
def shutdown_ros():
    global ros_initialized
    if ros_initialized:
        try:
            rclpy.shutdown()
            ros_initialized = False
        except Exception as e:
            print(f"ROS 종료 중 오류 발생: {str(e)}")

# 애플리케이션 시작 시 ROS 노드 시작
start_ros_node()

# 애플리케이션 종료 시 ROS 종료 등록
import atexit
atexit.register(shutdown_ros)

@bp.route('/')
def view():
    return render_template('map/map_view.html')

# 지도 이미지 직접 제공
@bp.route('/map-image')
def map_image():
    try:
        global map_image_data
        
        if map_image_data is None:
            map_image_data = create_default_map_image()
        
        # 디버깅 로그 추가
        print(f"지도 이미지 요청 처리: {len(map_image_data)} 바이트")
        
        return Response(
            map_image_data,
            content_type='image/png',
            headers={
                'Cache-Control': 'no-cache, no-store, must-revalidate',
                'Pragma': 'no-cache',
                'Expires': '0'
            }
        )
    except Exception as e:
        print(f"지도 이미지 처리 에러: {str(e)}")
        # 오류 발생 시 기본 이미지 반환
        default_image = create_default_map_image()
        return Response(
            default_image,
            content_type='image/png'
        )

# 지도 이미지 업데이트 API (다른 시스템에서 호출 가능)
@bp.route('/update-map', methods=['POST'])
def update_map():
    try:
        global map_image_data
        
        if 'image' in request.files:
            # 파일로 업로드된 경우
            file = request.files['image']
            map_image_data = file.read()
            return "지도 이미지가 업데이트되었습니다.", 200
        elif request.data:
            # 바이너리 데이터로 전송된 경우
            map_image_data = request.data
            return "지도 이미지가 업데이트되었습니다.", 200
        else:
            return "이미지 데이터가 없습니다.", 400
    except Exception as e:
        print(f"지도 이미지 업데이트 에러: {str(e)}")
        return "지도 이미지 업데이트 실패", 500

# 로봇 위치 업데이트 API
@bp.route('/update-robot-pose', methods=['POST'])
def update_robot_pose():
    try:
        global robot_position
        data = request.json
        
        if data and 'x' in data and 'y' in data:
            robot_position = {
                'x': data.get('x', 0),
                'y': data.get('y', 0),
                'theta': data.get('theta', 0)
            }
            # 웹소켓으로 클라이언트에 위치 정보 전송
            forward_robot_pose(robot_position)
            return "로봇 위치가 업데이트되었습니다.", 200
        else:
            return "유효하지 않은 위치 데이터입니다.", 400
    except Exception as e:
        print(f"로봇 위치 업데이트 에러: {str(e)}")
        return "로봇 위치 업데이트 실패", 500

# 웹소켓 이벤트 핸들러
@socketio.on('connect', namespace='/map')
def handle_connect():
    print("클라이언트가 /map 네임스페이스에 연결되었습니다.")
    # 연결 즉시 현재 로봇 위치 전송
    emit('robot_pose', robot_position)

@socketio.on('disconnect', namespace='/map')
def handle_disconnect():
    print("클라이언트가 /map 네임스페이스에서 연결 해제되었습니다.")

# 로봇 위치 업데이트 이벤트 (지도 서버에서 받은 데이터를 클라이언트에 전달)
def forward_robot_pose(data):
    socketio.emit('robot_pose', data, namespace='/map') 