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

bp = Blueprint('map', __name__, url_prefix='/map')

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

# 지도 상태 저장용 변수
map_image_data = create_default_map_image()
robot_position = {'x': 0, 'y': 0, 'theta': 0}

@bp.route('/')
def view():
    return render_template('map/map_view.html')

# 지도 이미지 직접 제공
@bp.route('/map-image')
def map_image():
    try:
        global map_image_data
        
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
        return "지도 이미지를 가져올 수 없습니다.", 404

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