import cv2
import socket
import pickle
import struct
import numpy as np
import math

import rclpy as rp
from rclpy.node import Node
# from control_msgs.msg import ArucoPose

# ip = '192.168.0.6'
# port = 50001       

def rotMat2degree(rotation_matrix):
    sy = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        y = math.atan2(-rotation_matrix[2, 0], sy)
        z = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        x = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        y = math.atan2(-rotation_matrix[2, 0], sy)
        z = 0

    x = math.degrees(x)
    y = math.degrees(y)
    z = math.degrees(z)
    
    return x, y, z

# class ArucoPosePublisher(Node):
#     def __init__(self):
#         super().__init__('aruco_pose_publisher')
#         self.publisher = self.create_publisher(ArucoPose, '/aruco_pose', 10)
    
#     def send_aruco_pose(self, id, tvecs, angles):
#         msg = ArucoPose()
#         msg.id = id
#         msg.tvecs = tvecs
#         msg.angles = angles
#         self.publisher.publish(msg)


rp.init()
# aruco_pose_node = ArucoPosePublisher()
# src/circle/vision/calib_data/MultiMatrix.npz
calib_data_path = "circle/src/circle/vision/calib_data/MultiMatrix.npz"

with np.load(calib_data_path) as calib_data:
    print(calib_data.files)

    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]
    r_vectors = calib_data["rVector"]
    t_vectors = calib_data["tVector"]

marker_length = 0.04
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters_create()
# 소켓 네트워크 연결 부분 제거된 코드

# Aruco 마커 검출 및 프레임 처리 루프
while True:
    # 프레임을 직접 받아오는 방식으로 변경해야 함 (예: OpenCV로 카메라에서 직접 읽기)
    # frame = cv2.VideoCapture(0).read()[1]  # 예시 코드, 실제 사용 방식에 맞게 수정 필요

    # Aruco 마커 검출
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        # Aruco 마커의 3D 위치 및 자세 추정
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, cam_mat, dist_coef)

        for i in range(len(ids)):
            # 검출된 마커의 좌표계 및 마커 표시
            cv2.drawFrameAxes(frame, cam_mat, dist_coef, rvecs[i], tvecs[i], marker_length)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # 회전 벡터를 회전 행렬로 변환
            rotation_matrix, _ = cv2.Rodrigues(rvecs[i])

            # 회전 행렬을 오일러 각도로 변환
            rotation_degree = rotMat2degree(rotation_matrix)

            # 변환된 값 출력
            print(f'Rotation : {rotation_matrix}, Translation : {tvecs[i]}')
            print(f'angle : {rotation_degree}')

            # ROS2 퍼블리시 (주석 처리된 부분을 활성화하면 ROS2 메시지를 전송할 수 있음)
            # aruco_pose_node.send_aruco_pose(ids[i], tvecs[i], rotation_degree)

    # 영상 출력
    cv2.imshow('Frame', frame)

    # 'q' 키를 누르면 종료
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# 모든 창 닫기
cv2.destroyAllWindows()



