import cv2
import torch
import logging
import numpy as np
from ultralytics import YOLO
from collections import defaultdict
from math import atan2, degrees  # ✅ 각도 계산을 위한 추가

# ✅ YOLO의 불필요한 로그 메시지 제거
logging.getLogger("ultralytics").setLevel(logging.CRITICAL)

# ✅ YOLO 모델 로드 (CUDA 사용)
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO("runs/detect/fod_ver1/weights/best.pt").to(device)

# ✅ Homography 행렬 (실제 값으로 교체 필요)
homography_matrix = np.array([
    [-1.33501421e-01,  8.38672822e-03,  4.06410103e+01],
    [-3.71847207e-03,  4.26407866e-02, -7.44009040e+01],
    [ 4.94951733e-05, -4.08950562e-03,  1.00000000e+00]
])

# ✅ Homography 변환을 이용한 거리 추정 함수
def estimate_distance(cx, cy):
    """ 픽셀 좌표 (cx, cy)를 실세계 거리로 변환 """
    pixel_point = np.array([[[cx, cy]]], dtype=np.float32)  # 형태 맞추기
    real_point = cv2.perspectiveTransform(pixel_point, homography_matrix)[0][0]  # ✅ 올바른 변수명 사용
    real_x, real_y = real_point[0], real_point[1]
    distance = (real_x ** 2 + real_y ** 2) ** 0.5  # 유클리드 거리 계산
    return distance

def estimate_real_angle(cx, cy, homography_matrix):
    """ 실제 거리 기반 실각도 계산 """
    
    # ✅ 픽셀 좌표 → 실좌표 변환 (Homography 사용)
    pixel_point = np.array([[[cx, cy]]], dtype=np.float32)
    real_point = cv2.perspectiveTransform(pixel_point, homography_matrix)[0][0]
    real_x, real_y = real_point[0], real_point[1]  # 실세계 좌표

    # ✅ 카메라의 실제 위치 (가정: 중앙에 위치, 높이 0)
    camera_x, camera_y = 0, 0  # (필요하면 카메라 높이 추가 가능)

    # ✅ 실세계 거리 변화량 계산
    dx_real = real_x - camera_x  # 가로 거리 차이
    dy_real = real_y - camera_y  # 세로 거리 차이

    # ✅ 실각도 계산 (atan2 사용)
    real_angle = degrees(atan2(dx_real, dy_real))

    return real_angle

# ✅ 각도 추정 함수
def estimate_angle(cx, cy, max_y):
    """ 바운딩 박스 중심점 (cx, cy)과 바닥 중앙 (320, max_y) 간의 각도 계산 """
    angle = degrees(atan2(cx - 320, max_y - cy))  # ✅ 카메라 중앙(320, max_y) 기준 각도 계산
    return angle

# ✅ 웹캠 열기
cap = cv2.VideoCapture("http://192.168.0.6:5000/video_feed")
if not cap.isOpened():
    print("❌ 웹캠을 열 수 없습니다.")
    exit()

# ✅ 감지된 객체 저장 (프레임 누락 방지)
detected_objects = defaultdict(lambda: {"count": 0, "bbox": None, "distance": None, "angle": None})

# ✅ 감지를 유지할 프레임 수 (예: 10프레임)
max_missed_frames = 10

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("❌ 웹캠에서 프레임을 읽을 수 없습니다.")
        break

    # ✅ 프레임의 바닥 좌표 (max_y)
    max_y = 480  # ✅ 프레임의 높이 (바닥 좌표)

    # ✅ YOLO 감지 수행
    results = model(frame, conf=0.2)

    # ✅ 현재 프레임에서 감지된 객체 추적
    current_objects = set()

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # 중심 좌표 계산
            
            # ✅ y 좌표 필터 적용 (바닥에 있는 객체만 감지)
            if cy > 250:  
                distance = estimate_distance(cx, cy)  # ✅ Homography 변환 적용
                angle = estimate_real_angle(cx, cy, homography_matrix)  # ✅ 각도 계산
                
                label = f"FOD {distance:.2f} cm, Angle {angle:.2f}°"  # 거리 & 각도 정보 추가
                print(f"FOD {distance:.2f} cm | Angle {angle:.2f}'")

                # ✅ 현재 감지된 객체 저장
                current_objects.add(label)
                detected_objects[label]["count"] = max_missed_frames  # 유지 시간 초기화
                detected_objects[label]["bbox"] = (x1, y1, x2, y2)
                detected_objects[label]["distance"] = distance
                detected_objects[label]["angle"] = angle

    # ✅ 탐지가 끊긴 객체도 일정 시간 유지
    for obj, data in list(detected_objects.items()):
        if data["count"] > 0:
            x1, y1, x2, y2 = data["bbox"]
            distance = data["distance"]
            angle = data["angle"]

            # ✅ 감지된 객체 바운딩 박스 및 정보 표시
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"D: {distance:.2f} cm", (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Angle: {angle:.2f}'", (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            detected_objects[obj]["count"] -= 1  # 유지 시간 감소

    # ✅ 화면 출력
    cv2.imshow("FOD Detection - YOLOv8", frame)

    # ✅ 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
