import cv2
import torch
from ultralytics import YOLO
from collections import defaultdict

# ✅ YOLO 모델 로드
model = YOLO("runs/detect/fod_ver1/weights/best.pt")

# ✅ 특정 클래스만 감지 (Bolt, Nut, Nail)
target_classes = [1, 5, 6]  # Sheet(4) 없음

# ✅ 웹캠 열기
cap = cv2.VideoCapture("http://192.168.0.6:5000/video_feed")
if not cap.isOpened():
    print("❌ 웹캠을 열 수 없습니다.")
    exit()

# ✅ 감지된 객체 저장 (프레임 누락 방지)
detected_objects = defaultdict(lambda: {"count": 0, "bbox": None})

# ✅ 감지를 유지할 프레임 수 (예: 10프레임)
max_missed_frames = 10

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("❌ 웹캠에서 프레임을 읽을 수 없습니다.")
        break

    # ✅ YOLO 감지 수행 (❗ 특정 클래스만 감지)
    results = model(frame, conf=0.2, classes=target_classes)

    # ✅ 현재 프레임에서 감지된 객체 추적
    current_objects = set()

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
            class_id = int(box.cls[0])  # 클래스 ID

            # ✅ 모든 감지된 객체를 "FOD"로 표기
            label = "FOD"

            # ✅ 현재 감지된 객체 저장
            current_objects.add(label)
            detected_objects[label]["count"] = max_missed_frames  # 유지 시간 초기화
            detected_objects[label]["bbox"] = (x1, y1, x2, y2)

    # ✅ 탐지가 끊긴 객체도 일정 시간 유지
    for obj, data in detected_objects.items():
        if data["count"] > 0:
            x1, y1, x2, y2 = data["bbox"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{obj}", (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            detected_objects[obj]["count"] -= 1  # 유지 시간 감소

    # ✅ 화면 출력
    cv2.imshow("FOD Detection - YOLOv8", frame)

    # ✅ 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
