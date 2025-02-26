import cv2
import torch
import numpy as np
from ultralytics import YOLO

# ✅ YOLO 모델 로드
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = YOLO("runs/segment/tools_training/weights/best.pt").to(device)

# ✅ 클래스 정보
class_names = ["Driver", "Hammer", "Spanner"]

# ✅ 비디오 스트림 사용 (카메라 인덱스 2번)
cap = cv2.VideoCapture(2)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # ✅ YOLO 감지 실행 (conf=0.7 설정해서 불필요한 감지 줄이기)
    results = model(frame, conf=0.6)

    for result in results:
        if result.masks is not None:
            for mask, cls in zip(result.masks.xy, result.boxes.cls):
                mask = np.array(mask, dtype=np.int32)  # 다각형 좌표 정수 변환
                class_id = int(cls)  # 클래스 ID 정수 변환
                
                # ✅ 거미줄 문제 해결: 너무 작은 폴리곤 무시
                if len(mask) < 10:  # 점 개수가 10개 미만이면 거미줄 가능성 높음 → 무시
                    continue
                
                # ✅ 거미줄 문제 해결: 연결된 점들의 거리 체크 (너무 길면 제외)
                distances = np.linalg.norm(mask - np.roll(mask, shift=1, axis=0), axis=1)
                if np.any(distances > 200):  # 두 점 사이 거리가 200px 이상이면 거미줄일 가능성 높음
                    continue

                # ✅ 외곽선(폴리곤) 그리기 (초록색)
                cv2.polylines(frame, [mask], isClosed=True, color=(0, 255, 0), thickness=2)

                # ✅ 객체 중심 계산 후 클래스명 표시
                M = cv2.moments(mask)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.putText(frame, class_names[class_id], (cX, cY - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

    # ✅ 결과 프레임 표시
    cv2.imshow("YOLO Instance Segmentation - Improved Polygon", frame)

    # ✅ 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
