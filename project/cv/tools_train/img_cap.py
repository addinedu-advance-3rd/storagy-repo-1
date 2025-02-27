import cv2
import os

# ✅ 저장할 폴더 경로 설정
save_dir = "/home/addinedu/Downloads/tools2"
os.makedirs(save_dir, exist_ok=True)  # 폴더 없으면 생성

# ✅ 웹캠 열기 (카메라 인덱스 2번)
cap = cv2.VideoCapture(2)

frame_count = 0  # 프레임 카운트
image_count = 0  # 저장된 이미지 개수

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1  # 프레임 증가

    # ✅ 30프레임마다 한 장 캡처
    if frame_count % 45 == 0:
        image_count += 1
        image_path = os.path.join(save_dir, f"image_{image_count:04d}.jpg")
        cv2.imwrite(image_path, frame)
        print(f"📸 캡처: {image_path}")

    # ✅ 현재 화면 표시
    cv2.imshow("Webcam Capture", frame)

    # ✅ 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ✅ 자원 해제
cap.release()
cv2.destroyAllWindows()
