from ultralytics import YOLO
import torch

# ✅ YOLOv8 세그멘테이션 모델 로드
model = YOLO('yolov8n-seg.pt')  # 필요한 경우 'yolov8s-seg.pt' 사용 가능

# ✅ CUDA 사용 여부 확인 및 설정
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
if torch.cuda.is_available():
    print(f"🚀 Using GPU: {torch.cuda.get_device_name(0)}")
else:
    print("⚡ Using CPU")

# ✅ 모델 학습 시작
model.train(
    data="/home/addinedu/venv/facedetection/tools_train/tools2.v1i.yolov8/data.yaml",  # 데이터셋 설정 파일
    epochs=40,  # 학습 횟수 (필요에 따라 조정 가능)
    imgsz=640,  # 입력 이미지 크기
    batch=8,  # 배치 크기 (GPU VRAM에 맞게 조정, 부족하면 4로 변경)
    device=device,  # 자동으로 GPU/CPU 선택
    name="tools_ver2",  # 저장되는 모델 폴더명 (runs/segment/tools_ver2/)
    val=True  # 검증 데이터셋 사용하여 성능 평가 (기본값 True, 명시적으로 추가)
)
