import cv2
from facenet_pytorch import MTCNN, InceptionResnetV1
import torch
from torchvision import transforms
import numpy as np
import os
import json
import time
import uuid
import hashlib


# CUDA 사용 여부 확인
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
if torch.cuda.is_available():
    print(f"Using GPU: {torch.cuda.get_device_name(0)}")
else:
    print("Using CPU")

# 고유 ID 안써도될듯
# # 해시된 UUID 생성 함수 
# def generate_hashed_uuid(length=8):
#     full_uuid = str(uuid.uuid4())
#     hash_object = hashlib.sha256(full_uuid.encode())
#     hashed_uuid = hash_object.hexdigest()[:length]  # 원하는 길이로 자름
#     return hashed_uuid
    
# 모델 초기화
mtcnn = MTCNN(keep_all=False, device=device)  # 얼굴 검출용
facenet = InceptionResnetV1(pretrained='vggface2').eval().to(device)  # 얼굴 임베딩 추출용

# 얼굴 비교 함수 (유클리드 거리 계산)
def calculate_distance(embedding1, embedding2):
    return np.linalg.norm(embedding1 - embedding2)

# 얼굴 임베딩 및 바운딩 박스 추출 함수
def extract_embedding_and_boxes(image):
    boxes, _ = mtcnn.detect(image)
    if boxes is not None and len(boxes) > 0:  # 얼굴이 검출된 경우
        x1, y1, x2, y2 = [int(b) for b in boxes[0]]
        x1, y1 = max(x1, 0), max(y1, 0)
        x2, y2 = min(x2, image.shape[1]), min(y2, image.shape[0])
        face = image[y1:y2, x1:x2]
        if face.size == 0:
            print("Empty face region detected.")
            return None, None, None
        transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((160, 160)),
            transforms.ToTensor(),
            transforms.Normalize([0.5, 0.5, 0.5], [0.5, 0.5, 0.5])
        ])
        face_tensor = transform(face).unsqueeze(0).to(device)
        embedding = facenet(face_tensor).detach().cpu().numpy().flatten()
        return embedding, [boxes[0]], face  # 얼굴 이미지 추가 반환
    else:
        print("No face detected.")
        return None, None, None

# 새로운 얼굴의 임베딩 및 메타데이터 저장
def save_new_face_and_embedding(embedding, folder_path, metadata,reference_embeddings):
    # user_name = input("이름을 입력하세요: ")  # 사용자로부터 이름 입력
    # user_id = generate_hashed_uuid(4)  # 4자리 해시된 UUID 생성

    # JSON 파일 경로
    metadata_path = os.path.join(folder_path, "face_metadata.json")

    
    #이름 자동 생성. 근로자 1,2...
    user_name = f"Worker{len(metadata) + 1}"

    # 새로운 데이터 추가
    metadata[user_name] = {
        "embedding": embedding.tolist()
    }

    
    # JSON 파일에 저장
    with open(metadata_path, "w", encoding="utf-8") as file:
        json.dump(metadata, file, indent=4, ensure_ascii=False)

    # reference_embeddings에 즉시 반영
    reference_embeddings[user_name] = embedding

    print(f"{user_name}님의 임베딩 정보와 분석 결과가 저장되었습니다.")

# 폴더 내 모든 이미지 로드 및 임베딩 추출
def load_embeddings_from_folder(folder_path):
    metadata_path = os.path.join(folder_path, "face_metadata.json")

    # JSON 파일이 없으면 빈 파일 생성 후 빈 딕셔너리 반환
    if not os.path.exists(metadata_path):
        print("Metadata file not found. Creating a new one...")
        with open(metadata_path, "w", encoding="utf-8") as file:
            json.dump({}, file, indent=4, ensure_ascii=False)
        return {}

    # JSON 파일 읽기
    try:
        with open(metadata_path, "r", encoding="utf-8") as file:
            metadata = json.load(file)
    except json.JSONDecodeError:
        print("Metadata file is corrupted. Reinitializing...")
        metadata = {}
        with open(metadata_path, "w", encoding="utf-8") as file:
            json.dump(metadata, file, indent=4, ensure_ascii=False)

    # 임베딩 데이터 로드 
    return {user_id: np.array(data["embedding"]) for user_id, data in metadata.items()}




# 기준 이미지 폴더 설정
img_src_folder = 'img_src'
os.makedirs(img_src_folder, exist_ok=True)

# 메타데이터 로드
reference_embeddings = load_embeddings_from_folder(img_src_folder)
if not reference_embeddings:
    print("No valid face embeddings found in the folder.")
else:
    print(f"Loaded {len(reference_embeddings)} reference embeddings.")

# 웹캠으로 실시간 얼굴 검출 및 비교
cap = cv2.VideoCapture(0)

# 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Failed to open webcam.")
    exit()

print("Press 'q' to quit.")
match_start_time = None  # 매칭된 시간을 저장
no_match_start_time = None  # 매칭되지 않은 시간을 기록
matched = False  # 매칭 상태를 저장
current_user = None # 사용자 변경시 초기화 하는 플래그

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Failed to read from webcam.")
        break

    # BGR -> RGB 변환
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    embedding, boxes, face_image = extract_embedding_and_boxes(frame_rgb)

    if embedding is not None: # 얼굴이 있는 상태
        best_match = "No Match"
        min_distance = float('inf')

        # 기준 얼굴들과 비교
        for file_id, ref_embedding in reference_embeddings.items():
            distance = calculate_distance(ref_embedding, embedding)
            if distance < min_distance:
                min_distance = distance
                best_match = file_id if min_distance < 0.7 else "No Match"

        if best_match != "No Match":
            matched_name = best_match
            if not matched:
                print(f"{matched_name}님 환영합니다. 3초 뒤 종료됩니다.")
                current_user = matched_name # todo: id 로 저장해야됨
                match_start_time = time.time()
                matched = True
                no_match_start_time = None

            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    x1, y1, x2, y2 = [int(b) for b in box]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{matched_name}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # 3초 후 종료
            if current_user == matched_name:
                if matched and time.time() - match_start_time > 3:
                    print(f"{matched_name}님 환영합니다. 이제 종료합니다.")
                    break  # 루프 탈출 조건 추가
            else:
                # 현재 사용자와 매칭된 사용자가 달라지면 초기화
                print(f"사용자가 변경되었습니다: 이전 사용자: {current_user}, 새 사용자: {matched_name}")
                current_user = None
                match_start_time = None
                matched = False        

        else: # best_match = "No Match"
            matched = False # 매칭이 실패한 경우 상태 초기화
            if no_match_start_time is None:
                no_match_start_time = time.time()
            
            # 매칭되지 않고 2초 이상 경과한 경우
            if no_match_start_time and time.time() - no_match_start_time > 2:
                print("새로운 얼굴이 감지되었습니다. 임베딩 정보를 저장합니다.")

                # 얼굴 재확인
                embedding, boxes, face_image= extract_embedding_and_boxes(frame_rgb)
                if embedding is not None and boxes is not None and len(boxes) > 0:
                    print("얼굴이 확인되었습니다. 이미지를 저장합니다.")

                    metadata = load_embeddings_from_folder(img_src_folder)

                    save_new_face_and_embedding(embedding, img_src_folder, metadata,reference_embeddings)
                    
                    
                    no_match_start_time = None
                else:
                    print("얼굴이 사라졌습니다. NO MATCH ")
                    no_match_start_time = None
                no_match_start_time = None
            elif embedding is not None and boxes is not None and len(boxes) > 0:
                # 얼굴이 다시 확인되면 NO MATCH 상태 초기화
                print("얼굴이 다시 확인되었습니다.")

            # 매칭되지 않은 경우 바운딩 박스 표시
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    x1, y1, x2, y2 = [int(b) for b in box]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, "No Match", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    else:
        print("No face detected.")
        no_match_start_time = None
        match_start_time = None

    cv2.imshow("Webcam Face Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()