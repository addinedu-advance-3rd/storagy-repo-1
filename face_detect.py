import cv2
import json
import os
import numpy as np
import torch
from facenet_pytorch import MTCNN, InceptionResnetV1
from torchvision import transforms


#얼굴 인식만 하는 클래스 json에서 데이터 로드
class FaceDetect:
    IMG_SRC_FOLDER = "img_src"  # 얼굴 데이터가 저장될 폴더
    METADATA_PATH = os.path.join(IMG_SRC_FOLDER, "face_metadata.json")  # 얼굴 메타데이터 경로

    def __init__(self):
        # CUDA 사용 여부 확인
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if torch.cuda.is_available():
            print(f"Using GPU: {torch.cuda.get_device_name(0)}")
        else:
            print("Using CPU")
        self.mtcnn = MTCNN(keep_all=False, device=self.device)  # 하나의 얼굴만 감지
        #얼굴비교모델 facenet
        self.facenet = InceptionResnetV1(pretrained='vggface2').eval().to(self.device)
        self.reference_embeddings = self.load_embeddings()  # 기존 얼굴 데이터 로드

    #임베딩 정보 로드
    def load_embeddings(self):
        if os.path.exists(self.METADATA_PATH):
            with open(self.METADATA_PATH, "r", encoding="utf-8") as file:
                try:
                    metadata = json.load(file)
                except json.JSONDecodeError:
                    metadata = {}
        else:
            metadata = {}
        return {id: np.array(data["embedding"]) for id, data in metadata.items()}

    #얼굴 감지 및 임베딩 추출
    def extract_embedding(self, image):
        boxes, _ = self.mtcnn.detect(image)
        if boxes is not None and len(boxes) > 0:
            x1, y1, x2, y2 = [int(b) for b in boxes[0]]
            face = image[max(y1, 0):min(y2, image.shape[0]), max(x1, 0):min(x2, image.shape[1])]
            if face.size == 0:
                return None, None
            transform = transforms.Compose([
                transforms.ToPILImage(),
                transforms.Resize((160, 160)),
                transforms.ToTensor(),
                transforms.Normalize([0.5, 0.5, 0.5], [0.5, 0.5, 0.5])
            ])
            face_tensor = transform(face).unsqueeze(0).to(self.device)
            embedding = self.facenet(face_tensor).detach().cpu().numpy().flatten()
            return embedding, boxes[0]
        return None, None

    #얼굴비교 -> 유클리드 거리 계산
    def calculate_distance(self, embedding1, embedding2):
        return np.linalg.norm(embedding1 - embedding2)

    # main -> 캠 열고 매칭
    def recognize_face(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            print("Failed to open webcam.")
            return

        print("Press 'q' to quit.")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Failed to read from webcam.")
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            embedding, box = self.extract_embedding(frame_rgb)

            if embedding is not None:
                best_match = "No Match"
                min_distance = float('inf')

                #저장된 얼굴 데이터랑 현재 얼굴의 임베딩을 비교하고, 가장 가까운 작업자 찾기
                for user_id, ref_embedding in self.reference_embeddings.items():
                    distance = self.calculate_distance(ref_embedding, embedding)
                    if distance < min_distance:
                        min_distance = distance
                        best_match = user_id if min_distance < 0.7 else "No Match"

                #바운딩 박스 및 텍스트 표시
                x1, y1, x2, y2 = [int(b) for b in box]
                color = (0, 255, 0) if best_match != "No Match" else (0, 0, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, best_match, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                print(f"최근 사용자: {best_match}")

            cv2.imshow("Face Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = FaceDetect()
    detector.recognize_face()
