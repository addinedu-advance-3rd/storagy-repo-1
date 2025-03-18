import cv2
import json
import os
import numpy as np
import torch
from facenet_pytorch import MTCNN, InceptionResnetV1
from torchvision import transforms
import multiprocessing

#얼굴 인식만 하는 클래스 json에서 데이터 로드
class FaceDetect:
    IMG_SRC_FOLDER = "/home/addinedu/venv/develop/project/cv/img_src"
    METADATA_PATH = os.path.join(IMG_SRC_FOLDER, "face_metadata.json")  # 얼굴 메타데이터 경로

    def __init__(self,latest_worker):
        # CUDA 사용 여부 확인
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if torch.cuda.is_available():
            print(f"Using GPU: {torch.cuda.get_device_name(0)}")
        else:
            print("Using CPU")
        self.mtcnn = MTCNN(keep_all=False, device=self.device)  # 하나의 얼굴만 감지
        #얼굴비교모델 facenet
        self.facenet = InceptionResnetV1(pretrained='vggface2').eval().to(self.device)
        self.reference_embeddings = self.load_embeddings()  # 기존 얼굴 데이터 로드
        self.latest_worker = latest_worker  # 공유 메모리 객체
        self.last_valid_worker = None  # "No Match"가 아닐 때의 최근 사용자 저장


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

        frame_count = 0 #프레임 카운터 -> 상태 업데이트 횟수 조정 

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
                        best_match = user_id if min_distance < 0.85 else "No Match"
                
                if best_match != "No Match":
                    self.last_valid_worker = best_match

                # 사용자 바뀔 때 상태 출력
                if frame_count % 15 == 0 and self.latest_worker.value != self.last_valid_worker:
                    self.latest_worker.value = self.last_valid_worker if self.last_valid_worker is not None else "No Match"
                    print(f"🔹 최근 감지된 사용자: {self.latest_worker.value}")
                
                frame_count += 1



                #바운딩 박스 및 텍스트 표시
                x1, y1, x2, y2 = [int(b) for b in box]
                color = (0, 255, 0) if best_match != "No Match" else (0, 0, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, best_match, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                

            cv2.imshow("Face Detection", frame)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    manager = multiprocessing.Manager()
    latest_worker = manager.Value("s", "No Match")  # 공유 메모리 생성

    detector = FaceDetect(latest_worker)  # ✅ latest_worker 전달
    detector.recognize_face()
