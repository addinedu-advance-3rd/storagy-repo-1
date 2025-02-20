import json
import os
import torch
from facenet_pytorch import MTCNN, InceptionResnetV1
from torchvision import transforms
import cv2
import numpy as np

#얼굴 저장 클래스
class FaceSaver:
    #폴더 경로
    IMG_SRC_FOLDER = "img_src"
    METADATA_PATH = os.path.join(IMG_SRC_FOLDER, "face_metadata.json")

    def __init__(self):
        #쓰는 모델들
        # CUDA 사용 여부 확인
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if torch.cuda.is_available():
            print(f"Using GPU: {torch.cuda.get_device_name(0)}")
        else:
            print("Using CPU")
        self.mtcnn = MTCNN(keep_all=False, device=self.device)
        self.facenet = InceptionResnetV1(pretrained='vggface2').eval().to(self.device)

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

    #json에 새 얼굴 데이터 저장
    def save_new_face(self, embedding):
        try:
            with open(self.METADATA_PATH, "r", encoding="utf-8") as file:
                metadata = json.load(file)
        except (FileNotFoundError, json.JSONDecodeError):
            metadata = {}

        user_name = f"Worker{len(metadata) + 1}"
        metadata[user_name] = {"embedding": embedding.tolist()}

        with open(self.METADATA_PATH, "w", encoding="utf-8") as file:
            json.dump(metadata, file, indent=4, ensure_ascii=False)

        print(f"{user_name} 등록 완료!")

    #main -> 캠실행 ->s누르면 저장
    def register_face(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            print("Failed to open webcam.")
            return

        print("Press 's' to save new face, 'q' to quit.")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Failed to read from webcam.")
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            embedding, box = self.extract_embedding(frame_rgb)

            #바운딩 박스
            if embedding is not None and box is not None:
                x1, y1, x2, y2 = [int(b) for b in box]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.imshow("Face Registration", frame)
            key = cv2.waitKey(1) & 0xFF

            #저장 타이밍 -> s
            if key == ord('s') and embedding is not None:
                self.save_new_face(embedding)

            if key == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    saver = FaceSaver()
    saver.register_face()
