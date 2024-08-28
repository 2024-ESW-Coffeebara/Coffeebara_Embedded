from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import serial
import time
import math
import os
import numpy as np

class_ids = {
    0 : "cup",
    1 : "entrance",
    2 : "holder",
    3 : "straw"
}

cup_class_id = 0
entrance_class_id = 1
holder_class_id = 2
straw_class_id = 3

class Cup:
    def __init__(self, x1, y1, x2, y2, width, height):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.width = width
        self.height = height

class Entrance:
    def __init__(self, x1, y1, x2, y2, width, height):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.width = width
        self.height = height

class Holder:
    def __init__(self, x1, y1, x2, y2, width, height):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.width = width
        self.height = height

# 딥러닝 모델 로드 (예: 사전 훈련된 모델)
model_path = "../0812/best.pt"
model = YOLO(model_path)

# 이미지 전처리 함수
def preprocess_image(image_path):
    image = Image.open(image_path)
    image = image.resize((224, 224))  # MobileNetV2의 입력 크기
    image = np.array(image) / 255.0   # 스케일링
    image = np.expand_dims(image, axis=0)  # 배치 차원 추가
    return image

# 폴더 내 모든 이미지에 대해 처리
image_folder = './captured_images'  # 이미지 폴더 경로
images = [os.path.join(image_folder, file) for file in os.listdir(image_folder) if file.endswith(('.jpg', '.png', '.jpeg'))]

# 그래프 설정
plt.figure(figsize=(10, 30))

for i, image_path in enumerate(images):
    
    img = cv2.imread(image_path, cv2.COLOR_BGR2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    img = img[:, : 350]

    results = model(img, imgsz = (640, 640))

    largest_cup = None
    largest_entrance = None
    largest_holder = None

    for result in results:
        for box in result.boxes:
            confidence = math.ceil((box.conf[0] * 100)) / 100

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            width = x2 - x1
            height = y2 - y1

            if confidence < 0.5:
                continue

            object_size = width * height

            class_id = int(box.cls[0])

            if class_id == cup_class_id:
                if largest_cup is None or object_size > (largest_cup.width * largest_cup.height):
                    largest_cup = Cup(x1, y1, x2, y2, width, height)

            elif class_id == entrance_class_id:
                if largest_entrance is None or object_size > (largest_entrance.width * largest_entrance.height):
                    largest_entrance = Entrance(x1, y1, x2, y2, width, height)

            elif class_id == holder_class_id:
                if largest_holder is None or object_size > (largest_holder.width * largest_holder.height):
                    largest_holder = Holder(x1, y1, x2, y2, width, height)

    if largest_cup is None or largest_entrance is None :
        continue

    # 원본 이미지 로드
    result_img = results[0].plot()
    
    # 서브플롯 설정
    plt.subplot(4, 4, i+1)
    plt.imshow(result_img)
    plt.axis('off')
    
    # 예측 결과 텍스트 추가
    text = f"Largest Ent Width: {largest_entrance.x2 - largest_entrance.x1}\nLargest cup Height : {largest_cup.y2 - largest_entrance.y2}\nConfidence: {confidence:.2f}"
    plt.text(5, 5, text, fontsize=12, color='white', bbox=dict(facecolor='black', alpha=0.5))

# 전체 그래프 표시
plt.show()
