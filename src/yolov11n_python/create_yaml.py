import os
import glob
import cv2
import random
import yaml
import matplotlib.pyplot as plt
from ultralytics import YOLO

# 1. 경로 설정 (사용자 환경에 맞게 수정됨)
BASE_DIR = "../yolo11n"
IMAGE_DIR = os.path.join(BASE_DIR, "dataset/my_data/test/images")
YAML_SAVE_PATH = os.path.join(BASE_DIR, "custom_data.yaml")

# 2. 이미지 로드 확인 및 시각화 (SSH 환경 대응)
image_paths = glob.glob(os.path.join(IMAGE_DIR, "*.jpg"))

if not image_paths:
    print(f"❌ 이미지를 찾을 수 없습니다: {IMAGE_DIR}")
else:
    print(f"✅ {len(image_paths)}개의 이미지를 발견했습니다.")
    
    # 랜덤 이미지 출력 함수 (cv2.imshow 대신 plt 사용)
    selected_path = random.choice(image_paths)
    img = cv2.imread(selected_path)
    
    if img is not None:
        print(f"선택된 이미지 크기: {img.shape}")
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        plt.figure(figsize=(10, 6))
        plt.imshow(img_rgb)
        plt.title(f"Preview: {os.path.basename(selected_path)}")
        plt.axis('off')
        plt.show()
    else:
        print("❌ 이미지 파일을 읽지 못했습니다.")

# 3. 데이터셋 설정용 YAML 파일 생성
# 주의: YAML 내부의 train/val 경로도 로컬 절대 경로로 적어주는 것이 안전합니다.
data_config = {
    'train': os.path.join(BASE_DIR, 'dataset/my_data/train/images'),
    'val': os.path.join(BASE_DIR, 'dataset/my_data/valid/images'), # 또는 valid 경로
    'test': os.path.join(BASE_DIR, 'dataset/my_data/test/images'),
    'nc': 2,
    'names': ['dummy', 'car']
}

with open(YAML_SAVE_PATH, 'w') as f:
    yaml.dump(data_config, f)
    print(f"✅ YAML 파일이 생성되었습니다: {YAML_SAVE_PATH}")
