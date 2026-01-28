import os
import glob
import cv2
import random
import yaml
import matplotlib.pyplot as plt
from ultralytics import YOLO
from ultralytics import settings 

settings.update({"tensorboard": True})

# 1. 경로 설정 (사용자 환경에 맞게 수정됨)
BASE_DIR = "/home/coddyd/Desktop/workspace/Rokey/yolo11n"
YAML_SAVE_PATH = os.path.join(BASE_DIR, "custom_data.yaml")
EPOCHS = 50
BATCH_SIZE = 4
LEARNING_RATE = 0.00000001


# 4. YOLO11n 모델 학습 시작
# 로컬 GPU 환경을 사용한다고 가정합니다.
model = YOLO('yolo11n.pt')

try:
    model.train(
        data=YAML_SAVE_PATH,
        epochs=EPOCHS,
        patience= 10, 
        batch=BATCH_SIZE,
        imgsz=640,
        lr0=LEARNING_RATE,
        device=0,
        project=os.path.join(BASE_DIR, "runs"),
        name=f"yolo11n_ep{EPOCHS}_bc{BATCH_SIZE}_lr{LEARNING_RATE}",
        
        # --- [추가된 증강 파라미터] ---
        mosaic=1.0, 
        mixup=0.2, 
        copy_paste=0.1,
        degrees=10.0, 
        fliplr=0.5,
        hsv_h=0.015, 
        hsv_v=0.4,
        shear=2.0,            # 전단 변형 (기울여서 보기)
        perspective=0.0001,   # 원근감 변화
        flipud=0.0,           # 상하 반전 (바닥에 있는 물체라면 0.0, 하늘에서도 본다면 0.5)
        hsv_s=0.7,            # 채도(Saturation) 변화
        # --- [Cosine Annealing & Warmup 설정] ---
        cos_lr=True,               # Cosine Annealing 사용 (학습률을 코사인 곡선처럼 부드럽게 감소)
        warmup_epochs=3.0,         # 초반 3 epoch 동안은 아주 작은 학습률로 시작 (Warmup)
        warmup_momentum=0.8,       # Warmup 기간 동안의 모멘텀 설정
        warmup_bias_lr=0.1,        # Warmup 기간 동안의 초기 bias 학습률

    )
except Exception as e:
    print(f"❌ 학습 중 오류 발생: {e}")

metrics = model.val()
print(f"✅ 최종 mAP50: {metrics.box.map50}")
