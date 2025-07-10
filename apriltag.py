import cv2
import numpy as np
from pyapriltags import Detector

# 摄像头参数
FX, FY = 800.0, 800.0
CX, CY = 320.0, 240.0
TAG_SIZE = 0.1

cap = cv2.VideoCapture("cam 0")
detector = Detector(families='tag36h11')
camera_params = (FX, FY, CX, CY)

while True:
    ret, frame = cap.read()
    if not ret: break
              #  print("no cam")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, estimate_tag_pose=True,
                                 camera_params=camera_params,
                                 tag_size=TAG_SIZE)

    for detection in detections:
        # 计算距离和角度
        distance = float(np.linalg.norm(detection.pose_t))
        yaw = float(np.arctan2(detection.pose_t[0], detection.pose_t[2]))
        pitch = float(np.arctan2(detection.pose_t[1], detection.pose_t[2]))

        # 打印结果（替代 GUI 显示）
        print(f"Detected Tag ID: {detection.tag_id}")
        print(f"Distance: {distance:.2f} m")
        print(f"Yaw: {np.degrees(yaw):.1f} deg")
        print(f"Pitch: {np.degrees(pitch):.1f} deg")
        print("-" * 30)



cap.release()