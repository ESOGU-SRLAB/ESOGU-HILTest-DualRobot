#!/usr/bin/env python3
from ultralytics import YOLO
import cv2

model = YOLO("/home/cem/colcon_ws/src/yolo_ros/best.pt")

results = model.predict(
    source="/home/cem/Pictures/Screenshots/Screenshot from 2026-03-05 14-23-40.png",
    conf=0.3,
    device="cpu",
    show=False,
    save=False,
)

for r in results:
    print(f"Tespit sayısı: {len(r.boxes)}")
    for box in r.boxes:
        cls_id = int(box.cls)
        cls_name = model.names[cls_id]
        conf = float(box.conf)
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        print(f"  {cls_name} ({conf:.2f}) -> [{x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}]")

    annotated = r.plot()
    cv2.imwrite("/home/cem/colcon_ws/src/yolo_ros/detection_result.png", annotated)
    print("Sonuç kaydedildi: /home/cem/colcon_ws/src/yolo_ros/detection_result.png")
