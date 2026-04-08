# 🤖 ROS2 Object Detection — YOLOv8 + Gazebo

Real-time object detection node for ROS2 Humble.  
Subscribes to a Gazebo camera, runs **YOLOv8 nano** (CPU-friendly), and publishes annotated images + JSON detections.

---

## 📦 Dependencies

```bash
# ROS2 packages
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs

# Python ML dependencies
pip install ultralytics opencv-python
```

> YOLOv8 nano (`yolov8n.pt`, ~6 MB) downloads automatically on first run.

---

## 🗂️ Package Structure

```
object_detection/
├── object_detection/
│   └── object_detection_node.py   # Main node
├── launch/
│   └── detection_launch.py        # Gazebo + detection bringup
├── config/
│   └── params.yaml                # Tunable parameters
├── package.xml
└── setup.py
```

---

## 🔧 Build

```bash
cd ~/ros2_ws
cp -r ~/object_detection src/
colcon build --packages-select object_detection
source install/setup.bash
```

---

## 🚀 Run

### Option A — Full launch (Gazebo + detection together)
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch object_detection detection_launch.py
```

### Option B — Detection node only (camera already running)
```bash
ros2 run object_detection detection_node \
  --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p confidence:=0.5 \
  -p show_window:=true
```

---

## 📡 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Input from Gazebo camera |
| `/detections` | `std_msgs/String` | JSON list of detections |
| `/detections/image` | `sensor_msgs/Image` | Annotated output image |

### Sample `/detections` output
```json
[
  {"class": "person",  "confidence": 0.87, "bbox": [120, 80, 300, 450]},
  {"class": "bottle",  "confidence": 0.72, "bbox": [400, 200, 480, 390]}
]
```

---

## ⚙️ Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model` | `yolov8n.pt` | YOLO model file (n/s/m/l) |
| `confidence` | `0.5` | Detection confidence threshold |
| `camera_topic` | `/camera/image_raw` | Input image topic |
| `show_window` | `true` | Show OpenCV window |

---

## 🎯 What it detects

80 COCO classes out of the box — person, bottle, chair, laptop, cup, car, dog, cat, and more.  
No custom training needed for the demo.

---

## 💡 Tips for demo

- Open **RViz2** → add `Image` display → set topic to `/detections/image`
- Echo detections live: `ros2 topic echo /detections`
- Lower confidence to `0.3` to get more detections in an empty scene
- Place 3D models in Gazebo (bottle, chair) to trigger detections
