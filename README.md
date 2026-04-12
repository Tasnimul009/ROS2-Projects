# 🤖 ROS2 Robotics Projects — Tasnimul009

A collection of ROS2 packages for autonomous robot navigation, behavior-tree-based patrol systems, frontier exploration, and real-time object detection using YOLO.

---

## 📦 Packages Overview

| Package | Description |
|---|---|
| [`bt_navigator`](#bt_navigator) | Behavior tree navigator for goal-driven robot movement |
| [`bt_patrol`](#bt_patrol) | Behavior tree logic for automated patrol routines |
| [`frontier_explorer_pkg`](#frontier_explorer_pkg) | Frontier-based autonomous exploration |
| [`frontier_yolo_explorer_pkg`](#frontier_yolo_explorer_pkg) | Frontier exploration enhanced with YOLO object detection |
| [`patrol_blackboard`](#patrol_blackboard) | Shared blackboard for patrol behavior tree nodes |
| [`patrol_bt_pkg`](#patrol_bt_pkg) | Core patrol behavior tree package |
| [`patrol_robot`](#patrol_robot) | Top-level robot patrol integration package |
| [`ros2_object_detection`](#ros2_object_detection) | Real-time object detection pipeline using YOLO |

---

## 🗂️ Package Details

### `bt_navigator`
Goal-driven behavior tree navigator. Handles navigation action requests and routes the robot to target waypoints using Nav2-compatible action interfaces.

### `bt_patrol`
Implements patrol behavior tree logic, defining the sequence of actions the robot takes while patrolling a predefined set of waypoints or zones.

### `frontier_explorer_pkg`
Classic frontier-based exploration package. Detects unexplored frontiers in the map and autonomously drives the robot toward them for full environment coverage.

### `frontier_yolo_explorer_pkg`
An enhanced version of the frontier explorer that integrates YOLO for real-time object detection during exploration. Enables the robot to identify and log objects of interest while mapping the environment.

### `patrol_blackboard`
Manages shared state across patrol behavior tree nodes via a ROS2-compatible blackboard. Allows patrol nodes to read and write mission-critical data such as waypoint indices and status flags.

### `patrol_bt_pkg`
The core package tying together all patrol behavior tree components. Defines the tree structure and coordinates between navigator, blackboard, and sensor nodes.

### `patrol_robot`
Top-level integration package that launches the full patrol stack, including navigation, behavior trees, and any sensor pipelines required for a complete patrol mission.

### `ros2_object_detection`
Standalone object detection pipeline built on YOLO. Subscribes to camera topics, runs inference, and publishes annotated images and detection results to downstream nodes.

---

## ⚙️ Requirements

- **ROS2** (Humble / Iron / Jazzy)
- **Nav2** navigation stack
- **Python 3.10+**
- **OpenCV**
- **Ultralytics YOLO** (`pip install ultralytics`)
- A robot with a LiDAR and/or depth camera (for exploration packages)

---

## 🚀 Getting Started

### 1. Clone the repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/Tasnimul009/<your-repo-name>.git
```

### 2. Install dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip install ultralytics opencv-python
```

### 3. Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## ▶️ Usage

### Launch patrol robot

```bash
ros2 launch patrol_robot patrol_launch.py
```

### Launch frontier explorer

```bash
ros2 launch frontier_explorer_pkg explore.launch.py
```

### Launch YOLO-enhanced explorer

```bash
ros2 launch frontier_yolo_explorer_pkg yolo_explore.launch.py
```

### Run object detection node

```bash
ros2 run ros2_object_detection detection_node
```

---

## 🗺️ Architecture

```
patrol_robot
├── patrol_bt_pkg          ← Behavior tree root
│   ├── bt_navigator       ← Navigation actions
│   ├── bt_patrol          ← Patrol sequences
│   └── patrol_blackboard  ← Shared state
│
├── frontier_explorer_pkg  ← Map exploration
└── frontier_yolo_explorer_pkg  ← Exploration + detection
     └── ros2_object_detection  ← YOLO inference
```

---

## 🤝 Contributing

Pull requests are welcome! For major changes, please open an issue first to discuss what you'd like to change.

---

## 👤 Author

**Tasnimul009**
GitHub: [@Tasnimul009](https://github.com/Tasnimul009)

---
