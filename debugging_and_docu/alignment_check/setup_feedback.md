# Software and Hardware Setup Feedback

This document analyzes the alignment between the claims made in the bachelor's thesis ([main.pdf](file:///home/holybro/ws_sensor_combined/debugging_and_docu/alignment_check/main.pdf)) and the actual software reality on the physical robot platform.

---

## 1. Autonomy Control Architecture & Behavior Trees (RQ1 Context)
*   **Thesis Claim (Sections 4.5.2 & 4.5.6):** 
    The thesis claims that the on-board controller runs a "sensor-agnostic Behavior Tree" within the `docking_controller` package to coordinate three docking phases (`GPS approach` → `vision align` → `precise dock`). It states that all three localization sources (AprilTag, YOLOv8-pose, and LiDAR ICP) publish their relative pose estimations onto a unified `/localization/docking_pose` topic.
*   **Physical Reality:**
    *   No `docking_controller` package or Behavior Tree XML/C++ files exist in the workspace.
    *   The `/localization/docking_pose` topic is entirely absent from the codebase.
    *   **Actual Implementation:** The system coordinates docking using python scripts in `docking_test_suite` (using `sensor_sweep_node.py` which runs a finite state machine) and the `navigation` package (using `rtk_sweep_capture.py` and `dock_goal_sender.py`). Detections are communicated via specific, separate topics:
        *   **AprilTags:** Detections are published as `AprilTagDetectionArray` on `/tag_detections` (handled by `station_detection_APRILTAG`).
        *   **LiDAR:** Centroid/confidence is published on `/station_confidence` and `/station_marker` (handled by `station_detection_LIDAR`).
        *   **YOLO:** Bearing is published on `/station/bearing` (handled by `navigation/yolo_station_node.py`).

---

## 2. YOLOv8-pose Localizer Node
*   **Thesis Claim (Sections 4.4.2 & 4.5.2):** 
    The thesis describes the learned-keypoint (YOLOv8-pose) detector as a real-time 3D localization source running on-board on the NVIDIA Jetson Orin NX companion computer, publishing relative 3D pose estimations alongside AprilTag and LiDAR ICP.
*   **Physical Reality:**
    *   The custom YOLO node ([yolo_station_node.py](file:///home/holybro/ws_sensor_combined/src/navigation/navigation/yolo_station_node.py)) ONLY publishes 2D/angular metrics: `/station/detected` (Bool), `/station/bearing` (Float32), and `/station/info` (JSON string containing bearing in degrees, confidence, etc.). It does not perform PnP or compute a 3D relative pose.
    *   The YOLO node is not registered as a component in [process_manager_node.py](file:///home/holybro/ws_sensor_combined/src/cmdvel_to_px4/cmdvel_to_px4/process_manager_node.py). As a result, it cannot be dynamically managed or monitored via Foxglove.
    *   The [yolo_station_node.py](file:///home/holybro/ws_sensor_combined/src/navigation/navigation/yolo_station_node.py) and [rtk_sweep_capture.py](file:///home/holybro/ws_sensor_combined/src/navigation/navigation/rtk_sweep_capture.py) scripts are not registered in the `navigation` package's [CMakeLists.txt](file:///home/holybro/ws_sensor_combined/src/navigation/CMakeLists.txt) as runnable program targets. Consequently, they cannot be run via standard `ros2 run navigation ...` commands after building unless they are invoked directly with python3.

---

## 3. LiDAR Station Detector Parameters
*   **Thesis Claim (Section 4.4.2):**
    The LiDAR pipeline uses principal component analysis (PCA) to compute an oriented bounding box (OBB) scored against the known physical extent of the station (1.25 m × 1.25 m).
*   **Physical Reality:**
    In [station_detector.py](file:///home/holybro/ws_sensor_combined/src/station_detection_LIDAR/station_detection_LIDAR/station_detector.py) and the launch file [LIDAR_detection.launch.py](file:///home/holybro/ws_sensor_combined/src/station_detection_LIDAR/launch/LIDAR_detection.launch.py), the parameters are configured as `expected_length: 2.0` and `expected_width: 1.5` (with a high tolerance of 1.5 m). This is significantly larger than the physical station size claimed in the text.

---

## 4. BMS Telemetry and Launch File Duplications
*   **Thesis Claim (Section 4.4.2):**
    "A single launch file (startup_combined.launch.py) manages PX4 bridging, BMS telemetry, state publication, and remote monitoring bridges."
*   **Physical Reality:**
    *   While [startup_combined.launch.py](file:///home/holybro/ws_sensor_combined/src/cmdvel_to_px4/launch/startup_combined.launch.py) does include [px4_bridge.launch.py](file:///home/holybro/ws_sensor_combined/src/cmdvel_to_px4/launch/px4_bridge.launch.py) which starts `litime_bms_ros_node`, there is service duplication.
    *   [startup_combined.launch.py](file:///home/holybro/ws_sensor_combined/src/cmdvel_to_px4/launch/startup_combined.launch.py) starts `px4_arm_service/command_services` which provides `/px4/arm` and `/px4/offboard`. At the same time, `px4_bridge.launch.py` starts `cmdvel_to_px4_4` (direct control) which automatically arms and triggers offboard mode inside its own tick loop. This overlap causes conflicts over who controls offboard mode and arming.

---

## 5. Camera Driver Package
*   **Thesis Claim (Section 3.4.1):**
    The thesis lists `jetson_camera` as the ROS 2 driver node for the RGB camera.
*   **Physical Reality:**
    No `jetson_camera` package exists in the current workspace. The camera driver is actually standard `realsense2_camera` node (as started in [APRILTAG_detection.launch.py](file:///home/holybro/ws_sensor_combined/src/station_detection_APRILTAG/launch/APRILTAG_detection.launch.py)).
