# Sensor Sweep Test Suite

This package provides a topic-driven test suite to evaluate the performance and accuracy of AprilTag-based localization and LiDAR station detection during static tests (distance profiles and angular profiles). The robot remains stationary during these tests while the target is moved.

## 1. Overview
The suite records synchronized multi-sensor data into CSV files and `rosbag2` databases. 
Tests are purely topic-driven: a central node orchestrates test steps, provides user feedback, and handles data recording and uploading to Google Drive via `rclone`.

Supported scenarios:
- **Distance Sweep:** Farthest to closest distance sweeps (e.g., 10m to 1m).
- **Angular Sweep:** Rotational sweep at a fixed distance (e.g., 0° to 180° in 20° steps).

## 2. Configuration
The main configuration file is located at `config/test_config.yaml`. 
Here you can configure:
* **Sweep Test Parameters:** Distances, angles, and capture duration per step.
* **Topics:** Camera, AprilTag, and LiDAR topic names.
* **Google Drive:** `rclone` remote name and remote directory for auto-uploading logs and rosbag files.

## 3. Usage

### 3.1 Launching the node
Start the main orchestration node:
```bash
ros2 launch docking_test_suite sensor_sweep.launch.py
```

### 3.2 Configuring a test
Send a single `/sweep_test/cmd/configure` JSON message to set the test scenario, environment mode, and active sensors.
```bash
ros2 topic pub --once /sweep_test/cmd/configure std_msgs/String \
  'data: "{\"mode\":\"outdoor\",\"scenario\":\"distance\",\"sensors\":\"both\"}"'
```

### 3.3 Watching Status and Placing the Target
Monitor the `/sweep_test/status` and `/sweep_test/target_placement` topics to know the current state and where to place the target next.
```bash
ros2 topic echo /sweep_test/status
ros2 topic echo /sweep_test/target_placement
```

### 3.4 Capturing Data
Once the target is placed at the specified location, trigger the 10-second capture window:
```bash
ros2 topic pub --once /sweep_test/cmd/capture std_msgs/Empty '{}'
```

Wait 10 seconds. The node will automatically save a CSV, a rosbag, upload them to Google Drive (if configured), and advance to the next step.
Repeat the placement and capture process until all steps are complete.

### 3.5 Aborting
To immediately stop an ongoing capture and discard the data for the current step:
```bash
ros2 topic pub --once /sweep_test/cmd/abort std_msgs/Empty '{}'
```

## 4. File Output
The node saves files in the configured `output_dir` (default `~/sweep_test_data`).
Filename convention: `<YYYYMMDD>_<HHMMSS>_<mode>_<scenario>_<step_label>`

Example outputs for a step:
- `20260505_143200_out_dist_08p0m.csv`
- `20260505_143200_out_dist_08p0m_bag/` (rosbag2 directory)
