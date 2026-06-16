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
The sweep node is auto-started and the capture automatically triggered when sending the configure command. The specified wait grace period allows the sensors to spin up before the 10-second capture window begins. Wait 10 seconds. The node will automatically save a CSV and a rosbag locally, and complete the recording.

### 3.5 Uploading Data
To upload the collected data for the current sweep to Google Drive (if configured), trigger the upload command:
```bash
ros2 topic pub --once /sweep_test/cmd/start_data_upload std_msgs/Empty '{}'
```
You can track the upload progress by monitoring the `/sweep_test/upload_status` topic.

### 3.6 Aborting
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

---

# 5. Automated full sweep (self-driving, 100 points)

Instead of moving the target by hand, the rover can **drive itself** to a polar grid of
positions around the docking station using **RTK GPS only** (no map / no nav2 / no depth),
**face the station with the YOLO detector**, and trigger a capture at each stop. The capture
backend is the same `sensor_sweep_node` documented above.

Grid: rays `[0, 30, 45, 90, 135, 180, 225, 270, 315, 330]°` (swept CCW), distances
`10 → 1 m` in 1 m steps = **100 capture positions**. Precomputed in
`../navigation/config/sweep_targets.csv`.

## 5.1 Components
| File | Role |
|------|------|
| `navigation/navigation/rtk_sweep_capture.py` | the driver+capture controller |
| `navigation/navigation/yolo_station_node.py` | YOLOv8-pose station detector → `/station/bearing` |
| `navigation/config/dock_station.yaml` | station GPS position + 0°-axis heading |
| `navigation/config/sweep_targets.csv` | the 100 target GPS positions (reference) |
| `src/yolo/best.pt` | trained YOLOv8-pose weights (8 station corners) |

## 5.2 How it works (per target)
1. **Drive** to the target GPS position. Tracked rovers turn abruptly, so it never steers
   while moving: it **aims → drives a straight ≤1.5 m segment → stops → re-aims**, and locks
   onto a straight final approach near the target. It only moves when **RTK is good**
   (`fix_type ≥ 5` AND `eph ≤ 5 cm`); if the fix degrades it stops and waits.
2. **Face the station** (must be centered for a good capture):
   - swing to the GPS bearing-to-station (compass) so the station enters the camera view;
   - **YOLO visual servo**: rotate (in no-overshoot *pulses*) until the station is centered
     in the image (`/station/bearing → 0`) — this is the accurate, primary reference;
   - LIDAR forward-object sanity check; GPS+compass fallback if the camera is unavailable.
   - Note: the camera is mounted upside-down — `yolo_station_node` rotates the image 180°.
3. **Capture**: publishes `/sweep_test/cmd/configure` and waits for `step_complete`
   (records CSV + rosbag exactly as in §3–4).

## 5.3 Prerequisites (start these first)
The robot is managed by the process_manager via `/start_*` topics. Start:
```bash
ros2 topic pub --once /start_rtk_ntrip          std_msgs/Empty "{}"   # RTK corrections (needed for fix)
ros2 topic pub --once /start_camera_stream      std_msgs/Empty "{}"   # camera (YOLO + capture)
ros2 topic pub --once /start_livox_driver       std_msgs/Empty "{}"   # lidar (capture + sanity)
ros2 topic pub --once /start_static_tf_livox    std_msgs/Empty "{}"
ros2 topic pub --once /start_docking_test_suite std_msgs/Empty "{}"   # this sensor_sweep_node
```
Also required (normally already up): the `cmdvel_to_px4` bridge (consumes `/cmd_vel`; the rover
auto-arms + enters OFFBOARD on the first `/cmd_vel`). Verify RTK is **FIXED/FLOAT with eph ≤ 5 cm**
on `/fmu/out/vehicle_gps_position` before starting.

## 5.4 Run it
```bash
cd ~/ws_sensor_combined
source /opt/ros/humble/setup.bash && source install/setup.bash

# 1) start the YOLO station detector (loads best.pt, ~10 s; publishes /station/bearing)
python3 src/navigation/navigation/yolo_station_node.py &

# 2a) DRY RUN first — prints every target + live RTK, NO motion:
python3 src/navigation/navigation/rtk_sweep_capture.py --rays all --auto

# 2b) SHAKEDOWN one point (recommended before the full run):
python3 src/navigation/navigation/rtk_sweep_capture.py --rays 0 --distances 10 --go --auto

# 2c) FULL automatic run — drives + faces + captures all 100:
python3 src/navigation/navigation/rtk_sweep_capture.py --rays all --go --auto --keep-going
```
`--go` is required for any motion (without it, dry run). `--auto` = hands-off loop;
omit `--auto` for **manual stepping** (publish `std_msgs/Empty` to
`/start_next_measure_point` to drive to the next point and `/start_capture` to capture).
`--keep-going` skips a point and continues if a drive or capture fails (recommended for the
unattended 100-point run). Resume an interrupted run with `--start-index N` (skips the first
N targets). Captures land in `~/sweep_test_data/combined_series/<timestamp>_out_cust_both/`.

## 5.5 Useful flags
- `--rtk-fix {5,6}` min fix type to drive (5 = RTK_FLOAT, eph-gated [default]; 6 = RTK_FIXED, stricter)
- `--rtk-eph 0.05` max horizontal accuracy to drive, metres
- `--rays 0,45,90` / `--distances 10,5,1` run a subset
- `--no-face` skip auto-facing (operator orients manually)
- `--no-yolo` disable YOLO, use GPS+compass facing only
- `--max-lin 0.7 --max-yaw 0.8` drive/turn command magnitudes (normalized, 1.0 = full)
- `--pos-tol 0.30` stop tolerance (m); the rover stops coarse — operator may fine-position
- `--face-vis-tol-deg 3.5` how centred YOLO must get the station before capture

## 5.6 Safety
- The rover only drives when RTK passes the gate; it stops and holds on any dropout.
- Publishing `/cmd_vel` auto-arms the rover — the script defaults to dry run; `--go` is explicit.
- Ctrl-C (or `ros2 topic pub --once /stop std_msgs/Empty "{}"`) stops the rover immediately.
