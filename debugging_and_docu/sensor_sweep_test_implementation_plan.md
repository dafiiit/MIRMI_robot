# Sensor Sweep Test Suite — Implementation Plan

> **Target package:** `src/docking_test_suite/`  
> **Approach:** Extend (not replace) the existing package. New modules and topics sit alongside the current test A–D nodes. The robot never drives; only data capture is controlled.

---

## 1. Design Goals

| Goal | How it is met |
|---|---|
| State always visible via topics | `/sweep_test/status` publishes a `String` at 2 Hz with full JSON state |
| Trigger / control via topics | Three trigger topics (see §3) — publish once to act |
| Indoor / outdoor mode | `mode` field in trigger message; affects which reference is used |
| Camera-only, LiDAR-only, or both | `sensors` field in trigger message |
| 10 s capture window | Configurable, default 10 s; capturing flag broadcast on status |
| Know where to place target | `/sweep_test/target_placement` publishes a `String` with human-readable placement instruction |
| Distance sweep 10 m → 1 m in 1 m steps | `distance` scenario in `sensor_sweep_node.py` |
| Angular sweep 3 m fixed, 0°→180° in 20° steps | `angular` scenario in `sensor_sweep_node.py` |
| Raw sensor data + high-level topics saved | rosbag2 for raw; CSV for high-level detection data |
| Google Drive upload | Reuse existing `rclone` method from `backup_data.sh` (shell call) |
| Filename encodes time + mode + scenario | `YYYYMMDD_HHMMSS_<mode>_<scenario>_<step>` |

---

## 2. Architecture Overview

```
 ┌────────────────────────────────────────────────────────────────────┐
 │  sensor_sweep_node  (single ROS 2 node)                            │
 │                                                                    │
 │  Subscriptions (trigger topics)                                    │
 │   /sweep_test/cmd/configure   String (JSON)  ─┐                   │
 │   /sweep_test/cmd/capture     Empty          ─┼─► state machine   │
 │   /sweep_test/cmd/abort       Empty          ─┘                   │
 │                                                                    │
 │  Publications (status topics)                                      │
 │   /sweep_test/status          String (JSON)  2 Hz heartbeat       │
 │   /sweep_test/target_placement String         on each step change  │
 │   /sweep_test/capturing        Bool            true while recording │
 │   /sweep_test/step_complete    String (JSON)   after each step     │
 │                                                                    │
 │  Data capture                                                      │
 │   CSV  <- AprilTag detections, camera pose, LiDAR detections       │
 │   rosbag2 <- raw camera + LiDAR topics (subprocess)                │
 │                                                                    │
 │  Post-capture                                                      │
 │   rclone copy <local_dir>  gdrive:<remote_folder>                  │
 └────────────────────────────────────────────────────────────────────┘
```

---

## 3. ROS 2 Topic Interface

### 3.1 Trigger / Command Topics (subscribe)

#### `/sweep_test/cmd/configure`  — `std_msgs/String`

Publish a **JSON string** to select mode, scenario, and sensors before capturing.

```json
{
  "mode": "indoor",         // "indoor" | "outdoor"
  "scenario": "distance",   // "distance" | "angular"
  "sensors": "both",        // "camera" | "lidar" | "both"
  "step_index": 0           // optional: jump to specific step (0-indexed)
}
```

Example shell command:
```bash
ros2 topic pub --once /sweep_test/cmd/configure std_msgs/String \
  'data: "{\"mode\":\"outdoor\",\"scenario\":\"distance\",\"sensors\":\"both\"}"'
```

#### `/sweep_test/cmd/capture`  — `std_msgs/Empty`

Trigger a 10 s capture of the **current step**. After capture completes, the node automatically advances to the next step and publishes the new `target_placement`.

```bash
ros2 topic pub --once /sweep_test/cmd/capture std_msgs/Empty '{}'
```

#### `/sweep_test/cmd/abort`  — `std_msgs/Empty`

Immediately stop any ongoing capture and discard buffered data for the current capture window.

```bash
ros2 topic pub --once /sweep_test/cmd/abort std_msgs/Empty '{}'
```

---

### 3.2 Status / Feedback Topics (publish)

#### `/sweep_test/status`  — `std_msgs/String` (2 Hz)

JSON heartbeat — always tells you the full state:

```json
{
  "state": "IDLE",
  "mode": "outdoor",
  "scenario": "distance",
  "sensors": "both",
  "step_index": 2,
  "step_total": 10,
  "step_label": "dist_08p0m",
  "capturing": false,
  "capture_elapsed_s": 0.0,
  "capture_duration_s": 10.0,
  "samples_this_step": 0,
  "last_file": "",
  "gdrive_upload_ok": null,
  "timestamp_ns": 1714900000000000000
}
```

**States:**
- `IDLE` — waiting for `/configure`
- `CONFIGURED` — ready; waiting for first `/capture` trigger
- `CAPTURING` — 10 s window active
- `UPLOADING` — rclone running after capture completes
- `DONE` — all steps finished; send `/configure` to restart

#### `/sweep_test/target_placement`  — `std_msgs/String`

Human-readable instruction published on node start, after configure, and after each step advances:

```
[DISTANCE SWEEP | step 3/10] Place the AprilTag target at 8.0 m directly in front of the robot. Robot stays fixed.
```
```
[ANGULAR SWEEP | step 4/10] Keep the AprilTag at 3.0 m. Rotate target to 60° (left) relative to robot forward axis.
```

#### `/sweep_test/capturing`  — `std_msgs/Bool`

Simple boolean — `True` while a 10 s window is active. Useful for triggering other nodes or a physical LED.

#### `/sweep_test/step_complete`  — `std_msgs/String`

Published once when a capture window ends (before upload starts):

```json
{
  "step_label": "dist_08p0m",
  "samples": 127,
  "csv_file": "/home/holybro/sweep_test_data/20260505_143200_out_dist_08p0m.csv",
  "bag_dir":  "/home/holybro/sweep_test_data/20260505_143200_out_dist_08p0m_bag",
  "duration_s": 10.0
}
```

---

## 4. File Naming Convention

```
<YYYYMMDD>_<HHMMSS>_<mode>_<scenario>_<step_label>
```

| Token | Example values |
|---|---|
| `YYYYMMDD` | `20260505` |
| `HHMMSS` | `143200` |
| `mode` | `in` (indoor) / `out` (outdoor) |
| `scenario` | `dist` / `ang` |
| `step_label` | `dist_08p0m`, `dist_01p0m`, `ang_p020deg`, `ang_p180deg` |

**Examples:**
- `20260505_143200_out_dist_08p0m.csv`
- `20260505_143200_out_dist_08p0m_bag/` (rosbag2 directory)
- `20260505_143215_in_ang_p040deg.csv`

---

## 5. Data Saved

### 5.1 CSV (high-level — one file per capture step)

Columns (always timestamped with `timestamp_ns`):

| Column | Source |
|---|---|
| `timestamp_ns` | ROS wall clock nanoseconds |
| `elapsed_s` | seconds since capture start |
| `step_label` | e.g. `dist_08p0m` |
| `mode` | `indoor` / `outdoor` |
| `scenario` | `distance` / `angular` |
| `tag_id` | AprilTag ID |
| `tag_center_x/y` | pixel coords |
| `tag_c0..3_x/y` | corner pixels |
| `cam_rel_x/y/z` | solvePnP translation (m) |
| `cam_rel_rvec_x/y/z` | solvePnP rotation vector |
| `cam_distance_m` | Euclidean distance from camera |
| `lidar_detected` | Bool — did LiDAR node publish a detection? |
| `lidar_x/y/z` | LiDAR detection centroid if available |
| `px4_x/y/z` | PX4 odometry position |
| `px4_qw/qx/qy/qz` | PX4 odometry quaternion |
| `image_filename` | saved JPEG filename (empty if camera not selected) |

### 5.2 rosbag2 (raw — one bag directory per capture step)

Topics recorded depend on `sensors` parameter:

| Sensor flag | Topics recorded |
|---|---|
| `camera` | `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info`, `/apriltag/overlay/compressed`, `/tag_detections` |
| `lidar` | `/livox/lidar` (raw PointCloud2), LiDAR detection topic |
| `both` | All of the above |

Always recorded regardless of sensor flag:
- `/sweep_test/status`
- `/sweep_test/capturing`
- `/sweep_test/target_placement`

> **Why both CSV and bag?**  
> The CSV is immediately analysis-ready in pandas/Excel. The bag preserves 100% of raw data for offline reprocessing with different algorithms.

---

## 6. Google Drive Upload

Reuse the **`rclone`** approach already established in `backup_data.sh`.  
The node calls `rclone copy <local_step_dir> gdrive:sweep_test_data/` as a subprocess after each step completes.

Config keys added to `test_config.yaml`:
```yaml
google_drive:
  enabled: true
  rclone_remote: "gdrive"           # rclone remote name (run `rclone config` to set up)
  remote_folder: "sweep_test_data"  # folder on Google Drive
  auto_upload: true                 # upload after each step
```

The existing `gdrive_uploader.py` (OAuth2 Google Drive API) can still be used as an alternative if rclone is not configured — the node will try rclone first, fall back to the Python API.

---

## 7. Files to Create / Modify

### 7.1 New Files

```
src/docking_test_suite/
├── docking_test_suite/
│   ├── sensor_sweep_node.py        # NEW — main topic-driven node
│   ├── sweep_recorder.py           # NEW — multi-sensor recorder (CSV + bag)
│   └── sweep_scenarios.py          # NEW — step generators for distance & angular
├── launch/
│   └── sensor_sweep.launch.py      # NEW — launch file
└── config/
    └── test_config.yaml            # EDIT — add sweep_test + google_drive sections
```

### 7.2 Modified Files

| File | Change |
|---|---|
| `setup.py` | Add `sensor_sweep = docking_test_suite.sensor_sweep_node:main` entry point |
| `config/test_config.yaml` | Add `sweep_test:`, `google_drive.rclone_remote`, `google_drive.remote_folder` |

---

## 8. Module Details

### 8.1 `sweep_scenarios.py`

```python
def distance_sweep_steps(start_m=10.0, stop_m=1.0, step_m=1.0):
    """Return list of step dicts for distance sweep."""
    distances = [start_m - i*step_m for i in range(int((start_m-stop_m)/step_m)+1)]
    steps = []
    for d in distances:
        label = f"dist_{d:04.1f}m".replace('.', 'p')   # e.g. dist_08p0m
        instr = (
            f"[DISTANCE SWEEP] Place AprilTag at {d:.1f} m directly in front "
            f"of the robot. Robot stays fixed."
        )
        steps.append({'label': label, 'value': d, 'instruction': instr})
    return steps


def angular_sweep_steps(fixed_dist_m=3.0, start_deg=0, stop_deg=180, step_deg=20):
    """Return list of step dicts for angular sweep."""
    angles = list(range(start_deg, stop_deg + step_deg, step_deg))
    steps = []
    for a in angles:
        sign = 'p' if a >= 0 else 'n'
        label = f"ang_{sign}{abs(a):03d}deg"
        instr = (
            f"[ANGULAR SWEEP] Keep AprilTag at {fixed_dist_m:.1f} m. "
            f"Rotate target to {a}° (measured from robot forward, CCW positive). "
            f"Robot stays fixed."
        )
        steps.append({'label': label, 'value': a, 'instruction': instr})
    return steps
```

### 8.2 `sweep_recorder.py`

Inherits the CSV-writing logic from the existing `DataRecorder` but adds:
- `start_bag(topics, output_dir)` — spawns `ros2 bag record` as a subprocess
- `stop_bag()` — sends SIGINT to the bag subprocess
- LiDAR detection subscriber
- Columns for `lidar_detected`, `lidar_x/y/z`
- `sensors` parameter gates which subscriptions are active

### 8.3 `sensor_sweep_node.py`

State machine with 5 states: `IDLE → CONFIGURED → CAPTURING → UPLOADING → DONE`

Key methods:
- `_cmd_configure_cb(msg)` — parse JSON, set mode/scenario/sensors, advance to CONFIGURED
- `_cmd_capture_cb(msg)` — if state == CONFIGURED, start 10 s capture; ignore if CAPTURING
- `_cmd_abort_cb(msg)` — set abort flag; stop bag; discard CSV buffer
- `_status_timer_cb()` — publish JSON status + Bool capturing every 0.5 s
- `_capture_done()` — stop bag, flush CSV, publish step_complete, trigger upload, advance step

---

## 9. Test Scenarios — Step Sequence

### Distance Sweep (10 steps)

| Step | Label | Instruction |
|---|---|---|
| 1 | `dist_10p0m` | Place target at **10.0 m** in front of robot |
| 2 | `dist_09p0m` | Move target to **9.0 m** |
| 3 | `dist_08p0m` | Move target to **8.0 m** |
| ... | ... | ... |
| 10 | `dist_01p0m` | Move target to **1.0 m** |

### Angular Sweep (10 steps, 0° → 180° in 20° increments)

| Step | Label | Instruction |
|---|---|---|
| 1 | `ang_p000deg` | Target at **3.0 m**, **0°** (directly forward) |
| 2 | `ang_p020deg` | Target at **3.0 m**, rotate **20°** CCW |
| 3 | `ang_p040deg` | Target at **3.0 m**, rotate **40°** CCW |
| ... | ... | ... |
| 10 | `ang_p180deg` | Target at **3.0 m**, **180°** (directly behind robot) |

---

## 10. Workflow (Operator Procedure)

```
1. Launch:
   ros2 launch docking_test_suite sensor_sweep.launch.py

2. Configure (distance sweep, outdoor, both sensors):
   ros2 topic pub --once /sweep_test/cmd/configure std_msgs/String \
     'data: "{\"mode\":\"outdoor\",\"scenario\":\"distance\",\"sensors\":\"both\"}"'

3. Watch status + placement instructions:
   ros2 topic echo /sweep_test/status
   ros2 topic echo /sweep_test/target_placement

4. Place target where instructed, then trigger capture:
   ros2 topic pub --once /sweep_test/cmd/capture std_msgs/Empty '{}'

5. Wait 10 s (monitor /sweep_test/capturing: True → False)

6. Step auto-advances. Read new target_placement. Repeat from step 4.

7. After all steps: state → DONE. Files auto-uploaded to Google Drive.
```

---

## 11. Config Additions (`test_config.yaml`)

```yaml
# -----------------------------------------------
# SWEEP TEST SETTINGS
# -----------------------------------------------
sweep_test:
  capture_duration: 10.0          # seconds per step
  output_dir: "~/sweep_test_data"

  distance_sweep:
    start_m: 10.0
    stop_m:  1.0
    step_m:  1.0

  angular_sweep:
    fixed_distance_m: 3.0
    start_deg:  0
    stop_deg:  180
    step_deg:   20

  sensors:
    camera_raw_topic:   "/camera/camera/color/image_raw"
    camera_info_topic:  "/camera/camera/color/camera_info"
    camera_overlay_topic: "/apriltag/overlay/compressed"
    apriltag_topic:     "/tag_detections"
    lidar_raw_topic:    "/livox/lidar"
    lidar_detection_topic: "/station_detection_LIDAR/pole_detection"  # adjust to actual

google_drive:
  enabled: true
  rclone_remote: "gdrive"
  remote_folder: "sweep_test_data"
  auto_upload: true
  # Fallback: OAuth2 Python API (existing gdrive_uploader.py)
  credentials_file: "~/docking_test_data/credentials.json"
  token_file:       "~/docking_test_data/token.json"
  folder_id: ""
```

---

## 12. Implementation Order

1. **`sweep_scenarios.py`** — pure Python, no ROS, easy to unit-test
2. **`test_config.yaml`** — add new sections
3. **`sweep_recorder.py`** — extend `DataRecorder`; add bag subprocess + LiDAR sub
4. **`sensor_sweep_node.py`** — state machine + all topic publishers/subscribers
5. **`sensor_sweep.launch.py`** — launch file
6. **`setup.py`** — register entry point
7. **`colcon build`** and smoke-test with `ros2 topic pub`

---

## 13. Open Questions / Decisions Needed

> [!IMPORTANT]
> Please confirm these before implementation begins.

1. **LiDAR detection topic** — what is the exact topic name published by `station_detection_LIDAR`? (Need to check `station_detector.py`)
2. **LiDAR raw topic** — is it `/livox/lidar` or something else?
3. **Angular sweep direction** — does the *target* rotate around the robot (you carry the target around), or does *only the tag face angle* change at a fixed spot?
4. **Angular sweep range** — "180° in 20° steps" — is this 0°→180° (one side only, 10 steps) or −90°→+90° (symmetric, 10 steps)?
5. **rclone remote name** — is it `gdrive` (as in `backup_data.sh`) or something different?
6. **Google Drive folder** — should sweep test data go into an existing folder or a new `sweep_test_data` folder?
7. **rosbag format** — `mcap` (default in newer ROS 2) or `sqlite3`?

---

## 14. Estimated Effort

| Module | Complexity | LOC estimate |
|---|---|---|
| `sweep_scenarios.py` | Low | ~50 |
| `sweep_recorder.py` | Medium | ~200 |
| `sensor_sweep_node.py` | High | ~350 |
| Launch + config + setup.py | Low | ~80 |
| **Total** | | **~680** |
