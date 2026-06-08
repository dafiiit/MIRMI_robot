# Navigation Stack — Tracked AGV

Autonomous navigation for the MIRMI tracked AGV using **rtabmap** (2D LIDAR SLAM) and **nav2** (path planning + control).

---

## Architecture

```
Livox MID360 (/_livox/lidar)
        │
        ▼ pointcloud_to_laserscan
      /scan (LaserScan, 2D horizontal slice)
        │
        ├──────────────────────────────────────────────►  rtabmap
        │                                               (SLAM: builds /map,
        │                 /odometry/filtered ───────────  publishes map→odom_filtered TF)
        │                 (robot_localization EKF,          │
        │                  fuses /odom_px4 from PX4)        │
        │                                                   │
        │                                                   ▼
        └──────────────────────────────────────────────►  nav2
                                                        (plans path, controls robot)
                                                            │
                                                            ▼ /cmd_vel
                                                        cmdvel_to_px4
                                                            │
                                                            ▼
                                                          PX4 (rover)
```

### TF tree

```
map
 └── odom_filtered          ← published by rtabmap
      └── base_link_ekf     ← published by robot_localization EKF
           ├── livox_frame  ← static identity transform (LIDAR at robot CoM)
           └── camera_link  ← from URDF / robot_state_publisher
```

> **Note:** Frame names are non-standard: `base_link_ekf` instead of `base_link`,
> `odom_filtered` instead of `odom`. All nav2 / rtabmap params are set accordingly.

---

## Prerequisites

### 1 — Install ROS packages (once, needs sudo)

```bash
sudo apt install -y \
  ros-humble-rtabmap-ros \
  ros-humble-nav2-bringup ros-humble-nav2-bt-navigator \
  ros-humble-nav2-controller ros-humble-nav2-planner \
  ros-humble-nav2-behaviors ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-velocity-smoother ros-humble-nav2-waypoint-follower \
  ros-humble-nav2-costmap-2d ros-humble-nav2-navfn-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-core ros-humble-nav2-behavior-tree
```

### 2 — Build the package

```bash
cd ~/ws_sensor_combined
colcon build --packages-select navigation
source install/setup.bash
```

---

## How to run

### Terminal 1 — existing robot stack
```bash
ros2 launch cmdvel_to_px4 startup_combined.launch.py
```
This starts the PX4 bridge, cmd_vel → PX4 converter, Foxglove, etc.

### Terminal 2 — start Livox LIDAR
Use the process manager or manually:
```bash
ros2 launch station_detection_LIDAR LIDAR_detection.launch.py
```
Verify: `ros2 topic hz /_livox/lidar` should show ~10 Hz.

### Terminal 3 — navigation stack
```bash
source ~/ws_sensor_combined/install/setup.bash
ros2 launch navigation navigation.launch.py
```

**With a known GPS datum** (see GPS navigation section):
```bash
ros2 launch navigation navigation.launch.py \
  datum_lat:=48.262340 datum_lon:=11.668720 datum_alt:=490.0
```

---

## Sending navigation goals

### Option A — Map frame goal (metres, direct)

Send the robot to a specific position in the map frame:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {
    position: {x: 1.0, y: 0.0, z: 0.0},
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}"
```

> `x: 1.0` means 1 metre in the map X direction (East) from the map origin —
> **not** 1 m forward from the robot. To drive 1 m forward from the current
> position, first read the robot's current pose from `/odometry/filtered` and add
> to it.

### Option B — Dock-relative goal (distance + angle)

The `dock_goal_sender` node uses the pre-computed docking station position
(`config/dock_station.yaml`) to translate a polar goal into a nav2 pose.

**Station coordinate system:**
- **0°** = directly in front of the station (station faces East ≈ 88.8° compass)
- Angles increase **CCW** (counter-clockwise, looking from above)
- The robot always ends up **facing the station** at the target pose

```
         90° (North of station)
              │
  135° ───────┼─────── 45°
              │ Station
  180° (West)─┼──── 0° (East, "front")
              │
  Station 0° axis ≈ East (88.8° compass)
```

**Send a dock goal:**
```bash
ros2 topic pub --once /dock_goal geometry_msgs/msg/Point \
  "{x: 4.0, y: 45.0, z: 0.0}"
# → Robot drives to 4 m NNE of station, then turns to face the station
```

**Test case (45°, 4 m):**
The node computes:
- Target is 2.77 m East + 2.89 m North of the station
- Robot arrives facing 224° compass (WSW, toward station)

**Common positions:**
| Goal | Description |
|------|-------------|
| `{x: 2.0, y: 0.0}` | 2 m directly in front (docking approach) |
| `{x: 5.0, y: 90.0}` | 5 m to the North (left side) |
| `{x: 5.0, y: 180.0}` | 5 m behind the station |
| `{x: 4.0, y: 45.0}` | 4 m NNE (test case) |

**Station position source:** computed offline from 23 sweep measurements
(session 20260521_140405, GPS std dev 0.002–0.005 m).  
To recompute: `ros2 run navigation compute_dock_pose`

---

### Option C — GPS coordinate goal

The `gps_goal_sender` node converts a GPS lat/lon to the map frame automatically.

**How the datum works:**
- The **datum** is the GPS coordinate of the map frame origin — i.e., where the
  robot was when rtabmap started building the map.
- If no datum is given at launch, it is auto-set from the first GPS fix received.
  This means: **always start the robot at the same physical spot** each session,
  or provide the datum explicitly.

**Send a GPS goal:**
```bash
ros2 topic pub --once /navigate_gps sensor_msgs/msg/NavSatFix \
  "{latitude: 48.262345, longitude: 11.668731, altitude: 490.0}"
```

The node prints feedback:
```
GPS goal  lat=48.2623450 lon=11.6687310 → map (2.30 m East, 1.10 m North)
Distance remaining: 2.89 m
Navigation succeeded — goal reached.
```

**With a fixed final heading** (0° = East, 90° = North, 180° = West, 270° = South):
```bash
ros2 launch navigation navigation.launch.py \
  datum_lat:=48.262340 datum_lon:=11.668720 datum_alt:=490.0
# Then:
ros2 param set /gps_goal_sender goal_yaw_deg 90.0
```

---

## Configuration files

### `config/rtabmap_params.yaml`

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `frame_id` | `base_link_ekf` | Robot base frame |
| `odom_frame_id` | `odom_filtered` | Odometry frame |
| `Grid/CellSize` | `0.05` | Map resolution (5 cm) |
| `Grid/RangeMax` | `20.0` | Max usable LIDAR range (m) |
| `RGBD/LinearUpdate` | `0.05` | Add new node every 5 cm of travel |
| `Mem/IncrementalMemory` | `true` | SLAM mode (set `false` for localization-only) |

**To switch to localization-only mode** (robot already has a saved map):
1. Set `database_path` to the `.db` file path.
2. Set `Mem/IncrementalMemory: "false"`.
3. Restart the navigation stack.

### `config/nav2_params.yaml`

**Smoothness tuning for tracked robot:**

| Parameter | Location | Effect |
|-----------|----------|--------|
| `max_accel: [0.15, 0, 0.25]` | `velocity_smoother` | Lower = gentler start/stop |
| `max_decel: [-0.25, 0, -0.40]` | `velocity_smoother` | Braking rate |
| `desired_linear_vel: 0.25` | `FollowPath` (RPP) | Cruise speed [m/s] |
| `rotate_to_heading_min_angle: 0.5` | `FollowPath` (RPP) | Rotate in place when heading error > 0.5 rad (~29°). This is the key setting that prevents combined turn+drive motion (the main cause of jerkiness on tracks). |
| `rotate_to_heading_angular_vel: 0.35` | `FollowPath` (RPP) | In-place rotation speed [rad/s] |
| `xy_goal_tolerance: 0.10` | `goal_checker` | Stop within 10 cm of goal |

**Robot footprint:** Currently set to 0.70 m × 0.60 m. Adjust to actual chassis in both `local_costmap` and `global_costmap`:
```yaml
footprint: "[[-0.35, -0.30], [-0.35, 0.30], [0.35, 0.30], [0.35, -0.30]]"
```

---

## LIDAR height slice tuning

**Livox MID360 vertical FOV: −7° to +52°** — it scans almost entirely *upward* with only 7° below horizontal. At 2 m distance the lowest beam drops only ~24 cm below the sensor, so a wide negative `min_height` will include floor returns.

The `pointcloud_to_laserscan` node takes a vertical slice to produce the 2D scan. Current settings:

```python
'min_height':  0.10,   # 10 cm above livox_frame (avoids floor returns)
'max_height':  2.00,   # 2 m above livox_frame (catches walls, people, obstacles)
'use_inf': False,      # drop sky/no-return beams (avoids Foxglove ∞ errors)
```

**Tuning guide:**

| Symptom | Fix |
|---------|-----|
| Spurious obstacles right at robot's feet (floor returns) | Raise `min_height` (e.g. 0.15 or 0.20) |
| Missing low obstacles (kerbs, small boxes) | Lower `min_height` toward 0.0 |
| Missing tall obstacles (walls above lidar) | Raise `max_height` |
| Foxglove shows ∞ errors on `/scan` | Keep `use_inf: False` |

**To debug the raw 3D cloud in Foxglove:** add `/livox/lidar` as a PointCloud2 topic and set the 3D panel display frame to `livox_frame` — this shows the sensor output before any 2D conversion, without requiring the full TF chain to `map`.

If the LIDAR is physically offset from `base_link_ekf`, update the static transform in `navigation.launch.py` (the `static_tf_lidar` node arguments).

---

## GPS datum — how to find it

1. Drive the robot to the map starting position.
2. Wait for a GPS fix: `ros2 topic echo /fmu/out/vehicle_global_position --once`
3. Note `lat`, `lon`, `alt` — these are your datum values.
4. Use them at launch:
   ```bash
   ros2 launch navigation navigation.launch.py \
     datum_lat:=<lat> datum_lon:=<lon> datum_alt:=<alt>
   ```

---

## Saving and reusing a map

```bash
# Save the current rtabmap database (run while navigation stack is running):
ros2 service call /rtabmap/backup_database std_srvs/srv/Empty

# The database is saved to the path set in rtabmap_params.yaml > database_path.
# Default (empty string) = not persisted. Set a real path to keep it:
#   database_path: /home/holybro/maps/rtabmap.db
```

To use the saved map next session: set `Mem/IncrementalMemory: "false"` in `rtabmap_params.yaml` and restart.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Robot doesn't move | nav2 not activated yet | Wait for `Lifecycle manager is active` log line |
| `NavigateToPose` rejected | `map` not yet published | Drive the robot a bit so rtabmap builds an initial map |
| Robot drives jerkily | Acceleration too high | Lower `max_accel` in `velocity_smoother` |
| Robot oscillates around the goal | Goal tolerance too tight | Increase `xy_goal_tolerance` |
| LIDAR scan looks wrong | Wrong height slice | Tune `min_height`/`max_height` in `navigation.launch.py` |
| GPS goal off by many metres | Wrong datum | Verify datum lat/lon matches the actual map origin position |
| `No datum yet` error | Robot has no GPS fix | Check PX4 GPS lock (`ros2 topic echo /fmu/out/vehicle_global_position`) |
