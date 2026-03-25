# Debugging and Documentation: Current Code Structure

Generated: `2026-03-20` (from the current workspace snapshot at `/home/holybro/ws_sensor_combined`)

## 1) Workspace overview

This is a ROS 2 workspace containing a set of sensor and PX4 bridging nodes plus a “docking test suite” used to collect synchronized data (Vicon pose + AprilTag detections + camera calibration) and evaluate docking accuracy.

All ROS packages live under:

- `src/<package_name>/...`

Top-level package list (from `package.xml` files):

- `docking_test_suite` (ament_python)
- `mirmi_apriltag` (ament_python)
- `video_streamer` (ament_python)
- `gps_position_publisher` (ament_python)
- `ir_led_tracker` (ament_python)
- `ros2-vicon-bridge` (ament_cmake, C++)
- `px4_arm_service` (ament_python)
- `cmdvel_to_px4` (ament_python)
- `jetson_camera` (ament_python)
- `px4_localization_bridge` (ament_python)
- `litime_bms_ros` (ament_python)
- `px4_msgs` (ament_cmake, generated PX4 message definitions)

## 2) High-level directory structure

### `src/docking_test_suite/`

- `package.xml`, `setup.py`
- `config/test_config.yaml`
- `launch/`
  - `tf_tree.launch.py`
  - `debug_tools.launch.py`
  - `test_a.launch.py`, `test_b.launch.py`, `test_c.launch.py`, `test_d.launch.py`
- Python package: `docking_test_suite/`
  - Console scripts (from `setup.py`):
    - `test_a_distance` -> `docking_test_suite.test_a_distance:main`
    - `test_b_angular` -> `docking_test_suite.test_b_angular:main`
    - `test_c_dynamic` -> `docking_test_suite.test_c_dynamic:main`
    - `test_d_environmental` -> `docking_test_suite.test_d_environmental:main`
    - `robot_driver_cli` -> `docking_test_suite.robot_driver:main`
    - `docking_diagnostics` -> `docking_test_suite.diagnostics:main`
    - `gdrive_upload` -> `docking_test_suite.gdrive_uploader:main`
    - `docking_analyze` -> `docking_test_suite.analysis:main`
    - `pose_tf_broadcaster` -> `docking_test_suite.pose_tf_broadcaster:main`
  - Core modules:
    - `data_recorder.py` (synchronized recording to CSV/images)
    - `config_loader.py` (YAML load + validation)
    - `robot_driver.py` (publishes `/cmd_vel`, optionally PX4 heartbeat/commands)
    - `diagnostics.py` (topic rate and existence checks)
    - `transforms.py` (offset transforms, quaternion math, solvePnP wrapper)
    - `test_*` scripts (drive + record)

### `src/mirmi_apriltag/`

- `package.xml`, `setup.py`
- `launch/`
  - `apriltag_detection.launch.py` (Jetson-style assumptions: `/camera/image_raw`, `/camera/camera_info`)
  - `usb_apriltag_detection.launch.py` (RealSense or USB camera based on `camera_type`)
- `config/`
  - `tags.yaml`
  - `isaac_ros_apriltag_params.yaml`
- Python package: `mirmi_apriltag/`
  - Console scripts:
    - `apriltag_visualizer` -> `mirmi_apriltag.apriltag_visualizer:main`
    - `depth_visualizer` -> `mirmi_apriltag.depth_visualizer:main`
    - `accuracy_investigation` -> `mirmi_apriltag.accuracy_investigation:main`

### `src/jetson_camera/`

- `package.xml`, `setup.py`
- Python package: `jetson_camera/`
  - Console scripts:
    - `camera_node` -> `jetson_camera.camera_node:main`
    - `usb_camera_node` -> `jetson_camera.usb_camera_node:main`

### `src/video_streamer/`

- `package.xml`, `setup.py`
- `video_streamer/gstreamer_node.py`
  - Console script:
    - `video_stream` -> `video_streamer.gstreamer_node:main`

### `src/gps_position_publisher/`

- `package.xml`, `setup.py`
- `gps_position_publisher/gps_publisher_node.py`
  - Console script:
    - `gps_publisher` -> `gps_position_publisher.gps_publisher_node:main`

### `src/ir_led_tracker/`

- `package.xml`, `setup.py`
- `launch/tracker.launch.py`
- `ir_led_tracker/ir_tracker_node.py`
  - Console script:
    - `ir_tracker_node` -> `ir_led_tracker.ir_tracker_node:main`

### `src/ros2-vicon-bridge/` (C++)

- `package.xml`, `CMakeLists.txt`
- `launch/all_segments.launch.py`
- C++ executables:
  - `vicon_bridge` (publish TF + PoseStamped per Vicon segment)
  - `vicon_bridge_retiming` (retiming variant; TF only)
- Libraries:
  - Compiles Vicon SDK sources from `vicon_sdk/**/**.cpp` into a local library (`vicon_sdk`)

### `src/cmdvel_to_px4/`

- `package.xml`, `setup.py`
- `launch/startup_combined.launch.py`
- `launch/px4_bridge.launch.py`
- Node implementations:
  - `cmdvel_node.py` (`cmdvel_to_actuators_safe`)
  - `cmdvel_node2.py` (`cmdvel_to_actuators_safe` / rover mode; see notes below)
  - `cmdvel_node3.py` (`cmdvel_to_px4_3`; actually provides SetBool services)

Console scripts:

- `cmdvel_to_px4` -> `cmdvel_to_px4.cmdvel_node:main`
- `cmdvel_to_px4_2` -> `cmdvel_to_px4.cmdvel_node2:main`
- `cmdvel_to_px4_3` -> `cmdvel_to_px4.cmdvel_node3:main`

### `src/px4_localization_bridge/`

- `package.xml`, `setup.py`
- Node implementations:
  - `odom_to_px4.py` (nav_msgs/Odometry -> `VehicleOdometry`)
  - `vicon_odometry_pub.py` (Vicon PoseStamped -> `VehicleOdometry` (visual odometry))
  - `fake_odometry_pub.py` (circular fake odometry -> `VehicleOdometry`)
  - `fake_pub.py` (stationary fake VIO -> `VehicleOdometry`)

Console scripts:

- `fake_odometry_pub`
- `odom_to_px4`
- `fake_odometry_pub_2`
- `vicon_odometry_pub`

### `src/px4_arm_service/`

- `package.xml`, `setup.py`
- `arm_service.py`
  - Console script:
    - `command_services` -> `px4_arm_service.arm_service:main`
  - Provides `/px4/arm` and `/px4/offboard` as `std_srvs/SetBool` and publishes `px4_msgs/VehicleCommand`.

### `src/litime_bms_ros/`

- `package.xml`, `setup.py`
- `litime_bms_ros_node.py`
  - Console script:
    - `litime_bms_ros_node` -> `litime_bms_ros.litime_bms_ros_node:main`
  - BLE polling/notifications (via `bleak`) and publishes:
    - `litime_bms/state` (`sensor_msgs/BatteryState`)
    - `litime_bms/raw` (`std_msgs/String`, UUID + raw hex)

### `src/px4_msgs/` (generated message definitions)

- `package.xml`, `CMakeLists.txt`
- `msg/` (many `.msg` definitions)
- `srv/VehicleCommand.srv`
- Built via `rosidl_default_generators` to provide `px4_msgs.msg.*` types used by the PX4 bridge nodes.

## 3) TF frames and Vicon naming

### 3.1 `ros2-vicon-bridge/vicon_bridge` output naming

Vicon TF and pose topics are created per Vicon subject/segment.

Defaults (from `vicon_bridge.hpp`):

- `tf_namespace_ = "vicon"`
- `world_frame_id_ = "world"`
- `update_rate_hz_ = 250.0`

For each (subject, segment), the bridge uses:

- TF:
  - parent frame: `tf_namespace_/world_frame_id_` -> `vicon/world`
  - child frame: `tf_namespace_/subject/segment` -> `vicon/<SubjectName>/<SegmentName>`
- PoseStamped topic:
  - topic: `tf_namespace_/subject/segment/pose` -> `vicon/<SubjectName>/<SegmentName>/pose`

The Vicon bridge also publishes TF as it is created:

- `geometry_msgs::msg::TransformStamped` on TF
- `geometry_msgs::msg::PoseStamped` published from the transform.

### 3.2 `docking_test_suite/launch/tf_tree.launch.py`

This launch adds “conventional roots/aliases” and fixed docking-related frame links so downstream code can use consistent frame names even if Vicon is disconnected.

It publishes these static transforms:

- `world` -> `vicon/world` (identity)
- `map` -> `vicon/world` (identity)
- `vicon/Robot_1/Robot_1` -> `base_link` (identity)
- `vicon/Tag_0/Tag_0` -> `dock_tag` (translation `x=-0.122`, `qz=qw=0.70710678` etc)
- back-compat alias:
  - `vicon/Tag_0/Tag_0` -> `vicon_real_target`

### 3.3 `ros2-vicon-bridge/launch/all_segments.launch.py`

This launch starts `vicon_bridge` and also publishes static docking-related frames:

- `map` -> `vicon/world`
- `vicon/Robot_1/Robot_1` -> `base_link`
- `vicon/Tag_0/Tag_0` -> `vicon_real_target`
- `vicon/Tag_0/Tag_0` -> `dock_tag`

Note: this overlaps with `docking_test_suite/tf_tree.launch.py` for several static transforms. Running both simultaneously can result in duplicate static TF broadcasters for the same frame pairs.

## 4) Consolidated ROS entities: packages -> nodes -> topics/services/frames

### 4.1 `ros2-vicon-bridge` (C++)

Executable: `vicon_bridge`

Publishes:

- TF:
  - `vicon/world` -> `vicon/<subject>/<segment>`
- PoseStamped:
  - `vicon/<subject>/<segment>/pose`

Executable: `vicon_bridge_retiming`

Publishes:

- TF only (same `vicon/<subject>/<segment>` child naming and `world_frame_id_`/`tf_namespace_` defaults as in its header)

Launch: `ros2-vicon-bridge/launch/all_segments.launch.py`

- Starts Vicon bridge with parameter `host_name` defaulted to `10.157.174.250:801`
- Adds static transforms for `map`, `base_link`, `dock_tag`, and `vicon_real_target`

### 4.2 `docking_test_suite`

#### Core config (topic names + calibration)

Config file:

- `src/docking_test_suite/config/test_config.yaml`

Key topic names in that YAML:

- Vicon:
  - `/vicon/Robot_1/Robot_1/pose` (`vicon_robot_pose`)
  - `/vicon/Tag_0/Tag_0/pose` (`vicon_target_pose`)
- AprilTag detections:
  - `detection_msg_type: "isaac_ros"`
  - `/tag_detections` (`apriltag_detections`)
- Camera:
  - `/camera/camera/color/image_raw` (`camera_image_raw`)
  - `/apriltag/overlay/compressed` (`camera_compressed`)
  - `/camera/camera/color/camera_info` (`camera_info`)
- Robot control:
  - `/cmd_vel`
- PX4 odometry fallback:
  - `/fmu/out/vehicle_odometry`
- PX4 command interfaces:
  - `/fmu/in/vehicle_command`
  - `/fmu/in/offboard_control_mode`

Calibration offsets used for ground-truth relative pose:

- `vicon_target_to_real_target`
  - translation: `[-0.122, 0.0, 0.0]`
  - rotation: `[0.0, 0.0, 0.70710678, 0.70710678]`
- `vicon_robot_to_camera`
  - translation: `[0.1875, 0.0, -0.1875]`
  - rotation: `[0.5, -0.5, 0.5, -0.5]`
- `tag_size: 0.162` (meters)

System flags:

- `use_vicon: true`
- `use_px4_odom_fallback: true`

#### Nodes / entry points

`pose_tf_broadcaster`

- Subscribes:
  - `robot_pose_topic` (default `/vicon/Robot_1/Robot_1/pose`)
  - `target_pose_topic` (default `/vicon/Tag_0/Tag_0/pose`)
- TF output:
  - parent frame: `vicon/world` by default (`parent_frame`)
  - robot child: `vicon/Robot_1/Robot_1` by default
  - target child: `vicon/Tag_0/Tag_0` by default

`docking_diagnostics`

- Subscribes (from YAML):
  - camera info: `/camera/camera/color/camera_info`
  - camera compressed overlay (if present): `/apriltag/overlay/compressed`
  - detections (type depends on `detection_msg_type`):
    - `/tag_detections` as `isaac_ros_apriltag_interfaces/AprilTagDetectionArray`
  - Vicon poses:
    - `/vicon/Robot_1/Robot_1/pose`
    - `/vicon/Tag_0/Tag_0/pose`
  - `/cmd_vel`
  - optional `/fmu/out/vehicle_odometry` (if px4 messages available)

`robot_driver_cli` (interactive CLI)

- Publishes:
  - Twist on `cmd_vel` topic from YAML (`/cmd_vel`)
  - If PX4 messages are available:
    - `OffboardControlMode` on YAML `offboard_control_mode` (`/fmu/in/offboard_control_mode`)
    - `VehicleCommand` on YAML `vehicle_command` (`/fmu/in/vehicle_command`)
- Subscribes (feedback):
  - if `use_vicon: true`: `PoseStamped` on YAML `vicon_robot_pose` (`/vicon/Robot_1/Robot_1/pose`)
  - otherwise (fallback): `VehicleOdometry` on YAML `px4_odometry` (default `/fmu/out/vehicle_odometry`)

`test_a_distance` (Test A)

- Records data only (no robot driving)
- Uses `DataRecorder` to subscribe:
  - camera info: `/camera/camera/color/camera_info`
  - detections: `/tag_detections` (isaac_ros type)
  - compressed overlay: `/apriltag/overlay/compressed`
  - Vicon robot/target poses + synchronized callback (because `use_vicon: true`)
- Outputs:
  - CSV into `recording.output_dir` (default `~/docking_test_data`)
  - Optional annotated JPEG frames saved under a session-specific images folder

`test_b_angular` (Test B)

- Same underlying recording mechanics as Test A, but changes labeling with fixed angles

`test_c_dynamic` (Test C)

- Drives robot and records:
  - Robot movement via `RobotDriver`:
    - publishes `/cmd_vel` (Twist)
    - sends PX4 offboard heartbeat + arm via `VehicleCommand`/`OffboardControlMode` (if px4 msgs are available)
  - Records with `DataRecorder` until tag is within `stop_distance` or timeout occurs

`test_d_environmental` (Test D)

- Same as Test C but repeats across conditions (labels only; simulation environment changes are external)

Data recording internals: `data_recorder.py`

- Subscribes:
  - `CameraInfo` (required): YAML `camera_info` topic
  - compressed overlay (optional but needed for saving images if enabled): YAML `camera_compressed`
  - detections:
    - Vicon synchronized mode: approximates time sync of:
      - `PoseStamped` (robot pose)
      - `PoseStamped` (target pose)
      - AprilTag detection array
    - detection-only mode exists but is selected by `system.use_vicon`
  - optional PX4 odometry fallback: YAML `px4_odometry` if enabled
- Saves:
  - CSV rows with:
    - Vicon world robot/target poses
    - ground-truth relative pose (computed using configured offsets)
    - solvePnP camera estimate from tag corners + calibration
    - error metrics
    - image filename pointer

Launch files (how tests run):

- `docking_test_suite/launch/test_a.launch.py`:
  - starts node `test_a_distance` with `config_path` launch argument
- `docking_test_suite/launch/test_b.launch.py`:
  - starts node `test_b_angular`
- `docking_test_suite/launch/test_c.launch.py`:
  - starts node `test_c_dynamic` and forwards optional `condition_label` argument (used for Test D naming)
- `docking_test_suite/launch/test_d.launch.py`:
  - starts node `test_d_environmental`
- `docking_test_suite/launch/debug_tools.launch.py`:
  - starts `docking_diagnostics` and `robot_driver_cli` (in a separate terminal via `xterm -e`)
- `docking_test_suite/launch/tf_tree.launch.py`:
  - provides docking-specific static TF aliases

### 4.3 `mirmi_apriltag`

#### `apriltag_visualizer` (CPU visualization + overlay)

Subscribes:

- Image:
  - `/camera/image_raw` (Jetson launch style) OR remapped in the RealSense/USB launch variants
- AprilTag detections:
  - `/tag_detections` or `/usb_camera/tag_detections` depending on launch remaps

Publishes:

- `CompressedImage` overlay:
  - `/camera/tag_detections_image/compressed` (default internal name)
  - remapped by launch to docking suite expected overlay:
    - RealSense path remaps output to `/apriltag/overlay/compressed`

#### `depth_visualizer`

Parameters:

- `input_topic` default `/camera/camera/aligned_depth_to_color/image_raw`
- `output_topic` default `/camera/depth_visualization`

Subscribes:

- `input_topic` (sensor_msgs/Image)

Publishes:

- `output_topic/raw` (Image)
- `output_topic/compressed` (CompressedImage)

#### `accuracy_investigation`

Subscriptions (hard-coded in code):

- `/vicon/Robot_1/Robot_1/pose`
- `/vicon/Tag/Tag/pose` (note: this differs from `Tag_0/Tag_0` naming used elsewhere)
- `/camera/detections`
- CameraInfo:
  - `/camera/camera/color/camera_info`

Outputs:

- Writes CSVs to `~/accuracy_data/accuracy_<timestamp>.csv`

#### Launch: `apriltag_detection.launch.py`

Jetson-style pipeline:

- Uses ISAAC ROS composed pipeline:
  - RectifyNode remaps:
    - image_raw -> `/camera/image_raw`
    - camera_info -> `/camera/camera_info`
  - AprilTagNode uses `config/isaac_ros_apriltag_params.yaml`
- Starts `apriltag_visualizer` node subscribing to `/camera/image_raw` and `/tag_detections`
- Starts a static transform:
  - `base_link` -> `camera_color_optical_frame` with translation/rotation matching docking calibration assumptions

#### Launch: `usb_apriltag_detection.launch.py`

Camera type switch (default `camera_type = realsense`):

If `realsense`:

- Starts `realsense2_camera_node` with:
  - `enable_infra1: false`, `enable_infra2: false` (infra streams disabled)
  - `enable_depth: false` (depth streams disabled)
  - publishes color image + camera info under:
    - `/camera/camera/color/image_raw`
    - `/camera/camera/color/camera_info`
- Starts ISAAC composed pipeline for AprilTag:
  - RectifyNode remaps:
    - `/camera/camera/color/image_raw` -> rectify input `image_raw`
    - `/camera/camera/camera/color/camera_info` -> rectify input `camera_info`
  - AprilTagNode publishes `/tag_detections`
- Starts visualizer:
  - `apriltag_visualizer` remaps to write compressed overlay:
    - to `/apriltag/overlay/compressed` (so docking tests find it)
- Starts `depth_visualizer` (only when `camera_type == realsense`), configured with:
  - input `/camera/camera/aligned_depth_to_color/image_raw`
  - output `/camera/depth_visualization`
  - Note: this is currently inconsistent with the RealSense driver config (depth disabled).

If `usb`:

- Starts `jetson_camera/usb_camera_node` with `device_id: 1`
- Starts ISAAC composed pipeline for AprilTag:
  - RectifyNode uses `/usb_camera/image_raw` and `/usb_camera/camera_info`
  - AprilTagNode remaps detections topic to `/usb_camera/tag_detections`
- Starts visualizer remapped accordingly
- Starts no explicit `depth_visualizer` for USB

Important observation (USB camera raw image):

- `jetson_camera/usb_camera_node` currently comments out publication of raw images (`self.raw_pub_.publish(...)` is commented).
- Because the ISAAC pipeline is configured to use `/usb_camera/image_raw`, the USB path may not work unless raw image publishing is re-enabled.

### 4.4 `jetson_camera`

#### `camera_node` (Jetson CSI camera)

Publishes (sensor_msgs):

- `/camera/image_raw/compressed` (CompressedImage, BEST_EFFORT QoS)
- `/camera/image_raw` (Image, RELIABLE QoS)
- `/camera/camera_info` (CameraInfo)

Frame:

- `frame_id = "camera_link"` is set on image + camera info headers.
- No TF is published by this node.

#### `usb_camera_node`

Parameters:

- `device_id` (default `1`)

Publishes:

- `/usb_camera/image_raw/compressed` (CompressedImage)
- `/usb_camera/camera_info` (dummy calibration values)

Raw image publication:

- In the current code, raw `Image` publication is commented out, so subscribers to `/usb_camera/image_raw` will not receive data.

Frame:

- `frame_id = "camera_color_optical_frame"` (header set on messages)

### 4.5 `ir_led_tracker`

Launch: `ir_led_tracker/launch/tracker.launch.py`

- Starts `ir_tracker_node` with defaults:
  - `threshold=40`, `target_frequency=4.0`, `frequency_tolerance=1.0`, `history_size=60`, `debug_update_rate=10.0`

Node: `IRTrackerNode` (`ir_tracker_node`)

Subscribes:

- Infra mono image:
  - `/camera/camera/infra1/image_rect_raw`
- Infra camera info:
  - `/camera/camera/infra1/camera_info`
- Depth:
  - `/camera/camera/depth/image_rect_raw`

Publishes:

- Pose estimate (3D from pixel + depth):
  - `/ir_led/pose` (PoseStamped; uses incoming header stamp)
- Debug:
  - `/ir_led/debug_view` (Image; currently not published because raw debug publish is disabled)
  - `/ir_led/debug_view/compressed` (CompressedImage JPEG at throttled rate)
- Status:
  - `/ir_led/status_debug` (geometry_msgs/Point)
  - `/ir_led/is_tracked` (std_msgs/Bool)

Frame semantics:

- It does not publish TF; it uses `PoseStamped.header.frame_id` copied from message header.

### 4.6 `gps_position_publisher`

Node: `GpsPublisher`

Parameters (defaults):

- `topic = "Position/GPS"`
- `publish_rate_hz = 5.0`
- `latitude = 0.0`, `longitude = 0.0`, `altitude = 0.0`
- `frame_id = "gps"`

Publishes:

- `/Position/GPS` (NavSatFix)

### 4.7 `litime_bms_ros`

Node: `LiTimeBMSNode`

Parameters (defaults):

- `mac = "C8:47:80:18:AE:45"`
- `query_interval = 1.0` seconds
- `notify_uuids` and `write_uuids` set to default LiTime BLE characteristic UUIDs
- `send_registration = True`
- `query_command_hex` default derived from protocol constants

Publishes:

- `litime_bms/state` (sensor_msgs/BatteryState)
- `litime_bms/raw` (std_msgs/String with UUID and raw hex payload)

### 4.8 `video_streamer`

Node: `GStreamerNode`

Parameters:

- `width`, `height`, `framerate`, `bitrate`
- `host_ip` (default `10.157.175.72`)
- `port` (default `5000`)

Behavior:

- Starts a `gst-launch-1.0` pipeline via `subprocess.Popen(...)`.
- Streams an H264 RTP/UDP output to `host_ip:port`.

ROS topics:

- No ROS publishers/subscribers besides the ROS node lifetime itself.

### 4.9 `px4_localization_bridge`

#### `odom_to_px4`

Subscribes:

- `/odom` (nav_msgs/Odometry)

Publishes:

- `/fmu/in/vehicle_odometry` (px4_msgs/VehicleOdometry)

Frame fields and timestamp/sample timestamp are populated where supported by message fields.

#### `vicon_odometry_pub`

Parameters:

- `vicon_topic` default:
  - `/vicon/Robot_1/Robot_1/pose`

Subscribes:

- `vicon_topic` (PoseStamped)

Publishes:

- `/fmu/in/vehicle_visual_odometry` (px4_msgs/VehicleOdometry)

Transform convention implemented in code:

- Position:
  - ENU (Vicon) -> NED (PX4):
    - `x_n = p.y`
    - `y_e = p.x`
    - `z_d = -p.z`
- Quaternion:
  - applies `Q_FLU2FRD` and `Q_ENU2NED` via quaternion multiplication

#### `fake_odometry_pub` and `fake_odometry_pub_2`

Publish to:

- `/fmu/in/vehicle_visual_odometry`

They are dummy producers for EKF testing.

### 4.10 `px4_arm_service`

Node: `command_services` (`px4_arm_service.arm_service:main`)

Publishes:

- `/fmu/in/vehicle_command` (px4_msgs/VehicleCommand)

Provides services (std_srvs/SetBool):

- `/px4/arm`
  - when `data=true`, sends VEHICLE_CMD_COMPONENT_ARM_DISARM (400) with param1=1.0
- `/px4/offboard`
  - when enabled:
    - VEHICLE_CMD_DO_SET_MODE (176) with param2=6.0 (OFFBOARD)
  - when disabled:
    - returns to POSCTL mode param2=3.0

### 4.11 `cmdvel_to_px4`

Note: this package currently contains both:

- Twist->PX4 actuator/rover control (nodes 1/2)
- PX4 SetBool service provider (node 3)

#### Node `cmdvel_to_px4` (`cmdvel_node.py`): `CmdVelToActuatorsSafe`

Subscribes:

- `/cmd_vel` (geometry_msgs/Twist)

Publishes:

- `/fmu/in/offboard_control_mode` (OffboardControlMode; sets `direct_actuator=True`)
- `/fmu/in/vehicle_command` (VehicleCommand for:
  - DO_SET_MODE (offboard)
  - COMPONENT_ARM_DISARM (400))
- `/fmu/in/actuator_motors` (ActuatorMotors)

Control mapping (hard-coded):

- throttle command:
  - Twist.linear.x -> normalized throttle -> `ActuatorMotors.control[motor_index=0]`
- steering command:
  - Twist.angular.z -> normalized steer -> `ActuatorMotors.control[steer_index=1]`
- tool command:
  - Twist.linear.z -> normalized tool -> `ActuatorMotors.control[tool_index=2]`

Implements:

- deadman timeout (`/cmd_vel` age > `0.3s` => commands set to 0)
- warmup delay before offboard+arm

#### Node `cmdvel_to_px4_2` (`cmdvel_node2.py`): `CmdVelToPx4Rover`

Subscribes:

- `/cmd_vel` (Twist)
- `/stop` (std_msgs/Empty)

Publishes:

- `/fmu/in/offboard_control_mode` (direct actuator)
- `/fmu/in/vehicle_command` (DO_SET_MODE, ARM/DISARM, and DO_SET_ACTUATOR)
- `/fmu/in/actuator_motors` (ActuatorMotors)
- `/fmu/in/actuator_servos` (ActuatorServos)

Implements:

- warmup before offboard + arm
- deadman stop if command times out
- optional invert speed/steer (hard-coded defaults)
- steering actuator may be mapped to servo bank (default `steering_on_main=False`)

#### Node `cmdvel_to_px4_3` (`cmdvel_node3.py`): `Px4CommandServices` (SetBool)

Publishes:

- `/fmu/in/vehicle_command`

Provides services:

- `/px4/arm` (std_srvs/SetBool)
- `/px4/offboard` (std_srvs/SetBool)

Important: this service-provider functionality overlaps with `px4_arm_service`.

### 4.12 `cmdvel_to_px4` launch files (current wiring)

#### `cmdvel_to_px4/launch/px4_bridge.launch.py`

- Creates nodes:
  - `litime_bms_ros_node` (always launched)
  - `cmdvel_to_px4_3` (service provider) launched after `TimerAction(period=3.0, actions=[cmdvel])`
- It also defines but comments out:
  - `vicon_odometry_pub` (px4_localization_bridge) in this launch

#### `cmdvel_to_px4/launch/startup_combined.launch.py`

- Includes:
  - `cmdvel_to_px4/launch/px4_bridge.launch.py`
  - `foxglove_bridge/launch/foxglove_bridge_launch.xml`
- Starts additional nodes:
  - `px4_localization_bridge/vicon_odometry_pub` with param `vicon_topic=/vicon/Robot_1/Robot_1/pose`
  - `px4_arm_service/command_services` (service provider)
  - `mirmi_apriltag/launch/usb_apriltag_detection.launch.py` (default `camera_type=realsense`)
- Calls (after 5s):
  - `ros2 service call /px4/offboard std_srvs/srv/SetBool '{data: true}'`

## 5) Topic-by-topic “who talks to who” (current documented view)

This section is derived from the code and YAML configuration in-tree. Where launch files remap topics, the remapped names are included.

### Perception / vision

- Camera:
  - `/camera/camera/color/image_raw`:
    - publisher: `realsense2_camera_node` (in `mirmi_apriltag/usb_apriltag_detection.launch.py`)
  - `/camera/camera/color/camera_info`:
    - publisher: `realsense2_camera_node`
  - `/tag_detections`:
    - publisher: `isaac_ros_apriltag` (in `mirmi_apriltag/usb_apriltag_detection.launch.py` RealSense path)
  - Compressed tag overlay (expected by docking suite):
    - `/apriltag/overlay/compressed`:
      - publisher: `mirmi_apriltag/apriltag_visualizer` via launch remap

### Docking test suite recording (synchronized)

- Subscribes (from `docking_test_suite/config/test_config.yaml`):
  - `/camera/camera/color/camera_info`
  - `/apriltag/overlay/compressed`
  - `/tag_detections` (isaac_ros AprilTagDetectionArray)
  - `/vicon/Robot_1/Robot_1/pose`
  - `/vicon/Tag_0/Tag_0/pose`
  - optional `/fmu/out/vehicle_odometry`

- Publishes:
  - No ROS output topics; outputs are CSV files and optional saved annotated JPEG frames on disk.

### Robot control

- `/cmd_vel`:
  - publisher: docking suite:
    - `robot_driver` publishes Twist (used in Test C/D driving)
  - subscriber:
    - `cmdvel_to_px4` node (cmdvel_node / cmdvel_node2) converts Twist into PX4 commands

- `/stop` (Empty):
  - subscriber: `cmdvel_to_px4_2` rover node

### PX4 command and odometry bridges

- `/fmu/in/offboard_control_mode`:
  - publisher: cmdvel controller nodes and docking suite driver when PX4 msgs are present
  - consumer: PX4 / fmu (external)

- `/fmu/in/vehicle_command`:
  - publisher:
    - cmdvel controller nodes (arm + offboard + etc)
    - `px4_arm_service` and `cmdvel_to_px4_3` (services)
  - consumer: PX4 / fmu (external)

- `/fmu/in/vehicle_visual_odometry`:
  - publisher:
    - `px4_localization_bridge/vicon_odometry_pub`
    - `fake_odometry_pub` and `fake_pub`
  - consumer: PX4 EKF2 / EKF (external)

- `/fmu/in/vehicle_odometry`:
  - publisher: `px4_localization_bridge/odom_to_px4`
  - consumer: PX4 EKF (external)

- `/fmu/out/vehicle_odometry`:
  - subscriber: docking test suite driver/recorder fallback (if enabled)

### Vicon pose/TF

- `/vicon/<subject>/<segment>/pose`:
  - publisher: `ros2-vicon-bridge/vicon_bridge`
  - subscribers:
    - docking test suite (Vicon poses)
    - `px4_localization_bridge/vicon_odometry_pub`
    - `docking_test_suite/pose_tf_broadcaster`

## 6) Known “current wiring” issues/quirks found in-code

These are not “guesses”: they are directly implied by the current launch parameters and the current published/subscribed topic names in code.

1. Duplicate TF static publishers for docking frames
   - `ros2-vicon-bridge/launch/all_segments.launch.py` and `docking_test_suite/launch/tf_tree.launch.py` both publish:
     - `map -> vicon/world`
     - `vicon/Robot_1/Robot_1 -> base_link`
     - `vicon/Tag_0/Tag_0 -> dock_tag`
     - `vicon/Tag_0/Tag_0 -> vicon_real_target`
   - If both are running together, TF tools may warn about multiple publishers for the same static transform.

2. Duplicate `/px4/arm` and `/px4/offboard` service providers
   - `cmdvel_to_px4/launch/px4_bridge.launch.py` starts `cmdvel_to_px4_3` (SetBool services).
   - `cmdvel_to_px4/launch/startup_combined.launch.py` also starts `px4_arm_service/command_services` (same service names + types).
   - This can cause service creation conflicts (only one node can own a ROS service name).

3. RealSense depth and infra are disabled but dependent nodes are configured to use them
   - `mirmi_apriltag/usb_apriltag_detection.launch.py` sets:
     - `enable_depth: false`
     - `enable_infra1: false`, `enable_infra2: false`
   - Yet the same launch includes:
     - `mirmi_apriltag/depth_visualizer` subscribing to `/camera/camera/aligned_depth_to_color/image_raw`
   - And the repo has `ir_led_tracker` subscribing to:
     - `/camera/camera/infra1/image_rect_raw`
     - `/camera/camera/depth/image_rect_raw`
   - As-is, depth/infra-based tracking/visualization may not receive any data when using the default `realsense` camera config.

4. USB camera path likely does not work “as-is”
   - `jetson_camera/usb_camera_node.py` has the raw publish commented out.
   - But `mirmi_apriltag/usb_apriltag_detection.launch.py` expects `/usb_camera/image_raw` (for ISAAC RectifyNode input).

## 7) Minimal “how everything works right now” startup recipe (based on the launch files)

Because the repo contains multiple “stacks” (sensing, Vicon, AprilTag, PX4, docking-test recording), you generally need to start several pieces together.

### A) Docking accuracy evaluation (Test A/B/C/D)

Prerequisites:

- Start Vicon bridge so that `/vicon/<subject>/<segment>/pose` topics exist and TF frames exist:
  - `ros2-vicon-bridge/launch/all_segments.launch.py`
- Start TF aliases/docking frames (optional but recommended for consistent downstream frames):
  - `docking_test_suite/launch/tf_tree.launch.py`
- Start camera + AprilTag detection pipeline so that:
  - `/camera/camera/color/camera_info` exists
  - `/tag_detections` exists
  - `/apriltag/overlay/compressed` exists

Then run:

- `docking_test_suite/launch/test_a.launch.py`
- `docking_test_suite/launch/test_b.launch.py`
- `docking_test_suite/launch/test_c.launch.py`
- `docking_test_suite/launch/test_d.launch.py`

These will:

- load `docking_test_suite/config/test_config.yaml` (unless overridden with `config_path`)
- record synchronized samples into CSV (+ optional images)
- for Test C/D: drive the robot using `/cmd_vel`

### B) PX4 offboard + sensor setup (combined launch)

Start:

- `cmdvel_to_px4/launch/startup_combined.launch.py`

This will bring up:

- BMS node
- Vicon->PX4 visual odometry publication
- AprilTag detection (RealSense default camera)
- offboard mode enable service call

Important:

- This combined launch does not start the actual `/cmd_vel` -> PX4 actuator controller nodes (`cmdvel_node` / `cmdvel_node2`).
- If you want the robot to move, you likely need to start one of the controller nodes (`cmdvel_to_px4` or `cmdvel_to_px4_2`) in addition, and ensure only one node provides `/px4/arm` and `/px4/offboard`.

