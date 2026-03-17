# Docking Test Suite

This package provides a comprehensive suite for testing the accuracy and performance of the robot's docking system, including static distance profiles, angular profiles, dynamic approaches, and environmental stress tests.

## 1. Overview
The suite records synchronized sensor data (Vicon, AprilTag detections, camera images, and PX4 odometry) to evaluate the performance of AprilTag-based localization and the robot's control system during docking maneuvers.

## 2. Configuration
The main configuration file is located at `config/test_config.yaml`. 
Here you can configure:
* **Topics**: Vicon, camera, and PX4 topic names.
* **Calibration**: Offsets between Vicon markers and the actual sensors/targets.
* **Test Parameters**: Distances, angles, and speeds for different tests.
* **Recording Settings**: Output directory (`~/docking_test_data` by default), image saving rate, etc.

## 3. Usage

### Running Tests
To run the automated tests, use the provided launch files or run the test nodes directly after building the workspace.

For example, to run the node that performs test A (Static Distance Profile):
```bash
ros2 run docking_test_suite test_a_static_distance
```
*(Make sure to source the workspace first: `source install/setup.bash`)*

### Analyzing Data
The test suite records data into CSV files and directories with pictures inside `~/docking_test_data`. You can find analysis scripts in the `docking_test_suite/analysis.py` module to parse and visualize this data.

## 4. Google Drive Backup
To sync the recorded test data to Google Drive, a script `backup_docking_data.sh` is provided in the workspace root. It uses `rclone` to mirror the local `~/docking_test_data` directory to Google Drive.

### Setup (if not already done)
1. Install rclone:
   ```bash
   sudo apt install rclone
   ```
2. Configure rclone for Google Drive:
   ```bash
   rclone config
   ```
   * Press `n` to create a new remote.
   * Name it: `gdrive`
   * Select `drive` (Google Drive) as the storage type.
   * Follow the steps for authentication in your browser.

### Running the Backup
Run the backup script from the workspace root:

```bash
cd ~/ws_sensor_combined
./backup_docking_data.sh
```
This will sync all data from `~/docking_test_data` to a folder named `docking_test_data` on your Google Drive.
