# System Dokumentation & Setup Guide

## Übersicht
Dieses Repository beinhaltet den Workspace für einen Roboter, der über ein **Holybro Pixhawk Jetson Baseboard (SKU: 20338X)** gesteuert wird.

### Hardware
*   **Baseboard**: Holybro Pixhawk Jetson Baseboard
*   **Verbindung**: Pixhawk und Jetson sind intern per Ethernet verbunden.

### Software & Versionen
*   **Jetson Board**: Ubuntu 22.04.5 LTS (Jammy) | ROS 2 Humble
*   **Pixhawk Flight Controller**: PX4 v1.17 alpha (oder v1.16)
*   **Steuerung**: PWM Signale für Fortbewegung und Rotation

---

## 1. Netzwerk

### WiFi
*   **SSID**: `frl_2.4GHz`
*   **Passwort**: `Frl2024!`
*   **Hostname/IP Check**: `hostname -I` (Sollte `10.157.175.187` sein, sonst im Bridged Modus prüfen)

### SSH Verbindung
Verbindung zum Jetson Board:
```bash
ssh holybro@10.157.174.254
```
**Passwort**: `frl2025!`

---

## 2. Vicon & Lokalisierung

### Debugging
*   **Problem**: Navigation Error (position / global position)
*   **Lösung**: Reboot des Jetson.
*   **Alternativ**: In QGroundControl: `Vehicle configuration` -> `parameter` -> `tools` -> `reboot vehicle`.
*   **Hinweis**: EKF2 benötigt einen Delay von ca. **45 ms**. Wenn der Delay vom Vicon System zu groß wird, entstehen Probleme.

### Vicon System Starten
1.  Switch einschalten.
2.  Einloggen am Vicon PC:
    *   **Account**: `MSA-Mocap`
    *   **Passwort**: `msA!54`
3.  App öffnen: **Vicon Tracker**
4.  Drücke: **Go live**
5.  **Requested Frame Rate**: `30` (überprüfen)

### Vicon Bridge (Ubuntu)
Repository: [ros2-vicon-bridge](https://github.com/dasc-lab/ros2-vicon-bridge)

Sicherstellen, dass man im selben Netz ist wie der Roboter. Dann die Bridge starten:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch vicon_bridge all_segments.launch.py
```

### PX4 Localization Bridge (Auf dem Roboter)
Die Odometrie-Daten an PX4 senden:
```bash
ros2 run px4_localization_bridge vicon_odometry_pub --ros-args -p vicon_topic:=/vicon/Robot_1/Robot_1/pose
```

---

## 3. Roboter Steuerung

### Arming (Motoren freischalten)
Per SSH auf dem Roboter ausführen:
```bash
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{timestamp: $(($(date +%s%N)/1000)), command: 400, param1: 1.0, target_system: 1, target_component: 1, from_external: true}"
```

### Offboard Modus
Wechsel in den Modus, in dem der Jetson die Steuerung übernimmt:
```bash
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{timestamp: $(($(date +%s%N)/1000)), command: 176, param1: 1.0, param2: 6.0, target_system: 1, target_component: 1, from_external: true}"
```

---

## 4. Sensoren & Vision

### Kamera
Kamera Node auf dem Jetson starten:
```bash
ros2 run jetson_camera camera_node
```

### AprilTag Erkennung
```bash
ros2 launch mirmi_apriltag usb_apriltag_detection.launch.py
```

### LiDAR Pole Detection (`rover_control/pole_detector`)
Node starten:
```bash
cd ~/Repos/MIRMI_robot
source install/setup.bash
ros2 run rover_control pole_detector
```

Wichtige Topics:
* Input PointCloud: Parameter `cloud_topic` (Default: `/livox/lidar`)
* Output Pole Position: `/pole_estimate` (`geometry_msgs/PointStamped`)
* Output Confidence: `/pole_confidence` (`std_msgs/Float32`)

Wichtige Parameter (live änderbar mit `ros2 param set`):
* Geometrie/ROI: `z_min`, `z_max`, `r_min`, `r_max`
* Cluster: `grid_res`, `min_points_per_cell`, `min_cells_per_cluster`, `min_points_per_cluster`
* Pole Prior: `expected_diameter`, `diameter_tolerance`, `expected_range`, `range_tolerance`
* Tracking: `ema_alpha`, `lock_frames`, `track_gate_distance`
* RANSAC: `use_ransac`, `ransac_max_iters`, `ransac_inlier_thresh`, `ransac_min_inlier_ratio`, `ransac_min_inliers`

Beispiele:
```bash
# RANSAC aktivieren/deaktivieren
ros2 param set /pole_detector use_ransac true
ros2 param set /pole_detector use_ransac false

# RANSAC robuster gegen Ausreißer machen (strenger / lockerer)
ros2 param set /pole_detector ransac_inlier_thresh 0.025
ros2 param set /pole_detector ransac_min_inlier_ratio 0.50
```

---

## 5. Visualisierung & GCS

### Foxglove
Verbindung über WebSocket.
*   **IP Adresse**: `10.157.174.254` (oder via `ip -a` unter `wlP1p1s0` prüfen)
*   **URL**: `ws://10.157.174.254:8765`

Foxglove Bridge auf dem Jetson starten:
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### QGroundControl (QGC)
1.  Per USB-C mit dem Baseboard verbinden.
2.  QGroundControl starten.

---

## 6. Genauigkeits-Evaluierung

Um die Genauigkeit der AprilTag Erkennung gegenüber dem Vicon System zu evaluieren:

```bash
ros2 run mirmi_apriltag accuracy_investigation
```

Das Skript speichert die Daten in einem lokalen Ordner mit Zeitstempel:
`~/accuracy_data/accuracy_YYYY-MM-DD_HH-MM-SS.csv`

---

## 7. Daten Backup (Google Drive)

Um die gesamten Messdaten zu sichern und zu synchronisieren, verwenden wir `rclone`.

### Installation & Setup
1.  Rclone installieren (Benötigt sudo):
    ```bash
    sudo apt install rclone
    ```
2.  Google Drive konfigurieren:
    ```bash
    rclone config
    ```
    *   `n` (New remote)
    *   Name: `gdrive`
    *   Storage: `drive` (Google Drive)
    *   Folge den Anweisungen zur Authentifizierung.

### Backup & Synchronisierung
Nutzen Sie das bereitgestellte Skript, um den lokalen Ordner `~/accuracy_data` mit dem Cloud-Ordner `accuracy_data` zu synchronisieren:

```bash
cd ~/ws_sensor_combined
./backup_data.sh
```
