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
