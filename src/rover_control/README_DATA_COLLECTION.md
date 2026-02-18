# Rover Data Collection (BC -> LfI)

Diese Datei beschreibt nur das Aufnehmen von Trainingsdaten mit den neuen Nodes in `rover_control`.

## Voraussetzungen

- Workspace gebaut und gesourced
- Mux/Teleop Setup läuft mit:
  - `/cmd_vel_policy`, `/cmd_vel_human`, `/cmd_vel`
  - `/tool_speed_policy`, `/tool_speed_human`, `/tool_speed_cmd`
  - `/intervening`

## 1) Build + Source

```bash
cd /home/lennart/Repos/MIRMI_robot
colcon build --packages-select rover_control
source install/setup.bash
```

## 2) Pole-Detector starten

Der Pole-Detector liefert die Features für `extra` im Logger:
- `/pole_estimate`
- `/pole_confidence`
- `/pole_diameter`

Beispielstart (an dein Setup anpassen):

```bash
ros2 run rover_control pole_detector --ros-args \
  -p cloud_topic:=/livox/lidar \
  -p expected_range:=0.50 \
  -p expected_diameter:=0.10 \
  -p min_diameter:=0.07 \
  -p max_diameter:=0.25 \
  -p r_max:=1.5 \
  -p track_gate_distance:=0.22
```

Checks:

```bash
ros2 topic echo /pole_estimate --once
ros2 topic echo /pole_confidence --once
ros2 topic echo /pole_diameter --once
```

## 3) LiDAR preprocess starten (PointCloud2 -> feste Polar-Bins)

```bash
ros2 run rover_control lidar_preprocess_node --ros-args \
  -p cloud_topic:=/livox/lidar \
  -p scan_topic:=/lidar_polar \
  -p num_bins:=360 \
  -p z_min:=-0.2 \
  -p z_max:=1.5 \
  -p r_min:=0.05 \
  -p r_max:=10.0
```

Check:

```bash
ros2 topic echo /lidar_polar --once
```


## 4) BC/LfI Mux starten

Neuer Mux für Datensammlung:
- `cmd_vel_mux_lfi`
- publisht `/cmd_vel`, `/tool_speed_cmd`, `/intervening`, `/mux_source`

BC-Modus (nur Human als Quelle):

```bash
ros2 run rover_control cmd_vel_mux_lfi --ros-args \
  -p mode:=bc \
  -p cmd_human_topic:=/cmd_vel_human \
  -p cmd_policy_topic:=/cmd_vel_policy \
  -p cmd_out_topic:=/cmd_vel \
  -p tool_human_topic:=/tool_speed_human \
  -p tool_policy_topic:=/tool_speed_policy \
  -p tool_out_topic:=/tool_speed_cmd
```

LfI-Modus (Policy default, Human override):

```bash
ros2 run rover_control cmd_vel_mux_lfi --ros-args \
  -p mode:=lfi \
  -p timeout_human_s:=0.3 \
  -p timeout_policy_s:=0.3
```

Optional: Tool-Speed ueber `Twist.linear.z` statt separater `tool_speed_*` Topics:

```bash
ros2 run rover_control cmd_vel_mux_lfi --ros-args \
  -p mode:=lfi \
  -p use_cmd_linear_z_for_tool:=true
```

Checks:

```bash
ros2 topic echo /intervening --once
ros2 topic echo /mux_source --once
```

## 5) Logger starten

```bash
ros2 run rover_control lfi_logger_node --ros-args \
  -p out_dir:=/tmp/lfi_logs \
  -p session_name:=bc_demo_01 \
  -p sample_hz:=15.0 \
  -p require_pole_conf_min:=0.2
```

Stoppen mit `Ctrl+C`.
Der Logger schreibt dann automatisch eine Datei:

- `/tmp/lfi_logs/<session_name>_YYYYmmdd_HHMMSS.npz`

## 6) BC-Daten sammeln (erste Phase)

- Policy aus oder ignorieren
- Du fährst manuell (Teleop)
- Saubere Demonstrationen, verschiedene Startpositionen
- 20-60 Minuten gesamt

Empfehlung: mehrere Sessions statt einer sehr langen.

## 7) LfI-Daten sammeln (zweite Phase)

- BC-Policy fährt
- Du greifst nur bei Fehlern ein (Mux -> `/intervening=true`)
- 10-20 Minuten pro Session
- Mehrere Iterationen aufnehmen

## 8) Wichtige NPZ-Felder (für Training in `/home/lennart/ml_training`)

Der Logger speichert:

- `t`
- `lidar`
- `extra` = `[pole_x, pole_y, pole_conf, pole_diam, v, w]`
- `a_policy` = `[v, w, tool_speed]`
- `a_human` = `[v, w, tool_speed]`
- `a_exec` = `[v, w, tool_speed]`
- `intervening`

## 9) Training danach (Referenz)

In deinem Training-Ordner:

```bash
cd /home/lennart/ml_training
source .venv/bin/activate
```

BC:

```bash
python train_bc.py --train_npz /tmp/lfi_logs/bc_demo_01_*.npz --out_dir models --device cpu --batch_size 64
```

LfI:

```bash
python train_lfi.py \
  --train_npz /tmp/lfi_logs/lfi_rollout_*.npz \
  --init_ckpt models/policy_bc.pt \
  --stats_path models/norm_stats_bc.npz \
  --out_dir models \
  --device cpu \
  --batch_size 64
```

## 10) Typische Fehler

- Keine Daten gespeichert: prüfe, ob `/lidar_polar` und `/pole_estimate` wirklich kommen.
- Sehr wenige Samples: `require_pole_conf_min` zu hoch.
- Falsche Action-Labels: prüfe Topic-Namen für Human/Policy/Exec im Logger.
