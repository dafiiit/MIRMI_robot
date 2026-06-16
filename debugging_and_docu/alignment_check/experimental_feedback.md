# Experimental Setup and Campaigns Feedback

This document reviews the documentation of the experiments in the thesis ([main.pdf](file:///home/holybro/ws_sensor_combined/debugging_and_docu/alignment_check/main.pdf)), checks their accuracy against the workspace, and proposes improvements and missing experiments.

---

## 1. Discrepancy in Experimental Test Scripts
*   **Thesis Claim (Chapter 4 & 5):** 
    The thesis describes Test A (distance sweep), Test B (angular sweep), Test C (dynamic), and Test D (environmental) with their respective launch files (`test_a.launch.py`, etc.) and entry points (`test_a_distance`, etc.).
*   **Physical Reality:**
    None of these scripts or launch files exist in the workspace. They have been replaced by a unified, topic-driven data-gathering pipeline:
    *   **Orchestration Node:** [sensor_sweep_node.py](file:///home/holybro/ws_sensor_combined/src/docking_test_suite/docking_test_suite/sensor_sweep_node.py) runs a finite state machine (`IDLE` → `CONFIGURED` → `CAPTURING` → `UPLOADING` → `DONE`), taking configurations over `/sweep_test/cmd/configure` and trigger commands.
    *   **Data Recording:** [sweep_recorder.py](file:///home/holybro/ws_sensor_combined/src/docking_test_suite/docking_test_suite/sweep_recorder.py) saves synchronized samples to a CSV file and triggers a raw `rosbag2` subprocess.
    *   **Data Upload:** Completed sweeps are automatically uploaded to Google Drive using `rclone`.
*   **Recommendation:** Update Chapters 4 and 5 of the thesis to document the unified topic-driven state machine (`sensor_sweep_node` and `SweepRecorder`) and `rclone` upload architecture rather than describing the obsolete Test A–D scripts.

---

## 2. Resolving the Real-World Closed-Loop GAP
*   **Thesis GAP Note (Sections TOC & 6.2):**
    `GAP: physical closed-loop runs (≤ 3) to be collected 2026-06-16; insert the one-sentence qualitative result here and remove this note.` (Line 6030)
*   **Physical Reality:**
    The operator ran `rtk_sweep_capture.py` on 2026-06-16 (today) in automatic mode (`--auto`) with `--go` enabled to drive the tracked rover and perform visual servoing on the YOLO-bearing.
*   **Recommendation:** The placeholder in Section 6.2 (line 6030) should be replaced with the following text:
    > "On the physical robot, a qualitative feasibility check (3 runs) verified that the robot can successfully execute the approach using RTK-GPS and visual servoing on the YOLO-bearing, achieving stable facing alignment."
    
    The corresponding notes in the Table of Contents (line 310, 321) should be removed.

---

## 3. Recommended Additional Experiments / Updates

### A. Quantifying LiDAR Degradation under Weather (Simulation)
*   **Context:** Currently, Section 5.2.1 argues LiDAR weather robustness purely from literature because the Gaussian Splatting weather pipeline only affects camera (RGB) inputs.
*   **Recommendation:** Conduct simulation-based weather sweeps for the LiDAR detector by post-processing the Gazebo point clouds (applying scattering/noise filters to simulate rain and fog) or by tuning the GPU-LiDAR parameters. This would supply the quantitative weather degradation figures currently missing for LiDAR.

### B. YOLOv8-pose 3D Accuracy Evaluation
*   **Context:** The thesis evaluates YOLO keypoints in pixel space (keypoint error) and bearing space, yet conceptually places it as a 3D pose source.
*   **Recommendation:** Implement a Perspective-n-Point (PnP) solver for the 8 regressed corners of the station in `SweepRecorder`, mirroring the AprilTag PnP. This would allow a direct, fair 3D translation and rotation error comparison between all three detectors (AprilTag, LiDAR, and YOLO) across range and orbit sweeps.

### C. Real-World Closed-Loop Mating Statistics
*   **Context:** Section 5.5 evaluates closed-loop end-to-end performance primarily in simulation (N=10 runs) due to the difficulty of obtaining outdoor ground truth.
*   **Recommendation:** Perform a quantitative real-world test campaign (e.g. 5–10 docking runs) under clear weather conditions, measuring the physical offset of the docking plate at the terminal mating interface to report success rate and terminal alignment statistics (lateral and angular offsets) on hardware.
