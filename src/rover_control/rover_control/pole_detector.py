#!/usr/bin/env python3
import math
from collections import defaultdict, deque
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32

Cell = Tuple[int, int]


class PoleDetector(Node):
    """
    Detects a pole/tree trunk in PointCloud2 data using XY clustering.

    Pipeline:
    1) Filter by z-band and radial ROI.
    2) Project to XY and build grid occupancy.
    3) Cluster occupied cells via 8-neighborhood BFS.
    4) Score clusters using expected diameter/range and tracking continuity.
    5) Publish pole estimate and confidence.
    """

    def __init__(self) -> None:
        super().__init__("pole_detector")

        self.declare_parameter("cloud_topic", "/livox/lidar")
        self.declare_parameter("z_min", 0.10)
        self.declare_parameter("z_max", 1.40)
        self.declare_parameter("r_min", 0.20)
        self.declare_parameter("r_max", 2.00)
        self.declare_parameter("grid_res", 0.03)
        self.declare_parameter("min_points_per_cell", 2)
        self.declare_parameter("min_cells_per_cluster", 4)
        self.declare_parameter("min_points_per_cluster", 25)
        self.declare_parameter("min_cluster_diameter", 0.06)
        self.declare_parameter("max_cluster_diameter", 0.18)
        self.declare_parameter("expected_diameter", 0.10)
        self.declare_parameter("diameter_tolerance", 0.06)
        self.declare_parameter("expected_range", 0.50)
        self.declare_parameter("range_tolerance", 0.35)
        self.declare_parameter("ema_alpha", 0.30)
        self.declare_parameter("lock_frames", 3)
        self.declare_parameter("track_gate_distance", 0.35)
        self.declare_parameter("w_density", 0.02)
        self.declare_parameter("w_points", 0.50)
        self.declare_parameter("w_diameter", 0.80)
        self.declare_parameter("w_range", 1.00)
        self.declare_parameter("w_track", 2.00)
        self.declare_parameter("w_diam_penalty", 2.00)
        self.declare_parameter("detection_log_min_conf", 0.40)
        self.declare_parameter("detection_log_interval_s", 1.0)
        self.declare_parameter("lost_log_miss_frames", 5)

        self._has_track = False
        self._track_xy = np.zeros(2, dtype=np.float32)
        self._track_conf = 0.0
        self._stable_count = 0
        self._was_detected = False
        self._last_detection_log_ns = 0
        self._missed_frames = 0

        cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.sub = self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, 10)
        self.pub_pt = self.create_publisher(PointStamped, "/pole_estimate", 10)
        self.pub_conf = self.create_publisher(Float32, "/pole_confidence", 10)

        self.get_logger().info(f"PoleDetector listening on {cloud_topic}")

    def on_cloud(self, msg: PointCloud2) -> None:
        # Parameters are read on each callback so they can be tuned live via `ros2 param set`.
        z_min = float(self.get_parameter("z_min").value)
        z_max = float(self.get_parameter("z_max").value)
        r_min = float(self.get_parameter("r_min").value)
        r_max = float(self.get_parameter("r_max").value)
        grid_res = float(self.get_parameter("grid_res").value)
        min_ppc = int(self.get_parameter("min_points_per_cell").value)
        min_cells_cluster = int(self.get_parameter("min_cells_per_cluster").value)
        min_pts_cluster = int(self.get_parameter("min_points_per_cluster").value)
        min_diam = float(self.get_parameter("min_cluster_diameter").value)
        max_diam = float(self.get_parameter("max_cluster_diameter").value)
        exp_diam = float(self.get_parameter("expected_diameter").value)
        diam_tol = float(self.get_parameter("diameter_tolerance").value)
        exp_r = float(self.get_parameter("expected_range").value)
        r_tol = float(self.get_parameter("range_tolerance").value)
        ema_alpha = float(self.get_parameter("ema_alpha").value)
        lock_frames = int(self.get_parameter("lock_frames").value)
        gate_dist = float(self.get_parameter("track_gate_distance").value)
        w_density = float(self.get_parameter("w_density").value)
        w_points = float(self.get_parameter("w_points").value)
        w_diameter = float(self.get_parameter("w_diameter").value)
        w_range = float(self.get_parameter("w_range").value)
        w_track = float(self.get_parameter("w_track").value)
        w_diam_penalty = float(self.get_parameter("w_diam_penalty").value)

        # 1) Point filtering: keep only points in relevant height/range window.
        pts_xy: List[Tuple[float, float]] = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            if z < z_min or z > z_max:
                continue
            radius = math.hypot(x, y)
            if radius < r_min or radius > r_max:
                continue
            pts_xy.append((x, y))

        if len(pts_xy) < min_pts_cluster:
            self._publish_none(msg)
            return

        # 2) XY grid occupancy: compact representation for robust clustering.
        xy = np.asarray(pts_xy, dtype=np.float32)
        inv = 1.0 / max(grid_res, 1e-6)
        ix = np.floor(xy[:, 0] * inv).astype(np.int32)
        iy = np.floor(xy[:, 1] * inv).astype(np.int32)

        cell_counts: Dict[Cell, int] = defaultdict(int)
        cell_points: Dict[Cell, List[Tuple[float, float]]] = defaultdict(list)
        for x, y, cx, cy in zip(xy[:, 0], xy[:, 1], ix, iy):
            cell = (int(cx), int(cy))
            cell_counts[cell] += 1
            cell_points[cell].append((float(x), float(y)))

        occ_cells = {cell for cell, cnt in cell_counts.items() if cnt >= min_ppc}
        if len(occ_cells) < min_cells_cluster:
            self._publish_none(msg)
            return

        # 3) Connected components over occupied grid cells (8-neighborhood).
        neighbors = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1), (0, 1),
            (1, -1), (1, 0), (1, 1),
        ]
        visited = set()
        clusters: List[List[Cell]] = []
        for cell in occ_cells:
            if cell in visited:
                continue
            queue = deque([cell])
            visited.add(cell)
            component = [cell]
            while queue:
                current = queue.popleft()
                for dx, dy in neighbors:
                    nxt = (current[0] + dx, current[1] + dy)
                    if nxt in occ_cells and nxt not in visited:
                        visited.add(nxt)
                        queue.append(nxt)
                        component.append(nxt)
            if len(component) >= min_cells_cluster:
                clusters.append(component)

        if not clusters:
            self._publish_none(msg)
            return

        best_centroid: Optional[np.ndarray] = None
        best_diam = 0.0
        best_pts = 0
        best_score = -1e18

        # 4) Evaluate each cluster and keep the best pole candidate.
        for component in clusters:
            pts_list: List[Tuple[float, float]] = []
            total_pts = 0
            for cell in component:
                pts_cell = cell_points.get(cell, [])
                total_pts += len(pts_cell)
                pts_list.extend(pts_cell)

            if total_pts < min_pts_cluster:
                continue

            points = np.asarray(pts_list, dtype=np.float32)
            centroid = points.mean(axis=0)

            # Hard gate around current track to avoid jumping to nearby distractors.
            if self._has_track and float(np.linalg.norm(centroid - self._track_xy)) > gate_dist:
                continue

            # Approximate diameter from cluster spread around centroid.
            dist = np.sqrt(((points - centroid) ** 2).sum(axis=1))
            diameter = 2.0 * float(dist.max())
            if diameter < min_diam or diameter > max_diam:
                continue

            # Score terms: geometric priors + density + track continuity.
            density = total_pts / max(diameter, 1e-3)
            range_c = float(np.linalg.norm(centroid))
            range_score = -abs(range_c - exp_r) / max(r_tol, 1e-3)
            diam_score = -abs(diameter - exp_diam) / max(diam_tol, 1e-3)

            track_bonus = 0.0
            if self._has_track:
                dist_to_track = float(np.linalg.norm(centroid - self._track_xy))
                track_bonus = -w_track * dist_to_track

            score = (
                w_density * density
                + w_points * math.log(total_pts + 1.0)
                + w_range * range_score
                + w_diameter * diam_score
                + track_bonus
                - w_diam_penalty * diameter
            )

            if score > best_score:
                best_score = score
                best_centroid = centroid
                best_diam = diameter
                best_pts = total_pts

        if best_centroid is None:
            self._publish_none(msg)
            return

        # 5) Confidence: combine evidence from sample count, diameter match, and range match.
        conf = 0.0
        conf += min(1.0, best_pts / (min_pts_cluster * 2.0)) * 0.55
        conf += max(0.0, 1.0 - abs(best_diam - exp_diam) / max(diam_tol, 1e-3)) * 0.25
        conf += max(0.0, 1.0 - abs(float(np.linalg.norm(best_centroid)) - exp_r) / max(r_tol, 1e-3)) * 0.20
        conf = float(np.clip(conf, 0.0, 1.0))

        # 6) EMA tracking smooths frame-to-frame noise and stabilizes control input.
        if not self._has_track:
            self._track_xy = best_centroid.copy()
            self._track_conf = conf
            self._has_track = True
            self._stable_count = 1
        else:
            jump = float(np.linalg.norm(best_centroid - self._track_xy))
            if jump < 0.25:
                self._stable_count += 1
            else:
                self._stable_count = 1

            a = float(np.clip(ema_alpha, 0.01, 0.99))
            self._track_xy = (1.0 - a) * self._track_xy + a * best_centroid
            self._track_conf = (1.0 - a) * self._track_conf + a * conf

        locked = self._stable_count >= lock_frames
        # Before lock is reached, confidence is intentionally reduced.
        pub_conf = self._track_conf if locked else (0.3 * self._track_conf)
        self._missed_frames = 0
        self._maybe_log_detection(self._track_xy, pub_conf, best_diam, msg.header.frame_id)
        self._publish_estimate(msg, self._track_xy, pub_conf)

    def _publish_estimate(self, cloud_msg: PointCloud2, xy: np.ndarray, conf: float) -> None:
        pole = PointStamped()
        pole.header = cloud_msg.header
        pole.point.x = float(xy[0])
        pole.point.y = float(xy[1])
        pole.point.z = 0.0
        self.pub_pt.publish(pole)

        confidence = Float32()
        confidence.data = float(np.clip(conf, 0.0, 1.0))
        self.pub_conf.publish(confidence)

    def _publish_none(self, cloud_msg: PointCloud2) -> None:
        # If detection is temporarily lost, keep last track but decay confidence.
        self._missed_frames += 1
        if self._has_track:
            self._track_conf *= 0.85
            self._maybe_log_detection(self._track_xy, self._track_conf * 0.2, None, cloud_msg.header.frame_id)
            self._publish_estimate(cloud_msg, self._track_xy, self._track_conf * 0.2)
        else:
            self._maybe_log_detection(np.zeros(2, dtype=np.float32), 0.0, None, cloud_msg.header.frame_id)
            confidence = Float32()
            confidence.data = 0.0
            self.pub_conf.publish(confidence)

    def _maybe_log_detection(
        self,
        xy: np.ndarray,
        conf: float,
        diameter: Optional[float],
        frame_id: str,
    ) -> None:
        # Throttled runtime logging to avoid flooding the terminal.
        min_conf = float(self.get_parameter("detection_log_min_conf").value)
        interval_s = float(self.get_parameter("detection_log_interval_s").value)
        miss_limit = int(self.get_parameter("lost_log_miss_frames").value)
        now_ns = self.get_clock().now().nanoseconds

        # Below threshold we treat it as "not reliably detected".
        if conf < min_conf:
            if self._was_detected and self._missed_frames >= max(1, miss_limit):
                self.get_logger().warn("Pole detection lost.")
                self._was_detected = False
            return

        # Log on first valid detection and then periodically.
        should_log = (
            not self._was_detected
            or (now_ns - self._last_detection_log_ns) >= int(interval_s * 1e9)
        )
        if should_log:
            dist = float(np.linalg.norm(xy))
            if diameter is None:
                self.get_logger().info(
                    f"Pole detected: frame={frame_id}, x={xy[0]:.2f} m, y={xy[1]:.2f} m, "
                    f"range={dist:.2f} m, conf={conf:.2f}"
                )
            else:
                self.get_logger().info(
                    f"Pole detected: frame={frame_id}, x={xy[0]:.2f} m, y={xy[1]:.2f} m, "
                    f"range={dist:.2f} m, diam={diameter:.2f} m, conf={conf:.2f}"
                )
            self._last_detection_log_ns = now_ns
        self._was_detected = True


def main() -> None:
    rclpy.init()
    node = PoleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
