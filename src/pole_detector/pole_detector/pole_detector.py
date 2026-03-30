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
    Detects multiple poles/tree trunks in PointCloud2 data using XY clustering.
    """

    def __init__(self) -> None:
        super().__init__("pole_detector")

        # Parameters
        self.declare_parameter("cloud_topic", "/livox/lidar")
        self.declare_parameter("z_min", 0.10)
        self.declare_parameter("z_max", 1.40)
        self.declare_parameter("r_min", 0.20)
        self.declare_parameter("r_max", 3.00)
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

        cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.sub = self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, 10)

        # Publishers
        self.pub_pt = self.create_publisher(PointStamped, "/pole_estimate", 10)
        self.pub_conf = self.create_publisher(Float32, "/pole_confidence", 10)
        self.pub_radius = self.create_publisher(Float32, "/pole_radius", 10)

        self.get_logger().info(f"PoleDetector listening on {cloud_topic}")

    def on_cloud(self, msg: PointCloud2) -> None:

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
            return  # no trees

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
            return

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
            return

        detected_trees = []  

        for component in clusters:
            pts_list = []
            total_pts = 0

            for cell in component:
                pts_cell = cell_points.get(cell, [])
                total_pts += len(pts_cell)
                pts_list.extend(pts_cell)

            if total_pts < min_pts_cluster:
                continue

            points = np.asarray(pts_list, dtype=np.float32)
            centroid = points.mean(axis=0)

            # Diameter
            dist = np.sqrt(((points - centroid) ** 2).sum(axis=1))
            diameter = 2.0 * float(dist.max())
            if diameter < min_diam or diameter > max_diam:
                continue

            detected_trees.append((centroid, diameter, total_pts))

        if not detected_trees:
            return

        for centroid, diameter, total_pts in detected_trees:

            # Confidence (same formula as before)
            conf = 0.0
            conf += min(1.0, total_pts / (min_pts_cluster * 2.0)) * 0.55
            conf += max(0.0, 1.0 - abs(diameter - exp_diam) / max(diam_tol, 1e-3)) * 0.25
            conf += max(0.0, 1.0 - abs(float(np.linalg.norm(centroid)) - exp_r) / max(r_tol, 1e-3)) * 0.20
            conf = float(np.clip(conf, 0.0, 1.0))

            # Publish tree position
            pole = PointStamped()
            pole.header = msg.header
            pole.point.x = float(centroid[0])
            pole.point.y = float(centroid[1])
            pole.point.z = 0.0
            self.pub_pt.publish(pole)

            # Publish confidence
            c = Float32()
            c.data = conf
            self.pub_conf.publish(c)

            #publish radii(radius?)
            r=Float32()
            r.data = diameter/2
            self.pub_radius.publish(r)




def main() -> None:
    rclpy.init()
    node = PoleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
