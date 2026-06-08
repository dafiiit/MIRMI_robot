#!/usr/bin/env python3
import math
from collections import defaultdict, deque
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


Cell = Tuple[int, int]

def quaternion_from_euler(ai: float, aj: float, ak: float) -> List[float]:
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    return [
        cj*sc - sj*cs,
        cj*ss + sj*cc,
        cj*cs - sj*sc,
        cj*cc + sj*ss
    ]

class StationDetector(Node):
    """
    Detects a large docking station structure ('tiny house') in PointCloud2 datastream
    using 2D point clustering and Oriented Bounding Box (OBB) calculations via PCA.
    """

    def __init__(self) -> None:
        super().__init__("station_detector")

        # Configurable Parameters
        self.declare_parameter("cloud_topic", "/livox/lidar")  # direct topic — no remap needed
        self.declare_parameter("z_min", 0.10)
        self.declare_parameter("z_max", 2.20)
        
        # Grid settings - resolution is slightly coarser for larger objects
        self.declare_parameter("grid_res", 0.10)
        self.declare_parameter("min_points_per_cell", 2)
        self.declare_parameter("min_cells_per_cluster", 20)
        self.declare_parameter("min_points_per_cluster", 50)
        
        # Station dimensions
        self.declare_parameter("expected_length", 2.0)  # depth
        self.declare_parameter("expected_width", 1.5)   # wide
        self.declare_parameter("length_tolerance", 0.6)
        self.declare_parameter("width_tolerance", 0.5)
        
        # Ranges
        self.declare_parameter("expected_range", 3.0)
        self.declare_parameter("range_tolerance", 2.0)

        # Marker specific
        self.declare_parameter("marker_mesh_resource", "package://station_detection_LIDAR/meshes/station.stl")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.sub = self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos)

        self.pub_conf = self.create_publisher(Float32, "/station_confidence", qos)
        self.pub_marker = self.create_publisher(Marker, "/station_marker", qos)

        # Health status tracking
        self.last_cloud_time = 0.0
        self.last_recognition_time = 0.0
        self.pub_status = self.create_publisher(String, "/lidar/status", 10)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f"StationDetector listening on {cloud_topic}")

    def on_cloud(self, msg: PointCloud2) -> None:
        self.last_cloud_time = self.get_clock().now().nanoseconds / 1e9
        z_min = float(self.get_parameter("z_min").value)
        z_max = float(self.get_parameter("z_max").value)
        grid_res = float(self.get_parameter("grid_res").value)
        min_ppc = int(self.get_parameter("min_points_per_cell").value)
        min_cells_cluster = int(self.get_parameter("min_cells_per_cluster").value)
        min_pts_cluster = int(self.get_parameter("min_points_per_cluster").value)
        
        exp_length = float(self.get_parameter("expected_length").value)
        exp_width = float(self.get_parameter("expected_width").value)
        len_tol = float(self.get_parameter("length_tolerance").value)
        width_tol = float(self.get_parameter("width_tolerance").value)
        
        exp_r = float(self.get_parameter("expected_range").value)
        r_tol = float(self.get_parameter("range_tolerance").value)

        # 1. Downsampling & Filtering using NumPy
        struct_pts = point_cloud2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        if len(struct_pts) == 0:
            return

        # Stack into unstructured 2D float32 array of shape (N, 3)
        points = np.stack([struct_pts['x'], struct_pts['y'], struct_pts['z']], axis=-1)

        # Fast NumPy vector filtering for height limits
        z_mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        filtered_points = points[z_mask]

        if len(filtered_points) < min_pts_cluster:
            return

        # Extract X and Y coordinates
        xy = filtered_points[:, :2].astype(np.float32)

        # Safe decimation/downsampling if the point cloud is excessively dense
        # Capping the points processed by the cell-assignment Python loop at 2000 points
        max_loop_points = 2000
        if len(xy) > max_loop_points:
            decimation = len(xy) // max_loop_points
            xy = xy[::decimation]

        inv = 1.0 / max(grid_res, 1e-6)
        ix = np.floor(xy[:, 0] * inv).astype(np.int32)
        iy = np.floor(xy[:, 1] * inv).astype(np.int32)

        # 2. Assign Points to 2D Grid Cells
        cell_counts: Dict[Cell, int] = defaultdict(int)
        cell_points: Dict[Cell, List[Tuple[float, float]]] = defaultdict(list)

        for x, y, cx, cy in zip(xy[:, 0], xy[:, 1], ix, iy):
            cell = (int(cx), int(cy))
            cell_counts[cell] += 1
            cell_points[cell].append((float(x), float(y)))

        occ_cells = {cell for cell, cnt in cell_counts.items() if cnt >= min_ppc}
        if len(occ_cells) < min_cells_cluster:
            return

        # 3. Clustering Connected Grid Cells
        neighbors = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),          (0,  1),
            (1, -1),  (1,  0), (1,  1)
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

        detected_stations = []

        # 4. Shape Fitting using PCA for Oriented Bounding Box
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
            
            # PCA logic to find axes
            mean = points.mean(axis=0)
            centered = points - mean
            
            # Use safe covariance to prevent errors on collinear points
            if len(centered) < 2:
                continue
            cov = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)
            
            # Sort highest-to-lowest eigenvalues
            sort_indices = np.argsort(eigenvalues)[::-1]
            eigenvalues = eigenvalues[sort_indices]
            eigenvectors = eigenvectors[:, sort_indices]
            
            p1 = eigenvectors[:, 0]
            p2 = eigenvectors[:, 1]
            
            # Project points onto axes
            proj1 = centered.dot(p1)
            proj2 = centered.dot(p2)
            
            meas_dim1 = float(proj1.max() - proj1.min())
            meas_dim2 = float(proj2.max() - proj2.min())
            
            meas_max = max(meas_dim1, meas_dim2)
            meas_min = min(meas_dim1, meas_dim2)
            
            target_max = max(exp_length, exp_width)
            target_min = min(exp_length, exp_width)
            
            # Validation Step
            if abs(meas_max - target_max) > len_tol:
                continue
            if abs(meas_min - target_min) > width_tol:
                continue
            
            # Calculate OBB Center
            center_proj_1 = (proj1.max() + proj1.min()) / 2.0
            center_proj_2 = (proj2.max() + proj2.min()) / 2.0
            box_center = mean + center_proj_1 * p1 + center_proj_2 * p2
            
            # Yaw representation
            # Determine which axis represents the longest dimension
            if meas_dim1 >= meas_dim2:
                yaw = math.atan2(p1[1], p1[0])
            else:
                yaw = math.atan2(p2[1], p2[0])
            
            detected_stations.append((
                box_center, meas_max, meas_min, yaw, total_pts
            ))

        if not detected_stations:
            return

        # Pick the most prominent / accurate cluster if multiple
        best_station = None
        best_conf = -1.0
        
        for box_center, meas_max, meas_min, yaw, total_pts in detected_stations:
            # Range check logic
            r = math.hypot(box_center[0], box_center[1])
            if abs(r - exp_r) > r_tol:
                continue
            
            # Calculate Confidence Score (0.0 to 1.0)
            conf = 0.0
            # Higher pts = better up to a limit
            conf += min(1.0, total_pts / (min_pts_cluster * 3.0)) * 0.4
            
            # Dimensional acc factor
            target_max = max(exp_length, exp_width)
            target_min = min(exp_length, exp_width)
            acc_penalty = abs(meas_max - target_max) / max(len_tol, 1e-3) + abs(meas_min - target_min) / max(width_tol, 1e-3)
            acc_factor = max(0.0, 1.0 - (acc_penalty / 2.0))
            
            conf += acc_factor * 0.4
            
            # Range preference
            conf += max(0.0, 1.0 - abs(r - exp_r) / max(r_tol, 1e-3)) * 0.2
            
            conf = float(np.clip(conf, 0.0, 1.0))
            
            if conf > best_conf:
                best_conf = conf
                best_station = (box_center, yaw)

        if best_station is not None:
            self.last_recognition_time = self.get_clock().now().nanoseconds / 1e9
            box_center, yaw = best_station
            
            # Publish Confidence
            c = Float32()
            c.data = best_conf
            self.pub_conf.publish(c)
            
            # Publish Marker
            q = quaternion_from_euler(0.0, 0.0, yaw)
            
            marker = Marker()
            marker.header = msg.header
            marker.ns = "station"
            marker.id = 0
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            
            mesh_res = str(self.get_parameter("marker_mesh_resource").value)
            marker.mesh_resource = mesh_res
            marker.mesh_use_embedded_materials = False
            
            marker.pose.position.x = float(box_center[0])
            marker.pose.position.y = float(box_center[1])
            # Assuming bottom of station is z=0, so use global zero height ideally, 
            # might need adapting based on mesh origin convention 
            marker.pose.position.z = 0.0 
            
            marker.pose.orientation.x = float(q[0])
            marker.pose.orientation.y = float(q[1])
            marker.pose.orientation.z = float(q[2])
            marker.pose.orientation.w = float(q[3])
            
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            
            # Use somewhat transparent color
            marker.color.r = 0.2
            marker.color.g = 0.6
            marker.color.b = 0.8
            marker.color.a = 0.8
            
            marker.lifetime = rclpy.duration.Duration(seconds=0, nanoseconds=500000000).to_msg()
            
            self.pub_marker.publish(marker)

    def publish_status(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        data_received = (now - self.last_cloud_time) < 2.0
        
        cloud_topic = str(self.get_parameter("cloud_topic").value)
        topic_posted = self.count_publishers(cloud_topic) > 0
        
        recognized = (now - self.last_recognition_time) < 2.0
        
        status_msg = String()
        status_msg.data = f"data_received:{str(data_received).lower()},topic_posted:{str(topic_posted).lower()},recognized:{str(recognized).lower()}"
        self.pub_status.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StationDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
