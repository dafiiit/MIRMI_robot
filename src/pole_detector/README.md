# Pole Detector

A ROS 2 package for detecting cylindrical objects such as poles or tree trunks in `sensor_msgs/PointCloud2` data.

## Features
- XY-based clustering on an occupancy grid.
- Filters points by height (Z-axis) and radial distance from the origin (R-axis).
- Validates clusters based on physical dimensions (diameter).
- Provides a confidence score for each detection based on density and expected characteristics.

## Published Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/pole_estimate` | `geometry_msgs/PointStamped` | The 2D XY position of the detected pole (Z=0.0). |
| `/pole_confidence` | `std_msgs/Float32` | A value from 0.0 to 1.0 representing detection reliability. |
| `/pole_radius` | `std_msgs/Float32` | The estimated radius of the detected object. |

## Subscribed Topics
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Input LIDAR data (configurable via parameters). |

## Parameters
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `cloud_topic` | `/livox/lidar` | Topic to subscribe to for PointCloud2 messages. |
| `z_min` / `z_max` | `0.10` / `1.40` | Height filters in meters. |
| `r_min` / `r_max` | `0.20` / `3.00` | Radial distance filters from the LIDAR frame origin. |
| `grid_res` | `0.03` | Resolution of the occupancy grid for clustering (m). |
| `min_points_per_cell` | `2` | Points required for a grid cell to be considered "occupied". |
| `min_cells_per_cluster`| `4` | Minimum adjacent cells to form a cluster. |
| `expected_diameter` | `0.10` | The target pole diameter in meters. |
| `max_cluster_diameter`| `0.18` | Maximum possible diameter for any detected object. |

## Running the Node
Ensure you have sourced your ROS 2 workspace, then run:

```bash
ros2 run pole_detector pole_detector
```

## How it Works
1. **Filtering**: The LIDAR points are filtered to stay within the specified Z-range and R-range.
2. **Grid Clustering**: Points are projected onto a 2D grid (`grid_res`). Cells with at least `min_points_per_cell` are clustered using neighborhood connectivity.
3. **Validation**: Clusters are checked for density (`min_points_per_cluster`) and physical dimensions (`min_cluster_diameter` and `max_cluster_diameter`).
4. **Scoring**: A weighted confidence score is calculated based on point count, diameter accuracy, and expected range.
