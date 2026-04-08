# station_detection_LIDAR

This package calculates the position and orientation of a large docking station ("tiny house") from `/livox/lidar` point clouds using computationally efficient 2D spatial clustering and publishes its presence, position, and confidence. 

## Features

- **Coarse Resolution 2D Clustering**: Maps the LIDAR points to a 2D occupancy grid to filter out small debris / noise and quickly cluster larger objects.
- **PCA Bounding Box Calculation**: Calculates the main orientation eigenvectors of clusters pointing out the general yaw (orientation).
- **Physical Extent Validation**: Compares calculated box dimensions against expected dimensions of the tiny house.
- **Confidence Output**: Publishes confidence continuously, increasing when dimensions properly match the station and are within the 1m-5m range. 
- **Visualization Marker**: Exposes the tiny house position in RViz via a 3D Mesh Marker natively (STL).

## Configuration

The package relies on `.stl` for the 3D representation and supports setting tolerance and target sizes natively via launch parameters. 

**Default Configuration:**
*   Expected Range: 3.0m (Tolerance: 2.0m, meaning detections within **1m - 5m** are considered valid).
*   Station Dimensions: 1.5m wide $\times$ 2.0m deep $\times$ 2.0m high (represented implicitly via extents in the implementation).

Place your `.stl` mesh file into the `meshes` directory matching the filename specified in the launch file. Currently expected at `meshes/station.stl`.

**Launch File:**
`ros2 launch station_detection_LIDAR station_detector.launch.py`

## Node Outputs

*   `/station_marker` ([visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html)): Published matching the geometry and yaw, using the `.stl` mesh.
*   `/station_confidence` ([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html)): Returns a value from `0.0` to `1.0`. `1.0` indicates an exact dimensional match at the expected 3.0m range, with maximum cluster points.

## How it works

1.  **Filtering**: Downsamples raw point clouds. Drops anything outside of the z-range (`[0.10m, 2.20m]`).
2.  **Grid Map**: Applies points to a defined 2D grid structure using a `0.10m` cellular resolution. 
3.  **Component Labeling**: Locates blobs in the grid and groups neighboring cells.
4.  **Shape Extraction (PCA)**: Determines the largest structure size via covariance and extracting highest variants via PCA eigenvectors.
5.  **Output Analysis**: Validates bounding boxes of detected features against `[expected_length ± tolerance]` and `[expected_width ± tolerance]` and scores it!
