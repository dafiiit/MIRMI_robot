
import rclpy
from rclpy.node import Node
import message_filters
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
import numpy as np
import cv2
import csv
import os
import time

class AccuracyInvestigator(Node):
    def __init__(self):
        super().__init__('accuracy_evaluator')

        # Create subscribers
        self.vicon_robot_sub = message_filters.Subscriber(self, PoseStamped, '/vicon/Robot_1/Robot_1/pose')
        self.vicon_tag_sub = message_filters.Subscriber(self, PoseStamped, '/vicon/Tag/Tag/pose')
        self.camera_sub = message_filters.Subscriber(self, AprilTagDetectionArray, '/camera/detections')
        
        # Camera Info subscriber (latched or separate)
        self.camera_info = None
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        # Synchronize topics
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.vicon_robot_sub, self.vicon_tag_sub, self.camera_sub],
            queue_size=50,
            slop=0.5
        )
        self.ts.registerCallback(self.data_callback)

        self.start_time = None
        self.data_buffer = []
        
        # Output setup
        self.output_dir = os.path.expanduser('~/accuracy_data')
        os.makedirs(self.output_dir, exist_ok=True)
        
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        self.csv_file_path = os.path.join(self.output_dir, f'accuracy_{timestamp}.csv')
        
        # Tag Config
        self.tag_size = 0.150  # meters
        # Define object points for the tag (centered at 0,0,0)
        # Order: bottom-left, bottom-right, top-right, top-left (check if this matches detention order)
        # Standard Apriltag: 0=BL, 1=BR, 2=TR, 3=TL? Or CCW starting from BL.
        # Let's assume standard CCW from BL: (-x, +y), (+x, +y), (+x, -y), (-x, -y)
        s = self.tag_size / 2.0
        self.obj_points = np.array([
            [-s,  s, 0],
            [ s,  s, 0],
            [ s, -s, 0],
            [-s, -s, 0]
        ], dtype=np.float32)

        self.get_logger().info('Accuracy Evaluator Node Started. Collecting data...')

    def camera_info_callback(self, msg):
        self.camera_info = msg
        # Unsubscribe after receiving once if strictly static, but better keep it in case it changes or checks validity
        # For now, just store it.

    def pose_to_matrix(self, pose):
        """Convert geometry_msgs/Pose to 4x4 transformation matrix"""
        q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        if np.linalg.norm(q) > 1e-6:
             q /= np.linalg.norm(q)
        x, y, z, w = q
        
        R = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ])
        
        matrix = np.eye(4)
        matrix[0:3, 0:3] = R
        matrix[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return matrix

    def data_callback(self, robot_msg, tag_msg, detection_msg):
        if not detection_msg.detections:
            return
        if self.camera_info is None:
            self.get_logger().warn('Waiting for camera info...', throttle_duration_sec=2.0)
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = current_time

        elapsed_time = current_time - self.start_time

        # Vicon Computation
        T_world_robot = self.pose_to_matrix(robot_msg.pose)
        T_world_tag = self.pose_to_matrix(tag_msg.pose)
        
        # T_robot_tag = T_world_robot^-1 * T_world_tag
        T_world_robot_inv = np.linalg.inv(T_world_robot)
        T_robot_tag_vicon = np.dot(T_world_robot_inv, T_world_tag)
        
        vicon_pos = T_robot_tag_vicon[0:3, 3]

        # Camera Measurement (SolvePnP)
        det = detection_msg.detections[0]
        
        image_points = np.array([
            [det.corners[0].x, det.corners[0].y],
            [det.corners[1].x, det.corners[1].y],
            [det.corners[2].x, det.corners[2].y],
            [det.corners[3].x, det.corners[3].y]
        ], dtype=np.float32)

        K = np.array(self.camera_info.k).reshape(3, 3)
        D = np.array(self.camera_info.d)

        success, rvec, tvec = cv2.solvePnP(self.obj_points, image_points, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        
        if success:
            cam_pos = tvec.flatten()
            
            # NOTE: Tag frame in Apriltag is usually: x-right, y-down, z-forward.
            # Vicon robot detection might have different frame conventions.
            # But the user asked to compare 3D positions and assumes they align at t=0.
            # Usage of SOLVEPNP_IPPE_SQUARE is good for flat targets.

            self.data_buffer.append([
                elapsed_time,
                vicon_pos[0], vicon_pos[1], vicon_pos[2],
                cam_pos[0], cam_pos[1], cam_pos[2]
            ])
            
            if len(self.data_buffer) % 100 == 0:
                self.get_logger().info(f'Collected {len(self.data_buffer)} samples')

    def save_csv(self):
        self.get_logger().info(f'Saving {len(self.data_buffer)} samples to {self.csv_file_path}')
        with open(self.csv_file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'vicon_x', 'vicon_y', 'vicon_z', 'cam_x', 'cam_y', 'cam_z'])
            writer.writerows(self.data_buffer)
        self.get_logger().info('CSV saved.')

def main(args=None):
    rclpy.init(args=args)
    node = AccuracyInvestigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
