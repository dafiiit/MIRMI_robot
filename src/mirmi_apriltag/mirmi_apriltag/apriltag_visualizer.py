#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class AprilTagVisualizer(Node):
    def __init__(self):
        super().__init__('apriltag_visualizer')
        self.bridge = CvBridge()
        self.latest_detections = None
        
        # QoS muss zum Camera Node passen (jetzt Reliable oder Compatible)
        # Wir nutzen "10" (Standard Reliable), das passt zum neuen Camera-Setup.
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10 
        )
        
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        
        # Output: Best Effort für WLAN, damit Foxglove flüssig bleibt
        qos_out = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.annotated_pub = self.create_publisher(
            CompressedImage,
            '/camera/tag_detections_image/compressed',
            qos_out
        )

    def detection_callback(self, msg):
        self.latest_detections = msg

    def image_callback(self, msg):
        try:
            # 1. Bild immer konvertieren
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. Prüfen ob Detections da sind UND aktuell sind (optional)
            # Wir zeichnen einfach was wir haben.
            found_tags = False
            if self.latest_detections is not None:
                # Einfacher Check: Wenn die Detection viel älter ist als das Bild,
                # könnte man sie ignorieren. Hier lassen wir es für Debugging einfach zu.
                
                for detection in self.latest_detections.detections:
                    found_tags = True
                    # Box zeichnen
                    pts = np.array([[int(c.x), int(c.y)] for c in detection.corners], dtype=np.int32)
                    cv2.polylines(cv_image, [pts], True, (0, 255, 0), 3)
                    
                    # ID schreiben
                    center = (int(detection.centre.x), int(detection.centre.y))
                    cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
                    cv2.putText(cv_image, f"ID: {detection.id}", (center[0]+10, center[1]), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # 3. Status-Text zeichnen (damit du siehst, dass das Bild live ist)
            if not found_tags:
                cv2.putText(cv_image, "Scanning... No Tags", (30, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            else:
                cv2.putText(cv_image, f"Tags detected: {len(self.latest_detections.detections)}", (30, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            # 4. IMMER Komprimieren und Senden
            out_msg = CompressedImage()
            out_msg.header = msg.header
            out_msg.format = "jpeg"
            success, encoded_image = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            
            if success:
                out_msg.data = encoded_image.tobytes()
                self.annotated_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Visualizer Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()