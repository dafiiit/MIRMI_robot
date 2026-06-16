#!/usr/bin/env python3
"""
yolo_station_node — detect the docking station with the trained YOLOv8-pose model
and publish where it is relative to the camera, so the rover can FACE it.

The model (src/yolo/best.pt) returns, per detection, a bounding box plus 8
keypoints = the station's 3D bounding-box corners (pixels).  From the detection we
publish the horizontal bearing of the station centre relative to the camera optical
axis — this is the facing error: rotate the rover until it is zero.

Published topics
----------------
  /station/detected   std_msgs/Bool       True while the station is seen
  /station/bearing    std_msgs/Float32    bearing of station centre [rad],
                                           + = station is to the RIGHT of centre
                                           (rover should turn right / CW to face it)
  /station/info       std_msgs/String     JSON: conf, u_center, width_px, n_corners,
                                           asym (left/right face width ratio → which
                                           way the hut is angled)

Subscribed
----------
  <image topic>        sensor_msgs/Image       default /camera/camera/color/image_raw
  <camera_info topic>  sensor_msgs/CameraInfo   default /camera/camera/color/camera_info

Inference runs in a background thread at whatever rate the CPU allows (it does NOT
block the ROS executor), always on the most recent frame.
"""

import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32, String


def image_to_bgr(msg: Image):
    """Convert sensor_msgs/Image (rgb8/bgr8/mono8) to an HxWx3 BGR uint8 array."""
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(h, w, 3)
        if enc == 'rgb8':
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)
    if enc == 'mono8':
        g = buf.reshape(h, w)
        return np.ascontiguousarray(np.stack([g, g, g], -1))
    # fallback: assume 3-channel
    return np.ascontiguousarray(buf.reshape(h, w, -1)[:, :, :3])


class YoloStationNode(Node):
    def __init__(self):
        super().__init__('yolo_station_node')
        self.declare_parameter('model', '/home/holybro/ws_sensor_combined/src/yolo/best.pt')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('imgsz', 480)         # 480≈1.4fps vs 640≈1.1fps on CPU
        self.declare_parameter('rotate_180', True)   # camera is mounted upside-down

        model_path = self.get_parameter('model').value
        self._conf = float(self.get_parameter('conf').value)
        self._imgsz = int(self.get_parameter('imgsz').value)
        self._rotate180 = bool(self.get_parameter('rotate_180').value)

        self.get_logger().info(f'Loading YOLO model: {model_path} ...')
        from ultralytics import YOLO          # imported here so the node fails loudly if missing
        self._model = YOLO(model_path)
        self.get_logger().info(f'Model loaded (task={self._model.task}).')

        self._lock = threading.Lock()
        self._latest_img = None               # (bgr, stamp)
        self._fx = None
        self._cx = None

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self._img_cb, be)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self._info_cb, be)

        self._pub_det = self.create_publisher(Bool, '/station/detected', 10)
        self._pub_brg = self.create_publisher(Float32, '/station/bearing', 10)
        self._pub_info = self.create_publisher(String, '/station/info', 10)

        self._run = True
        threading.Thread(target=self._infer_loop, daemon=True).start()
        self.get_logger().info('yolo_station_node ready — publishing /station/{detected,bearing,info}')

    def _img_cb(self, msg: Image):
        try:
            bgr = image_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'image convert failed: {e}')
            return
        with self._lock:
            self._latest_img = (bgr, time.time())

    def _info_cb(self, msg: CameraInfo):
        with self._lock:
            self._fx = msg.k[0]
            self._cx = msg.k[2]

    def _infer_loop(self):
        last_stamp = 0.0
        while self._run and rclpy.ok():
            with self._lock:
                entry = self._latest_img
                fx, cx = self._fx, self._cx
            if entry is None or entry[1] == last_stamp:
                time.sleep(0.02)
                continue
            bgr, last_stamp = entry
            if self._rotate180:                 # camera upside-down → make it upright
                bgr = bgr[::-1, ::-1].copy()
            if fx is None:                      # no camera_info yet → assume 60° HFOV
                fx = bgr.shape[1] / (2 * math.tan(math.radians(30)))
                cx = bgr.shape[1] / 2.0
            elif self._rotate180:               # 180° rotation maps cx → W - cx
                cx = bgr.shape[1] - cx
            try:
                res = self._model.predict(bgr, conf=self._conf, imgsz=self._imgsz,
                                          verbose=False)[0]
            except Exception as e:
                self.get_logger().warn(f'inference failed: {e}')
                time.sleep(0.1)
                continue
            self._publish(res, fx, cx)

    def _publish(self, res, fx, cx):
        detected = False
        bearing = 0.0
        info = {'conf': 0.0}
        if res.boxes is not None and len(res.boxes) > 0:
            # pick the most confident detection
            confs = res.boxes.conf.cpu().numpy()
            i = int(np.argmax(confs))
            box = res.boxes.xyxy.cpu().numpy()[i]      # x1,y1,x2,y2
            u_center = 0.5 * (box[0] + box[2])
            width_px = float(box[2] - box[0])
            asym = float('nan')
            n_corners = 0
            if res.keypoints is not None and res.keypoints.xy is not None \
                    and len(res.keypoints.xy) > i:
                kp = res.keypoints.xy.cpu().numpy()[i]   # (8,2)
                valid = kp[(kp[:, 0] > 0) | (kp[:, 1] > 0)]
                n_corners = int(len(valid))
                if n_corners >= 4:
                    u_center = float(np.mean(valid[:, 0]))   # corner centroid is steadier
                    # asym: spread of corners left vs right of centroid → hut angle hint
                    left = np.sum(valid[:, 0] < u_center)
                    right = np.sum(valid[:, 0] > u_center)
                    asym = float(left - right)
            bearing = math.atan2(u_center - cx, fx)
            detected = True
            info = {'conf': float(confs[i]), 'u_center': round(float(u_center), 1),
                    'width_px': round(width_px, 1), 'n_corners': n_corners,
                    'asym': asym if asym == asym else None,
                    'bearing_deg': round(math.degrees(bearing), 2)}
        self._pub_det.publish(Bool(data=detected))
        self._pub_brg.publish(Float32(data=float(bearing)))
        self._pub_info.publish(String(data=json.dumps(info)))


def main(args=None):
    rclpy.init(args=args)
    node = YoloStationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._run = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
