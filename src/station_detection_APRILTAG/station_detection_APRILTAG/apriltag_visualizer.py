#!/usr/bin/env python3
"""GPU-accelerated AprilTag visualizer for Jetson Orin.

Original CPU bottlenecks (was burning ~120 % of one CPU core):
  1. cv2.imencode  – full-frame CPU JPEG encoding on EVERY camera frame (~70 % of cost)
  2. No FPS cap    – encoding runs at the full camera rate (e.g. 30 Hz)
  3. imgmsg_to_cv2 / tobytes – two full-frame memory copies per frame
  4. Unnecessary status-text overlay drawn every frame

Optimizations applied:
  1. FPS cap (default 10 Hz)  – skips imgmsg_to_cv2 *and* encoding for dropped frames
  2. GPU JPEG via nvjpegenc   – JPEG encoding offloaded to Jetson's hardware encoder
                                (GStreamer: BGR → videoconvert → nvvidconv → nvjpegenc)
  3. cv2.imencode fallback    – used automatically if the GStreamer pipeline fails
  4. Minimal drawing          – bounding box + tag ID only, no status overlay
"""
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

# ---------------------------------------------------------------------------
# GPU JPEG encoder – GStreamer nvjpegenc (Jetson hardware)
# ---------------------------------------------------------------------------
class _GstNvJpegEncoder:
    """Encodes single frames to JPEG using the Jetson hardware JPEG encoder
    (nvjpegenc GStreamer element).  The pipeline is built lazily on the first
    frame so that image dimensions are known; it is rebuilt only when they
    change (rare).
    """

    def __init__(self, quality: int = 50) -> None:
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst  # type: ignore[import]
        Gst.init(None)
        self._Gst = Gst
        self._quality = quality
        self._pipeline = None
        self._appsrc = None
        self._appsink = None
        self._w = self._h = 0

    def _rebuild(self, w: int, h: int) -> None:
        Gst = self._Gst
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
        # BGR → videoconvert (CPU, trivial) → nvvidconv (GPU) → nvjpegenc (GPU)
        desc = (
            f'appsrc name=src format=time emit-signals=false '
            f'caps="video/x-raw,format=BGR,width={w},height={h},framerate=0/1" ! '
            f'videoconvert ! nvvidconv ! '
            f'nvjpegenc quality={self._quality} ! '
            f'appsink name=sink sync=false emit-signals=false max-buffers=1 drop=true'
        )
        self._pipeline = Gst.parse_launch(desc)
        self._appsrc  = self._pipeline.get_by_name('src')
        self._appsink = self._pipeline.get_by_name('sink')
        ret = self._pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('nvjpegenc GStreamer pipeline failed to start')
        self._w, self._h = w, h

    def encode(self, bgr: np.ndarray) -> Optional[bytes]:
        Gst = self._Gst
        h, w = bgr.shape[:2]
        if w != self._w or h != self._h:
            self._rebuild(w, h)
        # tobytes() is a single memcpy; necessary to hand ownership to GStreamer
        gst_buf = Gst.Buffer.new_wrapped(bgr.tobytes())
        if self._appsrc.emit('push-buffer', gst_buf) != Gst.FlowReturn.OK:
            return None
        sample = self._appsink.emit('pull-sample')
        if sample is None:
            return None
        cbuf = sample.get_buffer()
        ok, mi = cbuf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        data = bytes(mi.data)
        cbuf.unmap(mi)
        return data

    def stop(self) -> None:
        if self._pipeline is not None:
            self._pipeline.set_state(self._Gst.State.NULL)


def _try_build_gst_encoder(quality: int) -> Optional[_GstNvJpegEncoder]:
    """Return a GPU encoder if nvjpegenc is available, else None."""
    try:
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst  # type: ignore[import]
        Gst.init(None)
        if Gst.ElementFactory.find('nvjpegenc') is None:
            return None
        return _GstNvJpegEncoder(quality)
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class AprilTagVisualizer(Node):
    # Maximum visualization rate; frames arriving faster are silently dropped
    # before any decoding or encoding work is done.
    TARGET_FPS: float = 10.0
    JPEG_QUALITY: int = 50

    def __init__(self) -> None:
        super().__init__('apriltag_visualizer')
        self.bridge = CvBridge()
        self.latest_detections = None
        self._last_pub_time: float = 0.0
        self._min_interval: float = 1.0 / self.TARGET_FPS

        # Prefer GPU encoder; fall back to CPU cv2.imencode automatically
        self._gst_encoder = _try_build_gst_encoder(self.JPEG_QUALITY)
        if self._gst_encoder is not None:
            self.get_logger().info('JPEG encoder: nvjpegenc (GPU)')
        else:
            self.get_logger().info('JPEG encoder: cv2.imencode (CPU) – nvjpegenc not available')

        qos_in = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_in)
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray, '/tag_detections', self.detection_callback, qos_in)

        qos_out = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        self.annotated_pub = self.create_publisher(
            CompressedImage, '/camera/tag_detections_image/compressed', qos_out)

    def detection_callback(self, msg: AprilTagDetectionArray) -> None:
        self.latest_detections = msg

    def image_callback(self, msg: Image) -> None:
        # 1. Skip when no one is watching – saves all downstream work
        if self.annotated_pub.get_subscription_count() == 0:
            return

        # 2. FPS cap – drop frames that arrive above the target rate.
        #    This check happens BEFORE imgmsg_to_cv2, so dropped frames cost
        #    almost nothing (just a monotonic clock read).
        now = time.monotonic()
        if now - self._last_pub_time < self._min_interval:
            return
        self._last_pub_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 3. Draw bounding boxes + tag IDs (fast CPU operation)
            if self.latest_detections:
                for det in self.latest_detections.detections:
                    pts = np.array(
                        [[int(c.x), int(c.y)] for c in det.corners],
                        dtype=np.int32)
                    cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)
                    cx, cy = int(det.center.x), int(det.center.y)
                    cv2.putText(cv_image, f'ID:{det.id}',
                                (cx + 8, cy - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 255), 2, cv2.LINE_AA)

            # 4. JPEG encode – GPU path first, CPU fallback
            data: Optional[bytes] = None
            if self._gst_encoder is not None:
                try:
                    data = self._gst_encoder.encode(cv_image)
                except Exception as exc:
                    self.get_logger().warn(
                        f'GPU encoder failed ({exc}), falling back to CPU')
                    self._gst_encoder = None  # stop retrying
            if data is None:
                ok, buf = cv2.imencode(
                    '.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, self.JPEG_QUALITY])
                data = buf.tobytes() if ok else None

            if data is None:
                return

            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = data
            self.annotated_pub.publish(out)

        except Exception as e:
            self.get_logger().error(f'Visualizer error: {e}')

    def destroy_node(self) -> None:
        if self._gst_encoder is not None:
            self._gst_encoder.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()