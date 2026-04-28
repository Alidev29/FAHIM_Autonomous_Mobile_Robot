"""
FAHIM — YOLOv11 vision node.

Subscribes:
  /camera/color/image_raw                    (sensor_msgs/Image, rgb8 or bgr8)
  /camera/aligned_depth_to_color/image_raw   (sensor_msgs/Image, 16UC1, mm)
  /camera/color/camera_info                  (sensor_msgs/CameraInfo)

Publishes:
  /camera/yolo/image_annotated   (sensor_msgs/Image)           — boxes + labels on RGB
  /camera/yolo/detections        (vision_msgs/Detection2DArray) — 2-D bounding boxes
  /camera/yolo/markers           (visualization_msgs/MarkerArray) — 3-D RViz spheres
  /camera/yolo/detections_3d     (vision_msgs/Detection3DArray)  — 3-D poses

3-D projection:
  For each detection, the median depth inside the bounding box (ignoring
  zero/invalid pixels) is combined with the colour camera's pinhole model:
    X = (u_c - cx) * Z / fx
    Y = (v_c - cy) * Z / fy
  The resulting point is expressed in camera_color_optical_frame, which
  robot_state_publisher keeps connected to the rest of the TF tree.

Image conversion:
  cv_bridge is NOT used — all sensor_msgs/Image ↔ numpy conversions are done
  with pure numpy (np.frombuffer / ndarray.tobytes). This eliminates the
  cv_bridge C-extension ABI dependency on a specific numpy version.

Requirements (install once on Jetson):
  pip3 install ultralytics --break-system-packages
  # model is auto-downloaded on first run to ~/.config/Ultralytics/
"""

import math
import threading

import cv2
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from vision_msgs.msg import (
    Detection2D, Detection2DArray,
    Detection3D, Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

# Suppress Ultralytics verbose startup banner
import os
os.environ['YOLO_VERBOSE'] = 'False'

# ── torchvision NMS patch ────────────────────────────────────────────────
# The PyPI torchvision binary is compiled against standard PyTorch 2.5.0.
# Jetson torch is 2.5.0a0+nv24.8 — different C++ ABI — so torchvision's
# C++ extension fails to load.  We replace nms / batched_nms with a
# numpy-based CPU implementation:
#   • One GPU→CPU copy of the whole box array  (not one sync per iteration)
#   • Pure numpy inner loop  (C speed, no GIL per-element overhead)
#   • Result copied back to the original device as a flat int64 tensor
# This is exact greedy NMS, identical in output to torchvision.ops.nms.
def _numpy_nms(boxes: torch.Tensor,
               scores: torch.Tensor,
               iou_threshold: float) -> torch.Tensor:
    """Fast NMS via numpy — drop-in for torchvision.ops.nms."""
    if boxes.numel() == 0:
        return torch.empty((0,), dtype=torch.int64, device=boxes.device)

    dev = boxes.device
    # Single GPU→CPU transfer
    b = boxes.detach().cpu().numpy().astype(np.float32)
    s = scores.detach().cpu().numpy().astype(np.float32)

    x1, y1, x2, y2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = s.argsort()[::-1]          # descending score

    keep = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        order = order[1:]
        xx1 = np.maximum(x1[i], x1[order])
        yy1 = np.maximum(y1[i], y1[order])
        xx2 = np.minimum(x2[i], x2[order])
        yy2 = np.minimum(y2[i], y2[order])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        iou   = inter / np.maximum(areas[i] + areas[order] - inter, 1e-6)
        order = order[iou <= iou_threshold]

    return torch.tensor(keep, dtype=torch.int64, device=dev)


def _numpy_batched_nms(boxes: torch.Tensor,
                       scores: torch.Tensor,
                       idxs: torch.Tensor,
                       iou_threshold: float) -> torch.Tensor:
    """Batched NMS — drop-in for torchvision.ops.batched_nms."""
    if boxes.numel() == 0:
        return torch.empty((0,), dtype=torch.int64, device=boxes.device)
    max_coord = float(boxes.max())
    offsets   = idxs.to(boxes.dtype) * (max_coord + 1.0)
    return _numpy_nms(boxes + offsets[:, None], scores, iou_threshold)


import torchvision.ops as _tvops
_tvops.nms         = _numpy_nms
_tvops.batched_nms = _numpy_batched_nms
try:                                   # also patch the sub-module path
    import torchvision.ops.boxes as _tvboxes
    _tvboxes.nms         = _numpy_nms
    _tvboxes.batched_nms = _numpy_batched_nms
except Exception:
    pass
# also patch ultralytics' own nms import if it cached the reference
try:
    import ultralytics.utils.ops as _ul_ops
    _ul_ops.torchvision = type(_ul_ops.torchvision)  # type: ignore
except Exception:
    pass
# ── end torchvision NMS patch ────────────────────────────────────────────

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# ═══════════════════════════════════════════════════════════════
#  Pure-numpy ROS ↔ OpenCV image helpers  (no cv_bridge needed)
# ═══════════════════════════════════════════════════════════════

def _ros_to_bgr(msg: Image) -> np.ndarray:
    """
    Convert a sensor_msgs/Image to a BGR uint8 numpy array.
    Handles rgb8, bgr8, and rgba8 encodings (RealSense publishes rgb8).
    """
    enc = msg.encoding.lower()
    if enc in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        channels = 4 if enc in ('rgba8', 'bgra8') else 3
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, channels)
        if enc == 'rgb8':
            return arr[:, :, ::-1].copy()   # RGB → BGR
        if enc == 'rgba8':
            return arr[:, :, 2::-1].copy()  # RGBA → BGR
        if enc == 'bgra8':
            return arr[:, :, :3].copy()     # BGRA → BGR
        return arr.copy()                   # bgr8 — already correct
    raise ValueError(f'_ros_to_bgr: unsupported encoding "{msg.encoding}"')


def _ros_to_depth16(msg: Image) -> np.ndarray:
    """
    Convert a sensor_msgs/Image (16UC1, depth in mm) to a uint16 numpy array.
    """
    return np.frombuffer(msg.data, dtype=np.uint16).reshape(
        msg.height, msg.width).copy()


def _bgr_to_ros(arr: np.ndarray, header) -> Image:
    """
    Convert a BGR uint8 numpy array to a sensor_msgs/Image (bgr8).
    Used only when raw publishing is explicitly requested.
    """
    out = Image()
    out.header   = header
    out.height   = arr.shape[0]
    out.width    = arr.shape[1]
    out.encoding = 'bgr8'
    out.is_bigendian = 0
    out.step     = arr.shape[1] * 3
    out.data     = arr.tobytes()
    return out


def _bgr_to_compressed(arr: np.ndarray, header,
                        quality: int = 80) -> CompressedImage:
    """
    JPEG-compress a BGR numpy array into a sensor_msgs/CompressedImage.

    Why JPEG instead of raw:
      Raw 640×360 BGR  = 691 200 bytes → DDS serialize+copy ~134 ms on Jetson.
      JPEG q=80        ≈  30–60 KB     → DDS serialize+copy  ~5–8 ms.
    The ~10× size reduction cuts publish time by ~10×, freeing ~125 ms/frame.
    """
    ok, buf = cv2.imencode('.jpg', arr,
                           [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        # Fallback: return an empty compressed message rather than crashing
        out = CompressedImage()
        out.header = header
        out.format = 'jpeg'
        out.data   = bytes()
        return out
    out = CompressedImage()
    out.header = header
    out.format = 'jpeg'
    out.data   = buf.tobytes()
    return out


# ═══════════════════════════════════════════════════════════════
#  Colour palette
# ═══════════════════════════════════════════════════════════════

_PALETTE = [
    (255,  56,  56), (255, 157, 151), (255, 112,  31), (255, 178,  29),
    (207, 210,  49), ( 72, 249,  10), (146, 204,  23), ( 61, 219, 134),
    ( 26, 147,  52), (  0, 212, 187), ( 44, 153, 168), (  0, 194, 255),
    ( 52,  69, 147), (100, 115, 255), (  0,  24, 236), (132,  56, 255),
    ( 82,   0, 133), (203,  56, 255), (255, 149, 200), (255,  55, 199),
]


def _color(cls_id: int):
    return _PALETTE[int(cls_id) % len(_PALETTE)]          # BGR tuple for OpenCV


def _color_rgba(cls_id: int, alpha: float = 0.9):
    r, g, b = _PALETTE[int(cls_id) % len(_PALETTE)]
    return r / 255.0, g / 255.0, b / 255.0, alpha


# ═══════════════════════════════════════════════════════════════
#  YoloNode
# ═══════════════════════════════════════════════════════════════

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('model',           'yolo11n.pt')
        self.declare_parameter('conf_threshold',  0.40)
        self.declare_parameter('iou_threshold',   0.45)
        self.declare_parameter('device',          '')      # '' = auto
        self.declare_parameter('process_every_n', 2)
        self.declare_parameter('depth_scale',     0.001)  # RealSense: 1 LSB = 1 mm
        self.declare_parameter('max_depth_m',     5.0)
        self.declare_parameter('marker_lifetime', 0.3)
        # Resize the annotated output image before publishing to reduce DDS
        # serialization overhead.  0 = publish at full input resolution.
        # 640 ≈ 4× less data than 1280×720, cutting publish time dramatically.
        self.declare_parameter('publish_width',   640)

        model_name    = self.get_parameter('model').value
        self.conf_thr = float(self.get_parameter('conf_threshold').value)
        self.iou_thr  = float(self.get_parameter('iou_threshold').value)
        device_param  = self.get_parameter('device').value   # may be '' or 'cuda:0'
        self.skip_n   = int(self.get_parameter('process_every_n').value)
        self.d_scale  = float(self.get_parameter('depth_scale').value)
        self.max_d    = float(self.get_parameter('max_depth_m').value)
        self.mk_life  = float(self.get_parameter('marker_lifetime').value)
        self.pub_w    = int(self.get_parameter('publish_width').value)

        # ── Load YOLO model ─────────────────────────────────────
        if not YOLO_AVAILABLE:
            self.get_logger().fatal(
                'ultralytics not installed — '
                'run: pip3 install ultralytics --break-system-packages')
            raise RuntimeError('ultralytics not installed')

        # Determine target device before loading
        if device_param:
            target_device = device_param
        else:
            target_device = 'cuda:0' if torch.cuda.is_available() else 'cpu'

        self.get_logger().info(
            f'Loading {model_name} → {target_device}  '
            f'(CUDA available: {torch.cuda.is_available()}) …')

        # Jetson torch nv24.8 defaults to weights_only=True (PyTorch 2.6 policy).
        # YOLO .pt files contain Python objects — patch torch.load temporarily.
        import functools
        _orig_load = torch.load

        @functools.wraps(_orig_load)
        def _load_unsafe(*args, **kwargs):
            kwargs.setdefault('weights_only', False)
            return _orig_load(*args, **kwargs)

        torch.load = _load_unsafe
        self.model  = YOLO(model_name)
        torch.load  = _orig_load                    # restore immediately after load

        self.model.to(target_device)
        self.infer_device = target_device
        self.names        = self.model.names         # {0: 'person', …}
        self.get_logger().info(
            f'Model ready — {len(self.names)} classes on {target_device}')

        # ── State ───────────────────────────────────────────────
        self.cam_info    = None
        self.depth_img   = None
        self.depth_lock  = threading.Lock()
        self.frame_count = 0

        # ── QoS ─────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=4,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ── Subscriptions ───────────────────────────────────────
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self._cam_info_cb, reliable_qos)
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            self._depth_cb, sensor_qos)
        self.create_subscription(
            Image, '/camera/color/image_raw',
            self._color_cb, sensor_qos)

        # ── Publishers ──────────────────────────────────────────
        # Image is published as JPEG CompressedImage to avoid DDS serializing
        # a raw 676 KB BGR frame every callback (which cost 134 ms on Jetson).
        # JPEG q=80 ≈ 30–60 KB → serialize in ~5 ms instead of ~134 ms.
        # RViz: use the "Image" display, set topic to
        #   /camera/yolo/image_annotated/compressed   (transport: compressed)
        # or simply add a "Camera" display pointed at this topic.
        self.pub_img     = self.create_publisher(CompressedImage,  '/camera/yolo/image_annotated/compressed', 4)
        self.pub_img_raw = self.create_publisher(Image,             '/camera/yolo/image_annotated',            4)
        self.pub_det2d   = self.create_publisher(Detection2DArray, '/camera/yolo/detections',      10)
        self.pub_det3d   = self.create_publisher(Detection3DArray, '/camera/yolo/detections_3d',   10)
        self.pub_mkr     = self.create_publisher(MarkerArray,      '/camera/yolo/markers',          10)

        self._timing_calls = 0          # count processed frames for timing log
        self.get_logger().info('yolo_node ready — waiting for camera frames …')

    # ── Callbacks ───────────────────────────────────────────────────────── #

    def _cam_info_cb(self, msg: CameraInfo):
        if self.cam_info is None:
            self.cam_info = msg
            self.get_logger().info(
                f'Camera intrinsics: '
                f'fx={msg.k[0]:.1f}  fy={msg.k[4]:.1f}  '
                f'cx={msg.k[2]:.1f}  cy={msg.k[5]:.1f}')

    def _depth_cb(self, msg: Image):
        with self.depth_lock:
            self.depth_img = _ros_to_depth16(msg)

    def _color_cb(self, msg: Image):
        # Frame-skip
        self.frame_count += 1
        if self.frame_count % self.skip_n != 0:
            return
        if self.cam_info is None:
            return

        import time
        _t0 = time.monotonic()

        # ROS Image → BGR numpy (no cv_bridge)
        try:
            bgr = _ros_to_bgr(msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return

        # Snapshot latest depth (thread-safe)
        with self.depth_lock:
            depth = self.depth_img.copy() if self.depth_img is not None else None

        _t1 = time.monotonic()

        # ── YOLO inference ──────────────────────────────────────
        results = self.model.predict(
            bgr,
            conf=self.conf_thr,
            iou=self.iou_thr,
            device=self.infer_device,
            verbose=False,
        )

        _t2 = time.monotonic()

        # Log timing for the first 10 processed frames, then every 50
        self._timing_calls += 1
        if self._timing_calls <= 10 or self._timing_calls % 50 == 0:
            self.get_logger().info(
                f'[timing #{self._timing_calls}]  '
                f'decode={(_t1-_t0)*1000:.1f}ms  '
                f'predict={(_t2-_t1)*1000:.1f}ms  '
                f'img={msg.width}x{msg.height}'
            )

        # ── Build output messages ───────────────────────────────
        det2d_arr = Detection2DArray()
        det2d_arr.header = msg.header

        det3d_arr = Detection3DArray()
        det3d_arr.header = msg.header
        det3d_arr.header.frame_id = 'camera_color_optical_frame'

        marker_arr = MarkerArray()
        annotated  = bgr.copy()

        for result in results:
            for box in result.boxes:
                cls_id       = int(box.cls[0])
                conf         = float(box.conf[0])
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                label        = f'{self.names[cls_id]} {conf:.0%}'
                color        = _color(cls_id)

                # 2-D detection
                det2d = Detection2D()
                det2d.header = msg.header
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(cls_id)
                hyp.hypothesis.score    = conf
                det2d.results.append(hyp)
                det2d.bbox.center.position.x = float((x1 + x2) / 2)
                det2d.bbox.center.position.y = float((y1 + y2) / 2)
                det2d.bbox.size_x = float(x2 - x1)
                det2d.bbox.size_y = float(y2 - y1)
                det2d_arr.detections.append(det2d)

                # 3-D projection
                xyz = self._project_3d(x1, y1, x2, y2, depth)

                # Annotate image
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                (tw, th), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(annotated,
                              (x1, y1 - th - 6), (x1 + tw + 4, y1),
                              color, -1)
                cv2.putText(annotated, label,
                            (x1 + 2, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 255, 255), 1, cv2.LINE_AA)

                if xyz is not None:
                    x3, y3, z3 = xyz
                    dist = math.sqrt(x3**2 + y3**2 + z3**2)

                    # Depth label on image
                    cv2.putText(annotated, f'{dist:.2f}m',
                                (x1 + 2, y2 - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                color, 1, cv2.LINE_AA)

                    # 3-D detection
                    det3d = Detection3D()
                    det3d.header = det3d_arr.header
                    hyp3 = ObjectHypothesisWithPose()
                    hyp3.hypothesis.class_id      = str(cls_id)
                    hyp3.hypothesis.score         = conf
                    hyp3.pose.pose.position.x     = x3
                    hyp3.pose.pose.position.y     = y3
                    hyp3.pose.pose.position.z     = z3
                    hyp3.pose.pose.orientation.w  = 1.0
                    det3d.results.append(hyp3)
                    det3d_arr.detections.append(det3d)

                    mk_lifetime = Duration(seconds=self.mk_life).to_msg()

                    # Sphere marker
                    mk = Marker()
                    mk.header    = det3d_arr.header
                    mk.ns        = 'yolo_3d'
                    mk.id        = len(marker_arr.markers)
                    mk.type      = Marker.SPHERE
                    mk.action    = Marker.ADD
                    mk.pose.position.x    = x3
                    mk.pose.position.y    = y3
                    mk.pose.position.z    = z3
                    mk.pose.orientation.w = 1.0
                    mk.scale.x = mk.scale.y = mk.scale.z = 0.15
                    r, g, b, a = _color_rgba(cls_id)
                    mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, a
                    mk.lifetime = mk_lifetime
                    marker_arr.markers.append(mk)

                    # Text marker
                    txt = Marker()
                    txt.header    = det3d_arr.header
                    txt.ns        = 'yolo_text'
                    txt.id        = len(marker_arr.markers)
                    txt.type      = Marker.TEXT_VIEW_FACING
                    txt.action    = Marker.ADD
                    txt.pose.position.x    = x3
                    txt.pose.position.y    = y3
                    txt.pose.position.z    = z3 + 0.12
                    txt.pose.orientation.w = 1.0
                    txt.scale.z   = 0.10
                    txt.color.r = txt.color.g = txt.color.b = 1.0
                    txt.color.a   = 1.0
                    txt.text      = f'{self.names[cls_id]}\n{dist:.2f}m'
                    txt.lifetime  = mk_lifetime
                    marker_arr.markers.append(txt)

        _t3 = time.monotonic()

        # Downscale before JPEG encoding (640 wide is plenty for RViz viewing)
        if self.pub_w > 0 and annotated.shape[1] > self.pub_w:
            scale  = self.pub_w / annotated.shape[1]
            new_h  = int(annotated.shape[0] * scale)
            to_pub = cv2.resize(annotated, (self.pub_w, new_h),
                                interpolation=cv2.INTER_LINEAR)
        else:
            to_pub = annotated

        # Publish — JPEG CompressedImage (30–60 KB) instead of raw BGR (676 KB)
        self.pub_img.publish(_bgr_to_compressed(to_pub, msg.header, quality=80))
        self.pub_img_raw.publish(_bgr_to_ros(to_pub, msg.header))
        self.pub_det2d.publish(det2d_arr)
        self.pub_det3d.publish(det3d_arr)
        self.pub_mkr.publish(marker_arr)

        _t4 = time.monotonic()

        if self._timing_calls <= 10 or self._timing_calls % 50 == 0:
            self.get_logger().info(
                f'[timing #{self._timing_calls}]  '
                f'post={(_t3-_t2)*1000:.1f}ms  '
                f'publish={(_t4-_t3)*1000:.1f}ms  '
                f'total_cb={(_t4-_t0)*1000:.1f}ms  '
                f'pub_img={to_pub.shape[1]}x{to_pub.shape[0]}'
            )

    # ── 3-D projection ──────────────────────────────────────────────────── #

    def _project_3d(self, x1, y1, x2, y2, depth_img):
        """
        Return (X, Y, Z) metres in camera_color_optical_frame, or None.
        Uses the median of valid depth pixels inside the bounding box patch
        to avoid edge noise and background bleed-through.
        """
        if depth_img is None or self.cam_info is None:
            return None

        h, w = depth_img.shape
        x1c, x2c = max(0, x1), min(w - 1, x2)
        y1c, y2c = max(0, y1), min(h - 1, y2)
        if x1c >= x2c or y1c >= y2c:
            return None

        patch = depth_img[y1c:y2c, x1c:x2c].astype(np.float32) * self.d_scale
        valid = patch[(patch > 0.1) & (patch < self.max_d)]
        if valid.size < 5:
            return None

        Z  = float(np.median(valid))
        u  = (x1 + x2) / 2.0
        v  = (y1 + y2) / 2.0
        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]

        return (u - cx) * Z / fx, (v - cy) * Z / fy, Z


# ═══════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
