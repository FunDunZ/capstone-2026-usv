import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from usv_msgs.msg import YoloDetectionArray, DetectionArray, Detection
from cv_bridge import CvBridge
import numpy as np
from usv_autonomy.autonomy.object_localizer import localize_object
from usv_autonomy.label_map import map_label
from usv_autonomy.mavlink.connection import connect_usv
from usv_autonomy.mavlink.mission import get_vessel_pos, pump_messages


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridge = CvBridge()
        self._depth_image  = None
        self._focal_length = None
        self._image_width  = None
        self._image_height = None

        self._usv = connect_usv()
        self.get_logger().info("MAVLink connected")

        self.create_subscription(YoloDetectionArray, '/usv/yolo/detections', self._yolo_callback, 10)
        self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self._depth_callback, 10)
        self.create_subscription(CameraInfo, '/zed/zed_node/rgb/camera_info', self._camera_info_callback, 1)

        self._pub = self.create_publisher(DetectionArray, '/usv/detections/localized', 10)
        self.get_logger().info("Detection node ready")

    def _camera_info_callback(self, msg):
        if self._focal_length is None:
            self._focal_length = msg.k[0]
            self._image_width  = msg.width
            self._image_height = msg.height
            self.get_logger().info(f"Camera info: fx={self._focal_length:.1f} {self._image_width}x{self._image_height}")

    def _depth_callback(self, msg):
        try:
            self._depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f"Depth conversion failed: {e}")

    def _yolo_callback(self, msg):
        if self._focal_length is None:
            self.get_logger().warn("Waiting for camera info...", throttle_duration_sec=5.0)
            return
        if self._depth_image is None:
            self.get_logger().warn("Waiting for depth image...", throttle_duration_sec=5.0)
            return

        pump_messages(self._usv)
        vessel_pos = get_vessel_pos(self._usv)
        if vessel_pos is None:
            self.get_logger().warn("No vessel position yet", throttle_duration_sec=5.0)
            return

        out_msg = DetectionArray()
        out_msg.header = msg.header

        for det in msg.detections:
            obj_type, color = map_label(det.label)
            if obj_type == "DISCARD":
                continue

            px = int(np.clip(det.bbox_cx, 0, self._image_width  - 1))
            py = int(np.clip(det.bbox_cy, 0, self._image_height - 1))
            depth_m = float(self._depth_image[py, px])

            if not np.isfinite(depth_m):
                continue

            localized = localize_object(
                bbox_cx=det.bbox_cx, bbox_cy=det.bbox_cy,
                depth_m=depth_m,
                image_width=self._image_width, image_height=self._image_height,
                focal_length_px=self._focal_length,
                vessel_pos=vessel_pos,
                obj_type=obj_type, confidence=det.confidence,
            )
            if localized is None:
                continue

            d = Detection()
            d.header   = msg.header
            d.obj_type = obj_type
            d.color    = color if color else ""
            d.label    = det.label
            d.lat      = localized["lat"]
            d.lon      = localized["lon"]
            d.confidence = det.confidence
            out_msg.detections.append(d)

        if out_msg.detections:
            self._pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
