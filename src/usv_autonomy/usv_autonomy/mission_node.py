import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from usv_msgs.msg import DetectionArray
from usv_autonomy.mavlink.connection import connect_usv
from usv_autonomy.mavlink.mission import mission_download, get_vessel_pos, get_current_wp, pump_messages
from usv_autonomy.autonomy.mission_manager import MissionManager

TASK_MODE = "GATE"
TIMER_HZ  = 5.0


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self._usv = connect_usv()
        self.get_logger().info("MAVLink connected")

        self._manager = MissionManager(self._usv)
        self._manager.task_mode = TASK_MODE

        wp_count, mission = mission_download(self._usv)
        if wp_count == 0:
            self.get_logger().error("Mission download failed or empty!")
        else:
            self._manager.set_mission(mission)
            self.get_logger().info(f"Mission loaded: {wp_count} waypoints")

        self._latest_detections = []

        self.create_subscription(DetectionArray, '/usv/detections/localized', self._detection_callback, 10)
        self._state_pub = self.create_publisher(String, '/usv/state', 10)
        self.create_timer(1.0 / TIMER_HZ, self._tick)
        self.get_logger().info("Mission node ready")

    def _detection_callback(self, msg):
        self._latest_detections = []
        for d in msg.detections:
            det = {"type": d.obj_type, "lat": d.lat, "lon": d.lon, "confidence": d.confidence}
            if d.color:
                det["color"] = d.color
            self._latest_detections.append(det)

    def _tick(self):
        pump_messages(self._usv)
        vessel_pos = get_vessel_pos(self._usv)
        current_wp = get_current_wp(self._usv)

        if vessel_pos is None:
            self.get_logger().warn("No vessel position", throttle_duration_sec=5.0)
            return

        self._manager._update(vessel_pos, current_wp, self._latest_detections)

        state_msg = String()
        state_msg.data = self._manager.state
        self._state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
