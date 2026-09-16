import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

FRONT_OFFSET = (0.5183, 0.0, 0.0)
REAR_OFFSET  = (-0.5183, 0.0, math.pi)

OUT_ANGLE_MIN = -math.pi
OUT_ANGLE_MAX = math.pi
OUT_SAMPLES   = 720
OUT_RANGE_MIN = 0.10
OUT_RANGE_MAX = 20.0
PUBLISH_HZ    = 15.0


class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')

        self.declare_parameter('robot_name', '')
        ns = self.get_parameter('robot_name').value
        prefix = f'/{ns}' if ns else ''

        self._out_frame = f'{ns}/base_footprint' if ns else 'base_footprint'
        self._front_msg = None
        self._rear_msg  = None

        self.create_subscription(LaserScan, f'{prefix}/scan_front', self._front_cb, 10)
        self.create_subscription(LaserScan, f'{prefix}/scan_rear',  self._rear_cb, 10)
        self._pub = self.create_publisher(LaserScan, f'{prefix}/scan', 10)

        self._angle_increment = (OUT_ANGLE_MAX - OUT_ANGLE_MIN) / OUT_SAMPLES
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_merged)
        self.get_logger().info(f'scan_merger ready for "{ns}" ({prefix}/scan_front + {prefix}/scan_rear -> {prefix}/scan)')

    def _front_cb(self, msg): self._front_msg = msg
    def _rear_cb(self, msg):  self._rear_msg = msg

    def _project_into_output(self, msg, offset, out_ranges):
        ox, oy, oyaw = offset
        cos_o, sin_o = math.cos(oyaw), math.sin(oyaw)
        angle = msg.angle_min
        for r in msg.ranges:
            if not (msg.range_min <= r <= msg.range_max):
                angle += msg.angle_increment
                continue
            sx, sy = r * math.cos(angle), r * math.sin(angle)
            bx = cos_o * sx - sin_o * sy + ox
            by = sin_o * sx + cos_o * sy + oy
            b_range = math.hypot(bx, by)
            b_angle = math.atan2(by, bx)
            if OUT_RANGE_MIN <= b_range <= OUT_RANGE_MAX:
                idx = int((b_angle - OUT_ANGLE_MIN) / self._angle_increment)
                if 0 <= idx < OUT_SAMPLES and b_range < out_ranges[idx]:
                    out_ranges[idx] = b_range
            angle += msg.angle_increment

    def _publish_merged(self):
        if self._front_msg is None or self._rear_msg is None:
            return
        out_ranges = [float('inf')] * OUT_SAMPLES
        self._project_into_output(self._front_msg, FRONT_OFFSET, out_ranges)
        self._project_into_output(self._rear_msg, REAR_OFFSET, out_ranges)
        clean_ranges = [r if r != float('inf') else OUT_RANGE_MAX + 0.1 for r in out_ranges]

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._out_frame
        msg.angle_min, msg.angle_max = OUT_ANGLE_MIN, OUT_ANGLE_MAX
        msg.angle_increment = self._angle_increment
        msg.scan_time = 1.0 / PUBLISH_HZ
        msg.range_min, msg.range_max = OUT_RANGE_MIN, OUT_RANGE_MAX
        msg.ranges = clean_ranges
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()