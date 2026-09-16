import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import struct
import math

FOOTPRINT_POINTS = [  # matches your costmap footprint, sampled as a point ring
    (0.665, 0.476), (0.665, -0.476), (-0.665, -0.476), (-0.665, 0.476)
]
PUBLISH_HZ = 10.0


class RobotObstaclePublisher(Node):
    def __init__(self):
        super().__init__('robot_obstacle_publisher')
        self.declare_parameter('robot_name', '')
        self._ns = self.get_parameter('robot_name').value

        self._x = self._y = self._yaw = 0.0

        self.create_subscription(Odometry, f'/{self._ns}/odom', self._odom_cb, 10)
        self._pub = self.create_publisher(PointCloud2, '/fleet_layer/points', 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_footprint)

    def _odom_cb(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        qz, qw = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        self._yaw = 2.0 * math.atan2(qz, qw)

    def _publish_footprint(self):
        cos_y, sin_y = math.cos(self._yaw), math.sin(self._yaw)
        points = []
        for (lx, ly) in FOOTPRINT_POINTS:
            wx = self._x + cos_y * lx - sin_y * ly
            wy = self._y + sin_y * lx + cos_y * ly
            points.append((wx, wy, 0.0))

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        msg.data = b''.join(struct.pack('fff', *p) for p in points)
        msg.is_dense = True
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()