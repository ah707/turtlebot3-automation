import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Laser topic exists in TurtleBot3 sim
        self.scan_topic = '/scan'

        # Parameters (easy to explain in report)
        self.declare_parameter('distance_threshold', 0.8)   # meters
        self.declare_parameter('min_cluster_size', 5)       # consecutive points

        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.detection_pub = self.create_publisher(String, '/detections', 10)

        self.get_logger().info(f"ObjectDetectionNode (LaserScan) started. Subscribed to {self.scan_topic}")

    def scan_cb(self, msg: LaserScan):
        threshold = float(self.get_parameter('distance_threshold').value)
        min_cluster = int(self.get_parameter('min_cluster_size').value)

        # ranges: list of distances; can include inf or nan
        ranges = msg.ranges

        clusters = 0
        current_cluster_size = 0

        for r in ranges:
            if r > 0.0 and r < threshold:
                current_cluster_size += 1
            else:
                if current_cluster_size >= min_cluster:
                    clusters += 1
                current_cluster_size = 0

        # handle cluster at end
        if current_cluster_size >= min_cluster:
            clusters += 1

        out = String()
        out.data = f"Detected {clusters} obstacle cluster(s) within {threshold:.2f} m"
        self.detection_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
