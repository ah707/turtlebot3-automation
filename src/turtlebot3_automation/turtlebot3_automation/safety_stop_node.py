import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SafetyStopNode(Node):
    def __init__(self):
        super().__init__('safety_stop_node')

        self.create_subscription(String, '/detections', self.detection_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.alert_pub = self.create_publisher(String, '/health_alerts', 10)

        self.get_logger().info("SafetyStopNode started: listening to /detections")

    def detection_cb(self, msg: String):
        text = msg.data.strip()

        try:
            count = int(text.split()[1])
        except Exception:
            return

        if count > 0:
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.cmd_pub.publish(stop)

            alert = String()
            alert.data = f"SAFETY STOP: {count} obstacle(s) detected"
            self.alert_pub.publish(alert)

            self.get_logger().warn(alert.data)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
