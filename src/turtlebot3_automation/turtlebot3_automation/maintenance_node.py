import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from datetime import datetime
import os


class MaintenanceNode(Node):
    def __init__(self):
        super().__init__('maintenance_node')

        # --- Topics (adjustable later)
        self.battery_topic = '/battery_state'
        self.odom_topic = '/odom'
        self.diag_topic = '/diagnostics'

        # --- Subscribers
        self.create_subscription(BatteryState, self.battery_topic, self.battery_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(DiagnosticArray, self.diag_topic, self.diag_callback, 10)

        # --- Alerts publisher
        self.alert_pub = self.create_publisher(String, '/health_alerts', 10)

        # --- Internal state
        self.last_battery = None
        self.last_battery_time = None
        self.last_odom_time = None
        self.last_diag_time = None
        self.last_diag_status_summary = None

        # --- Logging
        self.log_dir = os.path.expanduser('~/turtlebot3_logs')
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, 'health_log.txt')

        # --- Timer (every 10 seconds)
        self.create_timer(10.0, self.health_check)

        self.get_logger().info('Maintenance node started')

    def log(self, level, message):
        timestamp = datetime.now().isoformat()
        line = f"[{timestamp}] [{level}] {message}"
        self.get_logger().info(line)
        with open(self.log_file, 'a') as f:
            f.write(line + '\n')

    def alert(self, message):
        msg = String()
        msg.data = message
        self.alert_pub.publish(msg)
        self.log("ALERT", message)

    # --- Callbacks
    def battery_callback(self, msg: BatteryState):
        self.last_battery = msg
        self.last_battery_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.last_odom_time = self.get_clock().now()

    def diag_callback(self, msg: DiagnosticArray):
        self.last_diag_time = self.get_clock().now()

        # Create a tiny summary you can show in logs/demo
        worst = 0  # 0 OK, 1 WARN, 2 ERROR, 3 STALE (ROS convention)
        worst_name = None
        for st in msg.status:
            if st.level > worst:
                worst = st.level
                worst_name = st.name

        if worst == 0:
            self.last_diag_status_summary = "Diagnostics OK"
        elif worst == 1:
            self.last_diag_status_summary = f"Diagnostics WARN: {worst_name}"
        elif worst == 2:
            self.last_diag_status_summary = f"Diagnostics ERROR: {worst_name}"
        else:
            self.last_diag_status_summary = f"Diagnostics STALE: {worst_name}"

    # --- Periodic checks
    def health_check(self):
        now = self.get_clock().now()

        # Battery check (BatteryState)
        if self.last_battery_time:
            dt = (now - self.last_battery_time).nanoseconds / 1e9
            percent = self.last_battery.percentage * 100.0
            voltage = self.last_battery.voltage

            self.log("INFO", f"Battery: {percent:.1f}% | Voltage: {voltage:.2f}V | Last update: {dt:.1f}s ago")

            if percent >= 0 and percent < 20.0:
                self.alert("Battery below 20%")
            if dt > 30.0:
                self.alert("Battery data stale (no recent updates)")
        else:
            self.log("WARN", f"No battery data received on {self.battery_topic}")

        # Odometry check
        if self.last_odom_time:
            dt = (now - self.last_odom_time).nanoseconds / 1e9
            self.log("INFO", f"Odometry OK | Last update: {dt:.1f}s ago")
            if dt > 5.0:
                self.alert("No odometry data recently (motors/sensors may be down)")
        else:
            self.log("WARN", f"No odometry data received on {self.odom_topic}")

        # Diagnostics check
        if self.last_diag_time:
            dt = (now - self.last_diag_time).nanoseconds / 1e9
            summary = self.last_diag_status_summary or "Diagnostics received"
            self.log("INFO", f"{summary} | Last update: {dt:.1f}s ago")
            if dt > 15.0:
                self.alert("Diagnostics stale")
            if summary.startswith("Diagnostics ERROR"):
                self.alert(summary)
        else:
            self.log("INFO", f"No diagnostics yet on {self.diag_topic} (this is normal if bringup not running)")


def main(args=None):
    rclpy.init(args=args)
    node = MaintenanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

