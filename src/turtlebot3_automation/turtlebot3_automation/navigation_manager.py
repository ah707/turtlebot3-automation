import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

import math


class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Goal parameters (change from CLI)
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw_deg', 0.0)

        self.goal_sent = False
        self.timer = self.create_timer(2.0, self.try_send_goal)

        self.get_logger().info("NavigationManager started. Waiting for /navigate_to_pose...")

    def try_send_goal(self):
        if self.goal_sent:
            return

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 not ready yet. Still waiting...")
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.build_goal_pose()

        self.get_logger().info(
            f"Sending goal -> x={goal.pose.pose.position.x:.2f}, y={goal.pose.pose.position.y:.2f}, frame=map"
        )

        self.goal_sent = True
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def build_goal_pose(self) -> PoseStamped:
        x = float(self.get_parameter('goal_x').value)
        y = float(self.get_parameter('goal_y').value)
        yaw_deg = float(self.get_parameter('goal_yaw_deg').value)
        yaw = math.radians(yaw_deg)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # planar yaw -> quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
            self.goal_sent = False
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        result = future.result().result
        self.get_logger().info(f"Navigation finished. status={status}, result={result}")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
