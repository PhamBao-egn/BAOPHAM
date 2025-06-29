#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

class GoalStatusChecker(Node):
    def __init__(self):
        super().__init__('goal_status_checker')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_reached = None

    def send_goal(self, x, y, z, w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'  # Khớp với global_frame trong navigation.yaml
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.w = w

        self.get_logger().info(f'Đang gửi goal pose: x={x}, y={y}, z={z}, w={w}')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server /navigate_to_pose không khả dụng!')
            self.goal_reached = False
            return
        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: Khoảng cách còn lại: {feedback.distance_remaining:.2f} m')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal bị từ chối!')
            self.goal_reached = False
            return
        self.get_logger().info('Goal được chấp nhận!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Robot đã đến điểm đích!')
            self.goal_reached = True
        else:
            self.get_logger().info(f'Hành động không thành công. Trạng thái: {status}')
            self.goal_reached = False

    def is_goal_reached(self):
        return self.goal_reached

def main(args=None):
    rclpy.init(args=args)
    node = GoalStatusChecker()

    # Yêu cầu người dùng nhập tọa độ từ terminal
    print("Vui lòng nhập tọa độ goal_pose:")
    try:
        x = float(input("Nhập x: "))
        y = float(input("Nhập y: "))
        z = float(input("Nhập z: "))
        w = float(input("Nhập w (quaternion): "))
    except ValueError:
        node.get_logger().error('Tọa độ phải là số thực (float). Ví dụ: 2.0, 3.0, 2.0, 1.0')
        rclpy.shutdown()
        return

    # Gửi goal với tọa độ nhập từ terminal
    node.send_goal(x=x, y=y, z=z, w=w)

    # Chờ vô thời hạn cho đến khi có kết quả
    while rclpy.ok() and node.is_goal_reached() is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    # In kết quả
    if node.is_goal_reached():
        print("True")
    else:
        print("False")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()