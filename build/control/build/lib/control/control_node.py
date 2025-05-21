#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        self.decoration_sub = self.create_subscription(
            PointStamped,
            '/cake_vision/decoration_point',
            self.decoration_callback,
            10)
        
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10)
        
        self.get_logger().info("Control Node initialized (placeholder)")
    
    def decoration_callback(self, msg):
        self.get_logger().info(
            f"Received decoration point: ({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})"
        )
        # Placeholder: Publish a dummy trajectory
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=1, nanosec=0)
        trajectory.points.append(point)
        self.trajectory_pub.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
