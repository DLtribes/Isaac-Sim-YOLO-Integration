import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import time


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__('trajectory_bridge_node')

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/isaac_joint_command',
            10
        )

        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            '/panda_arm_controller/follow_joint_trajectory',
            self.handle_goal
        )

        self.get_logger().info("Bridge action server ready.")

    def handle_goal(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received trajectory goal from MoveIt.")
        traj = goal_handle.request.trajectory
        start_time = self.get_clock().now()

        for idx, point in enumerate(traj.points):
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = traj.joint_names
            joint_state.position = point.positions
            joint_state.velocity = point.velocities
            joint_state.effort = point.effort
            self.joint_state_pub.publish(joint_state)

            self.get_logger().info(f"Published point {idx + 1}/{len(traj.points)}")

            # sleep based on time_from_start (or fixed step fallback)
            if idx + 1 < len(traj.points):
                t_curr = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                t_next = traj.points[idx + 1].time_from_start.sec + traj.points[idx + 1].time_from_start.nanosec * 1e-9
                time.sleep(max(0.0, t_next - t_curr))

        goal_handle.succeed()
        return FollowJointTrajectory.Result()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
