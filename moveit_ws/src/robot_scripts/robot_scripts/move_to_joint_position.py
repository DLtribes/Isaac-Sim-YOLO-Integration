import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointMover(Node):
    def __init__(self):
        super().__init__('joint_mover')
        self.publisher = self.create_publisher(JointState, '/isaac_joint_command', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.sent = False

    def timer_callback(self):
        if self.sent:
            return

        joint_cmd = JointState()
        joint_cmd.name = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        joint_cmd.position = [
            0.3, -0.7, 0.0, -1.5, 0.0, 1.2, 0.4  # Target pose
        ]

        self.publisher.publish(joint_cmd)
        self.get_logger().info('Sent joint position command.')
        self.sent = True

def main(args=None):
    rclpy.init(args=args)
    node = JointMover()
    rclpy.spin_once(node, timeout_sec=2.0)  # Just publish once
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
