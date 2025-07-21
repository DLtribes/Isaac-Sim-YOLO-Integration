import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class ScissorsMover(Node):
    def __init__(self):
        super().__init__('scissors_mover')
        self.subscription = self.create_subscription(String, '/object_label', self.label_callback, 10)
        self.publisher = self.create_publisher(JointState, '/isaac_joint_command', 10)
        self.target_sent = False
        self.get_logger().info("Waiting for 'scissors' label on /object_label...")

    def label_callback(self, msg):
        if msg.data.lower() == 'scissors' and not self.target_sent:
            self.get_logger().info("Scissors detected! Moving robot...")
            self.send_joint_command()

    def send_joint_command(self):
        joint_cmd = JointState()
        joint_cmd.name = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        joint_cmd.position = [
            -0.2, -0.5, 0.3, -1.8, 0.1, 1.4, 0.2  # Replace with your desired pose
        ]

        self.publisher.publish(joint_cmd)
        self.target_sent = True
        self.get_logger().info("Joint command for scissors sent.")

def main(args=None):
    rclpy.init(args=args)
    node = ScissorsMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
