import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry

class MultiRobotPositionSubscriber(Node):
    def __init__(self):
        super().__init__('multi_robot_position_subscriber')

        self.create_subscription(Odometry, '/odom_Robot0', self.robot0_callback, 10)
        self.create_subscription(Odometry, '/odom_Robot1', self.robot1_callback, 10)
        self.create_subscription(Odometry, '/odom_Robot2', self.robot2_callback, 10)

    def robot0_callback(self, msg):
        self.print_position('Robot0', msg)

    def robot1_callback(self, msg):
        self.print_position('Robot1', msg)

    def robot2_callback(self, msg):
        self.print_position('Robot2', msg)

    def print_position(self, robot_name, msg):
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        z = position.z
        print(f'{robot_name}: X={x:.2f}, Y={y:.2f}, Z={z:.2f}')

def main(args=None):
    rclpy.init(args=args)

    node = MultiRobotPositionSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()