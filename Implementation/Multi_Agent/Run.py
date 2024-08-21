import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.topic = '/cmd_vel_Robot0'
        self.publisher_ = self.create_publisher(Twist, self.topic, 10)
        self.active_keys = set()

    def publish_velocity(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.x = angular_x
        msg.angular.y = angular_y
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f'Mensaje publicado en {self.topic}: "{msg}"')

    def publish_stop(self):
        self.publish_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def handle_key_press(self, key):
        if key not in self.active_keys:
            self.active_keys.add(key)
            if key == 'w' or key == 'W':
                self.publish_velocity(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            elif key == 's' or key == 'S':
                self.publish_velocity(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            elif key == 'a' or key == 'A':
                self.publish_velocity(0.0, 0.0, 0.0, 0.0, 0.0, -1.0)
            elif key == 'd' or key == 'D':
                self.publish_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            elif key == 'e' or key == 'E':
                self.change_topic('next')
            elif key == 'q' or key == 'Q':
                self.change_topic('previous')

    def handle_key_release(self, key):
        if key in self.active_keys:
            self.active_keys.remove(key)
            self.publish_stop()

    def change_topic(self, direction):
        topic_map = {
            '/cmd_vel_Robot0': '/cmd_vel_Robot1',
            '/cmd_vel_Robot1': '/cmd_vel_Robot2',
            '/cmd_vel_Robot2': '/cmd_vel_Robot0'
        }
        reverse_map = {v: k for k, v in topic_map.items()}
        
        if direction == 'next':
            new_topic = topic_map.get(self.topic, '/cmd_vel_Robot0')
        elif direction == 'previous':
            new_topic = reverse_map.get(self.topic, '/cmd_vel_Robot0')
        else:
            return
        
        self.topic = new_topic
        self.publisher_ = self.create_publisher(Twist, self.topic, 10)
        self.get_logger().info(f'Tópico cambiado a {self.topic}')

def on_press(key):
    try:
        if hasattr(key, 'char') and key.char:
            velocity_publisher.handle_key_press(key.char.lower())
    except AttributeError:
        pass

def on_release(key):
    try:
        if hasattr(key, 'char') and key.char:
            velocity_publisher.handle_key_release(key.char.lower())
    except AttributeError:
        pass

def main(args=None):
    global velocity_publisher

    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while rclpy.ok():
        rclpy.spin_once(velocity_publisher, timeout_sec=0.1)

    listener.stop()
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
