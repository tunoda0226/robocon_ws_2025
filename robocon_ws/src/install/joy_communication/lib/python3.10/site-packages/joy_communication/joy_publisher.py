import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.joy_publisher = self.create_publisher(Joy, '/remote_joy', 10)

    def joy_callback(self, msg):
        self.get_logger().info(f'Received Joy: {msg.axes}, {msg.buttons}')
        self.joy_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

