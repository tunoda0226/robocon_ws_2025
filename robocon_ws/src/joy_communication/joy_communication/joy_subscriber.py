import rclpy
import can

from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.joy_subscription = self.create_subscription(
            Joy,
            '/remote_joy',
            self.joy_callback,
            10
        )
        # CAN通信の初期化
        self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan')

    def joy_callback(self, msg):
        try:
            axes_data = [int((axis + 1) * 127) for axis in msg.axes[:4]]
            buttons_data = [int(button) for button in msg.buttons[:12]]
            can_data = [33] + axes_data + buttons_data  # ASCII value of '!' is 33
            #can_data = can_data[:8] + [0] * (8 - len(can_data))  # 長さが8バイトになるように調整する
            can_message = can.Message(arbitration_id=0x123, data=can_data, is_extended_id=False)
            # CANメッセージを送信する
            #self.can_bus.send(can_message)
            self.get_logger().info(f'Sent CAN message: {can_data}')
        except Exception as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
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

