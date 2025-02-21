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
        #self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
    
    def joy_callback(self, msg):
        try:
            # axes_dataは最初のCANメッセージに含める
            axes_data = [int((axis + 1) * 127) for axis in msg.axes[:4]]
            
            # 最初の8個のボタンデータを送信
            buttons_data_part1 = [int(button) for button in msg.buttons[:8]]
            can_data_part1 = axes_data + buttons_data_part1  # 4軸 + 最初の8個のボタン
            
            # 長さが8バイトになるように調整
            can_data_part1 = can_data_part1[:8] + [0] * (8 - len(can_data_part1))
            can_message_part1 = can.Message(arbitration_id=0x123, data=can_data_part1, is_extended_id=False)
            #self.can_bus.send(can_message_part1)
            self.get_logger().info(f'Sent first CAN message: {can_data_part1}')
            
            # ボタンデータの後半の8個を送信
            buttons_data_part2 = [int(button) for button in msg.buttons[8:16]]
            can_data_part2 = buttons_data_part2  # 残りの8個のボタン
            
            # 長さが8バイトになるように調整
            can_data_part2 = can_data_part2[:8] + [0] * (8 - len(can_data_part2))
            can_message_part2 = can.Message(arbitration_id=0x124, data=can_data_part2, is_extended_id=False)
            #self.can_bus.send(can_message_part2)
            self.get_logger().info(f'Sent second CAN message: {can_data_part2}')
        
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

