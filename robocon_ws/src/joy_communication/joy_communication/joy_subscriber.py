import rclpy
import can
from rclpy.node import Node
from sensor_msgs.msg import Joy
import traceback


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
            # データの変換
            axes_data = [int((axis + 1) * 127) for axis in msg.axes[:4]]
            buttons_data = [int(button) for button in msg.buttons[:12]]
            
            # データを結合して1つのリストに
            combined_data = [33] + axes_data + buttons_data  # '!' + axes_data + buttons_data

            # データを8バイトごとに分割
            data_chunks = [
                combined_data[i:i + 8] for i in range(0, len(combined_data), 8)
            ]

            # 各チャンクを送信
            for chunk in data_chunks:
                chunk = chunk + [0] * (8 - len(chunk))  # 8バイトに調整
                self.send_can_message(0x123, chunk)

        except Exception as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')
            self.get_logger().error(traceback.format_exc())

    def send_can_message(self, arbitration_id, data):
        # CANメッセージを送信
        can_message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        self.can_bus.send(can_message)
        self.get_logger().info(f'Sent CAN message: ID={hex(arbitration_id)}, Data={data}')

    def destroy_node(self):
        # CAN通信のクリーンアップ
        self.can_bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Exception: {e}')
        node.get_logger().error(traceback.format_exc())
    finally:
        # ノードのクリーンアップを安全に実行
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

