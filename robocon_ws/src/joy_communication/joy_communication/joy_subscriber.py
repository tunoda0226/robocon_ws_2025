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
            # 左スティックでの移動 (vx, vy)
            vx = msg.axes[0]  # 左スティック x軸方向の速度
            vy = msg.axes[1]  # 左スティック y軸方向の速度
            
            # 右スティックでの旋回 (ω) — x軸を使って旋回
            omega = msg.axes[2]  # 右スティックのx軸が回転に対応
            
            # lx + lyは車輪の配置に関わる定数（ 0.25 と設定）
            lx = 0.25
            ly = 0.25

            # 各ホイールの速度を計算
            v1 = vx + vy + (lx + ly) * omega
            v2 = -vx + vy + (lx + ly) * omega
            v3 = -vx - vy + (lx + ly) * omega
            v4 = vx - vy + (lx + ly) * omega

            # モーターの速度は範囲を0~254に制限
            motor_speeds = [self.clamp_speed(v1), self.clamp_speed(v2), self.clamp_speed(v3), self.clamp_speed(v4)]
            
            # ボタンデータを送信
            # 最初の8個のボタンデータ
            buttons_data_part1 = [int(button) for button in msg.buttons[:8]]
            can_data_part1 = motor_speeds + buttons_data_part1  # 4つのモーター速度 + 最初の8個のボタン
            
            # 長さが8バイトになるように調整
            can_data_part1 = can_data_part1[:8] + [0] * (8 - len(can_data_part1))
            can_message_part1 = can.Message(arbitration_id=0x123, data=can_data_part1, is_extended_id=False)
            #self.can_bus.send(can_message_part1)
            self.get_logger().info(f'Sent first CAN message: {can_data_part1}')
            
            # 残りの8個のボタンデータ
            buttons_data_part2 = [int(button) for button in msg.buttons[8:16]]
            can_data_part2 = buttons_data_part2  # 残りの8個のボタン
            
            # 長さが8バイトになるように調整
            can_data_part2 = can_data_part2[:8] + [0] * (8 - len(can_data_part2))
            can_message_part2 = can.Message(arbitration_id=0x124, data=can_data_part2, is_extended_id=False)
            #self.can_bus.send(can_message_part2)
            self.get_logger().info(f'Sent second CAN message: {can_data_part2}')
        
        except Exception as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')

    def clamp_speed(self, speed):
        """速度を0~254の範囲に制限"""
        # -1.0 ~ 1.0 から 0 ~ 254 にスケールし、範囲を0~254に制限
        scaled_speed = (speed + 1) * 127  # -1.0~1.0 を 0~254 に変換
        return max(0, min(254, int(scaled_speed)))

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

