import rclpy
from rclpy.node import Node

from scorpius_main.msg import Teleop

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher_ = self.create_publisher(Teleop, '/scorpius/teleop', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Teleop()
        msg.posx = float(self.i)
        msg.posy = float(self.i) * 2
        msg.posz = float(self.i) * 4
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.posx} {msg.posy} {msg.posz}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    teleop_node = TeleopNode()

    # Try/Except here because ROS doesn't catch it as well on Python as on C++
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        teleop_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()