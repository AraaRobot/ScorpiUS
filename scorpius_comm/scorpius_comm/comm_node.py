#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from scorpius_main.msg import ServoAngles
import serial

HEAD = 0xAA
TAIL = 0xBB


class CommNode(Node):
    def __init__(self):
        super().__init__('Comm_Node')
        self.subscription = self.create_subscription(
            ServoAngles, '/scorpius/teleop', self.CB_teleop, 10)
        self.subscription  # prevent unused variable warning

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.ser.reset_input_buffer()

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

    def CB_teleop(self, msg):
        try:
            self.ser.write(self.build_packet(msg))
        except serial.SerialException as e:
            self.get_logger().error(str(e))

    def angle_to_uint8(self, angle):
        angle = int(max(-90, min(90, angle)))
        return angle & 0xFF

    def build_packet(self, msg):
        # order must match Arduino expectations
        values = [
            msg.vert_a,
            msg.vert_b,
            msg.vert_c,
            msg.vert_d,
            msg.vert_e,
            msg.vert_f,
            msg.horiz_a,
            msg.horiz_b,
            msg.horiz_c,
            msg.horiz_d,
            msg.horiz_e,
            msg.horiz_f,
        ]

        payload = bytes(self.angle_to_uint8(v) for v in values)
        length = len(payload)

        checksum = (sum(payload)) & 0xFF

        packet = bytes([HEAD, length]) + payload + bytes([checksum, TAIL])
        return packet


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
