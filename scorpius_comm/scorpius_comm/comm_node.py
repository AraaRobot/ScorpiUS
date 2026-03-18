#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from scorpius_main.msg import ServoAngles
from scorpius_main.srv import SerialConfig
import serial

HEAD = 0xAA
TAIL = 0xBB


class CommNode(Node):
    def __init__(self):
        super().__init__('Comm_Node')
        self.subscription = self.create_subscription(
            ServoAngles, '/scorpius/teleop', self.CB_teleop, 10)
        self.subscription  # prevent unused variable warning

        self.srv = self.create_service(
            SerialConfig, '/scorpius/serial_config', self.handle_serial_config)

        self.ser = None

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

    def CB_teleop(self, msg):
        if not getattr(self, 'ser', None) or not getattr(self.ser, 'is_open', False):
            self.get_logger().warning("Serial not open — dropping teleop message")
            return
        try:
            self.ser.write(self.build_packet(msg))
        except Exception as e:
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

    def handle_serial_config(self, request, response):
        try:
            if getattr(self, 'ser', None) and self.ser.is_open:
                self.get_logger().info("Serial already open; reopening with new config")
                self.ser.close()

            self.ser = serial.Serial(
                request.port,
                int(request.baud),
                timeout=float(request.timeout)
            )
            self.ser.reset_input_buffer()

            response.result = True
            response.response = f"Opened {request.port} @ {request.baud}"
            self.get_logger().info(response.response)

        except (serial.SerialException, FileNotFoundError) as e:
            response.result = False
            response.response = str(e)
            self.get_logger().error(response.response)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
