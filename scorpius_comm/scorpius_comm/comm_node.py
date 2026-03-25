#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from scorpius_main.msg import ServoAngles
from scorpius_main.msg import SerialStatus
from scorpius_main.srv import SerialConfig
import serial

HEAD = 0xAA
TAIL = 0xBB
COMMAND = 0x00
ERROR = 0x02
INFO = 0x01
HEARTBEAT = 0x03

INFO_TEXT = {
    0x01: "Init complete",
    0x02: "Servos homed",
}

ERROR_TEXT = {
    0x01: "Received invalid packet length",
    0x02: "Checksum mismatch",
    0x03: "Packet dropped",
    0x04: "Wrong msg type",
    0x05: "Invalid COMMAND (0x00) received",
    0x06: "Tried to send invalid command",
    0x07: "Invalid servo ID",
}

class CommNode(Node):
    def __init__(self):
        super().__init__('Comm_Node')
        self.subscription = self.create_subscription(
            ServoAngles, '/scorpius/teleop', self.CB_teleop, 10)
        self.subscription  # prevent unused variable warning

        self.srv = self.create_service(
            SerialConfig, '/scorpius/serial_config', self.handle_serial_config)

        self.status_pub = self.create_publisher(
            SerialStatus, '/scorpius/serial_status', 10)

        self.read_timer = self.create_timer(0.05, self.read_serial)

        self.ser = None
        self.rx_buffer = bytearray()

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
            COMMAND,
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

    def read_serial(self):
        if not getattr(self, 'ser', None) or not getattr(self.ser, 'is_open', False):
            return

        # Non-blocking check for incoming data
        if self.ser.in_waiting:
            try:
                data = self.ser.read(self.ser.in_waiting)
                self.process_serial_data(data)
            except Exception as e:
                self.get_logger().error(f"Serial read failed: {e}")
                self.publish_serial_status(False, f"Serial read failed: {e}")

    def process_serial_data(self, data: bytes):
        # Buffer incoming data and parse complete packets in the format:
        # HEAD | LENGTH | PAYLOAD... | CHECKSUM | TAIL
        # PAYLOAD case 1: ERROR (0x02) + error_code (1 byte)
        self.rx_buffer.extend(data)
        self.get_logger().debug(f"Serial RX append ({len(data)} bytes): {data.hex()}")

        while True:
            if len(self.rx_buffer) < 4:
                # Need at least HEAD, LENGTH, CHECKSUM, TAIL
                break

            if self.rx_buffer[0] != HEAD:
                self.get_logger().warning(f"Discarding byte before HEAD: {self.rx_buffer[0]:02X}")
                del self.rx_buffer[0]
                continue

            packet_len = self.rx_buffer[1]
            total_len = 4 + packet_len

            if len(self.rx_buffer) < total_len:
                # Wait for full packet
                break

            if self.rx_buffer[total_len - 1] != TAIL:
                self.get_logger().warning("Malformed packet: wrong TAIL, dropping HEAD byte")
                del self.rx_buffer[0]
                continue

            packet = bytes(self.rx_buffer[:total_len])
            payload = packet[2:2 + packet_len]
            checksum = packet[2 + packet_len]
            calc_chk = sum(payload) & 0xFF

            if checksum != calc_chk:
                self.get_logger().warning(
                    f"Bad checksum: received {checksum:02X}, expected {calc_chk:02X}; dropping packet"
                )
                del self.rx_buffer[0]
                continue

            self.handle_packet(payload)
            del self.rx_buffer[:total_len]

    def publish_serial_status(self, ok: bool, message: str = ""):
        status_msg = SerialStatus()
        status_msg.ok = ok
        status_msg.message = message
        self.status_pub.publish(status_msg)

    def handle_packet(self, payload: bytes):
        if len(payload) == 0:
            self.get_logger().warning("Received empty payload")
            self.publish_serial_status(False, "Empty payload")
            return

        packet_type = payload[0]

        if packet_type == INFO:
            if len(payload) < 2:
                self.get_logger().warning("INFO packet missing info code")
                self.publish_serial_status(False, "INFO packet missing code")
                return
            info_code = payload[1]
            status_text = self.get_info_text(info_code)
            self.get_logger().info(f"Received INFO packet: {status_text}")
            self.publish_serial_status(True, status_text)

        elif packet_type == ERROR:
            if len(payload) < 2:
                self.get_logger().warning("ERROR packet missing error code")
                self.publish_serial_status(False, "ERROR packet missing code")
                return
            error_code = payload[1]
            status_text = f"Error 0x{error_code:02X}: {self.get_error_text(error_code)}"
            self.get_logger().error(f"Received ERROR packet: {status_text}")
            self.publish_serial_status(False, status_text)

        elif packet_type == HEARTBEAT:
            self.get_logger().debug("Received HEARTBEAT packet")
            #self.publish_heartbeat(True) next PR

        else:
            self.get_logger().warning(
                f"Received unknown packet type=0x{packet_type:02X}, payload={payload.hex()}"
            )
            self.publish_serial_status(False, f"Unknown packet type 0x{packet_type:02X}")

    def get_info_text(self, code: int) -> str:
        return INFO_TEXT.get(code, f"Unknown INFO code 0x{code:02X}")

    def get_error_text(self, code: int) -> str:
        return ERROR_TEXT.get(code, f"Unknown ERROR code 0x{code:02X}")

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
