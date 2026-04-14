#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from scorpius_main.msg import ServoAngles
from scorpius_main.msg import SerialStatus
from scorpius_main.msg import SerialHeartbeat
from scorpius_main.srv import SerialConfig
from scorpius_main.srv import SerialPorts
from scorpius_main.srv import ControllerState
import serial
import serial.tools.list_ports

# Packet
HEAD = 0xAA
TAIL = 0xBB

# Msg types
COMMAND = 0x00
ERROR = 0x02
INFO = 0x01
HEARTBEAT = 0x03
STATE = 0x04

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
    0x08: "Invalid state received",
}

# States
HOME = 0x01
RUNNING = 0x02
REBOOT = 0x03


class CommNode(Node):
    def __init__(self):
        super().__init__('Comm_Node')
        self.subscription = self.create_subscription(
            ServoAngles, '/scorpius/teleop', self.CB_teleop, 10)
        self.subscription  # prevent unused variable warning

        self.srv = self.create_service(
            SerialConfig, '/scorpius/serial_config', self.handle_serial_config)

        self.port_srv = self.create_service(
            SerialPorts, '/scorpius/serial_ports', self.handle_serial_ports)

        self.status_pub = self.create_publisher(
            SerialStatus, '/scorpius/serial_status', 10)

        self.read_timer = self.create_timer(0.05, self.read_serial)

        heartbeat_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            deadline=Duration(seconds=1.0),
            lifespan=Duration(seconds=1.5),
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=1.5),
        )

        self.heartbeat_pub = self.create_publisher(
            SerialHeartbeat, '/scorpius/serial_heartbeat', heartbeat_qos)
        self.heartbeat_timer = self.create_timer(1.0, self.heartbeat_check)
        self.heartbeat_ok: bool = False
        self.heartbeat_sequence: int = 0

        self.state_controller = self.create_service(
            ControllerState, '/scorpius/state_controller', self.send_state)

        self.ser = None
        self.rx_buffer = bytearray()

    def destroy_node(self):
        if self.serial_ready():
            self.ser.close()
        super().destroy_node()

    def serial_ready(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _disable_heartbeat_and_flush(self) -> None:
        if self.serial_ready():
            try:
                self.ser.reset_input_buffer()
                self.ser.flush()
                self.ser.reset_output_buffer()
            except Exception as e:
                self.get_logger().warning(f"Serial flush failed: {e}")        
        self.heartbeat_ok = False
        self.publish_heartbeat(False)
        self.rx_buffer.clear()
        self.get_logger().info("Heartbeat disabled and serial buffer flushed")

    def send_state(self, request: ControllerState.Request, response: ControllerState.Response) -> ControllerState.Response:
        if not self.serial_ready():
            self.get_logger().warning(
                "Serial not open — dropping state controller message")
            response.success = False
            response.message = "Serial port not open"
            return response

        if not self.heartbeat_ok:
            self.get_logger().warning(
                "No heartbeat detected — dropping state controller message")
            response.success = False
            response.message = "No heartbeat detected"
            return response

        if request.state not in (HOME, RUNNING, REBOOT):
            response.success = False
            response.message = f"Invalid state {request.state}"
            return response
        try:
            state_byte = int(request.state).to_bytes(
                1, byteorder='little', signed=True)
            self.ser.write(self.build_state_packet(state_byte))
            if (request.state == REBOOT):
                self._disable_heartbeat_and_flush()
            response.success = True
            response.message = f"Serial write successful"
        except Exception as e:
            self.get_logger().error(str(e))
            response.success = False
            response.message = str(e)
        return response

    def CB_teleop(self, msg: ServoAngles) -> None:
        if not self.serial_ready():
            self.get_logger().warning(
                "Serial not open — dropping teleop message", throttle_duration_sec=30)
            return
        if not self.heartbeat_ok:
            self.get_logger().warning(
                "No heartbeat - dropping teleop message", throttle_duration_sec=30)
            return
        try:
            self.ser.write(self.build_command_packet(msg))
        except Exception as e:
            self.get_logger().error(str(e))

    def angle_to_uint8(self, angle: float) -> int:
        # map [-90,90] to uint8 by two's complement representation for signed int8 receiver
        angle = int(max(-90, min(90, angle)))
        return angle & 0xFF

    def build_command_packet(self, msg: ServoAngles) -> bytes:
        angles = [
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

        data = bytes(self.angle_to_uint8(v) for v in angles)
        packet_type = COMMAND
        length = 1 + len(data)  # msg type + data

        checksum = (packet_type + sum(data)) & 0xFF

        packet = bytes([HEAD, length, packet_type]) + \
            data + bytes([checksum, TAIL])
        return packet

    def build_state_packet(self, state: bytes) -> bytes:
        packet_type = STATE

        checksum = (packet_type + sum(state)) & 0xFF
        length = 2  # 1 msg type + 1 msg content
        packet = bytes([HEAD, length, packet_type]) + \
            state + bytes([checksum, TAIL])
        return packet

    def handle_serial_config(self, request: SerialConfig.Request, response: SerialConfig.Response) -> SerialConfig.Response:

        if request.command == SerialConfig.Request.CONNECT:
            try:
                if self.serial_ready():
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
        elif request.command == SerialConfig.Request.DISCONNECT:
            if self.serial_ready():
                self.ser.close()
                response.result = True
                response.response = "Closed port"
                self._disable_heartbeat_and_flush();
            else:
                response.result = False
                response.response = "Port was already closed"
        else:
            response.result = False
            response.response = f"Wrong command type {request.command}"

        return response

    def handle_serial_ports(self, request: SerialPorts.Request, response: SerialPorts.Response) -> SerialPorts.Response:
        ports = [p for p in serial.tools.list_ports.comports()
                 if p.description.lower() != 'n/a']
        response.ports = [p.device for p in ports]
        response.descriptions = [p.description for p in ports]
        return response

    def read_serial(self) -> None:
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

    def process_serial_data(self, data: bytes) -> None:
        # Buffer incoming data and parse complete packets in the format:
        # HEAD | LENGTH | PAYLOAD... | CHECKSUM | TAIL
        # PAYLOAD case 1: ERROR (0x02) + error_code (1 byte)
        self.rx_buffer.extend(data)
        self.get_logger().debug(
            f"Serial RX append ({len(data)} bytes): {data.hex()}")

        while True:
            if len(self.rx_buffer) < 4:
                # Need at least HEAD, LENGTH, CHECKSUM, TAIL
                break

            if self.rx_buffer[0] != HEAD:
                self.get_logger().warning(
                    f"Discarding byte before HEAD: {self.rx_buffer[0]:02X}")
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

    def publish_serial_status(self, ok: bool, message: str = "") -> None:
        status_msg = SerialStatus()
        status_msg.ok = ok
        status_msg.message = message
        self.status_pub.publish(status_msg)

    def handle_packet(self, payload: bytes) -> None:
        if len(payload) == 0:
            self.get_logger().warning("Received empty payload")
            self.publish_serial_status(True, "Empty payload")
            return

        packet_type = payload[0]

        if packet_type == INFO:
            if len(payload) < 2:
                self.get_logger().warning("INFO packet missing info code")
                self.publish_serial_status(True, "INFO packet missing code")
                return
            info_code = payload[1]
            status_text = self.get_info_text(info_code)
            self.get_logger().info(f"Received INFO packet: {status_text}")
            self.publish_serial_status(True, status_text)

        elif packet_type == ERROR:
            if len(payload) < 2:
                self.get_logger().warning("ERROR packet missing error code")
                self.publish_serial_status(True, "ERROR packet missing code")
                return
            error_code = payload[1]
            status_text = f"Error 0x{error_code:02X}: {self.get_error_text(error_code)}"
            self.get_logger().error(f"Received ERROR packet: {status_text}")
            self.publish_serial_status(True, status_text)

        elif packet_type == HEARTBEAT:
            self.get_logger().debug("Received HEARTBEAT packet")
            self.publish_heartbeat(True)
            self.heartbeat_ok = True

        else:
            self.get_logger().warning(
                f"Received unknown packet type=0x{packet_type:02X}, payload={payload.hex()}"
            )
            self.publish_serial_status(
                True, f"Unknown packet type 0x{packet_type:02X}")

    def get_info_text(self, code: int) -> str:
        return INFO_TEXT.get(code, f"Unknown INFO code 0x{code:02X}")

    def get_error_text(self, code: int) -> str:
        return ERROR_TEXT.get(code, f"Unknown ERROR code 0x{code:02X}")

    def publish_heartbeat(self, ok: bool) -> None:
        msg = SerialHeartbeat()
        msg.alive = ok
        msg.seq = self.heartbeat_sequence
        self.heartbeat_sequence = self.heartbeat_sequence + 1
        msg.stamp = self.get_clock().now().to_msg()
        self.heartbeat_pub.publish(msg)

    def heartbeat_check(self) -> None:
        if (not self.heartbeat_ok):
            self.publish_heartbeat(False)
        else:
            self.heartbeat_ok = False


def main(args=None):
    rclpy.init(args=args)

    Comm_Node = CommNode()

    # Try/Except here because ROS doesn't catch it as well on Python as on C++
    try:
        rclpy.spin(Comm_Node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        Comm_Node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
