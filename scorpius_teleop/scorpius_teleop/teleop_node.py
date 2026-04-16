import rclpy
from functools import partial
from rclpy.node import Node
from rclpy.duration import Duration
import math

from scorpius_main.msg import ServoAngles
from scorpius_main.msg import Joy
from scorpius_main.msg import Step
from scorpius_main.msg import StateChanged
from scorpius_main.srv import ControllerState

from .Vector2 import Vector2
from .HexapodAngles import HexapodAngles


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')

        # publisher/subscriber
        self.publisher_angles = self.create_publisher(
            ServoAngles, '/scorpius/teleop', 10)
        self.publisher_step = self.create_publisher(
            Step, '/scorpius/teleop/step', 10)
        self.publisher_state = self.create_publisher(
            StateChanged, '/scorpius/state_changed', 1
        )
        self.subscriber_input = self.create_subscription(
            Joy, '/scorpius/joy', self.subscriber_callback, 10)

        # subscribe members
        self.input_vector = Vector2(0, 0)
        self.speed = self.MIN_SPEED  # speed in positions / second
        self.step = self.MAX_STEP / 2  # step of the hexapod in mm

        # client
        self.state_controller_client = self.create_client(
            ControllerState, '/scorpius/state_controller')
        if not self.state_controller_client.wait_for_service(1):
            self.get_logger().warning("State controller service is not online")

        # client members
        self.last_state_change_time = self.get_clock().now()
        self.state_change_cooldown = Duration(seconds=2.0)

        # callbacks
        self.OUTPUT_RATE: int = 160  # rate of the output callback in Hz
        self.INPUT_RATE: int = 20 # rate of the read input callback in Hz
        self.input_timer = self.create_timer(1/self.INPUT_RATE, self.input_callback)
        self.output_timer = self.create_timer(
            1/self.OUTPUT_RATE, self.output_callback)
        self.step_output_timer = self.create_timer(
            0.1, self.step_output_callback)

        # publish members
        self.angles: HexapodAngles = HexapodAngles() # leg A is left from head, rest goes counterclockwise

        # logic members
        self.start_angles: HexapodAngles = HexapodAngles() # start angles for the legs, used for smooth movement
        self.target_angles: HexapodAngles = HexapodAngles() # target angles for the legs, used for smooth movement  
        self.leg_angle_offset: float = 30 # angle offset for the legs, degrees
        self.position_count: int = 32 # amount of positions in one whole movement
        self.leg_reach: float = 100 # leg reach, mm
        self.front_angle: float = 90 # front degrees in which the hexapod goes directly in the wanted direction
        self.movement_state: int = 0 # 0 -> idle, 1 -> forward, 2 -> backward, 3 -> turn right, 4 -> turn left
        self.position_state: int = 0 # 0 -> neutral, 1 -> left neutral, 2 -> right down, 3 -> right neutral, 4 -> left down
        self.output_counter: int = 0 # counter for the output callback

        # limits
        self.MAX_SPEED: float = self.position_count / 0.2 # maximum speed in positions / second
        self.MIN_SPEED: float = self.position_count / 2 # minimum speed in positions / second
        self.MAX_ABS_HORIZ_ANGLE: float = 45 # maximum absolute horizontal servo angle
        self.MAX_VERT_ANGLE: float = 90 # maximum vertical servo angle
        self.MIN_VERT_ANGLE: float = -90 # minimum vertical servo angle
        self.VERT_UP_ANGLE: float = 0 # vertical angle when the leg is up
        self.VERT_DOWN_ANGLE: float = -90 # vertical angle when the leg is down
        self.HORIZ_NEUTRAL_ANGLE: float = 0 # horizontal angle when the leg is in neutral position
        self.MAX_STEP: float = 2 * self.leg_reach # mm
        self.MIN_STEP: float = 5 # mm
        self.STEP_CHANGE: float = 5 # mm
        self.HEIGHT_CHANGE: float = 120 # height angle change per input callback, degrees / second
        self.TAIL_UP_ANGLE: float = 45 # maximum tail angle, degrees
        self.TAIL_DOWN_ANGLE: float = -45 # minimum tail angle, degrees

        # subscribe members
        self.input_vector: Vector2 = Vector2(0, 0)
        self.speed: float = self.MIN_SPEED # speed in positions / second
        self.step: float = self.MAX_STEP / 2 # step of the hexapod in mm
        self.left_height: float = 1 # multiplier for the left servo vertical angles
        self.right_height: float = 1 # multiplier for the right servo vertical angles
        self.tail_angle: float = 0 # angle of the tail servo, degrees

    def subscriber_callback(self, msg: Joy) -> None:
        # read msg
        data: Joy = msg

        # update normalized input vector
        self.input_vector.x = data.joy_data[Joy.JOYSTICK_LEFT_HORIZ]
        self.input_vector.y = data.joy_data[Joy.JOYSTICK_LEFT_VERT]
        self.input_vector = self.input_vector.normalized()

        # update speed
        if data.joy_data[Joy.R2] >= 1:
            self.speed = self.MAX_SPEED
        elif data.joy_data[Joy.R2] <= 0:
            self.speed = self.MIN_SPEED
        else:
            self.speed = data.joy_data[Joy.R2] * \
                (self.MAX_SPEED - self.MIN_SPEED) + self.MIN_SPEED

        # update step
        if data.joy_data[Joy.CROSS_UP]:
            self.step += self.STEP_CHANGE
        elif data.joy_data[Joy.CROSS_DOWN]:
            self.step -= self.STEP_CHANGE
        if self.step < self.MIN_STEP:
            self.step = self.MIN_STEP
        if self.step > self.MAX_STEP:
            self.step = self.MAX_STEP

        # update height
        if data.joy_data[Joy.R3]:
            self.left_height = 1
            self.right_height = 1
        else:
            height_input_vector = Vector2(data.joy_data[Joy.JOYSTICK_RIGHT_HORIZ], data.joy_data[Joy.JOYSTICK_RIGHT_VERT])
            if not math.isclose(height_input_vector.get_magnitude(), 0, abs_tol=1e-5):
                # apply height change
                angle = height_input_vector.normalized().get_angle()
                angle_change = self.HEIGHT_CHANGE / abs(self.VERT_UP_ANGLE - self.VERT_DOWN_ANGLE) / self.INPUT_RATE
                if abs(angle) < 60: # right
                    if angle < 0: # right down
                        self.right_height -= angle_change
                    else: # right up
                        self.right_height += angle_change
                elif abs(angle) <= 120: # middle
                    if angle < 0: # middle down
                        self.left_height = self.right_height = min(self.left_height, self.right_height) - angle_change
                    else: # middle up
                        self.left_height = self.right_height = max(self.left_height, self.right_height) + angle_change
                elif angle <= 180: # left
                    if angle < 0: # left down
                        self.left_height -= angle_change
                    else: # left up
                        self.left_height += angle_change
                # Clamp height values
                if self.left_height > 1:
                    self.left_height = 1
                if self.left_height < 0:
                    self.left_height = 0
                if self.right_height > 1:
                    self.right_height = 1
                if self.right_height < 0:
                    self.right_height = 0

        # update tail angle
        if data.joy_data[Joy.L2] >= 1:
            self.tail_angle = self.TAIL_DOWN_ANGLE
        elif data.joy_data[Joy.L2] <= 0:
            self.tail_angle = self.TAIL_UP_ANGLE
        else:
            self.tail_angle = (1 - data.joy_data[Joy.L2]) * (self.TAIL_UP_ANGLE - self.TAIL_DOWN_ANGLE) + self.TAIL_DOWN_ANGLE
        
        # state changes
        if data.joy_data[Joy.HOME]: # Home takes priority
            now = self.get_clock().now()
            if (now - self.last_state_change_time) > self.state_change_cooldown:
                self.last_state_change_time = now
                self.send_state_change_request(ControllerState.Request.HOME)
        elif data.joy_data[Joy.OPTS]: # Running priority 2
            now = self.get_clock().now()
            if (now - self.last_state_change_time) > self.state_change_cooldown:
                self.last_state_change_time = now
                self.send_state_change_request(ControllerState.Request.RUNNING)
        elif data.joy_data[Joy.SHARE]:
            now = self.get_clock().now()
            if (now - self.last_state_change_time) > self.state_change_cooldown:
                self.last_state_change_time = now
                self.send_state_change_request(ControllerState.Request.REBOOT)

    def input_callback(self):
        # update movement method
        self.update_movement_state()

    def output_callback(self):
        # angles calculations
        self.angles_calculations()

        # publish calculations
        self.publisher_callback()

    def step_output_callback(self):
        # publish step
        msg = Step()
        msg.step = self.step
        self.publisher_step.publish(msg)

    def publisher_callback(self):
        # create msg
        #self.angles.clamp(self.MIN_VERT_ANGLE, self.MAX_VERT_ANGLE, -self.MAX_ABS_HORIZ_ANGLE, self.MAX_ABS_HORIZ_ANGLE)
        msg = ServoAngles()
        msg = self.angles.to_servo_angles_msg(msg)

        # apply height multiplier
        msg.vert_a *= self.left_height
        msg.vert_b *= self.left_height
        msg.vert_c *= self.left_height
        msg.vert_d *= self.right_height
        msg.vert_e *= self.right_height
        msg.vert_f *= self.right_height

        # add tail angle
        msg.tail_angle = float(self.tail_angle)

        # publish servo angles msg
        self.publisher_angles.publish(msg)

        # debug
        # self.get_logger().info(
        #     f"\n================ Publishing ================\n"
        #     f"       vertical        horizontal\n"
        #     f"A :    {msg.vert_a:8.3f}      {msg.horiz_a:8.3f}\n"
        #     f"B :    {msg.vert_b:8.3f}      {msg.horiz_b:8.3f}\n"
        #     f"C :    {msg.vert_c:8.3f}      {msg.horiz_c:8.3f}\n"
        #     f"D :    {msg.vert_d:8.3f}      {msg.horiz_d:8.3f}\n"
        #     f"E :    {msg.vert_e:8.3f}      {msg.horiz_e:8.3f}\n"
        #     f"F :    {msg.vert_f:8.3f}      {msg.horiz_f:8.3f}\n"
        #     f"=============================================="
        # )

    def send_state_change_request(self, requested_state: int) -> None:
        if not self.state_controller_client.service_is_ready():
            self.get_logger().warn(
                "State controller service was unavailable, ignoring state change request",
                throttle_duration_sec=30)
            return

        request = ControllerState.Request()
        request.state = requested_state
        future = self.state_controller_client.call_async(request)
        future.add_done_callback(partial(self.state_change_response_callback, requested_state))

    def state_change_response_callback(self, requested_state: int, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"State controller service call failed: {exc}")
            return

        if response.success:
            msg = StateChanged()
            msg.state = requested_state
            self.publisher_state.publish(msg)
            self.get_logger().info(f"Published successful state change: {requested_state}")
        else:
            self.get_logger().warn(
                f"State controller rejected state change {requested_state}: {response.message}")

    def angles_calculations(self):
        # update target angles based on movement state
        if self.angles == self.target_angles:
            # update angles
            self.start_angles.set_from_hexapod_angles(self.target_angles)

            # create movement state
            state = (self.movement_state, self.position_state)

            # update target angle and half angle step for horizontal angles, degrees
            target_angle = self.input_vector.get_angle()
            if target_angle > 0:
                target_angle -= 90
            elif target_angle < 0:
                target_angle += 90
            # angle step for horizontal angles, degrees
            half_angle_step = math.degrees(
                math.asin(self.step / (2 * self.leg_reach)))

            # match new state and
            match state:
                case (0, _):  # to idle
                    self.target_angles.set(self.VERT_DOWN_ANGLE,
                                           self.HORIZ_NEUTRAL_ANGLE,
                                           self.VERT_DOWN_ANGLE,
                                           self.HORIZ_NEUTRAL_ANGLE,
                                           self.VERT_DOWN_ANGLE,
                                           self.HORIZ_NEUTRAL_ANGLE,
                                           self.VERT_DOWN_ANGLE,
                                           self.HORIZ_NEUTRAL_ANGLE,
                                           self.VERT_DOWN_ANGLE,
                                           self.HORIZ_NEUTRAL_ANGLE,
                                           self.VERT_DOWN_ANGLE,
                                           self.HORIZ_NEUTRAL_ANGLE)  # neutral position, 0
                    self.position_state = 0

                # forward / backward states
                case (1, 1) | (2, 3):  # forward, left neutral or backward, right neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE,
                                           target_angle + half_angle_step + self.leg_angle_offset,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle - half_angle_step,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle + half_angle_step - self.leg_angle_offset,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle + half_angle_step + self.leg_angle_offset,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle - half_angle_step,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle + half_angle_step - self.leg_angle_offset)  # right down, 2
                    self.position_state = 2
                case (1, 2) | (2, 4):  # forward, right down or backward, left down
                    self.target_angles.set(self.VERT_UP_ANGLE,
                                           target_angle + self.leg_angle_offset,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle,
                                           self.VERT_UP_ANGLE,
                                           target_angle - self.leg_angle_offset,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle + self.leg_angle_offset,
                                           self.VERT_UP_ANGLE,
                                           target_angle,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle - self.leg_angle_offset)  # right neutral, 3
                    self.position_state = 3
                case (1, 3) | (2, 1):  # forward, right neutral or backward, left neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE,
                                           target_angle - half_angle_step + self.leg_angle_offset,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle + half_angle_step,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle - half_angle_step - self.leg_angle_offset,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle - half_angle_step + self.leg_angle_offset,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle + half_angle_step,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle - half_angle_step - self.leg_angle_offset)  # left down, 4
                    self.position_state = 4
                # forward, left down or backward, right down or from idle
                case (1, 4) | (2, 2) | (1, 0) | (2, 0):
                    self.target_angles.set(self.VERT_DOWN_ANGLE,
                                           target_angle + self.leg_angle_offset,
                                           self.VERT_UP_ANGLE,
                                           target_angle,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle - self.leg_angle_offset,
                                           self.VERT_UP_ANGLE,
                                           target_angle + self.leg_angle_offset,
                                           self.VERT_DOWN_ANGLE,
                                           target_angle,
                                           self.VERT_UP_ANGLE,
                                           target_angle - self.leg_angle_offset)  # left neutral, 1
                    self.position_state = 1

                # turn right / turn left states
                case (3, 1) | (4, 3): # turn right, left neutral or turn left, right neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step) # right down, 2
                    self.position_state = 2
                case (3, 2) | (4, 4): # turn right, right down or turn left, left down
                    self.target_angles.set(self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE) # right neutral, 3  
                    self.position_state = 3
                case (3, 3) | (4, 1): # turn right, right neutral or turn left, left neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step) # left down, 4
                    self.position_state = 4
                case (3, 4) | (4, 2) | (3, 0) | (4, 0): # turn right, left down or turn left, right down or from idle
                    self.target_angles.set(self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE) # left neutral, 1
                    self.position_state = 1

                # default case
                case _:
                    self.get_logger().error(
                        f"Movement state: {state} not found.")
                    return
            # self.get_logger().info(f"Movement state: {self.movement_state}, new position state: {self.position_state}")
            # self.get_logger().info(f"Half angle step: {half_angle_step} degrees")
            # msg = self.target_angles.to_servo_angles_msg()
            # self.get_logger().info(f"\n============ Target angles for {self.position_state} ============\n"
            #             f"       vertical        horizontal\n"
            #             f"A :    {msg.vert_a:8.3f}      {msg.horiz_a:8.3f}\n"
            #             f"B :    {msg.vert_b:8.3f}      {msg.horiz_b:8.3f}\n"
            #             f"C :    {msg.vert_c:8.3f}      {msg.horiz_c:8.3f}\n"
            #             f"D :    {msg.vert_d:8.3f}      {msg.horiz_d:8.3f}\n"
            #             f"E :    {msg.vert_e:8.3f}      {msg.horiz_e:8.3f}\n"
            #             f"F :    {msg.vert_f:8.3f}      {msg.horiz_f:8.3f}\n"
            #             f"==============================================")

        # returning to neutral position, set speed to max
        if self.movement_state == 0:
            self.speed = self.MAX_SPEED

        # interpolate angles towards target angles based on speed
        if self.output_counter * 1/self.OUTPUT_RATE >= 1/self.speed:
            # self.get_logger().info("Updating angles to next position.")
            self.angles.interpolate(
                self.start_angles, self.target_angles, 1 / self.position_count)

            # Debug
            # msg = self.angles.to_servo_angles_msg()
            # self.get_logger().info(
            #     f"\n================ Angles ================\n"
            #     f"       vertical        horizontal\n"
            #     f"A :    {msg.vert_a:8.3f}      {msg.horiz_a:8.3f}\n"
            #     f"B :    {msg.vert_b:8.3f}      {msg.horiz_b:8.3f}\n"
            #     f"C :    {msg.vert_c:8.3f}      {msg.horiz_c:8.3f}\n"
            #     f"D :    {msg.vert_d:8.3f}      {msg.horiz_d:8.3f}\n"
            #     f"E :    {msg.vert_e:8.3f}      {msg.horiz_e:8.3f}\n"
            #     f"F :    {msg.vert_f:8.3f}      {msg.horiz_f:8.3f}\n"
            #     f"========================================"
            # )
            # msg = self.target_angles.to_servo_angles_msg()
            # self.get_logger().info(
            #     f"\n============= Target angles =============\n"
            #     f"       vertical        horizontal\n"
            #     f"A :    {msg.vert_a:8.3f}      {msg.horiz_a:8.3f}\n"
            #     f"B :    {msg.vert_b:8.3f}      {msg.horiz_b:8.3f}\n"
            #     f"C :    {msg.vert_c:8.3f}      {msg.horiz_c:8.3f}\n"
            #     f"D :    {msg.vert_d:8.3f}      {msg.horiz_d:8.3f}\n"
            #     f"E :    {msg.vert_e:8.3f}      {msg.horiz_e:8.3f}\n"
            #     f"F :    {msg.vert_f:8.3f}      {msg.horiz_f:8.3f}\n"
            #     f"========================================="
            # )

            self.output_counter = 0
        else:
            self.output_counter += 1

    def update_movement_state(self):
        # find movement state
        if math.isclose(self.input_vector.get_magnitude(), 0, abs_tol=1e-5):
            self.movement_state = 0
        else:
            angle = self.input_vector.normalized().get_angle() - 90   
            if abs(angle) <= self.front_angle / 2:
                self.movement_state = 1  # forward
            elif abs(angle) >= 180 - self.front_angle / 2:
                self.movement_state = 2  # backward
            elif angle < -self.front_angle / 2:
                self.movement_state = 3  # turn right
            elif angle > self.front_angle / 2:
                self.movement_state = 4  # turn left
            else:
                self.movement_state = 0
                self.get_logger().error("Movement state not found.")

        # self.get_logger().info(f"Mouvement state: {self.movement_state}")


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
