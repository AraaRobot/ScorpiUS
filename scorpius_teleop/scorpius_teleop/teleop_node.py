import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math

from scorpius_main.msg import ServoAngles
from scorpius_main.msg import Joy
from scorpius_main.msg import Step
from scorpius_main.srv import ControllerState

from .Vector2 import Vector2
from .HexapodAngles import HexapodAngles


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')

        # logic members
        # start angles for the legs, used for smooth movement
        self.start_angles = HexapodAngles()
        # target angles for the legs, used for smooth movement
        self.target_angles = HexapodAngles()
        self.leg_angle_offset = 30  # angle offset for the legs, degrees
        self.position_count = 40  # amount of positions in one whole movement
        self.leg_reach = 100  # leg reach, mm
        # front degrees in witch the hexapod goes directly in the wanted direction
        self.front_angle = 90
        # 0 -> idle, 1 -> forward, 2 -> backward, 3 -> turn right, 4 -> turn left
        self.movement_state = 0
        # 0 -> neutral, 1 -> left neutral, 2 -> right down, 3 -> right neutral, 4 -> left down
        self.position_state = 0
        self.output_counter = 0  # counter for the output callback

        # limits
        # maximum speed in positions / second
        self.MAX_SPEED = self.position_count / 0.3
        self.MIN_SPEED = self.position_count / 10  # minimum speed in positions / second
        self.MAX_ABS_HORIZ_ANGLE = 45  # maximum absolute horizontal servo angle
        self.MAX_VERT_ANGLE = 90  # maximum vertical servo angle
        self.MIN_VERT_ANGLE = 0  # minimum vertical servo angle
        self.VERT_UP_ANGLE = 0  # vertical angle when the leg is up
        self.VERT_DOWN_ANGLE = 90  # vertical angle when the leg is down
        self.HORIZ_NEUTRAL_ANGLE = 0  # horizontal angle when the leg is in neutral position
        self.MAX_STEP = 2 * self.leg_reach  # mm
        self.MIN_STEP = 5  # mm
        self.STEP_CHANGE = 5  # mm

        # publisher/subscriber
        self.publisher_angles = self.create_publisher(
            ServoAngles, '/scorpius/teleop', 10)
        self.publisher_step = self.create_publisher(
            Step, '/scorpius/teleop/step', 10)
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
        self.last_home_time = self.get_clock().now()
        self.home_cooldown = Duration(seconds=2.0)

        # callbacks
        self.OUTPUT_RATE = 160  # rate of the output callback in Hz
        # dummy timer to set the period of the subscriber callback
        self.input_timer = self.create_timer(0.5, self.input_callback)
        self.output_timer = self.create_timer(
            1/self.OUTPUT_RATE, self.output_callback)
        self.step_output_timer = self.create_timer(
            0.1, self.step_output_callback)

        # publish members
        self.angles = HexapodAngles()  # leg A is left from head, rest goes counterclockwise

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

        if data.joy_data[Joy.HOME]:
            now = self.get_clock().now()
            if (now - self.last_home_time) > self.home_cooldown:
                self.last_home_time = now
                if self.state_controller_client.service_is_ready():
                    request = ControllerState.Request()
                    request.state = ControllerState.Request.HOME
                    self.state_controller_client.call_async(request)
                else:
                    self.get_logger().warn("State controller service was unavailable, ignoring HOME command",
                                           throttle_duration_sec=30)

        # debug
        # self.get_logger().debug(f"Received : {data.joy_data[Joy.JOYSTICK_LEFT_HORIZ]} "
        #                         f"{data.joy_data[Joy.JOYSTICK_LEFT_VERT]} "
        #                         f"{data.joy_data[Joy.R2]} "
        #                         f"{data.joy_data[Joy.CROSS_UP]} "
        #                         f"{data.joy_data[Joy.CROSS_DOWN]}")

    def input_callback(self):
        # update movement method
        self.update_movement_state()

    def output_callback(self):
        # angles calculations
        self.angles_calculations()

        # publish calculations
        self.publisher_callback()

    def step_output_callback(self):
        msg = Step()
        msg.step = self.step
        self.publisher_step.publish(msg)

    def publisher_callback(self):
        # send msg
        # self.angles.clamp(self.MIN_VERT_ANGLE, self.MAX_VERT_ANGLE, -self.MAX_ABS_HORIZ_ANGLE, self.MAX_ABS_HORIZ_ANGLE)
        msg = self.angles.to_servo_angles_msg()
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
                case (4, 1) | (3, 3):  # turn right, left neutral or turn left, right neutral
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
                                           self.HORIZ_NEUTRAL_ANGLE - half_angle_step)  # right down, 2
                    self.position_state = 2
                case (4, 2) | (3, 4):  # turn right, right down or turn left, left down
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
                                           self.HORIZ_NEUTRAL_ANGLE)  # right neutral, 3
                    self.position_state = 3
                case (4, 3) | (3, 1):  # turn right, right neutral or turn left, left neutral
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
                                           self.HORIZ_NEUTRAL_ANGLE + half_angle_step)  # left down, 4
                    self.position_state = 4
                # turn right, left down or turn left, right down or from idle
                case (4, 4) | (3, 2) | (3, 0) | (4, 0):
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
                                           self.HORIZ_NEUTRAL_ANGLE)  # left neutral, 1
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
        # get normalized input
        normalized_input_vector = self.input_vector.normalized()

        # find movement state
        if normalized_input_vector.x == normalized_input_vector.y == 0:
            self.movement_state = 0
        else:
            angle = normalized_input_vector.get_angle() - 90
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
