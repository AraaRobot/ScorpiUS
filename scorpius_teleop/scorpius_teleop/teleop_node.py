import rclpy
from rclpy.node import Node
import math

from scorpius_main.msg import ServoAngles
from scorpius_main.msg import Joy

from .Vector2 import Vector2
from .HexapodAngles import HexapodAngles

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')

        # logic members
        self.start_angles = HexapodAngles() # start angles for the legs, used for smooth movement
        self.target_angles = HexapodAngles() # target angles for the legs, used for smooth movement  
        self.front_angle_offset = 30 # angle of the forward direction, degrees
        self.rear_angle_offset = -30 # angle of the backward direction, degrees 
        self.position_count = 60 # amount of positions in one whole movement
        self.leg_reach = 100 # leg reach, mm
        self.front_angle = 90 # front degrees in witch the hexapod goes directly in the wanted direction
        self.movement_state = 0 # 0 -> idle, 1 -> forward, 2 -> backward, 3 -> turn right, 4 -> turn left
        self.position_state = 0 # 0 -> neutral, 1 -> left neutral, 2 -> right down, 3 -> right neutral, 4 -> left down
        self.last_interpolation_time = self.get_clock().now() # time of the last interpolation, used for smooth movement

        # limits
        self.MAX_SPEED = self.position_count / 1 # maximum speed in position / second
        self.MIN_SPEED = self.position_count / 10 # minimum speed in position / second
        self.MAX_ABS_HORIZ_ANGLE = 45 # maximum absolute horizontal servo angle
        self.MAX_VERT_ANGLE = 90 # maximum vertical servo angle
        self.MIN_VERT_ANGLE = 0 # minimum vertical servo angle
        self.VERT_UP_ANGLE = 0 # vertical angle when the leg is up
        self.VERT_DOWN_ANGLE = 90 # vertical angle when the leg is down
        self.HORIZ_NEUTRAL_ANGLE = 0 # horizontal angle when the leg is in neutral position
        self.MAX_STEP = self.leg_reach * math.sqrt( math.pow(math.cos(self.MAX_ABS_HORIZ_ANGLE)-math.cos(self.front_angle-self.MAX_ABS_HORIZ_ANGLE), 2) + math.pow(math.sin(self.MAX_ABS_HORIZ_ANGLE)-math.sin(self.front_angle-self.MAX_ABS_HORIZ_ANGLE), 2) ) # mm
        self.MIN_STEP = 5 # mm
        self.STEP_CHANGE = 5 # mm

        # publisher/subscriber
        self.publisher_angles = self.create_publisher(ServoAngles, '/scorpius/teleop', 10)
        self.subscriber_input = self.create_subscription(Joy, '/scorpius/joy', self.subscriber_callback, 10)

        # callbacks
        logic_period = 0.5  # seconds
        self.timer_logic = self.create_timer(logic_period, self.logic_callback)

        # subscribe members
        self.input_vector = Vector2(0, 0)
        self.speed = self.MIN_SPEED # speed in degrees / second
        self.step = self.MIN_STEP # step of the hexapod in mm

        # publish members
        self.angles = HexapodAngles() # leg A is left from head, rest goes counterclockwise

    def subscriber_callback(self, msg):
        # read msg
        data : Joy = msg

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
            self.speed = data.joy_data[Joy.R2] * (self.MAX_SPEED - self.MIN_SPEED) + self.MIN_SPEED

        # update step
        if data.joy_data[Joy.CROSS_UP]:
            self.step += self.STEP_CHANGE
        elif data.joy_data[Joy.CROSS_DOWN]:
            self.step -= self.STEP_CHANGE
        if self.step < self.MIN_STEP:
            self.step = self.MIN_STEP
        if self.step > self.MAX_STEP:
            self.step = self.MAX_STEP

        # debug
        self.get_logger().debug(f"Received : {data.joy_data[Joy.JOYSTICK_LEFT_HORIZ]} "
                                f"{data.joy_data[Joy.JOYSTICK_LEFT_VERT]} "
                                f"{data.joy_data[Joy.R2]} "
                                f"{data.joy_data[Joy.CROSS_UP]} " 
                                f"{data.joy_data[Joy.CROSS_DOWN]}")
        
    def logic_callback(self):
        # update movement method
        self.update_movement_state()

        # angles calculations
        self.angles_calculations()

        # publish calculations
        self.publisher_callback()

    def publisher_callback(self):
        # send msg
        self.angles.clamp(self.MIN_VERT_ANGLE, self.MAX_VERT_ANGLE, -self.MAX_ABS_HORIZ_ANGLE, self.MAX_ABS_HORIZ_ANGLE)
        msg = self.angles.to_servo_angles_msg()
        self.publisher_angles.publish(msg)

        # debug
        # self.get_logger().debug(
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
        # debug
        msg = self.angles.to_servo_angles_msg()
        self.get_logger().debug(
            f"\n================ Angles ================\n"
            f"       vertical        horizontal\n"
            f"A :    {msg.vert_a:8.3f}      {msg.horiz_a:8.3f}\n"
            f"B :    {msg.vert_b:8.3f}      {msg.horiz_b:8.3f}\n"
            f"C :    {msg.vert_c:8.3f}      {msg.horiz_c:8.3f}\n"
            f"D :    {msg.vert_d:8.3f}      {msg.horiz_d:8.3f}\n"
            f"E :    {msg.vert_e:8.3f}      {msg.horiz_e:8.3f}\n"
            f"F :    {msg.vert_f:8.3f}      {msg.horiz_f:8.3f}\n"
            f"========================================"
        )

        # update target angles based on movement state
        if self.angles == self.target_angles:
            # update angles
            self.start_angles.set_from_hexapod_angles(self.target_angles)

            # match new state and update target angles
            state = (self.movement_state, self.position_state)
            half_angle_step = math.asin(self.step / (2 * self.leg_reach)) * 180 / math.pi # angle step for horizontal angles, degrees
            target_angle = self.input_vector.get_angle() - 90 # target angle for horizontal angles, degrees
            self.get_logger().debug(f"State: {state}, Target angle: {target_angle}")
            match state:
                case (0, _): # to idle
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
                                            self.HORIZ_NEUTRAL_ANGLE) # neutral position, 0
                    self.position_state = 0
                case (_, 0): # from idle
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
                case (1, 1): # forward, left neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE, 
                                            target_angle + half_angle_step - self.front_angle_offset, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle - half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            target_angle + half_angle_step - self.rear_angle_offset, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle + half_angle_step - self.rear_angle_offset, 
                                            self.VERT_DOWN_ANGLE, 
                                            target_angle - half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle + half_angle_step - self.front_angle_offset) # right down, 2
                    self.position_state = 2
                case (1, 2): # forward, right down
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
                case (1, 3): # forward, right neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE, 
                                            target_angle - half_angle_step - self.front_angle_offset, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle + half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            target_angle - half_angle_step - self.rear_angle_offset, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle - half_angle_step - self.rear_angle_offset, 
                                            self.VERT_DOWN_ANGLE, 
                                            target_angle + half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle - half_angle_step - self.front_angle_offset) # left down, 4
                    self.position_state = 4
                case (1, 4): # forward, left down
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
                case (2, 1): # backward, left neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE, 
                                            target_angle - half_angle_step - self.front_angle_offset, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle + half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            target_angle - half_angle_step - self.rear_angle_offset, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle - half_angle_step - self.rear_angle_offset, 
                                            self.VERT_DOWN_ANGLE, 
                                            target_angle + half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle - half_angle_step - self.front_angle_offset) # left down, 4
                    self.position_state = 4
                case (2, 2): # backward, right down
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
                case (2, 3): # backward, right neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE, 
                                            target_angle + half_angle_step - self.front_angle_offset, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle - half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            target_angle + half_angle_step - self.rear_angle_offset, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle + half_angle_step - self.rear_angle_offset, 
                                            self.VERT_DOWN_ANGLE, 
                                            target_angle - half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            target_angle + half_angle_step - self.front_angle_offset) # right down, 2
                    self.position_state = 2
                case (2, 4): # backward, left down
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
                case (3, 1): # turn right, left neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step) # right down, 2
                    self.position_state = 2
                case (3, 2): # turn right, right down
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
                case (3, 3): # turn right, right neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step) # left down, 4
                    self.position_state = 4
                case (3, 4): # turn right, left down
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
                case (4, 1): # turn left, left neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE + half_angle_step) # left down, 4
                    self.position_state = 4
                case (4, 2): # turn left, right down
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
                case (4, 3): # turn left, right neutral
                    self.target_angles.set(self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_DOWN_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step, 
                                            self.VERT_UP_ANGLE, 
                                            self.HORIZ_NEUTRAL_ANGLE - half_angle_step) # right down, 2
                    self.position_state = 2
                case (4, 4): # turn left, left down
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
                case _:
                    self.get_logger().error(f"Movement state: {state} not found.")
                    return
        
        # interpolate angles towards target angles based on speed
        if self.get_clock().now() - self.last_interpolation_time >= rclpy.duration.Duration(seconds=1/self.speed):
            # self.get_logger().debug("Interpolating angles.")
            self.angles.interpolate(self.start_angles, self.target_angles, 1 / self.position_count)
            self.last_interpolation_time = self.get_clock().now()

    def update_movement_state(self):
        # get normalized input
        normalized_input_vector = self.input_vector.normalized()

        # find movement state
        if normalized_input_vector.x == normalized_input_vector.y == 0:
            self.movement_state = 0
        else:
            angle = normalized_input_vector.get_angle() - 90   
            if abs(angle) <= self.front_angle / 2:
                self.movement_state = 1 # forward
            elif abs(angle) >= 180 - self.front_angle / 2:
                self.movement_state = 2 # backward
            elif angle < -self.front_angle / 2:
                self.movement_state = 3 # turn right
            elif angle > self.front_angle / 2:
                self.movement_state = 4 # turn left
            else:
                self.movement_state = 0
                self.get_logger().error("Movement state not found.")


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