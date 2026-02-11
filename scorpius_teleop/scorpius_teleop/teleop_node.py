import rclpy
from rclpy.node import Node
import math

from scorpius_main.msg import ServoAngles
from scorpius_main.msg import Inputs


class Vector2:
    def __init__(self, x:float = 0, y:float = 0):
        self.x = x
        self.y = y

    def get_magnitude(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y)
        
    def normalized(self):
        mag = self.get_magnitude()
        return Vector2(self.x / mag , self.y / mag)
    
    def get_angle(self) -> float:
        return math.degrees(math.atan2(self.y, self.x))
    
    @staticmethod
    def angle_between(fromVector, toVector) -> float:
        if not isinstance(fromVector, Vector2):
            raise TypeError(fromVector + " not a Vector2.")
        if not isinstance(toVector, Vector2):
            raise TypeError(fromVector + " not a Vector2.")
        dot = fromVector.x * toVector.x + fromVector.y * toVector.y
        return math.degrees(math.acos(dot / (fromVector.get_magnitude() * toVector.get_magnitude())))
    

class HexapodLegAngles:
    def __init__(self, verticalAngle:float=0, horizontalAngle:float=0):
        self.vAngle = verticalAngle
        self.hAngle = horizontalAngle

    def __eq__(self, other):
        return isinstance(other, HexapodLegAngles) and self.vAngle == other.vAngle and self.hAngle == other.hAngle
        

class HexapodAngles:
    def __init__(self, vA:float=0, hA:float=0, vB:float=0, hB:float=0, vC:float=0, hC:float=0, 
                 vD:float=0, hD:float=0, vE:float=0, hE:float=0, vF:float=0, hF:float=0):
        self.legA = HexapodLegAngles(vA, hA)
        self.legB = HexapodLegAngles(vB, hB)
        self.legC = HexapodLegAngles(vC, hC)
        self.legD = HexapodLegAngles(vD, hD)
        self.legE = HexapodLegAngles(vE, hE)
        self.legF = HexapodLegAngles(vF, hF)

    def __init__(self, angles:ServoAngles):
        self.legA = HexapodLegAngles(angles.vert_a, angles.horiz_a)
        self.legB = HexapodLegAngles(angles.vert_b, angles.horiz_b)
        self.legC = HexapodLegAngles(angles.vert_c, angles.horiz_c)
        self.legD = HexapodLegAngles(angles.vert_d, angles.horiz_d)
        self.legE = HexapodLegAngles(angles.vert_e, angles.horiz_e)
        self.legF = HexapodLegAngles(angles.vert_f, angles.vert_f)

    def to_servo_angles_msg(self) -> ServoAngles:
        angles = ServoAngles()
        angles.vert_a = self.legA.vAngle
        angles.vert_b = self.legB.vAngle
        angles.vert_c = self.legC.vAngle
        angles.vert_d = self.legD.vAngle
        angles.vert_e = self.legE.vAngle
        angles.vert_f = self.legF.vAngle
        angles.horiz_a = self.legA.hAngle
        angles.horiz_b = self.legB.hAngle
        angles.horiz_c = self.legC.hAngle
        angles.horiz_d = self.legD.hAngle
        angles.horiz_e = self.legE.hAngle
        angles.horiz_f = self.legF.hAngle
        return angles      
    
    def interpolate(self, start, target, ratio:float):
        if not isinstance(start, HexapodAngles):
            raise TypeError(start + " not a HexapodAngles.")
        if not isinstance(target, HexapodAngles):
            raise TypeError(target + " not a HexapodAngles.")
        if ratio < 0 or ratio > 1:
            raise ValueError("Ratio must be between 0 and 1.")
        self.legA.vAngle += math.min((target.legA.vAngle - start.legA.vAngle) * ratio, self.legA.vAngle - start.legA.vAngle)
        self.legA.hAngle += math.min((target.legA.hAngle - start.legA.hAngle) * ratio, self.legA.hAngle - start.legA.hAngle)
        self.legB.vAngle += math.min((target.legB.vAngle - start.legB.vAngle) * ratio, self.legB.vAngle - start.legB.vAngle)
        self.legB.hAngle += math.min((target.legB.hAngle - start.legB.hAngle) * ratio, self.legB.hAngle - start.legB.hAngle)
        self.legC.vAngle += math.min((target.legC.vAngle - start.legC.vAngle) * ratio, self.legC.vAngle - start.legC.vAngle)
        self.legC.hAngle += math.min((target.legC.hAngle - start.legC.hAngle) * ratio, self.legC.hAngle - start.legC.hAngle)
        self.legD.vAngle += math.min((target.legD.vAngle - start.legD.vAngle) * ratio, self.legD.vAngle - start.legD.vAngle)
        self.legD.hAngle += math.min((target.legD.hAngle - start.legD.hAngle) * ratio, self.legD.hAngle - start.legD.hAngle)
        self.legE.vAngle += math.min((target.legE.vAngle - start.legE.vAngle) * ratio, self.legE.vAngle - start.legE.vAngle)
        self.legE.hAngle += math.min((target.legE.hAngle - start.legE.hAngle) * ratio, self.legE.hAngle - start.legE.hAngle)
        self.legF.vAngle += math.min((target.legF.vAngle - start.legF.vAngle) * ratio, self.legF.vAngle - start.legF.vAngle)
        self.legF.hAngle += math.min((target.legF.hAngle - start.legF.hAngle) * ratio, self.legF.hAngle - start.legF.hAngle)

    def __eq__(self, other):
        return isinstance(other, HexapodAngles) and self.legA == other.legA and self.legB == other.legB and self.legC == other.legC and self.legD == other.legD and self.legE == other.legE and self.legF == other.legF
        

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')

        # publisher/subscriber
        self.publisher_angles = self.create_publisher(ServoAngles, '/scorpius/teleop', 10)
        self.subscriber_input = self.create_subscription(Inputs, '/scorpius/joy', self.subscriber_callback)

        # callbacks
        logic_period = 0.5  # seconds
        self.timer_logic = self.create_timer(logic_period, self.logic_callback)

        # subscribe members
        self.input_vector = Vector2()
        self.speed = 0 # speed in degrees / second
        self.step = 0 # step of the hexapod in mm

        # publish members
        self.angles = HexapodAngles() # leg A is left from head, rest goes counterclockwise

        # logic members
        self.start_angles = HexapodAngles() # start angles for the legs, used for smooth movement
        self.target_angles = HexapodAngles() # target angles for the legs, used for smooth movement  
        self.forward_angle = 0 # angle of the forward direction, degrees
        self.backward_angle = 0 # angle of the backward direction, degrees 
        self.position_count = 60 # amount of positions in one whole movement
        self.leg_reach = 100 # leg reach, mm
        self.front_angle = 90 # front degrees in witch the hexapod goes directly in the wanted direction
        self.input_dead_zone = 0.2 # minimum magnitude of the input vector
        self.movement_state = 0 # 0 -> idle, 1 -> forward, 2 -> backward, 3 -> turn right, 4 -> turn left
        self.position_state = 0 # 0 -> neutral, 1 -> left neutral, 2 -> right down, 3 -> right neutral, 4 -> left down

        # limits
        self.MAX_SPEED = 180 # maximum speed in degrees / second
        self.MIN_SPEED = 40 # minimum speed in degrees / second
        self.MAX_ABS_HORIZ_ANGLE = 45 # maximum absolute horizontal servo angle
        self.MAZ_VERT_ANGLE = 90 # maximum vertical servo angle
        self.MIN_VERT_ANGLE = 0 # minimum vertical servo angle
        self.VERT_UP_ANGLE = 0 # vertical angle when the leg is up
        self.VERT_DOWN_ANGLE = 90 # vertical angle when the leg is down
        self.HORIZ_NEUTRAL_ANGLE = 0 # horizontal angle when the leg is in neutral position
        self.MAX_STEP = self.leg_reach * math.sqrt( math.pow(math.cos(self.MAX_HORIZ_ANGLE)-math.cos(self.front_angle-self.MAX_HORIZ_ANGLE), 2) + math.pow(math.sin(self.MAX_HORIZ_ANGLE)-math.sin(self.front_angle-self.MAX_HORIZ_ANGLE), 2) ) # mm
        self.MIN_STEP = 5 # mm

    def subscriber_callback(self, msg):
        # read msg
        data = msg

        # update normalized input vector
        self.input_vector.x = data.x
        self.input_vector.y = data.y
        self.input_vector = self.input_vector.normalized()

        # update speed
        if data.speed >= 1:
            self.speed = self.MAX_SPEED
        elif data.speed <= 0:
            self.speed = self.MIN_SPEED
        else:
            self.speed = data.speed * (self.MAX_SPEED - self.MIN_SPEED) + self.MIN_SPEED

        # update step
        if data.step >= 1:
            self.step = self.MAX_STEP
        elif data.step <= 0:
            self.step = self.MIN_STEP
        else:
            self.step = data.step * (self.MAX_STEP - self.MIN_STEP) + self.MIN_STEP

        # debug
        self.get_logger().info(f"Received : {data.x} {data.y} {data.speed} {data.step}")
        
    def logic_callback(self):
        # update movement method
        self.update_movement_state()

        # angles calculations
        self.angles_calculations()

        # publish calculations
        self.publisher_callback()

    def publisher_callback(self):
        # send msg
        msg = self.angles.to_servo_angles_msg()
        self.publisher_.publish(msg)

        # debug
        self.get_logger().info(
            f"\n================ Publishing ================\n"
            f"       vertical        horizontal\n"
            f"A :    {msg.vertA:8.3f}      {msg.horizA:8.3f}\n"
            f"B :    {msg.vertB:8.3f}      {msg.horizB:8.3f}\n"
            f"C :    {msg.vertC:8.3f}      {msg.horizC:8.3f}\n"
            f"D :    {msg.vertD:8.3f}      {msg.horizD:8.3f}\n"
            f"E :    {msg.vertE:8.3f}      {msg.horizE:8.3f}\n"
            f"F :    {msg.vertF:8.3f}      {msg.horizF:8.3f}\n"
            f"=============================================="
        )
    
    def angles_calculations(self):
        # update target angles based on movement state
        if self.angles == self.target_angles:
            # update angles
            self.start_angles = self.angles

            # match state
            state = (self.movement_state, self.position_state)
            match state:
                case (0, _): # to idle
                    self.target_angles = HexapodAngles(self.VERT_DOWN_ANGLE, 
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
                    self.target_angles = HexapodAngles(self.VERT_DOWN_ANGLE, 
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
                    self.target_angles = HexapodAngles(90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0) # right down, 2
                    self.position_state = 2
                case (1, 2): # forward, right down
                    self.target_angles = HexapodAngles(self.VERT_UP_ANGLE, 
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
                    self.target_angles = HexapodAngles(90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0) # left down, 4
                    self.position_state = 4
                case (1, 4): # forward, left down
                    self.target_angles = HexapodAngles(self.VERT_DOWN_ANGLE, 
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
                    self.target_angles = HexapodAngles(90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0) # left down, 4
                    self.position_state = 4
                case (2, 2): # backward, right down
                    self.target_angles = HexapodAngles(self.VERT_DOWN_ANGLE, 
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
                    self.target_angles = HexapodAngles(90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0) # right down, 2
                    self.position_state = 2
                case (2, 4): # backward, left down
                    self.target_angles = HexapodAngles(self.VERT_UP_ANGLE, 
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
                    self.target_angles = HexapodAngles(90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0) # right down, 2
                    self.position_state = 2
                case (3, 2): # turn right, right down
                    self.target_angles = HexapodAngles(self.VERT_UP_ANGLE, 
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
                    self.target_angles = HexapodAngles(90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0) # left down, 4
                    self.position_state = 4
                case (3, 4): # turn right, left down
                    self.target_angles = HexapodAngles(self.VERT_DOWN_ANGLE, 
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
                    self.target_angles = HexapodAngles(90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0) # left down, 4
                    self.position_state = 4
                case (4, 2): # turn left, right down
                    self.target_angles = HexapodAngles(self.VERT_DOWN_ANGLE, 
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
                    self.target_angles = HexapodAngles(90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0) # right down, 2
                    self.position_state = 2
                case (4, 4): # turn left, left down
                    self.target_angles = HexapodAngles(self.VERT_UP_ANGLE, 
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
                    raise ValueError("Next movement not found.")
        
        # calculate next angles based on target angles
        self.angles.interpolate(self.start_angles, self.target_angles, 1 / self.position_count)

    def update_movement_state(self):
        # get normalized input
        normalized_input_vector = self.input_vector.normalized()

        # check dead zone
        if (normalized_input_vector.get_magnitude() < self.input_dead_zone):
            normalized_input_vector.x = normalized_input_vector.y = 0

        # find movement state
        if normalized_input_vector.x == normalized_input_vector.y == 0:
            self.movement_state = 0
        else:
            angle = normalized_input_vector.get_angle() - 90   
            if math.abs(angle) <= self.front_angle / 2:
                self.movement_state = 1 # forward
            elif math.abs(angle) >= 180 - self.front_angle / 2:
                self.movement_state = 2 # backward
            elif angle < -self.front_angle / 2:
                self.movement_state = 3 # turn right
            elif angle > self.front_angle / 2:
                self.movement_state = 4 # turn left
            else:
                raise ValueError("Movement state not found.")


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