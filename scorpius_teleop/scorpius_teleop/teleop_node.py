import rclpy
from rclpy.node import Node
import math

from scorpius_main.msg import servo_angles
from scorpius_main.msg import inputs


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
        return math.atan2(self.y, self.x)
    
    @staticmethod
    def angle_between(fromVector, toVector) -> float:
        if not isinstance(fromVector, Vector2):
            raise TypeError(fromVector + " not a Vector2.")
        if not isinstance(toVector, Vector2):
            raise TypeError(fromVector + " not a Vector2.")
        dot = fromVector.x * toVector.x + fromVector.y * toVector.y
        return math.acos(dot / (fromVector.get_magnitude() * toVector.get_magnitude()))
    
class Vector12:
    def __init__(self, vA:float=0, hA:float=0, vB:float=0, hB:float=0, vC:float=0, hC:float=0, 
                 vD:float=0, hD:float=0, vE:float=0, hE:float=0, vF:float=0, hF:float=0):
        self.vA = vA
        self.hA = hA
        self.vB = vB
        self.hB = hB
        self.vC = vC
        self.hC = hC
        self.vD = vD
        self.hD = hD
        self.vE = vE
        self.hE = hE
        self.vF = vF
        self.hF = hF

    def __init__(self, angles:servo_angles):
        self.vA = angles.vertA
        self.hA = angles.horizA
        self.vB = angles.vertB
        self.hB = angles.horizB
        self.vC = angles.vertC
        self.hC = angles.horizC
        self.vD = angles.vertD
        self.hD = angles.horizD
        self.vE = angles.vertE
        self.hE = angles.horizE
        self.vF = angles.vertF
        self.hF = angles.horizF

    def to_servo_angles(self) -> servo_angles:
        angles = servo_angles()
        angles.vertA = self.vA
        angles.vertB = self.vB
        angles.vertC = self.vC
        angles.vertD = self.vD
        angles.vertE = self.vE
        angles.vertF = self.vF
        angles.horizA = self.hA
        angles.horizB = self.hB
        angles.horizC = self.hC
        angles.horizD = self.hD
        angles.horizE = self.hE
        angles.horizF = self.hF
        return angles
        
        
class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        # publisher/subscriber
        self.publisher_angles = self.create_publisher(servo_angles, '/scorpius/teleop', 10)
        self.subscriber_input = self.create_subscription(inputs, '/scorpius/joy', self.subscriber_callback)
        # callbacks
        logic_period = 0.5  # seconds
        self.timer_logic = self.create_timer(logic_period, self.logic_callback)
        # subscribe members
        self.speed = 0 # speed, between 0 (min) and 1 (max)
        self.input_vector = Vector2()
        self.step = 0 # step of the hexapod in mm
        # publish members
        self.angles = Vector12() # leg A is 
        self.last_angles = Vector12()
        # parameter members
        self.leg_length = 0 # leg reach
        self.MAX_ANGLE = 45 # maximum servo angle
        self.MIN_ANGLE = -45 # minimum servo angle

    def subscriber_callback(self, msg):
        # read msg
        data = msg
        self.input_vector.x = data.x
        self.input_vector.y = data.y
        self.speed = data.speed
        self.step = data.step
        # debug
        self.get_logger().info(f"Received : {data.x} {data.y} {data.speed} {data.pas}")
        
    def logic_callback(self):
        # calculations
        self.angles_calculations()

        # publish calculations
        self.publisher_callback()

    def publisher_callback(self):
        # send msg
        msg = self.angles.to_servo_angles()
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
        # 
        self.angles = self.last_angles
        # update last angles
        self.last_angles = self.angles


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