# Class used to store hexapod leg position angles.
# By SimonP-4

import math
from scorpius_main.msg import ServoAngles

class HexapodLegAngles:
    def __init__(self, verticalAngle:float=0, horizontalAngle:float=0):
        self.vAngle = verticalAngle
        self.hAngle = horizontalAngle
        self.epsilon = 1e-6 # Epsilon based float comparison

    def __eq__(self, other):
        return isinstance(other, HexapodLegAngles) and math.isclose(self.vAngle, other.vAngle, abs_tol=self.epsilon) and math.isclose(self.hAngle, other.hAngle, abs_tol=self.epsilon)
        

class HexapodAngles:
    def __init__(self):
        self.legA = HexapodLegAngles(0, 0)
        self.legB = HexapodLegAngles(0, 0)
        self.legC = HexapodLegAngles(0, 0)
        self.legD = HexapodLegAngles(0, 0)
        self.legE = HexapodLegAngles(0, 0)
        self.legF = HexapodLegAngles(0, 0)

        self.epsilon = 1e-6

    def set(self, vA:float=0, hA:float=0, vB:float=0, hB:float=0, vC:float=0, hC:float=0, 
                   vD:float=0, hD:float=0, vE:float=0, hE:float=0, vF:float=0, hF:float=0):
        self.legA = HexapodLegAngles(vA, hA)
        self.legB = HexapodLegAngles(vB, hB)
        self.legC = HexapodLegAngles(vC, hC)
        self.legD = HexapodLegAngles(vD, hD)
        self.legE = HexapodLegAngles(vE, hE)
        self.legF = HexapodLegAngles(vF, hF)

    def set_from_hexapod_angles(self, angles):
        if not isinstance(angles, HexapodAngles):
            raise TypeError(angles + " not a HexapodAngles.")
        self.legA.vAngle = angles.legA.vAngle
        self.legA.hAngle = angles.legA.hAngle
        self.legB.vAngle = angles.legB.vAngle
        self.legB.hAngle = angles.legB.hAngle
        self.legC.vAngle = angles.legC.vAngle
        self.legC.hAngle = angles.legC.hAngle
        self.legD.vAngle = angles.legD.vAngle
        self.legD.hAngle = angles.legD.hAngle
        self.legE.vAngle = angles.legE.vAngle
        self.legE.hAngle = angles.legE.hAngle
        self.legF.vAngle = angles.legF.vAngle
        self.legF.hAngle = angles.legF.hAngle

    def clamp(self, minVertAngle:float, maxVertAngle:float, minHorizAngle:float, maxHorizAngle:float):
        self.legA.vAngle = max(minVertAngle, min(maxVertAngle, self.legA.vAngle))
        self.legA.hAngle = max(minHorizAngle, min(maxHorizAngle, self.legA.hAngle))
        self.legB.vAngle = max(minVertAngle, min(maxVertAngle, self.legB.vAngle))
        self.legB.hAngle = max(minHorizAngle, min(maxHorizAngle, self.legB.hAngle))
        self.legC.vAngle = max(minVertAngle, min(maxVertAngle, self.legC.vAngle))
        self.legC.hAngle = max(minHorizAngle, min(maxHorizAngle, self.legC.hAngle))
        self.legD.vAngle = max(minVertAngle, min(maxVertAngle, self.legD.vAngle))
        self.legD.hAngle = max(minHorizAngle, min(maxHorizAngle, self.legD.hAngle))
        self.legE.vAngle = max(minVertAngle, min(maxVertAngle, self.legE.vAngle))
        self.legE.hAngle = max(minHorizAngle, min(maxHorizAngle, self.legE.hAngle))
        self.legF.vAngle = max(minVertAngle, min(maxVertAngle, self.legF.vAngle))
        self.legF.hAngle = max(minHorizAngle, min(maxHorizAngle, self.legF.hAngle))

    def to_servo_angles_msg(self) -> ServoAngles:
        angles = ServoAngles()
        angles.vert_a = float(self.legA.vAngle)
        angles.vert_b = float(self.legB.vAngle)
        angles.vert_c = float(self.legC.vAngle)
        angles.vert_d = float(self.legD.vAngle)
        angles.vert_e = float(self.legE.vAngle)
        angles.vert_f = float(self.legF.vAngle)
        angles.horiz_a = float(self.legA.hAngle)
        angles.horiz_b = float(self.legB.hAngle)
        angles.horiz_c = float(self.legC.hAngle)
        angles.horiz_d = float(self.legD.hAngle)
        angles.horiz_e = float(self.legE.hAngle)
        angles.horiz_f = float(self.legF.hAngle)
        return angles      
    
    def interpolate(self, start, target, ratio:float):
        if not isinstance(start, HexapodAngles):
            raise TypeError(start + " not a HexapodAngles.")
        if not isinstance(target, HexapodAngles):
            raise TypeError(target + " not a HexapodAngles.")
        if ratio <= 0 or ratio > 1:
            raise ValueError("Ratio must be between 0 and 1. 0 is exclusive.")

        def next_angle(start_angle : float, target_angle : float, current_angle : float, exponent: float = 2):
            if math.isclose(start_angle, target_angle, abs_tol=self.epsilon):
                return target_angle
            k = max(target_angle, start_angle, key=abs)
            h = 1/ratio * (target_angle != 0)
            last_ratio = math.pow(abs((current_angle - k) / (ratio * abs(target_angle - start_angle))), 1/exponent) + h * ratio
            return math.copysign(k, abs(target_angle - start_angle) * math.pow(ratio, exponent) * math.pow(last_ratio + ratio - h, exponent)) + k
        
        self.legA.vAngle += min(next_angle(start.legA.vAngle, target.legA.vAngle, self.legA.vAngle) - self.legA.vAngle, self.legA.vAngle - start.legA.vAngle)
        self.legA.hAngle += min((target.legA.hAngle - start.legA.hAngle) * ratio, self.legA.hAngle - start.legA.hAngle)
        self.legB.vAngle += min(next_angle(start.legB.vAngle, target.legB.vAngle, self.legB.vAngle) - self.legB.vAngle, self.legB.vAngle - start.legB.vAngle)
        self.legB.hAngle += min((target.legB.hAngle - start.legB.hAngle) * ratio, self.legB.hAngle - start.legB.hAngle)
        self.legC.vAngle += min(next_angle(start.legC.vAngle, target.legC.vAngle, self.legC.vAngle) - self.legC.vAngle, self.legC.vAngle - start.legC.vAngle)
        self.legC.hAngle += min((target.legC.hAngle - start.legC.hAngle) * ratio, self.legC.hAngle - start.legC.hAngle)
        self.legD.vAngle += min(next_angle(start.legD.vAngle, target.legD.vAngle, self.legD.vAngle) - self.legD.vAngle, self.legD.vAngle - start.legD.vAngle)
        self.legD.hAngle += min((target.legD.hAngle - start.legD.hAngle) * ratio, self.legD.hAngle - start.legD.hAngle)
        self.legE.vAngle += min(next_angle(start.legE.vAngle, target.legE.vAngle, self.legE.vAngle) - self.legE.vAngle, self.legE.vAngle - start.legE.vAngle)
        self.legE.hAngle += min((target.legE.hAngle - start.legE.hAngle) * ratio, self.legE.hAngle - start.legE.hAngle)
        self.legF.vAngle += min(next_angle(start.legF.vAngle, target.legF.vAngle, self.legF.vAngle) - self.legF.vAngle, self.legF.vAngle - start.legF.vAngle)
        self.legF.hAngle += min((target.legF.hAngle - start.legF.hAngle) * ratio, self.legF.hAngle - start.legF.hAngle)

    def __eq__(self, other):
        return isinstance(other, HexapodAngles) and self.legA == other.legA and self.legB == other.legB and self.legC == other.legC and self.legD == other.legD and self.legE == other.legE and self.legF == other.legF
        