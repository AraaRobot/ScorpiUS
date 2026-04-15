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

    def to_servo_angles_msg(self, msg: ServoAngles) -> ServoAngles:
        msg.vert_a = float(self.legA.vAngle)
        msg.vert_b = float(self.legB.vAngle)
        msg.vert_c = float(self.legC.vAngle)
        msg.vert_d = float(self.legD.vAngle)
        msg.vert_e = float(self.legE.vAngle)
        msg.vert_f = float(self.legF.vAngle)
        msg.horiz_a = float(self.legA.hAngle)
        msg.horiz_b = float(self.legB.hAngle)
        msg.horiz_c = float(self.legC.hAngle)
        msg.horiz_d = float(self.legD.hAngle)
        msg.horiz_e = float(self.legE.hAngle)
        msg.horiz_f = float(self.legF.hAngle)
        return msg 
    
    def interpolate(self, start, target, ratio:float):
        if not isinstance(start, HexapodAngles):
            raise TypeError(start + " not a HexapodAngles.")
        if not isinstance(target, HexapodAngles):
            raise TypeError(target + " not a HexapodAngles.")
        if ratio <= 0 or ratio > 1:
            raise ValueError("Ratio must be between 0 and 1. 0 is exclusive.")

        def power_increment(start_angle : float, target_angle : float, current_angle : float, exponent: float = 2):
            if math.isclose(target_angle, start_angle, abs_tol=self.epsilon):
                return target_angle - current_angle
            if math.isclose(target_angle, 0, abs_tol=self.epsilon):
                k = max(target_angle, start_angle, key=abs)
                r = 1/ratio
                a = -k / math.pow(r, exponent)
                root = (current_angle-k)/a
                last_pos = math.pow(abs(root), 1/exponent)
                return a * math.pow(last_pos+1, exponent) + k - current_angle
            else:
                (h, k) = 1/ratio, max(target_angle, start_angle, key=abs)
                a = -k / math.pow(-h, exponent)
                root = (current_angle-k)/a
                last_pos = math.copysign(math.pow(abs(root), 1/exponent), -root) + h
                return a * math.pow(last_pos+1 - h, exponent) + k - current_angle
        
        # Interpolate horizontal angles linearly.
        self.legA.hAngle += min((target.legA.hAngle - start.legA.hAngle) * ratio, target.legA.hAngle - self.legA.hAngle, key=abs)
        self.legB.hAngle += min((target.legB.hAngle - start.legB.hAngle) * ratio, target.legB.hAngle - self.legB.hAngle, key=abs)
        self.legC.hAngle += min((target.legC.hAngle - start.legC.hAngle) * ratio, target.legC.hAngle - self.legC.hAngle, key=abs)
        self.legD.hAngle += min((target.legD.hAngle - start.legD.hAngle) * ratio, target.legD.hAngle - self.legD.hAngle, key=abs)
        self.legE.hAngle += min((target.legE.hAngle - start.legE.hAngle) * ratio, target.legE.hAngle - self.legE.hAngle, key=abs)
        self.legF.hAngle += min((target.legF.hAngle - start.legF.hAngle) * ratio, target.legF.hAngle - self.legF.hAngle, key=abs)

        # Interpolate vertical angles with a custom non linear function to make the movement more natural.
        self.legA.vAngle += min(power_increment(start.legA.vAngle, target.legA.vAngle, self.legA.vAngle), target.legA.vAngle - self.legA.vAngle, key=abs)
        self.legB.vAngle += min(power_increment(start.legB.vAngle, target.legB.vAngle, self.legB.vAngle), target.legB.vAngle - self.legB.vAngle, key=abs)
        self.legC.vAngle += min(power_increment(start.legC.vAngle, target.legC.vAngle, self.legC.vAngle), target.legC.vAngle - self.legC.vAngle, key=abs)
        self.legD.vAngle += min(power_increment(start.legD.vAngle, target.legD.vAngle, self.legD.vAngle), target.legD.vAngle - self.legD.vAngle, key=abs)
        self.legE.vAngle += min(power_increment(start.legE.vAngle, target.legE.vAngle, self.legE.vAngle), target.legE.vAngle - self.legE.vAngle, key=abs)
        self.legF.vAngle += min(power_increment(start.legF.vAngle, target.legF.vAngle, self.legF.vAngle), target.legF.vAngle - self.legF.vAngle, key=abs)

    def __eq__(self, other):
        return isinstance(other, HexapodAngles) and self.legA == other.legA and self.legB == other.legB and self.legC == other.legC and self.legD == other.legD and self.legE == other.legE and self.legF == other.legF
        