# This is a script to move mirte around in the simulation. Due to some 
# skill issues, the script simply sends commands through the terminal to 
# update the pose of mirte. This should be possible in a neater way but 
# Gazebo's documentation is very unclear (if it exists at all)


from subprocess import call
from time import sleep
import numpy as np

class PoseMatrix:

    def __init__(self, matrix=None):
        """init identity pose matrix"""
        if matrix is None: 
            self.matrix = np.identity(4)
        else:
            self.matrix = matrix


    def __mul__(self, other):
        return PoseMatrix(self.matrix @ other.matrix)
    
    def __str__(self):
        return f"{self.matrix}"

    def from_quat(self, quat):
        """Add rotation matrix from quaternion
        
        quaternion should be normalized and given as (x,y,z,w)
        """
        x, y, z, w = quat

        # Compute each element of the rotation matrix (wikipedia)
        r11 = 1 - 2 * (y ** 2 + z ** 2)
        r12 = 2 * (x * y - z * w)
        r13 = 2 * (x * z + y * w)

        r21 = 2 * (x * y + z * w)
        r22 = 1 - 2 * (x ** 2 + z ** 2)
        r23 = 2 * (y * z - x * w)

        r31 = 2 * (x * z - y * w)
        r32 = 2 * (y * z + x * w)
        r33 = 1 - 2 * (x ** 2 + y ** 2)

        # Form the rotation matrix
        rotation_matrix = np.array([
            [r11, r12, r13],
            [r21, r22, r23],
            [r31, r32, r33]
        ])

        self.matrix[:3,:3] = rotation_matrix
        return self
        
    def add_position(self, pos):
        """Add position part to pose matrix"""
        x, y, z = pos
        self.matrix[:3, 3] = x, y, z
        return self

    
    def get_quat(self):
        # Compute the trace of the matrix (courtesy of chatGPT)
        R = self.matrix[:3, :3]

        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            # Determine which major diagonal element has the greatest value
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s

        return np.array([x, y, z, w])
        


origin = PoseMatrix()
move_forward = PoseMatrix().add_position((1, 0, 0))

print(move_forward.matrix)

new_position = origin*move_forward*move_forward

print(new_position)


# # Setting

# v = 1  # m/s
# dt = 0.1  # s
# T = 2  # s

# omega = 0.2 * pi

# def angle_to_quaternion(angle):
#     """Convert rotation angle in z-axis to quaternion"""
#     return cos(angle/2), 0, 0, sin(angle/2)

# def quat_multiplication(old_quat, rot_quat):
#     """Return quaternion describing new position (only works for rot around z axis)"""
#     qo, _, _, zo = old_quat
#     qr, _, _, zr = rot_quat
#     s = qo*qo + zo*zo
#     return s*qr, 0, 0, s*zr


# x, y, z = 0, 0, 0

# for t in arange(0, T, dt):
#     position = f"{{x: {x}, y: {y}, z: {z}}}"
#     wq, xq, yq, zq = angle_to_quaternion(omega)
#     orientation = f"{{x: {xq}, y: {yq}, z: {zq}, w: {wq}}}"
#     call(["ign", "service", "-s", "/world/drone_cage_world/set_pose", "--timeout", 
#       "1000", "--reqtype", "ignition.msgs.Pose", "--reptype", 
#       "ignition.msgs.Boolean", "--req", 
#       f"name: 'mirte_master' id: 45 position: {position} orientation: {orientation}"])
#     x = x + v*dt
#     y = 0
#     z = 0
#     print(t)
#     print(position)


#     sleep(dt)




    



