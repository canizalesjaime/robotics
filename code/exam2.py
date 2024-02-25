import numpy as np
import math
from pyquaternion import Quaternion

###############################################################################
def question1():
    # 1
    qy = Quaternion(math.cos(math.radians(90/2)), 0, math.sin(math.radians(90/2)), 0)
    qx = Quaternion(math.cos(math.radians(180/2)), math.sin(math.radians(180/2)), 0, 0)
    qz = Quaternion(math.cos(math.radians(45/2)), 0, 0, math.sin(math.radians(45/2)))
    q_rot = qy*qx*qz
    print(q_rot)

    # 2
    v_1=np.array([0,1,0])
    v_1r=q_rot.rotate(v_1)
    print(np.around(v_1r,2))
 
###############################################################################
###############################################################################
print("1. ")
question1()
'''print("\n2. ")
question2()
print("\n3. ")
question3()
print("\n4. ")
question4()'''