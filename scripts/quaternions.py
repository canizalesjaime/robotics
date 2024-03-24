from pyquaternion import Quaternion
import math 
import numpy as np

###############################################################################
def quaternions_from_euler_zyx(alpha, beta, gamma):
    qz = Quaternion(math.cos(math.radians(alpha/2)), 0, 0, math.sin(math.radians(alpha/2)))
    qy = Quaternion(math.cos(math.radians(beta/2)), 0, math.sin(math.radians(beta/2)), 0)
    qx = Quaternion(math.cos(math.radians(gamma/2)), math.sin(math.radians(gamma/2)), 0, 0)
    return qz*qy*qx

###############################################################################
def convert_quaternion_to_matrix(quat):
    a,b,c,d=quat[0],quat[1],quat[2],quat[3]
    return np.array([[a,-b,-c,-d],
                     [b,a,-d,c],
                     [c,d,a,-b],
                     [d,-c,b,a]])

###############################################################################
def quaternion_matrix_rotation(q, vec):
    m=convert_quaternion_to_matrix(q)@np.array([0,vec[0], vec[1], vec[2]])
    return convert_quaternion_to_matrix(m)@np.array([q[0],-q[1], -q[2],-q[3]])

###############################################################################
def quaternion_matrix_mult(q1,q2):
    return convert_quaternion_to_matrix(q1)@q2

###############################################################################
def quaternion_angle_axis(angle, rotational_axis):
    angle=math.radians(angle/2)
    rotational_axis_hat=rotational_axis/np.linalg.norm(rotational_axis)
    real=math.cos(angle)
    i=math.sin(angle)*rotational_axis_hat[0]
    j=math.sin(angle)*rotational_axis_hat[1]
    k=math.sin(angle)*rotational_axis_hat[2]
    return Quaternion(real, i, j, k)

###############################################################################
def convert_quaternion_to_euler_angles(q):
    alpha = np.arctan2(2*(q[0]*q[1]+q[2]*q[3]),1-(2*(q[1]*q[1]+q[2]*q[2])) )
    
    beta = np.arcsin(2*(q[0]*q[2]-q[1]*q[3]))

    gamma = np.arctan2(2*(q[0]*q[3]+q[1]*q[2]),1-(2*(q[2]*q[2]+q[3]*q[3])) ) 

    return (math.degrees(alpha),math.degrees(beta),math.degrees(gamma))

###############################################################################
def solve_gimbal_lock(rot):
    q=Quaternion(matrix=rot)
    return q.yaw_pitch_roll
