#from pyquaternion import Quaternion ###You can check this library out for a quaternion class 
import numpy as np
import math
from eulerangles import rot_matrix_to_euler_angles_zyx, rot_mat_x, rot_mat_y, euler_angles_zyx_to_rot_matrix

###############################################################################
def convert_quaternion_to_matrix(quat):
    a,b,c,d=quat[0],quat[1],quat[2],quat[3]
    return np.array([[a,-b,-c,-d],
                     [b,a,-d,c],
                     [c,d,a,-b],
                     [d,-c,b,a]])

###############################################################################
def quaternion_matrix_mult(q1,q2):
    return convert_quaternion_to_matrix(q1)@q2

#part 1 question 5 
###############################################################################
def quaternion_matrix_rotation(q, vec):
    m=convert_quaternion_to_matrix(q)@np.array([0,vec[0], vec[1], vec[2]])
    return convert_quaternion_to_matrix(m)@np.array([q[0],-q[1], -q[2],-q[3]])

#part 1 question 6 
###############################################################################
def rot_mat_zyx_to_quat(rot):
   angles=rot_matrix_to_euler_angles_zyx(rot)
   print("angles: ", np.around(angles,2))
   q_z=np.array([math.cos(math.radians(angles[0][0]/2)), 0, 0, math.sin(math.radians(angles[0][0]/2))])
   q_y=np.array([math.cos(math.radians(angles[0][1]/2)), 0, math.sin(math.radians(angles[0][1]/2)), 0])
   q_x=np.array([math.cos(math.radians(angles[0][2]/2)), math.sin(math.radians(angles[0][2]/2)), 0, 0])
   
   q_zy=quaternion_matrix_mult(q_z,q_y)
   q_zyx=quaternion_matrix_mult(q_zy,q_x)
   return q_zyx

#part 1 question 7
###############################################################################
def convert_quaternion_to_euler_angles(q):
    alpha = np.arctan2(2*(q[0]*q[1]+q[2]*q[3]),1-(2*(q[1]*q[1]+q[2]*q[2])) )
    
    beta = np.arcsin(2*(q[0]*q[2]-q[1]*q[3]))

    gamma = np.arctan2(2*(q[0]*q[3]+q[1]*q[2]),1-(2*(q[2]*q[2]+q[3]*q[3])) ) 

    return (math.degrees(alpha),math.degrees(beta),math.degrees(gamma))

#part 1 question 1 
###############################################################################
def question1():
   p_1=np.array([-1,2,3])
   q1 = np.array([math.cos(math.radians(35/2)),math.sin(math.radians(35/2)),0,0])
   q2= np.array([math.cos(math.radians(175/2)),0,math.sin(math.radians(175/2)),0])
   q_rot=quaternion_matrix_mult(q1,q2)
   print("q_rot: ", np.around(q_rot,2))
   print("ans: ", np.around(quaternion_matrix_rotation(q_rot,p_1),2))
   print("check: ",np.around(rot_mat_x(35)@rot_mat_y(175)@p_1,2))

#part 1 question 2 
###############################################################################
def question2():
   q_zyx=rot_mat_zyx_to_quat(euler_angles_zyx_to_rot_matrix(50,150,200))
   print("ans: ", np.around(q_zyx,2))
   p_test = np.array([7,-1,5])
   print("check: ", np.around(quaternion_matrix_rotation(q_zyx,p_test),2), 
          np.around(euler_angles_zyx_to_rot_matrix(50,150,200)@p_test,2)) 

###############################################################################
def question3():
   mat=np.array([ [.28,.77,.57],
                  [-.94,.34,0],
                  [-.19,-.54,.82] ])
   q = rot_mat_zyx_to_quat(mat)
   p_test=np.array([12, 7, -8])
   print("ans: ", np.around(q,2))
   print("check: ",np.around(quaternion_matrix_rotation(q,p_test),2)
         ,np.around(mat@p_test,2))

###############################################################################
def question4():
   q =  np.array([0.85,0.19, 0.46, 0.19])
   print("ans: ",convert_quaternion_to_euler_angles(q)) 
   #print("check: ",
   #      rot_matrix_to_euler_angles_zyx(Quaternion(0.85,0.19, 0.46, 0.19).rotation_matrix)) 
