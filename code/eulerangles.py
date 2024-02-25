import math 
import numpy as np
import matplotlib.pyplot as plt

###############################################################################
def rot_mat_z(alpha_degree):
    alpha_rad=math.radians(alpha_degree)
    return np.array([[math.cos(alpha_rad), -math.sin(alpha_rad), 0.0],
                     [math.sin(alpha_rad), math.cos(alpha_rad),  0.0],
                     [0.0,             0.0,              1.0]])

###############################################################################
def rot_mat_y(beta_degree):
    beta_rad=math.radians(beta_degree)
    return np.array([[math.cos(beta_rad),  0.0, math.sin(beta_rad)  ],
                     [0.0,             1.0, 0.0             ], 
                     [-math.sin(beta_rad), 0.0, math.cos(beta_rad)  ]])

###############################################################################
def rot_mat_x(gamma_degree):
    gamma_rad=math.radians(gamma_degree)
    return np.array([[1.0,  0.0,             0.0             ],
                     [0.0,  math.cos(gamma_rad), -math.sin(gamma_rad)], 
                     [0.0,  math.sin(gamma_rad), math.cos(gamma_rad) ]])

###############################################################################   
def euler_angles_zyx_to_rot_matrix(alpha,beta,gamma):
    return rot_mat_z(alpha)@rot_mat_y(beta)@rot_mat_x(gamma)

###############################################################################
def non_euler_angles_xyz_to_rot_matrix(alpha,beta,gamma):
    return rot_mat_x(gamma)@rot_mat_y(beta)@rot_mat_z(alpha)

###############################################################################
def rot_matrix_to_euler_angles_zyx(rot, pos_angle=False):
    r31, r21, r11, r32, r33=rot[2][0],rot[1][0],rot[0][0],rot[2][1],rot[2][2]

    beta1 = -math.asin(r31)
    
    if beta1 == 90 or beta1 == -90:
        print('GIMBAL LOCK ERROR')
        return
    
    beta2 = math.pi - beta1

    alpha1 = math.degrees(math.atan2(r21/math.cos(beta1), r11/math.cos(beta1)))
    alpha2 = math.degrees(math.atan2(r21/math.cos(beta2), r11/math.cos(beta2)))
    
    gamma1 = math.degrees(math.atan2(r32/math.cos(beta1), r33/math.cos(beta1)))
    gamma2 = math.degrees(math.atan2(r32/math.cos(beta2), r33/math.cos(beta2)))

    beta1 = math.degrees(beta1)
    beta2 = math.degrees(beta2)

    if pos_angle:
        if beta1 < 0:
            beta1=beta1+360
        if beta2 < 0:
            beta2=beta2+360
        if alpha1 < 0:
            alpha1=alpha1+360
        if alpha2 < 0:
            alpha2=alpha2+360
        if gamma1 < 0:
            gamma1=gamma1+360
        if gamma2 < 0:
            gamma2=gamma2+360

    return np.array([[alpha1,beta1,gamma1],
                     [alpha2,beta2,gamma2]])

###############################################################################
def angle_between_vectors(v_1,v_2): 
    return math.degrees(np.arccos(np.dot(v_1, v_2)/(np.linalg.norm(v_1)*np.linalg.norm(v_2))))

###############################################################################
def rodrigues_rotation(v, angle_degree, axis):
    angle_radians = math.radians(angle_degree)
    axis_hat=axis/np.linalg.norm(axis)
    v_r=v*math.cos(angle_radians) + np.cross(axis_hat, v)* math.sin(angle_radians)\
        + axis_hat*(np.dot(axis_hat, v))*(1-math.cos(angle_radians))
    return v_r

###############################################################################
def angle_axis_to_rot_mat(angle_degree, axis):
    K = np.array([ [0,-axis[2],axis[1]],
                   [axis[2],0,-axis[0]],
                   [-axis[1],axis[0],0] ])
    I = np.array([ [1,0,0],
                   [0,1,0],
                   [0,0,1] ])
    return I + math.sin(math.radians(angle_degree))*K +\
           (1-math.cos(math.radians(angle_degree)))*(K@K)