import numpy as np
import math
import cmath
from frames import Frame3D, communicate_point_across_frames, transform_point
from eulerangles import rot_matrix_to_euler_angles_zyx, angle_between_vectors

###############################################################################
def question1():
    frame_ab=Frame3D(np.array([1,-2,5]),'A','B',yaw=45,pitch=100,roll=75 )
    frame_bc=Frame3D(np.array([-4,4,4]),'B','C',yaw=90,pitch=0,roll=15)
    frame_ad=Frame3D(np.array([1,-2,1]), 'A','D',rot_mat=np.array([[.15, .18,  -.97],
                                                                    [.09, -.98, -.16],
                                                                    [-.98, -.06,-.16]]))

    #1
    frame_cb = frame_bc.get_inverse()
    
    #2
    frame_ac=frame_ab*frame_bc
    frame_ca = frame_ac.get_inverse()
    apt1=np.array([1,2,-3])
    cpt2=np.array([-5,7,2])
    apt2=communicate_point_across_frames(frame_ca,frame_ac,cpt2)

    #3
    frame_ba= frame_ab.get_inverse()
    bpt2=communicate_point_across_frames(frame_ab,frame_ba,apt1)
    
    #4
    bpt3=np.array([3.14,-7.77,2.718])
    cpt3=communicate_point_across_frames(frame_ac,frame_cb,bpt3)

    #5
    apt1_prime=transform_point(np.array([-2,-3,-4]),apt1,theta1=50,theta2=150,theta3=200)

###############################################################################
def question2():   
    # part 1
    rot = np.array([[.15, .18,  -.97],
                    [.09, -.98, -.16],
                    [-.98, -.06,-.16]])
    angles = rot_matrix_to_euler_angles_zyx(rot)
    
    #2
    theta=angle_between_vectors(np.array([1,0,0]),np.array([.5,.5,.71]))
    
###############################################################################
def question3():
    # part 1
    p1=complex(8,-2)
    rot=complex(math.cos(math.radians(110)), math.sin(math.radians(110)) )