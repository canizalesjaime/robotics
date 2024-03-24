import numpy as np
from frames import Frame2D, Frame3D, communicate_point_across_frames
import math
from eulerangles import rot_matrix_to_euler_angles_zyx, rot_mat_z, rot_mat_y, rot_mat_x, angle_between_vectors

###############################################################################
def question1():
    #1
    v1=np.array([0,2,2.24])
    v2=np.array([4,0,-3])
    v1_cross_v2=np.cross(v1,v2)
    print(v1_cross_v2)

    #2
    print(angle_between_vectors(v1,v2))

    #3
    #none of the vectors are colinear based on 1 and 2

###############################################################################
def question2():
    #set up
    frame_ab=Frame2D(np.array([1,-1]), 30, 'A', 'B')
    frame_bc=Frame2D(np.array([2,1]), 90, 'B', 'C')

    apt1=np.array([-2,1])
    bpt2=np.array([1,3])
    cpt3=np.array([2,0])

    #1 
    frame_ba = frame_ab.get_inverse()
    bpt1=communicate_point_across_frames(frame_ab,frame_ba,apt1)
    print(bpt1)

    #2
    frame_ac = frame_ab*frame_bc
    apt3=communicate_point_across_frames(frame_ba, frame_ac,cpt3)
    print(apt3)

###############################################################################
def question3():
    #1
    rot_mat=np.array([ [.5, -.15, .85],
                       [.5,.85,-.15],
                       [-.71,.5,.5] ] )
    print(rot_matrix_to_euler_angles_zyx(rot_mat))
    
    #2
    frame_ab=Frame3D(np.array([2,.7,1]),'A','B',rot_mat=rot_mat)
    apt2=communicate_point_across_frames(frame_ab.get_inverse(),frame_ab,np.array([0,-1,0]))
    print(apt2)

    #3
    #if beta is 90, and did not occur

###############################################################################
def question4():
    #1
    a=complex(3,-1)
    b=complex(math.cos(math.radians(30)), math.sin(math.radians(30)) )
    print (b*a)

    #2
    print(np.around(rot_mat_z(90),2))
    print(np.around(rot_mat_x(0),2))
    print(np.around(rot_mat_y(45),2))
###############################################################################
###############################################################################
print("1. ")
question1()
print("\n2. ")
question2()
print("\n3. ")
question3()
print("\n4. ")
question4()