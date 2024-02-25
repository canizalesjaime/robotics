import math 
import numpy as np
from frames import Frame2D, communicate_point_across_frames, transform_point

###############################################################################
def question1():
    mat=np.array([[1,2,3],
                  [4,5,6],
                  [7,8,9] ])
    print(5*mat+mat)

###############################################################################
def question2():
    mat1=np.array([[7,0,8],
                   [2,4,3] ])
    mat2=np.array([[0,2],
                   [9,6],
                   [1,5] ])
    print(mat1@mat2) 
    print(mat2@mat1)

###############################################################################
def question3():
    mat1=np.array([[3,2,1],
                   [-1,8,7],
                   [2,3,1] ])
    print(np.around(np.linalg.inv(mat1),2))
    mat2=np.array([[1,-2,1],
                   [3,9,3],
                   [-9,-27,-9] ])
    print(np.linalg.det(mat2))

###############################################################################
def question4():
    mat=np.array([[3,7,1],
                  [1,-4,6],
                  [8,8,8] ])
    print(np.linalg.det(mat))

###############################################################################
def question5():
    v1=np.array([2,2,2])
    v2=np.array([8,-4,3])
    print(np.cross(v1,v2))

###############################################################################
def question6():
    v_1=np.array([1,-2,3])
    v_2=np.array([4,0,1])
    print(math.degrees(np.arccos(np.dot(v_1, v_2)/(np.linalg.norm(v_1)*np.linalg.norm(v_2)))))

###############################################################################
def question7():
    #frames
    frame_ab=Frame2D(np.array([3,4]),135, "A", "B")
    frame_bc=Frame2D(np.array([-2,2]),-30, "B", "C")
    frame_ad=Frame2D(np.array([3,-3]),0, "A", "D")
    frame_ae=Frame2D(np.array([0,0]), 60,"A", "E")
    frame_af=Frame2D(np.array([-1,-1]), 45,"A", "F")
    frame_fg=Frame2D(np.array([-2,5]), 90,"F", "G")
    
    #points
    apt1 = np.array([3,-2])
    bpt2 = np.array([8,6])
    cpt3 = np.array([-3,-5])
    dpt4 = np.array([-2,4])
    ept5 = np.array([.7,.7])
    fpt6 = np.array([-3.14,2.718])
    gpt7 = np.array([3,4])

    #2
    frame_ac=frame_ab*frame_bc
    frame_ag=frame_af*frame_fg
    
    #3
    frame_da = frame_ad.get_inverse()
    #Note the child frame is the frame!!
    #so below reads as to_frame=d, from_frame=a
    dpt1=communicate_point_across_frames(frame_ad,frame_da, apt1)
    apt4=communicate_point_across_frames(frame_da,frame_ad,dpt4)
    
    #4
    frame_ea = frame_ae.get_inverse()
    ept1=communicate_point_across_frames(frame_ae,frame_ea,apt1)
    apt5=communicate_point_across_frames(frame_ea,frame_ae,ept5)

    #5
    frame_ba = frame_ab.get_inverse()
    bpt1=communicate_point_across_frames(frame_ab,frame_ba,apt1)
    apt2=communicate_point_across_frames(frame_ba,frame_ab,bpt2)
    
    #6
    frame_fa = frame_af.get_inverse()
    fpt1=communicate_point_across_frames(frame_af,frame_fa,apt1)
    apt6=communicate_point_across_frames(frame_fa,frame_af,fpt6)

    #7
    fpt2=communicate_point_across_frames(frame_af,frame_fa,apt2)
    bpt6=communicate_point_across_frames(frame_ab,frame_ba,apt6)

    #8
    dpt5=communicate_point_across_frames(frame_ad,frame_da,apt5)
    ept4=communicate_point_across_frames(frame_ae,frame_ea,apt4)
    
    #9
    frame_ca = frame_ac.get_inverse()
    frame_cg=frame_ca*frame_ag
    cpt7=communicate_point_across_frames(frame_ac,frame_cg,gpt7)

    frame_ga = frame_ag.get_inverse()
    gpt3=communicate_point_across_frames(frame_ag,frame_ga*frame_ac,cpt3)

    #A
    apt1_prime=transform_point(np.array([-6,7]), apt1, -80)

    #B
    cpt7_prime=transform_point(np.array([4,-2]),cpt7,130)