import math 
import numpy as np
from eulerangles import rot_matrix_to_euler_angles_zyx, euler_angles_zyx_to_rot_matrix

###############################################################################
class Frame2D:
    def __init__(self, origin:np.array, orientation_degree:float, 
                 parent_frame:str, child_frame:str)->None:
        if len(origin) != 2:
            print("Origin for frame invalid.")
            return
        
        if not (isinstance(orientation_degree,float) or isinstance(orientation_degree,int)):
            print("Orientation for frame input invalid.")
            return 
        
        if not isinstance(parent_frame, str) or not isinstance(child_frame,str):
            print("Child or parent frame invalid.")
            return
         
        self.origin=origin
        self.orientation_degree=orientation_degree
        self.parent_frame=parent_frame
        self.child_frame=child_frame

    def frame_to_transformation(self)->np.array:
        orientation_rad=math.radians(self.orientation_degree)
        return np.array([[math.cos(orientation_rad), -math.sin(orientation_rad), self.origin[0]],
                         [math.sin(orientation_rad),  math.cos(orientation_rad), self.origin[1]],
                         [0,               0,                        1] ])

    def get_inverse(self):
       inv_T=np.linalg.inv(self.frame_to_transformation())
       return Frame2D(np.array([inv_T[0][2],inv_T[1][2]]),
                       -self.orientation_degree,self.child_frame,self.parent_frame)
    
    def frame_column_vectors(self):
        t=self.frame_to_transformation()
        return np.array([ [t[0][0]+t[0][2],t[0][1]+t[0][2]],
                          [t[1][0]+t[1][2],t[1][1]+t[1][2]] ])

    def __mul__(self, other)->np.array:
        if self.child_frame==other.parent_frame:
            T=self.frame_to_transformation()@other.frame_to_transformation()
            if (self.orientation_degree + other.orientation_degree) < 0:
                T[1][0]=-T[1][0]

            return Frame2D(np.array([T[0][2], T[1][2]]), math.degrees(np.arctan2(T[1][0],T[0][0])),
                            self.parent_frame, other.child_frame )
        
        else:
            print("Frames are not compatible")
            return
        
    def __str__(self) -> str:
        return 'orientation: '+ str(self.orientation_degree)+'\n'+'origin: '+ \
                str(self.origin)+'\n' +'Parent Frame: '+ str(self.parent_frame) + \
                '\n' + 'Child Frame: ' + str(self.child_frame)

###############################################################################
class Frame3D:
    def __init__(self, origin:np.array, parent_frame:str, child_frame:str,*, 
                 yaw:float=0.0, pitch:float=0.0, roll:float=0.0, 
                 rot_mat:np.array=np.array([[1,0,0],[0,1,0],[0,0,1]]))->None:
        if len(origin) != 3:
            print("Origin for frame invalid.")
            return
        
        if not isinstance(parent_frame, str) or not isinstance(child_frame,str):
            print("Child or parent frame invalid.")
            return
        
        if not (isinstance(yaw,float) or isinstance(yaw,int)) or \
           not (isinstance(pitch,float) or isinstance(pitch,int)) or \
           not (isinstance(roll,float) or isinstance(roll,int)):
            print("Orientation for frame input invalid.")
            return 
        
        self.origin=origin
        self.parent_frame=parent_frame
        self.child_frame=child_frame

        if not np.array_equal(rot_mat, np.array([[1,0,0],[0,1,0],[0,0,1]])) and rot_mat.shape==(3,3):
            angles=rot_matrix_to_euler_angles_zyx(rot_mat)
            self.yaw,self.pitch,self.roll = angles[0][0],angles[0][1],angles[0][2]
        
        else: 
            self.yaw=yaw
            self.pitch=pitch
            self.roll=roll

    def frame_to_transformation(self)->np.array:
        R=euler_angles_zyx_to_rot_matrix(self.yaw,self.pitch,self.roll)
        return np.array([ [R[0][0],R[0][1],R[0][2],self.origin[0]],
                          [R[1][0],R[1][1],R[1][2],self.origin[1]],
                          [R[2][0],R[2][1],R[2][2],self.origin[2]],
                          [0,0,0,1] ])

    def frame_column_vectors(self)->np.array:
        t=self.frame_to_transformation()
        return np.array([ [t[0][0]+t[0][3],t[0][1]+t[0][3],t[0][2]+t[0][3]],
                          [t[1][0]+t[1][3],t[1][1]+t[1][3],t[1][2]+t[1][3]], 
                          [t[2][0]+t[2][3],t[2][1]+t[2][3],t[2][2]+t[2][3]] ])        

    def get_inverse(self)->list:
       inv_T=np.linalg.inv(self.frame_to_transformation())
       R=rot_matrix_to_euler_angles_zyx(np.array([ [inv_T[0][0],inv_T[0][1],inv_T[0][2]],
                                                   [inv_T[1][0],inv_T[1][1],inv_T[1][2]],
                                                   [inv_T[2][0],inv_T[2][1],inv_T[2][2]] ]))
       yaw1, pitch1, roll1 = R[0][0], R[0][1], R[0][2]
       return Frame3D(np.array([inv_T[0][3],inv_T[1][3], inv_T[2][3]]),
                       self.child_frame,self.parent_frame,yaw=yaw1,pitch=pitch1,roll=roll1)

    def __mul__(self, other):
        if self.child_frame==other.parent_frame:
            T=self.frame_to_transformation()@other.frame_to_transformation()
            R=rot_matrix_to_euler_angles_zyx(np.array([ [T[0][0],T[0][1],T[0][2]],
                                                        [T[1][0],T[1][1],T[1][2]],
                                                        [T[2][0],T[2][1],T[2][2]] ]))
            yaw1, pitch1, roll1 = R[0][0], R[0][1], R[0][2]
            return Frame3D(np.array([T[0][3], T[1][3], T[2][3]]), self.parent_frame,
                            other.child_frame, yaw=yaw1, pitch=pitch1, roll=roll1 )
        
        else:
            print("Frames are not compatible")
            return
        
    def __str__(self)->str:
        return 'yaw, pitch, roll: '+ str(self.yaw)+', '+str(self.pitch) + ', '+ \
                str(self.roll)+'\n'+'origin: '+ str(self.origin)+'\n' + \
                'Parent Frame: '+ str(self.parent_frame) + \
                '\n' + 'Child Frame: ' + str(self.child_frame)
    
    
# Helper Functions
################################################################################
################################################################################
def transform_point(translation:np.array, point:np.array, theta1:float,*, 
                    theta2:float=0.0, theta3:float=0.0 ):
        if len(translation) == 3 and len(point)==3:
            R=euler_angles_zyx_to_rot_matrix(theta1,theta2,theta3)
            T=np.array([[R[0][0],R[0][1],R[0][2],translation[0]],
                        [R[1][0],R[1][1],R[1][2],translation[1]],
                        [R[2][0],R[2][1],R[2][2],translation[2]],
                        [0,0,0,1] ])
        
        elif len(translation)==2 and len(point)==2:
            theta_rad=math.radians(theta1)
            T=np.array([[math.cos(theta_rad), -math.sin(theta_rad), translation[0]],
                        [math.sin(theta_rad), math.cos(theta_rad),  translation[1]],
                        [0,               0,                        1] ])
        
        else:
            print("Translation or point are incorrectly sized.")
            return
        
        pt_final=T@np.append(point,1)
        return pt_final[:-1]


###############################################################################
def communicate_point_across_frames(to_frame, from_frame, point: np.array):
    if to_frame.child_frame == from_frame.parent_frame:
        point_prime=from_frame.frame_to_transformation()@np.append(point,1)
        return point_prime[:-1]

    else:
        print("Frames are not compatible")
        return
    