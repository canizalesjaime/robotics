import numpy as np
import math

# Get parameters from a column in dh table
############################################################################### 
def get_transformation_matrix_from_dh(a, alpha_degree, d, theta_degree):
    theta_radians = math.radians(theta_degree) 
    alpha_radians = math.radians(alpha_degree)
    np.array([ [math.cos(theta_radians), -math.sin(theta_radians), 0,  a], 
               [math.sin(theta_radians)*math.cos(alpha_radians), math.cos(theta_radians)*math.cos(alpha_radians), -math.sin(alpha_radians), -math.sin(alpha_radians)*d],
               [math.sin(theta_radians)*math.sin(alpha_radians), math.cos(theta_radians)*math.sin(alpha_radians), math.cos(alpha_radians),  math.cos(alpha_radians)*d],
               [0,0,0,1] ])
    
###############################################################################
###############################################################################


