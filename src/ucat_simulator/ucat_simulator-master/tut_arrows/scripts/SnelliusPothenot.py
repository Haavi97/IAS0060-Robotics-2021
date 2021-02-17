# -*- coding: utf-8 -*-
"""
Created on Thu Dec 11 18:00:53 2014

@author: keijo
"""

import numpy as np
import math

def rotateVec(vec, angle):
    x = vec[0] * math.cos(angle) - vec[1] * math.sin(angle)
    y = vec[1] * math.cos(angle) + vec[0] * math.sin(angle)
    return np.array([x, y])

def smallestAngle(target, source):
    a = target - source
    return (a + math.pi) % (2.0 * math.pi) - math.pi 

def solveSnelliusPothenot(A, B, C, alpha, beta):
    """
    Solve Snellius-Pothenot problem: http://en.wikipedia.org/wiki/Snellius%E2%80%93Pothenot_problem
    
    Parameters
    ----------
    A,B,C : arrays
        each is array of coordinates [x, y]    
    
    alpha : float (in radians)
        angle of segment AC
    
    beta : float (in radians)
        angle by segment CB
    
    Returns
    -------
    P : coordinate array
    """
    
    A = np.array(A, dtype='float')
    B = np.array(B, dtype='float')
    C = np.array(C, dtype='float')    
    
    AC = np.linalg.norm(C-A)
    BC = np.linalg.norm(C-B)
    AB = np.linalg.norm(B-A)
    
    gamma = math.acos( ( AC ** 2 + BC ** 2 - AB ** 2 ) / ( 2.0 * AC * BC ) ) # angle ACB
    
    theta = math.atan2(BC * math.sin(alpha), AC * math.sin(beta))
    
    K = 2.0 * np.pi - alpha - beta - gamma
    
    W = 2.0 * math.atan( math.tan(np.pi/4 - theta) * math.tan( 0.5 * (alpha+beta+gamma) ) )
    
    x = (K + W) / 2.0 # angle PAC
    y = (K - W) / 2.0 # angle PBC
    
    PC = None
    if abs(beta) > abs(alpha):
        PC = BC * math.sin(y) / math.sin(beta)
    else:
        PC = AC * math.sin(x) / math.sin(alpha)
    
    # Determine heading of CP by rotating CA and CB to the direction of CP.
    # Accepts such rotation where rotations for CA and CB agree. This last
    # step is used to determine which way the rotations should be, negative
    # or positive. 
    CA_dx = A[0] - C[0]
    CA_dy = A[1] - C[1]
    
    CB_dx = B[0] - C[0]
    CB_dy = B[1] - C[1]

    x_2 = np.pi - alpha - x # angle ACP
    y_2 = np.pi - beta - y # angle PBC
    
    heading = None
    m11 = math.atan2(CA_dy, CA_dx) + x_2
    m12 = math.atan2(CA_dy, CA_dx) - x_2 
    
    m21 = math.atan2(CB_dy, CB_dx) + y_2
    m22 = math.atan2(CB_dy, CB_dx) - y_2
    
    diff = [abs(smallestAngle(m11, m21)), abs(smallestAngle(m11, m22)), abs(smallestAngle(m12, m21)), abs(smallestAngle(m12, m22))]
    index = diff.index(min(diff))
    if index == 0 or index == 1:
        heading = m11
    else:
        heading = m12
    
    CP = np.array([math.cos(heading), math.sin(heading)]) * PC    
    
    return C + CP

    