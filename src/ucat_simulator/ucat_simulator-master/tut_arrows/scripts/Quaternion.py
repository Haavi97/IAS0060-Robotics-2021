#!/usr/bin/env python

"""
@author: keijo
"""

import numpy as np
import math

class Quaternion:
    def __init__(self, q):
        """
        q = [w, x, y, z]
        """
        self.q = np.array(q)
        
    def __repr__(self):
        return "w: {0:.3f}, x: {1:.3f}, y: {2:.3f}, z: {3:.3f}".format(*self.q)

    @classmethod
    def fromAxisAngle(cls, v, theta):
        """
        [w, x, y, z] = [cos(theta/2), sin(theta/2) * nx, sin(theta/2)* ny, sin(theta/2) * nz], 
        where theta is angle and [nx, ny, nz] is axis (i.e. axis-angle representation)
        """
        v = v / np.linalg.norm(v)
        x, y, z = v
        theta /= 2
        w = math.cos(theta)
        x = x * math.sin(theta)
        y = y * math.sin(theta)
        z = z * math.sin(theta)
        return cls([w, x, y, z])

    def toAxisAngle(self):
        w, v = self.q[0], self.q[1:]
        v_len = np.linalg.norm(v)
        
        angle = 2.0 * math.atan2(v_len, w);
        
        if v_len > 0.0:
            return v / v_len, angle
        else:
            return np.array([1.0, 0.0, 0.0]), angle
   
    def __mul__(self, other):
        w1, x1, y1, z1 = self.q
        w2, x2, y2, z2 = other.q
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return Quaternion([w, x, y, z])

    def __add__(self, other):
        return Quaternion(self.q + other.q)

    def __sub__(self, other):
        return Quaternion(self.q - other.q)

    def normalize(self):
        return Quaternion(self.q / np.linalg.norm(self.q))

    def conjugate(self):
        q_conj = np.array(self.q)
        q_conj[1:] *= -1
        return Quaternion(q_conj)

    def multVec(self, v):
        v = v / np.linalg.norm(v)
        q_v = Quaternion(np.append([0.0], v))
        q_v = self * q_v * self.conjugate()
        return q_v.q[1:]


def getRotation(q1, q0):
    """
    Get rotation from q0 to q1.
    
    Parameters
    ----------
    q1 : Quaternion
         final orientation
    q0 : Quaternion
         initial orientation
    
    Returns
    -------
    w : np.array
        rotation [rx, ry, rz] in rad
    """
    if np.dot(q1.q, q0.q) < 0.0:
        # Complementary rotation - both q1 and -q1 represent same rotation.
        # In case dot(q1, q0) is negative the two rotation have different
        # sign which gives long rotation along the circle. To avoid getting
        # long rotation simply negate one of the quaternions.
        q1 = Quaternion(-q1.q)
        
    r = q1 * q0.conjugate()
    v, angle = r.toAxisAngle()

    return np.array(v) * angle

        
def getAngularVelocity(q1, q0, dt):
    """
    Get angular velocity (rad/s) from orientations in two consequative time instants.
    
    Parameters
    ----------
    q1 : Quaternion
         orientation at time t+1
    q0 : Quaternion
         orientation at time t
    dt : float
         time interval between q0 and q1
    
    Returns
    -------
    w : np.array
        angular velocity [wx, wy, wz] in rad/s
    """
    rotation = getRotation(q1, q0)
    return rotation / dt

