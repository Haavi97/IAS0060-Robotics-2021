#!/usr/bin/env python

"""
@author: Keijo Kuusmik
"""

import numpy as np

class UcatDynamics:
    def __init__(self):
        """       
        M - system inertia matrix
        C - Coriolis-centripedal matrix
        D - damping matrix
        G - vector of gravitational/buoyancy forces and moments
        J - Jacobian matrix transforms [velocity, angular velocity] (body -> world).
            Inverse Jacobian implements (world -> body).
        """
        self.M = np.zeros((6, 6))
        self.M_inv = np.zeros((6, 6))
        self.C = np.zeros((6, 6))
        self.D = np.zeros((6, 6))
        self.G = np.zeros(6)
        self.J = np.zeros((6, 6))
        self.J_inv = np.zeros((6, 6))

        self.mass = 19.0 # mass in kg
        self.rho = 1000.0 # density of water
        self.rog = np.array([0.0, 0.0, 0.02]) # center of mass (body frame)
        self.gamma = np.deg2rad(30) # flipper mounting angle

        self.flipper_length = 0.18
        self.flipper_width = 0.1

        self.Ib = self._get_inertia_matrix()

        self.RG = np.array([0.0, 0.0, 0.02]) # center of gravity (body frame)
        self.RB = np.array([0.0, 0.0, 0.0]) # center of buoyancy (body frame)
        self.We = self.mass * 9.81 # gravity
        self.Bo = self.We * 1.0 # buoyancy

        # Damping coef - s
        self.Cd1 = 0.46
        self.Cd2 = 0.7
        self.Cd3 = 0.91
        self.Cd4 = 0.9
        self.Cd5 = 0.8
        self.Cd6 = 0.7

        self._init_mass_matrix()

    @staticmethod
    def _get_inertia_matrix():
        """
        Moment of inertia U-CAT (solidworks)
        """
        Iyy = 464600000
        Iyz = -253500
        Iyx = 88700

        Izy = Iyz
        Izz = 511510000
        Izx = -549900

        Ixy = Iyx
        Ixz = Izx
        Ixx = 157400000

        return 1e-9 * np.array([
                         [Ixx, -Ixy, -Ixz],
                         [-Iyx, Iyy, -Iyz],
                         [-Izx, -Izy, Izz]])

    @staticmethod
    def _S(k):
        """
        Cross product matrix based on vector v (skew symmetric matrix).
        v * _S(k) is equivalent to the cross product of vectors v and k.
        """
        return np.array([
                        [0, -k[2], k[1]],
                        [k[2], 0, -k[0]],
                        [-k[1], k[0], 0]])

    def _init_mass_matrix(self):
        self.M11 = self.mass * np.eye(3)
        self.M12 = -self.mass * self._S(self.rog)
        self.M21 = -self.M12

        # rigid body inertia matrix
        self.MRB = np.zeros((6, 6))
        self.MRB[0:3, 0:3] = self.M11
        self.MRB[0:3, 3:6] = -self.M12
        self.MRB[3:6, 0:3] = self.M21
        self.MRB[3:6, 3:6] = self.Ib

        # added mass - strip theory
        Xuh = self.MRB[0, 0] - 59.0
        Yvh = self.MRB[1, 1] - 40.0
        Zwh = self.MRB[2, 2] - 168.0
        Kph = 0.1269
        Mqh = -1.2052
        Nrh = self.MRB[5, 5] - 2.8179
        Mwh = -1.0154
        self.MAdded = -np.diag([Xuh, Yvh, Zwh, Kph, Mqh, Nrh])
        self.MAdded[2, 4] = Mwh
        self.MAdded[4, 2] = Mwh
        
        self.M = self.MAdded + self.MRB
        self.M_inv = np.linalg.inv(self.M)

    def _update_centripetal_matrix(self, velocity):
        v1 = np.transpose(velocity[0:3]) # linear velocity
        v2 = np.transpose(velocity[3:6]) # angular velocity

        C12 = -self._S(np.dot(self.M11, v1) + np.dot(self.M12, v2))
        C22 = -self._S(np.dot(self.M12, v1) + np.dot(self.Ib, v2))

        CRB = np.zeros((6, 6))
        CRB[0:3, 3:6] = -C12
        CRB[3:6, 0:3] = -C12
        CRB[3:6, 3:6] = -C22

        A11 = self.MAdded[0:3, 0:3]
        A12 = self.MAdded[0:3, 3:6]
        A21 = self.MAdded[3:6, 0:3]
        A22 = self.MAdded[3:6, 3:6]

        CA12 = -self._S(np.dot(A11, v1) + np.dot(A12, v2))
        CA22 = -self._S(np.dot(A21, v1) + np.dot(A22, v2))

        CAdded = np.zeros((6, 6))
        CAdded[0:3, 3:6] = CA12
        CAdded[3:6, 0:3] = CA12
        CAdded[3:6, 3:6] = CA22
        
        self.C = CRB + CAdded

    def _update_damping_matrix(self, velocity):
        # flippers angles
        fa1 = 1.57
        fa2 = 1.57
        fa3 = 1.57
        fa4 = 1.57

        SRx = 35000.0 * 1e-6 + self.flipper_width * self.flipper_length * (
            np.cos(fa1) + np.cos(fa2) + np.cos(fa3) + np.cos(fa4))

        SRy = 164000.0 * 1e-6 + self.flipper_width * self.flipper_length * (
            np.cos(fa1) + np.cos(fa2) + np.cos(fa3) + np.cos(fa4))

        self.D[0, 0] = 56.0 * abs(velocity[0])
        self.D[1, 1] = 551.0 * abs(velocity[1])
        self.D[2, 2] = 361.0 * abs(velocity[2])
        self.D[3, 3] = (0.5 * self.rho * self.Cd4 * SRx * (0.57 * 0.288)**2 * 
                        abs(velocity[3]))
        self.D[4, 4] = (0.5 * self.rho * self.Cd5 * SRy * (0.57 * 0.314)**2 *
                        abs(velocity[4]))
        self.D[5, 5] = 0.7226 * abs(velocity[5])

    def _update_gravity_vector(self, position):
        phi = position[3] # roll
        theta = position[4] # pitch
        #psi = position[5] # yaw

        self.G[0] = (self.We - self.Bo) * np.sin(theta)
        self.G[1] = -(self.We - self.Bo) * np.cos(theta) * np.sin(phi)
        self.G[2] = -(self.We - self.Bo) * np.cos(theta) * np.cos(phi)
        self.G[3] = (-(self.RG[1] * self.We - self.RB[1] * self.Bo) * 
            np.cos(theta) * np.cos(phi) +
            (self.RG[2] * self.We - self.RB[2] * self.Bo) * np.cos(theta) * 
            np.sin(phi))
        self.G[4] = ((self.RG[2] * self.We - self.RB[2] * self.Bo) * 
            np.sin(theta) + 
            (self.RG[0] * self.We - self.RB[0] * self.Bo) * np.cos(theta) * 
            np.cos(phi))
        self.G[5] = (-(self.RG[0] * self.We - self.RB[0] * self.Bo) * 
            np.cos(theta) * np.sin(phi) -
            (self.RG[1] * self.We - self.RB[1] * self.Bo) * np.sin(phi))

    def _update_jacobian_matrix(self, position):
        phi = position[3] # roll
        theta = position[4] # pitch
        psi = position[5] # yaw

        xrot = np.array([[1.0, 0.0, 0.0], 
                         [0.0, np.cos(phi), -np.sin(phi)],
                         [0.0, np.sin(phi), np.cos(phi)]])

        yrot = np.array([[np.cos(theta), 0.0, np.sin(theta)],
                         [0.0, 1.0, 0.0],
                         [-np.sin(theta), 0.0, np.cos(theta)]])

        zrot = np.array([[np.cos(psi), -np.sin(psi), 0.0],
                         [np.sin(psi), np.cos(psi), 0.0],
                         [0.0, 0.0, 1.0]])

        ROT = np.dot(np.dot(zrot, yrot), xrot)
        ROT_inv = np.linalg.inv(ROT)

        T = np.array([
            [1.0, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
            [0.0, np.cos(phi), -np.sin(phi)],
            [0.0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]])
        T_inv = np.linalg.inv(T)

        self.J[0:3, 0:3] = ROT
        self.J[3:6, 3:6] = T

        self.J_inv[0:3, 0:3] = ROT_inv
        self.J_inv[3:6, 3:6] = T_inv

    def update(self, position, velocity):
        """
        Update M, C, D, G, J matrices based on current state.
        
        position = [x, y, z, roll, pitch, yaw] in world frame
        velocity = [vx, vy, vz, v_roll, v_pitch, v_yaw] in body fixed frame
        """
        self._update_centripetal_matrix(velocity)
        self._update_damping_matrix(velocity)
        self._update_gravity_vector(position)
        self._update_jacobian_matrix(position)


import rospy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


class TestDynamics:
    def __init__(self):
        self.dt = 0.01
        self.dynamics = UcatDynamics()
        self.p = np.zeros(6)
        self.v = np.zeros(6)
        self.U = np.zeros(6)
        self.odom_pub = rospy.Publisher("dynamics_odom", Odometry)
        self.odom_msg = Odometry()
        self.wrench_sub = rospy.Subscriber("force_req", WrenchStamped, self.onWrenchMsg)
        self.wrench_msg = None
        self.i = 0
	self.rate = rospy.Rate(100)

    def onWrenchMsg(self, wrench_msg):
        self.wrench_msg = wrench_msg
        self.U[0] = wrench_msg.wrench.force.x
        self.U[1] = 0.0 # wrench_msg.wrench.force.y
        self.U[2] = 0.0 # wrench_msg.wrench.force.z
        self.U[3] = 0.0 # wrench_msg.wrench.torque.x
        self.U[4] = 0.0 # wrench_msg.wrench.torque.y
        self.U[5] = 0.0 # wrench_msg.wrench.torque.z

    def step(self):
        p = self.p
        v = self.v
        
        self.dynamics.update(p, v)
        M_inv = self.dynamics.M_inv
        C = self.dynamics.C
        D = self.dynamics.D
        G = self.dynamics.G
        J = self.dynamics.J

        pd = v #np.dot(J, v)
        #vd = np.dot(M_inv, (np.dot(C, v) + np.dot(D, -v) + G + self.U))
        vd = np.dot(M_inv, np.dot(C, v) + np.dot(D, -v) + self.U)

        self.p += pd * self.dt
        self.v += vd * self.dt

        self.i += 1
        
        #if self.i % 10 == 0:
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.pose.pose.position.x = self.p[0]
            #self.odom_msg.pose.pose.position.y = self.p[1]
            #self.odom_msg.pose.pose.position.z = self.p[2]
            #quat = quaternion_from_euler(self.p[3], self.p[4], self.p[5])
            #self.odom_msg.pose.pose.orientation.x = quat[0]
            #self.odom_msg.pose.pose.orientation.y = quat[1]
            #self.odom_msg.pose.pose.orientation.z = quat[2]
            #self.odom_msg.pose.pose.orientation.w = quat[3]
	self.odom_msg.twist.twist.linear.x = self.v[0]
            #self.odom_msg.twist.twist.linear.y = self.v[1]
            #self.odom_msg.twist.twist.linear.z = self.v[2]
            #self.odom_msg.twist.twist.angular.x = self.v[3]
            #self.odom_msg.twist.twist.angular.y = self.v[4]
            #self.odom_msg.twist.twist.angular.z = self.v[5]
        self.odom_pub.publish(self.odom_msg)

	self.rate.sleep()

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            #rospy.sleep(self.dt)

if __name__ == '__main__':
    rospy.init_node("TestDynamics")

    td = TestDynamics()
    td.run()

