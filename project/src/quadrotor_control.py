#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os
from traj import *

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Quadrotor():
    def __init__(self):
        # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback,queue_size=1)
        rospy.Rate(100)

        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.motor_speed = [0.0,0.0,0.0,0.0]

        self.desired = {}
        self.desired['x'] = 0.0
        self.desired['y'] = 0.0
        self.desired['z'] = 0.0
        self.desired['x_dot'] = 0.0
        self.desired['y_dot'] = 0.0
        self.desired['z_dot'] = 0.0
        self.desired['x_ddot'] = 0.0
        self.desired['y_ddot'] = 0.0
        self.desired['z_ddot'] = 0.0

        self.mutex_lock_on = False
        

        rospy.on_shutdown(self.save_data)
        # TODO: include initialization codes if needed


    def traj_evaluate(self):
        # TODO: evaluating the corresponding trajectories designed in Part 1
        #       to return the desired positions, velocities and accelerations
        desired_state = qd(self.t)
        self.desired['x'] = desired_state[0]
        self.desired['y'] = desired_state[1]
        self.desired['z'] = desired_state[2]
        self.desired['x_dot'] = desired_state[3]
        self.desired['y_dot'] = desired_state[4]
        self.desired['z_dot'] = desired_state[5]
        self.desired['x_ddot'] = desired_state[6]
        self.desired['y_ddot'] = desired_state[7]
        self.desired['z_ddot'] = desired_state[8]
        

    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        # obtain the desired values by evaluating the corresponding
        # trajectories
        self.traj_evaluate()

        g = 9.81
        m = 27.0e-3
        l = 46.0e-3
        Ix = 16.571710e-6
        Iy = 16.571710e-6
        Iz = 29.261652e-6
        Ip = 12.65625e-8
        kF = 1.28192e-8
        kM = 5.964552e-3
        wmax = 2618.0
        wmin = 0.0

        #initial gains
        # kp = 100.0
        # kd = 5
        # lambda_z = 5
        # lambda_phi = 12.5
        # lambda_theta = 12.5
        # lambda_psi = 5
        # K_z = 10.0
        # K_phi = 140.0
        # K_theta = 140.0
        # K_psi = 25.0

        #final gains
        kp = 100.0
        kd = 5
        lambda_z = 10
        lambda_phi = 15
        lambda_theta = 20
        lambda_psi = 5
        K_z = 5.0
        K_phi = 140.0
        K_theta = 110.0
        K_psi = 25.0

        x = xyz[0][0]
        y = xyz[1][0]
        z = xyz[2][0]
        x_dot = xyz_dot[0][0]
        y_dot = xyz_dot[1][0]
        z_dot = xyz_dot[2][0]

        phi = rpy[0][0]
        theta = rpy[1][0]
        psi = rpy[2][0]
        phi_dot = rpy_dot[0][0]
        theta_dot = rpy_dot[1][0]
        psi_dot = rpy_dot[2][0]
        print("rpy = ", np.transpose(rpy))
        print("rpy_dot", np.transpose(rpy_dot))

        xd = self.desired['x']
        yd = self.desired['y']
        zd = self.desired['z']
        xd_dot = self.desired['x_dot']
        yd_dot = self.desired['y_dot']
        zd_dot = self.desired['z_dot']
        xd_ddot = self.desired['x_ddot']
        yd_ddot = self.desired['y_ddot']
        zd_ddot = self.desired['z_ddot']

        # TODO: implement the Sliding Mode Control laws designed in Part 2 to
        # calculate the control inputs "u"
        # REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]
        Fx = m*(-kp*(x-xd)-kd*(x_dot-xd_dot)+xd_ddot)
        Fy = m*(-kp*(y-yd)-kd*(y_dot-yd_dot)+yd_ddot)
        u = np.matrix([[0.0], [0.0], [0.0], [0.0]])
        #Calculate u1
        fz = -g
        errorz = z - zd
        errorz_dot = z_dot - zd_dot
        gz = (cos(phi)*cos(theta))/m
        sz = errorz_dot + lambda_z*errorz
        boundary_z = 1.0
        saturated_sz = sz/boundary_z
        saturated_sz = min(1.0,max(-1.0,saturated_sz))
        urz = -K_z*saturated_sz
        u[0, 0] = (-fz + zd_ddot - lambda_z*errorz_dot + urz)/gz

        phid = np.arcsin(-Fy/u[0, 0])
        phid_dot = 0.0
        phid_ddot = 0.0

        thetad = np.arcsin(Fx/u[0, 0])
        thetad_dot = 0.0
        thetad_ddot = 0.0

        psid = 0.0
        psid_dot = 0.0
        psid_ddot = 0.0
        #Calculate u2
        omega = self.motor_speed[0] - self.motor_speed[1] + self.motor_speed[2] - self.motor_speed[3]

        fphi = (psi_dot*theta_dot*(Iy - Iz))/Ix - (Ip*theta_dot*omega)/Iy
        errorphi = phi - phid
        errorphi = (errorphi + np.pi) % (2 * np.pi) - np.pi
        errorphi_dot = phi_dot - phid_dot
        gphi = 1.0/Ix
        sphi = errorphi_dot + lambda_phi*errorphi
        boundary_phi = 1.0
        saturated_sphi = min(1.0,max(-1.0,sphi/boundary_phi))
        urphi = -K_phi*saturated_sphi
        u[1,0] = (-fphi + phid_ddot - lambda_phi*errorphi_dot + urphi)/gphi

        #calculate u3
        ftheta = (Ip*phi_dot*(omega))/Iy - (phi_dot*psi_dot*(Ix - Iz))/Iy
        errortheta = theta - thetad
        errortheta = (errortheta + np.pi) % (2 * np.pi) - np.pi
        errortheta_dot = theta_dot - thetad_dot
        gtheta = 1.0/Iy
        stheta = errortheta_dot + lambda_theta * errortheta
        boundary_theta = 1.0
        saturated_stheta = min(1.0,max(-1.0,stheta/boundary_theta))
        urtheta = -K_theta*saturated_stheta
        u[2,0] = (-ftheta + thetad_ddot - lambda_theta*errortheta_dot + urtheta)/gtheta

        #calculate u4
        fpsi = (phi_dot*theta_dot*(Ix - Iy))/Iz
        errorpsi= psi- psid
        errorpsi = (errorpsi + np.pi) % (2 * np.pi) - np.pi
        errorpsi_dot = psi_dot - psid_dot
        gpsi= 1.0/Iz
        spsi= errorpsi_dot + lambda_psi* errorpsi
        boundary_psi= 1.0
        saturated_spsi = min(1.0,max(-1.0,spsi/boundary_psi))
        urpsi = -K_psi*saturated_spsi
        u[3,0] = (-fpsi + psid_ddot - lambda_psi*errorpsi_dot + urpsi)/gpsi

        allocation_matrix = np.matrix([[1/(4*kF), -sqrt(2)/(4*kF*l), -sqrt(2)/(4*kF*l), -1/(4*kM*kF)],
                                       [1/(4*kF), -sqrt(2)/(4*kF*l), sqrt(2)/(4*kF*l), 1/(4*kM*kF)],
                                        [1/(4*kF), sqrt(2)/(4*kF*l), sqrt(2)/(4*kF*l), -1/(4*kM*kF)],
                                        [1/(4*kF), sqrt(2)/(4*kF*l), -sqrt(2)/(4*kF*l), 1/(4*kM*kF)]])
        w_squared = np.matmul(allocation_matrix, u)
        motor_vel = np.sqrt(w_squared)
        motor_vel = np.clip(motor_vel,a_min=wmin, a_max=wmax)
        print("motor_vel = ", motor_vel)
        # TODO: convert the desired control inputs "u" to desired rotor
        # velocities "motor_vel" by using the "allocation matrix"
        # TODO: maintain the rotor velocities within the valid range of [0 to 2618]
        # publish the motor velocities to the associated ROS topic
        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0], motor_vel[2,0], motor_vel[3,0]]
        self.motor_speed = motor_speed.angular_velocities
        self.motor_speed_pub.publish(motor_speed)

    # odometry callback function (DO NOT MODIFY)
    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([
        [1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],
        [0, np.cos(rpy[0]), -np.sin(rpy[0])],
        [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]
        ]), w_b)
        rpy = np.expand_dims(rpy, axis=1)

        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])
            # call the controller with the current states
            self.smc_control(xyz, xyz_dot, rpy, rpy_dot)
            # save the actual trajectory data

    def save_data(self):
        # TODO: update the path below with the correct path
        # with open("/home/sfarzan/rbe502_project/src/project/scripts/log.pkl",
        # "wb") as fp:
        fp = open("/home/oliver/rbe502_project/src/project/src/log.pkl", "wb")
        self.mutex_lock_on = True
        pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)

if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")