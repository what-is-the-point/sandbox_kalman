#!/usr/bin/env python3
from math import *
from datetime import datetime as date
import numpy as np


class KalmanFilter_3D_v2(object):
    #https://machinelearningspace.com/object-tracking-python/
    #https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/
    def __init__(self, dt, u_a, std_acc, pos_std_meas, vel_std_meas):
        """
        :param dt: sampling time (time for 1 cycle)
        :param u_a: control input: acceleration tuple
            u_a[0] = acceleration, x direction
            u_a[1] = acceleration, y direction
            u_a[2] = acceleration, z direction
        :param u_y: acceleration in y-direction
        :param std_acc: process noise magnitude
        :param pos_std_meas: standard deviation of the position measurement 
            pos_std_meas[0] = standard deviation of the x position measurement
            pos_std_meas[1] = standard deviation of the y position measurement
            pos_std_meas[2] = standard deviation of the z position measurement
        :param vel_std_meas: standard deviation of the measurement in y-direction
            vel_std_meas[0] = standard deviation of the x velocity measurement
            vel_std_meas[1] = standard deviation of the y velocity measurement
            vel_std_meas[2] = standard deviation of the z velocity measurement
        """
        # Define sampling time
        self.dt = dt
        # Define the  control input variables
        self.u = np.matrix([[u_a[0]], [u_a[1]], [u_a[2]]])
        # Intial State
        self.x = np.matrix([[0], [0], [0], [0], [0], [0]])
        # Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, 0, self.dt, 0, 0],
                            [0, 1, 0, 0, self.dt, 0],
                            [0, 0, 1, 0, 0, self.dt],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        # Define the Control Input Matrix B
        self.B = np.matrix([[(self.dt**2)/2, 0, 0],
                            [0, (self.dt**2)/2, 0],
                            [0, 0, (self.dt**2)/2],
                            [self.dt, 0, 0],
                            [0, self.dt, 0],
                            [0, 0, self.dt]])
        # Define Measurement Mapping Matrix
        # self.H = np.matrix([[1, 0, 0, 0, 0, 0],
        #                     [0, 1, 0, 0, 0, 0]])
        self.H = np.eye(self.A.shape[1], dtype=float)
        #Initial Process Noise Covariance
        a = (self.dt**4)/4.0
        b = (self.dt**3)/2.0
        c = self.dt**2.0
        self.Q = np.matrix([[a, 0, 0, b, 0, 0],
                            [0, a, 0, 0, b, 0],
                            [0, 0, a, 0, 0, b],
                            [b, 0, 0, c, 0, 0],
                            [0, b, 0, 0, c, 0],
                            [0, 0, b, 0, 0, c]]) * std_acc**2
        #Initial Measurement Noise Covariance
        # pos_std_meas = [x_std, y_std, z_std]
        # vel_std_meas = [xdot_std, ydot_std, zdot_std]
        xp = pos_std_meas[0] # X position measurement standard deviation
        yp = pos_std_meas[1] # Y position measurement standard deviation
        zp = pos_std_meas[2] # Z position measurement standard deviation
        xv = vel_std_meas[0] # X position measurement standard deviation
        yv = vel_std_meas[1] # Y position measurement standard deviation
        zv = vel_std_meas[2] # Z position measurement standard deviation
        self.R = np.matrix([[xp**2,0    ,0    ,xv*xp,0    ,0    ],
                            [0    ,yp**2,0    ,0    ,yv*yp,0    ],
                            [0    ,0    ,zp**2,0    ,0    ,zv*zp],
                            [xp*xv,0    ,0    ,xv**2,0    ,0    ],
                            [0    ,yp*yv,0    ,0    ,yv**2,0    ],
                            [0    ,0    ,zp*zv,0    ,0    ,zv**2]])
        #Initial Covariance Matrix
        self.P = np.matrix([[1, 0, 0, 1, 0, 0],
                            [0, 1, 0, 0, 1, 0],
                            [0, 0, 1, 0, 0, 1],
                            [1, 0, 0, 1, 0, 0],
                            [0, 1, 0, 0, 1, 0],
                            [0, 0, 1, 0, 0, 1]])
    
    def predict(self):
        # Refer to :Eq.(9) and Eq.(10) in https://machinelearningspace.com/object-tracking-python/
        # Update time state
        #x_k =Ax_(k-1) + Bu_(k-1)     Eq.(9)
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        # Calculate error covariance
        # P= A*P*A' + Q               Eq.(10)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.P, self.x
    
    def update(self, z):
        # Refer to :Eq.(11), Eq.(12) and Eq.(13) in https://machinelearningspace.com/object-tracking-python/
        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  #Eq.(11)
        # self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))   #Eq.(12)
        self.x = self.x + np.dot(K, (z - np.dot(self.H, self.x)))   #Eq.(12)
        I = np.eye(self.H.shape[1], dtype=float)
        # Update error covariance matrix
        self.P = (I - (K * self.H)) * self.P   #Eq.(13)
        # print(self.P)
        return self.P, self.x
    
    def get_state_estimate(self):
        return self.x
    
    def set_initial_state(self, state):
        """
        state[0] = x position
        state[1] = y position
        state[2] = z position
        state[3] = x velocity
        state[4] = y velocity
        state[5] = z velocity
        """
        # Set Intial State
        self.x = np.matrix([[state[0]],
                            [state[1]],
                            [state[2]],
                            [state[3]],
                            [state[4]],
                            [state[5]]])
    
    def print_parameters(self):
        print("u:", self.u)
        print("x:", self.x)
        print("A:", self.A)
        print("B:", self.B)
        print("H:", self.H)
        print("Q:", self.Q)
        print("R:", self.R)
        print("P:", self.P)


class KalmanFilter_3D(object):
    #https://machinelearningspace.com/object-tracking-python/
    #https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/
    def __init__(self, dt, u_a, std_acc, pos_std_meas, vel_std_meas):
        """
        :param dt: sampling time (time for 1 cycle)
        :param u_a: control input: acceleration tuple
            u_a[0] = acceleration, x direction
            u_a[1] = acceleration, y direction
            u_a[2] = acceleration, z direction
        :param u_y: acceleration in y-direction
        :param std_acc: process noise magnitude
        :param pos_std_meas: standard deviation of the position measurement 
            pos_std_meas[0] = standard deviation of the x position measurement
            pos_std_meas[1] = standard deviation of the y position measurement
            pos_std_meas[2] = standard deviation of the z position measurement
        :param vel_std_meas: standard deviation of the measurement in y-direction
            vel_std_meas[0] = standard deviation of the x velocity measurement
            vel_std_meas[1] = standard deviation of the y velocity measurement
            vel_std_meas[2] = standard deviation of the z velocity measurement
        """
        # Define sampling time
        self.dt = dt
        # Define the  control input variables
        self.u = np.matrix([[u_a[0]], [u_a[1]], [u_a[2]]])
        # Intial State
        self.x = np.matrix([[0], [0], [0], [0], [0], [0]])
        # Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, 0, self.dt, 0, 0],
                            [0, 1, 0, 0, self.dt, 0],
                            [0, 0, 1, 0, 0, self.dt],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        # Define the Control Input Matrix B
        self.B = np.matrix([[(self.dt**2)/2, 0, 0],
                            [0, (self.dt**2)/2, 0],
                            [0, 0, (self.dt**2)/2],
                            [self.dt, 0, 0],
                            [0, self.dt, 0],
                            [0, 0, self.dt]])
        # Define Measurement Mapping Matrix
        # self.H = np.matrix([[1, 0, 0, 0, 0, 0],
        #                     [0, 1, 0, 0, 0, 0]])
        self.H = np.eye(self.A.shape[1], dtype=float)
        #Initial Process Noise Covariance
        a = (self.dt**4)/4.0
        b = (self.dt**3)/2.0
        c = self.dt**2.0
        self.Q = np.matrix([[a, 0, 0, b, 0, 0],
                            [0, a, 0, 0, b, 0],
                            [0, 0, a, 0, 0, b],
                            [b, 0, 0, c, 0, 0],
                            [0, b, 0, 0, c, 0],
                            [0, 0, b, 0, 0, c]]) * std_acc**2
        #Initial Measurement Noise Covariance
        # pos_std_meas = [x_std, y_std, z_std]
        # vel_std_meas = [xdot_std, ydot_std, zdot_std]
        xpmv = pos_std_meas[0]**2 # X position measurement variance
        ypmv = pos_std_meas[1]**2 # Y position measurement variance
        zpmv = pos_std_meas[2]**2 # Z position measurement variance
        xvmv = vel_std_meas[0]**2 # X position measurement variance
        yvmv = vel_std_meas[1]**2 # Y position measurement variance
        zvmv = vel_std_meas[2]**2 # Z position measurement variance
        self.R = np.matrix([[xpmv,0,0,0,0,0],
                            [0,ypmv,0,0,0,0],
                            [0,0,zpmv,0,0,0],
                            [0,0,0,xvmv,0,0],
                            [0,0,0,0,yvmv,0],
                            [0,0,0,0,0,zvmv]])
        #Initial Covariance Matrix
        self.P = np.eye(self.A.shape[1], dtype=float)
    
    def predict(self):
        # Refer to :Eq.(9) and Eq.(10) in https://machinelearningspace.com/object-tracking-python/
        # Update time state
        #x_k =Ax_(k-1) + Bu_(k-1)     Eq.(9)
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        # Calculate error covariance
        # P= A*P*A' + Q               Eq.(10)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.P, self.x
    
    def update(self, z):
        # Refer to :Eq.(11), Eq.(12) and Eq.(13) in https://machinelearningspace.com/object-tracking-python/
        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  #Eq.(11)
        # self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))   #Eq.(12)
        self.x = self.x + np.dot(K, (z - np.dot(self.H, self.x)))   #Eq.(12)
        I = np.eye(self.H.shape[1], dtype=float)
        # Update error covariance matrix
        self.P = (I - (K * self.H)) * self.P   #Eq.(13)
        # print(self.P)
        return self.P, self.x
    
    def get_state_estimate(self):
        return self.x
    
    def set_initial_state(self, state):
        """
        state[0] = x position
        state[1] = y position
        state[2] = z position
        state[3] = x velocity
        state[4] = y velocity
        state[5] = z velocity
        """
        # Set Intial State
        self.x = np.matrix([[state[0]],
                            [state[1]],
                            [state[2]],
                            [state[3]],
                            [state[4]],
                            [state[5]]])
    
    def print_parameters(self):
        print("u:", self.u)
        print("x:", self.x)
        print("A:", self.A)
        print("B:", self.B)
        print("H:", self.H)
        print("Q:", self.Q)
        print("R:", self.R)
        print("P:", self.P)



class KalmanFilter_2D(object):
    #https://machinelearningspace.com/object-tracking-python/
    #https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/
    def __init__(self, dt, u_x,u_y, std_acc, x_std_meas, y_std_meas):
        """
        :param dt: sampling time (time for 1 cycle)
        :param u_x: acceleration in x-direction
        :param u_y: acceleration in y-direction
        :param std_acc: process noise magnitude
        :param x_std_meas: standard deviation of the measurement in x-direction
        :param y_std_meas: standard deviation of the measurement in y-direction
        """
        # Define sampling time
        self.dt = dt
        # Define the  control input variables
        self.u = np.matrix([[u_x],[u_y]])
        # Intial State
        self.x = np.matrix([[0], [0], [0], [0]])
        # Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        # Define the Control Input Matrix B
        self.B = np.matrix([[(self.dt**2)/2, 0],
                            [0, (self.dt**2)/2],
                            [self.dt,0],
                            [0,self.dt]])
        # Define Measurement Mapping Matrix
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])
        #Initial Process Noise Covariance
        self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                            [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, self.dt**2, 0],
                            [0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2
        #Initial Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas**2,0],
                            [0, y_std_meas**2]])
        #Initial Covariance Matrix
        self.P = np.eye(self.A.shape[1])
    
    def predict(self):
        # Refer to :Eq.(9) and Eq.(10) in https://machinelearningspace.com/object-tracking-python/
        # Update time state
        #x_k =Ax_(k-1) + Bu_(k-1)     Eq.(9)
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        # Calculate error covariance
        # P= A*P*A' + Q               Eq.(10)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x[0:2]
    
    def update(self, z):
        # Refer to :Eq.(11), Eq.(12) and Eq.(13) in https://machinelearningspace.com/object-tracking-python/
        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  #Eq.(11)
        self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))   #Eq.(12)
        I = np.eye(self.H.shape[1])
        # Update error covariance matrix
        self.P = (I - (K * self.H)) * self.P   #Eq.(13)
        return self.x[0:2]