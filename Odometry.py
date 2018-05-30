# -*- coding: utf-8 -*-

import numpy as np
import neato
from log import *
from math import pi, cos, sin, degrees

class Odometry(object):

    def __init__(self, delta_d=0, delta_th=0, theta_dot=0.0, x_ini=0, y_ini=0, speed=0, L_ini=0, R_ini=0, noise_d=0.0001, noise_th=0.000001):
        self.step = 'START'
        self.delta_d = delta_d
        self.delta_th = delta_th
        self.theta_dot = theta_dot
        self.suma_theta = delta_th
        self.speed = speed
        self.x = x_ini
        self.y = y_ini
        self.L = L_ini
        self.R = R_ini
        self.diffL = 0
        self.diffR = 0
        self.pose_est = np.array([x_ini, y_ini, delta_th])
        self.V = np.diag([noise_d, noise_th])
        self.Pk1 = np.zeros([3,3])

    def update(self, L, R, speed):
        #debug("Current (L, R) = (%i, %i)" % (L, R))
        self.diffL = L - self.L
        self.diffR = R - self.R
        #debug("Desplaçament (L,R) = (%i, %i)" % (self.diffL, self.diffR))
        self.L = L
        self.R = R
        self.speed = speed
        self.delta_d = (self.diffR + self.diffL)/2
        self.delta_th = (self.diffR - self.diffL)/(2*neato.Neato.S)
        noise_d, noise_th = self.get_noise()
        self.suma_theta = (self.suma_theta + self.delta_th + noise_th) % (2*pi)
        dx = (self.delta_d + noise_d)*cos(self.suma_theta)
        dy = (self.delta_d + noise_d)*sin(self.suma_theta)
        self.x = self.x + dx
        self.y = self.y + dy
        self.covariance_error()
    
    def covariance_error(self):
        def jacobine():
            F_x = np.matrix([[1, 0, -(self.delta_d*sin(self.suma_theta + self.delta_th))],
                [0, 1, -(self.delta_d*cos(self.suma_theta + self.delta_th))],
                [0, 0, 1]])
            F_v = np.matrix([[cos(self.suma_theta + self.delta_th), -self.delta_d*sin(self.suma_theta + self.delta_th)],
                [sin(self.suma_theta + self.delta_th), self.delta_d*cos(self.suma_theta + self.delta_th)],
                [0,1]])
            return F_x, F_v
        def taylor():
            self.pose_est = self.pose_est + F_x*(np.array([self.x, self.y, self.suma_theta]).reshape(3,1) - self.pose_est) + F_v*np.diag(self.V).reshape(2,1)
        def ricatty():
            self.Pk1 = F_x*self.Pk1*F_x.T + F_v*self.V*F_v.T
        F_x, F_v = jacobine()
        taylor()
        ricatty()

    def estimation_pose(self):
        x_est = self.pose_est[0,0]
        y_est = self.pose_est[1,1]
        delta_th_est = self.pose_est[2,2]
        return x_est, y_est, delta_th_est

    def get_noise(self):
        noise_d = self.V[0,0]
        noise_th = self.V[1,1]
        return noise_d, noise_th

    def show(self, step=''):
        self.step = step
        info(self)

    def __repr__(self):
        s = ''
        header = "###### ODOMETRY%s ######" % (' ' + self.step if self.step != '' else '')
        s = s + header
        s = s + "\n(x, y) = (%i, %i)" % (self.x, self.y)
        #s = s + "\n(L, R) = (%i, %i)" % (self.L, self.R)
        #s = s + "\nSpeed = %i" % self.speed
        s = s + "\nSuma Theta = %.2f" % degrees(self.suma_theta)
        #s = s + "\nTheta Dot = %.2f" % degrees(self.theta_dot)
        #s = s + "\nDelta D = %.2f" % self.delta_d
        #s = s + "\nDelta Theta = %.2f" % degrees(self.delta_th)
        #s = s + "\nNoise Area\n%s" % self.Pk1
        #x_est, y_est, delta_th_est = self.estimation_pose()
        #s = s + "\n(x, y) estimated = (%i, %i)" % (x_est, y_est)
        #s = s + "\nDelta Theta estimated = %.2f\n" % degrees(delta_th_est)
        s = s + '#' * len(header)
        return s